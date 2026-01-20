#!/bin/bash
# Initial network setup script for Swarm system
# Configures eth0 with a static IP and wlan0 for DHCP
# Installs and configures dnsmasq and hostapd for AP functionality
# Handles systemd-resolved conflicts and service masking issues

# Note: We intentionally do NOT use 'set -e' here to allow the script
# to continue and provide better error reporting even if individual
# commands fail. Critical failures are handled explicitly.

# Track if any critical step failed
CRITICAL_FAILURE=false

AP_SSID="Swarm_AP"
AP_PASS="swarmascend"
AP_IP="192.168.10.1"

echo "=== [Swarm Setup] Initial network configuration ==="

###############################################
# 0.1 CHECK ROOT PRIVILEGES
###############################################
if [[ $EUID -ne 0 ]]; then
    echo "[Swarm Setup] ❌ ERROR: This script must be run as root (use sudo)"
    exit 1
fi
echo "[Swarm Setup] ✅ Running with root privileges"

###############################################
# 0.2 DETECT UBUNTU VERSION AND SET COMPATIBILITY FLAGS
###############################################
UBUNTU_VERSION=""
UBUNTU_CODENAME=""

if [[ -f /etc/os-release ]]; then
    source /etc/os-release
    UBUNTU_VERSION="$VERSION_ID"
    UBUNTU_CODENAME="$VERSION_CODENAME"
fi

echo "[Swarm Setup] Detected: Ubuntu $UBUNTU_VERSION ($UBUNTU_CODENAME)"

# Validate supported versions
case "$UBUNTU_VERSION" in
    "22.04"|"24.04")
        echo "[Swarm Setup] ✅ Supported Ubuntu version"
        ;;
    *)
        echo "[Swarm Setup] ⚠️  WARNING: Ubuntu $UBUNTU_VERSION not officially tested"
        echo "[Swarm Setup] Supported versions: 22.04, 24.04"
        read -p "Continue anyway? [y/N]: " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
        ;;
esac

###############################################
# 0.3 DETECT NETWORK INTERFACES
###############################################
echo "[Swarm Setup] Detecting network interfaces..."

# Find ethernet interface (prefer eth0, then enp*, then ens*)
ETH_INTERFACE=""
if ip link show eth0 &>/dev/null; then
    ETH_INTERFACE="eth0"
elif ip link show | grep -q "enp"; then
    ETH_INTERFACE=$(ip link show | grep -oP 'enp[a-z0-9]+' | head -1)
elif ip link show | grep -q "ens"; then
    ETH_INTERFACE=$(ip link show | grep -oP 'ens[a-z0-9]+' | head -1)
elif ip link show | grep -q "end"; then
    ETH_INTERFACE=$(ip link show | grep -oP 'end[a-z0-9]+' | head -1)
fi

# Find WiFi interface (prefer wlan0, then wlp*)
WIFI_INTERFACE=""
if ip link show wlan0 &>/dev/null; then
    WIFI_INTERFACE="wlan0"
elif ip link show | grep -q "wlp"; then
    WIFI_INTERFACE=$(ip link show | grep -oP 'wlp[a-z0-9]+' | head -1)
elif ip link show | grep -q "wlx"; then
    WIFI_INTERFACE=$(ip link show | grep -oP 'wlx[a-z0-9]+' | head -1)
fi

echo "[Swarm Setup] Ethernet interface: ${ETH_INTERFACE:-not found}"
echo "[Swarm Setup] WiFi interface: ${WIFI_INTERFACE:-not found}"

if [[ -z "$ETH_INTERFACE" ]]; then
    echo "[Swarm Setup] ⚠️  WARNING: No ethernet interface found"
    echo "[Swarm Setup] Will use 'eth0' as default (may not work)"
    ETH_INTERFACE="eth0"
fi

if [[ -z "$WIFI_INTERFACE" ]]; then
    echo "[Swarm Setup] ⚠️  WARNING: No WiFi interface found"
    echo "[Swarm Setup] WiFi/AP features will not work without a wireless adapter"
    WIFI_INTERFACE="wlan0"
fi

###############################################
# 0.4 SSH CONNECTION DETECTION AND WARNING
###############################################
RUNNING_OVER_SSH=false
SKIP_NETPLAN_APPLY=false

# Detect if running over SSH
if [[ -n "$SSH_CONNECTION" ]] || [[ -n "$SSH_CLIENT" ]] || [[ -n "$SSH_TTY" ]]; then
    RUNNING_OVER_SSH=true

    # Get current SSH connection IP
    SSH_CLIENT_IP=$(echo "$SSH_CONNECTION" | awk '{print $1}')
    SSH_SERVER_IP=$(echo "$SSH_CONNECTION" | awk '{print $3}')

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ⚠️  WARNING: Running over SSH connection"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Current SSH connection:"
    echo "  • Client IP: $SSH_CLIENT_IP"
    echo "  • Server IP: $SSH_SERVER_IP"
    echo ""
    echo "This script will configure eth0 with static IP: 192.168.10.1"
    echo ""
    echo "Options:"
    echo "  1) Apply network changes NOW (may disconnect SSH)"
    echo "  2) Configure files only, apply on REBOOT (safer)"
    echo "  3) Abort"
    echo ""
    read -p "Choose option [1/2/3]: " -n 1 -r SSH_CHOICE
    echo ""

    case $SSH_CHOICE in
        1)
            echo "[Swarm Setup] Will apply network changes immediately"
            echo "[Swarm Setup] ⚠️  You may need to reconnect via 192.168.10.1"
            SKIP_NETPLAN_APPLY=false
            ;;
        2)
            echo "[Swarm Setup] Will configure files but NOT apply netplan"
            echo "[Swarm Setup] Changes will take effect after reboot"
            SKIP_NETPLAN_APPLY=true
            ;;
        3|*)
            echo "[Swarm Setup] Aborted by user"
            exit 0
            ;;
    esac
    echo ""
fi

###############################################
# 0. NETPLAN FIX - BACKUP EXISTING CONFIGURATIONS AND DISABLE CLOUD-INIT NETWORK MANAGEMENT
###############################################
echo "[Netplan Fix] Backing up existing configurations..."
sudo mkdir -p /etc/netplan/backup
sudo cp -f /etc/netplan/*.yaml /etc/netplan/backup/ 2>/dev/null || true

echo "[Netplan Fix] Disabling cloud-init network configuration..."

# Create the cloud-init disable file
sudo bash -c "cat > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg" <<EOF
# Disable cloud-init network configuration
# This prevents cloud-init from overwriting our custom network setup
network: {config: disabled}
EOF

echo "[Netplan Fix] Removing conflicting netplan files..."

# Remove cloud-init generated file
sudo rm -f /etc/netplan/50-cloud-init.yaml

# Remove any other cloud-init network configs
sudo rm -f /etc/netplan/*cloud*.yaml

###############################################
# 1. INSTALL DNSMASQ Y HOSTAPD
###############################################
echo "[Swarm Setup] Installing required packages..."
apt update

# Core packages (available on both 22.04 and 24.04)
apt install -y dnsmasq hostapd wireless-tools iw lsof wpasupplicant

# DHCP client - try dhclient first (22.04), fallback to dhcpcd5 (some 24.04 installs)
if ! command -v dhclient &>/dev/null; then
    echo "[Swarm Setup] dhclient not found, installing isc-dhcp-client..."
    apt install -y isc-dhcp-client || apt install -y dhcpcd5 || true
fi

echo "[Swarm Setup] ✅ Packages installed"

# Stop services first
sudo systemctl stop dnsmasq || true
sudo systemctl stop hostapd || true

###############################################
# 2. HANDLE SYSTEMD-RESOLVED PORT 53 CONFLICT
###############################################
echo "[Swarm Setup] Handling systemd-resolved port 53 conflict..."

# Function to check port 53
check_port_53() {
    if sudo lsof -i:53 2>/dev/null | grep -q "LISTEN"; then
        return 0
    else
        return 1
    fi
}

# Check if systemd-resolved is using port 53
if check_port_53; then
    echo "[Swarm Setup] Port 53 is in use, configuring systemd-resolved..."
    
    # Configure systemd-resolved to not use port 53
    sudo mkdir -p /etc/systemd/resolved.conf.d/
    sudo bash -c "cat > /etc/systemd/resolved.conf.d/99-disable-stub.conf" <<EOF
[Resolve]
DNSStubListener=no
DNS=8.8.8.8 8.8.4.4
FallbackDNS=1.1.1.1 1.0.0.1
EOF
    
    # Restart systemd-resolved to apply changes
    sudo systemctl restart systemd-resolved
    
    # Update resolv.conf symlink
    sudo rm -f /etc/resolv.conf
    sudo ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf
    
    echo "[Swarm Setup] Waiting for port 53 to be released..."
    sleep 3
    
    # Verify port 53 is free
    if check_port_53; then
        echo "[Swarm Setup] WARNING: Port 53 still in use, stopping systemd-resolved..."
        sudo systemctl stop systemd-resolved
        sudo systemctl disable systemd-resolved
        
        # Create a static resolv.conf
        sudo bash -c "cat > /etc/resolv.conf" <<EOF
nameserver 8.8.8.8
nameserver 8.8.4.4
nameserver 1.1.1.1
EOF
        # Fix hostname resolution
        echo "127.0.1.1 ubuntu" | sudo tee -a /etc/hosts > /dev/null
    fi
fi

###############################################
# 3. FIX HOSTNAME RESOLUTION
###############################################
echo "[Swarm Setup] Fixing hostname resolution..."

# Ensure hostname is in /etc/hosts
HOSTNAME=$(hostname)
if ! grep -q "127.0.1.1.*$HOSTNAME" /etc/hosts; then
    echo "127.0.1.1 $HOSTNAME" | sudo tee -a /etc/hosts > /dev/null
fi

###############################################
# 4. CHECK AND UNMASK HOSTAPD IF NEEDED
###############################################
echo "[Swarm Setup] Checking hostapd service status..."

# Check if hostapd is masked
if systemctl list-unit-files | grep -q "hostapd.service.*masked"; then
    echo "[Swarm Setup] hostapd is masked, unmasking..."
    sudo systemctl unmask hostapd
fi

# Disable auto-start for both services (will be managed by wifi-manager)
sudo systemctl disable dnsmasq || true
sudo systemctl disable hostapd || true

###############################################
# 5. CONFIGURE NETPLAN (FIXED VERSION)
###############################################
echo "[Swarm Setup] Configuring Netplan for interface: $ETH_INTERFACE"

NETPLAN_FILE="/etc/netplan/99-swarm-network.yaml"

# Create proper netplan configuration using detected interface
cat > $NETPLAN_FILE <<EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    $ETH_INTERFACE:
      dhcp4: no
      addresses:
        - 192.168.10.1/24
      optional: true
EOF

# Set correct permissions for netplan file (only root can read/write)
sudo chmod 600 $NETPLAN_FILE
sudo chown root:root $NETPLAN_FILE

echo "[Swarm Setup] Generating netplan configuration..."
netplan generate

if [[ "$SKIP_NETPLAN_APPLY" = true ]]; then
    echo "[Swarm Setup] ⏸️  Skipping netplan apply (will take effect on reboot)"
    echo "[Swarm Setup] To apply manually later: sudo netplan apply"
else
    echo "[Swarm Setup] Applying netplan configuration..."
    netplan apply || true  # Continue even if netplan apply has warnings
fi

# Note: wlan0 will be managed by hostapd when in AP mode, 
# or by wpa_supplicant/NetworkManager when in client mode

###############################################
# 6. DNSMASQ CONFIGURATION (permanent)
###############################################
echo "[Swarm Setup] Dnsmasq setup for interface: $WIFI_INTERFACE"

# Backup original config if exists
if [ -f /etc/dnsmasq.conf ]; then
    cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup
fi

cat > /etc/dnsmasq.conf <<EOF
# Swarm AP Configuration
# Generated for Ubuntu $UBUNTU_VERSION
interface=$WIFI_INTERFACE
bind-interfaces
dhcp-range=192.168.10.10,192.168.10.50,255.255.255.0,24h
domain-needed
bogus-priv
no-resolv
server=8.8.8.8
server=8.8.4.4
EOF

###############################################
# 7. HOSTAPD CONFIGURATION (permanent)
###############################################
echo "[Swarm Setup] Hostapd setup for interface: $WIFI_INTERFACE"

cat > /etc/hostapd/hostapd.conf <<EOF
# Swarm AP Configuration
# Generated for Ubuntu $UBUNTU_VERSION
interface=$WIFI_INTERFACE
driver=nl80211
ssid=$AP_SSID
hw_mode=g
channel=6
wmm_enabled=1
auth_algs=1
wpa=2
wpa_passphrase=$AP_PASS
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
ctrl_interface=/var/run/hostapd
ctrl_interface_group=0

# Required for regulatory compliance (compatible with 22.04 and 24.04)
country_code=ES
ieee80211n=1
ieee80211d=1
EOF

echo "DAEMON_CONF=\"/etc/hostapd/hostapd.conf\"" > /etc/default/hostapd

###############################################
# 8. CREATE KNOWN NETWORKS FILE AND ADD NETWORKS
###############################################
echo "[Swarm Setup] Configuring known WiFi networks..."

# Create the config file if it doesn't exist
if [ ! -f /etc/wifi_networks.conf ]; then
    cat > /etc/wifi_networks.conf <<EOF
# WiFi Networks Configuration
# Format: SSID:password
# Example: MyNetwork:mypassword123
EOF
fi

# Ask user if they want to add a WiFi network
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  📶 WiFi Network Configuration"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Current known networks:"
grep -v "^#" /etc/wifi_networks.conf | grep -v "^$" || echo "  (none configured)"
echo ""
echo "Would you like to add a WiFi network to connect to?"
echo "This allows the system to connect to your network instead of creating an AP."
echo ""
read -p "Add a WiFi network? [y/N]: " -n 1 -r ADD_WIFI
echo ""

if [[ $ADD_WIFI =~ ^[Yy]$ ]]; then
    # Scan for available networks if possible
    if command -v nmcli &> /dev/null; then
        echo ""
        echo "Scanning for available networks..."
        nmcli -t -f SSID dev wifi list 2>/dev/null | sort | uniq | head -10 || true
        echo ""
    elif command -v iw &> /dev/null && ip link show wlan0 &>/dev/null; then
        echo ""
        echo "Scanning for available networks..."
        ip link set wlan0 up 2>/dev/null || true
        iw wlan0 scan 2>/dev/null | grep "SSID:" | sed 's/.*SSID: //' | sort | uniq | head -10 || true
        echo ""
    fi

    while true; do
        read -p "Enter WiFi SSID (or 'done' to finish): " WIFI_SSID

        if [[ "$WIFI_SSID" == "done" ]] || [[ -z "$WIFI_SSID" ]]; then
            break
        fi

        read -s -p "Enter WiFi password for '$WIFI_SSID': " WIFI_PASS
        echo ""

        if [[ -n "$WIFI_PASS" ]]; then
            # Check if network already exists
            if grep -q "^${WIFI_SSID}:" /etc/wifi_networks.conf 2>/dev/null; then
                echo "  ⚠️  Network '$WIFI_SSID' already configured, updating password..."
                sed -i "/^${WIFI_SSID}:/d" /etc/wifi_networks.conf
            fi

            echo "${WIFI_SSID}:${WIFI_PASS}" >> /etc/wifi_networks.conf
            echo "  ✅ Added network: $WIFI_SSID"
        else
            echo "  ⚠️  Skipped (empty password)"
        fi

        read -p "Add another network? [y/N]: " -n 1 -r ADD_ANOTHER
        echo ""
        if [[ ! $ADD_ANOTHER =~ ^[Yy]$ ]]; then
            break
        fi
    done
fi

# Show final configuration
echo ""
echo "Configured WiFi networks:"
NETWORK_COUNT=$(grep -v "^#" /etc/wifi_networks.conf | grep -v "^$" | wc -l)
if [[ $NETWORK_COUNT -gt 0 ]]; then
    grep -v "^#" /etc/wifi_networks.conf | grep -v "^$" | cut -d: -f1 | while read ssid; do
        echo "  • $ssid"
    done
else
    echo "  (none - system will start in AP mode)"
fi
echo ""

###############################################
# 9. CREATE IMPROVED WIFI MANAGER
###############################################
echo "[Swarm Setup] Creating wifi_manager.sh..."

cat > /usr/local/bin/wifi_manager.sh <<'EOF'
#!/bin/bash

# Auto-detect WiFi interface at runtime
detect_wifi_interface() {
    if ip link show wlan0 &>/dev/null; then
        echo "wlan0"
    elif ip link show | grep -q "wlp"; then
        ip link show | grep -oP 'wlp[a-z0-9]+' | head -1
    elif ip link show | grep -q "wlx"; then
        ip link show | grep -oP 'wlx[a-z0-9]+' | head -1
    else
        echo "wlan0"  # fallback
    fi
}

WIFI_INTERFACE=$(detect_wifi_interface)
CONFIG_FILE="/etc/wifi_networks.conf"
AP_SSID="Swarm_AP"
AP_PASS="swarmascend"
AP_IP="192.168.10.1"
MAX_RETRIES=3

echo "[WiFi Manager] Starting network check..."
echo "[WiFi Manager] Using WiFi interface: $WIFI_INTERFACE"

# Function to check if port 53 is free
check_port_53() {
    if sudo lsof -i:53 2>/dev/null | grep -q "LISTEN"; then
        echo "[WiFi Manager] Port 53 is in use, attempting to free it..."
        
        # Try to kill systemd-resolved if it's using the port
        if sudo lsof -i:53 2>/dev/null | grep -q systemd-r; then
            echo "[WiFi Manager] Stopping systemd-resolved..."
            sudo systemctl stop systemd-resolved || true
            sleep 2
        fi
        
        # Kill any remaining process on port 53
        local pids=$(sudo lsof -ti:53 2>/dev/null)
        if [ ! -z "$pids" ]; then
            echo "[WiFi Manager] Killing processes on port 53: $pids"
            sudo kill -9 $pids 2>/dev/null || true
            sleep 2
        fi
    fi
}

# Function to start AP services with retry logic
start_ap_services() {
    local retries=0
    
    # Start dnsmasq with retry logic
    while [ $retries -lt $MAX_RETRIES ]; do
        check_port_53
        
        if sudo systemctl start dnsmasq 2>/dev/null; then
            echo "[WiFi Manager] ✅ dnsmasq started successfully"
            break
        else
            echo "[WiFi Manager] ⚠️ Failed to start dnsmasq, retry $((retries+1))/$MAX_RETRIES"
            retries=$((retries+1))
            sleep 2
        fi
    done
    
    if [ $retries -eq $MAX_RETRIES ]; then
        echo "[WiFi Manager] ❌ Failed to start dnsmasq after $MAX_RETRIES attempts"
        return 1
    fi
    
    # Start hostapd
    if ! sudo systemctl start hostapd 2>/dev/null; then
        echo "[WiFi Manager] ⚠️ Failed to start hostapd, checking if masked..."
        
        # Check and unmask if needed
        if systemctl list-unit-files 2>/dev/null | grep -q "hostapd.service.*masked"; then
            echo "[WiFi Manager] Unmasking hostapd..."
            sudo systemctl unmask hostapd
        fi
        
        # Try again
        if sudo systemctl start hostapd; then
            echo "[WiFi Manager] ✅ hostapd started successfully"
        else
            echo "[WiFi Manager] ❌ Failed to start hostapd"
            return 1
        fi
    else
        echo "[WiFi Manager] ✅ hostapd started successfully"
    fi
    
    return 0
}

# Function to connect with wpa_supplicant
connect_with_wpa() {
    local ssid="$1"
    local password="$2"
    
    echo "[WiFi Manager] Configuring wpa_supplicant for $ssid..."
    
    # Create wpa_supplicant configuration
    wpa_passphrase "$ssid" "$password" | sudo tee /etc/wpa_supplicant/wpa_supplicant-wlan0.conf > /dev/null
    
    # Add control interface
    sudo sed -i '1i ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\nupdate_config=1\ncountry=ES\n' /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
    
    # Start wpa_supplicant
    sudo wpa_supplicant -B -i "$WIFI_INTERFACE" -c /etc/wpa_supplicant/wpa_supplicant-wlan0.conf 2>/dev/null
    
    # Wait for connection
    sleep 5

    # Request DHCP (try multiple methods for compatibility)
    if command -v dhclient &>/dev/null; then
        sudo dhclient "$WIFI_INTERFACE" 2>/dev/null
    elif command -v dhcpcd &>/dev/null; then
        sudo dhcpcd "$WIFI_INTERFACE" 2>/dev/null
    else
        # Fallback: use networkctl if available (systemd-networkd)
        sudo networkctl reconfigure "$WIFI_INTERFACE" 2>/dev/null || true
    fi

    # Wait a bit more for IP assignment
    sleep 3

    # Check if connected
    if ip addr show "$WIFI_INTERFACE" | grep -q "inet "; then
        return 0
    else
        return 1
    fi
}

# Deactivate AP services first
sudo systemctl stop hostapd 2>/dev/null || true
sudo systemctl stop dnsmasq 2>/dev/null || true

# Kill any existing wpa_supplicant
sudo killall wpa_supplicant 2>/dev/null || true

CONNECTED=false

# Check if we have NetworkManager
if command -v nmcli &> /dev/null; then
    echo "[WiFi Manager] Using NetworkManager..."
    
    # Scan for available networks
    AVAILABLE=$(nmcli -t -f SSID dev wifi list 2>/dev/null | sort | uniq)
    
    if [ ! -z "$AVAILABLE" ]; then
        echo "[WiFi Manager] Detected networks:"
        echo "$AVAILABLE"
        
        # Try to connect to known networks
        if [ -f "$CONFIG_FILE" ]; then
            while IFS=: read -r SSID PASSWORD; do
                # Skip empty lines and comments
                [[ -z "$SSID" || "$SSID" =~ ^# ]] && continue
                
                if echo "$AVAILABLE" | grep -qx "$SSID"; then
                    echo "[WiFi Manager] Trying to connect to $SSID..."
                    if nmcli dev wifi connect "$SSID" password "$PASSWORD" ifname "$WIFI_INTERFACE" 2>/dev/null; then
                        CONNECTED=true
                        echo "[WiFi Manager] ✅ Connected to $SSID"
                        break
                    fi
                fi
            done < "$CONFIG_FILE"
        fi
    fi
else
    echo "[WiFi Manager] NetworkManager not found, using wpa_supplicant..."
    
    # Try wpa_supplicant for known networks
    if [ -f "$CONFIG_FILE" ]; then
        # Bring up the interface
        sudo ip link set "$WIFI_INTERFACE" up
        
        # Scan for networks
        SCAN_RESULT=$(sudo iw "$WIFI_INTERFACE" scan 2>/dev/null | grep "SSID:" | sed 's/.*SSID: //')
        
        while IFS=: read -r SSID PASSWORD; do
            # Skip empty lines and comments
            [[ -z "$SSID" || "$SSID" =~ ^# ]] && continue
            
            if echo "$SCAN_RESULT" | grep -q "$SSID"; then
                echo "[WiFi Manager] Found known network: $SSID"
                if connect_with_wpa "$SSID" "$PASSWORD"; then
                    CONNECTED=true
                    echo "[WiFi Manager] ✅ Connected to $SSID"
                    break
                fi
            fi
        done < "$CONFIG_FILE"
    fi
fi

# If not connected, start AP mode
if [ "$CONNECTED" = false ]; then
    echo "[WiFi Manager] No known network available, starting Access Point..."

    # Tell NetworkManager to release the interface (if NM is running)
    if command -v nmcli &> /dev/null; then
        echo "[WiFi Manager] Releasing wlan0 from NetworkManager..."
        sudo nmcli device set "$WIFI_INTERFACE" managed no 2>/dev/null || true
    fi

    # Configure interface for AP mode
    sudo ip link set "$WIFI_INTERFACE" down
    sudo ip addr flush dev "$WIFI_INTERFACE"
    sudo ip addr add "$AP_IP/24" dev "$WIFI_INTERFACE"
    sudo ip link set "$WIFI_INTERFACE" up
    
    # Start AP services
    if start_ap_services; then
        echo "[WiFi Manager] ✅ AP active: SSID=$AP_SSID, PASS=$AP_PASS"
    else
        echo "[WiFi Manager] ❌ Failed to start AP mode"
        exit 1
    fi
fi
EOF

sudo chmod +x /usr/local/bin/wifi_manager.sh

###############################################
# 10. CREATE WIFI CONNECTION HELPER
###############################################
echo "[Swarm Setup] Creating wifi connection helper..."

sudo bash -c "cat > /usr/local/bin/wifi_connect.sh" <<'EOF'
#!/bin/bash
# Helper script to add and connect to WiFi networks

CONFIG_FILE="/etc/wifi_networks.conf"

if [ $# -ne 2 ]; then
    echo "Usage: $0 <SSID> <PASSWORD>"
    echo "Example: $0 'MyWiFi' 'mypassword123'"
    exit 1
fi

SSID="$1"
PASSWORD="$2"

# Add to config file if not already there
if ! grep -q "^$SSID:" "$CONFIG_FILE" 2>/dev/null; then
    echo "$SSID:$PASSWORD" | sudo tee -a "$CONFIG_FILE" > /dev/null
    echo "✅ Network '$SSID' added to configuration"
else
    echo "ℹ️ Network '$SSID' already in configuration"
fi

# Restart wifi manager to connect
echo "Restarting WiFi manager to connect..."
sudo systemctl restart wifi-manager.service

echo "Check connection status with: sudo systemctl status wifi-manager"
EOF

sudo chmod +x /usr/local/bin/wifi_connect.sh

###############################################
# 11. SYSTEMD SERVICE FOR WIFI MANAGER
###############################################
echo "[Swarm Setup] Creating systemd service for wifi_manager.sh..."

cat > /etc/systemd/system/wifi-manager.service <<EOF
[Unit]
Description=WiFi Auto Manager (client or AP)
After=network.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/wifi_manager.sh
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

if [[ ! -f /etc/systemd/system/wifi-manager.service ]]; then
    echo "[Swarm Setup] ❌ ERROR: Failed to create wifi-manager.service"
    CRITICAL_FAILURE=true
else
    echo "[Swarm Setup] ✅ wifi-manager.service created"
fi

systemctl daemon-reload
if ! systemctl enable wifi-manager.service; then
    echo "[Swarm Setup] ⚠️ WARNING: Failed to enable wifi-manager.service"
fi

###############################################
# 12. TEST SERVICES
###############################################
echo "[Swarm Setup] Testing service configurations..."

# Function for testing in script
test_check_port_53() {
    if sudo lsof -i:53 2>/dev/null | grep -q "LISTEN"; then
        return 0
    else
        return 1
    fi
}

# Test if hostapd can start
echo -n "Testing hostapd... "
sudo systemctl stop hostapd 2>/dev/null || true
if sudo systemctl start hostapd 2>/dev/null; then
    echo "✅ OK"
    sudo systemctl stop hostapd
else
    echo "⚠️ Failed, attempting to fix..."
    sudo systemctl unmask hostapd 2>/dev/null || true
    if sudo systemctl start hostapd 2>/dev/null; then
        echo "✅ Fixed and OK"
        sudo systemctl stop hostapd
    else
        echo "❌ Still failing, check logs with: sudo journalctl -u hostapd"
    fi
fi

# Test if dnsmasq can start
echo -n "Testing dnsmasq... "
sudo systemctl stop dnsmasq 2>/dev/null || true

# Free port 53 if needed
if test_check_port_53; then
    echo "(freeing port 53 first...)"
    sudo systemctl stop systemd-resolved 2>/dev/null || true
    sudo killall -9 dnsmasq 2>/dev/null || true
    sleep 2
fi

if sudo systemctl start dnsmasq 2>/dev/null; then
    echo "✅ OK"
    sudo systemctl stop dnsmasq
else
    echo "❌ Failed, check logs with: sudo journalctl -u dnsmasq"
fi

# Start wifi-manager service
echo "Starting WiFi manager service..."
sudo systemctl restart wifi-manager.service

###############################################
# 13. FINAL VERIFICATION AND MESSAGE
###############################################

echo ""
echo "[Swarm Setup] Running final verification..."

# Verify critical files were created
VERIFICATION_ERRORS=0

if [[ ! -f /etc/systemd/system/wifi-manager.service ]]; then
    echo "  ❌ MISSING: /etc/systemd/system/wifi-manager.service"
    VERIFICATION_ERRORS=$((VERIFICATION_ERRORS + 1))
else
    echo "  ✅ OK: wifi-manager.service"
fi

if [[ ! -f /etc/netplan/99-swarm-network.yaml ]]; then
    echo "  ❌ MISSING: /etc/netplan/99-swarm-network.yaml"
    VERIFICATION_ERRORS=$((VERIFICATION_ERRORS + 1))
else
    echo "  ✅ OK: netplan configuration"
fi

if [[ ! -f /etc/hostapd/hostapd.conf ]]; then
    echo "  ❌ MISSING: /etc/hostapd/hostapd.conf"
    VERIFICATION_ERRORS=$((VERIFICATION_ERRORS + 1))
else
    echo "  ✅ OK: hostapd configuration"
fi

if [[ ! -f /etc/dnsmasq.conf ]]; then
    echo "  ❌ MISSING: /etc/dnsmasq.conf"
    VERIFICATION_ERRORS=$((VERIFICATION_ERRORS + 1))
else
    echo "  ✅ OK: dnsmasq configuration"
fi

if [[ ! -f /usr/local/bin/wifi_manager.sh ]]; then
    echo "  ❌ MISSING: /usr/local/bin/wifi_manager.sh"
    VERIFICATION_ERRORS=$((VERIFICATION_ERRORS + 1))
else
    echo "  ✅ OK: wifi_manager.sh"
fi

echo ""

if [[ $VERIFICATION_ERRORS -gt 0 ]] || [[ "$CRITICAL_FAILURE" = true ]]; then
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ❌ Network Setup FAILED with $VERIFICATION_ERRORS errors"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Please check the output above for errors and try again."
    echo "Common issues:"
    echo " • Script not run as root (use sudo)"
    echo " • Package installation failed"
    echo " • Filesystem permission issues"
    exit 1
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✅ Network Setup Completed Successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [[ "$SKIP_NETPLAN_APPLY" = true ]]; then
    echo "⚠️  IMPORTANT: Network changes NOT YET ACTIVE"
    echo ""
    echo "You chose to defer network changes. To activate:"
    echo "  Option A: Reboot the system"
    echo "            sudo reboot"
    echo ""
    echo "  Option B: Apply manually (will disconnect SSH!)"
    echo "            sudo netplan apply"
    echo ""
    echo "After reboot/apply, connect via: ssh pi@192.168.10.1"
    echo ""
fi

echo "Configuration (Ubuntu $UBUNTU_VERSION):"
echo " • $ETH_INTERFACE: Static IP 192.168.10.1/24"
echo " • $WIFI_INTERFACE: Managed by WiFi Manager"
echo " • AP Mode: SSID='$AP_SSID', Pass='$AP_PASS'"
echo ""
echo "Quick Commands:"
echo " • Add WiFi: sudo wifi_connect.sh 'SSID' 'password'"
echo " • Check status: sudo systemctl status wifi-manager"
echo " • View logs: sudo journalctl -u wifi-manager -f"
echo " • Restart: sudo systemctl restart wifi-manager"
echo ""
echo "The system will automatically:"
echo " 1. Try to connect to known WiFi networks"
echo " 2. If none available, create an Access Point"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"