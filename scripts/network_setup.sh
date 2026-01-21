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
# 4.5 DISABLE NETWORKMANAGER FOR WIFI (Ubuntu 24.04 fix)
###############################################
echo "[Swarm Setup] Configuring NetworkManager to not manage WiFi..."

# Create NetworkManager config to ignore our WiFi interface
if command -v nmcli &>/dev/null; then
    mkdir -p /etc/NetworkManager/conf.d

    cat > /etc/NetworkManager/conf.d/99-swarm-unmanaged.conf <<EOF
# Swarm: Prevent NetworkManager from managing WiFi interface
# WiFi is managed by wifi-manager.service instead
[keyfile]
unmanaged-devices=interface-name:$WIFI_INTERFACE;interface-name:wlan*;interface-name:wlp*
EOF

    # Also disable WiFi management in main config if it exists
    if [ -f /etc/NetworkManager/NetworkManager.conf ]; then
        if ! grep -q "wifi.scan-rand-mac-address=no" /etc/NetworkManager/NetworkManager.conf; then
            cat >> /etc/NetworkManager/NetworkManager.conf <<EOF

[device]
wifi.scan-rand-mac-address=no
EOF
        fi
    fi

    # Reload NetworkManager configuration
    systemctl reload NetworkManager 2>/dev/null || systemctl restart NetworkManager 2>/dev/null || true
    echo "[Swarm Setup] ✅ NetworkManager configured to ignore $WIFI_INTERFACE"
else
    echo "[Swarm Setup] NetworkManager not installed, skipping configuration"
fi

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

# ============================================================
# Swarm WiFi Manager
# Manages WiFi connectivity: connects to known networks or
# creates an Access Point if no known networks are available
# Compatible with Ubuntu 22.04 and 24.04
# ============================================================

LOG_TAG="WiFi Manager"

log_info() {
    echo "[$LOG_TAG] $1"
    logger -t "wifi-manager" "$1" 2>/dev/null || true
}

log_success() {
    echo "[$LOG_TAG] ✅ $1"
    logger -t "wifi-manager" "SUCCESS: $1" 2>/dev/null || true
}

log_warning() {
    echo "[$LOG_TAG] ⚠️ $1"
    logger -t "wifi-manager" "WARNING: $1" 2>/dev/null || true
}

log_error() {
    echo "[$LOG_TAG] ❌ $1"
    logger -t "wifi-manager" "ERROR: $1" 2>/dev/null || true
}

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
CONNECTION_TIMEOUT=30

log_info "============================================"
log_info "Starting WiFi Manager"
log_info "WiFi interface: $WIFI_INTERFACE"
log_info "Config file: $CONFIG_FILE"
log_info "============================================"

# ============================================================
# STEP 0: Stop conflicting services and clean up
# ============================================================
cleanup_services() {
    log_info "Cleaning up conflicting services..."

    # Stop AP services
    systemctl stop hostapd 2>/dev/null || true
    systemctl stop dnsmasq 2>/dev/null || true

    # Kill any existing wpa_supplicant for our interface
    killall wpa_supplicant 2>/dev/null || true

    # Release DHCP leases
    dhclient -r "$WIFI_INTERFACE" 2>/dev/null || true

    # Small delay to let services stop
    sleep 1
}

# ============================================================
# STEP 1: Disable NetworkManager control over WiFi interface
# This is CRITICAL on Ubuntu 24.04 to prevent conflicts
# ============================================================
disable_networkmanager_wifi() {
    if command -v nmcli &>/dev/null; then
        log_info "Configuring NetworkManager to not manage $WIFI_INTERFACE..."

        # Set device as unmanaged
        nmcli device set "$WIFI_INTERFACE" managed no 2>/dev/null || true

        # Also disable WiFi globally in NM if we're going to manage it ourselves
        # nmcli radio wifi off 2>/dev/null || true

        sleep 1
        log_success "NetworkManager released $WIFI_INTERFACE"
    fi
}

# ============================================================
# STEP 2: Check if port 53 is free (needed for dnsmasq)
# ============================================================
free_port_53() {
    if lsof -i:53 2>/dev/null | grep -q "LISTEN"; then
        log_warning "Port 53 is in use, attempting to free it..."

        # Try to configure systemd-resolved to not use port 53
        if lsof -i:53 2>/dev/null | grep -q systemd-r; then
            log_info "Stopping systemd-resolved..."
            systemctl stop systemd-resolved || true
            sleep 2
        fi

        # Kill any remaining process on port 53
        local pids=$(lsof -ti:53 2>/dev/null)
        if [ ! -z "$pids" ]; then
            log_info "Killing processes on port 53: $pids"
            kill -9 $pids 2>/dev/null || true
            sleep 1
        fi
    fi
}

# ============================================================
# STEP 3: Start AP services
# ============================================================
start_ap_services() {
    local retries=0

    # Ensure port 53 is free
    free_port_53

    # Start dnsmasq with retry logic
    while [ $retries -lt $MAX_RETRIES ]; do
        if systemctl start dnsmasq 2>/dev/null; then
            log_success "dnsmasq started"
            break
        else
            log_warning "Failed to start dnsmasq, retry $((retries+1))/$MAX_RETRIES"
            free_port_53
            retries=$((retries+1))
            sleep 2
        fi
    done

    if [ $retries -eq $MAX_RETRIES ]; then
        log_error "Failed to start dnsmasq after $MAX_RETRIES attempts"
        return 1
    fi

    # Start hostapd
    if ! systemctl start hostapd 2>/dev/null; then
        log_warning "Failed to start hostapd, checking if masked..."

        # Check and unmask if needed
        if systemctl list-unit-files 2>/dev/null | grep -q "hostapd.service.*masked"; then
            log_info "Unmasking hostapd..."
            systemctl unmask hostapd
        fi

        # Try again
        if systemctl start hostapd; then
            log_success "hostapd started"
        else
            log_error "Failed to start hostapd"
            journalctl -u hostapd --no-pager -n 10 2>/dev/null || true
            return 1
        fi
    else
        log_success "hostapd started"
    fi

    return 0
}

# ============================================================
# STEP 4: Connect with wpa_supplicant (non-NetworkManager method)
# ============================================================
connect_with_wpa() {
    local ssid="$1"
    local password="$2"
    local config_file="/etc/wpa_supplicant/wpa_supplicant-${WIFI_INTERFACE}.conf"

    log_info "Configuring wpa_supplicant for '$ssid'..."

    # Create wpa_supplicant configuration directory
    mkdir -p /etc/wpa_supplicant

    # Generate wpa_supplicant configuration
    cat > "$config_file" <<WPAEOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=ES

WPAEOF

    # Add network configuration
    wpa_passphrase "$ssid" "$password" >> "$config_file"

    # Remove the plaintext password comment for security
    sed -i '/#psk=/d' "$config_file"

    # Ensure interface is up
    ip link set "$WIFI_INTERFACE" up
    sleep 1

    # Start wpa_supplicant
    log_info "Starting wpa_supplicant..."
    wpa_supplicant -B -i "$WIFI_INTERFACE" -c "$config_file" 2>/dev/null

    # Wait for association
    log_info "Waiting for WiFi association..."
    local wait_time=0
    while [ $wait_time -lt $CONNECTION_TIMEOUT ]; do
        if iw "$WIFI_INTERFACE" link 2>/dev/null | grep -q "Connected"; then
            log_success "Associated with $ssid"
            break
        fi
        sleep 1
        wait_time=$((wait_time + 1))
    done

    if [ $wait_time -ge $CONNECTION_TIMEOUT ]; then
        log_warning "Association timeout for $ssid"
        return 1
    fi

    # Request DHCP (try multiple methods for compatibility)
    log_info "Requesting IP address via DHCP..."
    if command -v dhclient &>/dev/null; then
        dhclient -v "$WIFI_INTERFACE" 2>&1 | head -5
    elif command -v dhcpcd &>/dev/null; then
        dhcpcd "$WIFI_INTERFACE" 2>/dev/null
    else
        # Fallback: use networkctl if available (systemd-networkd)
        networkctl reconfigure "$WIFI_INTERFACE" 2>/dev/null || true
    fi

    # Wait for IP assignment
    local ip_wait=0
    while [ $ip_wait -lt 15 ]; do
        if ip addr show "$WIFI_INTERFACE" | grep -q "inet "; then
            local ip_addr=$(ip addr show "$WIFI_INTERFACE" | grep "inet " | awk '{print $2}')
            log_success "Got IP address: $ip_addr"
            return 0
        fi
        sleep 1
        ip_wait=$((ip_wait + 1))
    done

    log_warning "Failed to get IP address"
    return 1
}

# ============================================================
# STEP 5: Verify and maintain connection
# ============================================================
verify_connection() {
    # Check if we have an IP
    if ! ip addr show "$WIFI_INTERFACE" | grep -q "inet "; then
        return 1
    fi

    # Try to ping gateway
    local gateway=$(ip route | grep default | grep "$WIFI_INTERFACE" | awk '{print $3}' | head -1)
    if [ -n "$gateway" ]; then
        if ping -c 1 -W 2 "$gateway" &>/dev/null; then
            return 0
        fi
    fi

    # Alternatively, check if we can reach Google DNS
    if ping -c 1 -W 2 8.8.8.8 &>/dev/null; then
        return 0
    fi

    return 1
}

# ============================================================
# MAIN LOGIC
# ============================================================

# Step 0: Cleanup
cleanup_services

# Step 1: Disable NetworkManager control
disable_networkmanager_wifi

# Step 2: Check for known networks in config
CONNECTED=false

if [ -f "$CONFIG_FILE" ]; then
    # Count configured networks
    NETWORK_COUNT=$(grep -v "^#" "$CONFIG_FILE" | grep -v "^$" | wc -l)
    log_info "Found $NETWORK_COUNT configured network(s)"

    if [ "$NETWORK_COUNT" -gt 0 ]; then
        # Bring up interface for scanning
        ip link set "$WIFI_INTERFACE" up
        sleep 2

        # Scan for available networks
        log_info "Scanning for WiFi networks..."
        SCAN_RESULT=$(iw "$WIFI_INTERFACE" scan 2>/dev/null | grep "SSID:" | sed 's/.*SSID: //' | sort | uniq)

        if [ -z "$SCAN_RESULT" ]; then
            log_warning "No networks found in scan, retrying..."
            sleep 3
            SCAN_RESULT=$(iw "$WIFI_INTERFACE" scan 2>/dev/null | grep "SSID:" | sed 's/.*SSID: //' | sort | uniq)
        fi

        if [ -n "$SCAN_RESULT" ]; then
            log_info "Available networks:"
            echo "$SCAN_RESULT" | while read net; do echo "  - $net"; done
        else
            log_warning "Could not scan for networks"
        fi

        # Try to connect to known networks
        while IFS=: read -r SSID PASSWORD; do
            # Skip empty lines and comments
            [[ -z "$SSID" || "$SSID" =~ ^# ]] && continue

            log_info "Checking for known network: $SSID"

            if echo "$SCAN_RESULT" | grep -qF "$SSID"; then
                log_info "Found known network: $SSID - attempting connection..."

                if connect_with_wpa "$SSID" "$PASSWORD"; then
                    # Verify the connection is actually working
                    sleep 2
                    if verify_connection; then
                        CONNECTED=true
                        log_success "Connected and verified: $SSID"
                        break
                    else
                        log_warning "Connected but verification failed, trying next network..."
                        killall wpa_supplicant 2>/dev/null || true
                        dhclient -r "$WIFI_INTERFACE" 2>/dev/null || true
                    fi
                else
                    log_warning "Failed to connect to $SSID"
                fi
            else
                log_info "Network '$SSID' not in range"
            fi
        done < "$CONFIG_FILE"
    fi
else
    log_warning "Config file not found: $CONFIG_FILE"
fi

# Step 3: If not connected, start AP mode
if [ "$CONNECTED" = false ]; then
    log_info "============================================"
    log_info "No known network available - Starting AP mode"
    log_info "============================================"

    # Make sure wpa_supplicant is stopped
    killall wpa_supplicant 2>/dev/null || true

    # Configure interface for AP mode
    ip link set "$WIFI_INTERFACE" down
    ip addr flush dev "$WIFI_INTERFACE"
    ip addr add "$AP_IP/24" dev "$WIFI_INTERFACE"
    ip link set "$WIFI_INTERFACE" up

    sleep 1

    # Start AP services
    if start_ap_services; then
        log_success "============================================"
        log_success "Access Point Active"
        log_success "  SSID: $AP_SSID"
        log_success "  Password: $AP_PASS"
        log_success "  IP: $AP_IP"
        log_success "============================================"
    else
        log_error "Failed to start AP mode"
        exit 1
    fi
else
    log_success "============================================"
    log_success "WiFi Client Mode Active"
    log_success "============================================"
fi

log_info "WiFi Manager completed"
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

# Check NetworkManager config (only if NM is installed)
if command -v nmcli &>/dev/null; then
    if [[ ! -f /etc/NetworkManager/conf.d/99-swarm-unmanaged.conf ]]; then
        echo "  ⚠️  MISSING: NetworkManager unmanaged config (may cause WiFi conflicts)"
    else
        echo "  ✅ OK: NetworkManager unmanaged config"
    fi
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