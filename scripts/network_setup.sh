#!/bin/bash
# Initial network setup script for Swarm system
# Configures eth0 with a static IP and wlan0 for DHCP
# Installs and configures dnsmasq and hostapd for AP functionality
# Handles systemd-resolved conflicts and service masking issues

set -e

AP_SSID="Swarm_AP"
AP_PASS="swarmascend"
AP_IP="192.168.10.1"

echo "=== [Swarm Setup] Initial network configuration ==="

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
echo "[Swarm Setup] Installing dnsmasq and hostapd..."
sudo apt update
sudo apt install -y dnsmasq hostapd wireless-tools iw

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
echo "[Swarm Setup] Configuring Netplan..."

NETPLAN_FILE="/etc/netplan/99-swarm-network.yaml"

# Create proper netplan configuration without access points requirement
sudo bash -c "cat > $NETPLAN_FILE" <<EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.10.1/24
      optional: true
EOF

# Set correct permissions for netplan file (only root can read/write)
sudo chmod 600 $NETPLAN_FILE
sudo chown root:root $NETPLAN_FILE

echo "[Swarm Setup] Applying netplan configuration..."
sudo netplan generate
sudo netplan apply || true  # Continue even if netplan apply has warnings

# Note: wlan0 will be managed by hostapd when in AP mode, 
# or by wpa_supplicant/NetworkManager when in client mode

###############################################
# 6. DNSMASQ CONFIGURATION (permanent)
###############################################
echo "[Swarm Setup] Dnsmasq setup..."

# Backup original config if exists
if [ -f /etc/dnsmasq.conf ]; then
    sudo cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup
fi

sudo bash -c "cat > /etc/dnsmasq.conf" <<EOF
# Swarm AP Configuration
interface=wlan0
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
echo "[Swarm Setup] Hostapd setup..."

sudo bash -c "cat > /etc/hostapd/hostapd.conf" <<EOF
interface=wlan0
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
EOF

sudo bash -c "echo 'DAEMON_CONF=\"/etc/hostapd/hostapd.conf\"' > /etc/default/hostapd"

###############################################
# 8. CREATE KNOWN NETWORKS FILE
###############################################
echo "[Swarm Setup] Creating known networks file..."

if [ ! -f /etc/wifi_networks.conf ]; then
    sudo bash -c "cat > /etc/wifi_networks.conf" <<EOF
# WiFi Networks Configuration
# Format: SSID:password
# Example: MyNetwork:mypassword123
EOF
fi

###############################################
# 9. CREATE IMPROVED WIFI MANAGER
###############################################
echo "[Swarm Setup] Creating wifi_manager.sh..."

sudo bash -c "cat > /usr/local/bin/wifi_manager.sh" <<'EOF'
#!/bin/bash

WIFI_INTERFACE="wlan0"
CONFIG_FILE="/etc/wifi_networks.conf"
AP_SSID="Swarm_AP"
AP_PASS="swarmascend"
AP_IP="192.168.10.1"
MAX_RETRIES=3

echo "[WiFi Manager] Starting network check..."

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
    
    # Request DHCP
    sudo dhclient "$WIFI_INTERFACE" 2>/dev/null
    
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

sudo bash -c "cat > /etc/systemd/system/wifi-manager.service" <<EOF
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

sudo systemctl daemon-reload
sudo systemctl enable wifi-manager.service

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
# 13. FINAL MESSAGE
###############################################
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✅ Network Setup Completed Successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Configuration:"
echo " • eth0: Static IP 192.168.10.1/24"
echo " • wlan0: Managed by WiFi Manager"
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