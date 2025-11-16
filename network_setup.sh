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
# 1. INSTALL DNSMASQ Y HOSTAPD
###############################################
echo "[Swarm Setup] Installing dnsmasq and hostapd..."
sudo apt update
sudo apt install -y dnsmasq hostapd

# Stop services first
sudo systemctl stop dnsmasq || true
sudo systemctl stop hostapd || true

###############################################
# 2. HANDLE SYSTEMD-RESOLVED PORT 53 CONFLICT
###############################################
echo "[Swarm Setup] Handling systemd-resolved port 53 conflict..."

# Check if systemd-resolved is using port 53
if sudo lsof -i:53 | grep -q systemd-r; then
    echo "[Swarm Setup] systemd-resolved detected on port 53, disabling DNS stub listener..."
    
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
    if sudo lsof -i:53 | grep -q "LISTEN"; then
        echo "[Swarm Setup] WARNING: Port 53 still in use, attempting to stop systemd-resolved..."
        sudo systemctl stop systemd-resolved
        sudo systemctl disable systemd-resolved
        
        # Create a static resolv.conf
        sudo bash -c "cat > /etc/resolv.conf" <<EOF
nameserver 8.8.8.8
nameserver 8.8.4.4
nameserver 1.1.1.1
EOF
    fi
fi

###############################################
# 3. CHECK AND UNMASK HOSTAPD IF NEEDED
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
# 4. CONFIGURE NETPLAN
###############################################
echo "[Swarm Setup] Configuring Netplan..."

NETPLAN_FILE="/etc/netplan/99-swarm-network.yaml"

sudo bash -c "cat > $NETPLAN_FILE" <<EOF
network:
  version: 2
  renderer: networkd

  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.10.1/24

  wifis:
    wlan0:
      dhcp4: yes
      optional: true
EOF

sudo netplan apply

###############################################
# 5. DNSMASQ CONFIGURATION (permanent)
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
# 6. HOSTAPD CONFIGURATION (permanent)
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
# 7. CREATE KNOWN NETWORKS FILE
###############################################
echo "[Swarm Setup] Creating known networks file..."

sudo bash -c "cat > /etc/wifi_networks.conf" <<EOF
# Format:
# SSID:password
EOF

###############################################
# 8. CREATE IMPROVED WIFI MANAGER
###############################################
echo "[Swarm Setup] Creating wifi_manager.sh..."

sudo bash -c "cat > /usr/local/bin/wifi_manager.sh" <<'EOF'
#!/bin/bash
set -e

WIFI_INTERFACE="wlan0"
CONFIG_FILE="/etc/wifi_networks.conf"
AP_SSID="Swarm_AP"
AP_PASS="swarmascend"
AP_IP="192.168.10.1"
MAX_RETRIES=3

echo "[WiFi Manager] Starting network check..."

# Function to check if port 53 is free
check_port_53() {
    if sudo lsof -i:53 | grep -q "LISTEN"; then
        echo "[WiFi Manager] Port 53 is in use, attempting to free it..."
        
        # Try to kill systemd-resolved if it's using the port
        if sudo lsof -i:53 | grep -q systemd-r; then
            echo "[WiFi Manager] Stopping systemd-resolved..."
            sudo systemctl stop systemd-resolved || true
            sleep 2
        fi
        
        # Kill any remaining process on port 53
        local pids=$(sudo lsof -ti:53)
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
        if systemctl list-unit-files | grep -q "hostapd.service.*masked"; then
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

# Deactivate AP services first
sudo systemctl stop hostapd 2>/dev/null || true
sudo systemctl stop dnsmasq 2>/dev/null || true

# Check if we have NetworkManager
if command -v nmcli &> /dev/null; then
    # Scan for available networks using NetworkManager
    AVAILABLE=$(nmcli -t -f SSID dev wifi list 2>/dev/null | sort | uniq)
    echo "[WiFi Manager] Detected networks:"
    echo "$AVAILABLE"
    
    CONNECTED=false
    
    # Try to connect to known networks
    if [ -f "$CONFIG_FILE" ]; then
        while IFS=: read -r SSID PASSWORD; do
            # Skip empty lines and comments
            [[ -z "$SSID" || "$SSID" =~ ^# ]] && continue
            
            if echo "$AVAILABLE" | grep -qx "$SSID"; then
                echo "[WiFi Manager] Trying to connect to $SSID..."
                if nmcli dev wifi connect "$SSID" password "$PASSWORD" ifname "$WIFI_INTERFACE" 2>/dev/null; then
                    CONNECTED=true
                    break
                fi
            fi
        done < "$CONFIG_FILE"
    fi
    
    if [ "$CONNECTED" = true ]; then
        echo "[WiFi Manager] ✅ Successfully connected to a known network."
    else
        echo "[WiFi Manager] ⚠️ No known network found, creating Access Point..."
        
        # Configure interface for AP mode
        sudo ip link set $WIFI_INTERFACE down
        sudo ip addr flush dev $WIFI_INTERFACE
        sudo ip addr add $AP_IP/24 dev $WIFI_INTERFACE
        sudo ip link set $WIFI_INTERFACE up
        
        # Start AP services with error handling
        if start_ap_services; then
            echo "[WiFi Manager] ✅ AP active: SSID=$AP_SSID, PASS=$AP_PASS"
        else
            echo "[WiFi Manager] ❌ Failed to start AP mode"
            exit 1
        fi
    fi
else
    echo "[WiFi Manager] NetworkManager not found, starting AP mode directly..."
    
    # Configure interface for AP mode
    sudo ip link set $WIFI_INTERFACE down
    sudo ip addr flush dev $WIFI_INTERFACE
    sudo ip addr add $AP_IP/24 dev $WIFI_INTERFACE
    sudo ip link set $WIFI_INTERFACE up
    
    # Start AP services with error handling
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
# 9. SYSTEMD SERVICE FOR WIFI MANAGER
###############################################
echo "[Swarm Setup] Creating systemd service for wifi_manager.sh..."

sudo bash -c "cat > /etc/systemd/system/wifi-manager.service" <<EOF
[Unit]
Description=WiFi Auto Manager (client or AP)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/wifi_manager.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable wifi-manager.service
sudo systemctl restart wifi-manager.service

###############################################
# 10. TEST SERVICES
###############################################
echo "[Swarm Setup] Testing service configurations..."

# Test if hostapd can start
echo -n "Testing hostapd... "
if sudo systemctl start hostapd 2>/dev/null; then
    echo "✅ OK"
else
    echo "⚠️ Failed, attempting to fix..."
    sudo systemctl unmask hostapd 2>/dev/null || true
    if sudo systemctl start hostapd 2>/dev/null; then
        echo "✅ Fixed and OK"
    else
        echo "❌ Still failing, check logs with: sudo journalctl -u hostapd"
    fi
fi

# Test if dnsmasq can start
echo -n "Testing dnsmasq... "
check_port_53
if sudo systemctl start dnsmasq 2>/dev/null; then
    echo "✅ OK"
else
    echo "⚠️ Failed, attempting to fix restarting the service..."
    if sudo systemctl restart dnsmasq 2>/dev/null; then
        echo "✅ Fixed and OK"
    else
        echo "❌ Still failing, check logs with: sudo journalctl -u dnsmasq"
    fi
fi

###############################################
# 11. FINAL MESSAGE
###############################################
echo ""
echo "=============================================="
echo "  ✔ Network setup script completed successfully"
echo "=============================================="
echo ""
echo "Your Swarm system network is configured as follows:"
echo " - eth0 static IP: 192.168.10.1/24"
echo " - wlan0 uses DHCP to connect to known networks"
echo " - dnsmasq configured (for AP mode, not running by default)"
echo " - hostapd configured (for AP mode, not running by default)"
echo " - systemd-resolved conflicts handled"
echo ""
echo "To add known WiFi networks, edit:"
echo "    sudo nano /etc/wifi_networks.conf"
echo "    Format: SSID:password"
echo ""
echo "To start the Access Point mode manually, run:"
echo "    sudo /usr/local/bin/wifi_manager.sh"
echo ""
echo "To check service status:"
echo "    sudo systemctl status wifi-manager"
echo "    sudo systemctl status dnsmasq"
echo "    sudo systemctl status hostapd"
echo ""
echo "=============================================="
echo "   ✔ Swarm Network Setup ready to go!"
echo "=============================================="