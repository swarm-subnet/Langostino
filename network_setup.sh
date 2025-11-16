#!/bin/bash
# Initial network setup script for Swarm system
# Configures eth0 with a static IP and wlan0 for DHCP
# Installs and configures dnsmasq and hostapd for AP functionality

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

sudo systemctl stop dnsmasq || true
sudo systemctl stop hostapd || true
sudo systemctl disable dnsmasq || true
sudo systemctl disable hostapd || true

###############################################
# 2. CONFIGURE NETPLAN
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
# 3. DNSMASQ CONFIGURATION (permanent)
###############################################
echo "[Swarm Setup] Dnsmasq setup..."

sudo bash -c "cat > /etc/dnsmasq.conf" <<EOF
interface=wlan0
dhcp-range=192.168.10.10,192.168.10.50,255.255.255.0,24h
EOF

###############################################
# 4. HOSTAPD CONFIGURATION (permanent)
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
EOF

sudo bash -c "echo 'DAEMON_CONF=\"/etc/hostapd/hostapd.conf\"' > /etc/default/hostapd"

###############################################
# 5.  CREATE KNOWN NETWORKS FILE
###############################################
echo "[Swarm Setup] Creating known networks file..."

sudo bash -c "cat > /etc/wifi_networks.conf" <<EOF
# Format:
# SSID:password
EOF

##############################################################
# 6. wifi_manager.sh
##############################################################
echo "[Swarm Setup] Creating wifi_manager.sh..."

sudo bash -c "cat > /usr/local/bin/wifi_manager.sh" <<'EOF'
#!/bin/bash
set -e

WIFI_INTERFACE="wlan0"
CONFIG_FILE="/etc/wifi_networks.conf"
AP_SSID="Swarm_AP"
AP_PASS="swarmascend"
AP_IP="192.168.10.1"

echo "[WiFi Manager] Starting network check..."

# Deactivate AP services
sudo systemctl stop hostapd || true
sudo systemctl stop dnsmasq || true

# Scan for available networks
AVAILABLE=$(nmcli -t -f SSID dev wifi list | sort | uniq)
echo "[WiFi Manager] Detected netowrks:"
echo "$AVAILABLE"

CONNECTED=false

# Try to connect to known networks
while IFS=: read -r SSID PASSWORD; do
    if echo "$AVAILABLE" | grep -qx "$SSID"; then
        echo "[WiFi Manager] Trying to connect to $SSID..."
        nmcli dev wifi connect "$SSID" password "$PASSWORD" ifname "$WIFI_INTERFACE" >/dev/null 2>&1 && CONNECTED=true && break
    fi
done < "$CONFIG_FILE"

if [ "$CONNECTED" = true ]; then
    echo "[WiFi Manager] ✅ Successfully connected to a known network."
else
    echo "[WiFi Manager] ⚠️ No known network found, creating Access Point..."
    ip link set $WIFI_INTERFACE down
    ip addr flush dev $WIFI_INTERFACE
    ip addr add $AP_IP/24 dev $WIFI_INTERFACE
    ip link set $WIFI_INTERFACE up

    # Start AP services
    systemctl start dnsmasq
    systemctl start hostapd
    echo "[WiFi Manager] ✅ AP active: SSID=$AP_SSID, PASS=$AP_PASS"
fi
EOF

sudo chmod +x /usr/local/bin/wifi_manager.sh


##############################################################
# 7. SYSTEMD SERVICE FOR WIFI MANAGER
##############################################################
echo "[Swarm Setup] Creating systemd service for wifi_manager.sh..."

sudo bash -c "cat > /etc/systemd/system/wifi-manager.service" <<EOF
[Unit]
Description=WiFi Auto Manager (client or AP)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/local/bin/wifi_manager.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable wifi-manager.service

###############################################
# 8. FINAL MESSAGE
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
echo " - hostapd configuraed (for AP mode, not running by default)"
echo ""
echo "To start the Access Point mode manually, run:"
echo "    sudo systemctl start dnsmasq"
echo "    sudo systemctl start hostapd"
echo ""
echo "To stop the Access Point mode, run:"
echo "    sudo systemctl stop dnsmasq"
echo "    sudo systemctl stop hostapd"
echo ""
echo "=============================================="
echo "   ✔ Swarm Network Setup ready to go!"
echo "=============================================="
