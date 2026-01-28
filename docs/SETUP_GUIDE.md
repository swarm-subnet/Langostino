# Swarm ROS2 Environment Setup Guide

This guide provides step-by-step instructions for setting up the complete Swarm AI Integration drone control system on Ubuntu 22.04 and 24.04.

> 📖 **Deep Dive:** For a comprehensive understanding of the software architecture and how data flows through the system, see [Chapter 3: From Data to Motion](https://substack.com/home/post/p-177453660) on our Substack.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Quick Setup](#quick-setup)
- [Manual Setup](#manual-setup)
- [Hardware Configuration](#hardware-configuration)
- [Verification](#verification)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Hardware Requirements
- **Flight Controller**: INAV 8 compatible (connected via UART/Serial)
- **LiDAR Sensor**: I2C-based distance sensor (e.g., dTOF SEN0684  on address 0x08).
- **Onboard Computer**: Raspberry Pi 5 or Ubuntu 24.04 server (Also tested on Raspberry Pi 3b+ with Ubuntu 22.04 server).
- **GPS Module**: Connected to flight controller with UART.
- **IMU**: Integrated in flight controller.

### Software Requirements
- **OS**: Ubuntu 24.04 LTS
- **Internet Connection**: For downloading packages
- **Sudo Access**: Root privileges required for installation

### Hardware Connections
| Component | Interface | Device Path | Notes |
|-----------|-----------|-------------|-------|
| Flight Controller | UART/Serial | `/dev/ttyAMA0` | 115200 baud |
| LiDAR (Down) | I2C Bus 1 | Address `0x08` | 100Hz polling |
| GPS | via FC | - | MSP protocol |
| IMU | via FC | - | MSP protocol |

---

## Quick Setup
> **Note:** This section assumes a fully working Langostino (or your specific drone). Otherwise, you will be running flight code without a drone.
### 1. Clone the Repository

```bash
cd ~
git clone <your-repo-url> swarm-ros
cd swarm-ros
```

### 2. Run Setup Script

```bash
sudo ./setup.sh
```

**What it does:**
- ✅ Installs ROS2 Humble (if needed)
- ✅ Installs all dependencies (Python, I2C, UART)
- ✅ Creates Python virtual environment for AI packages
- ✅ Configures hardware permissions (I2C, UART)
- ✅ Configures network (WiFi AP + Ethernet static IP)
- ✅ Fixes workspace ownership (prevents permission errors)
- ✅ Builds ROS2 workspace
- ✅ Installs PM2 process manager (auto-start on boot)

**Available Options:**
- `--skip-ros`: Skip ROS2 installation (if already installed)
- `--skip-hardware-check`: Skip hardware connectivity checks
- `--skip-pm2`: Skip PM2 installation (installed by default)

**Example with options:**
```bash
# Skip PM2 if you don't want process management
sudo ./setup.sh --skip-pm2

# Skip ROS2 install if already present
sudo ./setup.sh --skip-ros
```

> **Note**: The script automatically fixes workspace permissions and cleans old build artifacts, preventing common `colcon build` permission errors.

### 3. Log Out and Back In

**Important**: You must log out and log back in for group changes to take effect!

```bash
logout
# Or reboot
sudo reboot
```

### 4. Verify Installation

```bash
cd ~/swarm-ros
./verify_setup.sh
```

### 5. Launch the System

**Option A: Using PM2 (Recommended - Auto-installed)**

```bash
# Launch with PM2 process manager
./launch.sh

# View status
pm2 list

# View logs in real-time
pm2 logs

# Stop the system
pm2 stop swarm-ros-launched
```

**Option B: Direct ROS2 Launch**

```bash
# Source the environment (if not in .bashrc)
source /opt/ros/humble/setup.bash
source ~/swarm-ros/install/setup.bash

# Launch directly (runs in foreground)
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

You can launch each node separately from the same terminal using PM2. See the [commands guide](COMMANDS_GUIDE.md).

> **PM2 Benefits**: Auto-restart on crash, survives reboots, centralized logging, resource monitoring

---

## Manual Setup

If you prefer to set up components individually:

### Step 1: Install ROS2 Humble

```bash
# Set up locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-jazzy-ros-base
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 2: Install System Dependencies

```bash
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    python3-venv \
    libi2c-dev \
    i2c-tools \
    python3-smbus \
    python3-serial \
    python3-numpy \
    python3-scipy
```

### Step 3: Install Python Dependencies

**System packages (for ROS nodes):**
```bash
pip3 install --upgrade pip
pip3 install numpy scipy pyserial smbus2
```

**Virtual environment for AI Flight Node:**
```bash
# Create virtual environment
python3 -m venv ~/ai_flight_node_env

# Activate virtual environment
source ~/ai_flight_node_env/bin/activate

# Install AI packages from requirements file
cd ~/swarm-ros
pip install --upgrade pip
pip install -r ai_model_requirements.txt

# Deactivate (optional)
deactivate
```

The `ai_model_requirements.txt` file contains:
- numpy>=1.24,<2.0
- typing-extensions
- gymnasium
- stable-baselines3
- torch

### Step 4: Configure I2C (for LiDAR)

```bash
# Enable I2C module
sudo modprobe i2c-dev
echo "i2c-dev" | sudo tee -a /etc/modules

# Add user to i2c group
sudo usermod -a -G i2c $USER

# Set I2C permissions
echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' \
  | sudo tee /etc/udev/rules.d/99-i2c.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 5: Configure UART (for Flight Controller)

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set UART permissions
cat << EOF | sudo tee /etc/udev/rules.d/99-serial.rules
KERNEL=="ttyAMA[0-9]*", GROUP="dialout", MODE="0660"
KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
KERNEL=="ttyACM[0-9]*", GROUP="dialout", MODE="0660"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 6: Configure Network (WiFi AP + Ethernet)

```bash
# Run the network setup script
cd ~/swarm-ros
chmod +x network_setup.sh
sudo ./network_setup.sh
```

**What it configures:**
- Static IP on `eth0`: `192.168.10.1/24`
- DHCP on `wlan0` for connecting to known WiFi networks
- Automatic WiFi Access Point fallback
- AP SSID: `Swarm_AP`
- AP Password: `swarmascend`
- DHCP server range: `192.168.10.10 - 192.168.10.50`
- Automatic network manager service

The system will automatically:
1. Try to connect to known WiFi networks on boot
2. If no known network is found, create an Access Point
3. Allow you to SSH into the drone at `192.168.10.1`

To add known WiFi networks, edit `/etc/wifi_networks.conf`:
```bash
sudo nano /etc/wifi_networks.conf
```

Add networks in the format:
```
SSID:password
MyHomeWiFi:mypassword123
WorkNetwork:workpassword
```

### Step 7: Build ROS2 Workspace

```bash
cd ~/swarm-ros
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Step 8: Configure Environment

Add to `~/.bashrc`:

```bash
# ROS2 Environment
source /opt/ros/jazzy/setup.bash
source ~/swarm-ros/install/setup.bash

# ROS2 Configuration
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Apply changes:
```bash
source ~/.bashrc
```

---

## Hardware Configuration

### I2C LiDAR Setup

#### Enable I2C on Raspberry Pi

Edit `/boot/firmware/config.txt`:
```bash
sudo nano /boot/firmware/config.txt
```

Add or uncomment:
```
dtparam=i2c_arm=on
```

Reboot:
```bash
sudo reboot
```

#### Verify I2C Connection

Check if I2C device is detected:
```bash
i2cdetect -y 1
```

You should see `08` in the output grid (LiDAR address).

### UART Flight Controller Setup

#### Enable UART on Raspberry Pi

Edit `/boot/firmware/config.txt`:
```bash
sudo nano /boot/firmware/config.txt
```

Add:
```
enable_uart=1
```

#### Disable Serial Console (if needed)

Edit `/boot/firmware/cmdline.txt` and remove:
```
console=serial0,115200
```

#### Verify UART Connection

```bash
ls -l /dev/ttyAMA0
```

Should show device with `crw-rw----` permissions and group `dialout`.

### Network Setup

The Swarm system is configured with a dual-network setup for maximum flexibility:

#### Network Interfaces

| Interface | Mode | IP Address | Purpose |
|-----------|------|------------|---------|
| `eth0` | Static IP | `192.168.10.1/24` | Main communication interface |
| `wlan0` | DHCP/AP | Auto/`192.168.10.1` | WiFi client or Access Point |

#### WiFi Auto-Manager

The system includes an automatic WiFi manager that:
1. **Scans** for known WiFi networks on boot
2. **Connects** to the first available known network (client mode)
3. **Falls back** to Access Point mode if no known networks found

#### Access Point Details

When in AP mode:
- **SSID**: `Swarm_AP`
- **Password**: `swarmascend`
- **IP Address**: `192.168.10.1`
- **DHCP Range**: `192.168.10.10 - 192.168.10.50`

#### Connect to the Swarm

**Via Access Point:**
```bash
# Connect your laptop to Swarm_AP WiFi network
# Password: swarmascend

# SSH into the drone
ssh pi@192.168.10.1
```

**Via Ethernet:**
```bash
# Connect ethernet cable to your laptop
# Configure your laptop's ethernet to 192.168.10.x (x != 1)

# SSH into the drone
ssh pi@192.168.10.1
```

#### Add Known WiFi Networks

Edit the known networks file:
```bash
sudo nano /etc/wifi_networks.conf
```

Add networks (one per line):
```
SSID:password
MyHomeWiFi:mypassword123
WorkNetwork:workpassword
```

Restart the WiFi manager service:
```bash
sudo systemctl restart wifi-manager.service
```

#### Network Service Management

```bash
# Check WiFi manager status
sudo systemctl status wifi-manager.service

# View WiFi manager logs
sudo journalctl -u wifi-manager.service -f

# Manually start Access Point
sudo systemctl start hostapd
sudo systemctl start dnsmasq

# Manually stop Access Point
sudo systemctl stop hostapd
sudo systemctl stop dnsmasq

# Check current network status
ip addr show
iwconfig
```

---

## Verification

### Run Verification Script

```bash
cd ~/swarm-ros
./verify_setup.sh
```

The script checks:
- ✓ Ubuntu version
- ✓ ROS2 Humble installation
- ✓ Python dependencies
- ✓ I2C configuration and LiDAR detection
- ✓ UART configuration and FC connection
- ✓ Workspace build status
- ✓ Launch files and configuration

### Manual Hardware Tests

#### Test I2C Communication

```bash
# Scan I2C bus
i2cdetect -y 1

# Read from LiDAR (if using i2c-tools)
i2cget -y 1 0x08 0x24 w
```

#### Test UART Communication

```bash
# Check if device is accessible
ls -l /dev/ttyAMA0

# Monitor serial data (Ctrl+C to exit)
cat /dev/ttyAMA0
```

#### Test ROS2 Nodes

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/swarm-ros/install/setup.bash

# List available nodes
ros2 pkg executables swarm_ai_integration

# Test individual node
ros2 run swarm_ai_integration lidar_reader_node
```

---

## Troubleshooting

For comprehensive troubleshooting, see [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md).

**Setup-specific quick fixes:**

| Issue | Quick Solution |
|-------|---------------|
| **ros2: command not found** | `source /opt/ros/jazzy/setup.bash` |
| **Permission denied (I2C/UART)** | `sudo usermod -a -G i2c,dialout $USER`, then logout |
| **Build permission errors** | `sudo chown -R $USER:$USER ~/swarm-ros && rm -rf build install log` |
| **Virtual env in /root** | `sudo rm -rf /root/ai_flight_node_env && python3 -m venv ~/ai_flight_node_env` |
| **Cannot connect to Swarm_AP** | `sudo systemctl restart wifi-manager.service` |
| **Package not found** | `rm -rf build install log && colcon build --symlink-install` |

See the following sections in the Troubleshooting Guide for detailed solutions:
- [Build & Installation Issues](TROUBLESHOOTING_GUIDE.md#build--installation-issues)
- [Hardware Issues](TROUBLESHOOTING_GUIDE.md#hardware-issues)
- [Network & Connectivity Issues](TROUBLESHOOTING_GUIDE.md#network--connectivity-issues)


## Configuration Files

### Main Configuration

**Location**: `src/swarm_ai_integration/config/swarm_params.yaml`

Key parameters to configure:
- Flight controller serial port
- LiDAR I2C address
- Safety limits (altitude, velocity, battery)
- AI model path

### Launch Configuration

**Location**: `src/swarm_ai_integration/launch/swarm_ai_launch.py`

Modify to enable/disable nodes as needed.


## Process Management with PM2

PM2 is **installed by default** by the setup script and provides powerful process management features.

### Quick Start

```bash
# Start the ROS system with PM2
./launch.sh

# Check status
pm2 list
```

### Common PM2 Commands

```bash
# View logs in real-time (all processes)
pm2 logs

# View logs for specific process
pm2 logs swarm-ros-launched

# Monitor resource usage (CPU, memory)
pm2 monit

# Stop the system
pm2 stop swarm-ros-launched

# Restart the system
pm2 restart swarm-ros-launched

# Stop all PM2 processes
pm2 stop all

# Restart all PM2 processes
pm2 restart all

# Delete process from PM2
pm2 delete swarm-ros-launched

# Show detailed process info
pm2 show swarm-ros-launched
```

### Auto-Start on Boot

PM2 is **automatically configured** to start on boot by the setup script:

```bash
# Verify startup configuration
pm2 startup

# Save current process list (auto-start this on boot)
pm2 save

# List saved processes
pm2 list
```

After running `pm2 save`, your ROS system will automatically start when the Raspberry Pi boots!

### Disable Auto-Start

If you don't want auto-start:

```bash
# Remove from startup
pm2 unstartup systemd

# Or delete specific process
pm2 delete swarm-ros-launched
pm2 save
```

### PM2 Log Files

PM2 stores logs in `~/.pm2/logs/`:

```bash
# View log file locations
ls -la ~/.pm2/logs/

# Tail error logs
tail -f ~/.pm2/logs/swarm-ros-launched-error.log

# Tail output logs
tail -f ~/.pm2/logs/swarm-ros-launched-out.log

# Clear all logs
pm2 flush
```

### Advanced PM2 Features

```bash
# Restart on file changes (development)
pm2 start launch.sh --watch

# Set environment variables
pm2 start launch.sh --name swarm-ai --env production

# Limit memory and restart on threshold
pm2 start launch.sh --max-memory-restart 1G

# Run multiple instances (if needed)
pm2 scale swarm-ros-launched 2
```
