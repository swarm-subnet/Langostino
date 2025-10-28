# Swarm ROS2 Environment Setup Guide

This guide provides step-by-step instructions for setting up the complete Swarm AI Integration drone control system on Ubuntu 22.04.

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
- **Flight Controller**: INAV 7 compatible (connected via UART/Serial)
- **LiDAR Sensor**: I2C-based distance sensor (e.g., TFMini-S on address 0x08)
- **Onboard Computer**: Raspberry Pi 4 or Ubuntu 22.04 server
- **GPS Module**: Connected to flight controller
- **IMU**: Integrated in flight controller

### Software Requirements
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
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

**Options:**
- `--skip-ros`: Skip ROS2 installation (if already installed)
- `--skip-hardware-check`: Skip hardware connectivity checks
- `--install-pm2`: Install PM2 process manager for auto-start

**Example with options:**
```bash
sudo ./setup.sh --install-pm2
```

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

```bash
# Source the environment
source /opt/ros/humble/setup.bash
source ~/swarm-ros/install/setup.bash

# Launch using PM2 (if installed)
./launch.sh

# Or launch directly
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

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
sudo apt install -y ros-humble-ros-base
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

### Step 6: Build ROS2 Workspace

```bash
cd ~/swarm-ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Step 7: Configure Environment

Add to `~/.bashrc`:

```bash
# ROS2 Environment
source /opt/ros/humble/setup.bash
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

### ROS2 Issues

**Problem**: `ros2: command not found`

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to make permanent
```

**Problem**: Package not found after building

**Solution**:
```bash
cd ~/swarm-ros
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### I2C Issues

**Problem**: `Could not open file /dev/i2c-1: Permission denied`

**Solution**:
```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER
# Log out and back in
```

**Problem**: No device at address 0x08

**Solution**:
- Check LiDAR is powered on
- Verify wiring (SDA, SCL, VCC, GND)
- Check if I2C is enabled in config.txt
- Try scanning: `i2cdetect -y 1`

### UART Issues

**Problem**: `/dev/ttyAMA0: Permission denied`

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Problem**: UART device not found

**Solution**:
- Check UART is enabled in `/boot/firmware/config.txt`
- Verify physical connection to flight controller
- Try alternative devices: `/dev/ttyUSB0` or `/dev/ttyACM0`
- Update config in `swarm_params.yaml`:
  ```yaml
  fc_comms_node:
    ros__parameters:
      serial_port: "/dev/ttyUSB0"  # Change as needed
  ```

### Build Issues

**Problem**: CMake or compilation errors

**Solution**:
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
cd ~/swarm-ros
rm -rf build install log
colcon build --symlink-install
```

**Problem**: Python import errors

**Solution**:
```bash
# Reinstall system Python dependencies
pip3 install --upgrade numpy scipy pyserial smbus2

# Reinstall AI packages in virtual environment
source ~/ai_flight_node_env/bin/activate
pip install --upgrade pip
pip install -r ~/swarm-ros/ai_model_requirements.txt
deactivate
```

### Runtime Issues

**Problem**: Nodes crash on startup

**Solution**:
- Check logs: `ros2 topic echo /rosout`
- Verify hardware is connected
- Check configuration in `config/swarm_params.yaml`
- Run verification: `./verify_setup.sh`

**Problem**: No GPS fix

**Solution**:
- Move to open sky location
- Wait 30-60 seconds for GPS lock
- Check GPS satellites: `ros2 topic echo /fc/gps_satellites`
- Verify HDOP: `ros2 topic echo /fc/gps_hdop`

---

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

---

## Process Management with PM2

If PM2 is installed:

```bash
# Start system
./launch.sh

# View status
pm2 list

# View logs
pm2 logs swarm-ros-launched

# Stop system
pm2 stop swarm-ros-launched

# Restart system
pm2 restart swarm-ros-launched

# Setup auto-start on boot
pm2 startup
pm2 save
```

---

## Safety Checklist

Before first flight:

- [ ] All hardware connections verified
- [ ] GPS has good fix (6+ satellites, HDOP < 2.0)
- [ ] LiDAR reading correctly
- [ ] Battery voltage correct
- [ ] Flight controller armed successfully
- [ ] Safety monitor active
- [ ] Home position set correctly
- [ ] Custom RTH tested in simulation
- [ ] Emergency stop procedure understood

---

## Support

For issues or questions:
- Check logs: `ros2 topic echo /rosout`
- Run diagnostics: `./verify_setup.sh`
- Review safety status: `ros2 topic echo /safety/status`
- Check flight logs: `flight-logs/` directory

---

## Next Steps

After successful setup:

1. **Configure Parameters**: Edit `config/swarm_params.yaml` for your hardware
2. **Test in Simulation**: Use AI adapter simulator nodes
3. **Calibrate Sensors**: Ensure GPS, IMU, and LiDAR are accurate
4. **Load AI Model**: Place trained model in configured path
5. **Ground Test**: Test with props off, verify all nodes working
6. **First Flight**: Test custom RTH and landing in safe environment

---

**Last Updated**: 2024-10-27
