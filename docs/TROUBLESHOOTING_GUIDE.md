# Troubleshooting Guide

Comprehensive troubleshooting reference for the Swarm AI Integration system.

---

## Table of Contents

1. [Quick Diagnostics](#quick-diagnostics)
2. [System & ROS2 Issues](#system--ros2-issues)
3. [Hardware Issues](#hardware-issues)
   - [Raspberry Pi Boot Configuration](#raspberry-pi-boot-configuration-bootfirmwareconfigtxt)
   - [I2C LiDAR Issues](#i2c-lidar-issues)
   - [Flight Controller UART Issues](#flight-controller-uart-issues)
   - [GPS Issues](#gps-issues)
4. [Build & Installation Issues](#build--installation-issues)
5. [Network & Connectivity Issues](#network--connectivity-issues)
6. [Runtime & Node Issues](#runtime--node-issues)
7. [Parameter & Configuration Issues](#parameter--configuration-issues)
8. [Performance Issues](#performance-issues)
9. [Safety & Emergency Procedures](#safety--emergency-procedures)

---

## Quick Diagnostics

### Run System Diagnostics

```bash
# Check system status
cd ~/swarm-ros
./verify_setup.sh

# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Check PM2 processes
pm2 list
```

### Check Logs

```bash
# PM2 logs (all nodes)
pm2 logs

# Specific node logs
pm2 logs fc_adapter --lines 100

# ROS2 console output
ros2 topic echo /rosout
```

### Quick Hardware Checks

```bash
# I2C LiDAR
i2cdetect -y 1

# Serial FC
ls -l /dev/ttyAMA0
sudo lsof /dev/ttyAMA0

# GPS status
ros2 topic echo /fc/gps_position

# Groups
groups  # Should include i2c, dialout
```

---

## System & ROS2 Issues

### System Won't Start

#### `ros2: command not found`

**Cause:** ROS2 environment not sourced.

**Solution:**
```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/swarm-ros/install/setup.bash

# Make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/swarm-ros/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

#### Workspace Not Built

**Symptoms:**
- Package not found errors
- Missing install directory

**Solution:**
```bash
cd ~/swarm-ros
ls -l install/  # Should contain setup.bash and package folders

# If missing, rebuild
colcon build --symlink-install
source install/setup.bash
```

---

#### Wrong ROS_DISTRO

**Check:**
```bash
echo $ROS_DISTRO
# Should show: humble (for 22.04) or jazzy (for 24.04)
```

**Solution:**
```bash
source /opt/ros/humble/setup.bash
```

---

### Package Not Found After Building

**Solution:**
```bash
cd ~/swarm-ros

# Clean old artifacts
rm -rf build install log

# Rebuild
colcon build --symlink-install
source install/setup.bash
```

---

## Hardware Issues

### Raspberry Pi Boot Configuration (`/boot/firmware/config.txt`)

The Raspberry Pi uses `/boot/firmware/config.txt` to configure hardware interfaces at boot time. Incorrect configuration here is a common cause of I2C and UART communication failures with the flight controller and sensors.

#### Verifying Current Configuration

```bash
# View current config.txt
cat /boot/firmware/config.txt

# Check if I2C is enabled
grep -E "^dtparam=i2c" /boot/firmware/config.txt

# Check UART settings
grep -E "^enable_uart|^dtoverlay=disable-bt|^\[pi5\]|^dtoverlay=uart" /boot/firmware/config.txt
```

#### Required Settings for Swarm System

Your `/boot/firmware/config.txt` should contain these lines:

```ini
# I2C for LiDAR sensor
dtparam=i2c_arm=on

# UART for Flight Controller
enable_uart=1
dtoverlay=disable-bt

# Pi 5 ONLY - required for GPIO 14/15 UART
[pi5]
dtoverlay=uart0-pi5
```

#### Common Configuration Errors

| Error | Symptom | Solution |
|-------|---------|----------|
| Missing `dtparam=i2c_arm=on` | `/dev/i2c-1` doesn't exist | Add the line and reboot |
| Missing `enable_uart=1` | `/dev/ttyAMA0` doesn't exist | Add the line and reboot |
| Missing `dtoverlay=disable-bt` | UART conflicts with Bluetooth | Add the line and reboot |
| Pi 5 missing `[pi5]` section | UART not mapped to GPIO 14/15 | Add `[pi5]` section with `dtoverlay=uart0-pi5` |
| Duplicate entries | Unpredictable behavior | Remove duplicates, keep one of each |
| Syntax errors (typos) | Setting not applied | Check spelling exactly |

#### Pi 3 vs Pi 5 UART Differences

**Raspberry Pi 3B / 3B+:**
- UART is on `/dev/ttyAMA0` by default after disabling Bluetooth
- Only needs `enable_uart=1` and `dtoverlay=disable-bt`
- Tested with: `setup_22_04.sh` on Raspberry Pi 3B and 3B+

**Raspberry Pi 5:**
- UART requires explicit overlay configuration
- Must add `[pi5]` section with `dtoverlay=uart0-pi5`
- Creates `/dev/ttyAMA0` mapped to GPIO 14 (TX) and GPIO 15 (RX)
- May also show `/dev/ttyAMA10` (different UART peripheral)
- Tested with: `setup_24_04.sh` on Raspberry Pi 5 (4GB and 8GB versions)

> **Note:** Raspberry Pi 4 has not been tested yet. It may work with either setup script, but UART configuration could differ. If you test on Pi 4, please report your results.

```bash
# Check Pi model
cat /proc/device-tree/model

# List available serial devices
ls -la /dev/ttyAMA*
```

#### Fixing Missing I2C Configuration

```bash
# Check if I2C is enabled
ls /dev/i2c*
# Should show /dev/i2c-1

# If missing, add to config.txt
sudo nano /boot/firmware/config.txt

# Add this line (if not present):
dtparam=i2c_arm=on

# Save and reboot
sudo reboot

# After reboot, verify
i2cdetect -y 1
```

#### Fixing Missing UART Configuration

```bash
# Check if UART device exists
ls -la /dev/ttyAMA0

# If missing on Pi 4:
sudo nano /boot/firmware/config.txt
# Add:
enable_uart=1
dtoverlay=disable-bt

# If missing on Pi 5, also add:
[pi5]
dtoverlay=uart0-pi5

# Disable Bluetooth services
sudo systemctl disable hciuart
sudo systemctl disable bluetooth

# Reboot
sudo reboot
```

#### cmdline.txt Serial Console Conflicts

The kernel command line (`/boot/firmware/cmdline.txt`) may have serial console enabled, which conflicts with flight controller communication.

**Check for conflicts:**
```bash
cat /boot/firmware/cmdline.txt
# Look for: console=serial0,115200 or console=ttyAMA0,115200
```

**Remove serial console (if present):**
```bash
# Backup first
sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.backup

# Edit and remove console=serial0,115200 and console=ttyAMA0,115200
sudo nano /boot/firmware/cmdline.txt

# Reboot
sudo reboot
```

#### Serial Getty Service Conflicts

The `serial-getty` service can lock the UART port, preventing flight controller communication.

**Check if serial-getty is running:**
```bash
# Pi 4
sudo systemctl status serial-getty@ttyAMA0.service

# Pi 5 (check both)
sudo systemctl status serial-getty@ttyAMA0.service
sudo systemctl status serial-getty@ttyAMA10.service
```

**Disable serial-getty:**
```bash
# Pi 4
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service

# Pi 5
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyAMA10.service
sudo systemctl disable serial-getty@ttyAMA10.service
```

#### Complete Configuration Check Script

Run this to verify all boot configuration is correct:

```bash
#!/bin/bash
echo "=== Checking /boot/firmware/config.txt ==="

# I2C
if grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt; then
    echo "[OK] I2C enabled"
else
    echo "[MISSING] dtparam=i2c_arm=on"
fi

# UART
if grep -q "^enable_uart=1" /boot/firmware/config.txt; then
    echo "[OK] UART enabled"
else
    echo "[MISSING] enable_uart=1"
fi

# Bluetooth disabled
if grep -q "^dtoverlay=disable-bt" /boot/firmware/config.txt; then
    echo "[OK] Bluetooth disabled for UART"
else
    echo "[MISSING] dtoverlay=disable-bt"
fi

# Pi 5 specific
if grep -q "Raspberry Pi 5" /proc/device-tree/model 2>/dev/null; then
    echo "Detected: Raspberry Pi 5"
    if grep -A5 "^\[pi5\]" /boot/firmware/config.txt | grep -q "dtoverlay=uart0-pi5"; then
        echo "[OK] Pi 5 UART overlay configured"
    else
        echo "[MISSING] [pi5] section with dtoverlay=uart0-pi5"
    fi
fi

echo ""
echo "=== Checking devices ==="
[ -e /dev/i2c-1 ] && echo "[OK] /dev/i2c-1 exists" || echo "[MISSING] /dev/i2c-1"
[ -e /dev/ttyAMA0 ] && echo "[OK] /dev/ttyAMA0 exists" || echo "[MISSING] /dev/ttyAMA0"

echo ""
echo "=== Checking cmdline.txt for serial console ==="
if grep -q "console=serial0\|console=ttyAMA0" /boot/firmware/cmdline.txt; then
    echo "[WARNING] Serial console enabled - may conflict with FC"
else
    echo "[OK] No serial console conflict"
fi
```

#### When to Rerun Setup Script

If your boot configuration is incorrect, the setup scripts will fix it automatically:

```bash
# Ubuntu 22.04
sudo ./scripts/setup_22_04.sh

# Ubuntu 24.04
sudo ./scripts/setup_24_04.sh

# Always reboot after running setup
sudo reboot
```

---

### I2C LiDAR Issues

#### Permission Denied: `/dev/i2c-1`

**Cause:** User not in i2c group.

**Solution:**
```bash
# Add user, pi for example, to i2c group
sudo usermod -a -G i2c $USER

# Verify
groups  # Should include i2c

# Log out and back in (or reboot)
logout
```

---

#### No Device at Address 0x08

**Check:**
```bash
i2cdetect -y 1
# Should show "08" in the grid
```

**Solutions:**
1. **Check power:**
   - Verify LiDAR has power (VCC, GND connected)
   - Check voltage (typically 3.3V or 5V)

2. **Check wiring:**
   - SDA → GPIO2 (Pin 3)
   - SCL → GPIO3 (Pin 5)
   - VCC → 3.3V or 5V
   - GND → Ground

3. **Check I2C enabled:**
   ```bash
   # Edit config
   sudo nano /boot/firmware/config.txt

   # Add or uncomment:
   dtparam=i2c_arm=on

   # Reboot
   sudo reboot
   ```

4. **Check I2C module:**
   ```bash
   lsmod | grep i2c
   # Should show i2c_dev

   # If not:
   sudo modprobe i2c-dev
   ```

---

#### LiDAR Node Crashes

**Check logs:**
```bash
pm2 logs lidar_reader --lines 50
```

**Common issues:**
- I2C device not found → See above
- Wrong address → Check `swarm_params.yaml` i2c_address
- Bus error → Check wiring, add pull-up resistors if needed

---

### Flight Controller UART Issues

#### Permission Denied: `/dev/ttyAMA0`

**Cause:** User not in dialout group.

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Verify
groups  # Should include dialout

# Log out and back in (or reboot)
logout
```

---

#### UART Device Not Found

**Check:**
```bash
ls -l /dev/ttyAMA0
# Should exist with dialout group
```

**Solutions:**
1. **Enable UART:**
   ```bash
   sudo nano /boot/firmware/config.txt

   # Add:
   enable_uart=1

   # Reboot
   sudo reboot
   ```

2. **Disable serial console:**
   ```bash
   # Backup first
   sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.backup

   # Edit
   sudo nano /boot/firmware/cmdline.txt

   # Remove:
   console=serial0,115200
   console=ttyAMA0,115200

   # Reboot
   sudo reboot
   ```

3. **Check alternative devices:**
   ```bash
   ls -l /dev/ttyUSB*
   ls -l /dev/ttyACM*

   # If using USB adapter, update swarm_params.yaml:
   # serial_port: "/dev/ttyUSB0"
   ```

---

#### Flight Controller Not Responding

**Check connection:**
```bash
# Verify device exists
ls -l /dev/ttyAMA0

# Check if in use
sudo lsof /dev/ttyAMA0
# Should show only fc_comms_node or nothing
```

**Check FC comms logs:**
```bash
pm2 logs fc_comms --lines 50
# Should show GPS, attitude, altitude data
```

**Solutions:**
1. **Restart FC physically**
2. **Check wiring:**
   - TX (FC) → RX (Pi)
   - RX (FC) → TX (Pi)
   - GND → GND
3. **Verify baud rate:**
   - Should be 115200 in both INAV and swarm_params.yaml
4. **Check MSP enabled in INAV**
5. **Disable serial conflicts:**
   ```bash
   sudo systemctl stop serial-getty@ttyAMA0
   sudo systemctl disable serial-getty@ttyAMA0
   ```

---

### GPS Issues

#### No GPS Fix

**Symptoms:**
- GPS position shows 0,0
- GPS satellite count is 0
- HDOP is very high

**Solutions:**
1. **Move to open sky location**
   - Away from buildings
   - Clear view of sky
   - No metal structures nearby

2. **Wait for GPS lock**
   - Can take 30-60 seconds from cold start
   - Check satellite count:
     ```bash
     ros2 topic echo /fc/gps_position
     ```

3. **Check GPS satellites:**
   ```bash
   # Monitor in real-time
   ros2 topic echo /fc/gps_speed_course
   ```

4. **Verify GPS connected to FC properly**

5. **Check min_gps_satellites parameter:**
   - Default is 5
   - May need to lower to 4 in poor conditions
   - See CONFIG_PARAMS_GUIDE.md

---

## Build & Installation Issues

### Permission Errors During Build

#### `Permission denied: 'log/build_XXXX'`

**Cause:** Workspace owned by root (ran setup with sudo).

**Solution:**
```bash
# Fix ownership
cd ~/swarm-ros
sudo chown -R $USER:$USER ~/swarm-ros

# Clean old builds
rm -rf log build install

# Rebuild
colcon build --symlink-install
```

---

### CMake or Compilation Errors

**Solutions:**
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
cd ~/swarm-ros
rm -rf build install log
colcon build --symlink-install
```

---

### Python Import Errors

#### System packages missing

**Solution:**
```bash
# Reinstall system Python dependencies
pip3 install --upgrade numpy scipy pyserial smbus2
```

#### AI packages missing

**Solution:**
```bash
# Reinstall in virtual environment
source ~/ai_flight_node_env/bin/activate
pip install --upgrade pip
pip install -r ~/swarm-ros/ai_model_requirements.txt
deactivate
```

---

### Virtual Environment Issues

#### Venv in wrong location (`/root/ai_flight_node_env`)

**Solution:**
```bash
# Remove incorrect venv
sudo rm -rf /root/ai_flight_node_env

# Create in correct location
python3 -m venv ~/ai_flight_node_env
~/ai_flight_node_env/bin/pip install --upgrade pip
~/ai_flight_node_env/bin/pip install -r ~/swarm-ros/ai_model_requirements.txt
```

---

## Network & Connectivity Issues

### Cannot Connect to Swarm_AP WiFi

**Check WiFi manager:**
```bash
# Check service status
sudo systemctl status wifi-manager.service

# Check hostapd (Access Point)
sudo systemctl status hostapd

# Check dnsmasq (DHCP server)
sudo systemctl status dnsmasq
```

**Solutions:**
```bash
# Restart WiFi manager
sudo systemctl restart wifi-manager.service

# Check logs
sudo journalctl -u wifi-manager.service -f

# Manually start AP mode
sudo systemctl start hostapd
sudo systemctl start dnsmasq
```

---

### WiFi Not Switching to Known Network

**Check configuration:**
```bash
# Verify networks file
cat /etc/wifi_networks.conf

# Should show:
# SSID:password
# MyNetwork:mypassword

# Edit if needed
sudo nano /etc/wifi_networks.conf
```

**Restart manager:**
```bash
sudo systemctl restart wifi-manager.service
```

**Manual scan:**
```bash
nmcli dev wifi list
```

---

### Ethernet Connection Not Working

**Check configuration:**
```bash
# Check eth0 IP
ip addr show eth0
# Should show: inet 192.168.10.1/24

# Check Netplan config
cat /etc/netplan/99-swarm-network.yaml

# Reapply if needed
sudo netplan apply
```

**Check laptop configuration:**
- Laptop must be on 192.168.10.x subnet (x ≠ 1)
- Example: 192.168.10.100

---

### Cannot SSH to 192.168.10.1

**Diagnostics:**
```bash
# From laptop, check connectivity
ping 192.168.10.1

# Check your IP
ip addr
# Should be 192.168.10.x

# Try verbose SSH
ssh -vvv pi@192.168.10.1
```

**Check SSH service on drone:**
```bash
sudo systemctl status ssh
sudo systemctl start ssh
```

---

## Runtime & Node Issues

### Node Crashes Immediately

**Check logs:**
```bash
pm2 logs <node_name> --lines 50
```

**Common issues by node:**

| Node | Common Issue | Solution |
|------|--------------|----------|
| lidar_reader | I2C device not found | Check `i2cdetect -y 1` |
| fc_comms | Serial permission denied | Add user to dialout group |
| ai_flight | Model file not found | Verify model path |
| fc_adapter | MSP timeout | Check FC connection |

---

### No Data on Topics

**Check node is running:**
```bash
ros2 node list
# Should show all nodes
```

**Monitor topic:**
```bash
ros2 topic echo /ai/action
# Press Ctrl+C to stop
```

**Check publishers:**
```bash
ros2 topic info /ai/action
# Shows number of publishers/subscribers
```

**Common causes:**
- Node crashed → Check PM2 logs
- Parameter misconfigured → Check swarm_params.yaml
- Hardware disconnected → Check hardware
- Network issues → Check ROS_DOMAIN_ID

---

### AI Model Not Loading

**Check model exists:**
```bash
ls -lh ~/swarm-ros/model/UID_3.zip
# Should exist and be readable
```

**Check virtual environment:**
```bash
ls -ld ~/ai_flight_node_env
# Should exist

~/ai_flight_node_env/bin/python --version
# Should show Python 3.x
```

**Test PyTorch:**
```bash
~/ai_flight_node_env/bin/python -c "import torch; print(torch.__version__)"
# Should print version without errors
```

**Reinstall packages:**
```bash
~/ai_flight_node_env/bin/pip install -r ~/swarm-ros/ai_model_requirements.txt
```
---

## Parameter & Configuration Issues

### Node Fails to Start

#### Invalid Parameter File Syntax

**Check YAML syntax:**
```bash
python3 -c "import yaml; yaml.safe_load(open('swarm_params.yaml'))"
```

**Common YAML errors:**
- Wrong indentation (use spaces, not tabs)
- Missing colons
- Unclosed brackets/quotes

---

#### Parameter Names Mismatch

**Check:**
- Must exactly match declared parameters in code
- Case-sensitive
- Underscores vs hyphens matter

**Verify loading:**
```bash
ros2 param list /node_name
ros2 param get /node_name parameter_name
```

---

### Parameters Not Applied

**Check launch file:**
```bash
# Verify parameters are loaded
ros2 param list /fc_adapter_node

# Check specific value
ros2 param get /fc_adapter_node max_velocity
```

**Ensure launch file loads config:**
- Check `parameters=[config]` in launch file
- Verify config file path is correct

---

### Invalid Parameter Values

**Check node logs:**
```bash
pm2 logs node_name
# or
ros2 run swarm_ai_integration node_name.py
```

Most nodes validate parameters and log warnings.

**Common issues:**
- Values out of range
- Wrong type (string vs number)
- Missing required parameter

**See:** CONFIG_PARAMS_GUIDE.md for valid ranges

---

## Performance Issues

### High CPU Usage

**Check stats:**
```bash
pm2 list
pm2 monit
top
```

**Common causes:**
1. **Control rates too high:**
   ```yaml
   control_rate_hz: 30  # Reduce from 40
   telemetry_rate: 20   # Reduce from 30
   prediction_rate: 8   # Reduce from 10
   ```

2. **Too many nodes logging:**
   - Disable debug logging
   - Reduce log rates

3. **AI inference slow:**
   - Check model on correct device (CPU vs GPU)
   - Reduce prediction_rate

4. **Serial communication issues:**
   - Check for errors in fc_comms logs

---

### High Memory Usage

**Check memory:**
```bash
free -h
pm2 monit
```

**Solutions:**
1. **Reduce buffer sizes:**
   ```yaml
   action_buffer_size: 10  # Reduce from 25
   log_buffer_size: 500    # Reduce from 1000
   ```

2. **Reduce log file sizes:**
   ```yaml
   max_log_file_size_mb: 50  # Reduce from 100
   ```

3. **Clear old logs:**
   ```bash
   pm2 flush
   rm ~/swarm-ros/flight-logs/*.csv
   ```

---

### Delayed Responses

**Check rates:**
```bash
# Topic publish rates
ros2 topic hz /ai/action
ros2 topic hz /fc/gps_position
```

**Solutions:**
1. **Increase rates (if CPU allows):**
   ```yaml
   control_rate_hz: 40-50
   telemetry_rate: 30-40
   prediction_rate: 10-15
   ```

2. **Check network latency:**
   ```bash
   # Check ROS_DOMAIN_ID
   echo $ROS_DOMAIN_ID

   # Ping between devices if distributed
   ping <other_device_ip>
   ```

3. **Reduce QoS depths:**
   ```yaml
   sensor_qos_depth: 1  # Use latest only
   ```

---

## Safety & Emergency Procedures

### Emergency Stop

**Immediate actions:**
1. **Kill throttle on RC transmitter**
   - Manual override always available
   - Disarm switch

2. **Stop PM2 processes:**
   ```bash
   pm2 stop all
   ```

3. **Kill ROS2:**
   ```bash
   pkill -9 ros
   pkill -9 python3
   ```

---

### Safety Override Triggered

**Check safety monitor:**
```bash
ros2 topic echo /fc_adapter/status
ros2 topic echo /safety/override
```

**Common triggers:**
- Altitude exceeded
- Distance from home exceeded
- Roll/pitch angle exceeded
- Battery too low
- GPS lost

**Check limits in swarm_params.yaml:**
```yaml
safety_monitor_node:
  ros__parameters:
    max_altitude: 10.0
    max_distance_from_home: 50.0
    max_roll_angle: 15.0
    max_pitch_angle: 15.0
```

---

### Lost GPS During Flight

**Automatic response:**
- Safety monitor activates
- RTH may engage (if configured)
- Hover mode activated

**Manual intervention:**
1. **Switch to manual mode**
2. **Land immediately**
3. **Don't rely on GPS navigation**

---

### Communication Lost with FC

**Symptoms:**
- No telemetry data
- FC not responding to commands

**Immediate actions:**
1. **Check FC powered**
2. **Manual RC takeover**
3. **Land safely**

**Debug after landing:**
```bash
# Check serial connection
ls -l /dev/ttyAMA0
sudo lsof /dev/ttyAMA0

# Check logs
pm2 logs fc_comms --lines 100
```

---

## Related Documentation

- [Setup Guide](SETUP_GUIDE.md) - Installation and configuration
- [Commands Guide](COMMANDS_GUIDE.md) - System operation commands
- [INAV Guide](INAV_GUIDE.md) - Flight controller parameters
- [Config Params Guide](CONFIG_PARAMS_GUIDE.md) - ROS2 parameters

---

## Getting Help

### Deep Dive Articles

For background context that may help with troubleshooting, check out our Substack series:
- [Chapter 1: Inside the Drone](https://substack.com/home/post/p-175604069) — Hardware components and drone anatomy
- [Chapter 2: The Wiring Brain](https://substack.com/home/post/p-176136139) — Wiring, connections, and power distribution
- [Chapter 3: From Data to Motion](https://substack.com/home/post/p-177453660) — Software architecture and data flow
- [Chapter 3.5: Additional Configurations](https://substack.com/home/post/p-180586067) — Advanced configuration and tuning

### Collecting Diagnostic Information

Before reporting issues, collect:

```bash
# System info
uname -a
cat /etc/os-release

# ROS2 info
echo $ROS_DISTRO
ros2 doctor

# Node status
ros2 node list
ros2 topic list

# PM2 status
pm2 list

# Hardware status
i2cdetect -y 1
ls -l /dev/ttyAMA0
groups

# Logs
pm2 logs --lines 200 > pm2_logs.txt
journalctl -u wifi-manager.service > wifi_logs.txt

# Configuration
cat ~/swarm-ros/src/swarm_ai_integration/config/swarm_params.yaml
```
