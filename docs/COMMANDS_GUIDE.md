# System Commands Guide

Quick reference for launching and managing the Swarm AI Integration system.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Launch Methods](#launch-methods)
3. [Individual Node Commands](#individual-node-commands)
4. [PM2 Management](#pm2-management)
5. [ROS2 Direct Commands](#ros2-direct-commands)
6. [Troubleshooting](#troubleshooting)

## Quick Start

### Launch Entire System

**Recommended:** Use the launch script for easy startup

```bash
cd ~/swarm-ros
./launch.sh
```

Or manually with PM2:

```bash
pm2 start "ros2 launch swarm_ai_integration swarm_ai_launch.py" --name swarm_ai
```

Or directly with ROS2:

```bash
cd ~/swarm-ros
source install/setup.bash
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

## Launch Methods

### Method 1: Launch Script (Recommended)

**Best for:** Production use, automatic startup, easy management

```bash
./launch.sh
```

**Features:**
- Starts all nodes automatically
- Uses PM2 for process management
- Auto-restart on failure
- Logs to files
- Background execution

---

### Method 2: ROS2 Launch File

**Best for:** Development, debugging, testing

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

**Features:**
- All nodes in one terminal
- Direct log output to console
- Easy to stop (Ctrl+C)
- Good for testing configuration changes

---

### Method 3: Individual Nodes with PM2

**Best for:** Testing specific components, custom configurations

See [Individual Node Commands](#individual-node-commands) below.


## Individual Node Commands

Launch specific nodes independently using PM2. Useful for testing, debugging, or custom configurations.

### 1. LiDAR Reader Node

**Purpose:** Reads distance data from LiDAR sensor via I2C

```bash
pm2 start "ros2 run swarm_ai_integration lidar_reader_node.py" --name lidar_reader
```

**Publishes to:**
- `/lidar/distance` - Distance measurements in meters

**Requirements:**
- I2C enabled and configured
- LiDAR connected to I2C bus 1 at address 0x08
- User in `i2c` group

**Test connection:**
```bash
i2cdetect -y 1
# Should show device at address 0x08
```

---

### 2. Flight Controller Communication Node

**Purpose:** Communicates with INAV flight controller via MSP serial protocol

```bash
pm2 start "ros2 run swarm_ai_integration fc_comms_node.py" --name fc_comms
```

**Publishes:**
- `/fc/gps_position` - GPS coordinates
- `/fc/gps_speed_course` - Speed and heading
- `/fc/attitude_euler` - Roll, pitch, yaw
- `/fc/altitude` - Barometric altitude and climb rate
- `/fc/battery_status` - Voltage and current

**Subscribes:**
- `/fc/rc_override` - RC channel overrides

**Requirements:**
- Serial port `/dev/ttyAMA0` available
- User in `dialout` group
- FC configured for MSP at 115200 baud

**Test connection:**
```bash
ls -l /dev/ttyAMA0
# Should be readable/writable
```

---

### 3. AI Adapter Node

**Purpose:** Transforms sensor data into observations for the AI model

```bash
pm2 start "ros2 run swarm_ai_integration ai_adapter_node.py" --name ai_adapter
```

**Subscribes:**
- `/lidar/distance`
- `/fc/gps_position`
- `/fc/attitude_euler`
- `/fc/altitude`

**Publishes:**
- `/ai/observation` - Formatted observation vector for AI

**Note:** This node acts as the bridge between raw sensor data and the AI model.

---

### 4. AI Flight Node

**Purpose:** Runs the trained AI model and generates flight commands

```bash
pm2 start "ros2 run swarm_ai_integration ai_flight_node.py --ros-args -p model_path:=/home/pi/swarm-ros/model/UID_3.zip" --name ai_flight
```

**Subscribes:**
- `/ai/observation` - State observations from sensors

**Publishes:**
- `/ai/action` - Flight commands (vx, vy, vz, speed)

**Parameters:**
- `model_path` - Path to trained model file (.zip)

**Requirements:**
- Model file exists at specified path
- Virtual environment configured (see setup.sh)
- PyTorch and stable-baselines3 installed

**Common model paths:**
```bash
# Development model
-p model_path:=~/swarm-ros/model/UID_3.zip

# Production model
-p model_path:=/opt/swarm/model/production.zip
```

---

### 5. Flight Controller Adapter Node

**Purpose:** Converts AI velocity commands to RC values and sends to FC via MSP

```bash
pm2 start "ros2 run swarm_ai_integration fc_adapter_node.py" --name fc_adapter
```

**Subscribes:**
- `/ai/action` - Velocity commands from AI
- `/fc/gps_speed_course` - Current velocity
- `/fc/attitude_euler` - Current orientation
- `/fc/altitude` - Current altitude and climb rate
- `/safety/override` - Safety override signal

**Publishes:**
- `/fc_adapter/status` - Node status
- `/fc_adapter/velocity_error` - Control error metrics
- `/fc/rc_override` - RC channel values

**Features:**
- Closed-loop velocity control with PID
- Direct MSP communication
- Pre-arm sequence (30 seconds)
- Warmup phase (40 frames)
- Throttle safety (1000 during arming)
- Rate limiting on throttle changes

**Important:** This node handles arming and RC override. Ensure parameters are correct.

---

### 6. Black Box Recorder Node

**Purpose:** Records all flight data to CSV files for analysis

```bash
pm2 start "ros2 run swarm_ai_integration black_box_recorder_node.py --ros-args -p log_directory:=~/swarm-ros/flight-logs" --name black_box
```

**Subscribes:**
- `/ai/observation`
- `/ai/action`
- `/fc/gps_position`
- `/fc/attitude_euler`
- `/fc/battery_status`
- `/lidar/distance`

**Parameters:**
- `log_directory` - Directory for flight logs

**Output:**
- CSV files with timestamp: `flight_log_YYYYMMDD_HHMMSS.csv`

**Log directory:**
```bash
# Default
-p log_directory:=~/swarm-ros/flight-logs

# Custom
-p log_directory:=/mnt/usb/logs
```

**View logs:**
```bash
ls -lh ~/swarm-ros/flight-logs/
tail -f ~/swarm-ros/flight-logs/flight_log_*.csv
```

## PM2 Management

### View Running Processes

```bash
pm2 list
```

**Output shows:**
- Process name
- Status (online/stopped/errored)
- CPU and memory usage
- Uptime
- Restart count

---

### View Logs

**All logs:**
```bash
pm2 logs
```

**Specific node:**
```bash
pm2 logs swarm_ai
pm2 logs lidar_reader
pm2 logs fc_comms
pm2 logs ai_adapter
pm2 logs ai_flight
pm2 logs fc_adapter
pm2 logs black_box
```

**Filter last N lines:**
```bash
pm2 logs --lines 100
```

**Clear logs:**
```bash
pm2 flush
```

---

### Process Control

**Stop all:**
```bash
pm2 stop all
```

**Stop specific node:**
```bash
pm2 stop swarm_ai
pm2 stop fc_adapter
```

**Restart all:**
```bash
pm2 restart all
```

**Restart specific node:**
```bash
pm2 restart fc_comms
```

**Delete process (removes from PM2):**
```bash
pm2 delete swarm_ai
pm2 delete all
```

---

### Monitoring

**Real-time monitoring:**
```bash
pm2 monit
```

**Process information:**
```bash
pm2 show swarm_ai
```

**Save process list (for auto-start on boot):**
```bash
pm2 save
```

**Setup auto-start on boot:**
```bash
pm2 startup
# Follow the instructions shown
```

---

## ROS2 Direct Commands

### Run Individual Nodes (Without PM2)

Useful for debugging with direct console output.

**LiDAR Reader:**
```bash
ros2 run swarm_ai_integration lidar_reader_node.py
```

**FC Communications:**
```bash
ros2 run swarm_ai_integration fc_comms_node.py
```

**AI Adapter:**
```bash
ros2 run swarm_ai_integration ai_adapter_node.py
```

**AI Flight:**
```bash
ros2 run swarm_ai_integration ai_flight_node.py --ros-args -p model_path:=~/swarm-ros/model/UID_3.zip
```

**FC Adapter:**
```bash
ros2 run swarm_ai_integration fc_adapter_node.py
```

**Black Box Recorder:**
```bash
ros2 run swarm_ai_integration black_box_recorder_node.py --ros-args -p log_directory:=~/swarm-ros/flight-logs
```

---

### ROS2 Diagnostic Commands

**List active nodes:**
```bash
ros2 node list
```

**List all topics:**
```bash
ros2 topic list
```

**Monitor specific topic:**
```bash
ros2 topic echo /ai/action
ros2 topic echo /fc/gps_position
ros2 topic echo /lidar/distance
```

**Check topic frequency:**
```bash
ros2 topic hz /ai/action
ros2 topic hz /fc/gps_position
```

**Show topic info:**
```bash
ros2 topic info /ai/action
```

**Node information:**
```bash
ros2 node info /lidar_reader_node
ros2 node info /fc_comms_node
ros2 node info /fc_adapter_velocity_node
```

**Check parameters:**
```bash
ros2 param list /fc_adapter_velocity_node
ros2 param get /fc_adapter_velocity_node control_rate_hz
```

**Set parameters at runtime:**
```bash
ros2 param set /fc_adapter_velocity_node kp_xy 25.0
```

## Troubleshooting

For comprehensive troubleshooting information, see the [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md).

**Quick diagnostics:**
```bash
# Check system status
pm2 list
ros2 node list

# Check logs
pm2 logs

# Hardware checks
i2cdetect -y 1        # LiDAR
ls -l /dev/ttyAMA0    # FC serial
```

**Common issues:**
- **Node won't start** → Check PM2 logs: `pm2 logs <node_name>`
- **No data on topics** → Verify node is running: `ros2 node list`
- **Permission errors** → Add user to groups: `sudo usermod -a -G i2c,dialout $USER`
- **High CPU** → Reduce control rates in swarm_params.yaml

See [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md) for detailed solutions.

## Quick Command Reference

| Task | Command |
|------|---------|
| **Start system** | `./launch.sh` or `pm2 start ...` |
| **Stop all** | `pm2 stop all` |
| **Restart all** | `pm2 restart all` |
| **View logs** | `pm2 logs` |
| **List processes** | `pm2 list` |
| **Monitor** | `pm2 monit` |
| **ROS2 launch** | `ros2 launch swarm_ai_integration swarm_ai_launch.py` |
| **List nodes** | `ros2 node list` |
| **List topics** | `ros2 topic list` |
| **Echo topic** | `ros2 topic echo /topic/name` |
| **Node info** | `ros2 node info /node_name` |
| **Check I2C** | `i2cdetect -y 1` |
| **Check serial** | `ls -l /dev/ttyAMA0` |



