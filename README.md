# Swarm ROS2 Integration

**Version:** 1.0.0
**Status:** Active Development
**Last Updated:** October 2024

ROS2 integration package for real-world deployment of Swarm AI flight control system with INAV 7 flight controller. This package enables autonomous flight using trained PPO models with real sensor data from I2C LiDAR, GPS, IMU, and MSP telemetry.

---

## 🚀 Quick Start

```bash
# 1. Clone repository
cd ~
git clone <your-repo-url> swarm-ros
cd swarm-ros

# 2. Run automated setup (Ubuntu 22.04)
sudo ./setup.sh --install-pm2

# 3. Log out and back in for group changes
logout

# 4. Verify installation
./verify_setup.sh

# 5. Launch system
./launch.sh  # or: ros2 launch swarm_ai_integration swarm_ai_launch.py
```

See [SETUP_GUIDE.md](SETUP_GUIDE.md) for detailed installation instructions.

---

## 🎯 Project Status

### Current Focus
**Autonomous mission execution with intelligent return-to-home and precision landing**

### Recent Updates (v1.0.0)
- ✅ **Custom RTH System**: Intelligent waypoint-based return to home (replaces INAV RTH)
- ✅ **LiDAR-Guided Landing**: Two-phase landing with GPS approach + LiDAR precision descent
- ✅ **Mission Completion Detection**: Automatic RTH activation when waypoints are completed
- ✅ **Automated Setup**: One-command installation for Ubuntu 22.04 servers
- ✅ **Hardware Verification**: Comprehensive I2C and UART connectivity checks
- ✅ **Coordinate System Enhancement**: Home position at [0,0,0] with ground-level targeting
- ✅ Implemented direct MSP serial communication with INAV 7
- ✅ Added pre-arm streaming protocol (30s arming sequence)
- ✅ Integrated PID velocity controller (vx, vy, vz → RC channels)
- ✅ Waypoint-based goal tracking with ENU coordinate conversion
- ✅ Relative positioning system with configurable initial offset
- ✅ Black box flight data recorder
- ✅ I2C-based LiDAR sensor support (front & down)

---

## 🏗️ Architecture Overview

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│  AI Flight Node │ ───> │  AI Adapter Node │ <─── │  FC Comms Node  │
│  (PPO Policy)   │      │  (131-D Obs)     │      │  (MSP Protocol) │
└─────────────────┘      └──────────────────┘      └─────────────────┘
         │                        │                          │
         │ /ai/action             │ /ai/observation          │ /fc/* topics
         ↓                        ↓                          ↓
┌─────────────────────────────────────────────────────────────────────┐
│                   Safety Monitor Node (NEW!)                        │
│  • Monitors: altitude, velocity, battery, distance, obstacles       │
│  • Mission Tracking: Detects waypoint completion                    │
│  • Custom RTH: Publishes home as goal waypoint                      │
│  • Landing: Two-phase GPS+LiDAR precision descent                   │
└─────────────────────────────────────────────────────────────────────┘
         │                                                     │
         │ /safety/custom_goal_geodetic                       │
         ↓                                                     ↓
┌─────────────────────────────────────────────────────────────────────┐
│                        FC Adapter Node                               │
│            (Velocity PID → MSP RC Commands)                          │
└─────────────────────────────────────────────────────────────────────┘
         │
         │ MSP_SET_RAW_RC (Direct Serial)
         ↓
┌──────────────────┐
│  INAV 7 FC       │
│  (Hardware)      │
└──────────────────┘
```

---

## 🎮 Core Features

### 🏠 Custom Return to Home (RTH)
**Intelligent mission completion and emergency RTH system**

**Key Features:**
- **Waypoint Mission Tracking**: Monitors INAV waypoint progress via `/fc/waypoint`
- **Automatic RTH Activation**: Triggers when mission completes or critical safety violations occur
- **Custom Goal Waypoints**: Publishes home position as AI navigation goal (bypasses INAV RTH)
- **Home Position Storage**: Maintains both ENU [0,0,0] and geodetic coordinates

**Activation Triggers:**
- Mission waypoint completion (waypoint #0 detected)
- Battery critically low (< 14V)
- Distance from home exceeded (> 100m)
- Communication timeout (> 2s)

**RTH Sequence:**
1. Safety monitor detects trigger condition
2. Publishes home position to `/safety/custom_goal_geodetic` [lat, lon, alt]
3. AI adapter overrides mission waypoint with home goal
4. AI navigates drone back to home position
5. Landing sequence activates when within 2m horizontally

### 🛬 LiDAR-Guided Precision Landing
**Two-phase landing system for safe ground touchdown**

**Phase 1: GPS-Guided Approach (Above 5m)**
- Uses GPS altitude for navigation
- Descends to ~3m above ground level
- Initiates when within 2m horizontal distance of home

**Phase 2: LiDAR-Guided Final Descent (Below 5m)**
- Switches to down-facing LiDAR when altitude < 5m
- Targets 0.5m above ground using precise LiDAR measurements
- Landing complete when LiDAR reads < 0.3m
- Prevents GPS altitude errors from causing hard landings

**Status Indicators:**
- `🛬 LANDING_GPS`: GPS-guided approach phase
- `🛬 LANDING_LIDAR`: LiDAR-guided final descent
- `✅ LANDING_COMPLETE`: Successfully landed

**Why LiDAR?**
- GPS altitude error: ±5-10m
- LiDAR altitude error: ±0.05m
- Handles terrain variations and prevents crashes

---

## 🧩 Core Components

### 1. AI Adapter Node (`ai_adapter_node.py`)
**Purpose:** Converts real sensor data into the 131-dimensional observation array for the PPO model

**Key Features:**
- **Relative ENU Positioning:** Origin at [0, 0, 3]m (3m initial altitude)
- **Home Position:** Ground level at [0, 0, 0]m for RTH targeting
- **Waypoint Goal Tracking:** Converts geodetic waypoints to local ENU offsets
- **Custom Goal Override**: Supports `/safety/custom_goal_geodetic` for RTH (NEW!)
- **Action Buffer:** Maintains 25-step action history for temporal awareness
- **GPS Quality Validation**: Requires 6+ satellites and HDOP < 4.0
- **Tiered Averaging**: 30-sample GPS averaging for stable origin

**Observation Structure (131-D):**
- [0:3] Relative position (E, N, U) in meters
- [3:6] Orientation Euler angles (roll, pitch, yaw) in radians
- [6:9] Velocity ENU (from GPS speed/course)
- [9:12] Angular velocity (from IMU)
- [12:112] Action buffer (25 steps × 4 actions)
- [112:128] LiDAR distances (16 rays, normalized 0-1)
- [128:131] Goal vector (ENU, normalized by 10m)

**Subscribed Topics:**
- `/fc/gps_fix` (sensor_msgs/NavSatFix) - GPS position
- `/fc/gps_satellites` (std_msgs/Int32) - Satellite count
- `/fc/gps_hdop` (std_msgs/Float32) - Horizontal dilution of precision
- `/fc/attitude_euler` (geometry_msgs/Vector3Stamped) - Euler angles [rad]
- `/fc/imu_raw` (sensor_msgs/Imu) - Angular velocity
- `/fc/gps_speed_course` (std_msgs/Float32MultiArray) - [speed_mps, course_deg]
- `/fc/waypoint` (std_msgs/Float32MultiArray) - Current mission waypoint
- `/safety/custom_goal_geodetic` (std_msgs/Float32MultiArray) - Custom RTH goal (NEW!)
- `/lidar_distance` (sensor_msgs/Range) - Down-facing LiDAR
- `/ai/action` (std_msgs/Float32MultiArray) - AI commands for action buffer

**Published Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - 131-D observation @ 30 Hz
- `/ai/observation_debug` (std_msgs/Float32MultiArray) - [E, N, U, yaw, down_lidar, action_flag]

**Integration Note:** To enable custom RTH, add subscription to `/safety/custom_goal_geodetic` and override goal when received.

---

### 2. AI Flight Node (`ai_flight_node.py`)
**Purpose:** Executes PPO policy inference for autonomous flight control

**Key Features:**
- **Secure Model Loading:** `weights_only=True` PyTorch loading
- **Safe Metadata:** Reads architecture from `safe_policy_meta.json`
- **Model Path:** Configurable in `swarm_params.yaml`
- **Inference Rate:** 10 Hz prediction cycle
- **Deterministic Actions:** `predict(obs, deterministic=True)`
- **NaN Safety:** Auto-replaces NaN/Inf with zeros

**Model Format:**
- Input: 131-D observation (float32)
- Output: [vx_east, vy_north, vz_up, speed] (4-D action in ENU frame)

**Subscribed Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - 131-D from adapter

**Published Topics:**
- `/ai/action` (std_msgs/Float32MultiArray) - [vx, vy, vz, speed] @ 10 Hz

---

### 3. Safety Monitor Node (`safety_monitor_node.py`) 🆕
**Purpose:** Comprehensive safety monitoring with intelligent RTH and precision landing

**Safety Checks @ 10 Hz:**
- **Altitude Limits:** Min/max bounds (default: 0-10m)
- **Velocity Limits:** Max speed threshold (default: 1 m/s)
- **Battery Monitoring:** Low voltage detection (default: <14V)
- **Geofencing:** Distance from home (default: 100m radius)
- **Obstacle Proximity:** LiDAR collision warning (default: <1m)
- **Communication Timeout:** Lost telemetry detection (default: 2s)
- **Mission Progress:** Waypoint completion tracking (NEW!)

**New Features:**
- 🏠 **Home Position Tracking**: Stores both ENU [0,0,0] and geodetic coordinates
- 📍 **Waypoint Mission Monitoring**: Tracks `/fc/waypoint` for completion detection
- 🎯 **Custom RTH Publisher**: Publishes home to `/safety/custom_goal_geodetic`
- 🛬 **Two-Phase Landing**: GPS approach + LiDAR final descent
- 📊 **Dual Altitude Display**: Shows both GPS and LiDAR altitude in status

**Failsafe Actions:**
- **Hover Override:** `/safety/override` → FC Adapter sends neutral RC
- **Custom RTH:** `/safety/custom_goal_geodetic` → Home position as AI goal (NEW!)
- **INAV RTH (Backup):** `/safety/rth_command` → CH9 = 1800 (legacy mode)

**Critical Violations Triggering RTH:**
- Low battery voltage
- Excessive distance from home
- Communication loss
- Mission waypoint completion (NEW!)

**Landing Logic:**
1. Detect drone within 2m of home horizontally
2. **Phase 1 (GPS)**: Descend to 3m using GPS altitude
3. **Phase 2 (LiDAR)**: Switch to LiDAR when reading < 5m
4. Target 0.5m above ground with LiDAR feedback
5. Complete landing when LiDAR < 0.3m

**Subscribed Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - Position, velocity, LiDAR
- `/ai/observation_debug` (std_msgs/Float32MultiArray) - Detailed state
- `/ai/action` (std_msgs/Float32MultiArray) - AI command monitoring
- `/fc/battery` (sensor_msgs/BatteryState) - Battery voltage
- `/fc/gps_fix` (sensor_msgs/NavSatFix) - GPS for home geodetic storage (NEW!)
- `/fc/waypoint` (std_msgs/Float32MultiArray) - Mission progress tracking (NEW!)

**Published Topics:**
- `/safety/override` (std_msgs/Bool) - Hover mode trigger
- `/safety/rth_command` (std_msgs/Bool) - INAV RTH (backup)
- `/safety/custom_goal_geodetic` (std_msgs/Float32MultiArray) - Custom RTH goal [lat, lon, alt] (NEW!)
- `/safety/status` (std_msgs/String) - Comprehensive status with GPS_ALT and LIDAR_ALT (NEW!)

**Status Message Format:**
```
🚨 CUSTOM_RTH_ACTIVE | 🛬 LANDING_LIDAR | GPS_ALT:2.1m | LIDAR_ALT:1.8m |
VEL:0.3m/s | BAT:14.2V | OBS:1.8m | HOME_DIST:0.5m | WP:COMPLETED
```

---

### 4. FC Communications Node (`fc_comms_node.py`)
**Purpose:** MSP protocol interface to INAV 7 flight controller

**Key Features:**
- **Threaded Serial I/O:** Non-blocking read/write with command queue
- **Auto-Reconnection:** 5-second retry interval on connection loss
- **Telemetry Polling @ 10 Hz:** Rotates through MSP commands
- **Waypoint Polling:**
  - `first`: Poll WP #1 @ 10 Hz (current mission waypoint)
  - `all`: Cycle WP #0 to #N @ 10 Hz
- **Derived Topics:** GPS speed/course, MSP status

**MSP Commands:**
- **Outgoing:** MSP_SET_RAW_RC, MSP_WP
- **Incoming:** MSP_RAW_IMU, MSP_RAW_GPS, MSP_ATTITUDE, MSP_STATUS, MSP_ANALOG, MSP_MOTOR, MSP_WP

**Published Topics:**
- `/fc/imu_raw` (sensor_msgs/Imu) - IMU data
- `/fc/gps_fix` (sensor_msgs/NavSatFix) - GPS position
- `/fc/gps_satellites` (std_msgs/Int32) - Satellite count (NEW!)
- `/fc/gps_hdop` (std_msgs/Float32) - HDOP value (NEW!)
- `/fc/attitude_euler` (geometry_msgs/Vector3Stamped) - [roll, pitch, yaw] rad
- `/fc/status` (std_msgs/String) - Human-readable FC status
- `/fc/battery` (sensor_msgs/BatteryState) - Battery voltage & current
- `/fc/motor_rpm` (std_msgs/Float32MultiArray) - Motor values
- `/fc/gps_speed_course` (std_msgs/Float32MultiArray) - [speed_mps, course_deg]
- `/fc/waypoint` (std_msgs/Float32MultiArray) - [wp_no, lat, lon, alt_m, heading, stay, navflag]

---

### 5. FC Adapter Node (`fc_adapter_node.py`)
**Purpose:** Closed-loop velocity control via PID → MSP RC commands

**Control Flow:**
1. **Pre-Arm Streaming (30s):** Sends ARM frame @ 40 Hz
2. **Warm-Up (40 frames):** Sends neutral RC to stabilize
3. **Closed-Loop Control @ 40 Hz:**
   - Reads `/ai/action` [vx, vy, vz, speed] in ENU frame
   - Converts GPS speed/course → ENU velocity
   - Transforms ENU → body frame using yaw
   - PID computes error → RC deviations
   - Sends MSP_SET_RAW_RC via direct serial
4. **Safety Modes:**
   - Timeout Hover: No action for >1s
   - Safety Override: `/safety/override` = True
   - RTH Mode: `/safety/rth_command` = True → CH9 = 1800

**PID Configuration:**
- **XY (Roll/Pitch):** kp=150, ki=10, kd=20
- **Z (Throttle):** kp=100, ki=5, kd=15
- **Output:** RC deviations ±400 (1100-1900 range)

**RC Channel Mapping:**
- CH1-4: Roll, Pitch, Throttle, Yaw
- CH5 (ARM): 1800
- CH6 (ANGLE): 1800
- CH7 (ALT HOLD): 1800
- CH8 (MSP OVERRIDE): 1800 (MUST be high)
- CH9 (NAV RTH): 1800 if RTH active

**Subscribed Topics:**
- `/ai/action` (std_msgs/Float32MultiArray)
- `/fc/gps_speed_course`, `/fc/attitude_euler`, `/fc/altitude`
- `/safety/override`, `/safety/rth_command`

**Published Topics:**
- `/fc_adapter/status` (std_msgs/String)
- `/fc_adapter/velocity_error` (geometry_msgs/Vector3Stamped)

---

### 6. LiDAR Reader Node (`lidar_reader_node.py`)
**Purpose:** I2C-based LiDAR sensor interface for ranging

**Key Features:**
- **I2C Protocol:** Reads 4-byte distance from register 0x24
- **Multi-Sensor Support:** Namespaced for down/front sensors
- **Moving Average Filter:** 5-sample window for noise reduction
- **Range Validation:** 0.05m - 50m bounds
- **Auto-Reconnect:** 5s retry on failure
- **Publishing Rate:** Up to 100 Hz

**I2C Configuration:**
- **Bus:** `/dev/i2c-1`
- **Down Sensor:** Address 0x08 (default)
- **Register:** 0x24 (distance in mm)

**Published Topics (per instance):**
- `lidar_distance` (sensor_msgs/Range) - Filtered measurement
- `lidar_status` (std_msgs/String) - Health status

---

### 7. Black Box Recorder Node (`black_box_recorder_node.py`)
**Purpose:** Flight data logging for analysis

**Key Features:**
- **Comprehensive Logging:** All critical topics
- **Timestamped CSV:** Nanosecond precision
- **Auto-Rotation:** 100MB file size limit
- **Buffered Writes:** 5s flush interval

**Log Directory:** `~/swarm-ros/flight-logs/`

**Logged Topics:**
- AI: `/ai/observation`, `/ai/action`, `/ai/observation_debug`
- FC: `/fc/gps_fix`, `/fc/attitude_euler`, `/fc/imu_raw`, `/fc/battery`
- Safety: `/safety/status`, `/safety/override`, `/safety/rth_command`

---

## 📦 Installation

### Automated Setup (Recommended)

**Ubuntu 22.04 Server:**

```bash
# 1. Clone repository
cd ~
git clone <your-repo-url> swarm-ros
cd swarm-ros

# 2. Run automated setup
sudo ./setup.sh --install-pm2

# 3. Log out and back in for group changes
logout

# 4. Verify installation
./verify_setup.sh
```

**What `setup.sh` Does:**
- ✅ Installs ROS2 Humble
- ✅ Installs Python dependencies (PyTorch, stable-baselines3, smbus2, pyserial)
- ✅ Configures I2C for LiDAR (module loading, permissions, udev rules)
- ✅ Configures UART for flight controller (serial port, group permissions)
- ✅ Checks hardware connectivity (scans I2C @ 0x08, verifies /dev/ttyAMA0)
- ✅ Builds ROS2 workspace with colcon
- ✅ Configures environment (.bashrc setup)
- ✅ Optionally installs PM2 for process management

**Setup Options:**
```bash
sudo ./setup.sh                      # Full installation
sudo ./setup.sh --skip-ros           # Skip ROS2 if already installed
sudo ./setup.sh --skip-hardware-check # Skip hardware verification
sudo ./setup.sh --install-pm2        # Include PM2 for auto-start
```

See [SETUP_GUIDE.md](SETUP_GUIDE.md) for detailed manual installation steps.

---

### Prerequisites

**Hardware:**
- Onboard computer: Raspberry Pi 4 or Ubuntu 22.04 server
- Flight controller: INAV 7 compatible (connected via UART)
- LiDAR: I2C-based distance sensor (address 0x08)
- GPS: Connected to flight controller
- IMU: Integrated in flight controller

**Hardware Connections:**
| Component | Interface | Device Path | Config |
|-----------|-----------|-------------|--------|
| Flight Controller | UART | `/dev/ttyAMA0` | 115200 baud |
| LiDAR (Down) | I2C Bus 1 | Address `0x08` | 100Hz |
| GPS | via FC | MSP protocol | - |
| IMU | via FC | MSP protocol | - |

**Software:**
- OS: Ubuntu 22.04 LTS
- ROS2: Humble
- Python: 3.10+

---

### Manual Installation

See [SETUP_GUIDE.md](SETUP_GUIDE.md) for step-by-step manual installation instructions including:
- ROS2 Humble installation
- System dependency installation
- Python package installation
- I2C configuration for LiDAR
- UART configuration for flight controller
- ROS2 workspace build
- Environment configuration

---

## 🚀 Usage

### Hardware Verification

After installation, verify hardware connections:

```bash
# Check I2C LiDAR
i2cdetect -y 1  # Should show device at 0x08

# Check UART flight controller
ls -l /dev/ttyAMA0  # Should be readable/writable

# Run full verification
./verify_setup.sh
```

---

### System Launch

**Using PM2 (Recommended for Production):**

```bash
# Start system
./launch.sh

# Monitor status
pm2 list
pm2 logs swarm-ros-launched

# Stop system
pm2 stop swarm-ros-launched
```

**Direct ROS2 Launch:**

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/swarm-ros/install/setup.bash

# Launch all nodes
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

**Testing Individual Nodes:**

```bash
# Test LiDAR
ros2 run swarm_ai_integration lidar_reader_node

# Test FC comms
ros2 run swarm_ai_integration fc_comms_node

# Test safety monitor
ros2 run swarm_ai_integration safety_monitor_node
```

---

### Monitoring

**Safety Status:**
```bash
ros2 topic echo /safety/status
```

**Sample output:**
```
🚨 CUSTOM_RTH_ACTIVE | 🛬 LANDING_LIDAR | GPS_ALT:2.1m | LIDAR_ALT:1.8m |
VEL:0.3m/s | BAT:14.2V | OBS:1.8m | HOME_DIST:0.5m | WP:COMPLETED
```

**AI Observation:**
```bash
ros2 topic echo /ai/observation_debug
```

**Flight Controller Status:**
```bash
ros2 topic echo /fc/status
```

**View All Topics:**
```bash
ros2 topic list
ros2 topic hz /ai/observation  # Check rate
```

---

## ⚙️ Configuration

### Main Configuration File

**Location:** `src/swarm_ai_integration/config/swarm_params.yaml`

**Key Parameters:**

```yaml
ai_adapter_node:
  ros__parameters:
    max_ray_distance: 20.0           # LiDAR normalization (meters)
    relative_start_enu: [0.0, 0.0, 3.0]  # Initial position [E, N, U]
    min_gps_satellites: 6            # Minimum for GPS lock

ai_flight_node:
  ros__parameters:
    model_path: "/home/pi/swarm-ros/model/UID_203.zip"
    prediction_rate: 10.0            # AI inference rate (Hz)

safety_monitor_node:
  ros__parameters:
    max_altitude: 10.0               # Maximum flight altitude (m)
    min_altitude: 0.0                # Minimum altitude (m)
    max_velocity: 1.0                # Speed limit (m/s)
    min_battery_voltage: 14.0        # Low battery threshold (V)
    max_distance_from_home: 100.0    # Geofence radius (m)
    obstacle_danger_distance: 1.0    # Collision warning (m)

fc_adapter_node:
  ros__parameters:
    kp_xy: 150.0                     # Roll/Pitch P gain
    ki_xy: 10.0                      # Roll/Pitch I gain
    kd_xy: 20.0                      # Roll/Pitch D gain
    kp_z: 100.0                      # Throttle P gain
    ki_z: 5.0                        # Throttle I gain
    kd_z: 15.0                       # Throttle D gain
    prearm_duration_sec: 30.0        # Arming sequence time
    startup_delay_sec: 20.0          # Operator setup window

fc_comms_node:
  ros__parameters:
    serial_port: "/dev/ttyAMA0"      # UART device
    baud_rate: 115200                # Serial baud rate
    waypoint_poll_mode: "first"      # WP polling: first/all/none

down_lidar/lidar_reader_down:
  ros__parameters:
    i2c_bus: 1                       # I2C bus number
    i2c_address: 0x08                # LiDAR I2C address
    publish_rate: 100.0              # Sensor rate (Hz)
```

---

### INAV Flight Controller Setup

**Required INAV Configuration:**

1. **Enable MSP RC Override:**
   - Configure CH8 as MSP RC Override mode (range 1700-2100)

2. **Set Flight Modes:**
   - CH5: ARM switch (1700-2100)
   - CH6: ANGLE mode
   - CH7: ALT HOLD mode (or POS HOLD)
   - CH9: NAV RTH (backup RTH mode)

3. **GPS Settings:**
   - Enable GPS and wait for fix
   - Configure RTH altitude and behavior (backup)

4. **Serial Port:**
   - Enable MSP on UART connected to Pi
   - Set baud rate to 115200

5. **Arming:**
   - Configure prearm checks
   - Set switch on CH5 (range 1700-2100)

---

## 🛡️ Safety Features

### Comprehensive Safety System

1. **Altitude Limits**: Prevents flying too high or too low
2. **Velocity Limits**: Limits maximum speed
3. **Geofencing**: Prevents flying too far from home
4. **Obstacle Avoidance**: Monitors LiDAR for collision warning
5. **Battery Monitoring**: Triggers RTH on low battery
6. **Communication Timeout**: Activates failsafe on lost telemetry
7. **Mission Monitoring**: Tracks waypoint completion (NEW!)
8. **Custom RTH**: Intelligent return to home navigation (NEW!)
9. **Precision Landing**: LiDAR-guided ground touchdown (NEW!)

### Safety Sequence

**Normal Operation:**
```
Mission Waypoint → AI Navigation → Velocity Control → Flight
```

**Safety Violation Detected:**
```
Violation → Hover Override → Safety Monitor Active
```

**Critical Violation (RTH Required):**
```
Critical Violation → Custom RTH Activated →
Navigate to Home → GPS Approach → LiDAR Landing → Complete
```

**Landing Sequence:**
```
Phase 1 (GPS): Descend to 3m using GPS altitude
Phase 2 (LiDAR): Switch to LiDAR < 5m, target 0.5m
Complete: LiDAR reads < 0.3m, motors stop
```

---

## 🔧 Troubleshooting

### Hardware Issues

**I2C LiDAR Not Detected:**
```bash
# Check I2C is enabled
ls -l /dev/i2c-1

# Scan for devices
i2cdetect -y 1  # Should show 08

# Check permissions
groups  # Should include 'i2c'

# Fix: Re-run setup
sudo ./setup.sh
```

**UART Flight Controller Not Accessible:**
```bash
# Check device exists
ls -l /dev/ttyAMA0

# Check permissions
groups  # Should include 'dialout'

# Fix: Add user to group
sudo usermod -a -G dialout $USER
logout  # Must log out and back in
```

---

### Software Issues

**ROS2 Command Not Found:**
```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/swarm-ros/install/setup.bash

# Add to .bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/swarm-ros/install/setup.bash" >> ~/.bashrc
```

**Build Errors:**
```bash
# Clean and rebuild
cd ~/swarm-ros
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

**GPS Not Getting Fix:**
- Move to open sky location
- Wait 30-60 seconds for satellite acquisition
- Check `/fc/gps_satellites` topic (need 6+)
- Check `/fc/gps_hdop` topic (should be < 4.0)

---

### Runtime Issues

**Custom RTH Not Working:**

Check AI adapter subscribes to `/safety/custom_goal_geodetic` and overrides mission waypoint when custom goal is published.

**Landing Not Using LiDAR:**

Check:
- LiDAR is publishing to `/lidar_distance`
- Safety monitor receives LiDAR data in observation
- Status shows "LIDAR_ALT" values
- LiDAR reading < 5m to trigger phase 2

**Waypoint Mission Not Completing:**

Check:
- `/fc/waypoint` topic shows waypoint #0 when complete
- Safety monitor logs "Waypoint mission completed"
- Custom RTH activates after completion

---

## 📊 Monitoring and Debugging

### Debug Commands

```bash
# Check topic rates
ros2 topic hz /ai/observation          # Should be ~30 Hz
ros2 topic hz /ai/action              # Should be ~10 Hz
ros2 topic hz /fc/gps_fix             # Should be ~1-10 Hz

# Monitor safety
ros2 topic echo /safety/status

# Check GPS quality
ros2 topic echo /fc/gps_satellites    # Need 6+
ros2 topic echo /fc/gps_hdop          # Should be < 4.0

# View observation
ros2 topic echo /ai/observation_debug

# Check LiDAR
ros2 topic echo /lidar_distance

# View waypoints
ros2 topic echo /fc/waypoint
```

### Log Files

**Flight Logs:**
```bash
cd ~/swarm-ros/flight-logs
# CSV files with timestamped flight data
```

**ROS Logs:**
```bash
ros2 topic echo /rosout  # All node logs
```

**PM2 Logs:**
```bash
pm2 logs swarm-ros-launched
```

---

## 🚁 Pre-Flight Checklist

Before first flight:

- [ ] **Hardware verified**: Run `./verify_setup.sh` with all checks passing
- [ ] **GPS lock obtained**: 6+ satellites, HDOP < 2.0
- [ ] **LiDAR functional**: Reading correct distances
- [ ] **Battery charged**: Voltage > 14.5V
- [ ] **Home position set**: Check safety monitor logs
- [ ] **Flight controller armed**: Test arming sequence
- [ ] **Safety limits configured**: Review `swarm_params.yaml`
- [ ] **Custom RTH tested**: Verify in simulation first
- [ ] **Landing sequence tested**: Check LiDAR transition at 5m
- [ ] **Emergency procedures**: Know how to stop system (PM2 or Ctrl+C)
- [ ] **Clear flight area**: No obstacles within geofence

---

## 📚 Additional Documentation

- **[SETUP_GUIDE.md](SETUP_GUIDE.md)**: Detailed installation instructions
- **[PARAMETER_CONFIGURATION.md](PARAMETER_CONFIGURATION.md)**: Full parameter reference
- **Launch Files**: `src/swarm_ai_integration/launch/`
- **Configuration**: `src/swarm_ai_integration/config/swarm_params.yaml`

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📄 License

MIT License - see LICENSE file for details.

---

## 💬 Support

**For issues and questions:**
- Create an issue on GitHub
- Check [SETUP_GUIDE.md](SETUP_GUIDE.md) troubleshooting section
- Run `./verify_setup.sh` for diagnostics
- Review `/safety/status` for system health
- Check flight logs in `flight-logs/` directory

**System Health Check:**
```bash
# Quick health check
ros2 topic list | grep -E "safety|fc|ai|lidar"
ros2 topic hz /ai/observation
ros2 topic echo /safety/status --once
```

---

**Last Updated:** October 2024
**Version:** 1.0.0
