# Swarm ROS2 Integration

**Version:** 1.0.0
**Status:** Active Development
**Last Updated:** October 2025

ROS2 integration package for real-world deployment of Swarm AI flight control system with INAV 7 flight controller. This package enables autonomous flight using trained PPO models with real sensor data from I2C LiDAR, GPS, IMU, and MSP telemetry.

## Project Status

**Current Focus:** Closed-loop velocity control with PID-based MSP RC command generation

**Recent Updates:**
- ✅ Implemented direct MSP serial communication with INAV 7
- ✅ Added pre-arm streaming protocol (30s arming sequence)
- ✅ Integrated PID velocity controller (vx, vy, vz → RC channels)
- ✅ Waypoint-based goal tracking with ENU coordinate conversion
- ✅ Relative positioning system with configurable initial offset
- ✅ Return to Home (RTH) safety integration via INAV CH9
- ✅ Black box flight data recorder
- ✅ I2C-based LiDAR sensor support (front & down)

**Architecture Overview:**

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│  AI Flight Node │ ───> │  AI Adapter Node │ <─── │  FC Comms Node  │
│  (PPO Policy)   │      │  (131-D Obs)     │      │  (MSP Protocol) │
└─────────────────┘      └──────────────────┘      └─────────────────┘
         │                        │                          │
         │ /ai/action             │ /ai/observation          │ /fc/* topics
         ↓                        ↓                          ↓
┌─────────────────────────────────────────────────────────────────────┐
│                        FC Adapter Node                               │
│            (Velocity PID → MSP RC Commands)                          │
└─────────────────────────────────────────────────────────────────────┘
         │                                                     │
         │ MSP_SET_RAW_RC (Direct Serial)                    │
         ↓                                                     ↓
┌──────────────────┐                              ┌───────────────────┐
│  INAV 7 FC       │                              │  Safety Monitor   │
│  (Hardware)      │                              │  (RTH, Override)  │
└──────────────────┘                              └───────────────────┘
```

## Core Components

The system consists of seven main nodes:

### 1. AI Adapter Node (`ai_adapter_node.py`)
**Purpose:** Converts real sensor data into the 131-dimensional observation array for the PPO model

**Key Features:**
- **Relative ENU Positioning:** Initial position set to [0, 0, 3]m, configurable via `RELATIVE_START_ENU`
- **Waypoint Goal Tracking:** Converts geodetic waypoints to local ENU offsets
- **Action Buffer:** Maintains 20-step action history for temporal awareness
- **Zero-Filling:** Automatically fills with zeros when no new action messages arrive
- **Observation Structure (131-D):**
  - [0:3] Relative position (E, N, U) in meters
  - [3:7] Orientation quaternion (from `/fc/attitude`)
  - [7:10] Orientation Euler angles (roll, pitch, yaw) in radians
  - [10:13] Velocity ENU (from GPS speed/course)
  - [13:16] Angular velocity (from IMU)
  - [16:20] Last action received
  - [20:100] Action buffer (20 × 4)
  - [100:112] Padding (12 zeros)
  - [112:128] LiDAR distances (16 rays, normalized)
  - [128:131] Goal vector (ENU, scaled by 10m)

**Subscribed Topics:**
- `/fc/gps_fix` (sensor_msgs/NavSatFix) - GPS position (lat, lon, alt)
- `/fc/attitude` (geometry_msgs/QuaternionStamped) - Orientation quaternion
- `/fc/attitude_euler` (geometry_msgs/Vector3Stamped) - Euler angles [rad]
- `/fc/imu_raw` (sensor_msgs/Imu) - Angular velocity only
- `/fc/gps_speed_course` (std_msgs/Float32MultiArray) - [speed_mps, course_deg]
- `/fc/waypoint` (std_msgs/Float32MultiArray) - Current goal waypoint
- `/lidar_distance` (sensor_msgs/Range) - Down-facing LiDAR
- `/ai/action` (std_msgs/Float32MultiArray) - AI commands for action buffer

**Published Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - 131-D observation array @ 30 Hz
- `/ai/observation_debug` (std_msgs/Float32MultiArray) - Debug vector [E, N, U, yaw, lidar, action_flag]

### 2. AI Flight Node (`ai_flight_node.py`)
**Purpose:** Executes PPO policy inference for autonomous flight control

**Key Features:**
- **Secure Model Loading:** `weights_only=True` PyTorch loading, no pickle exploits
- **Safe Metadata:** Reads network architecture from `safe_policy_meta.json` in model zip
- **No Training Environment:** Uses static dummy env, only policy weights loaded
- **Hardcoded Model Path:** `/home/pi/swarm-ros/model/UID_117.zip`
- **Fixed Inference Rate:** 10 Hz prediction cycle
- **Deterministic Actions:** `predict(obs, deterministic=True)`
- **NaN Safety:** Auto-replaces NaN/Inf with zeros

**Model Format:**
- Input: 131-D observation (float32)
- Output: [vx, vy, vz, speed] (4-D action)
- Architecture: PPO with MLP policy (configured via JSON metadata)

**Subscribed Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - 131-D observation from adapter

**Published Topics:**
- `/ai/action` (std_msgs/Float32MultiArray) - [vx, vy, vz, speed] @ 10 Hz

### 3. Safety Monitor Node (`safety_monitor_node.py`)
**Purpose:** Real-time safety monitoring with RTH integration

**Safety Checks @ 10 Hz:**
- **Altitude Limits:** Min/max bounds (default: 0.5m - 50m)
- **Velocity Limits:** Max speed threshold (default: 5 m/s)
- **Battery Monitoring:** Low voltage detection (default: <14V)
- **Geofencing:** Distance from home position (default: 100m)
- **Obstacle Proximity:** LiDAR-based collision warning (default: <1m)
- **Communication Timeout:** Detects lost telemetry (default: 2s)

**Failsafe Actions:**
- **Hover Override:** Publishes `/safety/override` → FC Adapter sends neutral RC
- **RTH Activation:** Publishes `/safety/rth_command` → FC Adapter sets CH9 = 1800 (INAV RTH mode)

**Critical Violations Triggering RTH:**
- Low battery
- Excessive distance from home
- Communication loss

**Subscribed Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - Parsed for position & LiDAR
- `/ai/action` (geometry_msgs/Twist) - AI command monitoring
- `/battery_state` (sensor_msgs/BatteryState) - Battery voltage
- `/drone/pose` (geometry_msgs/PoseStamped) - Current position
- `/drone/velocity` (geometry_msgs/Twist) - Current velocity

**Published Topics:**
- `/safety/override` (std_msgs/Bool) - Hover mode trigger
- `/safety/rth_command` (std_msgs/Bool) - RTH activation (INAV CH9)
- `/safety/status` (std_msgs/String) - Status summary

### 4. FC Communications Node (`fc_comms_node.py`)
**Purpose:** MSP protocol interface to INAV 7 flight controller

**Key Features:**
- **Threaded Serial I/O:** Non-blocking read/write with command queue
- **Auto-Reconnection:** 5-second retry interval on connection loss
- **Telemetry Polling @ 10 Hz:** Rotates through MSP_RAW_IMU, MSP_RAW_GPS, MSP_ATTITUDE, MSP_STATUS, MSP_ANALOG, MSP_MOTOR
- **Waypoint Polling:** Configurable modes:
  - `first`: Poll WP #1 @ 10 Hz (first mission waypoint)
  - `all`: Cycle WP #0 to #N @ 10 Hz (includes home/current)
- **RC Echo:** Optional MSP_RC request after MSP_SET_RAW_RC for verification
- **Derived Topics:** Publishes `/fc/gps_speed_course` [speed_mps, course_deg] and `/fc/msp_status` [cycle_time, i2c_errors, sensor_mask, box_flags, current_setting]

**MSP Commands Handled:**
- **Outgoing:** MSP_SET_RAW_RC, MSP_IDENT (heartbeat), MSP_WP
- **Incoming:** MSP_RAW_IMU, MSP_RAW_GPS, MSP_ATTITUDE, MSP_STATUS, MSP_ANALOG, MSP_MOTOR, MSP_WP, MSP_RC

**Subscribed Topics:**
- `/fc/msp_command` (std_msgs/Float32MultiArray) - Raw MSP command injection
- `/fc/rc_override` (std_msgs/Float32MultiArray) - RC channels [AETR + AUX1..AUXn]

**Published Topics:**
- `/fc/imu_raw` (sensor_msgs/Imu) - IMU data (accel, gyro; orientation = identity)
- `/fc/gps_fix` (sensor_msgs/NavSatFix) - GPS position (lat, lon, alt, fix status)
- `/fc/attitude` (geometry_msgs/QuaternionStamped) - Attitude quaternion (from Euler)
- `/fc/attitude_euler` (geometry_msgs/Vector3Stamped) - Euler angles [roll, pitch, yaw] rad
- `/fc/status` (std_msgs/String) - Human-readable FC status
- `/fc/msp_status` (std_msgs/Float32MultiArray) - Raw MSP_STATUS data
- `/fc/battery` (sensor_msgs/BatteryState) - Battery voltage & current
- `/fc/connected` (std_msgs/Bool) - Connection health
- `/fc/motor_rpm` (std_msgs/Float32MultiArray) - Motor values (4 motors)
- `/fc/gps_speed_course` (std_msgs/Float32MultiArray) - [speed_mps, course_deg]
- `/fc/waypoint` (std_msgs/Float32MultiArray) - [wp_no, lat, lon, alt_m, heading, stay, navflag]
- `/fc/altitude` (std_msgs/Float32MultiArray) - [baro_alt_m, vz_mps] (if implemented)

### 5. FC Adapter Node (`fc_adapter_node.py`)
**Purpose:** Closed-loop velocity control via PID → MSP RC commands with direct serial output

**Control Flow:**
1. **Pre-Arm Streaming (30s):** Sends ARM frame @ 40 Hz before accepting AI commands
   - Channels: `[1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]`
   - CH5 (ARM) = 1800, CH8 (MSP OVERRIDE) = 1800
2. **Warm-Up (40 frames):** Sends neutral RC to stabilize before closed-loop
3. **Closed-Loop Control @ 40 Hz:**
   - Reads `/ai/action` [vx, vy, vz, speed]
   - Converts GPS speed/course → ENU velocity
   - Transforms ENU → body frame using yaw
   - PID computes (vx_err, vy_err, vz_err) → (pitch_dev, roll_dev, throttle_dev)
   - Builds RC channels: `[1500+roll_dev, 1500+pitch_dev, 1500+throttle_dev, 1500, AUX...]`
   - Direct MSP_SET_RAW_RC via serial (bypasses fc_comms_node)
4. **Safety Modes:**
   - **Timeout Hover:** No `/ai/action` for >1s → neutral RC
   - **Safety Override:** `/safety/override` = True → neutral RC
   - **RTH Mode:** `/safety/rth_command` = True → CH9 = 1800 (INAV RTH)

**PID Configuration:**
- **XY (Roll/Pitch):** kp=150, ki=10, kd=20
- **Z (Throttle):** kp=100, ki=5, kd=15
- **Output:** RC deviations ±400 (1100-1900 range)

**RC Channel Mapping (16 channels):**
- CH1 (Roll), CH2 (Pitch), CH3 (Throttle), CH4 (Yaw)
- CH5 (ARM): 1800 if `arm_aux_high`, else 1000
- CH6 (ANGLE): 1800 if `enable_angle_mode`
- CH7 (ALT HOLD): 1800 if `enable_althold_mode`
- CH8 (MSP OVERRIDE): **MUST be 1800** for MSP RC control
- CH9 (NAV RTH): 1800 if RTH active

**Subscribed Topics:**
- `/ai/action` (std_msgs/Float32MultiArray) - [vx, vy, vz, speed]
- `/fc/gps_speed_course` (std_msgs/Float32MultiArray) - [speed_mps, course_deg]
- `/fc/attitude_euler` (geometry_msgs/Vector3Stamped) - [roll, pitch, yaw] rad
- `/fc/altitude` (std_msgs/Float32MultiArray) - [baro_alt_m, vz_mps]
- `/safety/override` (std_msgs/Bool) - Hover trigger
- `/safety/rth_command` (std_msgs/Bool) - RTH activation

**Published Topics:**
- `/fc_adapter/status` (std_msgs/String) - Operational status
- `/fc_adapter/velocity_error` (geometry_msgs/Vector3Stamped) - [ex, ey, ez] body frame

**Direct MSP Serial:**
- Port: `/dev/ttyAMA0` (configurable)
- Baud: 115200
- Writes MSP_SET_RAW_RC directly (no fc_comms_node dependency for RC commands)

### 6. LiDAR Reader Node (`lidar_reader_node.py`)
**Purpose:** I2C-based LiDAR sensor interface for front & down ranging

**Key Features:**
- **I2C Protocol:** Reads 4-byte little-endian uint32 from register 0x24 (distance in mm)
- **Multi-Sensor Support:** Namespaced instances for different mounting positions
- **Moving Average Filter:** Configurable window (default: 5 samples)
- **Range Validation:** Min/max bounds (default: 0.05m - 50m)
- **Auto-Reconnect:** 5s retry on I2C failure
- **Health Monitoring:** Consecutive invalid reading threshold (default: 10)
- **Publishing Rate:** Up to 100 Hz (sensor max spec)

**I2C Configuration:**
- **Bus:** `/dev/i2c-1` (default)
- **Front Sensor:** Address 0x08
- **Down Sensor:** Address 0x09
- **Register:** 0x24 (distance data)

**Sensor Positions:**
- `front`: Point along +X axis
- `down`: Point along -Z axis

**Published Topics (per sensor instance):**
- `lidar_distance` (sensor_msgs/Range) - Filtered distance measurement
- `lidar_raw` (std_msgs/Float32MultiArray) - [distance_m, timestamp]
- `lidar_status` (std_msgs/String) - Health, position, I2C stats
- `lidar_point` (geometry_msgs/PointStamped) - 3D point in sensor frame
- `lidar_healthy` (std_msgs/Bool) - Connection health

**Namespaced Instances:**
- `front_lidar/lidar_reader_front` → `/front_lidar/lidar_distance_front`
- `down_lidar/lidar_reader_down` → `/down_lidar/lidar_distance_down`

### 7. Black Box Recorder Node (`black_box_recorder_node.py`)
**Purpose:** Flight data logging for post-flight analysis and debugging

**Key Features:**
- **Comprehensive Logging:** Records all critical topics (observations, actions, telemetry, safety)
- **Timestamped CSV:** Human-readable format with nanosecond timestamps
- **Auto-Rotation:** Creates new files when size limit reached (default: 100MB)
- **Compression:** Optionally compresses old log files to save space
- **Session Management:** Organizes logs by session with max file limits
- **Buffered Writes:** Batched I/O with configurable flush interval (default: 5s)

**Log Directory:** `/var/log/swarm_blackbox` (default)

**Logged Topics:**
- `/ai/observation`, `/ai/action`, `/ai/observation_debug`
- `/fc/gps_fix`, `/fc/attitude_euler`, `/fc/imu_raw`
- `/fc/gps_speed_course`, `/fc/waypoint`, `/fc/battery`
- `/safety/status`, `/safety/override`, `/safety/rth_command`
- `/fc_adapter/status`, `/fc_adapter/velocity_error`

See `README_BlackBox.md` for detailed logging format and analysis tools.

## Installation

### Prerequisites
- **ROS 2 Humble** (or compatible distribution)
- **Python 3.10+**
- **Hardware:**
  - Raspberry Pi 4 (recommended) or similar SBC
  - INAV 7 flight controller with MSP support
  - I2C LiDAR sensors (optional: front & down)
  - GPS module
  - IMU (typically integrated in flight controller)

### System Dependencies

1. **Install I2C tools (for LiDAR):**
   ```bash
   sudo apt install i2c-tools python3-smbus2
   sudo usermod -a -G i2c $USER  # Add user to i2c group
   ```

2. **Install ROS 2 dependencies:**
   ```bash
   sudo apt install ros-humble-geometry-msgs ros-humble-sensor-msgs \
                    ros-humble-nav-msgs ros-humble-tf2-ros
   ```

### Package Setup

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-org/swarm-ros.git swarm-ros
   ```

2. **Install Python dependencies:**
   ```bash
   cd ~/ros2_ws/src/swarm-ros/src/swarm_ai_integration
   pip install -r ai_model_requirements.txt
   ```

   Dependencies: `numpy`, `torch`, `stable-baselines3`, `gymnasium`, `typing-extensions`, `pyserial`, `smbus2`

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select swarm_ai_integration
   source install/setup.bash
   ```

4. **Verify serial permissions:**
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyAMA0  # Or your FC serial port
   ```

## Usage

### Pre-Flight Checks

1. **Verify I2C LiDAR sensors:**
   ```bash
   i2cdetect -y 1  # Should show devices at 0x08 and 0x09
   ```

2. **Test MSP connection:**
   ```bash
   ros2 run swarm_ai_integration fc_comms_node.py --ros-args \
       -p serial_port:=/dev/ttyAMA0 -p baud_rate:=115200
   # Check for telemetry output
   ```

3. **Place AI model:**
   ```bash
   mkdir -p /home/pi/swarm-ros/model
   # Copy UID_117.zip to /home/pi/swarm-ros/model/
   ```

### Full System Launch

**Standard launch (all nodes):**
```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
    serial_port:=/dev/ttyAMA0 \
    baud_rate:=115200 \
    max_velocity:=3.0 \
    max_altitude:=50.0 \
    enable_safety:=true \
    enable_blackbox:=true \
    log_directory:=/var/log/swarm_blackbox
```

**Production deployment with PM2 (process monitoring and auto-restart):**
```bash
pm2 start "ros2 launch swarm_ai_integration swarm_ai_launch.py" --name swarm_ai
```

PM2 provides process monitoring, automatic restart on failure, and system startup integration. To enable auto-start on boot:
```bash
pm2 save
pm2 startup
```

**Minimal launch (testing without LiDAR):**
```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
    serial_port:=/dev/ttyAMA0 \
    baud_rate:=115200 \
    front_lidar_i2c_address:=999 \
    down_lidar_i2c_address:=999  # Dummy addresses to skip LiDAR
```

### Individual Node Testing

**FC Comms + Adapter only (MSP testing):**
```bash
ros2 run swarm_ai_integration fc_comms_node.py --ros-args \
    -p serial_port:=/dev/ttyAMA0 -p baud_rate:=115200 \
    -p wp_poll_mode:=first
```

**AI Adapter (observation generation):**
```bash
ros2 run swarm_ai_integration ai_adapter_node.py
```

**FC Adapter (velocity control testing):**
```bash
ros2 run swarm_ai_integration fc_adapter_node.py --ros-args \
    -p msp_port:=/dev/ttyAMA0 -p msp_baudrate:=115200 \
    -p prearm_enabled:=true -p prearm_seconds:=30.0 \
    -p startup_delay_sec:=20.0
```

**Safety Monitor:**
```bash
ros2 run swarm_ai_integration safety_monitor_node.py --ros-args \
    -p max_altitude:=50.0 -p min_altitude:=0.5 \
    -p max_velocity:=5.0
```

## Configuration

### Parameter Files

**Main config:** `config/swarm_params.yaml`

Key parameters to adjust:
- **AI Adapter:**
  - `MAX_RAY_DISTANCE`: LiDAR normalization scale (default: 10.0m)
  - `RELATIVE_START_ENU`: Initial relative position [E, N, U] (default: [0, 0, 3])
- **FC Adapter (PID):**
  - `kp_xy`, `ki_xy`, `kd_xy`: Roll/Pitch PID gains (default: 150, 10, 20)
  - `kp_z`, `ki_z`, `kd_z`: Throttle PID gains (default: 100, 5, 15)
  - `prearm_seconds`: Arming duration (default: 30.0s)
  - `startup_delay_sec`: Operator setup window (default: 20.0s)
- **Safety Monitor:**
  - `max_altitude`, `min_altitude`: Altitude bounds (50.0m, 0.5m)
  - `max_velocity`: Speed limit (5.0 m/s)
  - `min_battery_voltage`: Low battery threshold (14.0V)
  - `max_distance_from_home`: Geofence radius (100.0m)
- **FC Comms:**
  - `wp_poll_mode`: Waypoint polling (`first` or `all`)
  - `telemetry_rate`: MSP polling frequency (10.0 Hz)

### INAV Flight Controller Setup

**Required INAV Settings:**
1. **Enable MSP RC Override:**
   - Configure CH8 as MSP RC Override mode (range 1700-2100)
2. **Set Flight Modes:**
   - CH6: ANGLE mode
   - CH7: ALT HOLD mode (or POS HOLD for GPS stabilization)
   - CH9: NAV RTH (Return to Home)
3. **Configure Arming:**
   - CH5 as ARM switch (range 1700-2100)
   - Set prearm checks as needed
4. **GPS Settings:**
   - Enable GPS and set home position
   - Configure RTH altitude and behavior
5. **Serial Port:**
   - Enable MSP on UART that connects to Pi (typically UART1)
   - Set baud rate to 115200

### Hardware Wiring

**Flight Controller ↔ Raspberry Pi:**
- UART TX → Pi RX (GPIO 15 / `/dev/ttyAMA0`)
- UART RX → Pi TX (GPIO 14)
- GND → GND

**I2C LiDAR Sensors:**
- SDA → Pi GPIO 2 (I2C1 SDA)
- SCL → Pi GPIO 3 (I2C1 SCL)
- VCC → 3.3V or 5V (sensor dependent)
- GND → GND

## Safety Features

The system includes comprehensive safety monitoring:

1. **Altitude Limits**: Prevents flying too high or low
2. **Velocity Limits**: Limits maximum speed
3. **Geofencing**: Prevents flying too far from home
4. **Obstacle Avoidance**: Monitors LiDAR for close obstacles
5. **Battery Monitoring**: Triggers emergency landing on low battery
6. **Communication Timeout**: Activates failsafe on lost communication
7. **Emergency Landing**: Automated emergency landing procedures

### Safety Override

When safety violations are detected:
1. AI control is immediately disabled
2. Failsafe actions are activated
3. Emergency procedures may be triggered
4. System status is logged and published

## Monitoring and Debugging

### Status Topics

Monitor system status using these topics:
- `/ai/status` - AI system status and performance
- `/safety/status` - Safety monitoring status
- `/ai/observation_debug` - Debug information about observations

### Logging

Enable debug logging:
```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py debug_mode:=true
```

### Visualization

Use RViz2 to visualize system state:
```bash
ros2 run rviz2 rviz2 -d config/swarm_visualization.rviz
```

## Troubleshooting

### Common Issues

1. **Model Loading Failed**
   - Check model path is correct
   - Ensure Swarm package is in Python path
   - Verify model file format

2. **No Observation Data**
   - Check sensor topic names and data rates
   - Verify coordinate frame transformations
   - Check GPS origin configuration

3. **Safety Override Active**
   - Check safety parameter limits
   - Monitor `/safety/status` for violation details
   - Verify sensor data quality

4. **Poor Control Performance**
   - Check observation array construction
   - Verify LiDAR ray mapping
   - Monitor prediction timing

### Debug Commands

```bash
# Check topic rates
ros2 topic hz /ai/observation

# Monitor AI status
ros2 topic echo /ai/status

# Check safety status
ros2 topic echo /safety/status

# View observation data
ros2 topic echo /ai/observation_debug
```

## Hardware Requirements

- **Compute**: CPU with 4+ cores (GPU recommended for larger models)
- **Memory**: 8GB+ RAM
- **Sensors**: LiDAR, IMU, GPS, flight controller with telemetry
- **Communication**: Reliable low-latency connection to flight controller

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes and add tests
4. Submit a pull request

## Support

For issues and questions:
- Create an issue on GitHub
- Check the troubleshooting section
- Review system logs for error messages