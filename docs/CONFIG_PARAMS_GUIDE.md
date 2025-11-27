# ROS2 Parameter Configuration Guide

Comprehensive guide to all configurable parameters in the Swarm AI Integration system.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Parameter File Location](#parameter-file-location)
3. [AI Adapter Node](#ai-adapter-node)
4. [AI Flight Node](#ai-flight-node)
5. [Safety Monitor Node](#safety-monitor-node)
6. [FC Communications Node](#fc-communications-node)
7. [FC Adapter Node](#fc-adapter-node)
8. [Black Box Recorder Node](#black-box-recorder-node)
9. [LiDAR Reader Node](#lidar-reader-node)
10. [Using Parameters](#using-parameters)
11. [Tuning Guide](#tuning-guide)
12. [Troubleshooting](#troubleshooting)

---

## Introduction

All configurable parameters for the Swarm AI Integration system are centralized in the `swarm_params.yaml` file. This guide explains each parameter, its purpose, valid ranges, and tuning recommendations.

**Key Benefits:**
- **Centralized configuration** - All settings in one place
- **No recompilation needed** - Change values without rebuilding
- **Environment-specific configs** - Different files for dev/production/testing
- **Runtime flexibility** - Modify parameters on the fly
- **Version control** - Track configuration changes in git

---

## Parameter File Location

**Default path:**
```
~/swarm-ros/src/swarm_ai_integration/config/swarm_params.yaml
```

**Installation path (after build):**
```
~/swarm-ros/install/swarm_ai_integration/share/swarm_ai_integration/config/swarm_params.yaml
```

---

## AI Adapter Node

**Node name:** `ai_adapter_node`

**Purpose:** Transforms raw sensor data into observation vectors for the AI model.

### Parameters

#### Observation Generation

##### `telemetry_rate`
```yaml
telemetry_rate: 30.0  # Hz
```

**Description:** Rate at which observations are generated and published to the AI model.

**Range:** 1.0 - 100.0 Hz

**Tuning:**
- **Higher (30-50 Hz):** More responsive AI, smoother control, higher CPU usage
- **Lower (10-20 Hz):** Less CPU usage, slightly delayed reactions
- **Recommendation:** 30 Hz provides good balance for most applications

**Impact:** Directly affects AI reaction time and control loop stability.

---

##### `max_ray_distance`
```yaml
max_ray_distance: 20.0  # meters
```

**Description:** Maximum distance used for normalizing LiDAR observations.

**Range:** 5.0 - 50.0 meters

**CRITICAL:** Must match the value used during AI model training!

**Tuning:**
- Value should match training environment
- Affects how distance observations are scaled (0-1)
- Incorrect value will cause AI to misinterpret distances

**Impact:** Incorrect value will cause AI to make poor decisions.

---

##### `action_buffer_size`
```yaml
action_buffer_size: 25  # past actions
```

**Description:** Number of past AI actions to buffer and include in observations.

**Range:** 0 - 100

**Tuning:**
- **Larger (25-50):** AI has more historical context, uses more memory
- **Smaller (10-20):** Less memory, less historical awareness
- **Zero (0):** No action history (not recommended for trained models expecting it)

**Impact:** Must match model training configuration. Affects memory usage.

---

#### Initial Position

##### `relative_start_enu`
```yaml
relative_start_enu: [0.0, 0.0, 3.0]  # [East, North, Up] meters
```

**Description:** Initial relative position in ENU (East-North-Up) frame.

**Format:** `[east, north, up]` in meters

**Tuning:**
- **East:** Positive = east, negative = west
- **North:** Positive = north, negative = south
- **Up:** Altitude offset (typically 3.0 meters for takeoff)

**Common values:**
- `[0.0, 0.0, 3.0]` - Start at origin, 3m altitude
- `[5.0, 0.0, 2.0]` - Start 5m east, 2m altitude

---

#### GPS Settings

##### `min_gps_satellites`
```yaml
min_gps_satellites: 5  # satellites
```

**Description:** Minimum number of GPS satellites required for valid GPS fix.

**Range:** 4 - 12

**Tuning:**
- **Lower (4-5):** Works in obstructed areas, less accuracy
- **Higher (6-8):** Better accuracy, may not work indoors/urban
- **Recommendation:** 5 for most scenarios, 6-7 for open sky

**Impact:** Affects when GPS-dependent modes can engage.

---

#### QoS Settings

##### `sensor_qos_depth`
```yaml
sensor_qos_depth: 1  # messages
```

**Description:** Queue depth for sensor topics (best-effort QoS).

**Range:** 1 - 10

**Tuning:**
- **1:** Latest data only (recommended for sensors)
- **5-10:** Buffer multiple messages (if processing is slow)

---

##### `reliable_qos_depth`
```yaml
reliable_qos_depth: 10  # messages
```

**Description:** Queue depth for reliable topics (commands, configuration).

**Range:** 1 - 100

**Tuning:**
- **10:** Default, handles most cases
- **20-50:** For high-rate command topics

---

#### Debug Settings

##### `debug_mode`
```yaml
debug_mode: false
```

**Description:** Enable detailed debug logging.

**Values:** `true` / `false`

**Impact:** Increases log verbosity, helpful for troubleshooting.

---

##### `publish_debug_info`
```yaml
publish_debug_info: false
```

**Description:** Publish additional debug topics.

**Values:** `true` / `false`

**Impact:** Creates extra ROS topics for debugging, increases CPU usage.

---

## AI Flight Node

**Node name:** `ai_flight_node`

**Purpose:** Runs the trained AI model and generates flight commands.

### Parameters

#### Model Settings

##### `model_path`
```yaml
model_path: "/home/pi/swarm-ros/model/UID_3.zip"
```

**Description:** Absolute path to the trained AI model file (.zip format).

**Format:** String, absolute path

**Common paths:**
- `/home/pi/swarm-ros/model/UID_3.zip` - Development model
- `/opt/swarm/model/production.zip` - Production model
- `~/swarm-ros/model/latest.zip` - Latest trained model

**Troubleshooting:**
- File must exist and be readable
- Must be a valid stable-baselines3 model
- Check path with: `ls -lh /path/to/model.zip`

---

##### `device`
```yaml
device: "cpu"
```

**Description:** Computation device for PyTorch inference.

**Values:**
- `"cpu"` - CPU inference (Raspberry Pi, most systems)
- `"cuda"` - GPU inference (if NVIDIA GPU available)
- `"cuda:0"` - Specific GPU device

**Tuning:**
- Raspberry Pi: Use `"cpu"` only
- Jetson Nano/Xavier: Can use `"cuda"`
- Desktop with GPU: Use `"cuda"` for faster inference

---

#### Prediction Settings

##### `prediction_rate`
```yaml
prediction_rate: 10.0  # Hz
```

**Description:** Rate at which AI model makes predictions.

**Range:** 1.0 - 50.0 Hz

**Tuning:**
- **Higher (20-30 Hz):** Faster reactions, higher CPU usage
- **Lower (5-10 Hz):** Lower CPU, slightly delayed reactions
- **Recommendation:** 10 Hz is sufficient for most flight tasks

**Impact:** Affects CPU usage and AI responsiveness. Should be ≤ `telemetry_rate`.

---

#### QoS Settings

##### `qos_depth`
```yaml
qos_depth: 1  # messages
```

**Description:** Queue depth for AI topics (observations and actions).

**Range:** 1 - 10

**Tuning:**
- **1:** Latest data only (recommended)
- **5:** Buffer if AI inference is slow

---

## Safety Monitor Node

**Node name:** `safety_monitor_node`

**Purpose:** Monitors flight parameters and enforces safety limits.

### Parameters

#### Attitude Limits

##### `max_roll_angle`
```yaml
max_roll_angle: 15.0  # degrees
```

**Description:** Maximum allowed roll angle before safety intervention.

**Range:** 5.0 - 45.0 degrees

**Tuning:**
- **Conservative (10-15°):** Safer, limits aggressive maneuvers
- **Aggressive (20-30°):** Allows faster movement, less stable
- **Recommendation:** 15° for autonomous flight

**Safety:** Exceeding this triggers safety override.

---

##### `max_pitch_angle`
```yaml
max_pitch_angle: 15.0  # degrees
```

**Description:** Maximum allowed pitch angle before safety intervention.

**Range:** 5.0 - 45.0 degrees

**Tuning:** Same as `max_roll_angle`

---

#### Altitude Limits

##### `min_altitude`
```yaml
min_altitude: 0.0  # meters
```

**Description:** Minimum allowed altitude (AGL - Above Ground Level).

**Range:** 0.0 - 5.0 meters

**Tuning:**
- **0.0:** Ground level (allows landing)
- **0.5-1.0:** Prevents ground contact during autonomous flight
- **Recommendation:** 0.0 to allow automatic landing

---

##### `max_altitude`
```yaml
max_altitude: 10.0  # meters
```

**Description:** Maximum allowed altitude (AGL).

**Range:** 5.0 - 120.0 meters

**Tuning:**
- **Low (5-10m):** Indoor/testing, safer
- **Medium (20-30m):** Outdoor flight
- **High (50-100m):** Advanced operations
- **Legal limit:** Check local regulations (often 120m/400ft)

**Safety:** Exceeding triggers automatic descent.

---

#### Geofencing

##### `max_distance_from_home`
```yaml
max_distance_from_home: 50.0  # meters
```

**Description:** Maximum horizontal distance from takeoff point.

**Range:** 10.0 - 1000.0 meters

**Tuning:**
- **Small (20-50m):** Testing, confined spaces
- **Medium (100-200m):** Normal operations
- **Large (500+m):** Long-range missions

**Safety:** Exceeding triggers RTH (Return to Home).

---

#### Safety Actions

##### `hover_duration`
```yaml
hover_duration: 3.0  # seconds
```

**Description:** How long to hover before initiating landing after safety event.

**Range:** 0.0 - 10.0 seconds

**Tuning:**
- **0:** Immediate landing (aggressive)
- **3-5:** Gives time for operator intervention
- **10:** Maximum delay before landing

---

#### System Configuration

##### `monitor_rate`
```yaml
monitor_rate: 20.0  # Hz
```

**Description:** Rate at which safety checks are performed.

**Range:** 5.0 - 50.0 Hz

**Tuning:**
- **Higher (20-30 Hz):** Faster safety response, more CPU
- **Lower (10-15 Hz):** Less CPU, slightly delayed safety checks
- **Recommendation:** 20 Hz for good safety response

---

##### `qos_depth`
```yaml
qos_depth: 1  # messages
```

**Description:** Queue depth for safety topics.

**Range:** 1 - 10

---

## FC Communications Node

**Node name:** `fc_comms_node`

**Purpose:** Communicates with INAV flight controller via MSP serial protocol.

### Parameters

#### Serial Communication

##### `serial_port`
```yaml
serial_port: "/dev/ttyAMA0"
```

**Description:** Serial port device for INAV communication.

**Common values:**
- `/dev/ttyAMA0` - Raspberry Pi hardware UART
- `/dev/ttyUSB0` - USB serial adapter
- `/dev/ttyACM0` - Some flight controllers

**Verification:**
```bash
ls -l /dev/ttyAMA0
```

---

##### `baud_rate`
```yaml
baud_rate: 115200
```

**Description:** Serial communication baud rate.

**Values:** Must match INAV MSP baud rate (typically 115200)

**Common values:** 9600, 57600, 115200

---

##### `timeout`
```yaml
timeout: 1.0  # seconds
```

**Description:** Serial read timeout.

**Range:** 0.1 - 5.0 seconds

**Tuning:**
- **Lower (0.5-1.0):** Faster timeout detection
- **Higher (2.0-3.0):** More tolerant of slow responses

---

##### `max_rx_buffer_size`
```yaml
max_rx_buffer_size: 1024  # bytes
```

**Description:** Maximum receive buffer size.

**Range:** 256 - 4096 bytes

**Tuning:** 1024 is sufficient for standard MSP messages.

---

#### Connection Management

##### `reconnect_interval`
```yaml
reconnect_interval: 5.0  # seconds
```

**Description:** Time between reconnection attempts after connection loss.

**Range:** 1.0 - 30.0 seconds

---

##### `telemetry_rate`
```yaml
telemetry_rate: 10.0  # Hz
```

**Description:** Rate at which telemetry data is requested from FC.

**Range:** 1.0 - 50.0 Hz

**Tuning:**
- **Higher (20-30 Hz):** More frequent updates, higher bus load
- **Lower (5-10 Hz):** Less load, slightly delayed data
- **Recommendation:** 10 Hz for stable telemetry

---

##### `heartbeat_rate`
```yaml
heartbeat_rate: 1.0  # Hz
```

**Description:** Rate of heartbeat checks to FC.

**Range:** 0.5 - 10.0 Hz

---

##### `heartbeat_timeout`
```yaml
heartbeat_timeout: 10.0  # seconds
```

**Description:** Time without heartbeat before declaring connection dead.

**Range:** 3.0 - 30.0 seconds

---

##### `connection_timeout`
```yaml
connection_timeout: 10.0  # seconds
```

**Description:** Overall connection timeout.

**Range:** 5.0 - 60.0 seconds

---

#### Waypoint Polling

##### `waypoint_poll_mode`
```yaml
waypoint_poll_mode: "first"
```

**Description:** How to poll waypoints from FC.

**Values:**
- `"first"` - Poll only first waypoint
- `"all"` - Poll all waypoints up to max_waypoint_cycle
- `"none"` - Don't poll waypoints

---

##### `max_waypoint_cycle`
```yaml
max_waypoint_cycle: 10
```

**Description:** Maximum waypoint index to cycle through (if poll mode is "all").

**Range:** 1 - 60

---

#### Data Processing Scales

##### `imu_scale_accel`
```yaml
imu_scale_accel: 0.01916015625  # 9.81/512.0
```

**Description:** IMU accelerometer scale factor.

**Note:** This is defined by INAV firmware, do not change unless you know what you're doing.

---

##### `imu_scale_gyro`
```yaml
imu_scale_gyro: 0.001
```

**Description:** IMU gyroscope scale factor.

**Note:** Defined by INAV firmware.

---

##### `battery_voltage_scale`
```yaml
battery_voltage_scale: 10.0
```

**Description:** Battery voltage divisor (INAV reports voltage * 10).

**Note:** Defined by INAV protocol.

---

##### `battery_current_scale`
```yaml
battery_current_scale: 100.0
```

**Description:** Battery current divisor (INAV reports amperage * 100).

**Note:** Defined by INAV protocol.

---

#### QoS Settings

##### `command_qos_depth`
```yaml
command_qos_depth: 10  # messages
```

**Description:** Queue depth for command topics.

**Range:** 1 - 50

---

## FC Adapter Node

**Node name:** `fc_adapter_node`

**Purpose:** Converts AI velocity commands to RC values and controls the flight controller via MSP.

### Parameters

#### Control Rates

##### `control_rate_hz`
```yaml
control_rate_hz: 40.0  # Hz
```

**Description:** Main control loop rate.

**Range:** 10.0 - 100.0 Hz

**Tuning:**
- **Higher (40-50 Hz):** Tighter control, more CPU usage
- **Lower (20-30 Hz):** Less CPU, slightly less responsive
- **Recommendation:** 40 Hz for good control with reasonable CPU usage

**Impact:** Directly affects control loop responsiveness and PID performance.

---

##### `prearm_rate_hz`
```yaml
prearm_rate_hz: 40  # Hz
```

**Description:** Rate at which pre-arm frames are sent.

**Range:** 20 - 100 Hz

**Recommendation:** Match to `control_rate_hz`

---

#### Velocity Limits

##### `max_velocity`
```yaml
max_velocity: 0.1  # m/s
```

**Description:** Maximum velocity command accepted from AI.

**Range:** 0.1 - 5.0 m/s

**Tuning:**
- **Very slow (0.1-0.5 m/s):** Testing, indoor flight
- **Slow (0.5-1.5 m/s):** Safe outdoor flight
- **Medium (1.5-3.0 m/s):** Normal operations
- **Fast (3.0-5.0 m/s):** Advanced, requires good tuning

**CRITICAL:** Start with low values (0.1-0.5) for initial testing!

**Safety:** Commands exceeding this are clamped.

---

#### Command Processing

##### `command_timeout`
```yaml
command_timeout: 1.0  # seconds
```

**Description:** Time without AI commands before switching to hover mode.

**Range:** 0.5 - 5.0 seconds

**Tuning:**
- **Short (0.5-1.0s):** Quick safety response, may trigger on network hiccups
- **Long (2.0-3.0s):** More tolerant, slower safety response

---

#### Timing

##### `warmup_frames`
```yaml
warmup_frames: 40  # frames
```

**Description:** Number of neutral RC frames to send before accepting AI commands.

**Range:** 10 - 100 frames

**Purpose:** Ensures FC stabilizes before AI control begins.

**Calculation:** At 40 Hz, 40 frames = 1 second

---

##### `startup_delay_sec`
```yaml
startup_delay_sec: 20.0  # seconds
```

**Description:** Delay before starting pre-arm sequence (gives operator time to prepare).

**Range:** 0.0 - 60.0 seconds

**Tuning:**
- **0:** Immediate start (automated systems)
- **20-30:** Time for operator to prepare
- **60+:** Extended preparation time

---

##### `prearm_duration_sec`
```yaml
prearm_duration_sec: 30.0  # seconds
```

**Description:** Duration of pre-arm sequence (streaming ARM command to FC).

**Range:** 10.0 - 60.0 seconds

**Purpose:** Allows FC to arm and stabilize.

**IMPORTANT:** FC requires throttle at 1000 during this phase to arm properly.

---

#### Pre-arm Configuration

##### `prearm_enabled`
```yaml
prearm_enabled: true
```

**Description:** Enable pre-arm sequence.

**Values:** `true` / `false`

**Recommendation:** Always `true` for autonomous arming.

---

##### `arm_aux_high`
```yaml
arm_aux_high: true
```

**Description:** Set ARM AUX channel (CH5) high during arm sequence.

**Values:** `true` / `false`

---

##### `enable_angle_mode`
```yaml
enable_angle_mode: true
```

**Description:** Enable ANGLE mode (self-leveling) on CH6.

**Values:** `true` / `false`

**Recommendation:** `true` for autonomous flight (provides stability).

---

##### `enable_althold_mode`
```yaml
enable_althold_mode: true
```

**Description:** Enable altitude hold mode on CH7.

**Values:** `true` / `false`

**CRITICAL:** Must be `true` for vertical control to work properly!

---

##### `enable_nav_rth`
```yaml
enable_nav_rth: false
```

**Description:** Enable NAV RTH (Return to Home) on CH9.

**Values:** `true` / `false`

**Tuning:**
- `false` - Normal operations, AI controls navigation
- `true` - Force RTH mode

---

#### MSP Serial Settings

##### `msp_serial_port`
```yaml
msp_serial_port: "/dev/ttyAMA0"
```

**Description:** MSP serial port for direct FC communication.

**Common values:**
- `/dev/ttyAMA0` - Raspberry Pi UART
- `/dev/ttyUSB0` - USB adapter

**Note:** Can be same as `fc_comms_node` serial port if using direct MSP.

---

##### `msp_baud_rate`
```yaml
msp_baud_rate: 115200
```

**Description:** MSP baud rate.

**Must match:** INAV MSP configuration.

---

##### `msp_write_timeout`
```yaml
msp_write_timeout: 0.05  # seconds
```

**Description:** MSP write timeout.

**Range:** 0.01 - 1.0 seconds

---

#### RC Channel Settings

##### `rc_mid_value`
```yaml
rc_mid_value: 1500
```

**Description:** RC channel center value (neutral).

**Standard:** 1500 (1000-2000 range)

---

##### `rc_min_value`
```yaml
rc_min_value: 1450
```

**Description:** RC minimum value (safety limit).

**Range:** 1100 - 1500

**Purpose:** Prevents extreme RC values that could cause instability.

---

##### `rc_max_value`
```yaml
rc_max_value: 1550
```

**Description:** RC maximum value (safety limit).

**Range:** 1500 - 1900

---

##### `rc_deviation_limit`
```yaml
rc_deviation_limit: 150.0  # RC units
```

**Description:** Maximum deviation from center (1500) allowed by PID controller.

**Range:** 50.0 - 500.0

**Tuning:**
- **Small (50-150):** Conservative, limited tilt angles
- **Medium (150-300):** Normal operations
- **Large (300-500):** Aggressive flight

**Calculation:** At 150, allows RC values from 1350-1650.

---

#### Yaw Control

##### `enable_yaw_control`
```yaml
enable_yaw_control: false
```

**Description:** Enable dynamic yaw alignment (heading control).

**Values:** `true` / `false`

**IMPORTANT:**
- `false` - Fixed yaw (drone maintains heading), matches most training scenarios
- `true` - Dynamic yaw (drone aligns with velocity direction), experimental

**Recommendation:** Keep `false` unless model was trained with yaw control.

---

##### `yaw_kp`
```yaml
yaw_kp: 200.0
```

**Description:** Proportional gain for yaw control (only used if `enable_yaw_control = true`).

**Range:** 50.0 - 500.0

---

##### `yaw_rate_limit`
```yaml
yaw_rate_limit: 300.0  # deg/s
```

**Description:** Maximum yaw rate change (only used if `enable_yaw_control = true`).

**Range:** 50.0 - 500.0 deg/s

---

#### PID Controller Gains

##### `kp_xy`
```yaml
kp_xy: 150.0
```

**Description:** Proportional gain for XY (horizontal) velocity control.

**Range:** 50.0 - 300.0

**Tuning:**
- **Lower (50-100):** Softer, less aggressive corrections
- **Medium (100-200):** Balanced response
- **Higher (200-300):** Aggressive, tight tracking

**Symptoms:**
- Too low: Sluggish response, poor tracking
- Too high: Oscillations, overshooting

---

##### `ki_xy`
```yaml
ki_xy: 10.0
```

**Description:** Integral gain for XY velocity control.

**Range:** 0.0 - 50.0

**Tuning:**
- **Lower (0-5):** Less wind compensation, less drift correction
- **Medium (5-15):** Balanced
- **Higher (15-30):** Strong drift correction, risk of integral windup

**Symptoms:**
- Too low: Position drift over time
- Too high: Slow oscillations, "toilet bowling"

---

##### `kd_xy`
```yaml
kd_xy: 20.0
```

**Description:** Derivative gain for XY velocity control (damping).

**Range:** 0.0 - 100.0

**Tuning:**
- **Lower (0-10):** Less damping, bouncy response
- **Medium (10-30):** Balanced damping
- **Higher (30-50):** Strong damping, may feel sluggish

**Symptoms:**
- Too low: Oscillations, bouncing
- Too high: Slow, mushy response

---

##### `kp_z`, `ki_z`, `kd_z`
```yaml
kp_z: 100.0
ki_z: 5.0
kd_z: 15.0
```

**Description:** Z-axis (vertical) PID gains.

**Note:** Currently not used - keeping for compatibility. Z-axis uses direct vz→throttle mapping instead.

---

#### PID Limits

##### `pid_output_min`
```yaml
pid_output_min: -400.0  # RC units
```

**Description:** Minimum PID output value.

**Range:** -500.0 - 0.0

---

##### `pid_output_max`
```yaml
pid_output_max: 400.0  # RC units
```

**Description:** Maximum PID output value.

**Range:** 0.0 - 500.0

**Note:** Should match `rc_deviation_limit` or be slightly larger.

---

##### `pid_integral_max`
```yaml
pid_integral_max: 50.0  # RC units
```

**Description:** Maximum integral accumulation (anti-windup).

**Range:** 10.0 - 200.0

**Purpose:** Prevents integral term from becoming too large.

---

##### `pid_derivative_filter_alpha`
```yaml
pid_derivative_filter_alpha: 0.1
```

**Description:** Low-pass filter coefficient for derivative term (0-1).

**Range:** 0.0 - 1.0

**Tuning:**
- **Lower (0.05-0.1):** Stronger filtering, smoother but delayed
- **Higher (0.2-0.5):** Less filtering, more responsive but noisier

---

##### `pid_max_history_length`
```yaml
pid_max_history_length: 100  # samples
```

**Description:** Maximum PID history length for logging/analysis.

**Range:** 10 - 1000

---

#### Z-Axis Direct Control

##### `vz_to_throttle_scale`
```yaml
vz_to_throttle_scale: 100.0  # RC units per m/s
```

**Description:** Conversion factor from vertical velocity (m/s) to throttle RC units.

**Range:** 50.0 - 200.0

**Tuning:**
- **Lower (50-75):** Less responsive altitude control
- **Medium (75-125):** Balanced
- **Higher (125-200):** More aggressive altitude changes

**Calculation:** vz = 0.5 m/s → throttle deviation = 0.5 * 100 = 50 RC units

---

##### `throttle_rate_limit`
```yaml
throttle_rate_limit: 200.0  # RC units/second
```

**Description:** Maximum rate of throttle change.

**Range:** 50.0 - 500.0 units/s

**Purpose:** Prevents sudden throttle changes that could destabilize the drone.

**Tuning:**
- **Lower (50-100):** Smooth, gradual altitude changes
- **Medium (100-250):** Balanced
- **Higher (250-500):** Fast altitude response

---

## Black Box Recorder Node

**Node name:** `black_box_recorder_node`

**Purpose:** Records all flight data to CSV files for post-flight analysis.

### Parameters

#### Storage Settings

##### `log_directory`
```yaml
log_directory: "~/swarm-ros/flight-logs"
```

**Description:** Directory where flight log files are saved.

**Format:** String, supports `~` expansion

**Common values:**
- `~/swarm-ros/flight-logs` - Default location
- `/var/log/swarm_blackbox` - System log location
- `/mnt/usb/logs` - External storage

**Note:** Directory will be created if it doesn't exist.

---

##### `max_log_file_size_mb`
```yaml
max_log_file_size_mb: 100  # MB
```

**Description:** Maximum log file size before rotation.

**Range:** 10 - 1000 MB

**Tuning:**
- **Small (10-50 MB):** More files, easier to manage
- **Large (100-500 MB):** Fewer files, longer flights per file

---

#### Logging Behavior

##### `log_rate_hz`
```yaml
log_rate_hz: 10.0  # Hz
```

**Description:** Frequency of snapshot logging.

**Range:** 1.0 - 100.0 Hz

**Tuning:**
- **Lower (5-10 Hz):** Smaller log files, less detail
- **Higher (20-50 Hz):** Larger files, more detail
- **Recommendation:** 10 Hz captures sufficient detail

**Impact:** Higher rates create larger log files.

---

#### QoS Settings

##### `reliable_qos_depth`
```yaml
reliable_qos_depth: 10  # messages
```

**Description:** Queue depth for reliable data logging.

**Range:** 1 - 50

---

##### `sensor_qos_depth`
```yaml
sensor_qos_depth: 1  # messages
```

**Description:** Queue depth for sensor data logging.

**Range:** 1 - 10

---

## LiDAR Reader Node

**Node name:** `down_lidar/lidar_reader_down` (for downward-facing sensor)

**Purpose:** Reads distance measurements from VL53L0X LiDAR sensor via I2C.

### Parameters

#### I2C Communication

##### `i2c_bus`
```yaml
i2c_bus: 1
```

**Description:** I2C bus number.

**Common values:**
- Raspberry Pi: 1 (default)
- Some platforms: 0 or 2

**Verify:**
```bash
i2cdetect -l
```

---

##### `i2c_address`
```yaml
i2c_address: 0x08
```

**Description:** I2C device address of the LiDAR sensor.

**Format:** Hexadecimal (0x00 - 0x7F)

**Common values:**
- VL53L0X default: 0x29
- Custom address: 0x08 (as configured in your system)

**Verify:**
```bash
i2cdetect -y 1
# Should show device at configured address
```

---

##### `distance_register`
```yaml
distance_register: 0x24
```

**Description:** Register address for reading distance data.

**Note:** Sensor-specific, typically 0x24 for VL53L0X custom firmware.

---

#### Sensor Configuration

##### `frame_id`
```yaml
frame_id: "down_lidar_link"
```

**Description:** TF frame ID for this sensor.

**Purpose:** Used for coordinate transformations in ROS.

**Format:** String, valid TF frame name

---

##### `sensor_position`
```yaml
sensor_position: "down"
```

**Description:** Mounting position description.

**Values:** `"down"`, `"front"`, `"back"`, etc.

**Purpose:** Helps identify which sensor in multi-sensor setups.

---

##### `publish_rate`
```yaml
publish_rate: 100.0  # Hz
```

**Description:** Publishing rate for distance measurements.

**Range:** 1.0 - 100.0 Hz

**Tuning:**
- **Maximum:** 100 Hz (sensor hardware limit)
- **Recommended:** 50-100 Hz for responsive obstacle detection
- **Lower:** 20-50 Hz for reduced CPU usage

---

#### Range Limits

##### `max_range`
```yaml
max_range: 50.0  # meters
```

**Description:** Maximum valid measurement range.

**Range:** 1.0 - 50.0 meters

**Note:** VL53L0X typical max range is ~2m in good conditions, 50m is theoretical max.

---

##### `min_range`
```yaml
min_range: 0.05  # meters
```

**Description:** Minimum valid measurement range.

**Range:** 0.0 - 1.0 meters

**Note:** VL53L0X minimum is typically ~5cm.

---

##### `field_of_view`
```yaml
field_of_view: 0.035  # radians
```

**Description:** Sensor field of view cone angle.

**Value:** ~2 degrees for VL53L0X

**Note:** Used for visualization and obstacle detection algorithms.

---

#### Data Processing

##### `enable_filtering`
```yaml
enable_filtering: true
```

**Description:** Enable moving average filtering on distance readings.

**Values:** `true` / `false`

**Recommendation:** `true` to reduce noise and false readings.

---

##### `filter_window_size`
```yaml
filter_window_size: 5  # samples
```

**Description:** Moving average filter window size.

**Range:** 1 - 20

**Tuning:**
- **Small (3-5):** Less lag, less smoothing
- **Medium (5-10):** Balanced
- **Large (10-20):** Heavily smoothed, more lag

---

##### `max_invalid_readings`
```yaml
max_invalid_readings: 10  # consecutive readings
```

**Description:** Maximum consecutive invalid readings before reporting sensor error.

**Range:** 5 - 50

**Purpose:** Distinguishes temporary noise from actual sensor failure.

---

#### Timing

##### `status_publish_interval`
```yaml
status_publish_interval: 1.0  # seconds
```

**Description:** How often to publish sensor status messages.

**Range:** 0.5 - 10.0 seconds

---

##### `reconnect_interval`
```yaml
reconnect_interval: 5.0  # seconds
```

**Description:** Time between I2C reconnection attempts after sensor loss.

**Range:** 1.0 - 30.0 seconds

---

#### QoS Settings

##### `sensor_qos_depth`
```yaml
sensor_qos_depth: 1  # messages
```

**Description:** Queue depth for distance measurements.

**Range:** 1 - 10

**Recommendation:** 1 for latest reading only.

---

##### `reliable_qos_depth`
```yaml
reliable_qos_depth: 10  # messages
```

**Description:** Queue depth for status messages.

**Range:** 1 - 50

---

#### Conversion

##### `mm_to_m_divisor`
```yaml
mm_to_m_divisor: 1000.0
```

**Description:** Conversion factor from millimeters to meters.

**Value:** 1000.0 (standard conversion)

**Note:** Do not change unless sensor reports in different units.

---

## Using Parameters

### Loading from File

Parameters are automatically loaded by the launch file:

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

### Override at Launch

Override individual parameters:

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
  fc_adapter_node.max_velocity:=1.0 \
  fc_adapter_node.kp_xy:=200.0
```

### Use Custom Config File

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
  params_file:=/path/to/custom_params.yaml
```

### Runtime Parameter Changes

**List parameters:**
```bash
ros2 param list /fc_adapter_node
```

**Get parameter value:**
```bash
ros2 param get /fc_adapter_node max_velocity
```

**Set parameter value:**
```bash
ros2 param set /fc_adapter_node max_velocity 0.5
```

**Dump all parameters:**
```bash
ros2 param dump /fc_adapter_node > fc_adapter_backup.yaml
```

**Load parameters:**
```bash
ros2 param load /fc_adapter_node fc_adapter_backup.yaml
```

---

## Tuning Guide

### Initial Testing Configuration

Start with conservative values for first flights:

```yaml
fc_adapter_node:
  ros__parameters:
    max_velocity: 0.1          # Very slow
    kp_xy: 100.0               # Conservative P
    ki_xy: 5.0                 # Low I
    kd_xy: 15.0                # Moderate D
    rc_deviation_limit: 100.0  # Limited tilt

safety_monitor_node:
  ros__parameters:
    max_altitude: 5.0          # Low altitude
    max_distance_from_home: 20.0  # Close to home
```

### Progressive Tuning Steps

1. **Test hover and basic movement** (max_velocity: 0.1-0.3 m/s)
2. **Tune PID for smooth tracking** (adjust kp_xy, ki_xy, kd_xy)
3. **Increase speed gradually** (max_velocity: 0.5-1.0 m/s)
4. **Expand safety limits** (altitude, distance as needed)
5. **Fine-tune responsiveness** (control_rate_hz, prediction_rate)

### Common Tuning Scenarios

#### Drone drifts during hover
- Increase `ki_xy` (try 10-15)
- Check GPS satellite count
- Verify compass calibration

#### Oscillations during flight
- Decrease `kp_xy` (try 100-120)
- Increase `kd_xy` (try 25-30)
- Check `control_rate_hz` isn't too high

#### Sluggish response
- Increase `kp_xy` (try 180-200)
- Increase `max_velocity`
- Check `prediction_rate` is adequate (10+ Hz)

#### Aggressive altitude changes
- Reduce `vz_to_throttle_scale` (try 75-90)
- Increase `throttle_rate_limit` (smooth ramping)

---

## Troubleshooting

For comprehensive troubleshooting information, see the [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md).

**Parameter-specific issues:**
- **Invalid YAML syntax** → Validate with: `python3 -c "import yaml; yaml.safe_load(open('swarm_params.yaml'))"`
- **Parameters not applied** → Check: `ros2 param list /node_name`
- **Performance issues** → Reduce control rates, telemetry rates

See [Parameter & Configuration Issues](TROUBLESHOOTING_GUIDE.md#parameter--configuration-issues) and [Performance Issues](TROUBLESHOOTING_GUIDE.md#performance-issues) in the Troubleshooting Guide.

---

## Best Practices

### 1. Version Control
- Commit `swarm_params.yaml` to git
- Create separate configs for different scenarios
- Document changes in commit messages

### 2. Backup Before Changes
```bash
cp swarm_params.yaml swarm_params.yaml.backup
```

### 3. Test in Simulation First
- Test parameter changes in safe environment
- Verify behavior before real hardware

### 4. Document Custom Configurations
- Add comments to explain why values were changed
- Note hardware-specific requirements

### 5. Start Conservative
- Low velocities, low altitude, close to home
- Increase limits gradually as confidence builds

