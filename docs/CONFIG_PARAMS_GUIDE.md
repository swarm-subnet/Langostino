# ROS2 Parameter Configuration Guide

Comprehensive guide to all configurable parameters in the Swarm AI Integration system.

> 📖 **Deep Dive:** For context on how these parameters fit into the overall system architecture, see:
> - [Chapter 3: From Data to Motion](https://substack.com/home/post/p-177453660) — Software architecture and data flow
> - [Chapter 3.5: Additional Configurations](https://substack.com/home/post/p-180586067) — Advanced configuration and tuning

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

### 131-D Observation Array Structure

The AI Adapter Node produces a 131-dimensional observation array that matches the PyBullet training environment format:

| Index | Size | Component | Description | Units |
|-------|------|-----------|-------------|-------|
| 0-2 | 3 | **Position ENU** | Relative position from start point | meters [E, N, U] |
| 3-5 | 3 | **Orientation** | Euler angles | radians [roll, pitch, yaw] |
| 6-8 | 3 | **Velocity** | Linear velocity in ENU frame | m/s [vE, vN, vU] |
| 9-11 | 3 | **Angular Velocity** | Rotational velocity | rad/s [wx, wy, wz] |
| 12-111 | 100 | **Action Buffer** | History of 25 past actions (25 × 4) | normalized [-1, 1] |
| 112-127 | 16 | **LiDAR Distances** | Normalized ray distances | normalized [0, 1] |
| 128-130 | 3 | **Goal Vector** | Direction to goal in ENU | normalized by max_ray_distance |

**Total: 131 elements**

#### Component Details

- **Position ENU [0:3]**: Drone position relative to the starting point, in East-North-Up coordinates
- **Orientation [3:6]**: Roll, pitch, yaw angles from the IMU in radians
- **Velocity [6:9]**: Ground speed decomposed into ENU components from GPS
- **Angular Velocity [9:12]**: Gyroscope readings in rad/s
- **Action Buffer [12:112]**: Rolling buffer of the last 25 actions (each action is 4 values: thrust + 3-axis rotation), enabling the model to understand recent control history
- **LiDAR [112:128]**: 16 distance rays normalized by `max_ray_distance` (0 = at sensor, 1 = max range)
- **Goal Vector [128:131]**: Vector from current position to goal, scaled by `max_ray_distance`

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

**Description:** Initial relative position in ENU (East-North-Up) frame. Our models were trained with [ 0.0, 0.0, 3.0] as starting point.

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

**Disclaimer:** Other modes besides CPU have not yet been tested, and this parameter is intended for future implementations.

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

**Description:** Minimum allowed altitude (AGL).

**Range:** 0.0 - 10.0 meters

**Tuning:**
- **0.0:** No minimum altitude check (default)
- **1.0-2.0:** Prevents landing during flight
- **Higher:** Use for specific mission requirements

**Note:** Currently not actively used for safety triggers, reserved for future implementations.

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

#### GPS Altitude Fallback

##### `gps_altitude_timeout`
```yaml
gps_altitude_timeout: 5.0  # seconds
```

**Description:** How long GPS altitude data remains valid for use as barometer fallback.

**Range:** 1.0 - 30.0 seconds

**Purpose:** When barometer is unavailable or invalid, GPS altitude is used as fallback if it was received within this timeout period.

**Tuning:**
- **Short (2-5s):** Only use very recent GPS data
- **Long (10-30s):** Allow older GPS data as fallback

**Note:** GPS altitude is less accurate than barometer but provides backup when barometer fails.

---

## FC Adapter Node

**Node name:** `fc_adapter_node`

**Purpose:** Converts AI action vectors to RC values for the flight controller. Uses a normalization-based joystick-style mapping where the direction vector is normalized by its max component and scaled by a speed factor.

### Action Vector Mapping

The AI publishes `[vx, vy, vz, speed]` action vectors. The node maps these to RC channels using normalization:

1. Find the max absolute component in `(vx, vy, vz)`
2. Normalize each component by this max value (so the dominant axis reaches ±1)
3. Scale by `speed` (0-1)
4. Map to RC range: `-1 → rc_min`, `0 → rc_mid`, `+1 → rc_max`

**Examples:**
- `(1, 1, 1, 1)` and `(0.25, 0.25, 0.25, 1)` produce the same output (same direction & speed)
- `(1, -1, 0, 1)` → roll=1550, pitch=1450, throttle=1500

### Parameters

#### Control Rate

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

**Impact:** Directly affects control loop responsiveness.

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

#### Startup Sequence Timing

##### `warmup_duration_sec`
```yaml
warmup_duration_sec: 10.0  # seconds
```

**Description:** Total warmup duration before AI control begins.

**Range:** 5.0 - 30.0 seconds

**Purpose:** Overall warmup period that allows the flight controller to stabilize before AI takes control.

**Tuning:**
- **Shorter (5-10s):** Faster startup, less stabilization time
- **Longer (15-30s):** More time for GPS lock and FC stabilization

---

##### `arming_duration_sec`
```yaml
arming_duration_sec: 20.0  # seconds
```

**Description:** Duration of the arming phase with ARM channel high and throttle at minimum.

**Range:** 5.0 - 60.0 seconds

**Purpose:** Time for the flight controller to complete arming sequence, initialize sensors, and acquire GPS lock.

**Tuning:**
- **Shorter (10-15s):** Quick startup, may miss GPS lock
- **Longer (20-30s):** More time for GPS satellites and sensor initialization
- **Recommendation:** 20 seconds provides good balance for outdoor flights

**Startup Sequence:** This is Phase 0 (Arming) → followed by Phase 1 (Rise) → Phase 2 (Yaw Alignment) → Phase 3 (AI Control)

---

##### `rise_duration_sec`
```yaml
rise_duration_sec: 5.0  # seconds
```

**Description:** Duration of the rise phase where the drone climbs to operating altitude with POSHOLD enabled.

**Range:** 2.0 - 15.0 seconds

**Purpose:** Elevates the drone to a safe altitude before AI control begins.

**Tuning:**
- **Shorter (2-3s):** Quick rise, less time to stabilize altitude
- **Longer (5-10s):** Gradual rise, more stable altitude hold engagement
- **Recommendation:** 5 seconds for smooth transition to altitude hold

**Note:** During this phase, throttle is set to `rise_throttle` (hardcoded at 1550) with POSHOLD mode active.

---

#### RC Channel Settings

##### `rc_mid_value`
```yaml
rc_mid_value: 1500
```

**Description:** RC channel center value (neutral position).

**Standard:** 1500 (1000-2000 range)

**Note:** This is the standard RC midpoint. Do not change unless your FC is configured differently.

---

##### `rc_min_value`
```yaml
rc_min_value: 1450
```

**Description:** Minimum RC value output (safety limit).

**Range:** 1000 - 1500

**Purpose:** Limits how far below center the RC values can go, preventing extreme control inputs.

**Calculation:** With `rc_mid_value: 1500` and `rc_min_value: 1450`, maximum negative deflection is 50 RC units.

---

##### `rc_max_value`
```yaml
rc_max_value: 1550
```

**Description:** Maximum RC value output (safety limit).

**Range:** 1500 - 2000

**Purpose:** Limits how far above center the RC values can go, preventing extreme control inputs.

**Calculation:** With `rc_mid_value: 1500` and `rc_max_value: 1550`, maximum positive deflection is 50 RC units.

**IMPORTANT:** The range between `rc_min_value` and `rc_max_value` determines the maximum tilt/thrust the drone can achieve. The `half_range` (rc_max - rc_mid = 50) is the maximum RC deflection per axis. Start with conservative values (1450-1550) for testing.

---

### Tuning the RC Range

The control sensitivity is determined entirely by `rc_min_value` and `rc_max_value`. There are no per-axis gain parameters — the normalization approach ensures all axes use the full RC range proportionally.

1. **Start conservative:**
   ```yaml
   rc_min_value: 1450
   rc_max_value: 1550
   ```

2. **Increase range for more aggressive flight:**
   ```yaml
   rc_min_value: 1350
   rc_max_value: 1650
   ```

3. **For testing:** Keep the RC range narrow until behavior is verified.

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

**Purpose:** Reads distance measurements from DFRobot dTOF SEN0684 LiDAR sensor via I2C.

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
- dTOF SEN0684 default: 0x29
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

**Note:** Sensor-specific, typically 0x24 for dTOF SEN0684 custom firmware.

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

**Description:** Maximum valid measurement range. This value is normalized (0 to 1) to the maximum range used in model training (20 m). Any value above this range is considered 1.

**Range:** 1.0 - 50.0 meters

**Note:** dTOF SEN0684 typical max range is ~35m in good conditions, 50m is theoretical max.

---

##### `min_range`
```yaml
min_range: 0.05  # meters
```

**Description:** Minimum valid measurement range.

**Range:** 0.0 - 1.0 meters

**Note:** dTOF SEN0684 minimum is typically ~5cm.

---

##### `field_of_view`
```yaml
field_of_view: 0.035  # radians
```

**Description:** Sensor field of view cone angle.

**Value:** ~2 degrees for dTOF SEN0684

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
  fc_adapter_node.rc_max_value:=1600 \
  fc_adapter_node.rc_min_value:=1400
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
ros2 param get /fc_adapter_node rc_max_value
```

**Set parameter value:**
```bash
ros2 param set /fc_adapter_node rc_max_value 1600
```

**Dump all parameters:**
```bash
ros2 param dump /fc_adapter_node > fc_adapter_backup.yaml
```

**Load parameters:**
```bash
ros2 param load /fc_adapter_node fc_adapter_backup.yaml
```

## Tuning Guide

### Initial Testing Configuration

Start with conservative values for first flights:

```yaml
fc_adapter_node:
  ros__parameters:
    rc_min_value: 1450         # Limited RC range (50 units deflection)
    rc_max_value: 1550         # Limited RC range (50 units deflection)
    warmup_duration_sec: 10.0  # Good stabilization time
    arming_duration_sec: 20.0  # Time for GPS lock

safety_monitor_node:
  ros__parameters:
    max_altitude: 5.0          # Low altitude
    max_distance_from_home: 20.0  # Close to home
```

### Progressive Tuning Steps

1. **Test hover and basic movement** with narrow RC range (1450-1550)
2. **Verify stability** - drone should hold position without oscillations
3. **Increase RC range gradually** for more aggressive flight (1400-1600, then 1350-1650)
4. **Expand safety limits** (altitude, distance as needed)

### Common Tuning Scenarios

#### Drone movement is too slow
- Increase `rc_max_value` and decrease `rc_min_value` (wider range)

#### Drone overshoots or feels twitchy
- Narrow the RC range (`rc_min_value: 1450`, `rc_max_value: 1550`)

#### Altitude changes too aggressively
- Narrow the RC range via `rc_min_value`/`rc_max_value`

#### Drone drifts during hover
- This is typically a flight controller tuning issue (INAV PIDs)
- Check GPS satellite count
- Verify compass calibration
- See [INAV_GUIDE.md](INAV_GUIDE.md) for FC-level tuning

#### Sluggish response to AI commands
- Widen RC range (`rc_min_value`/`rc_max_value`)
- Check `prediction_rate` is adequate (10+ Hz)

## Troubleshooting

For comprehensive troubleshooting information, see the [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md).

**Parameter-specific issues:**
- **Invalid YAML syntax** → Validate with: `python3 -c "import yaml; yaml.safe_load(open('swarm_params.yaml'))"`
- **Parameters not applied** → Check: `ros2 param list /node_name`
- **Performance issues** → Reduce control rates, telemetry rates

See [Parameter & Configuration Issues](TROUBLESHOOTING_GUIDE.md#parameter--configuration-issues) and [Performance Issues](TROUBLESHOOTING_GUIDE.md#performance-issues) in the Troubleshooting Guide.

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
