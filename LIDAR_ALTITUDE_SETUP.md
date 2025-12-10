# LiDAR-to-Altitude Bridge Setup

## Overview

This document describes the LiDAR-to-Altitude bridge setup that replaces the barometer altitude sensor with LiDAR-based altitude measurements.

**Reason**: The barometer chip on the flight controller is non-functional, so we're using the downward-facing LiDAR sensor to provide altitude data instead.

## Architecture

```
┌─────────────────────┐
│  Downward LiDAR     │
│  (I2C Sensor)       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ lidar_reader_node   │
│ (down_lidar)        │
└──────────┬──────────┘
           │
           │ /down_lidar/lidar_distance_down
           │ (sensor_msgs/Range)
           ▼
┌─────────────────────────────┐
│ lidar_altitude_bridge_node  │
│ • Converts distance→altitude│
│ • Calculates vario (d/dt)   │
└──────────┬──────────────────┘
           │
           │ /fc/altitude
           │ (Float32MultiArray: [altitude_m, vario_m/s])
           ▼
┌─────────────────────┐
│ Rest of System      │
│ • safety_monitor    │
│ • ai_adapter        │
│ • black_box         │
└─────────────────────┘
```

## Changes Made

### 1. New Node: `lidar_altitude_bridge_node.py`

**Location**: `src/swarm_ai_integration/swarm_ai_integration/lidar_altitude_bridge_node.py`

**Function**:
- Subscribes to `/down_lidar/lidar_distance_down` (from downward-facing LiDAR)
- Converts LiDAR distance to altitude format
- Calculates vertical velocity (vario) using linear regression on recent altitude samples
- Publishes to `/fc/altitude` in barometer-compatible format: `[altitude_m, vario_m/s]`

**Key Features**:
- **Vario Calculation**: Uses a sliding window (default: 5 samples) to calculate vertical velocity
- **Data Validation**: Validates LiDAR readings against min/max range
- **Health Monitoring**: Publishes health status to `/lidar_altitude_healthy`
- **Status Publishing**: Provides diagnostic info on `/lidar_altitude_status`

**Parameters**:
- `lidar_topic`: Input LiDAR topic (default: `/lidar_distance`)
- `altitude_topic`: Output altitude topic (default: `/fc/altitude`)
- `vario_window_size`: Number of samples for vario calculation (default: 5)
- `vario_time_threshold`: Minimum time span for vario calculation (default: 0.5s)
- `max_vario`: Maximum reasonable vario value (default: 10.0 m/s)
- `publish_status`: Enable status publishing (default: True)
- `status_interval`: Status publishing interval (default: 1.0s)

### 2. Modified: `fc_comms_node.py`

**Change**: Disabled `MSP_ALTITUDE` request in telemetry sequence

**Location**: `fc_comms_node.py:99-109`

```python
# NOTE: MSP_ALTITUDE disabled because barometer is not available
# Altitude data is now provided by lidar_altitude_bridge_node
self.telemetry_commands = [
    MSPCommand.MSP_RAW_IMU,
    MSPCommand.MSP_RAW_GPS,
    MSPCommand.MSP_ATTITUDE,
    # MSPCommand.MSP_ALTITUDE,  # DISABLED - barometer unavailable, using LiDAR
    MSPCommand.MSP_STATUS,
    MSPCommand.MSP_ANALOG,
    MSPCommand.MSP_MOTOR
]
```

**Reason**: Prevents error messages from trying to read from a dead barometer sensor.

### 3. Modified: `swarm_ai_launch.py`

**Changes**:
- Added `lidar_altitude_bridge_node` definition (lines 179-194)
- Added node to launch sequence (line 347)

**Launch Order**:
1. `down_lidar_node` - Reads LiDAR sensor
2. **`lidar_altitude_bridge_node`** - Converts to altitude
3. `fc_comms_node` - Publishes other FC telemetry
4. Rest of the system nodes

### 4. Modified: `CMakeLists.txt`

**Change**: Added `lidar_altitude_bridge_node.py` to installed executables (line 30)

## Building and Running

### Build

```bash
cd ~/swarm-ros
colcon build --packages-select swarm_ai_integration
source install/setup.bash
```

### Launch

The bridge node is automatically launched with the standard launch file:

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py
```

### Verify Operation

Check that the bridge is working:

```bash
# Check if altitude topic is being published
ros2 topic echo /fc/altitude

# Check bridge health status
ros2 topic echo /lidar_altitude_healthy

# Check bridge diagnostic status
ros2 topic echo /lidar_altitude_status

# Check LiDAR input
ros2 topic echo /down_lidar/lidar_distance_down
```

Expected output format for `/fc/altitude`:
```
data: [1.234, -0.05]  # [altitude in meters, vario in m/s]
```

## Monitoring

### Log Messages

The bridge node logs altitude publications at INFO level, matching the original barometer format:

```
   ➜ Published to /fc/altitude | altitude=1.23m | vario=+0.05m/s [LiDAR source]
```

The `[LiDAR source]` tag helps identify that the data comes from LiDAR rather than barometer.

### Status Topic

Subscribe to `/lidar_altitude_status` for diagnostic information:

```bash
ros2 topic echo /lidar_altitude_status
```

Example output:
```
data: "Health: OK | Altitude: 1.234m | Vario: +0.050m/s | Msgs: 1523 | Rate: 100.2Hz | Source: LiDAR→Baro"
```

## Troubleshooting

### No altitude data published

**Check**:
1. Is the LiDAR node running and publishing?
   ```bash
   ros2 topic echo /down_lidar/lidar_distance_down
   ```

2. Is the bridge node running?
   ```bash
   ros2 node list | grep lidar_altitude_bridge
   ```

3. Check bridge logs:
   ```bash
   ros2 node info /lidar_altitude_bridge
   ```

### Vario shows 0.0

**Possible causes**:
- Insufficient samples collected (needs at least 2 samples)
- Time span between samples too short (< 0.5s by default)
- Aircraft not moving vertically

**Solution**: Wait a few seconds for vario calculation to stabilize.

### LiDAR reading out of valid range

**Symptom**: Warning messages like:
```
LiDAR reading out of valid range: 52.3m (valid: 0.05-50.0m)
```

**Cause**: LiDAR is reading beyond its valid range (typically 0.05m to 50m)

**Solution**:
- Check LiDAR sensor is functioning correctly
- Verify aircraft is within valid altitude range
- Check for obstructions blocking the LiDAR sensor

## Technical Details

### Vario Calculation Method

The vertical velocity (vario) is calculated using **linear regression** on recent altitude samples:

1. Collect time-stamped altitude measurements in a sliding window
2. Fit a line: `altitude = vario × time + offset`
3. Extract the slope (rate of change) as vario

**Advantages**:
- Smooths out noise from individual samples
- More accurate than simple difference calculation
- Adapts to varying sample rates

**Limitations**:
- Requires minimum time span (default: 0.5s)
- Assumes approximately linear motion over the window period
- Clamped to max ±10 m/s to prevent spurious values

### Data Format Compatibility

The bridge node publishes data in **exactly the same format** as the original barometer:

- Topic: `/fc/altitude`
- Type: `std_msgs/Float32MultiArray`
- Format: `[altitude_meters, vario_meters_per_second]`

This ensures **zero changes needed** to downstream consumers:
- `safety_monitor_node` - altitude limit checks
- `ai_adapter_node` - observation building
- `black_box_recorder_node` - flight logging

## Future Improvements

Potential enhancements:

1. **Sensor Fusion**: Combine LiDAR with GPS altitude for better accuracy
2. **Kalman Filtering**: Add state estimation for smoother vario calculation
3. **Multi-LiDAR Support**: Fuse data from multiple LiDAR sensors
4. **Terrain Following**: Distinguish between altitude AGL (LiDAR) and MSL (GPS)
5. **Fallback Mode**: Auto-switch to GPS altitude if LiDAR fails

## Notes

- **Performance**: The bridge adds negligible latency (~1ms)
- **Accuracy**: LiDAR altitude is more accurate than barometer for low-altitude flight
- **Limitations**: Only works over solid ground; won't work over water or transparent surfaces
- **Safety**: The existing safety monitor altitude limits still apply

## Reverting to Barometer

If the barometer is repaired, to revert:

1. Edit `fc_comms_node.py:105` - uncomment `MSPCommand.MSP_ALTITUDE`
2. Edit `swarm_ai_launch.py:347` - remove `lidar_altitude_bridge_node` from launch
3. Rebuild: `colcon build --packages-select swarm_ai_integration`

---

**Last Updated**: 2024-12-10
**Author**: System Integration
**Status**: Active (Production)
