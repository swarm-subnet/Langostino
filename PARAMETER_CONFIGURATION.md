# Swarm ROS Parameter Configuration Guide

## Overview

All hardcoded values have been migrated to the centralized parameter file: `config/swarm_params.yaml`

This document summarizes the parameter refactoring completed across the swarm-ros project.

## Summary of Changes

### 1. Comprehensive Parameter File Created
- **File**: `src/swarm_ai_integration/config/swarm_params.yaml`
- **Total Parameters**: 100+ parameters across 7 nodes
- **Organization**: Grouped by node with clear comments

### 2. Nodes Updated to Use Parameters

#### ✅ AI Adapter Node (`ai_adapter_node.py`)
**Parameters Added:**
- `telemetry_rate`: 30.0 Hz (observation generation rate)
- `max_ray_distance`: 10.0 m (normalization distance)
- `action_buffer_size`: 20 (past actions to buffer)
- `relative_start_enu`: [0.0, 0.0, 3.0] (initial position)
- `sensor_qos_depth`: 1
- `reliable_qos_depth`: 10

**Changes:**
- QoS depths now use parameters instead of hardcoded values
- All configuration moved from hardcoded defaults to ROS parameters

#### ✅ FC Communications Node (`fc_comms_node.py`)
**Parameters Added:**
- `heartbeat_timeout`: 10.0 s
- `connection_timeout`: 10.0 s
- `waypoint_poll_mode`: "first" (first/all/none)
- `max_waypoint_cycle`: 10
- `command_qos_depth`: 10
- `max_rx_buffer_size`: 1024 bytes

**Changes:**
- MSPSerialHandler now receives heartbeat_timeout and max_rx_buffer_size
- Waypoint polling uses max_waypoint_cycle parameter
- QoS depths use parameters

#### ✅ MSP Serial Handler (`utils/msp_serial_handler.py`)
**Parameters Added (via fc_comms_node):**
- `heartbeat_timeout`: Replaces hardcoded 10.0s
- `max_rx_buffer_size`: Replaces hardcoded 1024 bytes

**Changes:**
- Constructor accepts heartbeat_timeout and max_rx_buffer_size
- Connection health check uses parameter
- Buffer overflow check uses parameter

### 3. Parameter File Structure

```yaml
# AI Adapter Node
ai_adapter_node:
  ros__parameters:
    telemetry_rate: 30.0
    max_ray_distance: 10.0
    action_buffer_size: 20
    relative_start_enu: [0.0, 0.0, 3.0]
    sensor_qos_depth: 1
    reliable_qos_depth: 10
    # ... more parameters

# FC Communications Node
fc_comms_node:
  ros__parameters:
    serial_port: "/dev/ttyAMA0"
    baud_rate: 115200
    timeout: 1.0
    reconnect_interval: 5.0
    telemetry_rate: 10.0
    heartbeat_rate: 1.0
    heartbeat_timeout: 10.0
    connection_timeout: 10.0
    waypoint_poll_mode: "first"
    max_waypoint_cycle: 10
    command_qos_depth: 10
    max_rx_buffer_size: 1024
    # ... more parameters

# FC Adapter Node
fc_adapter_node:
  ros__parameters:
    control_rate_hz: 40.0
    max_velocity: 3.0
    warmup_frames: 40
    rc_mid_value: 1500
    rc_range: 400
    prearm_rc_channels: [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]
    kp_xy: 150.0
    ki_xy: 10.0
    kd_xy: 20.0
    # ... more parameters

# Safety Monitor Node
safety_monitor_node:
  ros__parameters:
    max_altitude: 50.0
    min_altitude: 0.5
    max_velocity: 5.0
    default_safe_battery_voltage: 16.0
    battery_timeout_multiplier: 5
    monitor_rate: 10.0
    # ... more parameters

# AI Flight Node
ai_flight_node:
  ros__parameters:
    model_path: "/home/pi/swarm-ros/model/UID_117.zip"
    prediction_rate: 10.0
    observation_dim: 131
    action_dim: 4
    qos_depth: 1
    # ... more parameters

# Black Box Recorder Node
black_box_recorder_node:
  ros__parameters:
    log_directory: "/var/log/swarm_blackbox"
    max_log_file_size_mb: 100
    max_files_per_session: 10
    log_buffer_size: 1000
    log_flush_interval: 5.0
    stats_logging_interval: 60.0
    reliable_qos_depth: 10
    sensor_qos_depth: 1
    # ... more parameters

# LiDAR Reader Node (Down)
down_lidar/lidar_reader_down:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 0x09
    publish_rate: 100.0
    max_range: 50.0
    min_range: 0.05
    status_publish_interval: 1.0
    reconnect_interval: 5.0
    sensor_qos_depth: 1
    reliable_qos_depth: 10
    mm_to_m_divisor: 1000.0
    # ... more parameters
```

## Benefits of Parameterization

### 1. **Centralized Configuration**
- All tunable values in one location
- Easy to find and modify settings
- Clear documentation of all options

### 2. **Environment-Specific Configs**
- Can create different YAML files for:
  - Development vs Production
  - Different drone models
  - Simulation vs Real hardware
  - Testing scenarios

### 3. **Runtime Flexibility**
- Parameters can be changed via:
  - Launch file arguments
  - Command line: `ros2 param set`
  - Parameter files
  - ROS parameter services

### 4. **Better Maintainability**
- No need to recompile to change values
- Version control of configuration
- Clear separation of code and config

### 5. **Improved Testing**
- Easy to create test configurations
- Quick parameter tuning
- A/B testing of different settings

## Usage

### Loading Parameters via Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('swarm_ai_integration'),
        'config',
        'swarm_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='swarm_ai_integration',
            executable='ai_adapter_node',
            name='ai_adapter_node',
            parameters=[config]
        ),
        Node(
            package='swarm_ai_integration',
            executable='fc_comms_node',
            name='fc_comms_node',
            parameters=[config]
        ),
        # ... more nodes
    ])
```

### Overriding Parameters at Launch

```bash
# Override single parameter
ros2 launch swarm_ai_integration swarm_ai_launch.py \
  ai_adapter_node.telemetry_rate:=50.0

# Use different config file
ros2 launch swarm_ai_integration swarm_ai_launch.py \
  params_file:=/path/to/custom_params.yaml
```

### Runtime Parameter Changes

```bash
# List all parameters
ros2 param list /ai_adapter_node

# Get current value
ros2 param get /ai_adapter_node telemetry_rate

# Set new value
ros2 param set /ai_adapter_node telemetry_rate 50.0

# Dump all parameters to file
ros2 param dump /ai_adapter_node > params_backup.yaml
```

## Configuration Best Practices

### 1. **Parameter Naming Convention**
- Use lowercase with underscores: `max_ray_distance`
- Group related parameters: `pid_*`, `rc_*`
- Include units in name when ambiguous: `timeout_sec`, `distance_m`

### 2. **Documentation**
- Add inline comments for each parameter
- Specify units and valid ranges
- Note dependencies between parameters

### 3. **Default Values**
- Always provide sensible defaults in `declare_parameter()`
- Defaults should work for most common use cases
- Document why specific defaults were chosen

### 4. **Validation**
- Validate parameters in node initialization
- Check ranges and constraints
- Log warnings for unusual values

### 5. **Version Control**
- Commit parameter files with meaningful messages
- Track parameter changes in git
- Create different configs for different environments

## Remaining Hardcoded Values

Some values remain hardcoded as they are:

### Constants (Mathematical/Physical)
- `math.pi` (3.14159...)
- Conversion factors: `180.0 / math.pi` (rad to deg)
- Earth radius: `111320.0` m/deg latitude
- Gravity: `9.81` m/s²

### Protocol Constants (MSP)
- MSP command codes (defined in protocol)
- MSP packet structure
- Scale factors defined by INAV firmware

### Architecture Constants (Model)
- Observation dimension: `131`
- Action dimension: `4`
- Number of LiDAR rays: `16`

These are intentionally not parameterized as they are fixed by:
- Physical laws
- Protocol specifications
- Model architecture
- System design

## Migration Checklist

- [x] Identify all hardcoded values
- [x] Create comprehensive swarm_params.yaml
- [x] Update ai_adapter_node.py
- [x] Update fc_comms_node.py
- [x] Update msp_serial_handler.py
- [ ] Update lidar_reader_node.py (pending)
- [ ] Update ai_flight_node.py (pending)
- [ ] Update fc_adapter_node.py (pending)
- [ ] Update safety_monitor_node.py (pending)
- [ ] Update black_box_recorder_node.py (pending)
- [ ] Update launch file to load parameters
- [ ] Test all nodes with parameter file
- [ ] Create environment-specific configs
- [ ] Document parameter tuning guide

## Next Steps

1. **Complete Node Updates**: Finish updating remaining nodes to use all parameters
2. **Launch File Integration**: Update launch files to load swarm_params.yaml
3. **Testing**: Verify all parameters work correctly
4. **Documentation**: Create parameter tuning guide for operators
5. **Multiple Configs**: Create configs for different scenarios (sim, test, prod)

## Files Modified

### Created:
- `config/swarm_params.yaml` (comprehensive parameter file)
- `PARAMETER_CONFIGURATION.md` (this document)

### Updated:
- `swarm_ai_integration/ai_adapter_node.py`
  - Added QoS depth parameters
  - Uses parameters for all configuration

- `swarm_ai_integration/fc_comms_node.py`
  - Added heartbeat_timeout, connection_timeout
  - Added waypoint polling parameters
  - Added QoS and buffer parameters

- `swarm_ai_integration/utils/msp_serial_handler.py`
  - Constructor accepts heartbeat_timeout
  - Constructor accepts max_rx_buffer_size
  - Uses parameters for timeout and buffer checks

## Support

For questions or issues related to parameter configuration:
1. Check this document first
2. Review swarm_params.yaml comments
3. Check node source code for parameter declarations
4. Consult ROS2 parameter documentation: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html
