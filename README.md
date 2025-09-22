# Swarm ROS2 Integration

ROS2 integration package for real-world deployment of Swarm AI flight control system. This package enables the use of trained Swarm AI models with real sensor data from LiDAR, IMU, GPS, and flight controllers.

## Architecture

The system consists of three main nodes:

### 1. AI Adapter Node (`ai_adapter_node.py`)
Converts real sensor data into the 131-dimensional observation array expected by the Swarm AI model:
- **Base observation (112-D)**: Position, orientation, velocity, angular velocity, RPM, etc.
- **LiDAR distances (16-D)**: Obstacle detection in 16 directions
- **Goal vector (3-D)**: Relative position to target

**Subscribed Topics:**
- `/lidar/scan` (sensor_msgs/LaserScan) - LiDAR point cloud data
- `/imu/data` (sensor_msgs/Imu) - IMU orientation and angular velocity
- `/gps/fix` (sensor_msgs/NavSatFix) - GPS position data
- `/mavros/local_position/velocity_local` (geometry_msgs/TwistStamped) - Velocity from flight controller
- `/fc/motor_rpm` (std_msgs/Float32MultiArray) - Motor RPM data
- `/move_base_simple/goal` (geometry_msgs/PoseStamped) - Target position

**Published Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - 131-D observation array for AI model

### 2. AI Flight Node (`ai_flight_node.py`)
Loads and executes the pre-trained Swarm AI model for real-time flight control:
- Loads the Swarm AI model using the secure loader
- Processes 131-D observations to generate control commands
- Implements safety limits and timeout handling

**Subscribed Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - Observation data from adapter
- `/ai/enable` (std_msgs/Bool) - Enable/disable AI control
- `/safety/override` (std_msgs/Bool) - Safety override signal

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Control commands for flight controller
- `/ai/status` (std_msgs/Float32MultiArray) - AI system status
- `/ai/model_ready` (std_msgs/Bool) - Model ready status

### 3. Safety Monitor Node (`safety_monitor_node.py`)
Monitors system safety and triggers failsafe procedures:
- Altitude and velocity limits
- Battery monitoring
- Obstacle proximity detection
- Communication timeout monitoring
- Emergency landing procedures

**Subscribed Topics:**
- `/ai/observation` (std_msgs/Float32MultiArray) - Current system state
- `/mavros/local_position/pose` (geometry_msgs/PoseStamped) - Drone position
- `/mavros/battery` (sensor_msgs/BatteryState) - Battery status

**Published Topics:**
- `/safety/override` (std_msgs/Bool) - Safety override signal
- `/safety/status` (std_msgs/String) - Safety status messages
- `/failsafe/action` (geometry_msgs/Twist) - Failsafe control commands

### 4. FC Communications Node (`fc_comms_node.py`)
Manages bidirectional MSP communication with INAV 7 flight controller:
- Serial communication with flight controller
- MSP protocol encoding/decoding
- Telemetry data collection (IMU, GPS, status)
- Connection health monitoring and recovery

**Subscribed Topics:**
- `/fc/msp_command` (std_msgs/Float32MultiArray) - Raw MSP commands
- `/fc/rc_override` (std_msgs/Float32MultiArray) - RC channel overrides

**Published Topics:**
- `/fc/imu_raw` (sensor_msgs/Imu) - IMU data from flight controller
- `/fc/gps_fix` (sensor_msgs/NavSatFix) - GPS data from flight controller
- `/fc/attitude` (geometry_msgs/Vector3Stamped) - Attitude (roll, pitch, yaw)
- `/fc/status` (std_msgs/String) - Flight controller status
- `/fc/battery` (sensor_msgs/BatteryState) - Battery status
- `/fc/connected` (std_msgs/Bool) - Connection status
- `/fc/motor_rpm` (std_msgs/Float32MultiArray) - Motor RPM data

### 5. FC Adapter Node (`fc_adapter_node.py`)
Converts AI velocity commands to MSP RC commands with safety validation:
- Velocity to RC channel conversion
- Command rate limiting and smoothing
- Safety limit enforcement
- Failsafe behavior implementation
- Manual override handling

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands from AI
- `/safety/override` (std_msgs/Bool) - Safety override signal
- `/fc/attitude` (geometry_msgs/Vector3Stamped) - Current drone attitude
- `/manual_control` (sensor_msgs/Joy) - Manual RC override

**Published Topics:**
- `/fc/rc_override` (std_msgs/Float32MultiArray) - MSP RC commands
- `/fc/msp_command` (std_msgs/Float32MultiArray) - Raw MSP commands
- `/fc_adapter/status` (std_msgs/String) - Adapter status
- `/fc_adapter/diagnostics` (std_msgs/Float32MultiArray) - Performance metrics

### 6. LiDAR Reader Node (`lidar_reader_node.py`)
Reads distance data from NLink TOFSense LiDAR sensors using proprietary protocol:
- Serial communication with NLink TOFSense sensors
- NLink_TOFSense_Frame0 protocol parsing
- Distance filtering and validation
- Multi-sensor support (front and down configurations)
- Real-time distance measurements at 100Hz

**Published Topics:**
- `/lidar_distance` (sensor_msgs/Range) - Distance measurements
- `/lidar_raw` (std_msgs/Float32MultiArray) - Raw sensor data
- `/lidar_status` (std_msgs/String) - Sensor status and diagnostics
- `/lidar_point` (geometry_msgs/PointStamped) - 3D point in sensor frame
- `/lidar_healthy` (std_msgs/Bool) - Sensor health status

**Dual Sensor Configuration:**
- **Front LiDAR**: `/dev/ttyAMA0` → `/front_lidar/lidar_distance_front`
- **Down LiDAR**: `/dev/ttyUSB1` → `/down_lidar/lidar_distance_down`

## Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select swarm_ai_integration
   source install/setup.bash
   ```

4. **Install Python dependencies:**
   ```bash
   pip install numpy scipy stable-baselines3 pyserial
   ```

## Usage

### Basic Launch

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py model_path:=/path/to/your/model.zip
```

### Launch with Custom Parameters

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
    model_path:=/path/to/model.zip \
    device:=cpu \
    max_velocity:=2.0 \
    max_altitude:=30.0 \
    enable_safety:=true \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    front_lidar_port:=/dev/ttyAMA0 \
    down_lidar_port:=/dev/ttyUSB1 \
    lidar_baud_rate:=921600
```

### Individual Node Launch

Launch nodes individually for debugging:

```bash
# AI Adapter Node
ros2 run swarm_ai_integration ai_adapter_node.py

# AI Flight Node
ros2 run swarm_ai_integration ai_flight_node.py --ros-args -p model_path:=/path/to/model.zip

# Safety Monitor Node
ros2 run swarm_ai_integration safety_monitor_node.py
```

## Configuration

Edit `config/swarm_params.yaml` to customize system parameters:

- **GPS Origin**: Set your local coordinate system origin
- **Safety Limits**: Configure altitude, velocity, and distance limits
- **LiDAR Settings**: Adjust range and resolution parameters
- **Battery Thresholds**: Set low battery warnings

## Topic Remapping

The system is designed to work with standard ROS2 sensor topics. Remap topics as needed for your specific setup:

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
    model_path:=/path/to/model.zip \
    --ros-args \
    -r /lidar/scan:=/your_lidar_topic \
    -r /imu/data:=/your_imu_topic
```

## Integration with Flight Controllers

### PX4/ArduPilot (via MAVROS)
Default configuration works with MAVROS. Ensure MAVROS is running:

```bash
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@127.0.0.1:14557
```

### Custom Flight Controllers
Implement the following interfaces:
- Velocity commands: Subscribe to `/cmd_vel`
- State feedback: Publish to `/mavros/local_position/pose` and `/mavros/local_position/velocity_local`
- Battery status: Publish to `/mavros/battery`

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