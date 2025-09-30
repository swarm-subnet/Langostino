# FC Adapter Setup and Configuration Guide

## Overview

This guide explains how to configure and test the velocity-based FC adapter for the Swarm AI integration with iNav.

## System Architecture

```
AI Model (PPO) → ai_flight_node → fc_adapter_velocity_node → iNav FC
                   (VEL actions)    (PID + MSP)              (MSP_SET_RAW_RC)
                                         ↑
                                    FC Sensors (GPS, IMU, Baro)
```

## Key Understanding: AI Model Output Format

### ActionType.VEL Format
The AI model uses `ActionType.VEL` from gym-pybullet-drones:

```python
action = [direction_x, direction_y, direction_z, magnitude_scale]
```

- **action[0:3]**: Direction vector (NOT normalized by model)
- **action[3]**: Magnitude multiplier [0, 1]
- **Final velocity**: `3.0 m/s * |action[3]| * normalize(action[0:3])`

### Coordinate Frame
**Body Frame** (relative to drone heading):
- `vx`: Forward/backward
- `vy`: Right/left
- `vz`: Up/down
- Yaw is maintained (not commanded by AI)

---

## Step 1: iNav Configuration

### 1.1 Enable MSP on Serial Port

In iNav Configurator:

```
Configuration → Ports
- Select UART port for MSP (e.g., UART1)
- Enable MSP
- Set baud rate: 115200
```

### 1.2 Configure Flight Modes

Set up modes on AUX channels:

```
Modes Tab:
- AUX 1 (Channel 5): ARM
  Range: [1700, 2100]

- AUX 2 (Channel 6): ANGLE mode
  Range: [1700, 2100]

- AUX 3 (Channel 7): ALTHOLD mode
  Range: [1700, 2100]
```

### 1.3 Receiver Configuration

```
Configuration → Receiver
- Receiver Mode: MSP RX (or set to accept MSP_SET_RAW_RC)
- Check "Enable Serial based receiver via MSP"
```

### 1.4 PID Tuning (Baseline)

Start with conservative PID gains:

```
PID Tuning Tab:
Roll:
  P: 40, I: 30, D: 23
Pitch:
  P: 40, I: 30, D: 23
Yaw:
  P: 85, I: 45, D: 0
Level (Horizon/Angle):
  P: 20, I: 15, D: 75
```

### 1.5 GPS Configuration

```
Configuration → GPS
- Protocol: UBLOX (or NMEA depending on GPS)
- Ground Assistance: Enabled
- Auto Config: Enabled
- Auto Baud: Enabled
```

---

## Step 2: ROS 2 Node Configuration

### 2.1 FC Adapter Parameters

Edit `fc_adapter_velocity_node` launch parameters or create a config file:

```yaml
# fc_adapter_config.yaml
fc_adapter_velocity_node:
  ros__parameters:
    # Control loop frequency
    control_rate_hz: 30.0

    # Safety limits (MUST match AI training!)
    max_velocity: 3.0  # m/s
    command_timeout_sec: 1.0

    # PID gains - Horizontal (XY)
    kp_xy: 150.0
    ki_xy: 10.0
    kd_xy: 20.0

    # PID gains - Vertical (Z)
    kp_z: 100.0
    ki_z: 5.0
    kd_z: 15.0

    # RC channel limits (conservative)
    rc_min: 1300
    rc_max: 1700

    # Control mode
    enable_closed_loop: true
    enable_angle_mode: true
    enable_althold_mode: true
```

### 2.2 Launch Configuration

Update your launch file to use the new velocity node:

```python
# swarm_ai_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_ai_integration',
            executable='ai_flight_node',
            name='ai_flight_node',
            parameters=[{
                'model_path': '/path/to/ppo_policy.zip',
                'device': 'cpu',
                'max_velocity': 3.0,
                'obs_dim': 131,
                'action_dim': 4,
            }]
        ),
        Node(
            package='swarm_ai_integration',
            executable='fc_adapter_velocity_node',
            name='fc_adapter_velocity',
            parameters=['/path/to/fc_adapter_config.yaml']
        ),
        Node(
            package='swarm_ai_integration',
            executable='ai_adapter_node',
            name='ai_adapter_node',
        ),
    ])
```

---

## Step 3: Safety Checklist

### Before Flight

- [ ] **GPS Lock**: Wait for 10+ satellites and HDOP < 2.0
- [ ] **iNav Modes**: Verify ANGLE + ALTHOLD are configured
- [ ] **Battery**: Fully charged, voltage monitoring enabled
- [ ] **Failsafe**: Set up RC failsafe in iNav (return to launch or land)
- [ ] **RC Override**: Keep manual transmitter ready for override
- [ ] **Geofence**: Set altitude and distance limits in iNav
- [ ] **Test Area**: Open area, no obstacles, no people

### Safety Override Setup

The system has multiple safety layers:

1. **ROS Safety Override** (`/safety/override` topic):
   ```bash
   ros2 topic pub /safety/override std_msgs/Bool "data: true"  # Disable AI
   ```

2. **Command Timeout**: Automatically hovers if no commands for 1 second

3. **Manual RC**: Keep transmitter on and ready to take over

4. **iNav Failsafe**: Configure in iNav for signal loss

---

## Step 4: Ground Testing

### 4.1 Test MSP Communication

```bash
# Terminal 1: Start FC adapter
ros2 run swarm_ai_integration fc_adapter_velocity_node

# Terminal 2: Check MSP commands are being published
ros2 topic echo /fc/msp_command

# Terminal 3: Verify FC is receiving commands (check iNav Configurator Receiver tab)
```

### 4.2 Test Velocity Command Processing

```bash
# Send test velocity command
ros2 topic pub /ai/action geometry_msgs/Twist "
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  z: 0.0
"

# Monitor RC output
ros2 topic echo /fc/msp_command
```

Expected: RC channels should deviate from 1500 (forward velocity → pitch channel)

### 4.3 Test Closed-Loop Control

```bash
# Monitor velocity error
ros2 topic echo /fc_adapter/velocity_error

# With GPS moving, error should be small (<0.2 m/s)
```

### 4.4 Props-Off Test

**IMPORTANT: Remove propellers before this test!**

1. Arm the drone (manual transmitter)
2. Enable AI control:
   ```bash
   ros2 topic pub /ai/enable std_msgs/Bool "data: true"
   ```
3. Send small velocity commands
4. Verify motors respond correctly:
   - Forward command → motors increase appropriately
   - Right command → motors respond for roll
   - Up command → all motors increase slightly

---

## Step 5: Hover Test (Propellers ON)

### 5.1 Manual Takeoff

1. **Arm drone** via manual RC
2. **Take off manually** to 1-2 meters altitude
3. **Enable ALTHOLD mode** (should hover stable)
4. **Verify GPS velocity** is being published:
   ```bash
   ros2 topic echo /fc/gps_speed_course
   ```

### 5.2 Enable AI Control

```bash
# Enable AI (with AI model running)
ros2 topic pub /ai/enable std_msgs/Bool "data: true"
```

### 5.3 Monitor Behavior

Watch for:
- Stable hover (oscillations < 0.5 m)
- Small velocity commands from AI (< 0.5 m/s initially)
- RC commands in safe range [1300-1700]

### 5.4 Emergency Procedures

If drone becomes unstable:

1. **Immediate**: Disable AI via safety override:
   ```bash
   ros2 topic pub /safety/override std_msgs/Bool "data: true"
   ```

2. **Failsafe**: Take manual RC control

3. **Last resort**: Disarm via RC transmitter

---

## Step 6: PID Tuning

If velocity tracking is poor, tune PID gains iteratively.

### 6.1 Identify Problem

Monitor `/fc_adapter/velocity_error`:

- **Oscillation**: Reduce D gain, then P gain
- **Slow response**: Increase P gain
- **Steady-state error**: Increase I gain (carefully!)
- **Overshoot**: Increase D gain, reduce P gain

### 6.2 Tuning Procedure

Start with horizontal axes (XY):

```yaml
# Conservative starting point
kp_xy: 100.0
ki_xy: 5.0
kd_xy: 15.0

# If too sluggish (slow to reach target velocity):
kp_xy: 150.0  # Increase P

# If oscillating (bouncy, unstable):
kd_xy: 30.0   # Increase D
kp_xy: 100.0  # Reduce P

# If steady-state error (never quite reaches target):
ki_xy: 15.0   # Increase I slowly
```

### 6.3 Live Tuning

Update parameters without restarting:

```bash
ros2 param set /fc_adapter_velocity kp_xy 150.0
ros2 param set /fc_adapter_velocity ki_xy 10.0
ros2 param set /fc_adapter_velocity kd_xy 20.0
```

---

## Step 7: Full Autonomous Flight

Once hover is stable and velocity tracking is good:

### 7.1 Pre-Flight Checklist

- [ ] All ground tests passed
- [ ] Hover test stable for 5+ minutes
- [ ] PID gains tuned
- [ ] GPS lock strong (10+ sats)
- [ ] Battery > 80%
- [ ] Safety override ready
- [ ] Manual RC transmitter ready

### 7.2 Flight Procedure

1. **Manual takeoff** to safe altitude (5+ meters)
2. **Enable ANGLE + ALTHOLD**
3. **Verify hover stability**
4. **Enable AI control**:
   ```bash
   ros2 topic pub /ai/enable std_msgs/Bool "data: true"
   ```
5. **Monitor behavior** for 30 seconds
6. **Gradually increase confidence** if stable

### 7.3 What to Expect

The AI model will:
- Navigate toward the hardcoded goal in observations
- Avoid obstacles detected by LiDAR (if available)
- Maintain stable altitude via ALTHOLD
- Stay within 3 m/s velocity limit

---

## Troubleshooting

### Problem: Drone doesn't respond to AI commands

**Check:**
1. Is `/ai/action` being published?
   ```bash
   ros2 topic hz /ai/action
   ```
2. Is FC adapter receiving commands?
   ```bash
   ros2 topic echo /fc/msp_command
   ```
3. Is iNav receiving MSP commands? (Check Receiver tab in Configurator)
4. Are flight modes enabled? (ANGLE + ALTHOLD aux channels)

### Problem: Drone oscillates/unstable

**Check:**
1. PID gains too high (reduce P and D)
2. GPS velocity noisy (check HDOP, satellite count)
3. Mechanical issues (bent props, loose screws)
4. FC vibration (check accelerometer logs)

### Problem: Velocity tracking error large

**Check:**
1. GPS quality (need 3D fix, low HDOP)
2. Compass calibration (mag calibration in iNav)
3. Coordinate frame transform (verify yaw is correct)
4. PID gains too low (increase P)

### Problem: Commands timing out

**Check:**
1. AI model running? (`ros2 topic hz /ai/observation`)
2. Network latency (if ROS 2 over network)
3. Increase `command_timeout_sec` parameter

---

## Advanced Configuration

### Using Different Speed Limits

If you want to fly slower/faster than training (3 m/s):

**⚠️ WARNING**: Changing speed limits may degrade AI performance!

```yaml
# AI flight node
max_velocity: 2.0  # Slower, safer

# FC adapter
max_velocity: 2.0  # Must match!
```

### Open-Loop Mode (Not Recommended)

For testing without GPS feedback:

```yaml
enable_closed_loop: false
```

This will use direct velocity-to-RC mapping without feedback. **Not safe for autonomous flight!**

### Custom PID Gains per Axis

For asymmetric tuning:

```python
# In fc_adapter_velocity_node.py, modify __init__:
self.pid_x = PIDController(kp=150, ki=10, kd=20)  # Forward
self.pid_y = PIDController(kp=130, ki=8, kd=18)   # Lateral (less aggressive)
self.pid_z = PIDController(kp=100, ki=5, kd=15)   # Vertical
```

---

## Summary

### What the System Does

1. **AI model** outputs direction + magnitude for velocity
2. **ai_flight_node** reconstructs velocity (3 m/s max)
3. **fc_adapter_velocity_node** uses PID to track velocity
4. **iNav** handles low-level stabilization (ANGLE + ALTHOLD)
5. **GPS + IMU** provide velocity feedback for closed-loop control

### Why It Won't Crash

- ✅ **Closed-loop control**: Corrects for drone-specific dynamics
- ✅ **Conservative limits**: RC clamped [1300, 1700]
- ✅ **iNav safety**: ANGLE mode prevents extreme tilts
- ✅ **ALTHOLD**: Automatic altitude management
- ✅ **Obstacle avoidance**: AI trained with LiDAR
- ✅ **Multiple failsafes**: Timeout, safety override, manual RC

### Critical Parameters

| Parameter | Value | Must Match |
|-----------|-------|------------|
| max_velocity | 3.0 m/s | AI training ✅ |
| SPEED_LIMIT | 3.0 m/s | gym-pybullet-drones ✅ |
| control_rate_hz | 30 Hz | FC ctrl_freq ✅ |

---

## Contact & Support

For issues or questions:
- Check logs: `ros2 topic echo /fc_adapter/status`
- Enable debug logging: `export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"`
- Review PID state: Monitor `/fc_adapter/velocity_error`

**Good luck with your first autonomous flight! 🚁**