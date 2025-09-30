# Swarm AI - Quick Reference Card

## Pre-Flight Checklist

```
□ GPS Lock: 10+ satellites, HDOP < 2.0
□ Battery: >80%, voltage monitoring ON
□ iNav Modes: ANGLE + ALTHOLD configured
□ RC Transmitter: Ready for manual override
□ Safety Override: Topic ready
□ Failsafe: Configured in iNav
□ Test Area: Open, no obstacles, no people
□ Logs: Recording enabled
```

## Launch Commands

```bash
# Full system
ros2 launch swarm_ai_integration swarm_ai_launch.py

# Individual nodes
ros2 run swarm_ai_integration ai_adapter_node
ros2 run swarm_ai_integration ai_flight_node
ros2 run swarm_ai_integration fc_adapter_velocity_node
```

## Control Commands

```bash
# Enable AI
ros2 topic pub /ai/enable std_msgs/Bool "data: true"

# EMERGENCY: Disable AI
ros2 topic pub /safety/override std_msgs/Bool "data: true"

# Test velocity (0.5 m/s forward)
ros2 topic pub /ai/action geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

## Monitoring

```bash
# Status
ros2 topic echo /fc_adapter/status

# Velocity error (should be <0.2 m/s)
ros2 topic echo /fc_adapter/velocity_error

# RC commands being sent
ros2 topic echo /fc/msp_command

# AI model running?
ros2 topic hz /ai/observation
```

## Key Parameters

| Parameter | Value | Why |
|-----------|-------|-----|
| max_velocity | 3.0 m/s | Matches training |
| control_rate_hz | 30 Hz | Matches FC ctrl freq |
| rc_min / rc_max | 1300 / 1700 | Conservative limits |
| kp_xy | 150.0 | Start conservative |
| command_timeout | 1.0 sec | Safety |

## AI Action Format (VEL)

```python
action = [dir_x, dir_y, dir_z, magnitude]
velocity = 3.0 * |magnitude| * normalize([dir_x, dir_y, dir_z])
```

**Coordinate Frame**: Body (forward, right, up)

## Emergency Procedures

### 1. Unstable Flight
```bash
# Disable AI immediately
ros2 topic pub /safety/override std_msgs/Bool "data: true"
```
Then take manual RC control.

### 2. Command Timeout
System automatically sends hover command after 1 second.

### 3. Total System Failure
Use manual RC transmitter to land.

## Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| No AI commands | Check `/ai/observation` publishing |
| Oscillating | Reduce `kp_xy` by 30% |
| Slow response | Increase `kp_xy` by 20% |
| Large vel error | Check GPS lock, increase `kp_xy` |
| Commands timeout | Check network, increase timeout |
| RC not responding | Check iNav serial port, MSP enabled |

## PID Tuning Quick Guide

```yaml
# Oscillating? (bouncy, unstable)
kp_xy: 100.0  # ⬇ Reduce P
kd_xy: 30.0   # ⬆ Increase D

# Too slow? (sluggish response)
kp_xy: 180.0  # ⬆ Increase P

# Steady-state error? (never reaches target)
ki_xy: 15.0   # ⬆ Increase I (carefully!)
```

Update live without restart:
```bash
ros2 param set /fc_adapter_velocity kp_xy 150.0
```

## Status Indicators

### Normal Operation
```
AI_ENABLED | CLOSED_LOOP
```

### Safety Active
```
SAFETY_OVERRIDE | CLOSED_LOOP
```

### Warning
```
AI_ENABLED | CMD_TIMEOUT | CLOSED_LOOP
```

## iNav Configuration Quick Check

```
Receiver Tab:
- All channels show values
- THROTTLE at ~1500 when hover cmd sent
- ROLL/PITCH deviate with velocity commands

Modes Tab:
- AUX1 (Ch5): ARM at 1800
- AUX2 (Ch6): ANGLE at 1800
- AUX3 (Ch7): ALTHOLD at 1800

Configuration → Ports:
- UART with MSP enabled
- Baud rate: 115200
```

## Flight Sequence

1. **Manual Arm** (RC transmitter)
2. **Manual Takeoff** to 2m
3. **Enable ALTHOLD** (verify stable hover)
4. **Enable AI**:
   ```bash
   ros2 topic pub /ai/enable std_msgs/Bool "data: true"
   ```
5. **Monitor for 30 seconds**
6. **If stable, continue**
7. **If unstable, DISABLE AI and land**

## Safety Margins

| Parameter | Min | Max | Safe Range |
|-----------|-----|-----|------------|
| Velocity | 0 | 3.0 m/s | 0-2.0 m/s initially |
| RC channels | 1000 | 2000 | 1300-1700 |
| Tilt angle | 0° | 30° | <15° |
| Altitude | 0 | unlimited | 2-10m for testing |

## Log Files

```bash
# ROS 2 logs
~/.ros/log/

# Enable debug logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
```

## Contact Info & Resources

- **Setup Guide**: `FC_ADAPTER_SETUP_GUIDE.md`
- **Implementation Summary**: `IMPLEMENTATION_SUMMARY.md`
- **Code Location**: `src/swarm_ai_integration/`

---

**Remember**: Keep manual RC transmitter ready at all times! 🚁