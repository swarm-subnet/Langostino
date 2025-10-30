# FC Full Loop Simulator - README

## Overview

This simulator tests the **complete closed-loop control system** including `fc_adapter_node`, which the other simulators bypass. It reveals real-world issues like PID interactions with ALT HOLD mode.

## What's Different?

### Previous Simulators (ai_adapter_simulator_node)
```
Sensors → AI Adapter → AI Flight → [SIMULATOR DIRECTLY MOVES DRONE]
                                    ↑ bypasses fc_adapter entirely
```

### This Simulator (fc_full_loop_simulator_node)
```
Sensors → AI Adapter → AI Flight → FC Adapter (PID) → RC Commands
   ↑                                                         ↓
   └───────────── [SIMULATOR with INAV physics] ←───────────┘
```

**Key Features:**
- ✅ Simulates INAV's ALT HOLD mode (throttle RC → climb rate)
- ✅ Motor dynamics (first-order lag, not instant)
- ✅ GPS noise and realistic update rates (1.5 Hz)
- ✅ Barometer drift
- ✅ Velocity estimation noise
- ✅ High-resolution physics (100 Hz)
- ✅ CSV logging of all states and commands

## Installation

1. **Build the package:**
   ```bash
   cd ~/swarm-ros
   colcon build --packages-select swarm_ai_integration
   source install/setup.bash
   ```

## Usage

### Basic Run (with ALT HOLD simulation)
```bash
ros2 launch swarm_ai_integration fc_loop_sim_launch.py
```

### Test WITHOUT ALT HOLD (direct throttle control)
```bash
ros2 launch swarm_ai_integration fc_loop_sim_launch.py enable_althold:=false
```

### Test with REDUCED PID gains (recommended fix)
```bash
ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=20.0
```

### Test with custom goal
```bash
ros2 launch swarm_ai_integration fc_loop_sim_launch.py \
  goal_x:=10.0 \
  goal_y:=10.0 \
  goal_z:=5.0
```

### Test multiple configurations
```bash
# Original settings (likely causes overshoot)
ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=100.0

# Recommended fix
ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=20.0

# Without ALT HOLD
ros2 launch swarm_ai_integration fc_loop_sim_launch.py enable_althold:=false
```

## What to Watch For

### Normal Behavior:
- Drone gradually climbs to 3m altitude
- Smooth horizontal movement toward goal
- Throttle RC stays near 1500 (±50) in ALT HOLD mode
- Motor thrust settles around 14.7N (1.5kg × 9.81)

### Problem Indicators (your real-world issue):
- ⚠️ **Rapid altitude gain** (going way higher than 3m)
- ⚠️ **Throttle RC bouncing between extremes** (1300-1700)
- ⚠️ **Vertical velocity oscillations** (±2 m/s)
- ⚠️ **High motor thrust** (>20N)

## Log Files

Logs are saved to:
```
~/swarm-ros/flight-logs/simulator/fc_loop_sim_YYYYMMDD_HHMMSS.csv
```

### Log Columns:
- `time_sec`, `iteration` - Timing
- `pos_e`, `pos_n`, `pos_u` - True position (ENU, meters)
- `vel_e`, `vel_n`, `vel_u` - True velocity (ENU, m/s)
- `roll`, `pitch`, `yaw` - Attitude (degrees)
- `rc_roll`, `rc_pitch`, `rc_throttle`, `rc_yaw` - RC commands from fc_adapter
- `rc_arm`, `rc_angle`, `rc_althold`, `rc_override`, `rc_rth` - Mode switches
- `motor_thrust`, `target_thrust` - Thrust (Newtons)
- `baro_alt`, `gps_alt` - Sensor altitudes (meters)
- `distance_to_goal`, `distance_traveled` - Navigation metrics

### Analyze Logs:
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load log
df = pd.read_csv('~/swarm-ros/flight-logs/simulator/fc_loop_sim_YYYYMMDD_HHMMSS.csv')

# Plot altitude over time
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(df['time_sec'], df['pos_u'], label='True Altitude')
plt.plot(df['time_sec'], df['baro_alt'], label='Baro Altitude', alpha=0.7)
plt.axhline(y=3.0, color='r', linestyle='--', label='Target (3m)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(df['time_sec'], df['rc_throttle'], label='Throttle RC', color='orange')
plt.axhline(y=1500, color='r', linestyle='--', label='Neutral')
plt.ylabel('RC Value')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(df['time_sec'], df['vel_u'], label='Vertical Velocity', color='green')
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('altitude_analysis.png')
plt.show()
```

## Comparing to Real Flight

### If Simulator Shows Problem:
✅ Your PID gains or ALT HOLD configuration is the issue
- Fix in `swarm_params.yaml` before real flight
- Recommended: reduce `kp_z` from 100 to 20

### If Simulator Works Fine:
⚠️ Something else is different in real flight:
- Check actual INAV ALT HOLD settings
- Check battery voltage (affects motor response)
- Check actual GPS quality during flight
- Check wind conditions
- Review real flight logs from fc_adapter

## Key Parameters

### In fc_loop_sim_launch.py:
- `enable_althold`: true/false - Simulate INAV ALT HOLD mode
- `kp_z`: PID proportional gain for Z-axis (vertical)
- `goal_x`, `goal_y`, `goal_z`: Target position in ENU

### In swarm_params.yaml (fc_adapter_node):
```yaml
fc_adapter_node:
  ros__parameters:
    # PID gains (adjust these!)
    kp_z: 100.0      # Try 20.0 instead
    ki_z: 5.0        # Try 1.0 instead
    kd_z: 15.0       # Try 5.0 instead

    # ALT HOLD mode
    enable_althold_mode: true  # Try false to disable

    # RC limits (reduce range if needed)
    rc_min_value: 1300  # Try 1400
    rc_max_value: 1700  # Try 1600
```

## Troubleshooting

### "No RC commands received"
- Wait 2-5 seconds for all nodes to start
- Check that ai_flight_node is publishing actions
- Check that fc_adapter_node is running (not skipping pre-arm)

### "Module not found: swarm_ai_integration"
```bash
cd ~/swarm-ros
colcon build --packages-select swarm_ai_integration
source install/setup.bash
```

### Simulation unstable/crashes
- Reduce PID gains
- Check that max_climb_rate is reasonable (3.0 m/s)
- Check motor_time_constant (0.1s is realistic)

### Can't find log files
```bash
mkdir -p ~/swarm-ros/flight-logs/simulator
```

## Next Steps After Testing

1. **If altitude control is stable in simulation:**
   - Your configuration is likely good
   - Proceed with cautious real-world testing
   - Start with very short test flight

2. **If altitude control oscillates/diverges:**
   - Reduce PID gains as shown above
   - Test again in simulation until stable
   - Consider disabling ALT HOLD (set `enable_althold_mode: false`)

3. **Compare simulation logs to real flight logs:**
   - Look for differences in RC throttle behavior
   - Check if vertical velocity estimates match
   - Verify GPS quality is similar

## Safety Notes

⚠️ **This is a simulation** - Real flight may still differ due to:
- Wind and turbulence
- GPS quality variations
- Motor response characteristics
- Battery voltage effects
- Sensor calibration

Always test with:
1. Props OFF first (check RC values in logs)
2. Short tether test
3. Gradual altitude increase
4. Emergency stop ready

## Contact

If you find issues or have questions about this simulator, check:
- Simulation logs in `~/swarm-ros/flight-logs/simulator/`
- fc_adapter output for RC commands
- ai_adapter output for observations
- This README for parameter tuning

---

**Generated for swarm-ros project**
**Purpose: Debug altitude control before real flight**
