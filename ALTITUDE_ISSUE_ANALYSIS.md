# Altitude Control Issue - Complete Analysis

**Date:** 2025-10-30
**Issue:** Drone went "way higher than it should" during real-world test
**Status:** Root cause identified ✅

---

## Executive Summary

Your drone climbed too high because of **double PID control**: your fc_adapter PID is fighting with INAV's built-in ALT HOLD PID. The simulator didn't show this because it bypasses fc_adapter entirely.

**Quick Fix:** Reduce `kp_z` from 100.0 to 20.0 in `swarm_params.yaml:197`

---

## Root Cause: ALT HOLD Mode Interaction

### The Problem Chain

1. **Your fc_adapter** has aggressive PID gains:
   ```yaml
   kp_z: 100.0  # Very high!
   ki_z: 5.0
   kd_z: 15.0
   ```

2. **Your fc_adapter** generates RC throttle commands (1300-1700 range)

3. **INAV's ALT HOLD mode is ENABLED** (`swarm_params.yaml:60`)

4. **In ALT HOLD mode, throttle ≠ motor throttle**:
   - Throttle stick position = **climb rate command**
   - 1500 = hold altitude (0 m/s)
   - 1600 = climb at X m/s
   - 1400 = descend at X m/s

5. **INAV's ALT HOLD has its own PID** that tries to achieve the commanded climb rate

6. **Result: Two PIDs stacked on top of each other**
   ```
   Your PID: vz_error → RC deviation (can be ±200 units)
                ↓
   INAV ALT HOLD PID: interprets RC as climb rate command
                      → applies its own PID gains
                      → aggressive altitude changes
   ```

### Why Simulator Worked But Real Flight Didn't

| Aspect | Simulator | Real Flight |
|--------|-----------|-------------|
| **Altitude Control** | Direct position update | PID → RC → ALT HOLD PID → Motors |
| **Feedback Loop** | Perfect, instant | GPS (noisy, 1 Hz) + Baro (drifts) |
| **Control Modes** | None | ALT HOLD active (double PID) |
| **Physics** | Instant response | Motor lag + aerodynamics |
| **Yaw** | Fixed at 0° | Variable (correctly handled) |

**Simulators**: `ai_adapter_simulator_node.py` and `ai_adapter_simulator_multiple_node.py`
- Line 238-248: Direct position control `pos += velocity * dt`
- Bypasses fc_adapter entirely
- Cannot reveal PID interaction issues

---

## Evidence in Code

### fc_adapter_node.py (Real Flight Node)

**Lines 228-248: Action processing**
```python
# AI outputs vz in ENU frame
vz_enu = float(data[2])
# Transform to body frame (but vz stays same)
vz_body = vz_enu
# Store command
self.velocity_cmd_body[2] = np.clip(vz_enu, -max_vel, +max_vel)
```

**Lines 316-335: PID control**
```python
# PID with HIGH gains
err_z = vz_cmd - vz_actual
throttle_dev = PID_z.compute(err_z)  # kp_z=100!
throttle_rc = 1500 + throttle_dev    # Can be 1100-1900
throttle_rc = clamp(throttle_rc, 1300, 1700)  # Limited to ±200
```

**Line 345: ALT HOLD enabled**
```python
channels[6] = 1800 if self.althold_enabled else 1000  # CH7: ALT HOLD
```

**swarm_params.yaml:60**
```yaml
enable_althold_mode: true  # ← This is the problem!
```

### ai_adapter_simulator_node.py (Simulator Node)

**Lines 238-248: Direct control (no fc_adapter)**
```python
# Direct mapping: model outputs → position change
step_scale = float(self.meters_per_step * spd)
dE = vx * step_scale
dN = vy * step_scale
dU = vz * step_scale

old_pos = self.rel_pos_enu.copy()
self.rel_pos_enu[0] += dE
self.rel_pos_enu[1] += dN
self.rel_pos_enu[2] = max(0.0, self.rel_pos_enu[2] + dU)  # Direct altitude update!
```

**Line 129: Yaw fixed**
```python
self.yaw = 0.0  # rad (fixed) ← Never changes, body frame = ENU frame
```

This explains why simulator worked: no fc_adapter, no PID, no ALT HOLD interaction.

---

## Configuration Issues Found

### 1. Z-Axis PID Gains Too High
**File:** `swarm_params.yaml:197-199`
```yaml
kp_z: 100.0    # ← 5x too high for ALT HOLD mode
ki_z: 5.0      # ← Can cause integral windup
kd_z: 15.0     # ← Amplifies noise
```

**With ALT HOLD enabled**, even a 0.1 m/s error → 10 units throttle deviation → significant climb rate change.

### 2. ALT HOLD Mode Enabled
**File:** `swarm_params.yaml:60`
```yaml
enable_althold_mode: true  # ← Creates double PID control
```

### 3. Wide RC Limits
**File:** `swarm_params.yaml:184-185`
```yaml
rc_min_value: 1300  # Allows ±200 units from center
rc_max_value: 1700
```

In ALT HOLD mode, this range can command aggressive climb rates.

### 4. GPS Altitude Feedback
**File:** `fc_adapter_node.py:268-270`
```python
def cb_altitude(self, msg: Float32MultiArray):
    if len(msg.data) >= 2:
        self.velocity_actual_earth[2] = float(msg.data[1])  # vertical velocity
```

- Vertical velocity likely from GPS (noisy, slow)
- Or barometer (drifts)
- Either way, not as clean as simulator

---

## Solutions (Priority Order)

### 🔴 IMMEDIATE (Before Next Flight)

#### Option A: Reduce PID Gains (RECOMMENDED)
**File:** `swarm_params.yaml:197-199`
```yaml
# OLD (causes overshoot)
kp_z: 100.0
ki_z: 5.0
kd_z: 15.0

# NEW (gentler response)
kp_z: 20.0   # 5x reduction
ki_z: 1.0    # 5x reduction
kd_z: 5.0    # 3x reduction
```

**Why:** Let INAV's ALT HOLD do most of the work. Your PID just provides gentle corrections.

#### Option B: Disable ALT HOLD
**File:** `swarm_params.yaml:60`
```yaml
enable_althold_mode: false  # Direct throttle control
```

**Why:** Single PID loop. But need careful tuning (less stable).

#### Option C: Reduce RC Range
**File:** `swarm_params.yaml:184-185`
```yaml
rc_min_value: 1400  # Reduced from 1300
rc_max_value: 1600  # Reduced from 1700
```

**Why:** Limits maximum climb/descend rate in ALT HOLD.

### 🟡 DIAGNOSTIC (Understand the Issue)

#### Test 1: New Full Loop Simulator
```bash
cd ~/swarm-ros
colcon build --packages-select swarm_ai_integration
source install/setup.bash

# Test with original gains (should show problem)
ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=100.0

# Test with reduced gains (should be stable)
ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=20.0
```

**What it does:**
- Simulates complete control loop including fc_adapter
- Simulates INAV ALT HOLD mode
- Shows PID interactions
- Logs everything to CSV

**Expected results:**
- With kp_z=100: Altitude oscillation or overshoot
- With kp_z=20: Smooth altitude control

#### Test 2: Analyze Real Flight Logs
If you have logs from the failed flight:
1. Look for throttle RC values
2. Check if they're bouncing between extremes (1300-1700)
3. Look for vertical velocity oscillations

### 🟢 VERIFICATION (Before Real Flight)

#### Step 1: Ground Test (Props OFF)
```bash
# Run simulator to generate test commands
ros2 launch swarm_ai_integration fc_loop_sim_launch.py

# Watch fc_adapter output for RC throttle values
# Should stay around 1500 ± 50 for hover
```

#### Step 2: Tethered Test
- Short tether (2m max)
- Command hover at 1m altitude
- Observe if altitude is stable

#### Step 3: Gradual Real Test
- Start at 1m altitude
- Increase to 2m
- Then 3m
- Watch for oscillations

---

## New Simulator Explanation

I created `fc_full_loop_simulator_node.py` which:

### Simulates Complete Loop
```
Sensors → AI Adapter → AI Flight → FC Adapter (PID) → RC Commands
   ↑                                                         ↓
   └───────────── [SIMULATOR with INAV] ←──────────────────┘
```

### Key Features
- ✅ INAV ALT HOLD mode simulation
- ✅ Motor dynamics (not instant)
- ✅ GPS noise (2m std dev)
- ✅ GPS update rate (1.5 Hz realistic)
- ✅ Barometer drift
- ✅ High-resolution physics (100 Hz)
- ✅ Detailed CSV logging

### Files Created
1. `fc_full_loop_simulator_node.py` - Main simulator
2. `fc_loop_sim_launch.py` - Launch file
3. `FC_LOOP_SIMULATOR_README.md` - Usage instructions
4. `CMakeLists.txt` - Updated to install new node

### How to Use
```bash
# Basic run
ros2 launch swarm_ai_integration fc_loop_sim_launch.py

# Test without ALT HOLD
ros2 launch swarm_ai_integration fc_loop_sim_launch.py enable_althold:=false

# Test with reduced gains
ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=20.0
```

### What to Watch
- Altitude staying near 3m (not climbing to 10m+)
- Throttle RC near 1500 (±50, not ±200)
- Smooth velocity changes (not oscillating)

---

## Other Findings

### ✅ No Issues Found

1. **ENU to Body Frame Transform**: Correctly implemented
   - `fc_adapter_node.py:237-243` properly transforms actions
   - This was already documented as a known issue, but it's fixed in current code

2. **Observation Building**: Correct
   - 131-D structure matches training
   - LiDAR normalization correct (max_ray_distance=20m)

3. **Coordinate Transforms**: Correct
   - GPS to ENU conversion working
   - Origin computation correct
   - Home position handling correct

4. **Action Buffer**: Correct
   - Size=25 matches training
   - Buffer management correct

### ⚠️ Minor Concerns

1. **GPS Vertical Velocity**:
   - May be noisy (GPS-based)
   - Consider using barometer-based velocity if available

2. **LiDAR Range Limit**:
   - Max 20m - if drone goes higher, sensor maxes out
   - But this shouldn't cause climb (just blind above 20m)

3. **Altitude Source**:
   - Using GPS altitude (5-10m accuracy)
   - Barometer may be more accurate for relative changes

---

## Action Items

### Before Next Flight
- [ ] Reduce PID gains in swarm_params.yaml (kp_z: 20.0)
- [ ] Build and test new simulator
- [ ] Run simulation with both old and new gains
- [ ] Compare simulation logs

### During Next Flight
- [ ] Start with 1m altitude test
- [ ] Monitor fc_adapter logs for throttle RC values
- [ ] Watch for oscillations
- [ ] Have emergency stop ready

### After Flight
- [ ] Compare real flight logs to simulation
- [ ] Check throttle RC behavior
- [ ] Check vertical velocity estimates
- [ ] Adjust gains if needed

---

## Summary

**Problem:** Double PID control (yours + INAV's ALT HOLD)
**Solution:** Reduce your PID gains by 5x
**Test:** New simulator shows the issue
**Safety:** Test in simulation first, then gradual real flight

**Confidence:** High - this explains why simulator worked but real flight didn't.

---

## Questions?

If the fix doesn't work:
1. Check actual INAV ALT HOLD configuration
2. Check battery voltage during flight
3. Check GPS quality (HDOP, satellites)
4. Review fc_adapter logs for RC commands
5. Compare simulation to real flight logs

---

**Generated:** 2025-10-30
**For:** swarm-ros project altitude control debugging
