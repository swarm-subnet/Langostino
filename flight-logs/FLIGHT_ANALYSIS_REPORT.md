# Flight Log Analysis Report
**Date:** 2025-10-20
**Flight:** blackbox_20251020_180749_000

## Executive Summary

**CRITICAL ISSUES IDENTIFIED:**

1. ✗ **Model output is constant regardless of drone position** - AI always commands Northeast (~40-45°)
2. ✗ **West movement failed** - Model commanded East instead of West
3. ✗ **Coordinate frame misalignment** - 25-46° rotation between AI commands and GPS course

---

## Detailed Analysis

### 1. AI Model Behavior by Movement Phase

| Phase | Expected | AI vx (East) | AI vy (North) | AI Heading | Result |
|-------|----------|--------------|---------------|------------|--------|
| **ai-1.txt** (North) | vy << 0 | +0.856 ± 0.019 | -1.000 ± 0.000 | ~40.5° NE | ✓ North component correct |
| **ai-2.txt** (East) | vx >> 0 | +0.847 ± 0.017 | -1.000 ± 0.000 | ~40.3° NE | ✓ East component correct |
| **ai-3.txt** (West) | vx << 0 | +0.855 ± 0.027 | -1.000 ± 0.000 | ~40.5° NE | ✗ **FAILED** - Still East! |
| **ai-4.txt** (N-S) | vy varies | +0.841 ± 0.031 | -1.000 ± 0.000 | ~40.1° NE | Partial |

### Key Observations:

**Problem 1: vy is LOCKED at -1.0**
- Standard deviation = 0.000 across ALL phases
- Model is **always** commanding maximum North velocity
- This suggests the goal position might be stuck

**Problem 2: vx is ALWAYS POSITIVE (East)**
- Even during "West movement", vx = +0.855 (should be negative!)
- vx only varies by ±2% (0.84 to 0.86)
- Model never commanded westward movement

**Problem 3: Actions are nearly identical**
- Despite manually moving drone N/E/W/S using compass
- Model output barely changes (std dev < 0.03)
- Suggests model is not responding to position changes

---

### 2. Coordinate Frame Misalignment

**AI Commanded Heading vs GPS Actual Course:**

| Sample | AI Heading | GPS Course | Difference |
|--------|-----------|------------|------------|
| 1 | 45.0° | 18.8° | 26.2° |
| 2 | 45.0° | 4.2° | 40.8° |
| 3 | 44.4° | 7.0° | 37.4° |
| 4 | 44.4° | 11.5° | 32.9° |
| 5 | 44.4° | 19.4° | 25.0° |
| Average | ~44.7° | ~12.3° | ~32.4° |

**Consistent ~30-45° rotation** between AI commands and actual movement suggests:
- INAV sensor alignment may need adjustment
- Magnetometer calibration offset
- Body frame orientation mismatch

---

### 3. Blackbox Data Summary

**Last 100 AI Actions:**
- vx (East): 0.999 ± 0.004 → Nearly maxed out East
- vy (North): -1.000 ± 0.000 → Locked at max North
- vz (Up): 0.132 ± 0.021 → Slight upward command
- **AI Heading: 45.0° ± 0.1°** → Locked Northeast

**GPS Data (Last 20 samples):**
- Course: 29.3° ± 75.7° → High variance, mostly North
- Speed: 0.307 m/s

---

## Root Cause Analysis

### Issue 1: Goal Position Not Updating

The model's constant output suggests the **relative goal position** in the observation vector is not changing. This could be because:

1. **Goal waypoint is fixed**: If the goal was set to one location and never updated during the four movement phases
2. **Position estimation is not updating**: If the drone's estimated position is stuck, the relative goal stays constant
3. **Observation vector corruption**: Goal position indices [128:131] may not be populated correctly

**Action Required:** Check the goal position in the observation vector during each phase.

### Issue 2: Coordinate Frame Rotation

The ~32° average rotation suggests:

**In INAV Configuration:**
- `align_board_roll` or `align_board_pitch` or `align_board_yaw` may be incorrect
- The flight controller might be mounted rotated relative to the drone body
- Magnetometer declination might be wrong (but this would affect compass, not frame rotation)

**Recommended Fix:**
```
# In INAV CLI, check current alignment:
get align_board

# If rotation is needed (example for 30° yaw rotation):
set align_board_yaw = 300  # or 30, depending on direction
save
```

### Issue 3: Model Not Responding to Position Changes

Since the model output is essentially constant across all four movement phases, the issue is likely:

1. **Goal was never changed**: User moved drone but goal waypoint stayed at original position
2. **Position source is frozen**: GPS or position estimator not updating
3. **Observation vector not updating**: Check if position values [0:3] in observation are changing

---

## Recommendations

### Immediate Actions:

1. **Verify goal position updates**:
   - Add logging to print goal position from observation vector indices [128:131]
   - Confirm goal changes when new waypoint is set

2. **Check position estimation**:
   - Verify GPS position is updating in observation vector [0:3]
   - Check if relative goal [128:131] = goal_world - position_world

3. **Fix coordinate frame rotation**:
   - In INAV CLI: `set align_board_yaw = 320` (try 320° or 40°)
   - Test and adjust based on actual heading error

4. **Test with clear goal changes**:
   - Set goal to North: [0, +5, 3]
   - Set goal to East: [+5, 0, 3]
   - Set goal to West: [-5, 0, 3]
   - Verify model output changes accordingly

### Verification Tests:

1. **Static test**: Place drone, set goal to different quadrants, verify AI output direction changes
2. **Dynamic test**: Move drone while keeping same goal, verify AI adjusts as relative position changes
3. **Compass test**: Rotate drone body, verify heading in observation matches actual compass heading

---

## Data Summary

**Total Samples Analyzed:**
- Blackbox: 1000+ lines
- ai-1.txt: 130 actions (North)
- ai-2.txt: 141 actions (East)
- ai-3.txt: 89 actions (West) ✗
- ai-4.txt: 123 actions (North-South)

**Coordinate System (ENU):**
- X-axis: East (positive = East, negative = West)
- Y-axis: North (positive = North, negative = South)
- Z-axis: Up (positive = Up, negative = Down)

**Compass Convention:**
- 0° = North
- 90° = East
- 180° = South
- 270° = West
