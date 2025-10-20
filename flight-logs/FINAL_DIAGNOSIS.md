# Flight Test Diagnosis - FINAL REPORT

## Critical Finding

**The goal waypoint was FIXED during all manual movement tests!**

---

## What Actually Happened

### Goal Position:
- **Fixed at:** (0.93, 10.28, -0.11) meters in ENU coordinates
- **Changed only 2 times** during entire flight (likely initial setup)
- **Standard deviation:** < 0.1m in all axes during manual tests

### Drone Movement:
- User manually moved drone North/East/West/South using compass
- Drone position changed from (0.17, -2.12) to (2.49, 2.12) meters
- **BUT:** Goal waypoint was never updated!

### AI Model Behavior:
- **Model is working correctly!** ✓
- Model consistently tried to navigate toward the fixed goal
- Average heading to goal: **354.9°** (almost due North)
- Model output: vx ≈ 0.85, vy = -1.0 (consistent across all tests)

---

## The Problem: Coordinate Frame Error

### Expected vs Actual:

**Goal Direction:**
- Relative position to goal: (-0.66m, +10.67m, ...)
- Expected heading: **355°** (almost due North, slightly West)

**AI Commands:**
- vx = +0.85 (commanding EAST)
- vy = -1.0 (commanding... direction unclear)
- Calculated heading from AI: **~45°** (Northeast)

**Actual GPS Movement:**
- GPS course: **~12-30°** (North-Northeast)

### The Mismatch:

| Direction | Angle | Analysis |
|-----------|-------|----------|
| Goal direction | 355° | Almost due North |
| AI commanded | 45° | Northeast |
| GPS actual | 12-30° | North-Northeast |
| AI vs Goal | **90° error!** | AI commands perpendicular to goal! |
| GPS vs Goal | ~30° error | Drone moves northeast instead of north |

---

## Root Cause Analysis

### Issue 1: AI Model Output Coordinate Frame Error

**The model's vx/vy outputs are NOT in ENU frame as expected!**

Evidence:
1. Goal is at 355° (North), but model commands vx=+0.85, vy=-1.0
2. If ENU: vx=East, vy=North, then model output = atan2(0.85, -1.0) = 40° = Northeast
3. But goal is North (355°), so model is **90° off!**

**Possible causes:**
- Model was trained in **body frame** (vx=forward, vy=left) but being applied in **ENU frame**
- Sign convention error in coordinate transformation
- Model outputs in NED frame (North-East-Down) instead of ENU (East-North-Up)

### Issue 2: Flight Controller Coordinate Rotation

Even with the model error, there's a second rotation:
- AI commands ~45° (Northeast)
- GPS shows ~12-30° (North-Northeast)
- Additional **~15-30° rotation** in INAV or sensor alignment

---

## Why Manual Tests "Failed"

### Test 1: North Movement (ai-1.txt)
- User moved drone North manually
- Goal stayed at 355° (North) from drone
- Model commanded 45° (Northeast) - **WRONG**
- But has North component, so appeared "partially correct"

### Test 2: East Movement (ai-2.txt)
- User moved drone East manually
- Goal still at 355° (North) from drone
- Model commanded 45° (Northeast) - **WRONG**
- But has East component, so appeared "partially correct"

### Test 3: West Movement (ai-3.txt) ❌
- User moved drone West manually
- Goal still at 355° (North) from drone
- Model commanded 45° (Northeast) - **WRONG**
- Should have commanded West, so **OBVIOUSLY FAILED**

### Test 4: North-South Movement (ai-4.txt)
- User moved drone North then South
- Goal still at 355° (North) from drone
- Model commanded 45° (Northeast) - **WRONG**
- Showed model is not responding to position changes

---

## Required Fixes

### Fix 1: Correct Coordinate Frame in ai_flight_node.py

The model output is likely being misinterpreted. Check `/ai/action` subscriber in `ai_flight_node.py`:

**Hypothesis A:** Model trained in body frame
- Current: Treating vx as ENU-East, vy as ENU-North
- Fix: Transform from body frame:
  ```python
  # If model outputs body frame (vx=forward, vy=left):
  enu_vx = vx * cos(yaw) - vy * sin(yaw)
  enu_vy = vx * sin(yaw) + vy * cos(yaw)
  ```

**Hypothesis B:** Sign convention error
- Current: vy = -1.0 interpreted as South
- Fix: Check if vy should be negated or axes swapped

### Fix 2: INAV Sensor Alignment

After fixing coordinate frame, check INAV alignment:
```bash
# In INAV CLI
get align_board_yaw
# If needed, rotate to match:
set align_board_yaw = XXX  # Adjust based on testing
save
```

### Fix 3: Test Protocol

**Correct way to test different directions:**

Instead of manually moving drone while goal is fixed, **update the goal waypoint**:

```bash
# Test North: Set goal north of starting position
ros2 topic pub /goal geometry_msgs/msg/Point "{x: 0.0, y: 5.0, z: 3.0}"

# Test East: Set goal east of starting position
ros2 topic pub /goal geometry_msgs/msg/Point "{x: 5.0, y: 0.0, z: 3.0}"

# Test West: Set goal west of starting position
ros2 topic pub /goal geometry_msgs/msg/Point "{x: -5.0, y: 0.0, z: 3.0}"

# Test South: Set goal south of starting position
ros2 topic pub /goal geometry_msgs/msg/Point "{x: 0.0, y: -5.0, z: 3.0}"
```

Then let the model fly autonomously to each goal.

---

## Immediate Action Items

1. **Check coordinate frame transformation** in `ai_flight_node.py:427-440` where model output is received
2. **Review training data** to confirm what coordinate frame model expects
3. **Add debug logging** to print:
   - Current position (x,y,z)
   - Goal position (x,y,z)
   - Relative goal vector
   - Expected heading to goal
   - Model output (vx, vy, vz)
   - Calculated heading from model output
4. **Compare** expected vs actual headings to diagnose the rotation
5. **Fix coordinate transformation** based on findings
6. **Re-test** with proper goal waypoint updates

---

## Summary

✗ **The test methodology was flawed** - goal should have been updated, not drone manually moved
✓ **Model is working** - it consistently navigates toward the goal (when goal is fixed)
✗ **Major coordinate frame error** - 90° rotation between goal direction and model output
✗ **Additional INAV rotation** - ~30° rotation between commanded and actual movement

**Next steps:** Fix coordinate frame transformation, then retest with proper goal waypoints.
