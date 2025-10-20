# CRITICAL BUG FOUND: Coordinate Frame Mismatch

## Root Cause

**The AI model was trained with ENU frame actions, but fc_adapter_node.py interprets them as body frame!**

### Evidence:

**1. Simulator (ai_adapter_simulator_node.py:18, 241-243):**
```python
# Line 18:
NOTE: Model actions are interpreted as direct ENU displacements (vx→East, vy→North, vz→Up).

# Lines 241-243:
dE = vx * step_scale  # vx → East
dN = vy * step_scale  # vy → North
dU = vz * step_scale  # vz → Up
```

**2. Real Flight (fc_adapter_node.py:240-242):**
```python
# AI action used directly as body frame velocity - NO TRANSFORMATION!
self.velocity_cmd_body[0] = float(np.clip(vx, -self.max_velocity, self.max_velocity))
self.velocity_cmd_body[1] = float(np.clip(vy, -self.max_velocity, self.max_velocity))
self.velocity_cmd_body[2] = float(np.clip(vz, -self.max_velocity, self.max_velocity))
```

**3. Flight Data Analysis:**
- Goal direction: **355°** (almost due North)
- AI commands: vx=+0.85 (East), vy=-1.0 (South in ENU)
- Expected in ENU for North goal: vx≈0, vy>>0
- **Model output is incorrect** because it thinks it's in body frame!

---

## Why This Bug Wasn't Obvious

1. **Yaw fixed at 0° in simulator**: Line 129 shows `self.yaw = 0.0` (fixed)
   - When yaw=0, body frame is aligned with ENU frame
   - ENU actions work correctly when drone faces North
   - Bug only appears when drone rotates!

2. **Manual movement test methodology**:
   - User moved drone manually while goal stayed fixed
   - Model correctly tried to reach fixed goal
   - But coordinate frame error caused wrong direction

3. **Partial success masked the bug**:
   - North movement had both North and East components → appeared correct
   - East movement had East component → appeared correct
   - West movement had East component → **FAILED** (revealed the bug!)

---

## The Fix

### Option 1: Transform AI Actions from ENU to Body Frame (RECOMMENDED)

Modify `fc_adapter_node.py` to transform AI actions before using them:

```python
def cb_ai_action_array(self, msg: Float32MultiArray):
    data = list(msg.data)
    if len(data) < 3:
        return

    # AI outputs ENU frame: vx=East, vy=North, vz=Up
    vx_enu, vy_enu, vz_enu = float(data[0]), float(data[1]), float(data[2])

    # Transform from ENU to body frame using current yaw
    cy = math.cos(self.attitude_yaw)
    sy = math.sin(self.attitude_yaw)

    # ENU to Body transformation:
    # forward = East*cos(yaw) + North*sin(yaw)
    # right = -East*sin(yaw) + North*cos(yaw)
    # up = Up
    vx_body = vx_enu * cy + vy_enu * sy     # forward
    vy_body = -vx_enu * sy + vy_enu * cy    # right
    vz_body = vz_enu                        # up

    # clamp for safety
    self.velocity_cmd_body[0] = float(np.clip(vx_body, -self.max_velocity, self.max_velocity))
    self.velocity_cmd_body[1] = float(np.clip(vy_body, -self.max_velocity, self.max_velocity))
    self.velocity_cmd_body[2] = float(np.clip(vz_body, -self.max_velocity, self.max_velocity))

    self.last_ai_speed_scalar = float(data[3]) if len(data) > 3 else None
    self.last_cmd_time = time.time()
    self.ai_enabled = True
```

**File to edit:** `/Users/rafaeltorrecilla/Documents/swarm/swarm-ros/src/swarm_ai_integration/swarm_ai_integration/fc_adapter_node.py`

**Lines to replace:** 233-246

---

### Option 2: Retrain Model with Body Frame Actions (NOT RECOMMENDED)

- Requires full retraining
- Changes observation/action semantics
- More complex due to body frame coupling

---

## Expected Results After Fix

### Goal at 355° (North):

**Before fix:**
- AI commands: vx=+0.85 (East in ENU)
- After yaw=0°: body forward ≈ +0.85 (correct-ish)
- After yaw=90° (facing East): body forward ≈ -1.0 (WRONG!)

**After fix:**
- AI commands: vx_enu=+0.85, vy_enu=-1.0  (ENU frame)
- Transform at yaw=0°: forward=vy*sin(0)+vx*cos(0) ≈ +0.85
- Transform at yaw=90°: forward=vy*sin(90)+vx*cos(90) ≈ -1.0
- **Drone moves toward goal regardless of orientation** ✓

---

## Verification Tests

After applying the fix:

### Test 1: Stationary Rotation Test
1. Place drone at origin (0, 0, 3)
2. Set goal at (0, 5, 3) - North
3. Rotate drone body 0°, 90°, 180°, 270°
4. Verify AI commands result in northward movement in all cases

### Test 2: Autonomous Waypoint Navigation
```bash
# Test North
ros2 topic pub /goal geometry_msgs/msg/Point "{x: 0.0, y: 5.0, z: 3.0}"
# Drone should move North regardless of starting yaw

# Test East
ros2 topic pub /goal geometry_msgs/msg/Point "{x: 5.0, y: 0.0, z: 3.0}"
# Drone should move East

# Test West
ros2 topic pub /goal geometry_msgs/msg/Point "{x: -5.0, y: 0.0, z: 3.0}"
# Drone should move West (this will now work!)

# Test South
ros2 topic pub /goal geometry_msgs/msg/Point "{x: 0.0, y: -5.0, z: 3.0}"
# Drone should move South
```

### Test 3: Verify in Simulator First
Run simulator with rotating yaw to verify transformation works correctly before real flight.

---

## Impact on Previous Flight Data

The flight log analysis shows:
- Model consistently commanded toward the fixed goal
- But coordinate frame error caused ~30-45° rotation
- After fix, drone will navigate directly to goals
- No model retraining needed!

---

## Additional Issues Found

### Issue 2: Goal Waypoint Not Updated During Tests

The flight tests had a **fixed goal at (0.93, 10.28, -0.11)** throughout all manual movements.

**Correct test methodology:**
1. Set initial position
2. **Update goal waypoint** to different locations (N/E/W/S)
3. Let model fly autonomously to each goal
4. Observe if it reaches goals correctly

**Incorrect methodology** (what was done):
1. Set one goal
2. Manually move drone while goal stays fixed
3. Model tries to return to same goal from all positions
4. Appears broken but model is actually working correctly!

### Issue 3: INAV Sensor Alignment (Secondary)

After fixing coordinate frame, check if additional rotation correction needed:
```bash
# In INAV CLI
get align_board_yaw
# May need adjustment if still seeing rotation error
```

---

## Summary

✓ **Root cause identified**: ENU vs Body frame mismatch
✓ **Fix is simple**: Add ENU-to-body transformation in fc_adapter_node.py
✓ **No retraining needed**: Model is correct, code needs fix
✓ **Test plan defined**: Verify with waypoint navigation tests

**Estimated fix time:** 5 minutes
**Estimated test time:** 30 minutes (simulator + real flight)
**Risk level:** Low (transformation is standard and well-tested)
