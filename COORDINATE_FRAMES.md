# Coordinate Frame Reference for Swarm AI Integration

## Overview

This document explains the coordinate frame conventions used in the swarm AI drone project and how transformations are handled between the training environment (PyBullet) and the ROS2 deployment system.

## Coordinate Systems

### 1. Body Frame (Training Environment - PyBullet)

The AI model was trained using **Body Frame** coordinates where axes are relative to the drone's orientation:

```
         ^ Body-Z (Up)
         |
         |
    Body-X (Forward)
        /
       /
      --------> Body-Y (Right)
```

- **Body-X (vx)**: Forward/Backward along the drone's nose direction
- **Body-Y (vy)**: Right/Left perpendicular to the nose (starboard/port)
- **Body-Z (vz)**: Up/Down relative to the drone's top

**Model Outputs:**
- `vx`: Forward/backward velocity command (Body-X)
- `vy`: Right/left velocity command (Body-Y)
- `vz`: Up/down velocity command (Body-Z)
- `speed`: Scalar speed multiplier

### 2. ENU Frame (ROS2 World Frame)

The ROS2 system uses **ENU (East-North-Up)** as the world reference frame:

```
      ^ North (ENU-Y)
      |
      |
      --------> East (ENU-X)
     /
    /
   v Up (ENU-Z)
```

- **East (E)**: Positive X-axis (eastward displacement)
- **North (N)**: Positive Y-axis (northward displacement)
- **Up (U)**: Positive Z-axis (altitude above ground)

## Critical Transformation: Body → ENU

### The Problem

When deploying the trained model in ROS2, the body-frame velocities must be transformed to ENU frame before integration. **The axes do NOT align even when yaw=0!**

### PyBullet Body Frame Convention (yaw=0)

When the drone has zero yaw angle in PyBullet:
- **Body-X (forward)** → **ENU-North** (NOT East!)
- **Body-Y (right)** → **ENU-East** (NOT North!)
- **Body-Z (up)** → **ENU-Up**

This is equivalent to a **-90° rotation** around the Z-axis compared to the standard "forward=East" convention.

### Transformation Equations

For a drone at yaw angle `θ` (in radians), the transformation from body velocities `(vx, vy, vz)` to ENU displacements is:

```python
E = vx * sin(θ) + vy * cos(θ)
N = vx * cos(θ) - vy * sin(θ)
U = vz
```

**Special case: yaw = 0**
```python
E = vy   # right → East
N = vx   # forward → North
U = vz   # up → Up
```

**Special case: yaw = 90° (π/2 rad)**
```python
E = vx   # forward → East
N = -vy  # left → North
U = vz   # up → Up
```

## Implementation

### Training Environment (`swarm/core/moving_drone.py`)

Ray directions are defined in body frame (lines 116-138):
```python
[1, 0, 0]          # Forward (Body-X)
[0, 1, 0]          # Right (Body-Y)
[-1, 0, 0]         # Back (Body-X negative)
[0, -1, 0]         # Left (Body-Y negative)
```

The model learns to navigate using these body-frame references.

### ROS2 Simulator (`ai_adapter_simulator_node.py`)

The corrected physics tick applies the transformation (lines 269-282):

```python
# Apply body-to-ENU rotation
cos_yaw = math.cos(self.yaw)
sin_yaw = math.sin(self.yaw)

# Transform body velocities to ENU frame
v_east = vx * sin_yaw + vy * cos_yaw
v_north = vx * cos_yaw - vy * sin_yaw
v_up = vz

# Scale by meters_per_step and speed
step_scale = float(self.meters_per_step * spd)
dE = v_east * step_scale
dN = v_north * step_scale
dU = v_up * step_scale
```

### Real Flight Controller Integration

When integrating with actual flight controllers (e.g., via MSP), ensure:

1. The flight controller's body frame matches PyBullet's convention
2. Apply the same body→ENU transformation before sending position/velocity commands
3. If the FC uses a different convention (e.g., NED), additional transformations may be needed

## Verification

### Expected Behavior (Corrected)

With the goal at ENU position `(5, 5, 3)` and starting at `(0, 0, 3)`:

**Model Output:**
```
vx=+0.064  (forward)
vy=+0.128  (right)
vz=+0.967  (up)
```

**After Transformation (yaw=0):**
```
E = vy = +0.128  → moves East (correct!)
N = vx = +0.064  → moves North (correct!)
U = vz = +0.967  → moves Up (correct!)
```

The drone should move toward the goal at (5, 5, 3).

### Previous Behavior (Incorrect)

**Before the fix**, the simulator directly used:
```python
dE = vx  # WRONG: forward → East
dN = vy  # WRONG: right → North
dU = vz
```

This caused the drone to move in the wrong direction.

## Debugging Frame Issues

If you observe unexpected movement:

1. **Check the yaw angle**: Ensure yaw=0 is correctly oriented
2. **Verify the transformation**: Print intermediate values (v_east, v_north)
3. **Log both frames**: Compare body-frame actions with ENU displacements
4. **Test cardinal directions**: Command pure forward (vx=1, vy=0), observe ENU motion
5. **Check ray casting**: Ensure obstacle rays use the same body frame as actions

## References

- PyBullet Documentation: https://pybullet.org/
- ROS REP 103 (Standard Units and Coordinate Conventions): https://www.ros.org/reps/rep-0103.html
- Swarm Environment: `swarm/core/moving_drone.py`
- ROS2 Simulator: `swarm-ros/src/swarm_ai_integration/swarm_ai_integration/ai_adapter_simulator_node.py`

## Summary

**Key Takeaway:** The AI model outputs velocities in **Body Frame**, which must be rotated by the yaw angle to obtain **ENU Frame** displacements. The PyBullet body frame has Body-X pointing North (not East) when yaw=0, requiring the transformation E=vy, N=vx.

**Files Modified:**
- `ai_adapter_simulator_node.py`: Added body→ENU transformation (lines 269-282)

**Date:** 2025-10-10
**Issue:** Frame mismatch causing incorrect drone navigation
**Resolution:** Implemented proper body→ENU coordinate transformation
