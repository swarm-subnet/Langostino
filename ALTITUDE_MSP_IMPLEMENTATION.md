# MSP_ALTITUDE Implementation Summary

## Overview
Implemented MSP_ALTITUDE (code=109) reading from the flight controller to provide barometer-based altitude and vertical velocity data through the `/fc/altitude` topic.

## Problem
`fc_adapter_node.py` was subscribed to `/fc/altitude` (line 174) but no data was being published because `fc_comms_node.py` wasn't requesting or handling MSP_ALTITUDE messages from the flight controller.

## Solution
Added complete MSP_ALTITUDE support to the telemetry pipeline.

---

## Files Modified

### 1. `msp_protocol.py`
**Added:** `unpack_altitude()` method to `MSPDataTypes` class

**Location:** Line 399-429

**Functionality:**
- Unpacks MSP_ALTITUDE (6 bytes)
- Returns dictionary with:
  - `altitude_m`: Barometer altitude in meters (converted from cm)
  - `vario`: Vertical velocity in m/s (converted from cm/s)

**MSP_ALTITUDE Format:**
```
Byte 0-3: int32  estimated_altitude (cm)
Byte 4-5: int16  vertical_velocity (cm/s)
Total: 6 bytes
```

### 2. `fc_comms_node.py`
**Modified:** Added MSP_ALTITUDE to telemetry polling and handling

**Changes:**

a) **Line 85** - Added to telemetry request sequence:
```python
self.telemetry_commands = [
    MSPCommand.MSP_RAW_IMU,
    MSPCommand.MSP_RAW_GPS,
    MSPCommand.MSP_ATTITUDE,
    MSPCommand.MSP_ALTITUDE,  # ← NEW
    MSPCommand.MSP_STATUS,
    MSPCommand.MSP_ANALOG,
    MSPCommand.MSP_MOTOR
]
```

b) **Line 206-207** - Added handler in `handle_msp_response()`:
```python
elif message.command == MSPCommand.MSP_ALTITUDE:
    self._handle_altitude(message.data)
```

c) **Line 267-271** - Added handler method:
```python
def _handle_altitude(self, data: bytes):
    """Handle altitude data"""
    altitude_msg = self.parser.parse_altitude_data(data)
    if altitude_msg:
        self.publisher.publish_altitude(altitude_msg)
```

### 3. `utils/msp_message_parser.py`
**Added:** `parse_altitude_data()` method

**Location:** Line 190-217

**Functionality:**
- Parses MSP_ALTITUDE raw bytes using `MSPDataTypes.unpack_altitude()`
- Creates `Float32MultiArray` message with format: `[altitude_m, vario_m/s]`
- Returns `None` on error

### 4. `utils/telemetry_publisher.py`
**Modified:** Added altitude publisher

**Changes:**

a) **Line 57** - Added to telemetry storage:
```python
self.last_telemetry: Dict[str, Any] = {
    'imu': None,
    'gps': None,
    'attitude_euler': None,
    'altitude': None,  # ← NEW
    'status': None,
    'battery': None
}
```

b) **Line 91-94** - Created publisher in `_create_publishers()`:
```python
# Altitude data (barometer altitude + vertical velocity)
self.altitude_pub = self.node.create_publisher(
    Float32MultiArray, '/fc/altitude', self.sensor_qos
)
```

c) **Line 186-197** - Added publish method:
```python
def publish_altitude(self, altitude_msg: Float32MultiArray):
    """Publish altitude data (barometer altitude + vertical velocity)"""
    self.altitude_pub.publish(altitude_msg)
    self.last_telemetry['altitude'] = altitude_msg

    altitude_m = altitude_msg.data[0]
    vario_mps = altitude_msg.data[1]
    self.node.get_logger().info(
        f'   ➜ Published to /fc/altitude | '
        f'altitude={altitude_m:.2f}m | '
        f'vario={vario_mps:+.2f}m/s'
    )
```

---

## ROS Topic

**Topic:** `/fc/altitude`
**Type:** `std_msgs/Float32MultiArray`
**QoS:** BEST_EFFORT (sensor data)
**Format:** `[altitude_m, vertical_velocity_m/s]`

**Data:**
- `data[0]`: Barometer altitude in meters (float)
- `data[1]`: Vertical velocity in m/s (float, positive = climbing)

**Example:**
```python
# Subscribe to altitude
self.create_subscription(
    Float32MultiArray,
    '/fc/altitude',
    self.altitude_callback,
    sensor_qos
)

def altitude_callback(self, msg):
    altitude = msg.data[0]       # meters
    vario = msg.data[1]          # m/s (vertical velocity)
```

---

## Usage in fc_adapter_node.py

The `fc_adapter_node.py` already has a subscriber for this topic (line 174):

```python
self.create_subscription(Float32MultiArray, '/fc/altitude', self.cb_altitude, sensor_qos)
```

And a callback that uses the vertical velocity (line 268-270):

```python
def cb_altitude(self, msg: Float32MultiArray):
    if len(msg.data) >= 2:
        self.velocity_actual_earth[2] = float(msg.data[1])  # vertical velocity
```

**Now this will work!** The vertical velocity from the barometer will be available for the PID controller.

---

## Benefits

### 1. **Better Altitude Feedback**
- Barometer provides more stable altitude than GPS (GPS has 5-10m accuracy)
- Updates faster than GPS (GPS typically 1-1.5 Hz)
- Vertical velocity from barometer is smoother than GPS-derived velocity

### 2. **Improved PID Control**
- `fc_adapter_node.py` now receives actual vertical velocity
- Better feedback for altitude PID controller
- Should reduce oscillations and overshoot

### 3. **Consistent with INAV**
- Uses INAV's fused altitude estimate
- Benefits from INAV's barometer + GPS fusion
- Same altitude value INAV uses for ALT HOLD

---

## Testing

### 1. Check Topic is Published
```bash
ros2 topic list | grep /fc/altitude
ros2 topic echo /fc/altitude
```

**Expected output:**
```
data: [182.45, 0.12]
---
data: [182.47, 0.15]
---
```

### 2. Check Update Rate
```bash
ros2 topic hz /fc/altitude
```

**Expected:** ~10 Hz (round-robin with other telemetry, 7 commands total)

### 3. Check Data Quality
```bash
ros2 topic echo /fc/altitude --once
```

- Altitude should match barometer reading (varies with weather/altitude)
- Vario should be small when hovering (±0.1 m/s)
- Vario should be positive when climbing, negative when descending

### 4. Verify fc_adapter Receives It
Check fc_adapter logs for vertical velocity updates:
```bash
ros2 run swarm_ai_integration fc_adapter_node
```

Look for altitude callback activity in the PID loop.

---

## Comparison: GPS Altitude vs Barometer Altitude

| Aspect | GPS Altitude | Barometer Altitude |
|--------|--------------|-------------------|
| **Accuracy** | ±5-10m | ±1m (relative) |
| **Update Rate** | 1-1.5 Hz | 10+ Hz (via MSP) |
| **Noise** | High | Low (smoothed) |
| **Drift** | None (absolute) | Yes (pressure changes) |
| **Best For** | Absolute altitude | Relative altitude, control |

**For PID control:** Barometer is much better (stable, fast, low noise)
**For position:** GPS is better (no drift)

**INAV fuses both:** Uses barometer for control, GPS for drift correction

---

## Integration with ALT HOLD Issue

This change directly addresses the altitude control issue:

### Before
- `fc_adapter_node` was subscribing to `/fc/altitude`
- But no data was published → vertical velocity was stale/zero
- PID controller had poor feedback → oscillations

### After
- Real barometer-based vertical velocity available
- PID controller gets accurate, low-latency feedback
- Should significantly improve altitude stability

### Recommended Next Steps
1. Test new simulator with altitude feedback
2. Verify barometer altitude in real flight (props OFF)
3. Test hover with reduced PID gains (kp_z=20)
4. Compare altitude stability to previous flights

---

## Code Quality

All changes follow existing patterns:
- ✅ Consistent naming conventions
- ✅ Proper error handling
- ✅ Logging for debugging
- ✅ Type hints
- ✅ Documentation
- ✅ Same structure as other MSP messages

No breaking changes:
- ✅ Only additions, no modifications to existing functionality
- ✅ Backwards compatible
- ✅ No changes to external APIs

---

## Build & Test

```bash
cd ~/swarm-ros
colcon build --packages-select swarm_ai_integration
source install/setup.bash

# Test fc_comms with real FC
ros2 run swarm_ai_integration fc_comms_node

# Check altitude topic
ros2 topic echo /fc/altitude

# Test full stack
ros2 launch swarm_ai_integration ai_flight_launch.py
```

---

## Summary

**Problem:** `/fc/altitude` topic was never published
**Solution:** Added complete MSP_ALTITUDE implementation
**Result:** Barometer altitude + vertical velocity now available
**Impact:** Better altitude control, reduced oscillations

**Files changed:** 4
**Lines added:** ~80
**Lines modified:** ~10
**Breaking changes:** 0

**Status:** ✅ Ready for testing

---

**Generated:** 2025-10-30
**For:** swarm-ros altitude control improvements
