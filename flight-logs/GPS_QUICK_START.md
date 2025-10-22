# GPS Quality System - Quick Start Guide

## TL;DR - What Changed?

✅ System now **requires 6+ satellites and 3D GPS fix** before flight
✅ Origin is **averaged over multiple samples** (10-30 depending on GPS quality)
✅ Clear **status messages** show GPS progress

## What You'll See

### ❌ Before GPS Lock (< 6 satellites)
```
⚠️  Waiting for GPS lock: 3 satellites (need 6), fix_type=0 (need 2+)
⚠️  GPS not ready: 3 satellites (need 6), fix_type=0
```
**Action:** Wait for GPS to acquire more satellites

---

### ✅ GPS Lock Achieved (6+ satellites)
```
✅ GPS LOCK: 8 satellites, fix_type=2
📊 Tiered averaging: collecting 20 samples for origin (~13.3s at 1.5 Hz)
```
**Action:** Wait for sample collection (automatic)

---

### ⏳ Collecting Samples
```
⏳ Collecting GPS samples for origin: 5/20
⏳ Collecting GPS samples for origin: 10/20
⏳ Collecting GPS samples for origin: 15/20
```
**Action:** Keep waiting (system is improving accuracy)

---

### 🎯 Origin Set - Ready to Fly!
```
🎯 ORIGIN SET: Averaged 20 GPS samples with 8 satellites
[AI observations start publishing]
```
**Action:** Drone is ready for autonomous flight

---

## Timing Expectations

| Satellites | Sample Count | Wait Time | Quality |
|-----------|--------------|-----------|---------|
| 10+ | 10 samples | ~7 sec | Excellent ⭐⭐⭐ |
| 8-9 | 20 samples | ~14 sec | Good ⭐⭐ |
| 6-7 | 30 samples | ~20 sec | OK ⭐ |
| < 6 | Rejected | Indefinite | Insufficient ❌ |

---

## How to Adjust Settings

### Change Minimum Satellites (Default: 6)

**In launch file:**
```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py min_gps_satellites:=8
```

**Recommendations:**
- **6** = Standard (safe for most flights) ✅
- **8** = Better quality (slightly longer wait) 🌟
- **10** = Excellent quality (longest wait) 💎
- **4-5** = ⚠️ NOT RECOMMENDED (unreliable positioning)

---

## Monitoring GPS Status

### Check Satellite Count
```bash
ros2 topic echo /fc/gps_satellites
```

### Check GPS Fix Status
```bash
ros2 topic echo /fc/gps_fix
# Look at: status.status
# 0 = NO_FIX ❌
# 1 = 2D FIX ⚠️
# 2 = 3D FIX ✅
```

### Monitor Observations (Should start after origin set)
```bash
ros2 topic hz /ai/observation
# Should be ~30 Hz after origin is set
```

---

## Troubleshooting

### Problem: Stuck at "Waiting for GPS lock"

**Possible Causes:**
1. Not enough satellites visible
2. GPS antenna obstructed
3. Indoor operation (GPS doesn't work indoors!)
4. GPS module not initialized properly

**Solutions:**
- Move to clear outdoor area
- Wait longer (GPS cold start can take 1-2 minutes)
- Check GPS antenna connection
- Verify GPS module power

---

### Problem: "Collecting samples" takes too long

**This is normal!** The system is improving accuracy.

**Timeframes:**
- 6-7 satellites: Up to 20 seconds
- 8-9 satellites: Up to 14 seconds
- 10+ satellites: Up to 7 seconds

If it's taking longer, check your GPS data rate (should be ~1.5 Hz).

---

### Problem: Flight log shows "NO_FIX" but system worked before

**This is a GOOD thing!**

The old system would attempt to use unreliable GPS data. The new system prevents this, making flights safer.

**Action:** Wait for proper GPS lock before attempting flight.

---

## Pre-Flight Checklist

1. ✅ Power on drone
2. ✅ Launch ROS system
3. ✅ Wait for GPS lock message (6+ satellites)
4. ✅ Wait for origin set message
5. ✅ Verify `/ai/observation` topic publishing
6. ✅ **NOW SAFE TO FLY**

---

## Testing Without Flying

### View GPS Status in Real-Time
```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py

# In another terminal:
ros2 topic echo /fc/gps_satellites
```

### Simulate GPS Lock Progress
Watch the node logs for:
1. Warning messages (< 6 satellites)
2. Lock achieved message
3. Sample collection progress
4. Origin set message

---

## When to Contact Support

- GPS never achieves 6+ satellites after 5 minutes outdoors
- System crashes during GPS initialization
- Observations never start publishing even after origin set
- Position drifts significantly (> 5m) during flight

---

## Advanced: Analyzing GPS Quality

Use the provided analysis tool:

```bash
python3 analyze_gps.py flight-logs/blackbox_XXXXXXXX.jsonl
```

This will show you:
- Satellite count over time
- Position noise/accuracy
- Benefit of averaging
- Recommendations for your location

---

## Summary

🎯 **Goal:** Ensure reliable GPS before autonomous flight
🔒 **Safety:** No flight with poor GPS quality
⏱️ **Trade-off:** ~10-20 second startup for better accuracy
✅ **Result:** More reliable positioning, safer flights

**The extra wait is worth the improved safety and accuracy!**
