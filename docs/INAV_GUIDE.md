# INAV Configuration Guide

Complete guide for configuring INAV flight controller parameters for autonomous drone operations.

> 📖 **Deep Dive:** For additional configuration tips and advanced tuning, see [Chapter 3.5: Additional Configurations](https://substack.com/home/post/p-180586067) on our Substack.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Navigation Parameters](#navigation-parameters)
   - [Bank Angle Limits](#bank-angle-limits)
   - [Angle Mode Inclination](#angle-mode-inclination)
3. [Climb Rate Configuration](#climb-rate-configuration)
   - [Automatic Climb Rate](#automatic-climb-rate)
   - [Manual Climb Rate](#manual-climb-rate)
4. [GPS Position Control (XY Velocity PID)](#gps-position-control-xy-velocity-pid)
   - [Proportional (P) Term](#proportional-p-term)
   - [Integral (I) Term](#integral-i-term)
   - [Derivative (D) Term](#derivative-d-term)
   - [Feedforward (FF) Term](#feedforward-ff-term)
5. [Attitude Control (Level PIDs)](#attitude-control-level-pids)
   - [Level P Term](#level-p-term)
   - [Level D Term](#level-d-term)
6. [MSP Configuration](#msp-configuration)
   - [RC Override Channels](#rc-override-channels)
   - [Hover Throttle](#hover-throttle)
   - [Altitude Hold Throttle Mode](#altitude-hold-throttle-mode)
   - [Deadband Settings](#deadband-settings)
   - [I-Term Relax](#i-term-relax)
7. [GPS Settings](#gps-settings)
8. [Quick Reference](#quick-reference)
9. [Tuning Tips](#tuning-tips)
10. [Applying Configuration](#applying-configuration)

## Introduction

This guide provides detailed explanations of critical INAV parameters for multicopter autonomous flight. These settings control navigation behavior, position hold accuracy, and overall flight characteristics.

**Recommended Use Cases:**

- Autonomous GPS navigation
- Position hold (PosHold) mode
- Altitude hold (AltHold) mode
- Return to Home (RTH)
- Waypoint navigation
- MSP RC override for external control

## Navigation Parameters

### Bank Angle Limits

#### `nav_mc_bank_angle`

```
set nav_mc_bank_angle = 35
```

**Description:**
Maximum tilt angle (in degrees) used by autonomous navigation modes including PosHold, AltHold, and Waypoint navigation.

**Purpose:**

- Limits how aggressively the drone can tilt during GPS navigation
- Prevents excessive tilting that could lead to loss of control
- Ensures smooth, predictable flight paths

**Configuration:**

- **Value:** `35` degrees
- **Range:** Typically 5-45 degrees
- **Recommendation:**
  - Start conservative (15-20°) for testing
  - Increase for faster navigation if needed
  - Lower values = smoother, slower movements
  - Higher values = faster, more aggressive navigation

### Angle Mode Inclination

#### `max_angle_inclination_rll` and `max_angle_inclination_pit`

```
set max_angle_inclination_rll = 200
set max_angle_inclination_pit = 200
```

**Description:**
Maximum allowed tilt angle for ACRO/ANGLE mode manual flight.

**Units:** 0.1 degrees (150 = 15.0°)

**Purpose:**

- Limits manual stick input tilt angles
- Independent from autonomous navigation limits
- Safety limit for manual control

**Configuration:**

- **Value:** `200` (20°)
- **Recommendation:**
  - Conservative setting for smooth manual flight
  - Prevents novice pilots from over-tilting
  - Can be increased for sport/acrobatic flight (up to 500 = 50°)

## Climb Rate Configuration

### Automatic Climb Rate

#### `nav_mc_auto_climb_rate`

```
set nav_mc_auto_climb_rate = 300
```

**Description:**
Vertical speed used automatically by INAV during autonomous operations.

**Units:** cm/s (100 = 1.0 m/s)

**Used For:**

- Taking off in autonomous modes
- Landing sequences
- RTH altitude adjustments
- Waypoint altitude changes

**Configuration:**

- **Value:** `500` cm/s (5.0 m/s)
- **Recommendation:**
  - Conservative for safe takeoffs/landings
  - Increase to 100-150 cm/s (1-1.5 m/s) for faster altitude changes
  - Lower values provide smoother, safer operations

### Manual Climb Rate

#### `nav_mc_manual_climb_rate`

```
set nav_mc_manual_climb_rate = 300
```

**Description:**
Controls how throttle stick adjusts altitude while in AltHold or PosHold modes.

**Units:** cm/s (100 = 1.0 m/s)

**Behavior:**

- **Throttle above mid:** Climb at configured rate
- **Throttle below mid:** Descend at configured rate
- **Throttle at mid:** Hold current altitude

**Configuration:**

- **Value:** `200` cm/s (2.0 m/s)
- **Recommendation:**
  - 50-100 cm/s for precise control
  - Higher values = faster altitude changes with stick input
  - Match to `nav_mc_auto_climb_rate` for consistent behavior


## GPS Position Control (XY Velocity PID)

The XY velocity PID loop controls horizontal position accuracy during GPS navigation. These parameters directly affect how tightly the drone holds position.

### Proportional (P) Term

#### `nav_mc_vel_xy_p`

```
set nav_mc_vel_xy_p = 35
```

**Function:**
Proportional response to horizontal velocity error.

**Behavior:**

- **Higher P** → Stronger, more aggressive corrections
- **Lower P** → Softer, floatier feel
- **Too high** → Oscillations and overshooting
- **Too low** → Slow corrections, poor position hold

**Configuration:**

- **Value:** `35`
- **Tuning:**
  - Start at 25, increase gradually if position hold is loose
  - If drone oscillates in position hold, reduce P
  - Typical range: 10-40

### Integral (I) Term

#### `nav_mc_vel_xy_i`

```
set nav_mc_vel_xy_i = 18
```

**Function:**
Corrects accumulated position error over time.

**Behavior:**

- Handles long-term drift
- Compensates for wind and bias
- **Too high** → Oscillations, "toilet bowling" effect
- **Too low** → Position drift over time, especially in wind

**Configuration:**

- **Value:** `18`
- **Tuning:**
  - If drone drifts slowly over time, increase I
  - If drone circles or oscillates slowly, reduce I
  - Typical range: 5-20

**Warning:** The I term is powerful but can cause instability if too high. Tune conservatively.

### Derivative (D) Term

#### `nav_mc_vel_xy_d`

```
set nav_mc_vel_xy_d = 105
```

**Function:**
Damping term that smooths velocity corrections.

**Behavior:**

- Reduces oscillations
- **Higher D** → Snappier, tighter position lock
- **Too high** → Twitchy, jittery movements
- **Too low** → Bouncy, oscillatory corrections

**Configuration:**

- **Value:** `105` (high for tight hold)
- **Tuning:**
  - If drone bounces around target position, increase D
  - If drone feels twitchy or nervous, reduce D
  - Typical range: 30-120

### Feedforward (FF) Term

#### `nav_mc_vel_xy_ff`

```
set nav_mc_vel_xy_ff = 40
```

**Function:**
Reduces delay between stick input and movement in GPS modes.

**Behavior:**

- Improves responsiveness
- **Higher FF** → More direct feel, less lag
- **Too high** → Aggressive, overshooting
- **Too low** → Sluggish response

**Configuration:**

- **Value:** `40`
- **Tuning:**
  - If drone feels sluggish in GPS modes, increase FF
  - If drone overshoots targets, reduce FF
  - Typical range: 10-50

### XY Velocity PID Summary

```
set nav_mc_vel_xy_p = 35
set nav_mc_vel_xy_i = 18
set nav_mc_vel_xy_d = 105
set nav_mc_vel_xy_ff = 40
```

**Quick Tuning Guide:**

| Issue                           | Solution                 |
| ------------------------------- | ------------------------ |
| Drone drifts from position      | Increase P or I          |
| Drone oscillates/bounces        | Decrease P or increase D |
| Slow circles ("toilet bowling") | Decrease I               |
| Overshooting target positions   | Decrease P.              |
| Twitchy, nervous movements      | Decrease D               |


## Attitude Control (Level PIDs)

Level PIDs control attitude stabilization in ANGLE/HORIZON modes and during GPS navigation. These are **NOT** GPS-related but affect how the drone levels itself.

### Level P Term

#### `mc_p_level`

```
set mc_p_level = 20
```

**Function:**
Controls how strongly the drone returns to level attitude.

**Behavior:**

- **Higher P** → More rigid, responsive leveling
- **Lower P** → Softer, less aggressive leveling
- **Too high** → Oscillations, overshooting level
- **Too low** → Slow to level, floaty feel

**Configuration:**

- **Value:** `20` (good for larger drones)
- **Tuning:**
  - Larger/heavier drones: 20-30
  - Smaller/lighter drones: 40-60
  - If drone wobbles when leveling, reduce P

### Level D Term

#### `mc_d_level`

```
set mc_d_level = 75
```

**Function:**
Damping for leveling response, reduces overshoot.

**Behavior:**

- Smooths stabilization
- Reduces bounce/oscillation when leveling
- **Too high** → Sluggish, soft feel
- **Too low** → Bouncy, oscillatory leveling

**Configuration:**

- **Value:** `75`
- **Tuning:**
  - If drone bounces when returning to level, increase D
  - If drone feels mushy or slow, reduce D
  - Typical range: 30-100

### Level PID Summary

```
set mc_p_level = 20
set mc_d_level = 75
```

These values provide smooth, predictable leveling for most medium-sized multirotors.

## MSP Configuration

### RC Override Channels

#### `msp_override_channels`

```
set msp_override_channels = 127
```

**Description:**
Bitmask defining which RC channels can be overridden via MSP (Multiwii Serial Protocol).

**Binary Explanation:**

- `127` in decimal = `01111111` in binary
- Each bit represents a channel (right to left: CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8)
- `1` = Channel can be overridden by MSP
- `0` = Channel cannot be overridden by MSP

**Configuration:**

- **Value:** `127` → MSP can override CH1-CH7
- **Alternative:** `255` → MSP can override CH1-CH8

**Use Case:**

- External flight computers (like this ROS2 system) can send RC commands via MSP
- Critical for autonomous flight using companion computers
- Allows programmatic control of roll, pitch, throttle, yaw, and aux channels

**Channel Mapping:**

- CH1: Roll
- CH2: Pitch
- CH3: Throttle
- CH4: Yaw
- CH5: ARM (AUX1)
- CH6: ANGLE mode (AUX2)
- CH7: ALT HOLD (AUX3)
- CH8: MSP RC Override Enable (AUX4)

### Hover Throttle

#### `nav_mc_hover_thr`

```
set nav_mc_hover_thr = 1500
```

**Description:**
RC value for throttle that achieves hover in altitude hold mode.

**Units:** RC PWM value (1000-2000)

**Purpose:**

- Baseline throttle for altitude hold calculations
- INAV uses this as reference point for climb/descent commands
- Affects how altitude hold interprets throttle deviations

**Configuration:**

- **Value:** `1500` (mid-stick)
- **Tuning:**
  - If drone climbs in AltHold with mid-throttle → Decrease value
  - If drone descends in AltHold with mid-throttle → Increase value
  - Typical range: 1400-1600
  - Must match actual hover throttle of your drone

**Important:** This should be calibrated to your specific drone's actual hover throttle point. This setting is per battery profile, so set it for all battery profiles you use.

### Altitude Hold Throttle Mode

#### `nav_mc_althold_throttle`

```
set nav_mc_althold_throttle = HOVER
```

**Description:**
Defines how throttle input is interpreted during altitude hold modes.

**Options:**

- `STICK` - Throttle stick directly controls climb/descent rate
- `MID_STICK` - Mid-stick position holds altitude, deviation controls climb/descent
- `HOVER` - Throttle at hover point holds altitude, more intuitive for most pilots

**Configuration:**

- **Value:** `HOVER`
- **Recommendation:**
  - `HOVER` is recommended for most use cases
  - Provides intuitive throttle behavior where hover throttle = hold altitude
  - Works well with `nav_mc_hover_thr` setting

---

### Deadband Settings

#### `pos_hold_deadband`

```
set pos_hold_deadband = 50
```

**Description:**
Stick deadband for position hold mode. Within this range, the drone holds position; outside, it moves.

**Units:** RC units (centered at 1500)

**Purpose:**

- Prevents unintended position changes from small stick movements
- Creates a "hold zone" around stick center
- Allows precise position holding without fighting stick noise

**Configuration:**

- **Value:** `50`
- **Recommendation:**
  - Higher values = larger deadband, easier to hold position
  - Lower values = more responsive to small inputs
  - Typical range: 20-100

#### `alt_hold_deadband`

```
set alt_hold_deadband = 50
```

**Description:**
Throttle stick deadband for altitude hold mode. Within this range, altitude is maintained.

**Units:** RC units (centered at 1500)

**Purpose:**

- Prevents unintended altitude changes from small throttle movements
- Creates a "hold zone" around throttle center/hover point
- Allows precise altitude holding

**Configuration:**

- **Value:** `50`
- **Recommendation:**
  - Higher values = larger deadband, easier to hold altitude
  - Lower values = more responsive to throttle inputs
  - Typical range: 20-100

### I-Term Relax

#### `mc_iterm_relax_cutoff`

```
set mc_iterm_relax_cutoff = 8
```

**Description:**
Cutoff frequency for I-term relax feature. Controls how quickly the I-term is suppressed during rapid stick movements.

**Units:** Hz

**Purpose:**

- Prevents I-term windup during aggressive maneuvers
- Reduces bounce-back after quick stick movements
- Improves overall flight feel and responsiveness

**Configuration:**

- **Value:** `8` Hz
- **Recommendation:**
  - Lower values = more aggressive I-term suppression
  - Higher values = less suppression, more traditional behavior
  - Typical range: 5-15 Hz
  - Lower values help with quick stops in position hold

## GPS Settings

### Minimum Satellites

#### `gps_min_sats`

```
set gps_min_sats = 6
```

**Description:**
Minimum number of GPS satellites required before GPS modes are enabled.

**Purpose:**

- Ensures sufficient GPS accuracy before allowing GPS navigation
- Prevents GPS modes from engaging with poor satellite coverage
- Safety feature to avoid unreliable position data

**Configuration:**

- **Value:** `6` satellites
- **Recommendation:**
  - Minimum safe value: 5
  - Better accuracy: 6-8
  - More satellites = better accuracy and reliability
  - Urban/obstructed areas may need lower threshold (5)
  - Open sky operations: can use 6-8 for better accuracy

**GPS Fix Quality:**

- 3-4 sats: Poor (not recommended)
- 5-6 sats: Adequate (minimum for navigation)
- 7-10 sats: Good (recommended)
- 10+ sats: Excellent

## Quick Reference

### Complete Configuration Script (INAV CLI commands)

Copy and paste this into INAV CLI to apply all settings:

```bash
# Navigation Angles
set nav_mc_bank_angle = 35
set max_angle_inclination_rll = 200
set max_angle_inclination_pit = 200

# Climb Rates
set nav_mc_auto_climb_rate = 300
set nav_mc_manual_climb_rate = 300

# XY Velocity PID (GPS Position Control)
set nav_mc_vel_xy_p = 35
set nav_mc_vel_xy_i = 18
set nav_mc_vel_xy_d = 105
set nav_mc_vel_xy_ff = 40

# Level PIDs (Attitude Control)
set mc_p_level = 20
set mc_d_level = 75

# Altitude Hold Settings
set nav_mc_althold_throttle = HOVER
set alt_hold_deadband = 50
set pos_hold_deadband = 50

# I-Term Relax
set mc_iterm_relax_cutoff = 8

# MSP Configuration
set msp_override_channels = 127

# GPS Settings
set gps_min_sats = 6

# Battery Profile Settings (apply to all battery profiles)
battery_profile 1
set nav_mc_hover_thr = 1500

# Save configuration
save
```

## Tuning Tips

### General Tuning Workflow

1. **Start Conservative:** Use default values from this guide
2. **Test Basic Flight:** Verify stable hover and basic navigation
3. **Tune One Parameter at a Time:** Change only one value per test flight
4. **Make Small Changes:** Adjust by 10-20% increments
5. **Document Changes:** Keep notes of what works and what doesn't

### Safety Checklist

Before autonomous flight:

- [ ] GPS has minimum satellite lock (5+)
- [ ] Position hold tested manually in open area
- [ ] Altitude hold stable at mid-throttle
- [ ] Return to home tested from short distance
- [ ] Failsafe configured and tested
- [ ] MSP override channels verified
- [ ] Manual mode available as backup

### INAV Tuning Quick Reference

These are common flight behavior issues specific to INAV parameter tuning. For comprehensive troubleshooting (hardware, software, system issues), see [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md).

| Flight Behavior            | Primary Adjustment                  | Secondary Adjustments                 |
| -------------------------- | ----------------------------------- | ------------------------------------- |
| **Position drift**         | Increase `nav_mc_vel_xy_i` (10-12)  | Check GPS, calibrate compass          |
| **Oscillations/bouncing**  | Decrease `nav_mc_vel_xy_p` (15-18)  | Increase `nav_mc_vel_xy_d` (60-70)    |
| **Toilet bowling**         | Decrease `nav_mc_vel_xy_i` (5-6)    | Calibrate compass, check interference |
| **Sluggish response**      | Increase `nav_mc_vel_xy_ff` (30-40) | Increase `nav_mc_vel_xy_p` (25-30)    |
| **Altitude drift up/down** | Adjust `nav_mc_hover_thr` (±50)     | Calibrate barometer, check for leaks  |

For detailed troubleshooting of these and other issues, see:

- [Flight Control Issues](TROUBLESHOOTING_GUIDE.md#flight-control-issues) - Detailed solutions with diagnostics
- [Hardware Issues](TROUBLESHOOTING_GUIDE.md#hardware-issues) - GPS, compass, sensor problems

## Applying Configuration

### Using INAV Configurator

1. Connect flight controller via USB
2. Open INAV Configurator
3. Go to **CLI** tab
4. Paste configuration commands
5. Type `save` and press Enter
6. Wait for FC to reboot

### Using MSP Commands

Configuration can also be applied programmatically via MSP serial protocol from companion computer.

### Verification

After applying configuration:

1. **Check in CLI:**

   ```bash
   get nav_mc_vel_xy_p
   get nav_mc_vel_xy_i
   # etc...
   ```

2. **Test Flight:**

   - Manual mode first
   - Then angle mode
   - Then altitude hold
   - Finally position hold
   - Test RTH last

3. **Monitor Behavior:**
   - Use blackbox logging
   - Review flight logs
   - Adjust as needed

## Additional Resources

**INAV Official Documentation:**

- [INAV Official Wiki](https://github.com/iNavFlight/inav/wiki)
- [INAV PID Tuning Guide](https://github.com/iNavFlight/inav/wiki/PID-tuning)
- [INAV GPS Configuration](https://github.com/iNavFlight/inav/wiki/GPS-and-Compass-setup)

## Notes

- These parameters are optimized for medium-sized multicopters (250-450mm)
- Smaller or larger drones may need different values
- Always test in safe, open areas with good GPS coverage
- Keep manual mode as backup on a switch
- Monitor battery voltage - low voltage affects position hold performance
