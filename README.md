# Langostino - The Swarm Drone

This repo is designed to allow anyone to build their on drone and power it with AI.

---

## Quick Start

**How to build and configure your drone** Check this articles:
1. [Chapter 1](https://substack.com/home/post/p-175604069) - Inside the drone
2. [Chapter 2](https://substack.com/home/post/p-176136139) - The wiring brain
3. [Chapter 3](https://substack.com/home/post/p-177453660) - From data to motion

**New to the system?** Start here:
1. [Setup Guide](SETUP_GUIDE.md) - Install and configure the system
2. [Commands Guide](COMMANDS_GUIDE.md) - Learn how to run the system
3. [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md) - Fix common issues

**Ready to fly?** Configure your drone:
1. [INAV Guide](INAV_GUIDE.md) - Tune flight controller parameters
2. [Config Params Guide](CONFIG_PARAMS_GUIDE.md) - Adjust ROS2 parameters

---

## How to configure your drone.
These videos by Joshua Bradwell are very useful for setting up your INAV drone.
[Complete INAV walk-throught](https://www.youtube.com/watch?v=xdf3yhlgJyc)
[Compass troubles](https://www.youtube.com/watch?v=HWV5b9ZT1eE)

## Documentation Index

### [Setup Guide](SETUP_GUIDE.md)
**Complete installation and configuration guide**

- System prerequisites and hardware requirements
- Automated setup with `setup.sh`
- Manual installation steps
- Hardware configuration (I2C, UART, Network)
- Verification procedures
- PM2 process management setup

**When to use:** First time setup, reinstalling system, adding new hardware

---

### [Commands Guide](COMMANDS_GUIDE.md)
**System operation and command reference**

- Launch methods (PM2, ROS2, manual)
- Individual node commands
- PM2 process management
- ROS2 diagnostic commands
- Quick command reference table

**When to use:** Daily operations, starting/stopping system, monitoring processes

---

### [INAV Guide](INAV_GUIDE.md)
**Flight controller parameter configuration**

- Navigation parameters (bank angle, inclination)
- Climb rate settings
- GPS position control (PID tuning)
- Attitude control (level PIDs)
- MSP configuration
- Flight behavior tuning
- Quick tuning reference table

**When to use:** Tuning flight behavior, adjusting GPS navigation, fixing oscillations/drift

---

### [Config Params Guide](CONFIG_PARAMS_GUIDE.md)
**ROS2 parameter reference**

- Complete parameter documentation for all nodes:
  - AI Adapter Node
  - AI Flight Node
  - Safety Monitor Node
  - FC Communications Node
  - FC Adapter Node
  - Black Box Recorder Node
  - LiDAR Reader Node
- Parameter ranges and recommendations
- Tuning scenarios
- Runtime parameter modification

**When to use:** Adjusting system behavior, tuning PID controllers, changing safety limits

---

### [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md)
**Comprehensive troubleshooting reference**

- Quick diagnostics
- System & ROS2 issues
- Hardware issues (I2C, UART, GPS)
- Build & installation issues
- Network & connectivity issues
- Runtime & node issues
- Flight control issues
- Parameter & configuration issues
- Performance issues
- Safety & emergency procedures

**When to use:** System not working, debugging issues, performance problems, flight issues

---

## Documentation Organization

The documentation is organized to eliminate redundancy and focus each guide on its specific topic:

- **Setup** → Installation and hardware configuration
- **Commands** → Daily operations and system control
- **INAV** → Flight controller tuning
- **Config Params** → ROS2 parameter reference
- **Troubleshooting** → Centralized problem-solving

All guides cross-reference each other for related information.

---

## Common Tasks

### First Time Setup
1. Follow [Setup Guide](SETUP_GUIDE.md) completely
2. Run verification: `./verify_setup.sh`
3. Check [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md) if issues arise

### Launching the System
1. See [Commands Guide](COMMANDS_GUIDE.md) for launch methods
2. Monitor with: `pm2 list` and `pm2 logs`
3. Verify topics: `ros2 topic list`

### Tuning Flight Behavior
1. Start with [INAV Guide](INAV_GUIDE.md) for FC parameters
2. Use [Config Params Guide](CONFIG_PARAMS_GUIDE.md) for ROS2 PID tuning
3. Check [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md#flight-control-issues) for specific issues

### Debugging Issues
1. Check [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md) first
2. Collect diagnostic information (logs, hardware status)
3. Reference specific guides for detailed parameter information

---

## File Reference

### Configuration Files
- `src/swarm_ai_integration/config/swarm_params.yaml` - ROS2 parameters
- `inav-params.txt` - INAV CLI commands
- `.bashrc` - ROS2 environment setup

### Scripts
- `setup.sh` - Automated installation
- `launch.sh` - System launcher
- `verify_setup.sh` - System verification

### Log Files
- `~/.pm2/logs/` - PM2 process logs
- `~/swarm-ros/flight-logs/` - Flight data recordings

---

## Getting Help

**Before reporting issues:**
1. Check [Troubleshooting Guide](TROUBLESHOOTING_GUIDE.md)
2. Run diagnostics: `./verify_setup.sh`
3. Collect logs: `pm2 logs --lines 200`
4. Check hardware: `i2cdetect -y 1`, `ls -l /dev/ttyAMA0`

**When reporting issues:**
- Include system information (`uname -a`, `echo $ROS_DISTRO`)
- Provide relevant logs
- Describe what you tried
- Include error messages
