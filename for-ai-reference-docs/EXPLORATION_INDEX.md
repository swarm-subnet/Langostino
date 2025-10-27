# Swarm ROS Project - Exploration Index

This directory contains comprehensive documentation generated during a thorough exploration of the Swarm ROS drone project (October 27, 2025).

## Generated Documentation Files

### 1. **EXPLORATION_REPORT.md** (Primary)
**Comprehensive technical report covering all 6 key areas:**
- Overall project structure and organization
- 7 ROS nodes with detailed documentation
- Waypoint management and publishing flows
- Coordinate transformation system (GPS to ENU)
- ROS topics for navigation and control
- Return-to-home and landing logic
- Known issues and fixes

**Best for:** Understanding system architecture, how components interact, and technical deep-dives.

### 2. **ARCHITECTURE_VISUAL.txt** (Visual Reference)
**ASCII diagrams and visual representations:**
- Complete data flow diagram (sensor to hardware)
- 131-D observation vector specification
- Coordinate system transformations
- Waypoint tracking process flow
- Safety monitor and RTH activation flow
- Parameter configuration locations
- Node startup sequence

**Best for:** Quick visual understanding, presentations, and reference guides.

### 3. **ABSOLUTE_PATHS_REFERENCE.txt** (File Locations)
**Complete file system reference:**
- All critical files with absolute paths
- Core configuration location
- Main ROS nodes (7 nodes documented)
- Utility module descriptions
- Protocol and control modules
- Documentation files
- Known issues locations

**Best for:** Finding specific files and understanding code organization.

## Quick Navigation

### For Different Audiences:

**Software Engineers / Developers:**
- Start: EXPLORATION_REPORT.md (Section 2: ROS Nodes)
- Then: ABSOLUTE_PATHS_REFERENCE.txt
- Reference: ARCHITECTURE_VISUAL.txt

**System Architects:**
- Start: ARCHITECTURE_VISUAL.txt (complete flow diagram)
- Deep-dive: EXPLORATION_REPORT.md (all sections)
- Details: ABSOLUTE_PATHS_REFERENCE.txt

**Project Managers / Non-Technical:**
- Start: ARCHITECTURE_VISUAL.txt
- Overview: EXPLORATION_REPORT.md (Section 1: Project Structure)
- Summary below

**Debugging & Troubleshooting:**
- Coordinate transforms: EXPLORATION_REPORT.md Section 4
- Waypoint issues: EXPLORATION_REPORT.md Section 3
- Safety/RTH issues: EXPLORATION_REPORT.md Section 6
- Known issues: EXPLORATION_REPORT.md Section 7

## Key Facts Summary

### System Type
- **ROS 2** autonomous drone flight control system
- **PPO (Proximal Policy Optimization)** AI policy inference
- **INAV 7** flight controller integration
- **Real drone** with GPS, IMU, LiDAR, battery monitoring

### Architecture Highlights
- **7 main ROS nodes** with clear separation of concerns
- **131-D observation** vector (position, orientation, velocity, actions, LiDAR, goal)
- **30 Hz observation** generation, **10 Hz AI** inference, **40 Hz control** loop
- **Centralized configuration** in single YAML file
- **ENU relative coordinates** with computed origin

### Critical System Features
1. **Home Position** is NOT [0,0,0], but [0,0,3] (takeoff height)
2. **Waypoints** converted from geodetic to ENU relative coordinates
3. **Safety** includes RTH activation via RC Channel 9
4. **Coordinate frame** bug: ENU vs body frame (documented fix available)
5. **GPS averaging** uses tiered strategy (10-30 samples by HDOP)

### Node Roles
| Node | Rate | Input | Output |
|------|------|-------|--------|
| AI Adapter | 30 Hz | Sensors | 131-D observation |
| AI Flight | 10 Hz | Observation | [vx,vy,vz,speed] |
| FC Comms | 10 Hz | Serial (MSP) | /fc/* topics |
| FC Adapter | 40 Hz | Actions | RC commands |
| Safety | 10 Hz | Observation | /safety/* |
| LiDAR | 100 Hz | I2C | Distance data |
| Black Box | var | Topics | Log files |

### Critical Parameters
- `relative_start_enu: [0, 0, 3]` - Initial position
- `max_ray_distance: 20.0` - LiDAR normalization (MUST match training)
- `max_altitude: 10.0` - Safety limit
- `max_distance_from_home: 100.0` - Geofence
- PID gains: xy(150,10,20), z(100,5,15)

## Exploration Methodology

### Thoroughness Level: Medium
Covered:
- All 7 ROS nodes (purposes, rates, interfaces)
- Configuration system (centralized YAML)
- Core utility modules
- Coordinate transformation system
- ROS topic architecture
- Safety and RTH logic
- Known issues with documented fixes

Not extensively covered (but documented):
- Individual code line-by-line analysis
- Mathematical proofs of transformations
- Detailed legacy code investigation
- Full test suite review

## How to Use These Documents

### Scenario 1: "I need to understand the overall system"
1. Read ARCHITECTURE_VISUAL.txt (5 min)
2. Read EXPLORATION_REPORT.md Section 1-2 (15 min)
3. Reference ABSOLUTE_PATHS_REFERENCE.txt as needed (5 min)

### Scenario 2: "I need to fix a coordinate transformation bug"
1. Go to EXPLORATION_REPORT.md Section 4 (15 min)
2. Reference coordinate_transforms.py via ABSOLUTE_PATHS_REFERENCE.txt
3. Review EXPLORATION_REPORT.md Section 7 for known issues

### Scenario 3: "I need to add a new safety check"
1. Read EXPLORATION_REPORT.md Section 6 (10 min)
2. Find safety_monitor_node.py via ABSOLUTE_PATHS_REFERENCE.txt
3. Review current checks structure in EXPLORATION_REPORT.md Section 6

### Scenario 4: "I need to understand the data flow"
1. Look at ARCHITECTURE_VISUAL.txt (Data Flow Diagram) (5 min)
2. Read EXPLORATION_REPORT.md Section 5 (ROS Topics) (10 min)
3. Reference specific node files via ABSOLUTE_PATHS_REFERENCE.txt

## Cross-References

### Coordinate System
- Explained in: EXPLORATION_REPORT.md Section 4
- Visualized in: ARCHITECTURE_VISUAL.txt (Coordinate System diagram)
- Code: `/utils/coordinate_transforms.py` (ABSOLUTE_PATHS_REFERENCE.txt)

### Waypoint Processing
- Explained in: EXPLORATION_REPORT.md Section 3
- Flow diagram: ARCHITECTURE_VISUAL.txt (Waypoint/Goal Tracking)
- Code: `ai_adapter_node.py` + `fc_comms_node.py`

### Return-to-Home
- Explained in: EXPLORATION_REPORT.md Section 6
- Flow diagram: ARCHITECTURE_VISUAL.txt (Safety Monitor & RTH)
- Code: `safety_monitor_node.py` + `fc_adapter_node.py`

### 131-D Observation
- Structure: ARCHITECTURE_VISUAL.txt (Observation Vector table)
- Details: EXPLORATION_REPORT.md (end of file)
- Code: `utils/observation_builder.py`

## File Locations Summary

**Generated Files (in project root):**
- `/EXPLORATION_REPORT.md` - Main technical report
- `/ARCHITECTURE_VISUAL.txt` - ASCII diagrams
- `/ABSOLUTE_PATHS_REFERENCE.txt` - File locations
- `/EXPLORATION_INDEX.md` - This file

**Original Project Files:**
- `/src/swarm_ai_integration/config/swarm_params.yaml` - All parameters
- `/src/swarm_ai_integration/launch/swarm_ai_launch.py` - Node startup
- `/src/swarm_ai_integration/swarm_ai_integration/*.py` - ROS nodes
- `/src/swarm_ai_integration/swarm_ai_integration/utils/*.py` - Utilities
- `/README.md` - Original project README

## Version Information

- **Exploration Date:** October 27, 2025
- **Project Branch:** v0.1.6
- **Thoroughness:** Medium
- **Time Invested:** ~2 hours comprehensive analysis
- **Files Analyzed:** 35+ Python files, 1 YAML, 1 shell script, 8+ markdown docs

## Next Steps Recommendations

### For Development:
1. Review EXPLORATION_REPORT.md Section 7 (known issues)
2. Apply coordinate frame bug fix (documented in flight-logs/COORDINATE_FRAME_BUG_AND_FIX.md)
3. Test with simulator before real flight

### For Deployment:
1. Verify all parameters in swarm_params.yaml
2. Check serial port configuration (/dev/ttyAMA0)
3. Verify I2C bus and LiDAR addresses
4. Test GPS lock and averaging (observe logs)

### For Understanding:
1. Trace data flow through ARCHITECTURE_VISUAL.txt
2. Read EXPLORATION_REPORT.md relevant sections
3. Review actual code files in order of data flow

## Contact & Issues

**Known Issues:**
- ENU vs body frame mismatch (documented, fix available)
- See EXPLORATION_REPORT.md Section 7

**Documentation:**
- flight-logs/COORDINATE_FRAME_BUG_AND_FIX.md
- flight-logs/FLIGHT_ANALYSIS_REPORT.md
- README.md (original)

---

**Generated by:** Claude Code Exploration Tool
**Format:** Markdown + ASCII + Text
**Completeness:** 6/6 requested areas covered
**Recommendation:** Start with ARCHITECTURE_VISUAL.txt for quick understanding, then EXPLORATION_REPORT.md for details.
