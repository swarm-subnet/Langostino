#!/usr/bin/env python3
"""
RC mapping test for fc_adapter_node_cruise (offline / no ROS2 dependency).

This script feeds sample /ai/action commands into a pure-Python adapter model
and prints the exact /fc/rc_override frame produced.

It prints:
1) Startup/fatal behavior RC frames
2) AI action mapping RC frames

Base assumptions:
- Drone heading is North (yaw = 0 deg)

Run:
  python3 src/swarm_ai_integration/swarm_ai_integration/tests/fc_adapter_node_cruise_action_to_rc_test.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import List, Tuple

# Make local test module import work when running directly from repo root.
TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

from fc_adapter_node_cruise_test import FCAdapterNodeCruiseTest


def _print_table_header(title: str):
    print()
    print(title)
    print('-' * 110)
    print(f"{'Sample':<20} {'Input':<30} RC Override")
    print('-' * 110)


def run_startup_and_safety_samples():
    adapter = FCAdapterNodeCruiseTest()
    adapter.set_attitude_degrees(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)  # North

    _print_table_header('Startup/Fatal Samples')

    # 1) Arming phase command
    rc = adapter.control_loop()
    print(f"{'arming_phase':<20} {'n/a':<30} {str(rc)}")

    # 2) Rise command below target altitude
    adapter.arming_complete = True
    adapter.rise_start_time = time.time()
    adapter.set_lidar_distance(altitude_m=2.5)
    rc = adapter.control_loop()
    print(f"{'rise_below_target':<20} {'lidar=2.5m':<30} {str(rc)}")

    # 3) Reach target altitude: transition to post-rise stabilize (hover)
    adapter.set_lidar_distance(altitude_m=3.0)
    rc = adapter.control_loop()
    print(f"{'rise_reach_3m':<20} {'lidar=3.0m':<30} {str(rc)}")

    # 4) LiDAR missing/stale during rise -> fatal abort/disarm command
    adapter2 = FCAdapterNodeCruiseTest()
    adapter2.set_attitude_degrees(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
    adapter2.arming_complete = True
    adapter2.rise_start_time = time.time()
    rc = adapter2.control_loop()
    print(f"{'rise_lidar_missing':<20} {'no lidar sample':<30} {str(rc)}")

    # 5) Fatal state is latched: subsequent ticks keep abort command
    rc = adapter2.control_loop()
    print(f"{'fatal_latched':<20} {'next tick':<30} {str(rc)}")

    print('-' * 110)


def run_action_mapping_samples(samples: List[Tuple[str, List[float]]]):
    adapter = FCAdapterNodeCruiseTest()

    # Force action-mapping assumptions.
    adapter.set_attitude_degrees(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)  # North
    adapter.set_lidar_distance(altitude_m=3.0)                               # 3 meters

    # Skip startup phases so control_loop goes directly into AI action mapping.
    adapter.arming_complete = True
    adapter.rise_complete = True
    adapter.yaw_alignment_complete = True
    adapter.post_rise_hover_until = None
    adapter.safety_override = False

    print('Assumptions for action mapping: yaw=0.0 deg (North), lidar=3.0 m, startup phases complete')
    print('Action format: [dir_x, dir_y, dir_z, speed_fraction]')
    print('Note: dir_z is ignored in CRUISE control; throttle is LiDAR-band controlled')
    print('RC format: [roll, pitch, throttle, yaw, arm, angle, nav, msp]')
    _print_table_header('Action Mapping Samples')

    for name, action in samples:
        adapter.set_ai_action(action)

        # Ensure no timeout path is taken.
        adapter.last_cmd_time = time.time()

        rc = adapter.control_loop()
        print(f"{name:<20} {str(action):<30} {str(rc)}")

    print('-' * 110)


def main():
    run_startup_and_safety_samples()

    sample_actions: List[Tuple[str, List[float]]] = [
        ('zero', [0.0, 0.0, 0.0, 0.0]),
        ('east_full', [1.0, 0.0, 0.0, 1.0]),
        ('north_full', [0.0, 1.0, 0.0, 1.0]),
        ('up_full', [0.0, 0.0, 1.0, 1.0]),
        ('down_full', [0.0, 0.0, -1.0, 1.0]),
        ('diag_xy_full', [1.0, 1.0, 0.0, 1.0]),
        ('diag_xy_half', [1.0, 1.0, 0.0, 0.5]),
        ('small_dir_full', [0.2, 0.2, 0.0, 1.0]),
        ('east_neg_speed', [1.0, 0.0, 0.0, -0.7]),
        ('xyz_full', [1.0, 1.0, 1.0, 1.0]),
    ]
    run_action_mapping_samples(sample_actions)


if __name__ == '__main__':
    main()
