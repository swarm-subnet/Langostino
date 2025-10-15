#!/usr/bin/env python3
"""
Test script to verify Body→ENU coordinate transformation.

This script tests the transformation logic independently to ensure
the coordinate frame fix is correct.
"""

import math
import numpy as np


def body_to_enu(vx, vy, vz, yaw_rad):
    """
    Transform body-frame velocities to ENU frame.

    Args:
        vx: Forward velocity (Body-X)
        vy: Right velocity (Body-Y)
        vz: Up velocity (Body-Z)
        yaw_rad: Yaw angle in radians

    Returns:
        (v_east, v_north, v_up) in ENU frame
    """
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    v_east = vx * sin_yaw + vy * cos_yaw
    v_north = vx * cos_yaw - vy * sin_yaw
    v_up = vz

    return v_east, v_north, v_up


def test_yaw_zero():
    """Test transformation at yaw=0 (body-X → North, body-Y → East)."""
    print("\n" + "="*80)
    print("TEST 1: Yaw = 0° (Body-X points North)")
    print("="*80)

    yaw = 0.0

    # Test 1: Pure forward (should go North)
    vx, vy, vz = 1.0, 0.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=0.0, N=+1.0, U=0.0")
    assert abs(E - 0.0) < 1e-6 and abs(N - 1.0) < 1e-6, "❌ FAILED: Forward should go North!"
    print("✓ PASSED: Forward → North")

    # Test 2: Pure right (should go East)
    vx, vy, vz = 0.0, 1.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=+1.0, N=0.0, U=0.0")
    assert abs(E - 1.0) < 1e-6 and abs(N - 0.0) < 1e-6, "❌ FAILED: Right should go East!"
    print("✓ PASSED: Right → East")

    # Test 3: Pure up (should go Up)
    vx, vy, vz = 0.0, 0.0, 1.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=0.0, N=0.0, U=+1.0")
    assert abs(E - 0.0) < 1e-6 and abs(U - 1.0) < 1e-6, "❌ FAILED: Up should stay Up!"
    print("✓ PASSED: Up → Up")

    # Test 4: Diagonal (forward-right, should go North-East)
    vx, vy, vz = 1.0, 1.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=+1.0, N=+1.0, U=0.0")
    assert abs(E - 1.0) < 1e-6 and abs(N - 1.0) < 1e-6, "❌ FAILED: Forward-Right should go North-East!"
    print("✓ PASSED: Forward-Right → North-East")

    print("\n" + "="*80)
    print("✓ ALL TESTS PASSED for yaw=0°")
    print("="*80)


def test_yaw_90():
    """Test transformation at yaw=90° (body-X → East, body-Y → South)."""
    print("\n" + "="*80)
    print("TEST 2: Yaw = 90° (Body-X points East)")
    print("="*80)

    yaw = math.pi / 2  # 90 degrees

    # Test 1: Pure forward (should go East)
    vx, vy, vz = 1.0, 0.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=+1.0, N≈0.0, U=0.0")
    assert abs(E - 1.0) < 1e-6 and abs(N - 0.0) < 1e-6, "❌ FAILED: Forward should go East at yaw=90°!"
    print("✓ PASSED: Forward → East")

    # Test 2: Pure right (should go South)
    vx, vy, vz = 0.0, 1.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E≈0.0, N=-1.0, U=0.0")
    assert abs(E - 0.0) < 1e-6 and abs(N - (-1.0)) < 1e-6, "❌ FAILED: Right should go South at yaw=90°!"
    print("✓ PASSED: Right → South")

    print("\n" + "="*80)
    print("✓ ALL TESTS PASSED for yaw=90°")
    print("="*80)


def test_actual_model_output():
    """Test the actual model output from the logs."""
    print("\n" + "="*80)
    print("TEST 3: Actual Model Output (from logs)")
    print("="*80)

    # From the logs: vx=+0.064, vy=+0.128, vz=+0.967
    vx, vy, vz = 0.064, 0.128, 0.967
    yaw = 0.0  # Simulator uses yaw=0

    E, N, U = body_to_enu(vx, vy, vz, yaw)

    print(f"\nGoal position: (5, 5, 3) in ENU")
    print(f"Start position: (0, 0, 3) in ENU")
    print(f"Required direction: +East, +North")
    print(f"\nModel output (Body Frame):")
    print(f"  vx={vx:+.3f} (forward)")
    print(f"  vy={vy:+.3f} (right)")
    print(f"  vz={vz:+.3f} (up)")
    print(f"\nAfter transformation (ENU Frame):")
    print(f"  E={E:+.3f} (should be positive - moving East)")
    print(f"  N={N:+.3f} (should be positive - moving North)")
    print(f"  U={U:+.3f} (should be positive - moving Up)")

    if E > 0 and N > 0 and U > 0:
        print("\n✓ CORRECT: Drone will move toward goal at (5, 5, 3)")
    else:
        print("\n❌ ERROR: Drone will move away from goal!")

    print("="*80)


def test_inverse_scenarios():
    """Test backward and left movements."""
    print("\n" + "="*80)
    print("TEST 4: Backward and Left Movements (yaw=0)")
    print("="*80)

    yaw = 0.0

    # Test 1: Pure backward (should go South)
    vx, vy, vz = -1.0, 0.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=0.0, N=-1.0, U=0.0")
    assert abs(E - 0.0) < 1e-6 and abs(N - (-1.0)) < 1e-6, "❌ FAILED: Backward should go South!"
    print("✓ PASSED: Backward → South")

    # Test 2: Pure left (should go West)
    vx, vy, vz = 0.0, -1.0, 0.0
    E, N, U = body_to_enu(vx, vy, vz, yaw)
    print(f"\nBody: vx={vx:+.1f}, vy={vy:+.1f}, vz={vz:+.1f}")
    print(f"ENU:  E={E:+.1f}, N={N:+.1f}, U={U:+.1f}")
    print(f"Expected: E=-1.0, N=0.0, U=0.0")
    assert abs(E - (-1.0)) < 1e-6 and abs(N - 0.0) < 1e-6, "❌ FAILED: Left should go West!"
    print("✓ PASSED: Left → West")

    print("\n" + "="*80)
    print("✓ ALL TESTS PASSED for inverse movements")
    print("="*80)


if __name__ == "__main__":
    print("\n" + "█"*80)
    print("█" + " "*78 + "█")
    print("█" + "  COORDINATE TRANSFORMATION TEST SUITE".center(78) + "█")
    print("█" + " "*78 + "█")
    print("█"*80)

    try:
        test_yaw_zero()
        test_yaw_90()
        test_actual_model_output()
        test_inverse_scenarios()

        print("\n" + "█"*80)
        print("█" + " "*78 + "█")
        print("█" + "  ✓ ALL TESTS PASSED!".center(78) + "█")
        print("█" + "  The coordinate transformation is CORRECT.".center(78) + "█")
        print("█" + " "*78 + "█")
        print("█"*80 + "\n")

    except AssertionError as e:
        print("\n" + "█"*80)
        print("█" + " "*78 + "█")
        print("█" + f"  ❌ TEST FAILED: {str(e)}".center(78) + "█")
        print("█" + " "*78 + "█")
        print("█"*80 + "\n")
        exit(1)
