#!/usr/bin/env python3
"""
Yaw Alignment Utility

Non-blocking state machine to align drone heading to north (0 degrees) after arming.
"""

import time
import math
from typing import Optional, Callable
from rclpy.node import Node


class YawAlignmentController:
    """
    Non-blocking yaw alignment controller for centering the drone to face north.

    This runs as a small state machine advanced once per control loop tick.
    Phases:
      1. ALIGN: send yaw left/right until heading is within tolerance or timeout
      2. DONE: neutral command, control returns to main loop
    """

    def __init__(
        self,
        node: Node,
        send_rc_command_callback: Callable,
        get_heading_callback: Callable[[], float],
        yaw_right_value: int = 1520,
        yaw_left_value: int = 1480,
        heading_tolerance_low: float = 350.0,
        heading_tolerance_high: float = 10.0,
        max_align_duration: float = 10.0
    ):
        """
        Initialize yaw alignment controller.

        Args:
            node: ROS2 node for logging
            send_rc_command_callback: Function to send RC commands to drone
                                     Expected signature: func(roll, pitch, throttle, yaw)
            get_heading_callback: Function to get current heading in degrees (0-360)
            yaw_right_value: Yaw channel value for turning right (default: 1520)
            yaw_left_value: Yaw channel value for turning left (default: 1480)
            heading_tolerance_low: Lower heading tolerance in degrees (default: 350)
            heading_tolerance_high: Upper heading tolerance in degrees (default: 10)
            max_align_duration: Max time allowed in ALIGN phase before timeout
        """
        self.node = node
        self.send_rc_command = send_rc_command_callback
        self.get_heading = get_heading_callback

        # Configuration parameters
        self.yaw_right_value = yaw_right_value
        self.yaw_left_value = yaw_left_value
        self.heading_tolerance_low = heading_tolerance_low
        self.heading_tolerance_high = heading_tolerance_high
        self.max_align_duration = max_align_duration

        # State tracking
        self.phase: str = 'idle'  # idle -> align -> done
        self.align_start_time: Optional[float] = None
        self.last_log_time: float = 0.0

    def is_heading_aligned(self, heading_deg: float) -> bool:
        """
        Check if heading is within tolerance of north (0°).

        Args:
            heading_deg: Current heading in degrees (0-360)

        Returns:
            True if heading is between 350-360° or 0-10°
        """
        return heading_deg >= self.heading_tolerance_low or heading_deg <= self.heading_tolerance_high

    def start_sequence(self):
        """Begin the align sequence."""
        if self.phase != 'idle':
            return
        now = time.time()
        self.phase = 'align'
        self.align_start_time = now
        self.last_log_time = 0.0
        self.node.get_logger().info(
            '🎯 Starting yaw alignment: align to north'
        )

    def tick(self) -> bool:
        """
        Advance the alignment state machine by one control-loop tick.

        Returns:
            True if alignment is finished (success or timeout), otherwise False.
        """
        if self.phase == 'idle':
            return False

        now = time.time()

        if self.phase == 'align':
            heading_deg = self.get_heading()
            elapsed = now - (self.align_start_time or now)

            if self.is_heading_aligned(heading_deg):
                self.node.get_logger().info(
                    f'✅ YAW aligned: heading={heading_deg:.1f}° (target north/0°)'
                )
                self.send_rc_command(roll=1500, pitch=1500, throttle=1500, yaw=1500)
                self.phase = 'done'
                return True

            if elapsed > self.max_align_duration:
                self.node.get_logger().warn(
                    f'⚠️ YAW alignment timeout after {elapsed:.1f}s; heading={heading_deg:.1f}°'
                )
                self.send_rc_command(roll=1500, pitch=1500, throttle=1500, yaw=1500)
                self.phase = 'done'
                return True

            # Determine yaw direction (shortest path to north)
            if heading_deg > 180:
                yaw_command = self.yaw_right_value
                direction = 'right (CW)'
            else:
                yaw_command = self.yaw_left_value
                direction = 'left (CCW)'

            # Throttle logging to reduce spam
            if (now - self.last_log_time) >= 0.5:
                self.node.get_logger().info(
                    f'🧭 Aligning yaw {direction}: heading={heading_deg:.1f}° → yaw={yaw_command}'
                )
                self.last_log_time = now

            # Send yaw correction with neutral roll/pitch/throttle (ALT HOLD active)
            self.send_rc_command(roll=1500, pitch=1500, throttle=1500, yaw=yaw_command)
            return False

        # Phase 'done'
        return True

    def reset(self):
        """Reset controller state for reuse."""
        self.phase = 'idle'
        self.align_start_time = None
        self.last_log_time = 0.0


def normalize_heading_to_360(heading_rad: float) -> float:
    """
    Normalize heading from radians to 0-360 degrees.

    DEPRECATED: Use /fc/attitude_degrees topic directly instead of converting
    from /fc/attitude_euler. This function is kept for backward compatibility.

    Args:
        heading_rad: Heading in radians

    Returns:
        Heading in degrees (0-360 range)
    """
    heading_deg = math.degrees(heading_rad)
    # Normalize to 0-360 range
    return heading_deg % 360.0
