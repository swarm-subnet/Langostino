#!/usr/bin/env python3
"""
Yaw Alignment Utility

Provides functionality to align drone heading to north (0 degrees) after arming.

Sequence:
1. Rise drone with ALT HOLD for 2 seconds (throttle at 1550)
2. Adjust yaw to face north (heading 0°)
   - If heading is 10°-180°: turn left (counter-clockwise) to reach 0°
   - If heading is 180°-350°: turn right (clockwise) to reach 0°
3. Return control to AI model

This function is designed to be called after the arm phase is complete.
"""

import time
import math
from typing import Optional, Callable
from rclpy.node import Node


class YawAlignmentController:
    """
    Controls the yaw alignment sequence for centering drone to face north.

    This controller executes a two-phase sequence:
    1. Rise phase: Drone rises with throttle at 1550 for 2 seconds
    2. Yaw alignment phase: Adjust yaw to face north (0°) within tolerance
    """

    def __init__(
        self,
        node: Node,
        send_rc_command_callback: Callable,
        get_heading_callback: Callable[[], float],
        rise_duration: float = 3.0,
        rise_throttle: int = 1550,
        yaw_turn_duration: float = 0.1,
        yaw_right_value: int = 1520,
        yaw_left_value: int = 1480,
        heading_tolerance_low: float = 350.0,
        heading_tolerance_high: float = 10.0
    ):
        """
        Initialize yaw alignment controller.

        Args:
            node: ROS2 node for logging
            send_rc_command_callback: Function to send RC commands to drone
                                     Expected signature: func(roll, pitch, throttle, yaw)
            get_heading_callback: Function to get current heading in degrees (0-360)
            rise_duration: Duration to rise in seconds (default: 3.0)
            rise_throttle: Throttle value during rise (default: 1550)
            yaw_turn_duration: Duration to apply yaw correction in seconds (default: 0.1)
            yaw_right_value: Yaw channel value for turning right (default: 1520)
            yaw_left_value: Yaw channel value for turning left (default: 1480)
            heading_tolerance_low: Lower heading tolerance in degrees (default: 350)
            heading_tolerance_high: Upper heading tolerance in degrees (default: 10)
        """
        self.node = node
        self.send_rc_command = send_rc_command_callback
        self.get_heading = get_heading_callback

        # Configuration parameters
        self.rise_duration = rise_duration
        self.rise_throttle = rise_throttle
        self.yaw_turn_duration = yaw_turn_duration
        self.yaw_right_value = yaw_right_value
        self.yaw_left_value = yaw_left_value
        self.heading_tolerance_low = heading_tolerance_low
        self.heading_tolerance_high = heading_tolerance_high

        # State tracking
        self.alignment_complete = False
        self.rise_complete = False

    def is_heading_aligned(self, heading_deg: float) -> bool:
        """
        Check if heading is within tolerance of north (0°).

        Args:
            heading_deg: Current heading in degrees (0-360)

        Returns:
            True if heading is between 350-360° or 0-10°
        """
        return heading_deg >= self.heading_tolerance_low or heading_deg <= self.heading_tolerance_high

    def execute_rise_phase(self):
        """
        Execute rise phase: rise drone with ALT HOLD for specified duration.

        During this phase:
        - Throttle: 1550
        - Roll/Pitch/Yaw: 1500 (neutral)
        - ALT HOLD: enabled
        """
        self.node.get_logger().info(
            f'🚁 Starting RISE phase: throttle={self.rise_throttle} for {self.rise_duration}s'
        )

        start_time = time.time()

        while (time.time() - start_time) < self.rise_duration:
            # Send rise command: neutral stick positions, elevated throttle
            self.send_rc_command(
                roll=1500,
                pitch=1500,
                throttle=self.rise_throttle,
                yaw=1500
            )
            time.sleep(0.025)  # 40Hz control rate

        self.rise_complete = True
        self.node.get_logger().info('✅ RISE phase complete')

    def execute_yaw_alignment_phase(self, max_iterations: int = 30) -> bool:
        """
        Execute yaw alignment phase: adjust yaw to face north.

        This phase will adjust yaw in increments until heading is aligned
        or max iterations is reached.

        Args:
            max_iterations: Maximum number of yaw correction attempts (default: 30)

        Returns:
            True if alignment successful, False if max iterations reached
        """
        self.node.get_logger().info('🧭 Starting YAW ALIGNMENT phase')

        # Send initial neutral command to maintain RC link during setup
        self.send_rc_command(roll=1500, pitch=1500, throttle=1500, yaw=1500)

        iteration = 0

        while iteration < max_iterations:
            heading_deg = self.get_heading()

            # Log current heading value received from callback
            self.node.get_logger().debug(
                f'🧭 Yaw alignment iteration {iteration}: heading={heading_deg:.1f}°'
            )

            # Check if already aligned
            if self.is_heading_aligned(heading_deg):
                self.node.get_logger().info(
                    f'✅ YAW ALIGNMENT complete: heading={heading_deg:.1f}° (target: north/0°)'
                )
                self.alignment_complete = True
                # Send final neutral command before returning to maintain RC link
                self.send_rc_command(roll=1500, pitch=1500, throttle=1500, yaw=1500)
                return True

            # Determine yaw correction direction (shortest path to north)
            # For headings > 180°: turn right (clockwise) is shorter
            # For headings < 180°: turn left (counter-clockwise) is shorter
            if heading_deg > 180:
                # Turn right (clockwise) - shortest path for 180-350°
                yaw_command = self.yaw_right_value
                direction = 'right (CW)'
            else:
                # Turn left (counter-clockwise) - shortest path for 10-180°
                yaw_command = self.yaw_left_value
                direction = 'left (CCW)'

            self.node.get_logger().info(
                f'🔄 Adjusting yaw {direction}: heading={heading_deg:.1f}° → '
                f'yaw={yaw_command} for {self.yaw_turn_duration}s'
            )

            # Apply yaw correction for specified duration
            start_time = time.time()
            while (time.time() - start_time) < self.yaw_turn_duration:
                self.send_rc_command(
                    roll=1500,
                    pitch=1500,
                    throttle=1500,  # Neutral throttle (maintain altitude in ALT HOLD)
                    yaw=yaw_command
                )
                time.sleep(0.025)  # 40Hz control rate

            # Brief pause to let drone settle (send neutral commands to maintain RC link)
            settle_start = time.time()
            while (time.time() - settle_start) < 0.5:
                self.send_rc_command(
                    roll=1500,
                    pitch=1500,
                    throttle=1500,
                    yaw=1500  # Return yaw to neutral during settling
                )
                time.sleep(0.025)  # 40Hz control rate

            iteration += 1

        # Max iterations reached without successful alignment
        heading_deg = self.get_heading()
        self.node.get_logger().warn(
            f'⚠️ YAW ALIGNMENT incomplete after {max_iterations} iterations. '
            f'Final heading: {heading_deg:.1f}°'
        )
        # Send final neutral command before returning to maintain RC link
        self.send_rc_command(roll=1500, pitch=1500, throttle=1500, yaw=1500)
        return False

    def execute_full_sequence(self) -> bool:
        """
        Execute the complete yaw alignment sequence.

        Returns:
            True if sequence completed successfully
        """
        self.node.get_logger().info(
            '🎯 Starting YAW ALIGNMENT sequence: rise + align to north'
        )

        # Phase 1: Rise
        self.execute_rise_phase()

        # Phase 2: Yaw alignment
        success = self.execute_yaw_alignment_phase()

        if success:
            self.node.get_logger().info(
                '✅ YAW ALIGNMENT sequence complete - ready for AI control'
            )
        else:
            self.node.get_logger().warn(
                '⚠️ YAW ALIGNMENT sequence finished with incomplete alignment'
            )

        return success

    def reset(self):
        """Reset controller state for reuse."""
        self.alignment_complete = False
        self.rise_complete = False


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
