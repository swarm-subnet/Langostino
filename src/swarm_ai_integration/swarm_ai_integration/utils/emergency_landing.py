#!/usr/bin/env python3
"""
Emergency Landing Utility

Non-blocking state machine that descends the drone using downward LiDAR
altitude feedback until it reaches ground level, then disarms.
"""

import time
from typing import Optional, Callable
from rclpy.node import Node


class EmergencyLandingController:
    """
    Non-blocking emergency landing controller using LiDAR altitude.

    Reads downward LiDAR distance to determine ground proximity and
    commands a slow descent until the drone reaches the ground, then disarms.

    Phases:
      1. DESCENDING: throttle at descent_throttle in ALT HOLD until LiDAR reads ≤ ground threshold
      2. DISARMING: send disarm command
      3. DONE: landing complete
    """

    # LiDAR sensor minimum range — readings at or below this mean "on the ground"
    GROUND_THRESHOLD_M = 0.10

    # If no LiDAR data arrives for this long during landing, disarm as safety fallback
    LIDAR_TIMEOUT_S = 10.0

    # Absolute maximum landing duration before forced disarm (safety net)
    MAX_LANDING_DURATION_S = 60.0

    def __init__(
        self,
        node: Node,
        send_rc_command_callback: Callable[[list], None],
        get_lidar_altitude_callback: Callable[[], Optional[float]],
        rc_mid: int = 1500,
        descent_throttle: int = 1450,
    ):
        """
        Initialize emergency landing controller.

        Args:
            node: ROS2 node for logging
            send_rc_command_callback: Function to publish an 8-channel RC array
            get_lidar_altitude_callback: Function returning current LiDAR distance
                                         in meters, or None if no data available
            rc_mid: RC neutral/center value
            descent_throttle: Throttle value for descent (below rc_mid in ALT HOLD)
        """
        self.node = node
        self.send_rc_command = send_rc_command_callback
        self.get_lidar_altitude = get_lidar_altitude_callback

        self.rc_mid = rc_mid
        self.descent_throttle = descent_throttle

        # State
        self.phase: str = 'idle'  # idle -> descending -> disarming -> done
        self.start_time: Optional[float] = None
        self.last_lidar_time: Optional[float] = None
        self.last_log_time: float = 0.0

    def start(self):
        """Begin emergency landing sequence."""
        if self.phase != 'idle':
            return
        now = time.time()
        self.phase = 'descending'
        self.start_time = now
        self.last_lidar_time = now  # Assume LiDAR was alive when we started
        self.last_log_time = 0.0
        self.node.get_logger().error(
            '🛬 Emergency landing started — descending using LiDAR altitude'
        )

    def tick(self) -> bool:
        """
        Advance the landing state machine by one control-loop tick.

        Returns:
            True if landing is finished (disarmed), otherwise False.
        """
        if self.phase == 'idle':
            return False

        if self.phase == 'done':
            return True

        now = time.time()
        elapsed = now - (self.start_time or now)

        # Safety net: forced disarm after MAX_LANDING_DURATION_S
        if elapsed >= self.MAX_LANDING_DURATION_S:
            self.node.get_logger().error(
                f'⚠️ Landing timeout ({self.MAX_LANDING_DURATION_S:.0f}s) — forcing disarm'
            )
            self._disarm()
            return True

        if self.phase == 'descending':
            altitude = self.get_lidar_altitude()

            if altitude is not None:
                self.last_lidar_time = now

                # Check if we reached the ground
                if altitude <= self.GROUND_THRESHOLD_M:
                    self.node.get_logger().info(
                        f'✅ Ground reached (LiDAR={altitude:.2f}m) — disarming'
                    )
                    self._disarm()
                    return True

                # Log progress
                if (now - self.last_log_time) >= 2.0:
                    self.node.get_logger().info(
                        f'🛬 Descending: LiDAR={altitude:.2f}m, '
                        f'throttle={self.descent_throttle}, '
                        f'elapsed={elapsed:.1f}s'
                    )
                    self.last_log_time = now
            else:
                # No LiDAR data
                lidar_age = now - (self.last_lidar_time or now)
                if lidar_age >= self.LIDAR_TIMEOUT_S:
                    self.node.get_logger().error(
                        f'⚠️ No LiDAR data for {lidar_age:.1f}s during landing — forcing disarm'
                    )
                    self._disarm()
                    return True

                if (now - self.last_log_time) >= 2.0:
                    self.node.get_logger().warn(
                        f'🛬 Descending (no LiDAR): throttle={self.descent_throttle}, '
                        f'elapsed={elapsed:.1f}s'
                    )
                    self.last_log_time = now

            # Send descent command: neutral roll/pitch/yaw, low throttle, POSHOLD + ANGLE
            self.send_rc_command([
                self.rc_mid,           # CH1: ROLL (neutral)
                self.rc_mid,           # CH2: PITCH (neutral)
                self.descent_throttle, # CH3: THROTTLE (below mid → descend in ALT HOLD)
                self.rc_mid,           # CH4: YAW (neutral)
                2000,                  # CH5: ARM (stay armed during descent)
                1500,                  # CH6: ANGLE mode
                1500,                  # CH7: NAV POSHOLD
                2000,                  # CH8: MSP RC OVERRIDE
            ])
            return False

        # phase == 'disarming' (already handled inline)
        return True

    def _disarm(self):
        """Send disarm command and transition to done."""
        self.send_rc_command([
            self.rc_mid,  # CH1: ROLL (neutral)
            self.rc_mid,  # CH2: PITCH (neutral)
            1000,         # CH3: THROTTLE (low)
            self.rc_mid,  # CH4: YAW (neutral)
            1000,         # CH5: ARM (low = disarmed)
            1500,         # CH6: ANGLE mode
            1000,         # CH7: NAV modes off
            2000,         # CH8: MSP RC OVERRIDE
        ])
        elapsed = time.time() - (self.start_time or 0)
        self.node.get_logger().error(
            f'🛑 DISARMED — emergency landing complete ({elapsed:.1f}s)'
        )
        self.phase = 'done'

    def cancel(self):
        """Cancel landing and return to idle."""
        if self.phase != 'done':
            self.node.get_logger().info('Emergency landing cancelled')
        self.phase = 'idle'
        self.start_time = None
        self.last_lidar_time = None
        self.last_log_time = 0.0

    @property
    def is_active(self) -> bool:
        """True if landing is in progress (descending, not idle/done)."""
        return self.phase == 'descending'

    @property
    def is_done(self) -> bool:
        """True if landing completed (disarmed)."""
        return self.phase == 'done'
