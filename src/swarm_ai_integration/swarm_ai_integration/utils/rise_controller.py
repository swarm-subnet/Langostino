#!/usr/bin/env python3
"""
Rise Controller Utility

Non-blocking state machine to lift the drone to a target altitude after arming.

Sequence:
  BURST (1 s at rise_throttle) → WAIT (2 s at neutral, check LiDAR)
  → if out of band: TICK (0.5 s at 1600 or 1400) → WAIT → check → …
  → exits when altitude is inside [target − band, target + band] or max_ticks reached
"""

import time
from typing import Callable, Optional

from rclpy.node import Node


class RiseController:
    """
    Non-blocking LiDAR-guided rise controller.

    Phases: idle → burst → wait → (tick → wait) × N → done

    Usage (same pattern as YawAlignmentController):
        controller.start_sequence()
        # in control loop:
        if controller.tick():
            # rise complete
    """

    def __init__(
        self,
        node: Node,
        send_rc_callback: Callable[[int], None],
        get_lidar_altitude_callback: Callable[[], Optional[float]],
        rise_throttle: int = 1600,
        target_altitude_m: float = 3.0,
        band_m: float = 0.5,
        burst_sec: float = 1.0,
        tick_sec: float = 0.5,
        wait_sec: float = 2.0,
        max_ticks: int = 10,
    ):
        """
        Args:
            node: ROS2 node (used for logging only).
            send_rc_callback: Called with a single throttle int. The node builds
                              the full channel array; the controller only decides
                              what throttle value to apply.
            get_lidar_altitude_callback: Returns current LiDAR altitude in metres
                                         (or None if unavailable).
            rise_throttle: Throttle value for climbing (default 1600).
            target_altitude_m: Target altitude in metres (default 3.0).
            band_m: Half-width of the acceptance band (default 0.5 m → 2.5–3.5 m).
            burst_sec: Duration of the initial lift burst (default 1.0 s).
            tick_sec: Duration of each correction tick (default 0.5 s).
            wait_sec: Settle time between ticks (default 2.0 s).
            max_ticks: Safety limit on correction ticks before forcing done (default 10).
        """
        self.node = node
        self._send_rc = send_rc_callback
        self._get_altitude = get_lidar_altitude_callback

        self.rise_throttle = rise_throttle
        self.target_altitude_m = target_altitude_m
        self.band_m = band_m
        self.burst_sec = burst_sec
        self.tick_sec = tick_sec
        self.wait_sec = wait_sec
        self.max_ticks = max_ticks

        # State
        self.phase: str = 'idle'          # idle | burst | wait | tick | done
        self.tick_count: int = 0
        self._tick_throttle: int = rise_throttle
        self._phase_start: Optional[float] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start_sequence(self):
        """Begin the rise sequence (must be called once before ticking)."""
        if self.phase != 'idle':
            return
        self.phase = 'burst'
        self._phase_start = time.time()
        self.node.get_logger().info(
            f'🚀 Rise started: burst {self.burst_sec}s @ {self.rise_throttle}'
        )

    def tick(self) -> bool:
        """
        Advance the rise state machine by one control-loop tick.

        Returns:
            True when the rise is complete (altitude in band or max ticks reached).
        """
        if self.phase in ('idle', 'done'):
            return self.phase == 'done'

        now = time.time()
        elapsed = now - (self._phase_start or now)
        alt = self._get_altitude()

        if self.phase == 'burst':
            self._send_rc(self.rise_throttle)
            if elapsed >= self.burst_sec:
                alt_str = f'{alt:.2f}m' if alt is not None else 'no data'
                self.node.get_logger().info(
                    f'Rise burst done → wait (alt={alt_str})'
                )
                self._enter_wait(now)

        elif self.phase == 'wait':
            self._send_rc(1500)  # neutral — let the drone settle
            if elapsed >= self.wait_sec:
                self._evaluate(alt, now)

        elif self.phase == 'tick':
            self._send_rc(self._tick_throttle)
            if elapsed >= self.tick_sec:
                self.node.get_logger().info(
                    f'Rise tick #{self.tick_count} done → wait'
                )
                self._enter_wait(now)

        return self.phase == 'done'

    def get_status(self) -> str:
        """Short string suitable for the status topic (e.g. 'burst', 'wait tick=2')."""
        if self.phase in ('burst', 'done', 'idle'):
            return self.phase
        return f'{self.phase} tick={self.tick_count}'

    def reset(self):
        """Reset to idle for reuse."""
        self.phase = 'idle'
        self.tick_count = 0
        self._tick_throttle = self.rise_throttle
        self._phase_start = None

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _enter_wait(self, now: float):
        self.phase = 'wait'
        self._phase_start = now

    def _evaluate(self, alt: Optional[float], now: float):
        """After a wait, decide: done, tick up, or tick down."""
        low = self.target_altitude_m - self.band_m
        high = self.target_altitude_m + self.band_m
        alt_str = f'{alt:.2f}m' if alt is not None else 'no data'

        if alt is not None and low <= alt <= high:
            self.phase = 'done'
            self.node.get_logger().info(
                f'✅ Rise complete: alt={alt:.2f}m in band [{low:.1f}m, {high:.1f}m]'
            )
            return

        if self.tick_count >= self.max_ticks:
            self.phase = 'done'
            self.node.get_logger().warn(
                f'⚠️ Rise complete (max ticks {self.max_ticks}): alt={alt_str}'
            )
            return

        # Decide correction direction
        if alt is None or alt < low:
            self._tick_throttle = self.rise_throttle  # climb
        else:
            self._tick_throttle = 1400  # descend

        self.tick_count += 1
        self.phase = 'tick'
        self._phase_start = now
        self.node.get_logger().info(
            f'Rise tick #{self.tick_count}: throttle={self._tick_throttle}'
            f' (alt={alt_str}, target={low:.1f}–{high:.1f}m)'
        )
