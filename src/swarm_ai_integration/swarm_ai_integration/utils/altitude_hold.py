#!/usr/bin/env python3
"""
Altitude Hold Controller — bang-bang controller using LiDAR altitude.

Same pattern as cruise node's _compute_preparation_throttle but as a
reusable class.  Returns a throttle PWM value based on whether the
drone is below, within, or above a target altitude band.
"""

import time


class AltitudeHoldController:
    """
    Bang-bang altitude hold using downward-facing LiDAR.

    Parameters
    ----------
    node : rclpy.node.Node
        ROS node (used only for logging).
    get_lidar_altitude_callback : callable
        Returns current LiDAR altitude in metres (or None).
    target_altitude_m : float
        Desired altitude (default 3.0 m).
    band_m : float
        Half-width of the deadband (default 0.5 m → band is 2.5-3.5 m).
    throttle_up : int
        Throttle value when below the band.
    throttle_down : int
        Throttle value when above the band.
    throttle_neutral : int
        Throttle value when inside the band (or no fresh data).
    lidar_timeout_sec : float
        If LiDAR reading is older than this, treat as stale.
    """

    def __init__(
        self,
        node,
        get_lidar_altitude_callback,
        target_altitude_m=3.0,
        band_m=0.5,
        throttle_up=1600,
        throttle_down=1400,
        throttle_neutral=1500,
        lidar_timeout_sec=1.0,
    ):
        self._node = node
        self._get_altitude = get_lidar_altitude_callback
        self.target_altitude_m = target_altitude_m
        self.band_m = band_m
        self.throttle_up = throttle_up
        self.throttle_down = throttle_down
        self.throttle_neutral = throttle_neutral
        self.lidar_timeout_sec = lidar_timeout_sec

        # Track when we last received a valid altitude
        self._last_altitude_time = 0.0
        self._last_altitude_value = None

    def compute_throttle(self) -> int:
        """Return the appropriate throttle PWM value.

        * Below band  → throttle_up
        * Above band  → throttle_down
        * In band / no fresh LiDAR → throttle_neutral
        """
        altitude = self._get_altitude()
        now = time.time()

        if altitude is not None:
            self._last_altitude_time = now
            self._last_altitude_value = altitude

        # Check for stale data
        if self._last_altitude_value is None or (now - self._last_altitude_time) > self.lidar_timeout_sec:
            self._node.get_logger().warn(
                'Altitude hold: no fresh LiDAR data — returning neutral throttle',
                throttle_duration_sec=2.0,
            )
            return self.throttle_neutral

        alt = self._last_altitude_value
        low = self.target_altitude_m - self.band_m
        high = self.target_altitude_m + self.band_m

        if alt < low:
            throttle = self.throttle_up
        elif alt > high:
            throttle = self.throttle_down
        else:
            throttle = self.throttle_neutral

        self._node.get_logger().info(
            f'Alt hold: alt={alt:.2f}m target={self.target_altitude_m:.1f}m '
            f'band=[{low:.1f},{high:.1f}] → throttle={throttle}',
            throttle_duration_sec=1.0,
        )
        return throttle
