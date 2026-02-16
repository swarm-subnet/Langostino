#!/usr/bin/env python3
"""
Pure-Python CRUISE adapter test model (no ROS2 dependency).

This mirrors the RC mapping logic in fc_adapter_node_cruise.py for
offline action-to-RC validation.
"""

from __future__ import annotations

import math
import time
from typing import List, Sequence


class FCAdapterNodeCruiseTest:
    """
    Minimal test adapter that emulates fc_adapter_node_cruise behavior
    without ROS publishers/subscribers.
    """

    def __init__(self):
        # Parameters aligned with fc_adapter_node_cruise defaults.
        self.cmd_timeout = 1.0
        self.rc_mid = 1500
        self.rc_min = 1000
        self.rc_max = 2000
        self.arming_duration = 20.0
        self.rise_throttle = 1600
        self.rise_target_altitude = 3.0
        self.rise_max_duration = 20.0
        self.lidar_missing_timeout = 1.0
        self.post_rise_hover_sec = 1.0
        self.prep_altitude_min = 2.5
        self.prep_altitude_max = 3.5
        self.prep_throttle_up = 1550
        self.prep_throttle_down = 1450
        self.speed_limit_cms = 300.0
        self.nav_manual_speed_cms = 300.0
        self.nav_mc_manual_climb_rate_cms = 300.0

        # Heading hold behavior aligned with tested adapter.
        self.yaw_right_value = 1520
        self.yaw_left_value = 1480
        self.heading_tolerance_low = 350.0
        self.heading_tolerance_high = 10.0

        # State.
        self.last_action = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_time = time.time()
        self.safety_override = False
        self.current_heading_deg = 0.0
        self.lidar_altitude = None
        self.last_lidar_update_time = None

        # Startup phase flags (tests can set these directly).
        self.arming_complete = False
        self.arming_start_time = time.time()
        self.rise_complete = False
        self.rise_start_time = None
        self.yaw_alignment_complete = False
        self.post_rise_hover_until = None

        self.fatal_error_active = False
        self.fatal_error_reason = ''

        self.last_rc_command: List[int] = []

    def set_ai_action(self, action: Sequence[float]):
        if len(action) < 4:
            return
        self.last_action = [
            self._clamp_unit(self._safe_float(action[0])),
            self._clamp_unit(self._safe_float(action[1])),
            self._clamp_unit(self._safe_float(action[2])),
            self._clamp_unit(self._safe_float(action[3])),
        ]
        self.last_cmd_time = time.time()

    def set_attitude_degrees(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        _ = roll_deg
        _ = pitch_deg
        self.current_heading_deg = self._safe_float(yaw_deg)

    def set_lidar_distance(self, altitude_m: float):
        altitude = self._safe_float(altitude_m)
        if not math.isfinite(altitude) or altitude < 0.0:
            self.lidar_altitude = None
            return
        self.lidar_altitude = altitude
        self.last_lidar_update_time = time.time()

    def control_loop(self) -> List[int]:
        now = time.time()

        if self.fatal_error_active:
            return self._send_abort_command()

        if not self.arming_complete:
            if (now - self.arming_start_time) >= self.arming_duration:
                self.arming_complete = True
                self.rise_start_time = now
            else:
                return self._send_arming_command()

        if not self.rise_complete:
            lidar_stale = (
                self.last_lidar_update_time is None or
                (now - self.last_lidar_update_time) > self.lidar_missing_timeout
            )
            if lidar_stale:
                self._trigger_fatal_error(
                    'LiDAR data missing/stale during rise '
                    f'(timeout={self.lidar_missing_timeout:.1f}s)'
                )
                return self._send_abort_command()

            rise_elapsed = now - (self.rise_start_time or now)
            target_reached = (
                self.lidar_altitude is not None and
                self.lidar_altitude >= self.rise_target_altitude
            )
            timed_out = rise_elapsed >= self.rise_max_duration

            if target_reached or timed_out:
                self.rise_complete = True
                self.post_rise_hover_until = now + self.post_rise_hover_sec
            else:
                return self._send_rise_command()

        if self.post_rise_hover_until is not None and now < self.post_rise_hover_until:
            return self._send_hover_command(use_altitude_controller=True)
        if not self.yaw_alignment_complete:
            return self._send_hover_command(use_altitude_controller=True)

        if (now - self.last_cmd_time) > self.cmd_timeout:
            return self._send_hover_command()
        if self.safety_override:
            return self._send_hover_command()

        return self._send_action_command()

    def _send_arming_command(self) -> List[int]:
        channels = [
            self.rc_mid,
            self.rc_mid,
            1000,
            self.rc_mid,
            2000,
            1500,
            1000,
            2000,
        ]
        return self._store_and_return(channels)

    def _send_rise_command(self) -> List[int]:
        channels = [
            self.rc_mid,
            self.rc_mid,
            self.rise_throttle,
            self.rc_mid,
            2000,
            1500,
            1500,
            2000,
        ]
        return self._store_and_return(channels)

    def _compute_preparation_throttle(self, default_throttle: int) -> int:
        throttle_cmd = int(default_throttle)
        if self.lidar_altitude is None:
            return throttle_cmd
        if self.last_lidar_update_time is None:
            return throttle_cmd
        if (time.time() - self.last_lidar_update_time) > self.lidar_missing_timeout:
            return throttle_cmd
        if self.lidar_altitude > self.prep_altitude_max:
            return self.prep_throttle_down
        if self.lidar_altitude < self.prep_altitude_min:
            return self.prep_throttle_up
        return throttle_cmd

    def _send_hover_command(self, use_altitude_controller: bool = False) -> List[int]:
        throttle_cmd = self.rc_mid
        if use_altitude_controller:
            throttle_cmd = self._compute_preparation_throttle(default_throttle=self.rc_mid)
        channels = [
            self.rc_mid,
            self.rc_mid,
            throttle_cmd,
            self.rc_mid,
            2000,
            1500,
            1500,
            2000,
        ]
        return self._store_and_return(channels)

    def _send_abort_command(self) -> List[int]:
        channels = [
            self.rc_mid,
            self.rc_mid,
            1000,
            self.rc_mid,
            1000,
            1500,
            1000,
            2000,
        ]
        return self._store_and_return(channels)

    def _trigger_fatal_error(self, reason: str):
        if self.fatal_error_active:
            return
        self.fatal_error_active = True
        self.fatal_error_reason = reason

    def _send_action_command(self) -> List[int]:
        ax, ay, az, speed_fraction = self.last_action

        ax = self._clamp_unit(ax)
        ay = self._clamp_unit(ay)
        az = self._clamp_unit(az)
        speed_fraction = self._clamp_unit(speed_fraction)

        dir_x, dir_y, dir_z = self._normalize_direction_l2(ax, ay, az)
        speed_abs = abs(speed_fraction)

        target_speed_cms = self.speed_limit_cms * speed_abs
        v_east_cms = dir_x * target_speed_cms
        v_north_cms = dir_y * target_speed_cms
        v_up_cms = dir_z * target_speed_cms

        v_forward_cms, v_right_cms = self._earth_to_body_horizontal(
            v_east_cms,
            v_north_cms,
            self.current_heading_deg,
        )

        pitch_norm = self._safe_ratio(v_forward_cms, self.nav_manual_speed_cms)
        roll_norm = self._safe_ratio(v_right_cms, self.nav_manual_speed_cms)
        throttle_norm = self._safe_ratio(v_up_cms, self.nav_mc_manual_climb_rate_cms)

        roll_rc = self._map_norm_to_rc(roll_norm)
        pitch_rc = self._map_norm_to_rc(pitch_norm)
        throttle_rc = self._map_norm_to_rc(throttle_norm)
        yaw_rc = self._heading_hold_yaw_command(self.current_heading_deg)

        channels = [
            roll_rc,
            pitch_rc,
            throttle_rc,
            yaw_rc,
            2000,
            1500,
            1500,
            2000,
        ]
        return self._store_and_return(channels)

    def _heading_hold_yaw_command(self, heading_deg: float) -> int:
        if self._is_heading_aligned(heading_deg):
            return self.rc_mid
        if heading_deg > 180.0:
            return self.yaw_right_value
        return self.yaw_left_value

    def _is_heading_aligned(self, heading_deg: float) -> bool:
        return heading_deg >= self.heading_tolerance_low or heading_deg <= self.heading_tolerance_high

    def _normalize_direction_l2(self, x: float, y: float, z: float):
        norm = math.sqrt((x * x) + (y * y) + (z * z))
        if norm == 0.0:
            return 0.0, 0.0, 0.0
        inv_norm = 1.0 / norm
        return x * inv_norm, y * inv_norm, z * inv_norm

    @staticmethod
    def _earth_to_body_horizontal(v_east_cms: float, v_north_cms: float, yaw_deg: float):
        yaw_rad = math.radians(yaw_deg)
        sin_yaw = math.sin(yaw_rad)
        cos_yaw = math.cos(yaw_rad)
        v_forward_cms = (v_east_cms * sin_yaw) + (v_north_cms * cos_yaw)
        v_right_cms = (v_east_cms * cos_yaw) - (v_north_cms * sin_yaw)
        return v_forward_cms, v_right_cms

    def _safe_ratio(self, num: float, den: float) -> float:
        if abs(den) <= 1e-6:
            return 0.0
        return self._clamp_unit(num / den)

    def _map_norm_to_rc(self, value: float) -> int:
        value = self._clamp_unit(value)
        if value >= 0.0:
            span = self.rc_max - self.rc_mid
        else:
            span = self.rc_mid - self.rc_min
        rc = self.rc_mid + int(round(value * span))
        return max(self.rc_min, min(self.rc_max, rc))

    def _store_and_return(self, channels: List[int]) -> List[int]:
        self.last_rc_command = [int(ch) for ch in channels]
        return self.last_rc_command

    @staticmethod
    def _safe_float(value) -> float:
        try:
            out = float(value)
        except Exception:
            return 0.0
        if math.isnan(out) or math.isinf(out):
            return 0.0
        return out

    @staticmethod
    def _clamp_unit(value: float) -> float:
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value
