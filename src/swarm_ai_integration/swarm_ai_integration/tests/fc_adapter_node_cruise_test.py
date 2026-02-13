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
        self.speed_limit_cms = 300.0
        self.nav_manual_speed_cms = 300.0
        self.nav_mc_manual_climb_rate_cms = 300.0
        self.direction_norm_epsilon = 1e-6

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

        # Startup phase flags (tests can set these directly).
        self.arming_complete = False
        self.rise_complete = False
        self.yaw_alignment_complete = False
        self.post_rise_hover_until = None

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

    def control_loop(self) -> List[int]:
        now = time.time()

        # In this offline tester we only emulate command timeout/safety and
        # the AI mapping phase. Startup steps are controlled by flags.
        if not self.arming_complete:
            return self._send_arming_command()
        if not self.rise_complete:
            return self._send_rise_command()
        if self.post_rise_hover_until is not None and now < self.post_rise_hover_until:
            return self._send_hover_command()
        if not self.yaw_alignment_complete:
            return self._send_hover_command()

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
            1600,
            self.rc_mid,
            2000,
            1500,
            1500,
            2000,
        ]
        return self._store_and_return(channels)

    def _send_hover_command(self) -> List[int]:
        channels = [
            self.rc_mid,
            self.rc_mid,
            self.rc_mid,
            self.rc_mid,
            2000,
            1500,
            1500,
            2000,
        ]
        return self._store_and_return(channels)

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
        if norm <= self.direction_norm_epsilon:
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
