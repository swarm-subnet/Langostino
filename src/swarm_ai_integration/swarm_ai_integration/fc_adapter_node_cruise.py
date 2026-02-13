#!/usr/bin/env python3
"""
FC Adapter Node (CRUISE Direction Mode)

Converts /ai/action commands into RC override commands for INAV CRUISE/POSHOLD.

Action semantics:
  /ai/action = [dir_x, dir_y, dir_z, speed_fraction]
  - First 3 entries define direction only (L2-normalized)
  - Fourth entry defines speed magnitude (absolute value)

Control behavior:
  1. Arming phase: arm with throttle low
  2. Rise phase: climb until LiDAR reaches target altitude (default 3.0 m)
  3. Stabilize phase: short neutral hover after rise
  4. Initial yaw alignment phase: rotate to north (0 deg)
  5. AI phase: map direction + speed to body-frame RC commands + yaw hold

RC channel mapping (AETR + AUX):
  CH1: Roll     (body-right velocity request)
  CH2: Pitch    (body-forward velocity request)
  CH3: Throttle (vertical velocity request)
  CH4: Yaw      (startup alignment + heading-hold during AI)
  CH5: ARM
  CH6: ANGLE
  CH7: NAV POSHOLD
  CH8: MSP RC OVERRIDE
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32MultiArray, String

from swarm_ai_integration.utils import YawAlignmentController


class FCAdapterNode(Node):
    """
    FC Adapter Node in CRUISE direction mode.

    Subscriptions:
      /ai/action           (std_msgs/Float32MultiArray): [dir_x, dir_y, dir_z, speed_fraction]
      /safety/override     (std_msgs/Bool)
      /fc/attitude_degrees (geometry_msgs/Vector3Stamped): roll/pitch/yaw in degrees
      /lidar_distance      (sensor_msgs/Range)

    Publications:
      /fc_adapter/status   (std_msgs/String)
      /fc/rc_override      (std_msgs/Float32MultiArray): [ch1..ch8]
    """

    def __init__(self):
        super().__init__('fc_adapter_node_cruise')

        # ------------ Parameters ------------
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('command_timeout', 1.0)

        # RC range (near full authority by default)
        self.declare_parameter('rc_mid_value', 1500)
        self.declare_parameter('rc_min_value', 1000)
        self.declare_parameter('rc_max_value', 2000)
        self.declare_parameter('rc_publish_rate_hz', 20.0)

        # Startup sequence
        self.declare_parameter('arming_duration_sec', 20.0)
        self.declare_parameter('rise_throttle_value', 1600)
        self.declare_parameter('rise_target_altitude_m', 3.0)
        self.declare_parameter('rise_max_duration_sec', 20.0)
        self.declare_parameter('post_rise_hover_sec', 1.0)

        # CRUISE mapping parameters (cm/s)
        self.declare_parameter('speed_limit_cms', 300.0)
        self.declare_parameter('nav_manual_speed_cms', 300.0)
        self.declare_parameter('nav_mc_manual_climb_rate_cms', 300.0)

        # ------------ Parameter values ------------
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout').value)

        self.rc_mid = int(self.get_parameter('rc_mid_value').value)
        self.rc_min = int(self.get_parameter('rc_min_value').value)
        self.rc_max = int(self.get_parameter('rc_max_value').value)
        self.rc_publish_rate_hz = float(self.get_parameter('rc_publish_rate_hz').value)

        self.arming_duration = float(self.get_parameter('arming_duration_sec').value)
        self.rise_throttle = int(self.get_parameter('rise_throttle_value').value)
        self.rise_target_altitude = float(self.get_parameter('rise_target_altitude_m').value)
        self.rise_max_duration = float(self.get_parameter('rise_max_duration_sec').value)
        self.post_rise_hover_sec = float(self.get_parameter('post_rise_hover_sec').value)

        self.speed_limit_cms = float(self.get_parameter('speed_limit_cms').value)
        self.nav_manual_speed_cms = float(self.get_parameter('nav_manual_speed_cms').value)
        self.nav_mc_manual_climb_rate_cms = float(self.get_parameter('nav_mc_manual_climb_rate_cms').value)

        # ------------ State ------------
        self.last_action = [0.0, 0.0, 0.0, 0.0]  # [dir_x, dir_y, dir_z, speed_fraction]
        self.last_cmd_time = time.time()
        self.safety_override = False

        self.current_heading_deg = 0.0
        self.lidar_altitude = None  # meters

        self.arming_complete = False
        self.arming_start_time = time.time()

        self.rise_complete = False
        self.rise_start_time = None
        self.post_rise_hover_until = None

        self.yaw_alignment_started = False
        self.yaw_alignment_complete = False
        self.last_yaw_hold_log_time = 0.0

        self.rc_publish_interval = 1.0 / max(1.0, self.rc_publish_rate_hz)
        self.last_rc_publish_time = 0.0

        self.yaw_controller = YawAlignmentController(
            node=self,
            send_rc_command_callback=self.send_rc_command_for_yaw_alignment,
            get_heading_callback=self.get_current_heading_degrees,
            yaw_right_value=1520,        # Right turn value
            yaw_left_value=1480,         # Left turn value
            heading_tolerance_low=350.0, # Lower tolerance (350°)
            heading_tolerance_high=10.0, # Upper tolerance (10°)
            max_align_duration=30.0      # Timeout for yaw alignment
        )

        # ------------ QoS & ROS I/O ------------
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Float32MultiArray, '/ai/action', self.cb_ai_action, control_qos)
        self.create_subscription(Bool, '/safety/override', self.cb_safety, control_qos)
        self.create_subscription(Vector3Stamped, '/fc/attitude_degrees', self.cb_attitude, sensor_qos)
        self.create_subscription(Range, '/lidar_distance', self.cb_lidar, sensor_qos)
        self.get_logger().info('📡 Subscribed to /fc/attitude_degrees, /lidar_distance')

        self.status_pub = self.create_publisher(String, '/fc_adapter/status', control_qos)
        self.rc_override_pub = self.create_publisher(Float32MultiArray, '/fc/rc_override', control_qos)

        self.control_period = 1.0 / max(1.0, self.control_rate)
        self.create_timer(self.control_period, self.control_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            'FC Adapter Node (CRUISE Direction Mode) started @ '
            f'{self.control_rate:.1f}Hz, RC=[{self.rc_min},{self.rc_mid},{self.rc_max}]'
        )
        self.get_logger().info(
            'Startup sequence: '
            f'arming={self.arming_duration:.1f}s, rise_target={self.rise_target_altitude:.2f}m, '
            f'rise_timeout={self.rise_max_duration:.1f}s'
        )
        self.get_logger().info(
            'CRUISE mapping: '
            f'speed_limit={self.speed_limit_cms:.1f}cm/s, nav_manual_speed={self.nav_manual_speed_cms:.1f}cm/s, '
            f'climb_rate={self.nav_mc_manual_climb_rate_cms:.1f}cm/s'
        )

    # ------------ Callbacks ------------
    def cb_ai_action(self, msg: Float32MultiArray):
        """Receive and sanitize AI action command."""
        data = list(msg.data)
        if len(data) < 4:
            return

        self.last_action = [
            self._clamp_unit(self._safe_float(data[0])),
            self._clamp_unit(self._safe_float(data[1])),
            self._clamp_unit(self._safe_float(data[2])),
            self._clamp_unit(self._safe_float(data[3])),
        ]
        self.last_cmd_time = time.time()

    def cb_safety(self, msg: Bool):
        """Safety override callback."""
        self.safety_override = bool(msg.data)
        if self.safety_override:
            self.get_logger().warn('Safety override ON -> neutral hover command')

    def cb_lidar(self, msg: Range):
        """Receive downward LiDAR range (meters)."""
        try:
            altitude = float(msg.range)
        except Exception:
            self.lidar_altitude = None
            return

        if not math.isfinite(altitude) or altitude < 0.0:
            self.lidar_altitude = None
            return

        self.lidar_altitude = altitude

    def cb_attitude(self, msg: Vector3Stamped):
        """Receive attitude and store yaw heading in degrees."""
        self.current_heading_deg = float(msg.vector.z)  # yaw in degrees
        self.get_logger().info(
            f'📐 Attitude callback: roll={msg.vector.x:.1f}°, pitch={msg.vector.y:.1f}°, '
            f'yaw={msg.vector.z:.1f}°',
            throttle_duration_sec=1.0,
        )

    def get_current_heading_degrees(self):
        """Get current heading in degrees from /fc/attitude_degrees yaw."""
        return self.current_heading_deg

    def send_rc_command_for_yaw_alignment(self, roll: int, pitch: int, throttle: int, yaw: int):
        """
        Send RC command during startup yaw alignment.

        This callback is used by YawAlignmentController.
        """
        channels = [
            roll,      # CH1 ROLL
            pitch,     # CH2 PITCH
            throttle,  # CH3 THROTTLE
            yaw,       # CH4 YAW
            2000,      # CH5 ARM
            1500,      # CH6 ANGLE
            1500,      # CH7 NAV POSHOLD
            2000,      # CH8 MSP override
        ]
        self._publish_rc_override(channels)

    # ------------ Control Loop ------------
    def control_loop(self):
        """Main control loop."""
        now = time.time()

        # Phase 0: Arming
        if not self.arming_complete:
            if (now - self.arming_start_time) >= self.arming_duration:
                self.arming_complete = True
                self.rise_start_time = now
                self.get_logger().info('Arming complete -> rise phase')
            else:
                self._send_arming_command()
                return

        # Phase 1: Rise until target altitude (default 3m) or timeout
        if not self.rise_complete:
            rise_elapsed = now - (self.rise_start_time or now)
            target_reached = (
                self.lidar_altitude is not None and
                self.lidar_altitude >= self.rise_target_altitude
            )
            timed_out = rise_elapsed >= self.rise_max_duration

            if target_reached or timed_out:
                self.rise_complete = True
                self.post_rise_hover_until = now + self.post_rise_hover_sec
                if target_reached:
                    self.get_logger().info(
                        f'Rise complete at {self.lidar_altitude:.2f}m '
                        f'(target {self.rise_target_altitude:.2f}m)'
                    )
                else:
                    alt_str = f'{self.lidar_altitude:.2f}m' if self.lidar_altitude is not None else 'no_lidar'
                    self.get_logger().warn(
                        f'Rise timeout at {rise_elapsed:.1f}s (alt={alt_str}) -> stabilize and continue'
                    )
            else:
                self._send_rise_command()
                return

        # Phase 2: short stabilization hover after rise
        if self.post_rise_hover_until is not None and now < self.post_rise_hover_until:
            self._send_hover_command(reason='post_rise_stabilize')
            return

        # Phase 3: Initial yaw alignment (north-facing model assumption)
        if not self.yaw_alignment_complete:
            if not self.yaw_alignment_started:
                self.yaw_alignment_started = True
                self.yaw_controller.start_sequence()

            alignment_done = self.yaw_controller.tick()
            if alignment_done:
                self.yaw_alignment_complete = True
                self.get_logger().info('✅ Yaw alignment complete - ready for AI control')
            return

        # Timeout and safety behavior during AI phase
        if (now - self.last_cmd_time) > self.cmd_timeout:
            self._send_hover_command(reason='timeout')
            return

        if self.safety_override:
            self._send_hover_command(reason='safety')
            return

        # Phase 4: Normal CRUISE AI control
        self._send_action_command()

    # ------------ Command Builders ------------
    def _send_arming_command(self):
        channels = [
            self.rc_mid,  # CH1 ROLL
            self.rc_mid,  # CH2 PITCH
            1000,         # CH3 THROTTLE low for arming
            self.rc_mid,  # CH4 YAW neutral
            2000,         # CH5 ARM
            1500,         # CH6 ANGLE
            1000,         # CH7 NAV off
            2000,         # CH8 MSP override
        ]
        self._publish_rc_override(channels)

    def _send_rise_command(self):
        channels = [
            self.rc_mid,         # CH1 ROLL neutral
            self.rc_mid,         # CH2 PITCH neutral
            self.rise_throttle,  # CH3 THROTTLE rise
            self.rc_mid,         # CH4 YAW neutral
            2000,                # CH5 ARM
            1500,                # CH6 ANGLE
            1500,                # CH7 NAV POSHOLD
            2000,                # CH8 MSP override
        ]
        self._publish_rc_override(channels)

    def _send_hover_command(self, reason: str):
        channels = [
            self.rc_mid,  # CH1 ROLL neutral
            self.rc_mid,  # CH2 PITCH neutral
            self.rc_mid,  # CH3 THROTTLE neutral
            self.rc_mid,  # CH4 YAW neutral
            2000,         # CH5 ARM
            1500,         # CH6 ANGLE
            1500,         # CH7 NAV POSHOLD
            2000,         # CH8 MSP override
        ]
        self._publish_rc_override(channels)
        self.get_logger().info(f'Hover command sent ({reason})', throttle_duration_sec=2.0)

    def _send_action_command(self):
        """
        Convert [dir_x, dir_y, dir_z, speed_fraction] into RC override.

        Steps:
          1) Clamp action to [-1, 1]
          2) L2-normalize direction
          3) Use speed = abs(speed_fraction)
          4) Build desired earth-frame velocity (cm/s)
          5) Convert horizontal velocity earth->body via current yaw
          6) Map body-forward/right and vertical requests to RC channels
          7) Apply yaw heading hold around north
        """
        ax, ay, az, speed_fraction = self.last_action

        # 1) Clamp all inputs
        ax = self._clamp_unit(ax)
        ay = self._clamp_unit(ay)
        az = self._clamp_unit(az)
        speed_fraction = self._clamp_unit(speed_fraction)

        # 2) L2-normalize direction; near-zero vector -> neutral direction
        dir_x, dir_y, dir_z = self._normalize_direction_l2(ax, ay, az)

        # 3) speed is magnitude only
        speed_abs = abs(speed_fraction)

        # 4) Desired earth-frame velocity (cm/s)
        target_speed_cms = self.speed_limit_cms * speed_abs
        v_east_cms = dir_x * target_speed_cms
        v_north_cms = dir_y * target_speed_cms
        v_up_cms = dir_z * target_speed_cms

        # 5) Convert earth horizontal velocity -> body forward/right using yaw
        v_forward_cms, v_right_cms = self._earth_to_body_horizontal(
            v_east_cms,
            v_north_cms,
            self.current_heading_deg,
        )

        # 6) Map desired velocities to RC stick deflections
        pitch_norm = self._safe_ratio(v_forward_cms, self.nav_manual_speed_cms)
        roll_norm = self._safe_ratio(v_right_cms, self.nav_manual_speed_cms)
        throttle_norm = self._safe_ratio(v_up_cms, self.nav_mc_manual_climb_rate_cms)

        roll_rc = self._map_norm_to_rc(roll_norm)
        pitch_rc = self._map_norm_to_rc(pitch_norm)
        throttle_rc = self._map_norm_to_rc(throttle_norm)

        heading_deg = self.get_current_heading_degrees()
        yaw_command, direction = self.yaw_controller.get_heading_hold_command(heading_deg)
        if yaw_command is None:
            yaw_rc = self.rc_mid
        else:
            yaw_rc = yaw_command
            now = time.time()
            if (now - self.last_yaw_hold_log_time) >= 0.5:
                self.get_logger().info(
                    f'🧭 Heading hold: heading={heading_deg:.1f}° → yaw {direction} ({yaw_rc})'
                )
                self.last_yaw_hold_log_time = now

        channels = [
            roll_rc,      # CH1 ROLL (body-right)
            pitch_rc,     # CH2 PITCH (body-forward)
            throttle_rc,  # CH3 THROTTLE (up)
            yaw_rc,       # CH4 YAW heading hold
            2000,         # CH5 ARM
            1500,         # CH6 ANGLE
            1500,         # CH7 NAV POSHOLD
            2000,         # CH8 MSP override
        ]
        self._publish_rc_override(channels)

        self.get_logger().info(
            f'Action=[{ax:+.2f},{ay:+.2f},{az:+.2f},s={speed_fraction:+.2f}] '
            f'Dir=[{dir_x:+.2f},{dir_y:+.2f},{dir_z:+.2f}] '
            f'Vel_earth=[{v_east_cms:+.1f},{v_north_cms:+.1f},{v_up_cms:+.1f}]cm/s '
            f'Vel_body=[fwd={v_forward_cms:+.1f},right={v_right_cms:+.1f}]cm/s '
            f'RC=[R={roll_rc},P={pitch_rc},T={throttle_rc},Y={yaw_rc}]',
            throttle_duration_sec=0.5,
        )

    # ------------ Math Helpers ------------
    def _normalize_direction_l2(self, x: float, y: float, z: float):
        norm = math.sqrt((x * x) + (y * y) + (z * z))
        if norm == 0.0:
            return 0.0, 0.0, 0.0
        inv_norm = 1.0 / norm
        return x * inv_norm, y * inv_norm, z * inv_norm

    def _earth_to_body_horizontal(self, v_east_cms: float, v_north_cms: float, yaw_deg: float):
        """
        Convert earth-frame horizontal velocity to body-frame forward/right.

        Heading convention expected here:
          yaw=0 deg means facing North, positive yaw turns clockwise toward East.
        """
        yaw_rad = math.radians(yaw_deg)
        sin_yaw = math.sin(yaw_rad)
        cos_yaw = math.cos(yaw_rad)

        v_forward_cms = (v_east_cms * sin_yaw) + (v_north_cms * cos_yaw)
        v_right_cms = (v_east_cms * cos_yaw) - (v_north_cms * sin_yaw)
        return v_forward_cms, v_right_cms

    def _safe_ratio(self, num: float, den: float):
        if abs(den) <= 1e-6:
            return 0.0
        return self._clamp_unit(num / den)

    def _map_norm_to_rc(self, value: float):
        value = self._clamp_unit(value)
        if value >= 0.0:
            span = self.rc_max - self.rc_mid
        else:
            span = self.rc_mid - self.rc_min
        rc = self.rc_mid + int(round(value * span))
        return max(self.rc_min, min(self.rc_max, rc))

    @staticmethod
    def _safe_float(value):
        try:
            out = float(value)
        except Exception:
            return 0.0
        if math.isnan(out) or math.isinf(out):
            return 0.0
        return out

    @staticmethod
    def _clamp_unit(value: float):
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value

    # ------------ RC Publishing ------------
    def _publish_rc_override(self, channels: list):
        """Publish RC override values, throttled to configured RC publish rate."""
        now = time.time()
        if (now - self.last_rc_publish_time) < self.rc_publish_interval:
            return
        self.last_rc_publish_time = now

        rc_msg = Float32MultiArray()
        rc_msg.data = [float(ch) for ch in channels]
        self.rc_override_pub.publish(rc_msg)
        self.get_logger().info(
            f'📤 Publishing RC override: R={channels[0]}, P={channels[1]}, T={channels[2]}, '
            f'Y={channels[3]}, ARM={channels[4]}, ANG={channels[5]}, NAV={channels[6]}, MSP={channels[7]}',
            throttle_duration_sec=1.0,
        )

    # ------------ Status Publishing ------------
    def publish_status(self):
        """Publish adapter status."""
        flags = []
        now = time.time()

        if not self.arming_complete:
            elapsed = now - self.arming_start_time
            remaining = max(0.0, self.arming_duration - elapsed)
            flags.append(f'ARMING({remaining:.1f}s)')
        elif not self.rise_complete:
            elapsed = now - (self.rise_start_time or now)
            remaining = max(0.0, self.rise_max_duration - elapsed)
            alt_str = f'{self.lidar_altitude:.2f}m' if self.lidar_altitude is not None else 'no_lidar'
            flags.append(
                f'RISING(alt={alt_str},target={self.rise_target_altitude:.2f}m,timeout={remaining:.1f}s)'
            )
        elif self.post_rise_hover_until is not None and now < self.post_rise_hover_until:
            remaining = self.post_rise_hover_until - now
            flags.append(f'STABILIZING({remaining:.1f}s)')
        elif not self.yaw_alignment_complete:
            heading = self.get_current_heading_degrees()
            flags.append(f'YAW_ALIGN({heading:.1f}°)')
        else:
            flags.append('READY')

        if self.safety_override:
            flags.append('SAFETY_OVERRIDE')

        if (now - self.last_cmd_time) > self.cmd_timeout:
            flags.append('CMD_TIMEOUT')

        flags.append('CRUISE_DIRECTION_MODE')

        msg = String()
        msg.data = ' | '.join(flags)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FCAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down FC Adapter Node (CRUISE)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
