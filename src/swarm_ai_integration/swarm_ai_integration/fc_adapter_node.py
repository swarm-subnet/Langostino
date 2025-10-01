#!/usr/bin/env python3
"""
FC Adapter Velocity Node (Closed-Loop)
- Subscribes to /ai/action (Float32MultiArray: [vx, vy, vz, speed])
- Uses GPS speed/course + yaw to estimate actual velocity in body frame
- PID (vx, vy, vz) -> RC deviations -> MSP_SET_RAW_RC (code 200) on /fc/msp_command
- Logs every action it sends to the FC
"""

import math
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Vector3Stamped

from pid_controller import VelocityPIDController

# MSP command code (fallback to literal 200 if enum isn't available)
try:
    from .msp_protocol import MSPCommand
    MSP_SET_RAW_RC_CODE = float(MSPCommand.MSP_SET_RAW_RC)
except Exception:
    MSP_SET_RAW_RC_CODE = 200.0


class FCAdapterVelocityNode(Node):
    """
    Subscriptions:
      /ai/action                 (std_msgs/Float32MultiArray) : [vx, vy, vz, speed]  (speed unused)
      /fc/gps_speed_course       (std_msgs/Float32MultiArray) : [speed_mps, course_deg]
      /fc/attitude_euler         (geometry_msgs/Vector3Stamped): roll,pitch,yaw (rad)
      /fc/altitude               (std_msgs/Float32MultiArray) : [baro_alt_m, vz_mps]
      /fc/msp_status             (std_msgs/Float32MultiArray) : [cycle_time, i2c_errors, sensor_mask, box_flags, current_setting]
      /safety/override           (std_msgs/Bool)

    Publications:
      /fc/msp_command            (std_msgs/Float32MultiArray) : [200, ch1..ch8] (1000..2000)
      /fc_adapter/status         (std_msgs/String)
      /fc_adapter/velocity_error (geometry_msgs/Vector3Stamped)
    """

    def __init__(self):
        super().__init__('fc_adapter_velocity_node')

        # Parameters
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('command_timeout_sec', 1.0)
        self.declare_parameter('fc_status_timeout_sec', 2.0)
        self.declare_parameter('warmup_frames', 40)
        self.declare_parameter('rc_min', 1300)
        self.declare_parameter('rc_max', 1700)

        self.declare_parameter('arm_aux_high', True)
        self.declare_parameter('enable_angle_mode', True)
        self.declare_parameter('enable_althold_mode', True)

        self.declare_parameter('kp_xy', 150.0)
        self.declare_parameter('ki_xy', 10.0)
        self.declare_parameter('kd_xy', 20.0)
        self.declare_parameter('kp_z', 100.0)
        self.declare_parameter('ki_z', 5.0)
        self.declare_parameter('kd_z', 15.0)

        # Param values
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.fc_status_timeout = float(self.get_parameter('fc_status_timeout_sec').value)
        self.warmup_frames_total = int(self.get_parameter('warmup_frames').value)
        self.rc_min = int(self.get_parameter('rc_min').value)
        self.rc_max = int(self.get_parameter('rc_max').value)

        self.arm_aux_high = bool(self.get_parameter('arm_aux_high').value)
        self.angle_mode_enabled = bool(self.get_parameter('enable_angle_mode').value)
        self.althold_enabled = bool(self.get_parameter('enable_althold_mode').value)

        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.ki_xy = float(self.get_parameter('ki_xy').value)
        self.kd_xy = float(self.get_parameter('kd_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)

        # State
        self.velocity_cmd_body = np.zeros(3, dtype=float)      # [vx, vy, vz]
        self.velocity_actual_earth = np.zeros(3, dtype=float)  # [east, north, up]
        self.velocity_actual_body = np.zeros(3, dtype=float)   # [vx, vy, vz]
        self.attitude_yaw = 0.0
        self.last_ai_speed_scalar: Optional[float] = None

        self.last_cmd_time = time.time()
        self.last_control_time = time.time()
        self.safety_override = False
        self.ai_enabled = False

        # FC health monitoring
        self.fc_connected = False
        self.fc_armed = False
        self.fc_box_flags = 0
        self.fc_sensor_mask = 0
        self.last_fc_status_time = 0.0

        # Warm-up state
        self.warmup_complete = False
        self.warmup_frames_sent = 0

        rc_dev_limit = 400.0
        self.velocity_controller = VelocityPIDController(
            kp_xy=self.kp_xy, ki_xy=self.ki_xy, kd_xy=self.kd_xy,
            kp_z=self.kp_z,  ki_z=self.ki_z,  kd_z=self.kd_z,
            output_min=-rc_dev_limit, output_max=rc_dev_limit
        )

        self.stats = {'loops': 0, 'msp_cmds': 0, 'failsafes': 0}

        # QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subs
        self.create_subscription(Float32MultiArray, '/ai/action', self.cb_ai_action_array, control_qos)
        self.create_subscription(Float32MultiArray, '/fc/gps_speed_course', self.cb_gps_speed_course, sensor_qos)
        self.create_subscription(Vector3Stamped, '/fc/attitude_euler', self.cb_attitude, sensor_qos)
        self.create_subscription(Float32MultiArray, '/fc/altitude', self.cb_altitude, sensor_qos)
        self.create_subscription(Float32MultiArray, '/fc/msp_status', self.cb_fc_status, sensor_qos)
        self.create_subscription(Bool, '/safety/override', self.cb_safety, control_qos)

        # Pubs
        self.msp_cmd_pub = self.create_publisher(Float32MultiArray, '/fc/msp_command', control_qos)
        self.status_pub = self.create_publisher(String, '/fc_adapter/status', control_qos)
        self.vel_error_pub = self.create_publisher(Vector3Stamped, '/fc_adapter/velocity_error', control_qos)

        # Timers
        self.control_period = 1.0 / max(1.0, self.control_rate)
        self.create_timer(self.control_period, self.control_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'FC Adapter Velocity Node ready @ {self.control_rate}Hz (listening to /ai/action Float32MultiArray)')

    # -------------------- Callbacks --------------------

    def cb_ai_action_array(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            return
        vx, vy, vz = float(data[0]), float(data[1]), float(data[2])

        # clamp for safety
        self.velocity_cmd_body[0] = float(np.clip(vx, -self.max_velocity, self.max_velocity))
        self.velocity_cmd_body[1] = float(np.clip(vy, -self.max_velocity, self.max_velocity))
        self.velocity_cmd_body[2] = float(np.clip(vz, -self.max_velocity, self.max_velocity))

        self.last_ai_speed_scalar = float(data[3]) if len(data) > 3 else None
        self.last_cmd_time = time.time()
        self.ai_enabled = True

    def cb_gps_speed_course(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        speed_mps = float(msg.data[0])
        course_deg = float(msg.data[1])
        cr = math.radians(course_deg)
        # ENU: east = +sin(course), north = +cos(course)
        self.velocity_actual_earth[0] = speed_mps * math.sin(cr)
        self.velocity_actual_earth[1] = speed_mps * math.cos(cr)
        # up set in altitude callback

    def cb_attitude(self, msg: Vector3Stamped):
        self.attitude_yaw = float(msg.vector.z)

    def cb_altitude(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.velocity_actual_earth[2] = float(msg.data[1])  # vertical velocity (up)

    def cb_fc_status(self, msg: Float32MultiArray):
        """
        MSP_STATUS callback: [cycle_time, i2c_errors, sensor_mask, box_flags, current_setting]
        box_flags contains mode bits (exact mapping depends on INAV configuration)
        """
        if len(msg.data) >= 4:
            self.fc_box_flags = int(msg.data[3])
            self.fc_sensor_mask = int(msg.data[2]) if len(msg.data) >= 3 else 0
            self.fc_connected = True
            self.last_fc_status_time = time.time()

            # Note: INAV doesn't have a direct "armed" bit in MSP_STATUS box_flags.
            # The ARM state is typically inferred from mode flags or separate status.
            # For now, we assume FC is "armed" if we're receiving status.
            # You may need to adjust this based on your specific INAV box mode configuration.
            self.fc_armed = True  # Conservative: assume armed if connected

    def cb_safety(self, msg: Bool):
        self.safety_override = bool(msg.data)
        if self.safety_override:
            self.get_logger().warn('Safety override ON → hover')
            self.velocity_controller.reset()

    # -------------------- Control Loop --------------------

    def control_loop(self):
        now = time.time()
        dt = max(1e-3, now - self.last_control_time)
        self.last_control_time = now

        # Warm-up sequence: send neutral frames before accepting AI commands
        if not self.warmup_complete:
            if self.warmup_frames_sent < self.warmup_frames_total:
                self.send_hover_command(reason='warmup')
                self.warmup_frames_sent += 1
                return
            else:
                self.warmup_complete = True
                self.get_logger().info(f'✓ Warm-up complete ({self.warmup_frames_sent} frames @ {self.control_rate}Hz)')

        # FC health check: verify connection
        if now - self.last_fc_status_time > self.fc_status_timeout:
            if self.fc_connected:
                self.get_logger().error('FC STATUS TIMEOUT - no MSP_STATUS received')
                self.fc_connected = False
                self.fc_armed = False
            self.send_hover_command(reason='fc_disconnected')
            return

        # Pre-flight validation: ensure FC is ready
        if not self.validate_fc_ready():
            self.send_hover_command(reason='fc_not_ready')
            return

        # Command timeout or safety → hover
        if (now - self.last_cmd_time > self.cmd_timeout) or self.safety_override or (not self.ai_enabled):
            reason = 'timeout' if (now - self.last_cmd_time > self.cmd_timeout) else ('safety/disabled' if self.safety_override else 'no_ai')
            self.send_hover_command(reason=reason)
            return

        # Earth (ENU) -> Body (Fwd/Right/Up) using yaw
        self._earth_to_body_velocity()

        # PID: (vx,vy,vz) error -> RC deviations (pitch, roll, throttle)
        pitch_dev, roll_dev, throttle_dev = self.velocity_controller.compute(
            vel_cmd_x=self.velocity_cmd_body[0],
            vel_cmd_y=self.velocity_cmd_body[1],
            vel_cmd_z=self.velocity_cmd_body[2],
            vel_actual_x=self.velocity_actual_body[0],
            vel_actual_y=self.velocity_actual_body[1],
            vel_actual_z=self.velocity_actual_body[2],
            dt=dt
        )

        # Deviations around 1500
        roll_rc = int(1500 + roll_dev)         # ch1
        pitch_rc = int(1500 + pitch_dev)       # ch2
        throttle_rc = int(1500 + throttle_dev) # ch3
        yaw_rc = 1500                          # ch4 (hold)

        # Clamp
        roll_rc = max(self.rc_min, min(self.rc_max, roll_rc))
        pitch_rc = max(self.rc_min, min(self.rc_max, pitch_rc))
        throttle_rc = max(self.rc_min, min(self.rc_max, throttle_rc))

        # Compose 8 channels
        channels = [1500] * 8
        channels[0] = roll_rc
        channels[1] = pitch_rc
        channels[2] = throttle_rc
        channels[3] = yaw_rc
        channels[4] = 1800 if self.arm_aux_high else 1000       # CH5: ARM (AUX1)
        channels[5] = 1800 if self.angle_mode_enabled else 1000  # CH6: ANGLE mode (AUX2)
        channels[6] = 1800 if self.althold_enabled else 1000     # CH7: ALT HOLD (AUX3)
        channels[7] = 1800                                       # CH8: MSP RC OVERRIDE (AUX4) - MUST be >1700

        # Publish MSP_SET_RAW_RC
        self._publish_msp_set_raw_rc(channels)

        # Log every action
        vx, vy, vz = self.velocity_cmd_body
        ex = vx - self.velocity_actual_body[0]
        ey = vy - self.velocity_actual_body[1]
        ez = vz - self.velocity_actual_body[2]
        spd = self.last_ai_speed_scalar if self.last_ai_speed_scalar is not None else 0.0

        self.get_logger().info(
            f"[loop {self.stats['loops']+1:06d}] AI(vx,vy,vz,speed)=({vx:+.3f},{vy:+.3f},{vz:+.3f},{spd:+.3f})  "
            f"BodyVel=({self.velocity_actual_body[0]:+.3f},{self.velocity_actual_body[1]:+.3f},{self.velocity_actual_body[2]:+.3f})  "
            f"Err=({ex:+.3f},{ey:+.3f},{ez:+.3f})  "
            f"RC[R,P,T,Y]=[{channels[0]},{channels[1]},{channels[2]},{channels[3]}]  "
            f"AUX[5..7]=[{channels[4]},{channels[5]},{channels[6]}]"
        )

        self.stats['loops'] += 1

        # Publish error vector for plotting
        err_msg = Vector3Stamped()
        err_msg.header.stamp = self.get_clock().now().to_msg()
        err_msg.header.frame_id = 'body'
        err_msg.vector.x = ex
        err_msg.vector.y = ey
        err_msg.vector.z = ez
        self.vel_error_pub.publish(err_msg)

    # -------------------- Helpers --------------------

    def validate_fc_ready(self) -> bool:
        """Verify FC is connected and ready for velocity commands"""
        if not self.fc_connected:
            # Only log once when state changes
            if not hasattr(self, '_last_fc_ready_warn') or self._last_fc_ready_warn != 'disconnected':
                self.get_logger().warn('FC not connected - waiting for MSP_STATUS')
                self._last_fc_ready_warn = 'disconnected'
            return False

        # Note: fc_armed check is conservative. Adjust based on your operational needs.
        # Some users may want to send commands even before arming for testing.
        # if not self.fc_armed:
        #     if not hasattr(self, '_last_fc_ready_warn') or self._last_fc_ready_warn != 'unarmed':
        #         self.get_logger().warn('FC not armed - refusing velocity control')
        #         self._last_fc_ready_warn = 'unarmed'
        #     return False

        self._last_fc_ready_warn = None  # Clear warning state
        return True

    def _earth_to_body_velocity(self):
        vx_e, vy_n, vz_u = self.velocity_actual_earth
        cy = math.cos(self.attitude_yaw)
        sy = math.sin(self.attitude_yaw)
        self.velocity_actual_body[0] = vx_e * cy + vy_n * sy     # forward
        self.velocity_actual_body[1] = -vx_e * sy + vy_n * cy    # right
        self.velocity_actual_body[2] = vz_u                      # up

    def _publish_msp_set_raw_rc(self, channels: list):
        ch = [int(max(1000, min(2000, c))) for c in (channels + [1500]*8)[:8]]
        msg = Float32MultiArray()
        msg.data = [MSP_SET_RAW_RC_CODE] + [float(v) for v in ch]
        self.msp_cmd_pub.publish(msg)
        self.stats['msp_cmds'] += 1

    def send_hover_command(self, reason: str):
        channels = [
            1500, 1500, 1500, 1500,                              # Roll, Pitch, Throttle, Yaw (neutral)
            1800 if self.arm_aux_high else 1000,                 # CH5: ARM (AUX1)
            1800 if self.angle_mode_enabled else 1000,           # CH6: ANGLE mode (AUX2)
            1800 if self.althold_enabled else 1000,              # CH7: ALT HOLD (AUX3)
            1800                                                 # CH8: MSP RC OVERRIDE (AUX4)
        ]
        self._publish_msp_set_raw_rc(channels)
        self.get_logger().warn(f"Hover command sent ({reason}); RC[R,P,T,Y]=[{channels[0]},{channels[1]},{channels[2]},{channels[3]}]")
        self.stats['failsafes'] += 1

    # -------------------- Status --------------------

    def publish_status(self):
        flags = []
        now = time.time()

        # FC health status
        if self.fc_connected:
            flags.append('FC_CONNECTED')
        else:
            flags.append('FC_DISCONNECTED')

        # Operational status
        if not self.warmup_complete:
            flags.append('WARMUP')
        if self.ai_enabled:
            flags.append('AI_ENABLED')
        if self.safety_override:
            flags.append('SAFETY_OVERRIDE')

        flags.append('CLOSED_LOOP')

        if now - self.last_cmd_time > self.cmd_timeout:
            flags.append('CMD_TIMEOUT')
        if now - self.last_fc_status_time > self.fc_status_timeout:
            flags.append('FC_STATUS_TIMEOUT')

        s = String()
        s.data = ' | '.join(flags) if flags else 'STANDBY'
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = FCAdapterVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down FC Adapter Velocity Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
