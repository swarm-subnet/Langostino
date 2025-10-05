#!/usr/bin/env python3
"""
FC Adapter Velocity Node (Closed-Loop, DIRECT MSP with pre-arm)
- Arms the FC for N seconds by streaming an ARM frame over MSP (like arm_only_simple.py).
- Then subscribes to /ai/action (Float32MultiArray: [vx, vy, vz, speed]) and flies closed-loop.
- Uses GPS speed/course + yaw to estimate actual velocity in body frame.
- PID (vx, vy, vz) -> RC deviations -> MSP_SET_RAW_RC sent over MSP serial (no fc_comms_node).
- Logs every action it sends to the FC.

Safety:
- TEST WITH PROPS OFF FIRST.
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

import serial

from swarm_ai_integration.pid_controller import VelocityPIDController
from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)


class FCAdapterVelocityNode(Node):
    """
    Subscriptions:
      /ai/action                 (std_msgs/Float32MultiArray) : [vx, vy, vz, speed]  (speed unused)
      /fc/gps_speed_course       (std_msgs/Float32MultiArray) : [speed_mps, course_deg]
      /fc/attitude_euler         (geometry_msgs/Vector3Stamped): roll,pitch,yaw (rad)
      /fc/altitude               (std_msgs/Float32MultiArray) : [baro_alt_m, vz_mps]
      /safety/override           (std_msgs/Bool)

    Publications:
      /fc_adapter/status         (std_msgs/String)
      /fc_adapter/velocity_error (geometry_msgs/Vector3Stamped)
    """

    def __init__(self):
        super().__init__('fc_adapter_velocity_node')

        # ------------ Parameters ------------
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('command_timeout_sec', 1.0)
        self.declare_parameter('warmup_frames', 40)
        self.declare_parameter('rc_min', 1300)
        self.declare_parameter('rc_max', 1700)

        self.declare_parameter('arm_aux_high', True)
        self.declare_parameter('enable_angle_mode', True)
        self.declare_parameter('enable_althold_mode', True)
        self.declare_parameter('enable_nav_rth', False)  # NAV RTH on CH9

        self.declare_parameter('kp_xy', 150.0)
        self.declare_parameter('ki_xy', 10.0)
        self.declare_parameter('kd_xy', 20.0)
        self.declare_parameter('kp_z', 100.0)
        self.declare_parameter('ki_z', 5.0)
        self.declare_parameter('kd_z', 15.0)

        # MSP serial (DIRECT) like arm_only_simple.py
        self.declare_parameter('msp_port', '/dev/ttyAMA0')
        self.declare_parameter('msp_baudrate', 115200)
        self.declare_parameter('msp_write_timeout_sec', 0.05)

        # Pre-arm streaming (like your working script)
        self.declare_parameter('prearm_enabled', True)
        self.declare_parameter('prearm_seconds', 30.0)   # stream ARM frame for 5 seconds
        self.declare_parameter('prearm_hz', 40)         # at 40 Hz
        self.declare_parameter('startup_delay_sec', 20.0)

        # ------------ Param values ------------
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.warmup_frames_total = int(self.get_parameter('warmup_frames').value)
        self.rc_min = int(self.get_parameter('rc_min').value)
        self.rc_max = int(self.get_parameter('rc_max').value)

        self.arm_aux_high = bool(self.get_parameter('arm_aux_high').value)
        self.angle_mode_enabled = bool(self.get_parameter('enable_angle_mode').value)
        self.althold_enabled = bool(self.get_parameter('enable_althold_mode').value)
        self.nav_rth_enabled = bool(self.get_parameter('enable_nav_rth').value)

        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.ki_xy = float(self.get_parameter('ki_xy').value)
        self.kd_xy = float(self.get_parameter('kd_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)

        self.msp_port = str(self.get_parameter('msp_port').value)
        self.msp_baudrate = int(self.get_parameter('msp_baudrate').value)
        self.msp_write_timeout = float(self.get_parameter('msp_write_timeout_sec').value)

        self.prearm_enabled = bool(self.get_parameter('prearm_enabled').value)
        self.prearm_seconds = float(self.get_parameter('prearm_seconds').value)
        self.prearm_hz = int(self.get_parameter('prearm_hz').value)
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)

        # ------------ State ------------
        self.velocity_cmd_body = np.zeros(3, dtype=float)      # [vx, vy, vz]
        self.velocity_actual_earth = np.zeros(3, dtype=float)  # [east, north, up]
        self.velocity_actual_body = np.zeros(3, dtype=float)   # [vx, vy, vz]
        self.attitude_yaw = 0.0
        self.last_ai_speed_scalar: Optional[float] = None

        self.last_cmd_time = time.time()
        self.last_control_time = time.time()
        self.safety_override = False
        self.ai_enabled = False

        # Warm-up state
        self.warmup_complete = False
        self.warmup_frames_sent = 0

        # Safety RTH override
        self.safety_rth_requested = False

        # Safety RTH override
        self.safety_rth_requested = False

        # PID
        rc_dev_limit = 400.0
        self.velocity_controller = VelocityPIDController(
            kp_xy=self.kp_xy, ki_xy=self.ki_xy, kd_xy=self.kd_xy,
            kp_z=self.kp_z,  ki_z=self.ki_z,  kd_z=self.kd_z,
            output_min=-rc_dev_limit, output_max=rc_dev_limit
        )

        self.stats = {'loops': 0, 'msp_cmds': 0, 'failsafes': 0}

        # ------------ MSP Serial (direct) ------------
        self.ser: Optional[serial.Serial] = None
        self._open_serial()
        

        # ----------- OPTIONAL STARTUP DELAY (operator setup window) -----------
        if self.startup_delay_sec > 0:
            self.get_logger().warn(f'Waiting {self.startup_delay_sec:.1f}s before arming (operator setup).')
            time.sleep(self.startup_delay_sec)

        # ----------- PRE-ARM: stream ARM frame like arm_only_simple.py -----------
        if self.prearm_enabled and (self.ser and self.ser.is_open):
            self._prearm_stream(seconds=self.prearm_seconds, hz=self.prearm_hz)
        else:
            if not self.prearm_enabled:
                self.get_logger().info('Pre-arm disabled by parameter.')
            else:
                self.get_logger().error('Pre-arm requested but MSP serial is not open.')

        # ------------ QoS & ROS I/O ------------
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
        self.create_subscription(Bool, '/safety/override', self.cb_safety, control_qos)
        self.create_subscription(Bool, '/safety/rth_command', self.cb_rth_command, control_qos)

        # Pubs
        self.status_pub = self.create_publisher(String, '/fc_adapter/status', control_qos)
        self.vel_error_pub = self.create_publisher(Vector3Stamped, '/fc_adapter/velocity_error', control_qos)

        # Timers (start AFTER pre-arm so nothing else transmits during arming)
        self.control_period = 1.0 / max(1.0, self.control_rate)
        self.create_timer(self.control_period, self.control_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'FC Adapter Velocity Node (DIRECT MSP + pre-arm) ready @ {self.control_rate}Hz; MSP on {self.msp_port}@{self.msp_baudrate}')

    # ------------ Serial ------------
    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.msp_port, self.msp_baudrate, timeout=0.02, write_timeout=self.msp_write_timeout)
            time.sleep(2.0)  # let FC UART settle
            self.get_logger().info('MSP serial opened.')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'MSP serial open failed: {e}')

    def _close_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info('MSP serial closed.')
        except Exception:
            pass

    # ------------ Pre-ARM like arm_only_simple.py ------------
    def _prearm_stream(self, seconds: float, hz: int):
        """
        Stream the ARM frame for N seconds at hz:
          AETR + AUX = [ROLL, PITCH, THROTTLE, YAW, AUX1(ARM), AUX2, AUX3, AUX4(OVERRIDE)]
          -> [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]
        """
        self.get_logger().warn(f'=== PRE-ARM: streaming ARM for {seconds:.1f}s at {hz}Hz ===')
        channels = [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]
        period = 1.0 / float(max(1, hz))
        t_end = time.time() + max(0.0, seconds)
        while time.time() < t_end:
            t0 = time.time()
            self._send_msp_set_raw_rc(channels)
            dt = time.time() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)
        self.get_logger().info('✅ PRE-ARM finished; proceeding to closed-loop.')

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

    def cb_safety(self, msg: Bool):
        self.safety_override = bool(msg.data)
        if self.safety_override:
            self.get_logger().warn('Safety override ON → hover')
            self.velocity_controller.reset()

    def cb_rth_command(self, msg: Bool):
        """Handle RTH (Return to Home) command from safety monitor"""
        self.safety_rth_requested = bool(msg.data)
        if self.safety_rth_requested:
            self.get_logger().error('🚨 RTH ACTIVATED - Return to Home mode engaged')
        else:
            self.get_logger().info('✓ RTH deactivated')

    # -------------------- Control Loop --------------------
    def control_loop(self):
        now = time.time()
        dt = max(1e-3, now - self.last_control_time)
        self.last_control_time = now

        # Ensure serial is open; attempt reopen once if lost
        if not (self.ser and self.ser.is_open):
            self._open_serial()

        # Warm-up sequence: send neutral frames before accepting AI commands
        if not self.warmup_complete:
            if self.warmup_frames_sent < self.warmup_frames_total:
                self.send_hover_command(reason='warmup')
                self.warmup_frames_sent += 1
                return
            else:
                self.warmup_complete = True
                self.get_logger().info(f'✓ Warm-up complete ({self.warmup_frames_sent} frames @ {self.control_rate}Hz)')

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

        # Clamp (gentle authority)
        roll_rc = max(self.rc_min, min(self.rc_max, roll_rc))
        pitch_rc = max(self.rc_min, min(self.rc_max, pitch_rc))
        throttle_rc = max(self.rc_min, min(self.rc_max, throttle_rc))

        # Compose 16 channels (INAV supports up to 18, we use 9 for RTH)
        channels = [1500] * 16
        channels[0] = roll_rc
        channels[1] = pitch_rc
        channels[2] = throttle_rc
        channels[3] = yaw_rc
        channels[4] = 1800 if self.arm_aux_high else 1000       # CH5: ARM (AUX1)
        channels[5] = 1800 if self.angle_mode_enabled else 1000  # CH6: ANGLE mode (AUX2)
        channels[6] = 1800 if self.althold_enabled else 1000     # CH7: ALT HOLD (AUX3)
        channels[7] = 1800                                       # CH8: MSP RC OVERRIDE (AUX4) - MUST be >1700
        channels[8] = 1800 if (self.nav_rth_enabled or self.safety_rth_requested) else 1000  # CH9: NAV RTH (AUX5)

        # Direct MSP send
        self._send_msp_set_raw_rc(channels)

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
    def _earth_to_body_velocity(self):
        vx_e, vy_n, vz_u = self.velocity_actual_earth
        cy = math.cos(self.attitude_yaw)
        sy = math.sin(self.attitude_yaw)
        self.velocity_actual_body[0] = vx_e * cy + vy_n * sy     # forward
        self.velocity_actual_body[1] = -vx_e * sy + vy_n * cy    # right
        self.velocity_actual_body[2] = vz_u                      # up

    def _publish_msp_set_raw_rc(self, channels: list):
        # Clamp and pad to 16 channels (INAV supports up to 18)
        ch = [int(max(1000, min(2000, c))) for c in (channels + [1500]*16)[:16]]
        msg = Float32MultiArray()
        msg.data = [MSP_SET_RAW_RC_CODE] + [float(v) for v in ch]
        self.msp_cmd_pub.publish(msg)
        self.stats['msp_cmds'] += 1

    def send_hover_command(self, reason: str):
        channels = [1500] * 16
        channels[0] = 1500  # Roll (neutral)
        channels[1] = 1500  # Pitch (neutral)
        channels[2] = 1500  # Throttle (hold altitude in AltHold)
        channels[3] = 1500  # Yaw (neutral)
        channels[4] = 1800 if self.arm_aux_high else 1000        # CH5: ARM (AUX1)
        channels[5] = 1800 if self.angle_mode_enabled else 1000  # CH6: ANGLE mode (AUX2)
        channels[6] = 1800 if self.althold_enabled else 1000     # CH7: ALT HOLD (AUX3)
        channels[7] = 1800                                       # CH8: MSP RC OVERRIDE (AUX4)
        channels[8] = 1800 if (self.nav_rth_enabled or self.safety_rth_requested) else 1000  # CH9: NAV RTH (AUX5)
        self._publish_msp_set_raw_rc(channels)
        self.get_logger().warn(f"Hover command sent ({reason}); RC[R,P,T,Y]=[{channels[0]},{channels[1]},{channels[2]},{channels[3]}]")
        self.stats['failsafes'] += 1

    # -------------------- Status --------------------
    def publish_status(self):
        flags = []
        now = time.time()

        # Serial status
        if self.ser and self.ser.is_open:
            flags.append('MSP_SERIAL_OK')
        else:
            flags.append('MSP_SERIAL_FAIL')

        # Operational status
        if not self.warmup_complete:
            flags.append('WARMUP')
        if self.ai_enabled:
            flags.append('AI_ENABLED')
        if self.safety_override:
            flags.append('SAFETY_OVERRIDE')

        flags.append('CLOSED_LOOP_DIRECT_MSP')

        if now - self.last_cmd_time > self.cmd_timeout:
            flags.append('CMD_TIMEOUT')

        s = String()
        s.data = ' | '.join(flags) if flags else 'STANDBY'
        self.status_pub.publish(s)

    # -------------------- Shutdown --------------------
    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


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
