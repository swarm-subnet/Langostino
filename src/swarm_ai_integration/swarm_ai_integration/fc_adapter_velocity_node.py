#!/usr/bin/env python3
"""
FC Adapter Node with Closed-Loop Velocity Control

This node implements proper velocity control for the AI model:
1. Receives body-frame velocity commands from ai_flight_node (/ai/action as Twist)
2. Reads actual velocity from FC via MSP (GPS + attitude)
3. Uses PID control to convert velocity error → RC commands
4. Sends MSP_SET_RAW_RC commands to iNav

Key Features:
- Closed-loop velocity tracking with PID
- Body-frame to earth-frame velocity transformation
- iNav ANGLE + ALTHOLD mode for stability
- Conservative safety limits
- Command watchdog and failsafe
"""

import numpy as np
import math
import time
from typing import Optional, Tuple
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import NavSatFix

from .pid_controller import VelocityPIDController
from .msp_protocol import MSPMessage, MSPCommand, MSPDataTypes


class FCAdapterVelocityNode(Node):
    """
    Flight Controller Adapter with Closed-Loop Velocity Control.

    Subscribers:
        /ai/action (geometry_msgs/Twist): Body-frame velocity commands from AI
        /fc/gps_fix (sensor_msgs/NavSatFix): GPS position
        /fc/gps_speed_course (std_msgs/Float32MultiArray): [speed_mps, course_deg]
        /fc/attitude_euler (geometry_msgs/Vector3Stamped): [roll, pitch, yaw] in radians
        /fc/altitude (std_msgs/Float32MultiArray): [baro_alt, vario/vz]
        /safety/override (std_msgs/Bool): Safety override

    Publishers:
        /fc/msp_command (std_msgs/Float32MultiArray): MSP commands
        /fc_adapter/status (std_msgs/String): Status
        /fc_adapter/velocity_error (geometry_msgs/Vector3Stamped): Velocity tracking error
    """

    def __init__(self):
        super().__init__('fc_adapter_velocity_node')

        # Declare parameters
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('max_velocity', 3.0)  # Must match AI training!
        self.declare_parameter('command_timeout_sec', 1.0)

        # PID gains (start conservative, tune in flight)
        self.declare_parameter('kp_xy', 150.0)
        self.declare_parameter('ki_xy', 10.0)
        self.declare_parameter('kd_xy', 20.0)
        self.declare_parameter('kp_z', 100.0)
        self.declare_parameter('ki_z', 5.0)
        self.declare_parameter('kd_z', 15.0)

        # RC limits
        self.declare_parameter('rc_min', 1300)
        self.declare_parameter('rc_max', 1700)

        # Safety
        self.declare_parameter('enable_closed_loop', True)
        self.declare_parameter('enable_angle_mode', True)
        self.declare_parameter('enable_althold_mode', True)

        # Get parameters
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.cmd_timeout = self.get_parameter('command_timeout_sec').value

        self.kp_xy = self.get_parameter('kp_xy').value
        self.ki_xy = self.get_parameter('ki_xy').value
        self.kd_xy = self.get_parameter('kd_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value

        self.rc_min = int(self.get_parameter('rc_min').value)
        self.rc_max = int(self.get_parameter('rc_max').value)

        self.closed_loop_enabled = self.get_parameter('enable_closed_loop').value
        self.angle_mode_enabled = self.get_parameter('enable_angle_mode').value
        self.althold_enabled = self.get_parameter('enable_althold_mode').value

        # Initialize PID controller
        rc_deviation_limit = 400.0  # Max deviation from 1500
        self.velocity_controller = VelocityPIDController(
            kp_xy=self.kp_xy, ki_xy=self.ki_xy, kd_xy=self.kd_xy,
            kp_z=self.kp_z, ki_z=self.ki_z, kd_z=self.kd_z,
            output_min=-rc_deviation_limit,
            output_max=rc_deviation_limit
        )

        # State variables
        self.velocity_cmd_body = np.zeros(3)  # [vx, vy, vz] commanded (body frame)
        self.velocity_actual_body = np.zeros(3)  # [vx, vy, vz] actual (body frame)
        self.velocity_actual_earth = np.zeros(3)  # [vx_east, vy_north, vz_up]

        self.attitude_euler = np.zeros(3)  # [roll, pitch, yaw] in radians
        self.gps_speed_mps = 0.0
        self.gps_course_deg = 0.0
        self.altitude_vz = 0.0  # Vertical velocity from barometer

        self.last_cmd_time = time.time()
        self.safety_override = False
        self.ai_enabled = False

        # Statistics
        self.stats = {
            'commands_sent': 0,
            'control_loops': 0,
            'failsafe_activations': 0,
        }

        # QoS profiles
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

        # Subscribers
        self.vel_cmd_sub = self.create_subscription(
            Twist, '/ai/action', self.velocity_cmd_callback, control_qos)

        self.gps_speed_sub = self.create_subscription(
            Float32MultiArray, '/fc/gps_speed_course', self.gps_speed_callback, sensor_qos)

        self.attitude_sub = self.create_subscription(
            Vector3Stamped, '/fc/attitude_euler', self.attitude_callback, sensor_qos)

        self.altitude_sub = self.create_subscription(
            Float32MultiArray, '/fc/altitude', self.altitude_callback, sensor_qos)

        self.safety_sub = self.create_subscription(
            Bool, '/safety/override', self.safety_callback, control_qos)

        # Publishers
        self.msp_cmd_pub = self.create_publisher(
            Float32MultiArray, '/fc/msp_command', control_qos)

        self.status_pub = self.create_publisher(
            String, '/fc_adapter/status', control_qos)

        self.vel_error_pub = self.create_publisher(
            Vector3Stamped, '/fc_adapter/velocity_error', control_qos)

        # Control loop timer
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('=' * 80)
        self.get_logger().info('FC Adapter Velocity Node Initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'PID gains XY: Kp={self.kp_xy}, Ki={self.ki_xy}, Kd={self.kd_xy}')
        self.get_logger().info(f'PID gains Z:  Kp={self.kp_z}, Ki={self.ki_z}, Kd={self.kd_z}')
        self.get_logger().info(f'RC limits: [{self.rc_min}, {self.rc_max}]')
        self.get_logger().info(f'Closed-loop: {self.closed_loop_enabled}')
        self.get_logger().info(f'Flight modes: ANGLE={self.angle_mode_enabled}, ALTHOLD={self.althold_enabled}')
        self.get_logger().info('=' * 80)

    # ==================== Callbacks ====================

    def velocity_cmd_callback(self, msg: Twist):
        """Receive velocity command from AI (body frame)"""
        self.velocity_cmd_body[0] = msg.linear.x  # Forward
        self.velocity_cmd_body[1] = msg.linear.y  # Right
        self.velocity_cmd_body[2] = msg.linear.z  # Up

        self.last_cmd_time = time.time()
        self.ai_enabled = True

    def gps_speed_callback(self, msg: Float32MultiArray):
        """
        GPS speed and course (earth frame).

        Format: [speed_mps, course_deg]
        Course is degrees clockwise from True North (0-360)
        """
        if len(msg.data) >= 2:
            self.gps_speed_mps = float(msg.data[0])
            self.gps_course_deg = float(msg.data[1])

            # Convert to earth-frame velocity (ENU convention)
            course_rad = math.radians(self.gps_course_deg)
            self.velocity_actual_earth[0] = self.gps_speed_mps * math.sin(course_rad)  # East
            self.velocity_actual_earth[1] = self.gps_speed_mps * math.cos(course_rad)  # North

    def attitude_callback(self, msg: Vector3Stamped):
        """Current attitude (roll, pitch, yaw) in radians"""
        self.attitude_euler[0] = msg.vector.x  # Roll
        self.attitude_euler[1] = msg.vector.y  # Pitch
        self.attitude_euler[2] = msg.vector.z  # Yaw

    def altitude_callback(self, msg: Float32MultiArray):
        """
        Altitude data from barometer.

        Format: [baro_altitude_m, vertical_velocity_mps]
        """
        if len(msg.data) >= 2:
            self.altitude_vz = float(msg.data[1])  # Vertical velocity
            self.velocity_actual_earth[2] = self.altitude_vz  # Up

    def safety_callback(self, msg: Bool):
        """Safety override"""
        self.safety_override = msg.data
        if self.safety_override:
            self.get_logger().warn('Safety override activated - stopping commands')
            self.velocity_controller.reset()

    # ==================== Control Loop ====================

    def control_loop(self):
        """Main control loop - runs at control_rate Hz"""

        # Check command timeout
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.ai_enabled:
                self.get_logger().warn('Command timeout - sending hover', throttle_duration_sec=5.0)
                self.send_hover_command()
            return

        # Check safety override
        if self.safety_override:
            return

        # Check if AI is enabled
        if not self.ai_enabled:
            return

        # Transform earth-frame velocity to body frame
        self.transform_velocity_to_body_frame()

        # Compute RC commands
        if self.closed_loop_enabled:
            # Closed-loop: Use PID with velocity feedback
            rc_channels = self.compute_rc_closed_loop()
        else:
            # Open-loop: Direct velocity-to-RC mapping (not recommended!)
            rc_channels = self.compute_rc_open_loop()

        # Send MSP command
        self.send_msp_rc_command(rc_channels)

        # Publish velocity error for monitoring
        self.publish_velocity_error()

        self.stats['control_loops'] += 1

    def transform_velocity_to_body_frame(self):
        """Transform earth-frame velocity to body frame using current yaw"""
        yaw = self.attitude_euler[2]

        # Rotation matrix (earth → body, 2D horizontal)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        vx_earth = self.velocity_actual_earth[0]  # East
        vy_earth = self.velocity_actual_earth[1]  # North

        # Transform horizontal velocity
        self.velocity_actual_body[0] = vx_earth * cos_yaw + vy_earth * sin_yaw    # Forward
        self.velocity_actual_body[1] = -vx_earth * sin_yaw + vy_earth * cos_yaw   # Right

        # Vertical velocity is the same in both frames
        self.velocity_actual_body[2] = self.velocity_actual_earth[2]

    def compute_rc_closed_loop(self) -> list:
        """Compute RC channels using PID control with velocity feedback"""

        # Compute PID outputs (RC deviations from center)
        pitch_dev, roll_dev, throttle_dev = self.velocity_controller.compute(
            vel_cmd_x=self.velocity_cmd_body[0],
            vel_cmd_y=self.velocity_cmd_body[1],
            vel_cmd_z=self.velocity_cmd_body[2],
            vel_actual_x=self.velocity_actual_body[0],
            vel_actual_y=self.velocity_actual_body[1],
            vel_actual_z=self.velocity_actual_body[2],
            dt=1.0 / self.control_rate
        )

        # Convert to RC values
        roll_rc = int(1500 + roll_dev)
        pitch_rc = int(1500 + pitch_dev)
        throttle_rc = int(1500 + throttle_dev)
        yaw_rc = 1500  # Maintain current yaw

        # Clamp to safe range
        roll_rc = max(self.rc_min, min(self.rc_max, roll_rc))
        pitch_rc = max(self.rc_min, min(self.rc_max, pitch_rc))
        throttle_rc = max(self.rc_min, min(self.rc_max, throttle_rc))

        return [roll_rc, pitch_rc, throttle_rc, yaw_rc]

    def compute_rc_open_loop(self) -> list:
        """
        Open-loop velocity-to-RC mapping (fallback, not recommended).

        This is a simple linear mapping without feedback.
        """
        vx = self.velocity_cmd_body[0]
        vy = self.velocity_cmd_body[1]
        vz = self.velocity_cmd_body[2]

        # Simple linear mapping
        gain_xy = 400.0 / self.max_velocity  # 400 RC units per max_velocity
        gain_z = 300.0 / self.max_velocity   # More conservative for vertical

        pitch_rc = int(1500 + vx * gain_xy)
        roll_rc = int(1500 + vy * gain_xy)
        throttle_rc = int(1500 + vz * gain_z)
        yaw_rc = 1500

        # Clamp
        roll_rc = max(self.rc_min, min(self.rc_max, roll_rc))
        pitch_rc = max(self.rc_min, min(self.rc_max, pitch_rc))
        throttle_rc = max(self.rc_min, min(self.rc_max, throttle_rc))

        return [roll_rc, pitch_rc, throttle_rc, yaw_rc]

    def send_msp_rc_command(self, rc_channels: list):
        """Send MSP_SET_RAW_RC command to flight controller"""

        # Build full 8-channel array
        channels = [1500] * 8
        channels[0] = rc_channels[0]  # Roll
        channels[1] = rc_channels[1]  # Pitch
        channels[2] = rc_channels[2]  # Throttle
        channels[3] = rc_channels[3]  # Yaw

        # Aux channels for flight modes
        channels[4] = 1800  # Aux 1: ARM (1000=disarmed, 1800=armed)

        if self.angle_mode_enabled:
            channels[5] = 1800  # Aux 2: ANGLE mode
        else:
            channels[5] = 1000

        if self.althold_enabled:
            channels[6] = 1800  # Aux 3: ALTHOLD mode
        else:
            channels[6] = 1000

        # Prepare MSP message
        # Format: [MSP_COMMAND_CODE, channel0, channel1, ..., channel7]
        msp_data = [float(MSPCommand.MSP_SET_RAW_RC)] + [float(ch) for ch in channels]

        msp_msg = Float32MultiArray()
        msp_msg.data = msp_data
        self.msp_cmd_pub.publish(msp_msg)

        self.stats['commands_sent'] += 1

        # Log periodically
        if self.stats['commands_sent'] % 30 == 0:
            self.get_logger().info(
                f'RC cmd: R={channels[0]}, P={channels[1]}, T={channels[2]}, Y={channels[3]} | '
                f'Vel err: vx={self.velocity_cmd_body[0]-self.velocity_actual_body[0]:.2f}, '
                f'vy={self.velocity_cmd_body[1]-self.velocity_actual_body[1]:.2f}, '
                f'vz={self.velocity_cmd_body[2]-self.velocity_actual_body[2]:.2f} m/s'
            )

    def send_hover_command(self):
        """Send hover command (zero velocity)"""
        hover_channels = [1500, 1500, 1500, 1500]
        self.send_msp_rc_command(hover_channels)
        self.stats['failsafe_activations'] += 1

    def publish_velocity_error(self):
        """Publish velocity tracking error for monitoring"""
        error_msg = Vector3Stamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'body'

        error_msg.vector.x = self.velocity_cmd_body[0] - self.velocity_actual_body[0]
        error_msg.vector.y = self.velocity_cmd_body[1] - self.velocity_actual_body[1]
        error_msg.vector.z = self.velocity_cmd_body[2] - self.velocity_actual_body[2]

        self.vel_error_pub.publish(error_msg)

    def publish_status(self):
        """Publish adapter status"""
        status_parts = []

        if self.ai_enabled:
            status_parts.append("AI_ENABLED")
        if self.safety_override:
            status_parts.append("SAFETY_OVERRIDE")
        if self.closed_loop_enabled:
            status_parts.append("CLOSED_LOOP")
        else:
            status_parts.append("OPEN_LOOP")

        if time.time() - self.last_cmd_time > self.cmd_timeout:
            status_parts.append("CMD_TIMEOUT")

        status_msg = String()
        status_msg.data = " | ".join(status_parts) if status_parts else "STANDBY"
        self.status_pub.publish(status_msg)

        # Log stats
        if self.stats['control_loops'] % 100 == 0 and self.stats['control_loops'] > 0:
            self.get_logger().info(
                f'Stats: loops={self.stats["control_loops"]}, '
                f'cmds_sent={self.stats["commands_sent"]}, '
                f'failsafes={self.stats["failsafe_activations"]}'
            )


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