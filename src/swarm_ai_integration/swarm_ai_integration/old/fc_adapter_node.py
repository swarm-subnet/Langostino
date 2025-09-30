#!/usr/bin/env python3
"""
Flight Controller Adapter Node - VEL to MSP Command Translation

This node transforms AI model velocity commands to MSP commands for INAV 7.
It performs the following functions:
- Converts Twist (velocity) messages to MSP RC commands
- Validates commands are within safe operational limits
- Implements command rate limiting and smoothing
- Provides failsafe behaviors for invalid commands
- Manages flight mode transitions for autonomous control

The node acts as a bridge between high-level velocity commands from the AI
and low-level MSP protocol commands understood by INAV.
"""

import numpy as np
import time
import math
from typing import Optional, List, Tuple, Dict
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String, Float32MultiArray, Header
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import Joy

from ..msp_protocol import MSPVelocityController, MSPMessage, MSPCommand, MSPDataTypes


class FCAdapterNode(Node):
    """
    Flight Controller Adapter Node for VEL to MSP command translation.

    Subscribers:
        /cmd_vel (geometry_msgs/Twist): Velocity commands from AI
        /safety/override (std_msgs/Bool): Safety override signal
        /fc/attitude (geometry_msgs/Vector3Stamped): Current drone attitude
        /manual_control (sensor_msgs/Joy): Manual RC override

    Publishers:
        /fc/rc_override (std_msgs/Float32MultiArray): MSP RC commands
        /fc/msp_command (std_msgs/Float32MultiArray): Raw MSP commands
        /fc_adapter/status (std_msgs/String): Adapter status
        /fc_adapter/diagnostics (std_msgs/Float32MultiArray): Performance metrics
    """

    def __init__(self):
        super().__init__('fc_adapter_node')

        # Declare parameters
        self.declare_parameter('max_velocity', 5.0)           # m/s
        self.declare_parameter('max_yaw_rate', 180.0)         # deg/s
        self.declare_parameter('max_acceleration', 3.0)       # m/s²
        self.declare_parameter('command_timeout', 1.0)        # seconds
        self.declare_parameter('rate_limit_enabled', True)
        self.declare_parameter('smooth_commands', True)
        self.declare_parameter('safety_checks_enabled', True)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('failsafe_mode', 'hover')      # hover, land, disarm

        # Get parameters
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        self.rate_limit_enabled = self.get_parameter('rate_limit_enabled').get_parameter_value().bool_value
        self.smooth_commands = self.get_parameter('smooth_commands').get_parameter_value().bool_value
        self.safety_checks_enabled = self.get_parameter('safety_checks_enabled').get_parameter_value().bool_value
        self.auto_arm = self.get_parameter('auto_arm').get_parameter_value().bool_value
        self.failsafe_mode = self.get_parameter('failsafe_mode').get_parameter_value().string_value

        # MSP velocity controller
        self.msp_controller = MSPVelocityController()
        self.msp_controller.max_velocity_ms = self.max_velocity
        self.msp_controller.max_yaw_rate_dps = self.max_yaw_rate

        # State variables
        self.current_velocity_cmd = np.zeros(4)  # [vx, vy, vz, yaw_rate]
        self.last_velocity_cmd = np.zeros(4)
        self.current_attitude = np.zeros(3)      # [roll, pitch, yaw]
        self.safety_override = False
        self.manual_override = False
        self.armed = False
        self.autonomous_mode = False

        # Command history for smoothing
        self.command_history = deque(maxlen=5)
        self.last_command_time = time.time()

        # Statistics
        self.stats = {
            'commands_processed': 0,
            'commands_rejected': 0,
            'rate_limited_commands': 0,
            'safety_violations': 0,
            'failsafe_activations': 0
        }

        # Safety limits (additional validation beyond parameters)
        self.absolute_limits = {
            'max_velocity': min(self.max_velocity, 10.0),     # Absolute maximum
            'max_yaw_rate': min(self.max_yaw_rate, 360.0),   # Absolute maximum
            'max_tilt_angle': 30.0,                          # degrees
            'min_throttle': 1100,                            # MSP value
            'max_throttle': 1900                             # MSP value
        }

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, control_qos)
        self.safety_sub = self.create_subscription(
            Bool, '/safety/override', self.safety_callback, reliable_qos)
        self.attitude_sub = self.create_subscription(
            Vector3Stamped, '/fc/attitude', self.attitude_callback, reliable_qos)
        self.manual_sub = self.create_subscription(
            Joy, '/manual_control', self.manual_control_callback, control_qos)

        # Publishers
        self.rc_pub = self.create_publisher(
            Float32MultiArray, '/fc/rc_override', control_qos)
        self.msp_cmd_pub = self.create_publisher(
            Float32MultiArray, '/fc/msp_command', reliable_qos)
        self.status_pub = self.create_publisher(
            String, '/fc_adapter/status', reliable_qos)
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray, '/fc_adapter/diagnostics', reliable_qos)

        # Timers
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)    # 1 Hz

        # Initialize MSP commands for autonomous mode
        if self.auto_arm:
            self.setup_autonomous_mode()

        self.get_logger().info('FC Adapter Node initialized')

    def cmd_vel_callback(self, msg: Twist):
        """Process velocity command from AI system"""
        current_time = time.time()
        self.last_command_time = current_time

        try:
            # Extract velocity components
            vx = msg.linear.x   # Forward velocity (m/s)
            vy = msg.linear.y   # Right velocity (m/s)
            vz = msg.linear.z   # Up velocity (m/s)
            yaw_rate = msg.angular.z  # Yaw rate (rad/s)

            # Convert yaw rate to degrees/second
            yaw_rate_dps = yaw_rate * 180.0 / math.pi

            # Create velocity command array
            velocity_cmd = np.array([vx, vy, vz, yaw_rate_dps])

            # Validate and process command
            if self.validate_command(velocity_cmd):
                processed_cmd = self.process_command(velocity_cmd)
                self.execute_command(processed_cmd)
                self.stats['commands_processed'] += 1
            else:
                self.stats['commands_rejected'] += 1
                self.get_logger().warn('Velocity command rejected - validation failed')

        except Exception as e:
            self.get_logger().error(f'Error processing velocity command: {e}')
            self.stats['commands_rejected'] += 1

    def validate_command(self, cmd: np.ndarray) -> bool:
        """Validate velocity command against safety limits"""
        if not self.safety_checks_enabled:
            return True

        # Check for NaN or infinite values
        if not np.all(np.isfinite(cmd)):
            self.get_logger().warn('Command contains NaN or infinite values')
            return False

        # Check velocity magnitude
        velocity_magnitude = np.linalg.norm(cmd[:3])
        if velocity_magnitude > self.absolute_limits['max_velocity']:
            self.get_logger().warn(f'Velocity too high: {velocity_magnitude:.2f} > {self.absolute_limits["max_velocity"]}')
            self.stats['safety_violations'] += 1
            return False

        # Check yaw rate
        if abs(cmd[3]) > self.absolute_limits['max_yaw_rate']:
            self.get_logger().warn(f'Yaw rate too high: {abs(cmd[3]):.2f} > {self.absolute_limits["max_yaw_rate"]}')
            self.stats['safety_violations'] += 1
            return False

        # Check if safety override is active
        if self.safety_override:
            self.get_logger().debug('Command rejected - safety override active')
            return False

        # Check if manual override is active
        if self.manual_override:
            self.get_logger().debug('Command rejected - manual override active')
            return False

        return True

    def process_command(self, cmd: np.ndarray) -> np.ndarray:
        """Process and condition velocity command"""
        processed_cmd = cmd.copy()

        # Apply rate limiting
        if self.rate_limit_enabled and len(self.command_history) > 0:
            processed_cmd = self.apply_rate_limiting(processed_cmd)

        # Apply command smoothing
        if self.smooth_commands:
            processed_cmd = self.apply_command_smoothing(processed_cmd)

        # Store in history
        self.command_history.append(processed_cmd)
        self.last_velocity_cmd = self.current_velocity_cmd.copy()
        self.current_velocity_cmd = processed_cmd

        return processed_cmd

    def apply_rate_limiting(self, cmd: np.ndarray) -> np.ndarray:
        """Apply acceleration/rate limiting to commands"""
        if len(self.command_history) == 0:
            return cmd

        dt = 0.1  # Assume 10Hz command rate
        last_cmd = self.command_history[-1]

        # Calculate desired acceleration
        desired_accel = (cmd[:3] - last_cmd[:3]) / dt

        # Limit acceleration
        accel_magnitude = np.linalg.norm(desired_accel)
        if accel_magnitude > self.max_acceleration:
            scale_factor = self.max_acceleration / accel_magnitude
            limited_accel = desired_accel * scale_factor
            cmd[:3] = last_cmd[:3] + limited_accel * dt
            self.stats['rate_limited_commands'] += 1

        # Limit yaw rate change
        max_yaw_accel = 90.0  # deg/s²
        yaw_rate_change = cmd[3] - last_cmd[3]
        if abs(yaw_rate_change / dt) > max_yaw_accel:
            sign = 1 if yaw_rate_change > 0 else -1
            cmd[3] = last_cmd[3] + sign * max_yaw_accel * dt

        return cmd

    def apply_command_smoothing(self, cmd: np.ndarray) -> np.ndarray:
        """Apply low-pass filtering to smooth commands"""
        if len(self.command_history) < 2:
            return cmd

        # Simple exponential moving average
        alpha = 0.7  # Smoothing factor (0 = no smoothing, 1 = no filtering)
        last_cmd = self.command_history[-1]
        smoothed_cmd = alpha * cmd + (1 - alpha) * last_cmd

        return smoothed_cmd

    def execute_command(self, cmd: np.ndarray):
        """Convert velocity command to MSP RC commands and send"""
        try:
            # Convert to RC channel values
            rc_channels = self.msp_controller.velocity_to_rc_channels(
                cmd[0], cmd[1], cmd[2], cmd[3] * math.pi / 180.0  # Convert back to rad/s
            )

            # Add additional channels for mode control
            full_rc_channels = rc_channels.copy()

            # Extend to 8 channels
            while len(full_rc_channels) < 8:
                full_rc_channels.append(1500)

            # Set arming channel (channel 5)
            if self.armed or self.auto_arm:
                full_rc_channels[4] = 2000  # Armed
            else:
                full_rc_channels[4] = 1000  # Disarmed

            # Set flight mode channels (channels 6-8) for autonomous mode
            if self.autonomous_mode:
                full_rc_channels[5] = 2000  # Enable autonomous mode
                full_rc_channels[6] = 1500  # Mode switch 2
                full_rc_channels[7] = 1500  # Mode switch 3

            # Validate RC values are within safe range
            full_rc_channels = [max(1000, min(2000, int(ch))) for ch in full_rc_channels]

            # Publish RC override command
            rc_msg = Float32MultiArray()
            rc_msg.data = [float(ch) for ch in full_rc_channels]
            self.rc_pub.publish(rc_msg)

            # Also send as raw MSP command for direct control
            msp_msg = Float32MultiArray()
            msp_msg.data = [float(MSPCommand.MSP_SET_RAW_RC)] + [float(ch) for ch in full_rc_channels]
            self.msp_cmd_pub.publish(msp_msg)

        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')

    def safety_callback(self, msg: Bool):
        """Handle safety override signal"""
        self.safety_override = msg.data

        if self.safety_override:
            self.activate_failsafe()
            self.get_logger().warn('Safety override activated - entering failsafe mode')
        else:
            self.get_logger().info('Safety override deactivated')

    def attitude_callback(self, msg: Vector3Stamped):
        """Update current attitude for safety monitoring"""
        self.current_attitude = np.array([
            msg.vector.x,  # roll (rad)
            msg.vector.y,  # pitch (rad)
            msg.vector.z   # yaw (rad)
        ])

        # Check for dangerous attitudes
        if self.safety_checks_enabled:
            max_tilt_rad = math.radians(self.absolute_limits['max_tilt_angle'])
            if abs(self.current_attitude[0]) > max_tilt_rad or abs(self.current_attitude[1]) > max_tilt_rad:
                self.get_logger().warn('Dangerous attitude detected - activating failsafe')
                self.activate_failsafe()

    def manual_control_callback(self, msg: Joy):
        """Handle manual control override"""
        # Check if manual override switch is active (assuming button 0)
        if len(msg.buttons) > 0:
            self.manual_override = bool(msg.buttons[0])

            if self.manual_override:
                self.get_logger().info('Manual override activated')
                # Send manual stick inputs directly
                if len(msg.axes) >= 4:
                    self.send_manual_rc_commands(msg.axes)

    def send_manual_rc_commands(self, axes: List[float]):
        """Send manual RC commands from joystick"""
        # Convert joystick axes to RC values
        rc_channels = [1500] * 8  # Initialize to center

        if len(axes) >= 4:
            # Map axes to RC channels (adjust mapping as needed)
            rc_channels[0] = int(1500 + axes[0] * 400)  # Roll
            rc_channels[1] = int(1500 + axes[1] * 400)  # Pitch
            rc_channels[2] = int(1500 + axes[2] * 400)  # Throttle
            rc_channels[3] = int(1500 + axes[3] * 400)  # Yaw

        # Clamp values
        rc_channels = [max(1000, min(2000, ch)) for ch in rc_channels]

        # Send manual RC commands
        rc_msg = Float32MultiArray()
        rc_msg.data = [float(ch) for ch in rc_channels]
        self.rc_pub.publish(rc_msg)

    def activate_failsafe(self):
        """Activate failsafe behavior"""
        self.stats['failsafe_activations'] += 1

        if self.failsafe_mode == 'hover':
            # Send hover command (zero velocity)
            self.send_hover_command()
        elif self.failsafe_mode == 'land':
            # Send landing command
            self.send_landing_command()
        elif self.failsafe_mode == 'disarm':
            # Disarm the aircraft
            self.send_disarm_command()

    def send_hover_command(self):
        """Send hover command (zero velocities)"""
        hover_channels = [1500, 1500, 1500, 1500, 2000, 1500, 1500, 1500]  # Center stick, armed

        rc_msg = Float32MultiArray()
        rc_msg.data = [float(ch) for ch in hover_channels]
        self.rc_pub.publish(rc_msg)

    def send_landing_command(self):
        """Send controlled landing command"""
        # Gradual descent at -0.5 m/s
        landing_velocity = np.array([0.0, 0.0, -0.5, 0.0])  # [vx, vy, vz, yaw_rate]

        rc_channels = self.msp_controller.velocity_to_rc_channels(
            landing_velocity[0], landing_velocity[1], landing_velocity[2], landing_velocity[3]
        )

        # Extend to full channel set
        while len(rc_channels) < 8:
            rc_channels.append(1500)
        rc_channels[4] = 2000  # Keep armed during landing

        rc_msg = Float32MultiArray()
        rc_msg.data = [float(ch) for ch in rc_channels]
        self.rc_pub.publish(rc_msg)

    def send_disarm_command(self):
        """Send disarm command to flight controller"""
        disarm_channels = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]  # All low, disarmed

        rc_msg = Float32MultiArray()
        rc_msg.data = [float(ch) for ch in disarm_channels]
        self.rc_pub.publish(rc_msg)

    def watchdog_check(self):
        """Check for command timeouts and system health"""
        current_time = time.time()

        # Check command timeout
        if current_time - self.last_command_time > self.command_timeout:
            if not self.safety_override and not self.manual_override:
                self.get_logger().warn('Command timeout - activating hover mode')
                self.send_hover_command()

        # Periodic system health check
        if current_time % 10 < 0.1:  # Every 10 seconds
            self.perform_health_check()

    def perform_health_check(self):
        """Perform periodic system health check"""
        # Check for excessive safety violations
        if self.stats['safety_violations'] > 100:
            self.get_logger().warn('Excessive safety violations detected')

        # Check command processing rate
        if self.stats['commands_processed'] == 0:
            self.get_logger().warn('No commands processed - check AI system')

        # Reset counters periodically
        if time.time() % 60 < 0.1:  # Every minute
            self.stats = {key: 0 for key in self.stats}

    def setup_autonomous_mode(self):
        """Setup flight controller for autonomous operation"""
        # Send MSP commands to configure autonomous mode
        # This would typically involve setting flight modes, enabling GPS, etc.

        self.autonomous_mode = True
        self.get_logger().info('Autonomous mode configured')

    def publish_status(self):
        """Publish adapter status and diagnostics"""
        # Status message
        status_parts = []

        if self.safety_override:
            status_parts.append("SAFETY_OVERRIDE")
        if self.manual_override:
            status_parts.append("MANUAL_OVERRIDE")
        if self.autonomous_mode:
            status_parts.append("AUTONOMOUS")
        if self.armed:
            status_parts.append("ARMED")

        status_msg = String()
        status_msg.data = " | ".join(status_parts) if status_parts else "STANDBY"
        self.status_pub.publish(status_msg)

        # Diagnostics message
        diagnostics_data = [
            float(self.stats['commands_processed']),
            float(self.stats['commands_rejected']),
            float(self.stats['rate_limited_commands']),
            float(self.stats['safety_violations']),
            float(self.stats['failsafe_activations']),
            float(np.linalg.norm(self.current_velocity_cmd[:3])),  # Current velocity magnitude
            float(abs(self.current_velocity_cmd[3])),  # Current yaw rate
            float(time.time() - self.last_command_time)  # Time since last command
        ]

        diag_msg = Float32MultiArray()
        diag_msg.data = diagnostics_data
        self.diagnostics_pub.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FCAdapterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()