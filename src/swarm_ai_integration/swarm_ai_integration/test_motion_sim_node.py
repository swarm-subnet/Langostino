#!/usr/bin/env python3
"""
Test Motion Simulator Node

Simulates drone motion physics to test fc_adapter_node's response to various commands.
Publishes coherent sensor data based on commanded actions and subscribes to RC outputs.

Publications:
  /ai/action              (std_msgs/Float32MultiArray) : [vx, vy, vz, speed]
  /fc/gps_speed_course    (std_msgs/Float32MultiArray) : [speed_mps, course_deg]
  /fc/attitude_euler      (geometry_msgs/Vector3Stamped): roll, pitch, yaw (rad)
  /fc/altitude            (std_msgs/Float32MultiArray) : [baro_alt_m, vz_mps]
  /safety/override        (std_msgs/Bool)

Subscriptions:
  /fc/rc_override         (std_msgs/Float32MultiArray) : [ch1..ch9]
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Vector3Stamped


class TestMotionSimNode(Node):
    """
    Test node that simulates drone physics and generates coherent sensor data.
    """

    def __init__(self):
        super().__init__('test_motion_sim_node')

        # ------------ Parameters ------------
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('test_sequence_enabled', True)
        self.declare_parameter('initial_altitude', 10.0)

        # Physics parameters
        self.declare_parameter('acceleration_rate', 2.0)  # m/s^2
        self.declare_parameter('pitch_per_accel', 0.1)    # radians per m/s^2
        self.declare_parameter('roll_per_accel', 0.1)     # radians per m/s^2
        self.declare_parameter('attitude_damping', 0.8)   # attitude return rate

        # Get parameter values
        self.update_rate = float(self.get_parameter('update_rate_hz').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.test_sequence_enabled = bool(self.get_parameter('test_sequence_enabled').value)
        self.initial_altitude = float(self.get_parameter('initial_altitude').value)
        self.accel_rate = float(self.get_parameter('acceleration_rate').value)
        self.pitch_per_accel = float(self.get_parameter('pitch_per_accel').value)
        self.roll_per_accel = float(self.get_parameter('roll_per_accel').value)
        self.attitude_damping = float(self.get_parameter('attitude_damping').value)

        # ------------ State Variables (ENU frame) ------------
        # Position (East, North, Up)
        self.position = [0.0, 0.0, self.initial_altitude]

        # Velocity (East, North, Up) - actual velocity
        self.velocity = [0.0, 0.0, 0.0]

        # Target velocity from command
        self.target_velocity = [0.0, 0.0, 0.0]

        # Attitude (roll, pitch, yaw in radians)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Current action command
        self.current_action = [0.0, 0.0, 0.0, 0.0]

        # Last RC override received
        self.last_rc_override: Optional[list] = None

        # Test sequence state
        self.test_index = 0
        self.test_hold_counter = 0
        self.test_hold_duration = int(3.0 * self.update_rate)  # Hold each command for 3 seconds

        # Define test sequence: [vx, vy, vz, speed, description]
        self.test_sequence = [
            ([0.0, 0.0, 0.0, 0.0], "Hover - no movement"),
            ([1.0, 0.0, 0.0, 0.5], "Move forward at 50% speed"),
            ([1.0, 0.0, 0.0, 1.0], "Move forward at 100% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop"),
            ([-1.0, 0.0, 0.0, 0.5], "Move backward at 50% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop"),
            ([0.0, 1.0, 0.0, 0.5], "Move right at 50% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop"),
            ([0.0, -1.0, 0.0, 0.5], "Move left at 50% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop"),
            ([0.0, 0.0, 1.0, 0.5], "Climb at 50% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - maintain altitude"),
            ([0.0, 0.0, -1.0, 0.5], "Descend at 50% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop"),
            ([1.0, 1.0, 0.0, 0.7], "Move forward-right diagonal at 70% speed"),
            ([0.0, 0.0, 0.0, 0.0], "Hover - final stop"),
        ]

        # ------------ QoS Profiles ------------
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

        # ------------ Publishers ------------
        self.action_pub = self.create_publisher(
            Float32MultiArray, '/ai/action', control_qos)
        self.gps_speed_course_pub = self.create_publisher(
            Float32MultiArray, '/fc/gps_speed_course', sensor_qos)
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, '/fc/attitude_euler', sensor_qos)
        self.altitude_pub = self.create_publisher(
            Float32MultiArray, '/fc/altitude', sensor_qos)
        self.safety_pub = self.create_publisher(
            Bool, '/safety/override', control_qos)

        # ------------ Subscribers ------------
        self.create_subscription(
            Float32MultiArray, '/fc/rc_override', self.cb_rc_override, control_qos)

        # ------------ Timers ------------
        self.update_period = 1.0 / self.update_rate
        self.create_timer(self.update_period, self.update_loop)

        # Initial safety override to False
        safety_msg = Bool()
        safety_msg.data = False
        self.safety_pub.publish(safety_msg)

        self.get_logger().info(
            f'Test Motion Simulator Node started @ {self.update_rate}Hz\n'
            f'  Max velocity: {self.max_velocity} m/s\n'
            f'  Initial altitude: {self.initial_altitude} m\n'
            f'  Test sequence: {"ENABLED" if self.test_sequence_enabled else "DISABLED"}'
        )

        if self.test_sequence_enabled:
            self.get_logger().info(f'  Test sequence has {len(self.test_sequence)} commands')

    # -------------------- Callback --------------------
    def cb_rc_override(self, msg: Float32MultiArray):
        """Receive RC override output from fc_adapter_node"""
        self.last_rc_override = list(msg.data)

    # -------------------- Update Loop --------------------
    def update_loop(self):
        """Main simulation loop - updates physics and publishes sensor data"""
        dt = self.update_period

        # Generate or update action command
        if self.test_sequence_enabled:
            self._update_test_sequence()
        else:
            # If test sequence disabled, just hover
            self.current_action = [0.0, 0.0, 0.0, 0.0]

        # Calculate target velocity from action command
        vx_cmd, vy_cmd, vz_cmd, speed_scalar = self.current_action

        # Normalize direction vector and apply speed
        direction_magnitude = math.sqrt(vx_cmd**2 + vy_cmd**2 + vz_cmd**2)

        if direction_magnitude > 1e-6:
            # Normalize and scale by speed and max_velocity
            scale = (speed_scalar * self.max_velocity) / direction_magnitude
            self.target_velocity[0] = vx_cmd * scale  # East
            self.target_velocity[1] = vy_cmd * scale  # North
            self.target_velocity[2] = vz_cmd * scale  # Up
        else:
            # No command, target is zero velocity
            self.target_velocity = [0.0, 0.0, 0.0]

        # Simulate acceleration toward target velocity
        for i in range(3):
            vel_error = self.target_velocity[i] - self.velocity[i]
            accel = self.accel_rate * math.copysign(min(abs(vel_error), 1.0), vel_error)
            self.velocity[i] += accel * dt

        # Update position based on velocity
        for i in range(3):
            self.position[i] += self.velocity[i] * dt

        # Ensure altitude doesn't go below ground
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            self.velocity[2] = max(0.0, self.velocity[2])

        # Update attitude based on velocity and acceleration
        self._update_attitude(dt)

        # Publish all sensor data
        self._publish_sensor_data()

        # Print current state with RC output
        self._print_state()

    # -------------------- Helper Methods --------------------
    def _update_test_sequence(self):
        """Update action command based on test sequence"""
        self.test_hold_counter += 1

        if self.test_hold_counter >= self.test_hold_duration:
            # Move to next test command
            self.test_hold_counter = 0
            self.test_index = (self.test_index + 1) % len(self.test_sequence)

            action, description = self.test_sequence[self.test_index]
            self.current_action = list(action)

            self.get_logger().info(
                f'\n{"="*80}\n'
                f'  TEST {self.test_index + 1}/{len(self.test_sequence)}: {description}\n'
                f'  Action: [vx={action[0]:+.1f}, vy={action[1]:+.1f}, vz={action[2]:+.1f}, speed={action[3]:.1f}]\n'
                f'{"="*80}'
            )

    def _update_attitude(self, dt: float):
        """Update roll, pitch, yaw based on current motion"""
        # Calculate acceleration (simplified - just velocity change rate)
        vx_e, vy_n, vz_u = self.velocity

        # Yaw aligns with horizontal velocity direction
        velocity_magnitude_xy = math.sqrt(vx_e**2 + vy_n**2)
        if velocity_magnitude_xy > 0.1:
            # Yaw = atan2(East, North) to match GPS course
            target_yaw = math.atan2(vx_e, vy_n)

            # Smoothly interpolate yaw
            yaw_error = target_yaw - self.yaw
            # Normalize to [-pi, pi]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            self.yaw += yaw_error * self.attitude_damping * dt * 2.0
        else:
            # No horizontal velocity, maintain current yaw
            pass

        # Pitch proportional to forward velocity (body frame approximation)
        # For small angles, pitch ≈ vx * pitch_per_accel
        # Positive vx (East when yaw=0) -> positive pitch (nose up slightly during acceleration)
        target_pitch = vx_e * self.pitch_per_accel * math.cos(self.yaw) + vy_n * self.pitch_per_accel * math.sin(self.yaw)
        self.pitch += (target_pitch - self.pitch) * self.attitude_damping * dt * 5.0

        # Roll proportional to lateral velocity (body frame approximation)
        # Positive vy (North when yaw=0) -> positive roll (right wing down for right turn)
        target_roll = -vx_e * self.roll_per_accel * math.sin(self.yaw) + vy_n * self.roll_per_accel * math.cos(self.yaw)
        self.roll += (target_roll - self.roll) * self.attitude_damping * dt * 5.0

        # Clamp attitudes to reasonable values
        self.roll = max(-0.5, min(0.5, self.roll))    # ±28.6 degrees
        self.pitch = max(-0.5, min(0.5, self.pitch))  # ±28.6 degrees

    def _publish_sensor_data(self):
        """Publish all simulated sensor data"""
        # Publish action command
        action_msg = Float32MultiArray()
        action_msg.data = [float(x) for x in self.current_action]
        self.action_pub.publish(action_msg)

        # Publish GPS speed and course
        vx_e, vy_n, vz_u = self.velocity
        speed_mps = math.sqrt(vx_e**2 + vy_n**2)

        # Course in degrees (0=North, 90=East, 180=South, 270=West)
        # atan2(East, North) gives bearing in radians
        if speed_mps > 0.01:
            course_rad = math.atan2(vx_e, vy_n)
            course_deg = math.degrees(course_rad)
            # Normalize to [0, 360)
            if course_deg < 0:
                course_deg += 360.0
        else:
            course_deg = 0.0  # Stationary, course is undefined

        gps_msg = Float32MultiArray()
        gps_msg.data = [float(speed_mps), float(course_deg)]
        self.gps_speed_course_pub.publish(gps_msg)

        # Publish attitude (roll, pitch, yaw)
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = 'body'
        attitude_msg.vector.x = float(self.roll)
        attitude_msg.vector.y = float(self.pitch)
        attitude_msg.vector.z = float(self.yaw)
        self.attitude_pub.publish(attitude_msg)

        # Publish altitude (baro altitude, vertical velocity)
        altitude_msg = Float32MultiArray()
        altitude_msg.data = [float(self.position[2]), float(vz_u)]
        self.altitude_pub.publish(altitude_msg)

    def _print_state(self):
        """Print current state and RC output"""
        vx_e, vy_n, vz_u = self.velocity
        speed = math.sqrt(vx_e**2 + vy_n**2)

        # Format RC channels
        if self.last_rc_override is not None and len(self.last_rc_override) >= 4:
            rc_str = (f"[R={self.last_rc_override[0]:.0f}, "
                     f"P={self.last_rc_override[1]:.0f}, "
                     f"T={self.last_rc_override[2]:.0f}, "
                     f"Y={self.last_rc_override[3]:.0f}]")
            if len(self.last_rc_override) >= 9:
                rc_str += (f" AUX[{self.last_rc_override[4]:.0f}, "
                          f"{self.last_rc_override[5]:.0f}, "
                          f"{self.last_rc_override[6]:.0f}, "
                          f"{self.last_rc_override[7]:.0f}, "
                          f"{self.last_rc_override[8]:.0f}]")
        else:
            rc_str = "[waiting...]"

        self.get_logger().info(
            f"Action=[{self.current_action[0]:+.1f}, {self.current_action[1]:+.1f}, "
            f"{self.current_action[2]:+.1f}, s={self.current_action[3]:.2f}]  "
            f"Vel=[{vx_e:+.2f}, {vy_n:+.2f}, {vz_u:+.2f}]m/s  "
            f"Speed={speed:.2f}m/s  "
            f"Alt={self.position[2]:.1f}m  "
            f"Att=[r={math.degrees(self.roll):+.1f}°, p={math.degrees(self.pitch):+.1f}°, y={math.degrees(self.yaw):+.1f}°]  "
            f"RC_OUT={rc_str}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestMotionSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Test Motion Simulator Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
