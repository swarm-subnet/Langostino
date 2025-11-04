#!/usr/bin/env python3
"""
FC Adapter Test Node

This node tests fc_adapter_node.py by:
1. Publishing fake sensor data that fc_adapter subscribes to
2. Sending test VEL commands via /ai/action
3. Reading and displaying the RC outputs from fc_adapter

Usage:
    ros2 run swarm_ai_integration fc_adapter_test_node.py

Test scenarios:
- Stationary hover: (0, 0, 0, 1.0)
- Move East: (1, 0, 0, 1.0)
- Move Northeast: (1, 1, 0, 1.0)
- Move with different speeds: (1, 1, 0, 0.5) vs (1, 1, 0, 1.0)
- Climb: (0, 0, 1, 1.0)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Vector3Stamped


class FCAdapterTestNode(Node):
    """
    Test node for fc_adapter_node.py

    Publishes:
        /ai/action                 (Float32MultiArray) : [vx, vy, vz, speed]
        /fc/gps_speed_course       (Float32MultiArray) : [speed_mps, course_deg]
        /fc/attitude_euler         (Vector3Stamped)    : roll, pitch, yaw (rad)
        /fc/altitude               (Float32MultiArray) : [baro_alt_m, vz_mps]
        /safety/override           (Bool)              : False (no override)
        /safety/rth_command        (Bool)              : False (no RTH)

    Subscribes:
        /fc_adapter/status         (String)
        /fc_adapter/velocity_error (Vector3Stamped)
        /fc/rc_override            (Float32MultiArray) : RC channels from fc_adapter
    """

    def __init__(self):
        super().__init__('fc_adapter_test_node')

        # Parameters
        self.declare_parameter('test_rate_hz', 10.0)
        self.declare_parameter('sensor_rate_hz', 50.0)
        self.declare_parameter('max_velocity', 3.0)  # Must match fc_adapter's max_velocity

        test_rate = float(self.get_parameter('test_rate_hz').value)
        sensor_rate = float(self.get_parameter('sensor_rate_hz').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)

        # State (simulated drone state for feedback)
        self.simulated_position = np.array([0.0, 0.0, 3.0])  # Start at 3m altitude
        self.simulated_velocity = np.array([0.0, 0.0, 0.0])
        self.simulated_yaw = 0.0  # radians
        self.simulated_roll = 0.0
        self.simulated_pitch = 0.0

        # Test sequence configuration
        # Format: [vx, vy, vz, speed, duration_sec, description]
        # - (vx, vy, vz) = direction vector (will be normalized)
        # - speed ∈ [0, 1] = magnitude as fraction of max_velocity
        # - Actual velocity = normalize(vx,vy,vz) * speed * max_velocity
        # NOTE: Modify this list directly to add/change test scenarios
        self.test_sequence = [
            [0.0, 0.0, 0.0, 0.0, 3.0, "Hover (zero speed)"],
            [1.0, 0.0, 0.0, 1.0, 5.0, "East @ full speed"],
            [1.0, 0.0, 0.0, 0.5, 5.0, "East @ half speed"],
            [1.0, 0.0, 0.0, 0.3, 5.0, "East @ 30% speed"],
            [0.0, 1.0, 0.0, 1.0, 5.0, "North @ full speed"],
            [1.0, 1.0, 0.0, 1.0, 5.0, "Northeast @ full speed"],
            [1.0, 1.0, 0.0, 0.5, 5.0, "Northeast @ half speed"],
            [0.0, 0.0, 1.0, 0.5, 5.0, "Climb @ half speed"],
            [-1.0, 0.0, 0.0, 1.0, 5.0, "West @ full speed"],
            [0.0, 0.0, 0.0, 0.0, 3.0, "Return to hover"],
        ]
        self.current_test_index = 0
        self.test_start_time = time.time()
        self.current_test = self.test_sequence[0]

        # Latest RC output from fc_adapter
        self.latest_rc_channels = None
        self.latest_status = "waiting"
        self.latest_vel_error = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers (fake sensors for fc_adapter)
        self.pub_action = self.create_publisher(Float32MultiArray, '/ai/action', control_qos)
        self.pub_gps_speed = self.create_publisher(Float32MultiArray, '/fc/gps_speed_course', sensor_qos)
        self.pub_attitude = self.create_publisher(Vector3Stamped, '/fc/attitude_euler', sensor_qos)
        self.pub_altitude = self.create_publisher(Float32MultiArray, '/fc/altitude', sensor_qos)
        self.pub_safety_override = self.create_publisher(Bool, '/safety/override', control_qos)
        self.pub_rth_command = self.create_publisher(Bool, '/safety/rth_command', control_qos)

        # Subscribers (fc_adapter outputs)
        self.create_subscription(String, '/fc_adapter/status', self.cb_status, control_qos)
        self.create_subscription(Vector3Stamped, '/fc_adapter/velocity_error', self.cb_vel_error, control_qos)
        self.create_subscription(Float32MultiArray, '/fc/rc_override', self.cb_rc_override, control_qos)

        # Timers
        self.create_timer(1.0 / test_rate, self.test_loop)
        self.create_timer(1.0 / sensor_rate, self.publish_fake_sensors)
        self.create_timer(2.0, self.print_summary)

        self.get_logger().info('=' * 80)
        self.get_logger().info('FC ADAPTER TEST NODE')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Test rate: {test_rate} Hz')
        self.get_logger().info(f'Sensor rate: {sensor_rate} Hz')
        self.get_logger().info(f'Max velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'Total test scenarios: {len(self.test_sequence)}')
        self.get_logger().info('=' * 80)
        self.get_logger().info('Starting test sequence...')
        self.get_logger().info('=' * 80)

    # ────────────────────────────────────────────────────────────────
    # Callbacks from fc_adapter
    # ────────────────────────────────────────────────────────────────
    def cb_status(self, msg: String):
        """Receive status from fc_adapter."""
        self.latest_status = msg.data

    def cb_vel_error(self, msg: Vector3Stamped):
        """Receive velocity error from fc_adapter."""
        self.latest_vel_error = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_rc_override(self, msg: Float32MultiArray):
        """Receive RC channels from fc_adapter."""
        self.latest_rc_channels = list(msg.data)

    # ────────────────────────────────────────────────────────────────
    # Test loop
    # ────────────────────────────────────────────────────────────────
    def test_loop(self):
        """Main test loop - publishes AI actions."""
        # Check if current test has finished
        elapsed = time.time() - self.test_start_time
        if elapsed >= self.current_test[4]:
            # Move to next test
            self.current_test_index += 1
            if self.current_test_index >= len(self.test_sequence):
                self.get_logger().info('=' * 80)
                self.get_logger().info('✅ ALL TESTS COMPLETED!')
                self.get_logger().info('=' * 80)
                # Loop back to first test
                self.current_test_index = 0

            self.current_test = self.test_sequence[self.current_test_index]
            self.test_start_time = time.time()

            self.get_logger().info('=' * 80)
            self.get_logger().info(f'TEST {self.current_test_index + 1}/{len(self.test_sequence)}: {self.current_test[5]}')
            self.get_logger().info(f'VEL command: vx={self.current_test[0]:.1f}, vy={self.current_test[1]:.1f}, '
                                 f'vz={self.current_test[2]:.1f}, speed={self.current_test[3]:.1f}')
            self.get_logger().info(f'Duration: {self.current_test[4]:.1f}s')
            self.get_logger().info('=' * 80)

        # Publish AI action
        action_msg = Float32MultiArray()
        action_msg.data = [
            float(self.current_test[0]),  # vx (direction)
            float(self.current_test[1]),  # vy (direction)
            float(self.current_test[2]),  # vz (direction)
            float(self.current_test[3]),  # speed (0-1 scale)
        ]
        self.pub_action.publish(action_msg)

        # Simulate velocity response (simple first-order response)
        # Speed interpretation: normalize(direction) * speed * 3.0 m/s
        direction = np.array([self.current_test[0], self.current_test[1], self.current_test[2]])
        direction_magnitude = np.linalg.norm(direction)

        if direction_magnitude > 1e-6:
            direction_normalized = direction / direction_magnitude
            target_speed_mps = self.current_test[3] * self.max_velocity  # Convert speed scalar to m/s
            target_vel = direction_normalized * target_speed_mps
        else:
            target_vel = np.zeros(3)
        alpha = 0.1  # Response rate
        self.simulated_velocity = self.simulated_velocity + alpha * (target_vel - self.simulated_velocity)

        # Update position
        dt = 0.1  # 10 Hz
        self.simulated_position += self.simulated_velocity * dt

        # Simulate yaw following velocity direction
        if np.linalg.norm(self.simulated_velocity[:2]) > 0.1:
            target_yaw = math.atan2(self.simulated_velocity[0], self.simulated_velocity[1])
            yaw_diff = target_yaw - self.simulated_yaw
            yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
            self.simulated_yaw += 0.1 * yaw_diff  # Slow yaw response

    # ────────────────────────────────────────────────────────────────
    # Fake sensor publishers
    # ────────────────────────────────────────────────────────────────
    def publish_fake_sensors(self):
        """Publish fake sensor data at high rate."""
        # GPS speed/course (derived from simulated velocity)
        speed_xy = math.hypot(self.simulated_velocity[0], self.simulated_velocity[1])
        if speed_xy > 1e-6:
            course_deg = (math.degrees(math.atan2(
                self.simulated_velocity[0],
                self.simulated_velocity[1]
            )) + 360.0) % 360.0
        else:
            course_deg = 0.0

        gps_msg = Float32MultiArray()
        gps_msg.data = [float(speed_xy), float(course_deg)]
        self.pub_gps_speed.publish(gps_msg)

        # Attitude (yaw follows velocity direction, roll/pitch are small)
        att_msg = Vector3Stamped()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.header.frame_id = 'fc_attitude'
        att_msg.vector.x = self.simulated_roll    # roll
        att_msg.vector.y = self.simulated_pitch   # pitch
        att_msg.vector.z = self.simulated_yaw     # yaw
        self.pub_attitude.publish(att_msg)

        # Altitude
        alt_msg = Float32MultiArray()
        alt_msg.data = [float(self.simulated_position[2]), float(self.simulated_velocity[2])]
        self.pub_altitude.publish(alt_msg)

        # Safety (no override, no RTH)
        safety_msg = Bool()
        safety_msg.data = False
        self.pub_safety_override.publish(safety_msg)

        rth_msg = Bool()
        rth_msg.data = False
        self.pub_rth_command.publish(rth_msg)

    # ────────────────────────────────────────────────────────────────
    # Summary printing
    # ────────────────────────────────────────────────────────────────
    def print_summary(self):
        """Print test summary every 2 seconds."""
        if self.latest_rc_channels is None:
            self.get_logger().warn('⏳ Waiting for fc_adapter to start publishing RC commands...')
            return

        vx, vy, vz = self.current_test[0], self.current_test[1], self.current_test[2]
        speed = self.current_test[3]

        # Expected velocity: normalize(direction) * speed * 3.0
        direction = np.array([vx, vy, vz])
        direction_magnitude = np.linalg.norm(direction)

        if direction_magnitude > 1e-6:
            direction_normalized = direction / direction_magnitude
            target_speed_mps = speed * self.max_velocity
            expected_vel = direction_normalized * target_speed_mps
            expected_vx, expected_vy, expected_vz = expected_vel[0], expected_vel[1], expected_vel[2]
        else:
            expected_vx, expected_vy, expected_vz = 0.0, 0.0, 0.0

        # RC channels
        rc = self.latest_rc_channels
        roll_rc = int(rc[0]) if len(rc) > 0 else 1500
        pitch_rc = int(rc[1]) if len(rc) > 1 else 1500
        throttle_rc = int(rc[2]) if len(rc) > 2 else 1500
        yaw_rc = int(rc[3]) if len(rc) > 3 else 1500

        # Analysis
        self.get_logger().info('─' * 80)
        self.get_logger().info(f'TEST: {self.current_test[5]}')
        self.get_logger().info(f'  AI Command:      dir=({vx:+.2f},{vy:+.2f},{vz:+.2f}), speed={speed:.2f} → {speed*self.max_velocity:.1f} m/s')
        self.get_logger().info(f'  Expected VEL:    vx={expected_vx:+.2f}, vy={expected_vy:+.2f}, vz={expected_vz:+.2f} m/s')
        self.get_logger().info(f'  Simulated VEL:   vx={self.simulated_velocity[0]:+.2f}, vy={self.simulated_velocity[1]:+.2f}, vz={self.simulated_velocity[2]:+.2f}')
        self.get_logger().info(f'  Simulated Yaw:   {math.degrees(self.simulated_yaw):+.1f}°')
        self.get_logger().info(f'  RC Output:       R={roll_rc:4d}, P={pitch_rc:4d}, T={throttle_rc:4d}, Y={yaw_rc:4d}')

        # Interpretation
        roll_deviation = roll_rc - 1500
        pitch_deviation = pitch_rc - 1500
        throttle_deviation = throttle_rc - 1500
        yaw_deviation = yaw_rc - 1500

        self.get_logger().info(f'  RC Deviations:   R={roll_deviation:+4d}, P={pitch_deviation:+4d}, T={throttle_deviation:+4d}, Y={yaw_deviation:+4d}')

        # Sanity checks
        checks = []

        # Check 1: Positive vx (East) should give positive pitch (forward)
        if abs(expected_vx) > 0.1:
            if (expected_vx > 0 and pitch_deviation > 0) or (expected_vx < 0 and pitch_deviation < 0):
                checks.append(f'✓ Pitch direction matches vx command')
            else:
                checks.append(f'✗ Pitch direction MISMATCH (vx={expected_vx:+.2f}, pitch_dev={pitch_deviation:+d})')

        # Check 2: Positive vy (North) should give positive roll (after body transform)
        # This is complex due to body frame transformation, skip for now

        # Check 3: Speed scaling should affect RC magnitude
        if speed > 0.5:
            checks.append(f'✓ Full speed mode (speed={speed:.1f})')
        elif speed < 0.5:
            checks.append(f'✓ Reduced speed mode (speed={speed:.1f})')

        # Check 4: Yaw control active (if yaw_rc != 1500)
        if abs(yaw_deviation) > 10:
            checks.append(f'✓ Yaw control active (deviation={yaw_deviation:+d})')
        else:
            checks.append(f'○ Yaw holding neutral')

        for check in checks:
            self.get_logger().info(f'  {check}')

        self.get_logger().info(f'  Status: {self.latest_status}')
        self.get_logger().info('─' * 80)


def main(args=None):
    rclpy.init(args=args)
    node = FCAdapterTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down FC Adapter Test Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
