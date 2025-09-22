#!/usr/bin/env python3
"""
Safety Monitor Node - Monitors drone state and triggers failsafe when needed

This node implements safety monitoring for the AI flight system, including:
- Altitude limits
- Velocity limits
- Battery monitoring
- Communication timeouts
- Obstacle proximity
- Emergency landing procedures
"""

import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class SafetyMonitorNode(Node):
    """
    Monitors drone safety and triggers failsafe procedures when necessary.

    Subscribers:
        /ai/observation (std_msgs/Float32MultiArray): Current observation data
        /ai/action (geometry_msgs/Twist): AI control commands
        /battery_state (sensor_msgs/BatteryState): Battery status
        /drone/pose (geometry_msgs/PoseStamped): Current drone position
        /drone/velocity (geometry_msgs/Twist): Current drone velocity

    Publishers:
        /safety/override (std_msgs/Bool): Safety override signal
        /safety/status (std_msgs/String): Safety status messages
        /safety/emergency_land (std_msgs/Bool): Emergency landing command
        /failsafe/action (geometry_msgs/Twist): Failsafe control commands
    """

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Declare parameters
        self.declare_parameter('max_altitude', 50.0)  # meters
        self.declare_parameter('min_altitude', 0.5)   # meters
        self.declare_parameter('max_velocity', 5.0)   # m/s
        self.declare_parameter('min_battery_voltage', 14.0)  # volts
        self.declare_parameter('max_distance_from_home', 100.0)  # meters
        self.declare_parameter('obstacle_danger_distance', 1.0)  # meters
        self.declare_parameter('communication_timeout', 2.0)  # seconds

        # Get parameters
        self.max_altitude = self.get_parameter('max_altitude').get_parameter_value().double_value
        self.min_altitude = self.get_parameter('min_altitude').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.min_battery_voltage = self.get_parameter('min_battery_voltage').get_parameter_value().double_value
        self.max_distance_from_home = self.get_parameter('max_distance_from_home').get_parameter_value().double_value
        self.obstacle_danger_distance = self.get_parameter('obstacle_danger_distance').get_parameter_value().double_value
        self.communication_timeout = self.get_parameter('communication_timeout').get_parameter_value().double_value

        # State variables
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.home_position = None
        self.battery_voltage = 16.0  # Default safe voltage
        self.obstacle_distances = np.full(16, 10.0)  # Default safe distances
        self.safety_override_active = False
        self.emergency_landing = False

        # Timing
        self.last_observation_time = time.time()
        self.last_action_time = time.time()
        self.last_battery_time = time.time()

        # Safety flags
        self.safety_violations = {
            'altitude_high': False,
            'altitude_low': False,
            'velocity_high': False,
            'battery_low': False,
            'distance_far': False,
            'obstacle_close': False,
            'communication_lost': False
        }

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/ai/observation', self.observation_callback, reliable_qos)
        self.action_sub = self.create_subscription(
            Twist, '/ai/action', self.action_callback, reliable_qos)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.pose_callback, reliable_qos)
        self.velocity_sub = self.create_subscription(
            Twist, '/drone/velocity', self.velocity_callback, reliable_qos)

        # Publishers
        self.override_pub = self.create_publisher(
            Bool, '/safety/override', reliable_qos)
        self.status_pub = self.create_publisher(
            String, '/safety/status', reliable_qos)
        self.emergency_land_pub = self.create_publisher(
            Bool, '/safety/emergency_land', reliable_qos)
        self.failsafe_action_pub = self.create_publisher(
            Twist, '/failsafe/action', reliable_qos)

        # Timer for safety monitoring
        self.monitor_timer = self.create_timer(0.1, self.monitor_safety)  # 10 Hz

        self.get_logger().info('Safety Monitor Node initialized')

    def observation_callback(self, msg: Float32MultiArray):
        """Process observation data for safety monitoring"""
        self.last_observation_time = time.time()

        if len(msg.data) >= 131:
            # Extract position from observation (first 3 elements)
            self.current_position = np.array(msg.data[:3])

            # Extract LiDAR distances (elements 112-128)
            if len(msg.data) >= 128:
                self.obstacle_distances = np.array(msg.data[112:128])

            # Set home position on first observation
            if self.home_position is None:
                self.home_position = self.current_position.copy()
                self.get_logger().info(f'Home position set: {self.home_position}')

    def action_callback(self, msg: Twist):
        """Monitor AI action commands"""
        self.last_action_time = time.time()

    def battery_callback(self, msg: BatteryState):
        """Monitor battery state"""
        self.battery_voltage = msg.voltage
        self.last_battery_time = time.time()

    def pose_callback(self, msg: PoseStamped):
        """Monitor drone position"""
        self.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def velocity_callback(self, msg: Twist):
        """Monitor drone velocity"""
        self.current_velocity = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z
        ])

    def monitor_safety(self):
        """Main safety monitoring function"""
        current_time = time.time()

        # Check communication timeouts
        obs_timeout = (current_time - self.last_observation_time) > self.communication_timeout
        action_timeout = (current_time - self.last_action_time) > self.communication_timeout
        battery_timeout = (current_time - self.last_battery_time) > (self.communication_timeout * 5)

        self.safety_violations['communication_lost'] = obs_timeout or action_timeout

        # Check altitude limits
        altitude = self.current_position[2]
        self.safety_violations['altitude_high'] = altitude > self.max_altitude
        self.safety_violations['altitude_low'] = altitude < self.min_altitude

        # Check velocity limits
        velocity_magnitude = np.linalg.norm(self.current_velocity)
        self.safety_violations['velocity_high'] = velocity_magnitude > self.max_velocity

        # Check battery level
        if not battery_timeout:
            self.safety_violations['battery_low'] = self.battery_voltage < self.min_battery_voltage

        # Check distance from home
        if self.home_position is not None:
            distance_from_home = np.linalg.norm(self.current_position[:2] - self.home_position[:2])
            self.safety_violations['distance_far'] = distance_from_home > self.max_distance_from_home

        # Check obstacle proximity
        min_obstacle_distance = np.min(self.obstacle_distances)
        self.safety_violations['obstacle_close'] = min_obstacle_distance < self.obstacle_danger_distance

        # Determine if safety override is needed
        any_violation = any(self.safety_violations.values())

        if any_violation and not self.safety_override_active:
            self.activate_safety_override()
        elif not any_violation and self.safety_override_active:
            self.deactivate_safety_override()

        # Handle critical situations
        critical_violations = [
            'altitude_low',
            'battery_low',
            'obstacle_close'
        ]

        if any(self.safety_violations[v] for v in critical_violations):
            self.handle_emergency()

        # Publish safety status
        self.publish_safety_status()

    def activate_safety_override(self):
        """Activate safety override"""
        self.safety_override_active = True

        # Publish override signal
        override_msg = Bool()
        override_msg.data = True
        self.override_pub.publish(override_msg)

        # Generate failsafe action
        self.generate_failsafe_action()

        violations = [k for k, v in self.safety_violations.items() if v]
        self.get_logger().warn(f'Safety override activated. Violations: {violations}')

    def deactivate_safety_override(self):
        """Deactivate safety override"""
        self.safety_override_active = False

        # Publish override signal
        override_msg = Bool()
        override_msg.data = False
        self.override_pub.publish(override_msg)

        self.get_logger().info('Safety override deactivated')

    def handle_emergency(self):
        """Handle emergency situations"""
        if not self.emergency_landing:
            self.emergency_landing = True

            # Publish emergency landing command
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_land_pub.publish(emergency_msg)

            self.get_logger().error('EMERGENCY: Initiating emergency landing procedure')

    def generate_failsafe_action(self):
        """Generate failsafe control action"""
        failsafe_action = Twist()

        # Default: hover in place
        failsafe_action.linear.x = 0.0
        failsafe_action.linear.y = 0.0
        failsafe_action.linear.z = 0.0
        failsafe_action.angular.z = 0.0

        # Handle specific violations
        if self.safety_violations['altitude_high']:
            # Descend slowly
            failsafe_action.linear.z = -0.5

        elif self.safety_violations['altitude_low']:
            # Ascend slowly
            failsafe_action.linear.z = 0.5

        elif self.safety_violations['obstacle_close']:
            # Move away from closest obstacle
            min_idx = np.argmin(self.obstacle_distances)

            # Simple obstacle avoidance based on direction
            if min_idx < 8:  # Horizontal obstacles
                angle = min_idx * np.pi / 4  # 0, 45, 90, ... degrees
                # Move in opposite direction
                failsafe_action.linear.x = -0.5 * np.cos(angle)
                failsafe_action.linear.y = -0.5 * np.sin(angle)
            else:
                # Vertical obstacles - hover
                pass

        elif self.safety_violations['distance_far']:
            # Return to home slowly
            if self.home_position is not None:
                direction = self.home_position[:2] - self.current_position[:2]
                distance = np.linalg.norm(direction)
                if distance > 0.1:
                    direction = direction / distance
                    failsafe_action.linear.x = direction[0] * 1.0
                    failsafe_action.linear.y = direction[1] * 1.0

        # Publish failsafe action
        self.failsafe_action_pub.publish(failsafe_action)

    def publish_safety_status(self):
        """Publish safety status message"""
        status_parts = []

        if self.safety_override_active:
            status_parts.append("OVERRIDE_ACTIVE")

        if self.emergency_landing:
            status_parts.append("EMERGENCY_LANDING")

        active_violations = [k for k, v in self.safety_violations.items() if v]
        if active_violations:
            status_parts.append(f"VIOLATIONS:{','.join(active_violations)}")

        # Add numerical status
        status_parts.extend([
            f"ALT:{self.current_position[2]:.1f}",
            f"VEL:{np.linalg.norm(self.current_velocity):.1f}",
            f"BAT:{self.battery_voltage:.1f}V",
            f"OBS:{np.min(self.obstacle_distances):.1f}m"
        ])

        status_str = " | ".join(status_parts) if status_parts else "NORMAL"

        status_msg = String()
        status_msg.data = status_str
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()