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


class SafetyMonitorNode(Node):
    """
    Monitors drone safety and triggers failsafe procedures when necessary.

    Subscribers:
        /ai/observation (std_msgs/Float32MultiArray): 131-D observation data [pos, euler, vel, ang_vel, actions, lidar, goal]
        /ai/observation_debug (std_msgs/Float32MultiArray): Debug data [E, N, U, yaw, down_lidar, used_action_flag]
        /ai/action (std_msgs/Float32MultiArray): AI velocity commands [vx, vy, vz, speed]
        /fc/battery (sensor_msgs/BatteryState): Battery status from flight controller

    Publishers:
        /safety/override (std_msgs/Bool): Safety override signal (hover mode)
        /safety/rth_command (std_msgs/Bool): RTH (Return to Home) activation command
        /safety/status (std_msgs/String): Safety status messages
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
        self.declare_parameter('max_ray_distance', 20.0)  # meters - MUST MATCH ai_adapter_node
        self.declare_parameter('communication_timeout', 2.0)  # seconds

        # Get parameters
        self.max_altitude = self.get_parameter('max_altitude').get_parameter_value().double_value
        self.min_altitude = self.get_parameter('min_altitude').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.min_battery_voltage = self.get_parameter('min_battery_voltage').get_parameter_value().double_value
        self.max_distance_from_home = self.get_parameter('max_distance_from_home').get_parameter_value().double_value
        self.obstacle_danger_distance = self.get_parameter('obstacle_danger_distance').get_parameter_value().double_value
        self.max_ray_distance = self.get_parameter('max_ray_distance').get_parameter_value().double_value
        self.communication_timeout = self.get_parameter('communication_timeout').get_parameter_value().double_value

        # State variables
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.home_position = None
        self.battery_voltage = 16.0  # Default safe voltage
        self.obstacle_distances = np.full(16, 10.0)  # Default safe distances
        self.safety_override_active = False
        self.rth_active = False

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
        self.obs_debug_sub = self.create_subscription(
            Float32MultiArray, '/ai/observation_debug', self.observation_debug_callback, reliable_qos)
        self.action_sub = self.create_subscription(
            Float32MultiArray, '/ai/action', self.action_callback, reliable_qos)
        self.battery_sub = self.create_subscription(
            BatteryState, '/fc/battery', self.battery_callback, sensor_qos)

        # Publishers
        self.override_pub = self.create_publisher(
            Bool, '/safety/override', reliable_qos)
        self.rth_command_pub = self.create_publisher(
            Bool, '/safety/rth_command', reliable_qos)
        self.status_pub = self.create_publisher(
            String, '/safety/status', reliable_qos)

        # Timer for safety monitoring
        self.monitor_timer = self.create_timer(0.1, self.monitor_safety)  # 10 Hz

        self.get_logger().info('Safety Monitor Node initialized')

    def observation_callback(self, msg: Float32MultiArray):
        """
        Process 131-D observation data for safety monitoring.
        Observation layout: [0:3] pos, [3:6] euler, [6:9] vel, [9:12] ang_vel,
                           [12:112] actions (25×4), [112:128] lidar (16 rays), [128:131] goal
        """
        self.last_observation_time = time.time()

        if len(msg.data) >= 131:
            # Extract position from observation (first 3 elements: E, N, U)
            self.current_position = np.array(msg.data[:3])

            # Extract velocity (elements 6-9: vx_east, vy_north, vz_up)
            self.current_velocity = np.array(msg.data[6:9])

            # Extract LiDAR distances (elements 112-128: 16 rays, normalized 0-1)
            self.obstacle_distances = np.array(msg.data[112:128])

            # Set home position on first observation
            if self.home_position is None:
                self.home_position = self.current_position.copy()
                self.get_logger().info(f'Home position set: {self.home_position}')

    def observation_debug_callback(self, msg: Float32MultiArray):
        """
        Process debug observation data [E, N, U, yaw, down_lidar, used_action_flag].
        Provides cleaner access to position and down lidar.
        """
        if len(msg.data) >= 6:
            # Update position from debug (more direct than full observation)
            self.current_position = np.array(msg.data[:3])

    def action_callback(self, msg: Float32MultiArray):
        """
        Monitor AI action commands [vx, vy, vz, speed].
        Actions are in ENU frame (vx=East, vy=North, vz=Up).
        """
        self.last_action_time = time.time()

    def battery_callback(self, msg: BatteryState):
        """Monitor battery state"""
        self.battery_voltage = msg.voltage
        self.last_battery_time = time.time()

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
        # Lidar values are normalized (0-1), denormalize to actual meters
        actual_obstacle_distances = self.obstacle_distances * self.max_ray_distance
        min_obstacle_distance = np.min(actual_obstacle_distances)
        self.safety_violations['obstacle_close'] = min_obstacle_distance < self.obstacle_danger_distance

        # Determine if safety override is needed
        any_violation = any(self.safety_violations.values())

        if any_violation and not self.safety_override_active:
            self.activate_safety_override()
        elif not any_violation and self.safety_override_active:
            self.deactivate_safety_override()

        # Handle critical situations requiring RTH
        critical_violations = [
            'battery_low',
            'distance_far',
            'communication_lost'
        ]

        if any(self.safety_violations[v] for v in critical_violations):
            self.activate_rth()
        elif not any_violation and self.rth_active:
            self.deactivate_rth()

        # Publish safety status
        self.publish_safety_status()

    def activate_safety_override(self):
        """Activate safety override (hover mode)"""
        self.safety_override_active = True

        # Publish override signal
        override_msg = Bool()
        override_msg.data = True
        self.override_pub.publish(override_msg)

        violations = [k for k, v in self.safety_violations.items() if v]
        self.get_logger().warn(f'⚠️  Safety override activated - HOVER MODE. Violations: {violations}')

    def deactivate_safety_override(self):
        """Deactivate safety override"""
        self.safety_override_active = False

        # Publish override signal
        override_msg = Bool()
        override_msg.data = False
        self.override_pub.publish(override_msg)

        self.get_logger().info('✓ Safety override deactivated')

    def activate_rth(self):
        """Activate Return to Home via INAV NAV RTH (CH9)"""
        if not self.rth_active:
            self.rth_active = True

            # Publish RTH command (will set CH9 = 1800)
            rth_msg = Bool()
            rth_msg.data = True
            self.rth_command_pub.publish(rth_msg)

            violations = [k for k, v in self.safety_violations.items() if v]
            self.get_logger().error(f'🚨 RTH ACTIVATED - INAV Return to Home engaged! Violations: {violations}')

    def deactivate_rth(self):
        """Deactivate Return to Home"""
        if self.rth_active:
            self.rth_active = False

            # Publish RTH deactivation (will set CH9 = 1000)
            rth_msg = Bool()
            rth_msg.data = False
            self.rth_command_pub.publish(rth_msg)

            self.get_logger().info('✓ RTH deactivated - resuming normal operation')

    def publish_safety_status(self):
        """Publish safety status message"""
        status_parts = []

        if self.rth_active:
            status_parts.append("🚨 RTH_ACTIVE")

        if self.safety_override_active:
            status_parts.append("⚠️  HOVER_OVERRIDE")

        active_violations = [k for k, v in self.safety_violations.items() if v]
        if active_violations:
            status_parts.append(f"VIOLATIONS:{','.join(active_violations)}")

        # Add numerical status
        # Denormalize lidar for display
        actual_obstacle_distances = self.obstacle_distances * self.max_ray_distance
        status_parts.extend([
            f"ALT:{self.current_position[2]:.1f}m",
            f"VEL:{np.linalg.norm(self.current_velocity):.1f}m/s",
            f"BAT:{self.battery_voltage:.1f}V",
            f"OBS:{np.min(actual_obstacle_distances):.1f}m"
        ])

        if self.home_position is not None:
            distance_from_home = np.linalg.norm(self.current_position[:2] - self.home_position[:2])
            status_parts.append(f"HOME_DIST:{distance_from_home:.1f}m")

        status_str = " | ".join(status_parts) if status_parts else "✓ NORMAL"

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