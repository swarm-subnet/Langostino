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
from sensor_msgs.msg import BatteryState, NavSatFix


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
        self.declare_parameter('max_altitude', 10.0)  # meters
        self.declare_parameter('min_altitude', 0.0)   # meters
        self.declare_parameter('max_velocity', 0.1)   # m/s
        self.declare_parameter('min_battery_voltage', 14.0)  # volts
        self.declare_parameter('default_safe_battery_voltage', 16.0)  # volts
        self.declare_parameter('battery_timeout_multiplier', 5)  # multiplier for battery timeout
        self.declare_parameter('max_distance_from_home', 100.0)  # meters
        self.declare_parameter('obstacle_danger_distance', 1.0)  # meters
        self.declare_parameter('default_safe_lidar_distance', 10.0)  # meters
        self.declare_parameter('max_ray_distance', 20.0)  # meters - MUST MATCH ai_adapter_node
        self.declare_parameter('communication_timeout', 2.0)  # seconds
        self.declare_parameter('monitor_rate', 10.0)  # Hz
        self.declare_parameter('qos_depth', 1)  # QoS depth for topics

        # Get parameters
        self.max_altitude = self.get_parameter('max_altitude').get_parameter_value().double_value
        self.min_altitude = self.get_parameter('min_altitude').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.min_battery_voltage = self.get_parameter('min_battery_voltage').get_parameter_value().double_value
        self.default_safe_battery_voltage = self.get_parameter('default_safe_battery_voltage').get_parameter_value().double_value
        self.battery_timeout_multiplier = self.get_parameter('battery_timeout_multiplier').get_parameter_value().integer_value
        self.max_distance_from_home = self.get_parameter('max_distance_from_home').get_parameter_value().double_value
        self.obstacle_danger_distance = self.get_parameter('obstacle_danger_distance').get_parameter_value().double_value
        self.default_safe_lidar_distance = self.get_parameter('default_safe_lidar_distance').get_parameter_value().double_value
        self.max_ray_distance = self.get_parameter('max_ray_distance').get_parameter_value().double_value
        self.communication_timeout = self.get_parameter('communication_timeout').get_parameter_value().double_value
        self.monitor_rate = self.get_parameter('monitor_rate').get_parameter_value().double_value
        self.qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

        # State variables
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.home_position = None  # Home position in ENU coordinates [E, N, U]
        self.home_position_geodetic = None  # Home position in geodetic [lat, lon, alt]
        self.battery_voltage = self.default_safe_battery_voltage  # Default safe voltage (from params)
        self.obstacle_distances = np.full(16, self.default_safe_lidar_distance)  # Default safe distances (from params)
        self.safety_override_active = False
        self.rth_active = False
        self.custom_rth_active = False  # Custom RTH using goal waypoint

        # Waypoint tracking
        self.current_waypoint_number = None
        self.waypoints_completed = False
        self.last_waypoint_time = time.time()

        # Landing state
        self.landing_initiated = False
        self.landing_altitude_target = 0.0  # Target altitude for landing (ground level)
        self.landing_complete = False
        self.lidar_landing_threshold = 0.3  # LiDAR distance threshold for landing completion (meters)
        self.lidar_transition_altitude = 5.0  # Altitude to switch from GPS to LiDAR (meters)

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
            depth=self.qos_depth
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.qos_depth
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
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fc/gps_fix', self.gps_callback, sensor_qos)
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, '/fc/waypoint', self.waypoint_callback, reliable_qos)

        # Publishers
        self.override_pub = self.create_publisher(
            Bool, '/safety/override', reliable_qos)
        self.rth_command_pub = self.create_publisher(
            Bool, '/safety/rth_command', reliable_qos)
        self.custom_goal_pub = self.create_publisher(
            Float32MultiArray, '/safety/custom_goal_geodetic', reliable_qos)
        self.status_pub = self.create_publisher(
            String, '/safety/status', reliable_qos)

        # Timer for safety monitoring
        monitor_period = 1.0 / self.monitor_rate
        self.monitor_timer = self.create_timer(monitor_period, self.monitor_safety)

        self.get_logger().info(f'Safety Monitor Node initialized (monitoring at {self.monitor_rate} Hz)')

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
            # NOTE: The coordinate system is set up so that the origin is at the takeoff point.
            # The drone starts at approximately [0, 0, 3] in ENU coordinates (3m altitude).
            # For custom RTH, we want to return to ground level at the origin, which is [0, 0, 0].
            if self.home_position is None:
                # Set home to [0, 0, 0] - the origin at ground level
                self.home_position = np.array([0.0, 0.0, 0.0])
                self.get_logger().info(
                    f'Home position set to origin: {self.home_position}\n'
                    f'   (Current position at startup: {self.current_position})'
                )

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

    def gps_callback(self, msg: NavSatFix):
        """
        Monitor GPS position and save home position in geodetic coordinates.

        IMPORTANT: The coordinate system is configured so that the initial GPS position
        maps to ENU coordinates [0, 0, 3] (3 meters altitude). For custom RTH to ground
        level [0, 0, 0], we need to store the geodetic coordinates that correspond to
        3 meters lower altitude than the initial GPS position.
        """
        # Save home position in geodetic coordinates on first GPS fix
        if self.home_position_geodetic is None:
            # Subtract 3 meters from altitude to get ground level coordinates
            # This is because the drone starts at [0, 0, 3] in ENU, and we want
            # to return to [0, 0, 0] which is 3 meters lower
            home_altitude_ground_level = msg.altitude - 3.0

            self.home_position_geodetic = np.array([
                msg.latitude,
                msg.longitude,
                home_altitude_ground_level
            ])

            self.get_logger().info(
                f'Home position (geodetic) set:\n'
                f'   Latitude: {msg.latitude:.7f}\n'
                f'   Longitude: {msg.longitude:.7f}\n'
                f'   Initial GPS altitude: {msg.altitude:.2f}m\n'
                f'   Ground level altitude (RTH target): {home_altitude_ground_level:.2f}m'
            )

    def waypoint_callback(self, msg: Float32MultiArray):
        """
        Monitor current waypoint from INAV mission.
        Format: [wp_no, lat, lon, alt, heading, stay_time, navflag]

        Detects when waypoints are completed:
        - Waypoint #0 indicates no active waypoint
        - Mission completion should trigger custom RTH
        """
        if len(msg.data) < 4:
            return

        wp_number = int(msg.data[0])
        self.last_waypoint_time = time.time()

        # Track waypoint changes
        if self.current_waypoint_number != wp_number:
            prev_wp = self.current_waypoint_number
            self.current_waypoint_number = wp_number

            # Waypoint #0 typically means no active waypoint
            if wp_number == 0 and prev_wp is not None and prev_wp > 0:
                self.waypoints_completed = True
                self.get_logger().info('🎯 Waypoint mission completed - activating custom RTH')
            elif wp_number > 0:
                self.waypoints_completed = False
                self.get_logger().info(f'📍 Current waypoint: #{wp_number}')

    def monitor_safety(self):
        """Main safety monitoring function"""
        current_time = time.time()

        # Check communication timeouts
        obs_timeout = (current_time - self.last_observation_time) > self.communication_timeout
        action_timeout = (current_time - self.last_action_time) > self.communication_timeout
        battery_timeout = (current_time - self.last_battery_time) > (self.communication_timeout * self.battery_timeout_multiplier)

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

        # Check if custom RTH should be activated
        # Conditions: critical violations OR waypoints completed
        should_activate_custom_rth = (
            any(self.safety_violations[v] for v in critical_violations) or
            self.waypoints_completed
        )

        if should_activate_custom_rth:
            self.activate_custom_rth()
        elif not any_violation and not self.waypoints_completed and self.custom_rth_active:
            self.deactivate_custom_rth()

        # Keep old RTH functionality for backward compatibility (can be disabled if needed)
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

    def activate_custom_rth(self):
        """
        Activate Custom Return to Home by publishing home position as goal waypoint.

        Instead of using INAV's built-in RTH (CH9), this publishes the home position
        as a custom goal point that the AI will navigate towards.
        """
        if not self.custom_rth_active:
            self.custom_rth_active = True

            # Check if we have home position saved
            if self.home_position_geodetic is None:
                self.get_logger().error('🚨 CUSTOM RTH: Home position not set! Cannot activate custom RTH.')
                return

            if self.home_position is None:
                self.get_logger().error('🚨 CUSTOM RTH: Home position (ENU) not set! Cannot activate custom RTH.')
                return

            # Publish home position as custom goal in geodetic coordinates
            # Format: [wp_no, lat, lon, alt] (same as /fc/waypoint)
            goal_msg = Float32MultiArray()

            # For landing, we want to go to home at ground level (altitude 0 in ENU)
            # The home_position_geodetic has the original altitude, but we want to land
            # So we can either:
            # 1. Keep the same altitude initially, then descend
            # 2. Set altitude to ground level immediately
            # Let's start by going to home at the same altitude, then initiate landing
            goal_msg.data = [
                0.0,  # waypoint number (0 = RTH/home)
                float(self.home_position_geodetic[0]),  # latitude
                float(self.home_position_geodetic[1]),  # longitude
                float(self.home_position_geodetic[2])   # altitude (original home altitude)
            ]
            self.custom_goal_pub.publish(goal_msg)

            violations = [k for k, v in self.safety_violations.items() if v]
            reason = f"violations: {violations}" if violations else "waypoints completed"
            self.get_logger().error(
                f'🚨 CUSTOM RTH ACTIVATED - Publishing home goal waypoint!\n'
                f'   Reason: {reason}\n'
                f'   Home (geodetic): lat={self.home_position_geodetic[0]:.7f}, '
                f'lon={self.home_position_geodetic[1]:.7f}, alt={self.home_position_geodetic[2]:.2f}m\n'
                f'   Home (ENU): E={self.home_position[0]:.2f}, N={self.home_position[1]:.2f}, U={self.home_position[2]:.2f}m\n'
                f'   Current (ENU): E={self.current_position[0]:.2f}, N={self.current_position[1]:.2f}, U={self.current_position[2]:.2f}m'
            )

        # Check if drone is close to home and initiate landing
        if self.home_position is not None and not self.landing_complete:
            distance_to_home = np.linalg.norm(self.current_position[:2] - self.home_position[:2])

            # Get current altitude using appropriate sensor
            # Denormalize LiDAR distance to actual meters
            lidar_altitude = self.obstacle_distances[1] * self.max_ray_distance
            gps_altitude = self.current_position[2]

            # Determine which altitude reference to use
            if lidar_altitude < self.lidar_transition_altitude:
                # Use LiDAR when below transition altitude and reading is valid
                current_altitude = lidar_altitude
                altitude_reference = "LIDAR"
            else:
                # Use GPS for higher altitudes
                current_altitude = gps_altitude
                altitude_reference = "GPS"

            # Phase 1: If within 2 meters horizontally and at reasonable altitude, initiate landing
            if distance_to_home < 2.0 and gps_altitude < 5.0 and not self.landing_initiated:
                self.initiate_landing()

            # Phase 2: Check if landing is complete using LiDAR
            if self.landing_initiated:
                # Landing complete when LiDAR shows we're very close to ground
                if altitude_reference == "LIDAR" and current_altitude < self.lidar_landing_threshold:
                    self.landing_complete = True
                    self.get_logger().info(
                        f'✅ LANDING COMPLETE!\n'
                        f'   LiDAR altitude: {current_altitude:.2f}m\n'
                        f'   Position: E={self.current_position[0]:.2f}, N={self.current_position[1]:.2f}'
                    )

        # Continue publishing goal to ensure AI navigates towards home
        if self.custom_rth_active and self.home_position_geodetic is not None and not self.landing_complete:
            goal_msg = Float32MultiArray()

            # If landing initiated, publish descending altitude
            if self.landing_initiated:
                # Get LiDAR altitude for landing guidance
                lidar_altitude = self.obstacle_distances[1] * self.max_ray_distance

                # Two-phase landing:
                # Phase 1 (GPS): Descend to ~3m using GPS
                # Phase 2 (LiDAR): Final descent using LiDAR when < 5m
                if lidar_altitude < self.lidar_transition_altitude:
                    # Phase 2: LiDAR-guided final landing
                    # Target 0.5m above ground (will complete at 0.3m threshold)
                    target_altitude_geodetic = self.home_position_geodetic[2] + 0.5
                    landing_phase = "LIDAR-GUIDED"
                else:
                    # Phase 1: GPS-guided descent to transition altitude
                    target_altitude_geodetic = self.home_position_geodetic[2] + 3.0
                    landing_phase = "GPS-GUIDED"

                goal_msg.data = [
                    0.0,  # waypoint number (0 = RTH/home)
                    float(self.home_position_geodetic[0]),  # latitude
                    float(self.home_position_geodetic[1]),  # longitude
                    float(target_altitude_geodetic)         # descending altitude
                ]

                # Log landing progress
                self.get_logger().info(
                    f'🛬 Landing in progress [{landing_phase}]: '
                    f'LiDAR={lidar_altitude:.2f}m, GPS_alt={self.current_position[2]:.2f}m, '
                    f'Target={target_altitude_geodetic:.2f}m',
                    throttle_duration_sec=2.0
                )
            else:
                # Navigate to home at original altitude
                goal_msg.data = [
                    0.0,  # waypoint number (0 = RTH/home)
                    float(self.home_position_geodetic[0]),
                    float(self.home_position_geodetic[1]),
                    float(self.home_position_geodetic[2])
                ]

            self.custom_goal_pub.publish(goal_msg)

    def deactivate_custom_rth(self):
        """Deactivate Custom Return to Home"""
        if self.custom_rth_active:
            self.custom_rth_active = False
            self.landing_initiated = False
            self.landing_complete = False
            self.get_logger().info('✓ Custom RTH deactivated - resuming normal operation')

    def initiate_landing(self):
        """
        Initiate two-phase landing sequence:
        Phase 1 (GPS): Descend to ~3m using GPS guidance
        Phase 2 (LiDAR): Final descent using LiDAR when < 5m, complete at < 0.3m

        This approach ensures accurate landing by using LiDAR for the final approach.
        """
        if not self.landing_initiated:
            self.landing_initiated = True
            self.landing_altitude_target = 0.0  # Target ground level

            lidar_altitude = self.obstacle_distances[1] * self.max_ray_distance
            gps_altitude = self.current_position[2]

            self.get_logger().warn(
                f'🛬 LANDING INITIATED - Two-Phase Landing Sequence\n'
                f'   Current GPS altitude: {gps_altitude:.2f}m\n'
                f'   Current LiDAR altitude: {lidar_altitude:.2f}m\n'
                f'   Phase 1: GPS-guided descent to ~3m\n'
                f'   Phase 2: LiDAR-guided final landing (< {self.lidar_landing_threshold:.1f}m)\n'
                f'   Home position: E={self.home_position[0]:.2f}, N={self.home_position[1]:.2f}'
            )

    def publish_safety_status(self):
        """Publish safety status message"""
        status_parts = []

        if self.custom_rth_active:
            if self.landing_complete:
                status_parts.append("✅ LANDING_COMPLETE")
            elif self.landing_initiated:
                # Determine landing phase
                lidar_altitude = self.obstacle_distances[1] * self.max_ray_distance
                if lidar_altitude < self.lidar_transition_altitude:
                    status_parts.append("🛬 LANDING_LIDAR")
                else:
                    status_parts.append("🛬 LANDING_GPS")
            else:
                status_parts.append("🚨 CUSTOM_RTH_ACTIVE")

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
        lidar_down_altitude = actual_obstacle_distances[1]  # Ray 1 is down-facing

        status_parts.extend([
            f"GPS_ALT:{self.current_position[2]:.1f}m",
            f"LIDAR_ALT:{lidar_down_altitude:.1f}m",
            f"VEL:{np.linalg.norm(self.current_velocity):.1f}m/s",
            f"BAT:{self.battery_voltage:.1f}V",
            f"OBS:{np.min(actual_obstacle_distances):.1f}m"
        ])

        if self.home_position is not None:
            distance_from_home = np.linalg.norm(self.current_position[:2] - self.home_position[:2])
            status_parts.append(f"HOME_DIST:{distance_from_home:.1f}m")

        # Add waypoint status
        if self.current_waypoint_number is not None:
            if self.waypoints_completed:
                status_parts.append("WP:COMPLETED")
            else:
                status_parts.append(f"WP:#{self.current_waypoint_number}")

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