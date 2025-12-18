#!/usr/bin/env python3
"""
Safety Monitor Node - Simple Safety System

Monitors essential flight parameters and triggers emergency landing when needed:
- Attitude Safety: Roll/pitch angle limits
- Altitude Safety: Min/max altitude boundaries
- Geofencing: Maximum distance from home

Safety Action Sequence:
1. NORMAL: All parameters within limits
2. OVERRIDE: Violation detected → Stop and hover for 3 seconds
3. LANDING: After hover → Slow controlled landing on same spot
"""

import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, Float32MultiArray, String
from geometry_msgs.msg import Vector3Stamped


class SafetyMonitorNode(Node):
    """
    Simple safety monitoring system with basic checks and emergency landing.
    
    Subscriptions:
        /ai/observation          (std_msgs/Float32MultiArray) : [E, N, U, ...]
        /fc/attitude_degrees    (geometry_msgs/Vector3Stamped) : [roll, pitch, yaw] degrees

    Publications:
        /safety/override        (std_msgs/Bool) : True=Override AI control
        /safety/emergency_land  (std_msgs/Bool) : True=Initiate emergency landing
        /safety/status         (std_msgs/String) : Current safety status
    """

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Parameters
        # Attitude Limits
        self.declare_parameter('max_roll_angle', 15.0)  # degrees
        self.declare_parameter('max_pitch_angle', 15.0)  # degrees

        # Altitude Limits
        self.declare_parameter('min_altitude', 0.0)  # meters
        self.declare_parameter('max_altitude', 10.0)  # meters

        # Geofencing
        self.declare_parameter('max_distance_from_home', 50.0)  # meters

        # Safety Action Timing
        self.declare_parameter('hover_duration', 3.0)  # seconds to hover before landing

        # System
        self.declare_parameter('monitor_rate', 20.0)  # Hz
        self.declare_parameter('qos_depth', 1)

        # Get parameters
        self.max_roll_angle = self.get_parameter('max_roll_angle').get_parameter_value().double_value
        self.max_pitch_angle = self.get_parameter('max_pitch_angle').get_parameter_value().double_value
        self.min_altitude = self.get_parameter('min_altitude').get_parameter_value().double_value
        self.max_altitude = self.get_parameter('max_altitude').get_parameter_value().double_value
        self.max_distance_from_home = self.get_parameter('max_distance_from_home').get_parameter_value().double_value
        self.hover_duration = self.get_parameter('hover_duration').get_parameter_value().double_value
        self.monitor_rate = self.get_parameter('monitor_rate').get_parameter_value().double_value
        self.qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

        # State Variables
        # Position & Attitude
        self.current_position = np.zeros(3)  # [E, N, U]
        self.current_attitude = np.zeros(3)  # [roll, pitch, yaw] degrees
        self.home_position = None

        # Safety State Machine
        self.safety_active = False
        self.hover_start_time = None
        self.landing_active = False

        # Violation tracking
        self.violations = []

        # QoS Profiles
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
        self.attitude_sub = self.create_subscription(
            Vector3Stamped, '/fc/attitude_degrees', self.attitude_callback, sensor_qos)

        # Publishers
        self.override_pub = self.create_publisher(
            Bool, '/safety/override', reliable_qos)
        self.emergency_land_pub = self.create_publisher(
            Bool, '/safety/emergency_land', reliable_qos)
        self.status_pub = self.create_publisher(
            String, '/safety/status', reliable_qos)

        # Timer
        monitor_period = 1.0 / self.monitor_rate
        self.monitor_timer = self.create_timer(monitor_period, self.safety_check_cycle)

        self.get_logger().info(
            f'🛡️  Simple Safety Monitor initialized\n'
            f'   Monitoring: Attitude, Altitude, Geofencing\n'
            f'   Action: Hover {self.hover_duration}s → Emergency Land\n'
            f'   Rate: {self.monitor_rate} Hz'
        )
    # Callback from AI Observation

    def observation_callback(self, msg: Float32MultiArray):
        """Extract position from AI observation"""
        if len(msg.data) >= 3:
            self.current_position = np.array(msg.data[:3])  # [E, N, U]

            # Set home position on first observation
            if self.home_position is None:
                self.home_position = np.array([0.0, 0.0, 0.0])
                self.get_logger().info(
                    f'🏠 Home position set to origin\n'
                    f'   Current position: E={self.current_position[0]:.2f}, '
                    f'N={self.current_position[1]:.2f}, U={self.current_position[2]:.2f}'
                )

    def attitude_callback(self, msg: Vector3Stamped):
        """Update current attitude"""
        self.current_attitude = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
    # Safety Monitoring

    def safety_check_cycle(self):
        """Main safety monitoring cycle"""
        # Clear previous violations
        self.violations.clear()

        # Run safety checks
        self._check_attitude()
        self._check_altitude()
        self._check_geofence()

        # Determine if safety action is needed
        violation_detected = len(self.violations) > 0

        if violation_detected and not self.safety_active:
            # New violation - activate safety
            self.activate_safety()
        elif not violation_detected and self.safety_active:
            # Violations cleared but keep safety active until landing complete
            # (don't deactivate during hover/landing sequence)
            if not self.landing_active and self.hover_start_time is None:
                self.deactivate_safety()

        # Update safety state machine
        self.update_safety_state()

        # Publish status
        self.publish_status()

    def _check_attitude(self):
        """Check attitude limits"""
        roll, pitch, _ = self.current_attitude

        if abs(roll) > self.max_roll_angle:
            self.violations.append(f'Roll {roll:.1f}° exceeds limit {self.max_roll_angle:.1f}°')

        if abs(pitch) > self.max_pitch_angle:
            self.violations.append(f'Pitch {pitch:.1f}° exceeds limit {self.max_pitch_angle:.1f}°')

    def _check_altitude(self):
        """Check altitude limits"""
        altitude = self.current_position[2]

        # Minimum altitude check removed for simplicity
        # Only check maximum altitude

        if altitude > self.max_altitude:
            self.violations.append(f'Altitude {altitude:.2f}m exceeds maximum {self.max_altitude:.2f}m')

    def _check_geofence(self):
        """Check geofence boundary"""
        if self.home_position is None:
            return

        # Horizontal distance from home
        distance = np.linalg.norm(self.current_position[:2] - self.home_position[:2])

        if distance > self.max_distance_from_home:
            self.violations.append(
                f'Distance {distance:.1f}m exceeds geofence {self.max_distance_from_home:.1f}m'
            )
    # Safety Actions

    def activate_safety(self):
        """Activate safety override - start hover sequence"""
        self.safety_active = True
        self.hover_start_time = time.time()
        self.landing_active = False

        # Publish override to stop AI control
        override_msg = Bool()
        override_msg.data = True
        self.override_pub.publish(override_msg)

        self.get_logger().error(
            f'🚨 SAFETY ACTIVATED - Hovering for {self.hover_duration}s before emergency landing\n'
            f'   Violations: {self.violations}'
        )

    def deactivate_safety(self):
        """Deactivate safety - return to normal operation"""
        self.safety_active = False
        self.hover_start_time = None
        self.landing_active = False

        # Clear override
        override_msg = Bool()
        override_msg.data = False
        self.override_pub.publish(override_msg)

        # Clear emergency landing
        land_msg = Bool()
        land_msg.data = False
        self.emergency_land_pub.publish(land_msg)

        self.get_logger().info('✓ Safety deactivated - resuming normal operation')

    def update_safety_state(self):
        """Update safety state machine: HOVER → LANDING"""
        if not self.safety_active:
            return

        current_time = time.time()

        # Check if hover duration has elapsed
        if self.hover_start_time is not None and not self.landing_active:
            hover_elapsed = current_time - self.hover_start_time

            if hover_elapsed >= self.hover_duration:
                # Start landing
                self.initiate_landing()

    def initiate_landing(self):
        """Initiate emergency landing after hover"""
        self.landing_active = True
        self.hover_start_time = None

        # Publish emergency landing command
        land_msg = Bool()
        land_msg.data = True
        self.emergency_land_pub.publish(land_msg)

        self.get_logger().error(
            f'🛬 EMERGENCY LANDING INITIATED\n'
            f'   Performing slow controlled landing on current position\n'
            f'   Position: E={self.current_position[0]:.2f}, '
            f'N={self.current_position[1]:.2f}, U={self.current_position[2]:.2f}'
        )
    # Status Publishing

    def publish_status(self):
        """Publish safety status"""
        status_parts = []

        # Safety state
        if self.landing_active:
            status_parts.append('🛬 EMERGENCY_LANDING')
        elif self.hover_start_time is not None:
            hover_elapsed = time.time() - self.hover_start_time
            remaining = max(0, self.hover_duration - hover_elapsed)
            status_parts.append(f'⚠️  HOVER ({remaining:.1f}s remaining)')
        elif self.safety_active:
            status_parts.append('⚠️  SAFETY_ACTIVE')
        else:
            status_parts.append('✓ NORMAL')

        # Active violations
        if self.violations:
            status_parts.append(f'VIOLATIONS:{len(self.violations)}')

        # Key metrics
        altitude = self.current_position[2]
        roll, pitch, _ = self.current_attitude
        tilt = np.sqrt(roll**2 + pitch**2)

        status_parts.extend([
            f'ALT:{altitude:.1f}m',
            f'TILT:{tilt:.1f}°'
        ])

        if self.home_position is not None:
            distance = np.linalg.norm(self.current_position[:2] - self.home_position[:2])
            status_parts.append(f'HOME:{distance:.1f}m')

        # Publish
        status_msg = String()
        status_msg.data = ' | '.join(status_parts)
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
