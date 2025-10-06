#!/usr/bin/env python3
"""
AI Adapter Node (Refactored) - Converts real sensor data to 131-D observation array

This node coordinates between modular components for clean separation of concerns:
- CoordinateTransforms: Geodetic ↔ ENU conversions
- SensorDataManager: Centralized sensor data storage
- ObservationBuilder: 131-D observation assembly
- DebugLogger: Detailed logging and debugging

Architecture:
    Sensors (ROS Topics) → AIAdapterNode → SensorDataManager
                                         ↓
                         CoordinateTransforms ← ObservationBuilder → DebugLogger
                                         ↓
                                    /ai/observation (131-D)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix, Imu, Range
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped

from swarm_ai_integration.utils import (
    CoordinateTransforms,
    SensorDataManager,
    ObservationBuilder,
    DebugLogger
)


class AIAdapterNode(Node):
    """
    AI Adapter Node - Orchestrates observation generation from sensor data.

    This node acts as a coordinator, delegating to modular components:
    - Sensor data management
    - Coordinate transformations
    - Observation building
    - Debug logging
    """

    def __init__(self):
        super().__init__('ai_adapter_node')

        # -------------------------
        # Configuration
        # -------------------------
        self.declare_parameter('telemetry_rate', 30.0)  # Hz
        self.declare_parameter('max_ray_distance', 10.0)  # meters
        self.declare_parameter('action_buffer_size', 20)
        self.declare_parameter('relative_start_enu', [0.0, 0.0, 3.0])  # [E, N, U] meters

        telemetry_rate = self.get_parameter('telemetry_rate').get_parameter_value().double_value
        max_ray_distance = self.get_parameter('max_ray_distance').get_parameter_value().double_value
        action_buffer_size = self.get_parameter('action_buffer_size').get_parameter_value().integer_value
        relative_start_enu = np.array(
            self.get_parameter('relative_start_enu').get_parameter_value().double_array_value,
            dtype=np.float32
        )

        # -------------------------
        # Initialize Modular Components
        # -------------------------
        self.sensor_manager = SensorDataManager(
            num_lidar_rays=16,
            relative_start_enu=relative_start_enu
        )

        self.obs_builder = ObservationBuilder(
            action_buffer_size=action_buffer_size,
            max_ray_distance=max_ray_distance
        )

        self.transforms = CoordinateTransforms()

        self.debug_logger = DebugLogger(
            node=self,
            max_ray_distance=max_ray_distance
        )

        # -------------------------
        # Statistics
        # -------------------------
        self.obs_count = 0

        # -------------------------
        # QoS Profiles
        # -------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -------------------------
        # Subscribers
        # -------------------------
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fc/gps_fix', self.gps_callback, sensor_qos
        )
        self.att_quat_sub = self.create_subscription(
            QuaternionStamped, '/fc/attitude', self.att_quat_callback, sensor_qos
        )
        self.att_euler_sub = self.create_subscription(
            Vector3Stamped, '/fc/attitude_euler', self.att_euler_callback, sensor_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, '/fc/imu_raw', self.imu_callback, sensor_qos
        )
        self.gps_speed_sub = self.create_subscription(
            Float32MultiArray, '/fc/gps_speed_course', self.gps_speed_callback, sensor_qos
        )
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, '/fc/waypoint', self.waypoint_callback, reliable_qos
        )
        self.lidar_sub = self.create_subscription(
            Range, '/lidar_distance', self.lidar_callback, sensor_qos
        )
        self.action_sub = self.create_subscription(
            Float32MultiArray, '/ai/action', self.action_callback, reliable_qos
        )

        # -------------------------
        # Publishers
        # -------------------------
        self.obs_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation', reliable_qos
        )
        self.debug_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation_debug', reliable_qos
        )

        # -------------------------
        # Timer
        # -------------------------
        timer_period = 1.0 / telemetry_rate
        self.timer = self.create_timer(timer_period, self.compute_observation)

        # -------------------------
        # Initialization Banner
        # -------------------------
        self.debug_logger.print_initial_banner(
            telemetry_rate=telemetry_rate,
            action_buffer_size=action_buffer_size,
            max_ray_distance=max_ray_distance,
            relative_start_enu=relative_start_enu
        )
        self.get_logger().info('AI Adapter Node initialized (131-D observation)')

    # ═══════════════════════════════════════════════════════════════════
    # Sensor Callbacks
    # ═══════════════════════════════════════════════════════════════════

    def gps_callback(self, msg: NavSatFix):
        """Handle GPS position updates."""
        first_fix = self.sensor_manager.update_gps_position(
            lat=msg.latitude,
            lon=msg.longitude,
            alt=msg.altitude
        )

        # Set relative origin on first fix
        if first_fix:
            origin = self.transforms.compute_origin_for_initial_position(
                current_lat=msg.latitude,
                current_lon=msg.longitude,
                current_alt=msg.altitude,
                desired_enu=self.sensor_manager.get_relative_start_enu()
            )
            self.sensor_manager.set_origin_geodetic(origin)

            # Verify initial position
            initial_rel = self.transforms.geodetic_to_enu(
                msg.latitude, msg.longitude, msg.altitude,
                origin[0], origin[1], origin[2]
            )
            self.debug_logger.log_origin_set(
                origin=origin,
                initial_rel=initial_rel,
                target_rel=self.sensor_manager.get_relative_start_enu()
            )

        # Log first GPS fix
        if not self.sensor_manager.data_received['gps']:
            self.debug_logger.log_first_data_received(
                'GPS fix',
                f'lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.3f} m'
            )
            self.sensor_manager.data_received['gps'] = True

    def waypoint_callback(self, msg: Float32MultiArray):
        """Handle waypoint (goal) updates."""
        if len(msg.data) < 4:
            self.get_logger().warn(
                f'Invalid /fc/waypoint payload (expected ≥4, got {len(msg.data)})',
                throttle_duration_sec=2.0
            )
            return

        wp_no = int(msg.data[0])
        lat = float(msg.data[1])
        lon = float(msg.data[2])
        alt = float(msg.data[3])

        self.sensor_manager.update_goal(lat, lon, alt)

        if not self.sensor_manager.data_received['waypoint']:
            self.debug_logger.log_first_data_received(
                'Waypoint',
                f'wp#{wp_no} (lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m)'
            )
            self.sensor_manager.data_received['waypoint'] = True

    def att_quat_callback(self, msg: QuaternionStamped):
        """Handle attitude quaternion updates."""
        self.sensor_manager.update_attitude_quaternion(
            x=msg.quaternion.x,
            y=msg.quaternion.y,
            z=msg.quaternion.z,
            w=msg.quaternion.w
        )

        if not self.sensor_manager.data_received['att_quat']:
            quat = self.sensor_manager.get_quaternion()
            self.debug_logger.log_first_data_received(
                'Attitude quaternion',
                f'q=[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]'
            )
            self.sensor_manager.data_received['att_quat'] = True

    def att_euler_callback(self, msg: Vector3Stamped):
        """Handle attitude Euler angles updates."""
        self.sensor_manager.update_attitude_euler(
            roll=msg.vector.x,
            pitch=msg.vector.y,
            yaw=msg.vector.z
        )

        if not self.sensor_manager.data_received['att_euler']:
            euler = self.sensor_manager.get_euler()
            self.debug_logger.log_first_data_received(
                'Attitude Euler',
                f'rpy=[{euler[0]:.6f}, {euler[1]:.6f}, {euler[2]:.6f}] rad'
            )
            self.sensor_manager.data_received['att_euler'] = True

    def imu_callback(self, msg: Imu):
        """Handle IMU angular velocity updates."""
        self.sensor_manager.update_angular_velocity(
            wx=msg.angular_velocity.x,
            wy=msg.angular_velocity.y,
            wz=msg.angular_velocity.z
        )

        if not self.sensor_manager.data_received['imu']:
            ang_vel = self.sensor_manager.get_angular_velocity()
            self.debug_logger.log_first_data_received(
                'IMU',
                f'ω=[{ang_vel[0]:.5f}, {ang_vel[1]:.5f}, {ang_vel[2]:.5f}] rad/s'
            )
            self.sensor_manager.data_received['imu'] = True

    def gps_speed_callback(self, msg: Float32MultiArray):
        """Handle GPS speed and course updates."""
        if len(msg.data) < 2:
            self.get_logger().warn(
                f'Invalid /fc/gps_speed_course payload (expected 2, got {len(msg.data)})',
                throttle_duration_sec=2.0
            )
            return

        speed_mps = float(msg.data[0])
        course_deg = float(msg.data[1])

        # Convert to ENU velocity
        velocity_enu = self.transforms.velocity_cog_to_enu(speed_mps, course_deg)

        self.sensor_manager.update_velocity_from_gps(speed_mps, course_deg, velocity_enu)

        if not self.sensor_manager.data_received['gps_speed']:
            vel = self.sensor_manager.get_velocity()
            self.debug_logger.log_first_data_received(
                'GPS speed/course',
                f'speed={speed_mps:.3f} m/s, course={course_deg:.1f}°, '
                f'vx_east={vel[0]:.3f}, vy_north={vel[1]:.3f}'
            )
            self.sensor_manager.data_received['gps_speed'] = True

    def lidar_callback(self, msg: Range):
        """Handle LiDAR range updates."""
        if np.isfinite(msg.range) and msg.range > 0.0:
            normalized_dist = self.obs_builder.normalize_lidar_distance(msg.range)
            self.sensor_manager.update_lidar_ray(1, normalized_dist)  # Ray 1 = down
        else:
            self.sensor_manager.update_lidar_ray(1, 1.0)  # Invalid = max range
            self.get_logger().warn(
                f'Invalid LiDAR reading: {msg.range}',
                throttle_duration_sec=2.0
            )

        if not self.sensor_manager.data_received['lidar']:
            lidar = self.sensor_manager.get_lidar_distances()
            self.debug_logger.log_first_data_received(
                'LiDAR',
                f'down_norm={lidar[1]:.4f}'
            )
            self.sensor_manager.data_received['lidar'] = True

    def action_callback(self, msg: Float32MultiArray):
        """Handle AI action updates."""
        if len(msg.data) < 4:
            self.get_logger().warn(
                f'Invalid /ai/action payload (expected 4, got {len(msg.data)})',
                throttle_duration_sec=2.0
            )
            return

        action = np.array(msg.data[:4], dtype=np.float32)
        self.obs_builder.update_action(action)

        if not self.sensor_manager.data_received['action']:
            self.sensor_manager.mark_action_received()
            self.debug_logger.log_first_data_received('AI action', f'{action}')

        action_count = self.obs_builder.get_action_count()
        print(f'[ACTION #{action_count}] [{action[0]:.4f}, {action[1]:.4f}, '
              f'{action[2]:.4f}, {action[3]:.4f}]')

    # ═══════════════════════════════════════════════════════════════════
    # Observation Computation
    # ═══════════════════════════════════════════════════════════════════

    def compute_observation(self):
        """Compute and publish 131-D observation."""
        # Check required data
        required = ['gps', 'att_quat', 'att_euler', 'imu', 'lidar', 'gps_speed', 'waypoint']
        if not self.sensor_manager.is_data_ready(required):
            missing = self.sensor_manager.get_missing_data(required)
            self.get_logger().warn(
                f'⚠️  Waiting for required data: {missing}',
                throttle_duration_sec=1.0
            )
            return

        try:
            # Prepare action for this observation tick
            last_action = self.obs_builder.prepare_action_for_observation()

            # Get sensor data
            position = self.sensor_manager.get_position()
            origin = self.sensor_manager.get_origin()
            goal = self.sensor_manager.get_goal()

            # Compute relative position ENU
            rel_pos_enu = self.transforms.geodetic_to_enu(
                position[0], position[1], position[2],
                origin[0], origin[1], origin[2]
            )

            # Compute goal vector
            goal_enu = self.transforms.geodetic_to_enu(
                goal[0], goal[1], goal[2],
                origin[0], origin[1], origin[2]
            )
            goal_vector = self.obs_builder.compute_goal_vector(goal_enu, rel_pos_enu)

            # Build full observation
            full_obs = self.obs_builder.build_full_observation(
                rel_pos_enu=rel_pos_enu,
                quat_att=self.sensor_manager.get_quaternion(),
                euler_att=self.sensor_manager.get_euler(),
                velocity=self.sensor_manager.get_velocity(),
                angular_velocity=self.sensor_manager.get_angular_velocity(),
                last_action=last_action,
                lidar_distances=self.sensor_manager.get_lidar_distances(),
                goal_vector=goal_vector
            )

            self.obs_count += 1

            # Debug logging
            breakdown = self.obs_builder.get_observation_breakdown(full_obs)
            speed_mps, course_deg = self.sensor_manager.get_speed_and_course()

            self.debug_logger.print_observation_detailed(
                full_obs=full_obs,
                breakdown=breakdown,
                speed_mps=speed_mps,
                course_deg=course_deg,
                goal_geodetic=goal,
                obs_count=self.obs_count,
                action_count=self.obs_builder.get_action_count(),
                data_status=self.sensor_manager.get_data_status()
            )

            # Publish observation
            obs_msg = Float32MultiArray()
            obs_msg.data = full_obs.tolist()
            self.obs_pub.publish(obs_msg)

            # Publish debug vector [E, N, U, yaw, down_lidar, used_action_flag]
            debug_msg = Float32MultiArray()
            used_action_flag = 1.0 if self.obs_builder.is_using_action(last_action) else 0.0
            debug_msg.data = [
                float(rel_pos_enu[0]),
                float(rel_pos_enu[1]),
                float(rel_pos_enu[2]),
                float(self.sensor_manager.get_euler()[2]),  # yaw
                float(breakdown['lidar_distances'][1]),      # down lidar
                used_action_flag
            ]
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'❌ Error computing observation: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    # ═══════════════════════════════════════════════════════════════════
    # Node Lifecycle
    # ═══════════════════════════════════════════════════════════════════

    def destroy_node(self):
        """Clean shutdown."""
        self.debug_logger.print_shutdown_stats(
            obs_count=self.obs_count,
            action_count=self.obs_builder.get_action_count(),
            action_buffer_size=self.obs_builder.action_buffer_size,
            data_status=self.sensor_manager.get_data_status()
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AIAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
