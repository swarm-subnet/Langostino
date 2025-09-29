#!/usr/bin/env python3
"""
AI Adapter Node - Converts real sensor data to 131-D observation array for Swarm AI model

This node subscribes to real sensor data (LiDAR, IMU, GPS, FC state) and constructs
the 131-dimensional observation array that the Swarm AI model expects:
- Base observation (112-D): Position, orientation, velocity, angular velocity, RPM, etc.
- LiDAR distances (16-D): Obstacle detection rays
- Goal vector (3-D): Relative position to target

Based on swarm/core/moving_drone.py:368-394 (_computeObs method)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Point
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose
import math


class AIAdapterNode(Node):
    """
    Converts real-world sensor data into 131-D observation array for Swarm AI model.

    Subscribers:
        /lidar_scan (sensor_msgs/LaserScan): LiDAR point cloud data
        /imu/data (sensor_msgs/Imu): IMU orientation and angular velocity
        /gps/fix (sensor_msgs/NavSatFix): GPS position data
        /fc/state (geometry_msgs/TwistStamped): Flight controller velocity data
        /fc/rpm (std_msgs/Float32MultiArray): Motor RPM data
        /goal_pose (geometry_msgs/PoseStamped): Target position

    Publishers:
        /ai/observation (std_msgs/Float32MultiArray): 131-D observation array
        /ai/observation_debug (std_msgs/Float32MultiArray): Debug info
    """

    def __init__(self):
        super().__init__('ai_adapter_node')

        # Initialize data storage
        self.position = np.zeros(3, dtype=np.float32)  # [x, y, z]
        self.orientation = np.zeros(4, dtype=np.float32)  # [x, y, z, w] quaternion
        self.velocity = np.zeros(3, dtype=np.float32)  # [vx, vy, vz]
        self.angular_velocity = np.zeros(3, dtype=np.float32)  # [wx, wy, wz]
        self.rpm = np.zeros(4, dtype=np.float32)  # [rpm1, rpm2, rpm3, rpm4]
        self.lidar_distances = np.full(16, 10.0, dtype=np.float32)  # Max distance 10m
        self.goal_position = np.zeros(3, dtype=np.float32)  # [gx, gy, gz]

        # State tracking
        self.last_position = np.zeros(3, dtype=np.float32)
        self.data_received = {
            'imu': False,
            'gps': False,
            'lidar': False,
            'fc_state': False,
            'rpm': False,
            'goal': False
        }

        # Constants from swarm simulation
        self.MAX_RAY_DISTANCE = 10.0  # Maximum LiDAR range in meters
        self.min_range = 0.05  # Minimum valid LiDAR range in meters
        self.GRAVITY = 9.8

        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profiles
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

        # Use BEST_EFFORT for motor RPM to match publisher QoS
        motor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/fc/imu_raw', self.imu_callback, sensor_qos)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fc/gps_fix', self.gps_callback, sensor_qos)
        self.lidar_sub = self.create_subscription(
            Range, '/lidar_distance', self.lidar_callback, sensor_qos)
        self.fc_state_sub = self.create_subscription(
            TwistStamped, '/fc/state', self.fc_state_callback, reliable_qos)
        self.rpm_sub = self.create_subscription(
            Float32MultiArray, '/fc/motor_rpm', self.rpm_callback, motor_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, reliable_qos)
        self.goal_point_sub = self.create_subscription(
            Point, '/set_goal', self.set_goal_callback, reliable_qos)

        # Publishers
        self.obs_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation', reliable_qos)
        self.debug_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation_debug', reliable_qos)

        # Timer for periodic observation computation
        self.timer = self.create_timer(0.033, self.compute_observation)  # ~30 Hz

        print('=' * 60)
        print('🤖 AI ADAPTER NODE INITIALIZED')
        print('=' * 60)
        print('📡 SUBSCRIBING TO:')
        print('  • /fc/imu_raw (sensor_msgs/Imu) [BEST_EFFORT]')
        print('  • /fc/gps_fix (sensor_msgs/NavSatFix) [BEST_EFFORT]')
        print('  • /lidar_distance (sensor_msgs/Range) [BEST_EFFORT]')
        print('  • /fc/state (geometry_msgs/TwistStamped) [RELIABLE]')
        print('  • /fc/motor_rpm (std_msgs/Float32MultiArray) [BEST_EFFORT]')
        print('  • /goal_pose (geometry_msgs/PoseStamped) [RELIABLE]')
        print('  • /set_goal (geometry_msgs/Point) [RELIABLE]')
        print('📤 PUBLISHING TO:')
        print('  • /ai/observation (std_msgs/Float32MultiArray) [RELIABLE]')
        print('  • /ai/observation_debug (std_msgs/Float32MultiArray) [RELIABLE]')
        print('=' * 60)

        self.get_logger().info('AI Adapter Node initialized with comprehensive logging')

    def imu_callback(self, msg: Imu):
        """Process IMU data for orientation and angular velocity"""
        # Orientation (quaternion)
        self.orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ], dtype=np.float32)

        # Angular velocity
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)

        if not self.data_received['imu']:
            self.get_logger().info('IMU data received - orientation and angular velocity updated')
        self.data_received['imu'] = True

        print(f'[IMU] Orientation (quat): [{self.orientation[0]:.3f}, {self.orientation[1]:.3f}, {self.orientation[2]:.3f}, {self.orientation[3]:.3f}]')
        print(f'[IMU] Angular Velocity: [{self.angular_velocity[0]:.3f}, {self.angular_velocity[1]:.3f}, {self.angular_velocity[2]:.3f}] rad/s')
        print(f'[IMU] Linear Acceleration: [{msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}] m/s²')

    def gps_callback(self, msg: NavSatFix):
        """Process GPS data for position (convert to local coordinates)"""
        # For real implementation, convert GPS lat/lon to local ENU coordinates
        # This is a simplified version - you'll need proper GPS->ENU conversion
        # based on your origin point

        # Placeholder: assume GPS provides position in meters relative to takeoff
        # In practice, use geographic projection library like pyproj
        self.position[0] = msg.latitude   # Replace with proper x conversion
        self.position[1] = msg.longitude  # Replace with proper y conversion
        self.position[2] = msg.altitude   # Replace with proper z conversion

        if not self.data_received['gps']:
            self.get_logger().info('GPS data received - position updated')
            self.get_logger().warn('GPS coordinates are being used directly as local position - implement proper GPS->ENU conversion for production use')
        self.data_received['gps'] = True

        print(f'[GPS] Position: [{self.position[0]:.6f}, {self.position[1]:.6f}, {self.position[2]:.3f}] (lat/lon/alt)')
        print(f'[GPS] Status: {msg.status.status}, Service: {msg.status.service}')
        if hasattr(msg, 'position_covariance'):
            print(f'[GPS] Covariance type: {msg.position_covariance_type}')

    def lidar_callback(self, msg: Range):
        """Process single LiDAR range data - simplified for single sensor setup"""
        # For now, use the single range measurement for all directions
        # This is a simplified approach since we have a single LiDAR sensor
        # In a full implementation, you'd have multiple sensors or a scanning LiDAR

        distance = msg.range

        if not self.data_received['lidar']:
            self.get_logger().info(f'LiDAR data received - frame: {msg.header.frame_id}, range: {distance:.3f}m')
            self.get_logger().info(f'LiDAR - Min range: {msg.min_range:.3f}m, Max range: {msg.max_range:.3f}m')

        print(f'[LIDAR] Raw distance: {distance:.3f}m, Frame: {msg.header.frame_id}')
        print(f'[LIDAR] Range limits: {msg.min_range:.3f}m - {msg.max_range:.3f}m, Radiation: {msg.radiation_type}')

        if np.isfinite(distance) and self.min_range <= distance <= msg.max_range:
            # Use this single measurement for multiple directions
            # This assumes obstacles are primarily in the sensor's direction

            # Based on sensor_position parameter, determine which directions to populate
            if hasattr(msg, 'header') and 'down' in msg.header.frame_id:
                # Down-facing sensor - primarily affects vertical measurements
                self.lidar_distances[1] = distance  # Down ray
                self.lidar_distances[0] = self.MAX_RAY_DISTANCE  # Up ray (no obstacles above)
                # Keep horizontal rays at max distance for down sensor
                self.lidar_distances[2:10] = self.MAX_RAY_DISTANCE
                print(f'[LIDAR] Down-facing sensor - distance: {distance:.3f}m')
            else:
                # Assume forward-facing or general purpose sensor
                # Use for forward direction and approximate others
                self.lidar_distances[2] = distance  # Forward (0°)
                # Set adjacent directions to similar values with some variation
                self.lidar_distances[3] = min(distance * 1.1, self.MAX_RAY_DISTANCE)  # 45°
                self.lidar_distances[9] = min(distance * 1.1, self.MAX_RAY_DISTANCE)  # 315°
                # Keep other directions at reasonable distances
                for i in [4, 5, 6, 7, 8]:  # Side and back directions
                    self.lidar_distances[i] = self.MAX_RAY_DISTANCE

                # Set vertical distances based on common assumptions
                self.lidar_distances[0] = self.MAX_RAY_DISTANCE  # Up (no obstacles above)
                self.lidar_distances[1] = max(distance * 0.8, 1.0)  # Down (approximate ground distance)
                print(f'[LIDAR] Forward-facing sensor - distance: {distance:.3f}m')

            # Diagonal rays (indices 10-15) - use approximations based on horizontal
            for i in range(6):
                horizontal_idx = 2 + (i % 4)
                self.lidar_distances[10 + i] = self.lidar_distances[horizontal_idx]

            print(f'[LIDAR] Updated distances array: {self.lidar_distances[:8].tolist()} (first 8 rays)')
        else:
            # Invalid reading - use maximum range for safety
            self.lidar_distances.fill(self.MAX_RAY_DISTANCE)
            print(f'[LIDAR] INVALID reading: {distance:.3f}m (valid range: {self.min_range:.3f}-{msg.max_range:.3f}m)')
            self.get_logger().warn(f'Invalid LiDAR reading: {distance:.3f}m (valid range: {self.min_range:.3f}-{msg.max_range:.3f}m)', throttle_duration_sec=2.0)

        self.data_received['lidar'] = True

    def fc_state_callback(self, msg: TwistStamped):
        """Process flight controller state for velocity"""
        self.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ], dtype=np.float32)

        if not self.data_received['fc_state']:
            self.get_logger().info('FC state data received - velocity updated')
        self.data_received['fc_state'] = True

        print(f'[FC_STATE] Linear Velocity: [{self.velocity[0]:.3f}, {self.velocity[1]:.3f}, {self.velocity[2]:.3f}] m/s')
        print(f'[FC_STATE] Angular Velocity: [{msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f}] rad/s')
        print(f'[FC_STATE] Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')

    def rpm_callback(self, msg: Float32MultiArray):
        """Process motor RPM data"""
        if len(msg.data) >= 4:
            self.rpm = np.array(msg.data[:4], dtype=np.float32)
            if not self.data_received['rpm']:
                self.get_logger().info('Motor RPM data received - RPM values updated')
            self.data_received['rpm'] = True

            print(f'[RPM] Motors: [{self.rpm[0]:.1f}, {self.rpm[1]:.1f}, {self.rpm[2]:.1f}, {self.rpm[3]:.1f}] RPM')
            print(f'[RPM] Total data points: {len(msg.data)}, Average RPM: {np.mean(self.rpm):.1f}')
        else:
            print(f'[RPM] INSUFFICIENT DATA: {len(msg.data)} values (expected 4) - Data: {list(msg.data)}')
            self.get_logger().warn(f'Insufficient RPM data received: {len(msg.data)} values (expected 4)', throttle_duration_sec=2.0)

    def goal_callback(self, msg: PoseStamped):
        """Process goal position"""
        self.goal_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float32)

        self.data_received['goal'] = True

        print(f'[GOAL] Position: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}] m')
        print(f'[GOAL] Orientation: [{msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f}]')
        print(f'[GOAL] Frame: {msg.header.frame_id}, Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')

        self.get_logger().info(f'Goal position updated: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]')

    def set_goal_callback(self, msg: Point):
        """Process simple goal point (convenience method)"""
        self.goal_position = np.array([
            msg.x,
            msg.y,
            msg.z
        ], dtype=np.float32)

        self.data_received['goal'] = True

        print(f'[SET_GOAL] Simple goal: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}] m')

        self.get_logger().info(f'Simple goal set to: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]')

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    def compute_base_observation(self):
        """
        Compute the 112-D base observation array matching gym_pybullet_drones format.
        This replicates the observation structure from the simulation.
        """
        # Convert quaternion to Euler angles
        euler = self.quaternion_to_euler(self.orientation)

        # Compute accelerations (simple finite difference approximation)
        dt = 0.033  # 30 Hz
        accel = (self.velocity - (self.velocity * 0.9)) / dt  # Simplified

        # Base observation structure (112-D) - simplified version
        # In practice, you'd need to match exactly the gym_pybullet_drones observation format
        obs = np.concatenate([
            self.position,              # [0:3] - position (x, y, z)
            self.orientation,           # [3:7] - quaternion (x, y, z, w)
            euler,                      # [7:10] - Euler angles (roll, pitch, yaw)
            self.velocity,              # [10:13] - velocity (vx, vy, vz)
            self.angular_velocity,      # [13:16] - angular velocity (wx, wy, wz)
            accel,                      # [16:19] - acceleration (ax, ay, az)
            np.zeros(3),                # [19:22] - angular acceleration (placeholder)
            self.rpm,                   # [22:26] - motor RPMs
            np.zeros(86)                # [26:112] - additional state variables (placeholder)
        ])

        return obs.astype(np.float32)

    def compute_observation(self):
        """
        Compute the full 131-D observation array:
        - Base observation (112-D)
        - LiDAR distances (16-D)
        - Goal vector (3-D)
        """
        # Check if we have minimum required data (goal is optional)
        required_data = ['imu', 'gps', 'lidar']
        if not all(self.data_received[key] for key in required_data):
            missing = [key for key in required_data if not self.data_received[key]]
            self.get_logger().warn(f'Missing sensor data: {missing}', throttle_duration_sec=1.0)

                # Print and log specific missing data with suggestions
            print(f'[MISSING] Sensor data missing: {missing}')
            for sensor in missing:
                if sensor == 'imu':
                    print('[MISSING] ❌ IMU data - check if /fc/imu_raw topic is publishing')
                    self.get_logger().warn('IMU data missing - check if /fc/imu_raw topic is publishing', throttle_duration_sec=5.0)
                elif sensor == 'gps':
                    print('[MISSING] ❌ GPS data - check if /fc/gps_fix topic is publishing')
                    self.get_logger().warn('GPS data missing - check if /fc/gps_fix topic is publishing', throttle_duration_sec=5.0)
                elif sensor == 'lidar':
                    print('[MISSING] ❌ LiDAR data - check if /lidar_distance topic is publishing')
                    self.get_logger().warn('LiDAR data missing - check if /lidar_distance topic is publishing', throttle_duration_sec=5.0)
            return

        # Warn about missing goal but continue processing
        if not self.data_received['goal']:
            self.get_logger().warn('No goal position set - using default goal', throttle_duration_sec=5.0)

        try:
            # Compute base observation (112-D)
            base_obs = self.compute_base_observation()

            # Scale LiDAR distances to [0,1] range
            lidar_scaled = self.lidar_distances / self.MAX_RAY_DISTANCE

            # Compute goal vector (relative position, scaled)
            if self.data_received['goal']:
                goal_vector = (self.goal_position - self.position) / self.MAX_RAY_DISTANCE
            else:
                # Use default goal slightly above current position if no goal is set
                default_goal = self.position + np.array([5.0, 0.0, 2.0])  # 5m forward, 2m up
                goal_vector = (default_goal - self.position) / self.MAX_RAY_DISTANCE

            # Concatenate to form 131-D observation
            full_obs = np.concatenate([
                base_obs,        # 112-D
                lidar_scaled,    # 16-D
                goal_vector      # 3-D
            ]).astype(np.float32)

            print(f'[OBSERVATION] Array shape: {full_obs.shape}, Min: {np.min(full_obs):.3f}, Max: {np.max(full_obs):.3f}, Mean: {np.mean(full_obs):.3f}')
            print(f'[OBSERVATION] Position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}]')
            print(f'[OBSERVATION] Velocity: [{self.velocity[0]:.3f}, {self.velocity[1]:.3f}, {self.velocity[2]:.3f}]')
            print(f'[OBSERVATION] LiDAR distances (first 8): {self.lidar_distances[:8].tolist()}')
            print(f'[OBSERVATION] Goal vector: [{goal_vector[0]:.3f}, {goal_vector[1]:.3f}, {goal_vector[2]:.3f}]')
            print(f'[OBSERVATION] Motor RPM: [{self.rpm[0]:.1f}, {self.rpm[1]:.1f}, {self.rpm[2]:.1f}, {self.rpm[3]:.1f}]')
            print('=' * 60)

            # Log data reception status periodically
            if hasattr(self, '_last_status_log'):
                if (self.get_clock().now().nanoseconds - self._last_status_log) > 5e9:  # 5 seconds
                    self._log_data_status()
            else:
                self._log_data_status()
                self._last_status_log = self.get_clock().now().nanoseconds

            # Publish observation
            obs_msg = Float32MultiArray()
            obs_msg.data = full_obs.tolist()
            self.obs_pub.publish(obs_msg)

            # Publish debug info
            debug_msg = Float32MultiArray()
            debug_data = np.concatenate([
                self.position,
                self.goal_position,
                [np.linalg.norm(goal_vector)],  # Distance to goal
                [np.mean(self.lidar_distances)],  # Average obstacle distance
                [float(all(self.data_received.values()))]  # All data received flag
            ])
            debug_msg.data = debug_data.tolist()
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error computing observation: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def _log_data_status(self):
        """Log current data reception status"""
        status_msg = '[STATUS] Data reception: '
        for sensor, received in self.data_received.items():
            status_msg += f'{sensor}:{"✓" if received else "✗"} '
        print(status_msg)
        self.get_logger().info(status_msg)

        if all(self.data_received.values()):
            print('[STATUS] ✅ All sensor data streams active - AI observation publishing')
            self.get_logger().info('All sensor data streams active - AI observation publishing')
        else:
            missing_sensors = [sensor for sensor, received in self.data_received.items() if not received]
            missing_count = len(missing_sensors)
            print(f'[STATUS] ❌ {missing_count} sensor streams inactive: {missing_sensors}')
            self.get_logger().warn(f'{missing_count} sensor streams inactive - check topic publishers: {missing_sensors}')


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