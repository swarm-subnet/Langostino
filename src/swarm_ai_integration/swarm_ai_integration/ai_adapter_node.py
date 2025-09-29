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
        self.lidar_distances = np.full(16, 20.0, dtype=np.float32)  # Max distance 20m
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
        
        # Constants - Updated for 20m max range
        self.LIDAR_MIN_RANGE = 0.05  # Minimum LiDAR range in meters
        self.LIDAR_MAX_RANGE = 20.0  # Maximum LiDAR range in meters
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
        print('   • /fc/imu_raw (sensor_msgs/Imu) [BEST_EFFORT]')
        print('   • /fc/gps_fix (sensor_msgs/NavSatFix) [BEST_EFFORT]')
        print('   • /lidar_distance (sensor_msgs/Range) [BEST_EFFORT]')
        print('   • /fc/state (geometry_msgs/TwistStamped) [RELIABLE]')
        print('   • /fc/motor_rpm (std_msgs/Float32MultiArray) [BEST_EFFORT]')
        print('   • /goal_pose (geometry_msgs/PoseStamped) [RELIABLE]')
        print('   • /set_goal (geometry_msgs/Point) [RELIABLE]')
        print('📤 PUBLISHING TO:')
        print('   • /ai/observation (std_msgs/Float32MultiArray) [RELIABLE]')
        print('   • /ai/observation_debug (std_msgs/Float32MultiArray) [RELIABLE]')
        print(f'🎯 LIDAR CONFIG: Range {self.LIDAR_MIN_RANGE}m - {self.LIDAR_MAX_RANGE}m')
        print('=' * 60)
        self.get_logger().info('AI Adapter Node initialized with 20m LiDAR range')
    
    def normalize_lidar_distance(self, distance):
        """
        Normalize LiDAR distance to [0, 1] range.
        - Clamps distance between 0.05m and 20m
        - Maps 0.05m -> 0.0 and 20m -> 1.0
        
        Args:
            distance: Raw distance in meters
            
        Returns:
            Normalized distance in [0, 1]
        """
        # Clamp distance to valid range
        clamped = np.clip(distance, self.LIDAR_MIN_RANGE, self.LIDAR_MAX_RANGE)
        
        # Normalize to [0, 1]
        normalized = (clamped - self.LIDAR_MIN_RANGE) / (self.LIDAR_MAX_RANGE - self.LIDAR_MIN_RANGE)
        
        return normalized
    
    def imu_callback(self, msg: Imu):
        """Process IMU data for orientation and angular velocity"""
        self.orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ], dtype=np.float32)
        
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
        """Process GPS data for position"""
        self.position[0] = msg.latitude
        self.position[1] = msg.longitude
        self.position[2] = msg.altitude
        
        if not self.data_received['gps']:
            self.get_logger().info('GPS data received - position updated')
            self.get_logger().warn('GPS coordinates are being used directly as local position - implement proper GPS->ENU conversion for production use')
            self.data_received['gps'] = True
            print(f'[GPS] Position: [{self.position[0]:.6f}, {self.position[1]:.6f}, {self.position[2]:.3f}] (lat/lon/alt)')
            print(f'[GPS] Status: {msg.status.status}, Service: {msg.status.service}')
    
    def lidar_callback(self, msg: Range):
        """
        Process single LiDAR range data - using only Pure Down ray (index 1)
        Maps distance to normalized [0,1] where:
        - 0.05m (min) -> 0.0
        - 20m (max) -> 1.0
        """
        distance = msg.range
        
        if not self.data_received['lidar']:
            self.get_logger().info(f'LiDAR data received - frame: {msg.header.frame_id}, range: {distance:.3f}m')
            self.get_logger().info(f'LiDAR - Will use Pure Down ray only (index 1)')
            print(f'[LIDAR] Raw distance: {distance:.3f}m, Frame: {msg.header.frame_id}')
            print(f'[LIDAR] Range limits: {msg.min_range:.3f}m - {msg.max_range:.3f}m')
            print(f'[LIDAR] Normalization: {self.LIDAR_MIN_RANGE}m->0.0, {self.LIDAR_MAX_RANGE}m->1.0')
        
        if np.isfinite(distance):
            # Store raw distance for Pure Down ray (index 1)
            self.lidar_distances[1] = distance
            
            # Set other rays to max distance
            self.lidar_distances[0] = self.LIDAR_MAX_RANGE  # Pure Up
            self.lidar_distances[2:] = self.LIDAR_MAX_RANGE  # All horizontal and diagonal rays
            
            # Calculate normalized value for debugging
            normalized = self.normalize_lidar_distance(distance)
            
            print(f'[LIDAR] Pure Down - Raw: {distance:.3f}m, Normalized: {normalized:.3f}')
            
            if distance > self.LIDAR_MAX_RANGE:
                print(f'[LIDAR] ⚠️  Distance capped from {distance:.3f}m to {self.LIDAR_MAX_RANGE}m')
            elif distance < self.LIDAR_MIN_RANGE:
                print(f'[LIDAR] ⚠️  Distance floored from {distance:.3f}m to {self.LIDAR_MIN_RANGE}m')
        else:
            self.lidar_distances.fill(self.LIDAR_MAX_RANGE)
            print(f'[LIDAR] INVALID reading: {distance:.3f}m - using max range')
            self.get_logger().warn(f'Invalid LiDAR reading: {distance:.3f}m', throttle_duration_sec=2.0)
        
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
    
    def rpm_callback(self, msg: Float32MultiArray):
        """Process motor RPM data"""
        if len(msg.data) >= 4:
            self.rpm = np.array(msg.data[:4], dtype=np.float32)
            if not self.data_received['rpm']:
                self.get_logger().info('Motor RPM data received - RPM values updated')
                self.data_received['rpm'] = True
                print(f'[RPM] Motors: [{self.rpm[0]:.1f}, {self.rpm[1]:.1f}, {self.rpm[2]:.1f}, {self.rpm[3]:.1f}] RPM')
        else:
            print(f'[RPM] INSUFFICIENT DATA: {len(msg.data)} values (expected 4)')
            self.get_logger().warn(f'Insufficient RPM data: {len(msg.data)} values', throttle_duration_sec=2.0)
    
    def goal_callback(self, msg: PoseStamped):
        """Process goal position"""
        self.goal_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float32)
        self.data_received['goal'] = True
        print(f'[GOAL] Position: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}] m')
        self.get_logger().info(f'Goal position updated: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]')
    
    def set_goal_callback(self, msg: Point):
        """Process simple goal point"""
        self.goal_position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self.data_received['goal'] = True
        print(f'[SET_GOAL] Simple goal: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}] m')
        self.get_logger().info(f'Simple goal set to: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]')
    
    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw], dtype=np.float32)
    
    def compute_base_observation(self):
        """Compute the 112-D base observation array"""
        euler = self.quaternion_to_euler(self.orientation)
        dt = 0.033
        accel = (self.velocity - (self.velocity * 0.9)) / dt
        
        obs = np.concatenate([
            self.position,
            self.orientation,
            euler,
            self.velocity,
            self.angular_velocity,
            accel,
            np.zeros(3),
            self.rpm,
            np.zeros(86)
        ])
        
        return obs.astype(np.float32)
    
    def compute_observation(self):
        """
        Compute the full 131-D observation array with normalized LiDAR distances
        """
        required_data = ['imu', 'gps', 'lidar']
        if not all(self.data_received[key] for key in required_data):
            missing = [key for key in required_data if not self.data_received[key]]
            self.get_logger().warn(f'Missing sensor data: {missing}', throttle_duration_sec=1.0)
            return
        
        if not self.data_received['goal']:
            self.get_logger().warn('No goal position set - using default goal', throttle_duration_sec=5.0)
        
        try:
            base_obs = self.compute_base_observation()
            
            # Normalize LiDAR distances to [0,1]
            lidar_normalized = np.array([
                self.normalize_lidar_distance(d) for d in self.lidar_distances
            ], dtype=np.float32)
            
            # Compute goal vector
            if self.data_received['goal']:
                goal_vector = (self.goal_position - self.position) / self.LIDAR_MAX_RANGE
            else:
                default_goal = self.position + np.array([5.0, 0.0, 2.0])
                goal_vector = (default_goal - self.position) / self.LIDAR_MAX_RANGE
            
            # Concatenate to form 131-D observation
            full_obs = np.concatenate([
                base_obs,
                lidar_normalized,
                goal_vector
            ]).astype(np.float32)
            
            print(f'[OBSERVATION] Array shape: {full_obs.shape}, Min: {np.min(full_obs):.3f}, Max: {np.max(full_obs):.3f}')
            print(f'[OBSERVATION] Position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}]')
            print(f'[OBSERVATION] Pure Down LiDAR: Raw={self.lidar_distances[1]:.3f}m, Normalized={lidar_normalized[1]:.3f}')
            print(f'[OBSERVATION] Goal vector: [{goal_vector[0]:.3f}, {goal_vector[1]:.3f}, {goal_vector[2]:.3f}]')
            print('=' * 60)
            
            # Publish observation
            obs_msg = Float32MultiArray()
            obs_msg.data = full_obs.tolist()
            self.obs_pub.publish(obs_msg)
            
            # Publish debug info
            debug_msg = Float32MultiArray()
            debug_data = np.concatenate([
                self.position,
                self.goal_position,
                [np.linalg.norm(goal_vector)],
                [self.lidar_distances[1]],  # Raw down distance
                [lidar_normalized[1]],  # Normalized down distance
                [float(all(self.data_received.values()))]
            ])
            debug_msg.data = debug_data.tolist()
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error computing observation: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')


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