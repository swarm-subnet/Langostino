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
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
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

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, sensor_qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar_scan', self.lidar_callback, sensor_qos)
        self.fc_state_sub = self.create_subscription(
            TwistStamped, '/fc/state', self.fc_state_callback, reliable_qos)
        self.rpm_sub = self.create_subscription(
            Float32MultiArray, '/fc/rpm', self.rpm_callback, reliable_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, reliable_qos)

        # Publishers
        self.obs_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation', reliable_qos)
        self.debug_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation_debug', reliable_qos)

        # Timer for periodic observation computation
        self.timer = self.create_timer(0.033, self.compute_observation)  # ~30 Hz

        self.get_logger().info('AI Adapter Node initialized')

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

        self.data_received['imu'] = True

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

        self.data_received['gps'] = True

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR data to create 16-directional distance measurements"""
        # Convert LaserScan to 16 specific directions matching simulation
        # The simulation uses these 16 directions (from moving_drone.py:116-138):
        # - 2 vertical (up/down)
        # - 8 horizontal (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°)
        # - 6 diagonal (±30° elevation)

        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            return

        # Map LiDAR scan to 16 specific directions
        num_rays = len(ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Define target angles for the 16 directions (in radians)
        target_angles = [
            0.0,                    # Forward
            np.pi/4,               # Forward-Right (45°)
            np.pi/2,               # Right (90°)
            3*np.pi/4,             # Back-Right (135°)
            np.pi,                 # Back (180°)
            5*np.pi/4,             # Back-Left (225°)
            3*np.pi/2,             # Left (270°)
            7*np.pi/4,             # Forward-Left (315°)
        ]

        # For horizontal rays (first 8), find closest LiDAR measurements
        for i, target_angle in enumerate(target_angles):
            # Find closest ray index
            ray_index = int((target_angle - angle_min) / angle_increment)
            ray_index = max(0, min(ray_index, num_rays - 1))

            distance = ranges[ray_index]
            if np.isfinite(distance):
                self.lidar_distances[i + 2] = min(distance, self.MAX_RAY_DISTANCE)
            else:
                self.lidar_distances[i + 2] = self.MAX_RAY_DISTANCE

        # For vertical and diagonal rays, use statistical approximation
        # Up/Down rays (indices 0, 1)
        self.lidar_distances[0] = self.MAX_RAY_DISTANCE  # Up (no obstacle above)
        self.lidar_distances[1] = np.min(valid_ranges) if len(valid_ranges) > 0 else self.MAX_RAY_DISTANCE  # Down

        # Diagonal rays (indices 10-15) - use horizontal approximations
        for i in range(6):
            horizontal_idx = 2 + (i % 4)  # Map to corresponding horizontal ray
            self.lidar_distances[10 + i] = self.lidar_distances[horizontal_idx]

        self.data_received['lidar'] = True

    def fc_state_callback(self, msg: TwistStamped):
        """Process flight controller state for velocity"""
        self.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ], dtype=np.float32)

        print(f"[DEBUG] FC State - Velocity: [{self.velocity[0]:.3f}, {self.velocity[1]:.3f}, {self.velocity[2]:.3f}]")

        self.data_received['fc_state'] = True

    def rpm_callback(self, msg: Float32MultiArray):
        """Process motor RPM data"""
        if len(msg.data) >= 4:
            self.rpm = np.array(msg.data[:4], dtype=np.float32)
            print(f"[DEBUG] FC RPM - Motors: [{self.rpm[0]:.1f}, {self.rpm[1]:.1f}, {self.rpm[2]:.1f}, {self.rpm[3]:.1f}]")
            self.data_received['rpm'] = True

    def goal_callback(self, msg: PoseStamped):
        """Process goal position"""
        self.goal_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float32)

        self.data_received['goal'] = True

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
        roll, pitch, yaw = euler

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
        # Check if we have minimum required data
        required_data = ['imu', 'gps', 'lidar', 'goal']
        if not all(self.data_received[key] for key in required_data):
            missing = [key for key in required_data if not self.data_received[key]]
            self.get_logger().warn(f'Missing sensor data: {missing}', throttle_duration_sec=1.0)
            return

        try:
            # Compute base observation (112-D)
            base_obs = self.compute_base_observation()

            # Scale LiDAR distances to [0,1] range
            lidar_scaled = self.lidar_distances / self.MAX_RAY_DISTANCE

            # Compute goal vector (relative position, scaled)
            goal_vector = (self.goal_position - self.position) / self.MAX_RAY_DISTANCE

            # Concatenate to form 131-D observation
            full_obs = np.concatenate([
                base_obs,        # 112-D
                lidar_scaled,    # 16-D
                goal_vector      # 3-D
            ]).astype(np.float32)

            print(f"[DEBUG] Observation Array - Shape: {full_obs.shape}, Min: {np.min(full_obs):.3f}, Max: {np.max(full_obs):.3f}, Mean: {np.mean(full_obs):.3f}")
            print(f"[DEBUG] LiDAR distances: {self.lidar_distances[:8]}")
            print(f"[DEBUG] Goal vector: [{goal_vector[0]:.3f}, {goal_vector[1]:.3f}, {goal_vector[2]:.3f}]")

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