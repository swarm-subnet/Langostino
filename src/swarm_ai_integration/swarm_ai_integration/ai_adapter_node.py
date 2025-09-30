#!/usr/bin/env python3
"""
AI Adapter Node - Converts real sensor data to 131-D observation array for Swarm AI model
This node subscribes to real sensor data (LiDAR, IMU, GPS, FC state) and constructs
the 131-dimensional observation array that the Swarm AI model expects:
- Base observation (112-D): Position, orientation, velocity, angular velocity, last action, action buffer
- LiDAR distances (16-D): Obstacle detection rays (normalized [0,1])
- Goal vector (3-D): Relative position to target (normalized by max_ray_distance)

DEBUG VERSION: Prints full 131-D array every time it's calculated
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
from collections import deque


class AIAdapterNode(Node):
    """
    Converts real-world sensor data into 131-D observation array for Swarm AI model.
    
    Subscribers:
        /lidar_distance (sensor_msgs/Range): LiDAR range data (down-facing)
        /fc/imu_raw (sensor_msgs/Imu): IMU orientation and angular velocity
        /fc/gps_fix (sensor_msgs/NavSatFix): GPS position data
        /fc/state (geometry_msgs/TwistStamped): Flight controller velocity data
        /fc/motor_rpm (std_msgs/Float32MultiArray): Motor RPM data (for monitoring)
        /goal_pose (geometry_msgs/PoseStamped): Target position
        /set_goal (geometry_msgs/Point): Simple target position
        /ai/action (std_msgs/Float32MultiArray): Actions from AI model (4-D)
        
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
        self.rpm = np.zeros(4, dtype=np.float32)  # [rpm1, rpm2, rpm3, rpm4] - for monitoring only
        self.goal_position = np.zeros(3, dtype=np.float32)  # [gx, gy, gz]
        
        # LiDAR: solo 1 rayo hacia abajo, resto en 1.0 (sin objeto)
        self.lidar_distances = np.ones(16, dtype=np.float32)  # All at 1.0 (normalized max)
        
        # Action tracking - último comando del modelo IA
        self.last_action = np.zeros(4, dtype=np.float32)  # [motor1, motor2, motor3, motor4] normalized
        
        # Action buffer: últimas 20 acciones para contexto temporal
        # Inicializado con ceros (deque automáticamente elimina el más antiguo cuando está lleno)
        self.action_buffer = deque(maxlen=20)
        for _ in range(20):
            self.action_buffer.append(np.zeros(4, dtype=np.float32))
        
        # State tracking
        self.data_received = {
            'imu': False,
            'gps': False,
            'lidar': False,
            'fc_state': False,
            'rpm': False,
            'goal': False,
            'action': False
        }
        
        # Statistics
        self.action_count = 0
        self.obs_count = 0
        
        # Constants - matching original code
        self.MAX_RAY_DISTANCE = 10.0  # Max distance for normalization (from original code)
        self.LIDAR_MIN_RANGE = 0.05   # Minimum LiDAR range in meters
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
        
        # Subscribers - Sensores
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
        
        # Subscribers - Goals
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, reliable_qos)
        
        self.goal_point_sub = self.create_subscription(
            Point, '/set_goal', self.set_goal_callback, reliable_qos)
        
        # Subscriber - AI Actions (IMPORTANTE)
        self.action_sub = self.create_subscription(
            Float32MultiArray,
            '/ai/action',
            self.action_callback,
            reliable_qos
        )
        
        # Publishers
        self.obs_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation', reliable_qos)
        
        self.debug_pub = self.create_publisher(
            Float32MultiArray, '/ai/observation_debug', reliable_qos)
        
        # Timer for periodic observation computation
        self.timer = self.create_timer(0.033, self.compute_observation)  # ~30 Hz
        
        print('=' * 80)
        print('🤖 AI ADAPTER NODE INITIALIZED - DEBUG MODE')
        print('=' * 80)
        print('📡 SUBSCRIBING TO:')
        print('   • /fc/imu_raw (sensor_msgs/Imu) [BEST_EFFORT]')
        print('   • /fc/gps_fix (sensor_msgs/NavSatFix) [BEST_EFFORT]')
        print('   • /lidar_distance (sensor_msgs/Range) [BEST_EFFORT]')
        print('   • /fc/state (geometry_msgs/TwistStamped) [RELIABLE]')
        print('   • /fc/motor_rpm (std_msgs/Float32MultiArray) [BEST_EFFORT]')
        print('   • /goal_pose (geometry_msgs/PoseStamped) [RELIABLE]')
        print('   • /set_goal (geometry_msgs/Point) [RELIABLE]')
        print('   • /ai/action (std_msgs/Float32MultiArray) [RELIABLE] ⭐')
        print('📤 PUBLISHING TO:')
        print('   • /ai/observation (std_msgs/Float32MultiArray) [RELIABLE]')
        print('   • /ai/observation_debug (std_msgs/Float32MultiArray) [RELIABLE]')
        print(f'🎯 OBSERVATION CONFIG:')
        print(f'   • Max ray distance: {self.MAX_RAY_DISTANCE}m (for normalization)')
        print(f'   • LiDAR range: {self.LIDAR_MIN_RANGE}m - ∞')
        print(f'   • Action buffer size: 20 steps (initialized with zeros)')
        print('🐛 DEBUG MODE: Full 131-D array will be printed on every computation')
        print('=' * 80)
        self.get_logger().info('AI Adapter Node initialized - 131-D observation space')
        self.get_logger().info('Waiting for AI model to publish actions on /ai/action...')
    
    def normalize_lidar_distance(self, distance):
        """
        Normalize LiDAR distance to [0, 1] range by dividing by MAX_RAY_DISTANCE.
        Matches original code: distances_scaled = distances_m / self.max_ray_distance
        
        Args:
            distance: Raw distance in meters
            
        Returns:
            Normalized distance in [0, 1]
        """
        # Clamp to positive values
        distance = max(0.0, distance)
        
        # Normalize by max_ray_distance (10m)
        normalized = distance / self.MAX_RAY_DISTANCE
        
        # Clamp to [0, 1]
        normalized = min(1.0, normalized)
        
        return normalized
    
    def action_callback(self, msg: Float32MultiArray):
        """
        Recibe las acciones generadas por el modelo IA y las guarda en el buffer.
        El buffer es un deque con maxlen=20, por lo que automáticamente elimina
        la acción más antigua cuando se agrega una nueva y el buffer está lleno.
        
        Args:
            msg: Float32MultiArray con 4 valores [motor1, motor2, motor3, motor4]
                 Ya deben estar normalizados en el rango apropiado
        """
        if len(msg.data) >= 4:
            action = np.array(msg.data[:4], dtype=np.float32)
            
            # Guardar como última acción
            self.last_action = action.copy()
            
            # Agregar al buffer circular
            # deque con maxlen automáticamente elimina el elemento más antiguo (índice 0)
            # cuando se hace append y el buffer está lleno
            self.action_buffer.append(action.copy())
            
            self.action_count += 1
            
            if not self.data_received['action']:
                self.get_logger().info('✓ AI action data received - action buffer active')
                self.data_received['action'] = True
                print(f'[ACTION] First action: [{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}, {action[3]:.3f}]')
                print(f'[ACTION] Buffer initialized with {len(self.action_buffer)} slots (20 max)')
            
            # Log cada acción recibida
            print(f'[ACTION #{self.action_count}] Received: [{action[0]:.4f}, {action[1]:.4f}, {action[2]:.4f}, {action[3]:.4f}]')
        else:
            self.get_logger().warn(f'Invalid action data: {len(msg.data)} values (expected 4)', 
                                   throttle_duration_sec=2.0)
    
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
            self.get_logger().info('✓ IMU data received')
            self.data_received['imu'] = True
            print(f'[IMU] Orientation (quat): [{self.orientation[0]:.3f}, {self.orientation[1]:.3f}, {self.orientation[2]:.3f}, {self.orientation[3]:.3f}]')
            print(f'[IMU] Angular Velocity: [{self.angular_velocity[0]:.3f}, {self.angular_velocity[1]:.3f}, {self.angular_velocity[2]:.3f}] rad/s')
    
    def gps_callback(self, msg: NavSatFix):
        """Process GPS data for position"""
        self.position[0] = msg.latitude
        self.position[1] = msg.longitude
        self.position[2] = msg.altitude
        
        if not self.data_received['gps']:
            self.get_logger().info('✓ GPS data received')
            self.get_logger().warn('⚠️  GPS coordinates used as local position - implement GPS->ENU for production')
            self.data_received['gps'] = True
            print(f'[GPS] Position: [{self.position[0]:.6f}, {self.position[1]:.6f}, {self.position[2]:.3f}] (lat/lon/alt)')
    
    def lidar_callback(self, msg: Range):
        """
        Process single down-facing LiDAR range data.
        Stores normalized distance at index 1 (Pure Down ray).
        All other rays remain at 1.0 (no object detected).
        """
        distance = msg.range
        
        if not self.data_received['lidar']:
            self.get_logger().info(f'✓ LiDAR data received - frame: {msg.header.frame_id}')
            print(f'[LIDAR] Using Pure Down ray only (index 1)')
            print(f'[LIDAR] Other 15 rays set to 1.0 (no object)')
            print(f'[LIDAR] Normalization by MAX_RAY_DISTANCE = {self.MAX_RAY_DISTANCE}m')
        
        if np.isfinite(distance) and distance > 0:
            # Normalize and store at index 1 (Pure Down)
            normalized = self.normalize_lidar_distance(distance)
            self.lidar_distances[1] = normalized
        else:
            # Invalid reading: set to max (1.0 = no object)
            self.lidar_distances[1] = 1.0
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
            self.get_logger().info('✓ FC state data received')
            self.data_received['fc_state'] = True
            print(f'[FC_STATE] Velocity: [{self.velocity[0]:.3f}, {self.velocity[1]:.3f}, {self.velocity[2]:.3f}] m/s')
    
    def rpm_callback(self, msg: Float32MultiArray):
        """
        Process motor RPM data - SOLO PARA MONITOREO.
        Los RPMs reales NO se usan para el action buffer.
        El action buffer se llena únicamente con las acciones del modelo IA.
        """
        if len(msg.data) >= 4:
            self.rpm = np.array(msg.data[:4], dtype=np.float32)
            
            if not self.data_received['rpm']:
                self.get_logger().info('✓ Motor RPM data received (monitoring only)')
                self.data_received['rpm'] = True
                print(f'[RPM] Motors: [{self.rpm[0]:.1f}, {self.rpm[1]:.1f}, {self.rpm[2]:.1f}, {self.rpm[3]:.1f}] RPM')
                print(f'[RPM] Note: RPM values are for monitoring, NOT used in action buffer')
        else:
            self.get_logger().warn(f'Insufficient RPM data: {len(msg.data)} values', throttle_duration_sec=2.0)
    
    def goal_callback(self, msg: PoseStamped):
        """Process goal position from PoseStamped"""
        self.goal_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float32)
        self.data_received['goal'] = True
        print(f'[GOAL] Position: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}] m')
        self.get_logger().info(f'✓ Goal updated: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]')
    
    def set_goal_callback(self, msg: Point):
        """Process simple goal point"""
        self.goal_position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self.data_received['goal'] = True
        print(f'[SET_GOAL] Simple goal: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}] m')
        self.get_logger().info(f'✓ Simple goal set: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]')
    
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
        """
        Compute the 112-D base observation array matching _getDroneStateVector format:
        - Position (3): [x, y, z]
        - Orientation quaternion (4): [qx, qy, qz, qw]
        - Orientation Euler (3): [roll, pitch, yaw]
        - Velocity (3): [vx, vy, vz]
        - Angular velocity (3): [wx, wy, wz]
        - Last action (4): [motor1, motor2, motor3, motor4] normalized
        - Action buffer (80): 20 previous actions × 4 values = 80
        - Padding (12): zeros to reach 112
        
        Total: 3 + 4 + 3 + 3 + 3 + 4 + 80 + 12 = 112
        """
        euler = self.quaternion_to_euler(self.orientation)
        
        # Flatten action buffer: 20 actions × 4 = 80 values
        # deque mantiene el orden: [más_antigua, ..., más_reciente]
        action_buffer_flat = np.concatenate(list(self.action_buffer)).astype(np.float32)
        
        # Construct base observation
        obs = np.concatenate([
            self.position,              # 3
            self.orientation,           # 4
            euler,                      # 3
            self.velocity,              # 3
            self.angular_velocity,      # 3
            self.last_action,           # 4
            action_buffer_flat,         # 80
            np.zeros(12)                # 12 padding
        ])
        
        return obs.astype(np.float32)
    
    def print_observation_detailed(self, full_obs, base_obs, lidar_obs, goal_vector):
        """
        Print detailed breakdown of the 131-D observation array
        """
        print('\n' + '=' * 80)
        print(f'🐛 OBSERVATION #{self.obs_count} - DETAILED BREAKDOWN')
        print('=' * 80)
        
        # Extract components from base_obs
        pos = base_obs[0:3]
        quat = base_obs[3:7]
        euler = base_obs[7:10]
        vel = base_obs[10:13]
        ang_vel = base_obs[13:16]
        last_act = base_obs[16:20]
        action_buf = base_obs[20:100]
        padding = base_obs[100:112]
        
        print(f'📍 POSITION [0:3] (3 values):')
        print(f'   {pos}')
        print(f'   x={pos[0]:.6f}, y={pos[1]:.6f}, z={pos[2]:.6f}')
        
        print(f'\n🔄 ORIENTATION QUATERNION [3:7] (4 values):')
        print(f'   {quat}')
        print(f'   qx={quat[0]:.4f}, qy={quat[1]:.4f}, qz={quat[2]:.4f}, qw={quat[3]:.4f}')
        
        print(f'\n📐 ORIENTATION EULER [7:10] (3 values):')
        print(f'   {euler}')
        print(f'   roll={np.rad2deg(euler[0]):.2f}°, pitch={np.rad2deg(euler[1]):.2f}°, yaw={np.rad2deg(euler[2]):.2f}°')
        
        print(f'\n⚡ VELOCITY [10:13] (3 values):')
        print(f'   {vel}')
        print(f'   vx={vel[0]:.4f}, vy={vel[1]:.4f}, vz={vel[2]:.4f} m/s')
        
        print(f'\n🌀 ANGULAR VELOCITY [13:16] (3 values):')
        print(f'   {ang_vel}')
        print(f'   wx={ang_vel[0]:.4f}, wy={ang_vel[1]:.4f}, wz={ang_vel[2]:.4f} rad/s')
        
        print(f'\n🎯 LAST ACTION [16:20] (4 values):')
        print(f'   {last_act}')
        print(f'   m1={last_act[0]:.4f}, m2={last_act[1]:.4f}, m3={last_act[2]:.4f}, m4={last_act[3]:.4f}')
        
        print(f'\n📚 ACTION BUFFER [20:100] (80 values = 20 actions × 4):')
        # Mostrar las primeras 3 y últimas 3 acciones del buffer
        action_buf_reshaped = action_buf.reshape(20, 4)
        print(f'   Oldest actions (first 3):')
        for i in range(min(3, 20)):
            print(f'      [{i}] {action_buf_reshaped[i]}')
        if len(action_buf_reshaped) > 6:
            print(f'   ... ({len(action_buf_reshaped)-6} more actions) ...')
        print(f'   Newest actions (last 3):')
        for i in range(max(0, len(action_buf_reshaped)-3), len(action_buf_reshaped)):
            print(f'      [{i}] {action_buf_reshaped[i]}')
        
        print(f'\n⬜ PADDING [100:112] (12 values):')
        print(f'   {padding}')
        
        print(f'\n📡 LIDAR DISTANCES [112:128] (16 values, normalized):')
        print(f'   {lidar_obs}')
        print(f'   Ray 0 (Up):    {lidar_obs[0]:.4f}')
        print(f'   Ray 1 (Down):  {lidar_obs[1]:.4f} ⭐ (active sensor)')
        print(f'   Rays 2-15:     {lidar_obs[2:]} (all inactive = 1.0)')
        
        print(f'\n🎯 GOAL VECTOR [128:131] (3 values, normalized):')
        print(f'   {goal_vector}')
        print(f'   gx={goal_vector[0]:.4f}, gy={goal_vector[1]:.4f}, gz={goal_vector[2]:.4f}')
        goal_distance_normalized = np.linalg.norm(goal_vector)
        goal_distance_real = goal_distance_normalized * self.MAX_RAY_DISTANCE
        print(f'   Distance to goal: {goal_distance_real:.2f}m (normalized: {goal_distance_normalized:.4f})')
        
        print(f'\n📊 FULL ARRAY STATISTICS:')
        print(f'   Shape: {full_obs.shape}')
        print(f'   Min:   {np.min(full_obs):.6f}')
        print(f'   Max:   {np.max(full_obs):.6f}')
        print(f'   Mean:  {np.mean(full_obs):.6f}')
        print(f'   Std:   {np.std(full_obs):.6f}')
        
        print(f'\n🔢 COMPLETE 131-D ARRAY:')
        # Print en grupos de 10 para mejor legibilidad
        for i in range(0, len(full_obs), 10):
            end_idx = min(i + 10, len(full_obs))
            values = ', '.join([f'{v:.4f}' for v in full_obs[i:end_idx]])
            print(f'   [{i:3d}:{end_idx:3d}] {values}')
        
        print(f'\n📈 OBSERVATION STATUS:')
        print(f'   Actions received: {self.action_count}')
        print(f'   Using AI actions: {"✓ YES" if self.data_received["action"] else "✗ NO (using zeros)"}')
        print(f'   All sensors ready: {"✓ YES" if all(self.data_received[k] for k in ["imu", "gps", "lidar", "fc_state"]) else "✗ NO"}')
        print('=' * 80 + '\n')
    
    def compute_observation(self):
        """
        Compute the full 131-D observation array:
        - Base observation (112-D)
        - LiDAR distances normalized (16-D): divided by MAX_RAY_DISTANCE
        - Goal vector normalized (3-D): (goal - pos) / MAX_RAY_DISTANCE
        
        Matches original code:
        np.concatenate([base_obs, distances_scaled, rel], axis=1)
        """
        required_data = ['imu', 'gps', 'lidar']
        if not all(self.data_received[key] for key in required_data):
            missing = [key for key in required_data if not self.data_received[key]]
            self.get_logger().warn(f'⚠️  Missing sensor data: {missing}', throttle_duration_sec=1.0)
            return
        
        if not self.data_received['goal']:
            self.get_logger().warn('⚠️  No goal set - using default', throttle_duration_sec=5.0)
        
        if not self.data_received['action']:
            self.get_logger().warn('⚠️  No AI actions received yet - using zero actions', throttle_duration_sec=3.0)
        
        try:
            # 1. Base observation (112-D)
            base_obs = self.compute_base_observation()
            
            # 2. LiDAR distances (16-D) - ya están normalizados en [0,1]
            lidar_obs = self.lidar_distances.copy()
            
            # 3. Goal vector (3-D) - normalizado por MAX_RAY_DISTANCE
            if self.data_received['goal']:
                goal_vector = (self.goal_position - self.position) / self.MAX_RAY_DISTANCE
            else:
                # Default goal: 5m adelante, 2m arriba
                default_goal = self.position + np.array([5.0, 0.0, 2.0])
                goal_vector = (default_goal - self.position) / self.MAX_RAY_DISTANCE
            
            # 4. Concatenate to form 131-D observation
            # Matching: np.concatenate([base_obs, distances_scaled, rel], axis=1)
            full_obs = np.concatenate([
                base_obs,           # 112-D
                lidar_obs,          # 16-D
                goal_vector         # 3-D
            ]).astype(np.float32)
            
            # Verify shape
            assert full_obs.shape == (131,), f"Observation shape mismatch: {full_obs.shape}"
            
            self.obs_count += 1
            
            # PRINT DETAILED BREAKDOWN EVERY TIME
            self.print_observation_detailed(full_obs, base_obs, lidar_obs, goal_vector)
            
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
                [lidar_obs[1]],  # Down ray normalized
                [float(all(self.data_received.values()))],
                [float(self.action_count)]
            ])
            debug_msg.data = debug_data.tolist()
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error computing observation: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = AIAdapterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n' + '=' * 80)
        print('🛑 AI ADAPTER NODE SHUTTING DOWN...')
        print('=' * 80)
        print(f'📊 FINAL STATISTICS:')
        print(f'   • Total observations published: {node.obs_count}')
        print(f'   • Total actions received: {node.action_count}')
        print(f'   • Action buffer size: {len(node.action_buffer)}/20')
        print(f'   • Sensors status:')
        for sensor, status in node.data_received.items():
            status_icon = '✓' if status else '✗'
            print(f'      {status_icon} {sensor}: {"Connected" if status else "Not connected"}')
        print('=' * 80)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()