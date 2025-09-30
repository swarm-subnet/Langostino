#!/usr/bin/env python3
"""
AI Adapter Node - Converts real sensor data to 131-D observation array for Swarm AI model

Field mapping per your instructions:
- Position (3):           /fc/gps_fix (NavSatFix.latitude, longitude, altitude)
- Orientation quaternion: /fc/attitude (QuaternionStamped.quaternion)
- Orientation Euler (3):  /fc/attitude_euler (Vector3Stamped.vector) [radians]
- Angular velocity (3):   /fc/imu_raw (Imu.angular_velocity)
- Velocity (3):           Derived from /fc/imu_raw (Imu.linear_acceleration) with gravity compensation and discrete integration
- Last action (4):        /ai/action (Float32MultiArray[4]); if no new action this tick => zeros for this tick
- Action buffer (80):     Last 20 actions × 4. If no new action this tick => pushes zeros to buffer
- Padding (12):           Zeros to reach 112-D base
- LiDAR (16):             Down-facing range on index 1 normalized; others fixed at 1.0
- Goal (3):               Hardcoded normalized vector [0.5, 0.0, 0.2]

DEBUG VERSION: Prints full 131-D array every time it's calculated.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix, Imu, Range
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped


class AIAdapterNode(Node):
    def __init__(self):
        super().__init__('ai_adapter_node')

        # -------------------------
        # Internal state (data)
        # -------------------------
        # Position: [lat, lon, alt]
        self.position = np.zeros(3, dtype=np.float32)

        # Orientation sources
        self.quat_att = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)  # /fc/attitude
        self.euler_att = np.zeros(3, dtype=np.float32)                    # /fc/attitude_euler (roll, pitch, yaw) [rad]

        # IMU-derived states
        self.angular_velocity = np.zeros(3, dtype=np.float32)             # /fc/imu_raw
        self.velocity = np.zeros(3, dtype=np.float32)                     # integrated from /fc/imu_raw
        self._prev_imu_time = None                                        # for Δt integration
        self._last_world_accel = np.zeros(3, dtype=np.float32)            # last gravity-compensated accel [debug]

        # LiDAR distances: 16 rays, only index 1 (down) is active, rest 1.0
        self.lidar_distances = np.ones(16, dtype=np.float32)

        # Actions
        self.last_action_received = np.zeros(4, dtype=np.float32)         # latest received action from /ai/action
        self.action_buffer = deque(maxlen=20)
        for _ in range(20):
            self.action_buffer.append(np.zeros(4, dtype=np.float32))

        # Stats / flags
        self.action_count = 0
        self._last_action_count_seen = 0
        self.obs_count = 0
        self.data_received = {
            'gps': False,
            'att_quat': False,
            'att_euler': False,
            'imu': False,
            'lidar': False,
            'action': False,  # ever received at least one
        }

        # -------------------------
        # Constants / config
        # -------------------------
        self.MAX_RAY_DISTANCE = 10.0       # meters, for normalization
        self.GRAVITY = 9.80665             # m/s^2
        self.MAX_DT = 0.5                  # s, integration guard
        self.MIN_DT = 1e-5                 # s, avoid division by ~0
        self.STATIC_GOAL_VECTOR = np.array([0.5, 0.0, 0.2], dtype=np.float32)  # already normalized (10 m range)

        # -------------------------
        # QoS setups
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
        # Position
        self.gps_sub = self.create_subscription(NavSatFix, '/fc/gps_fix', self.gps_callback, sensor_qos)

        # Attitude: quaternion & Euler
        self.att_quat_sub = self.create_subscription(QuaternionStamped, '/fc/attitude', self.att_quat_callback, reliable_qos)
        self.att_euler_sub = self.create_subscription(Vector3Stamped, '/fc/attitude_euler', self.att_euler_callback, reliable_qos)

        # IMU (angular vel + linear accel for velocity integration)
        self.imu_sub = self.create_subscription(Imu, '/fc/imu_raw', self.imu_callback, sensor_qos)

        # LiDAR: down-facing range
        self.lidar_sub = self.create_subscription(Range, '/lidar_distance', self.lidar_callback, sensor_qos)

        # AI actions
        self.action_sub = self.create_subscription(Float32MultiArray, '/ai/action', self.action_callback, reliable_qos)

        # -------------------------
        # Publishers
        # -------------------------
        self.obs_pub = self.create_publisher(Float32MultiArray, '/ai/observation', reliable_qos)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/ai/observation_debug', reliable_qos)

        # -------------------------
        # Timer (~30 Hz)
        # -------------------------
        self.timer = self.create_timer(0.033, self.compute_observation)

        # -------------------------
        # Banner
        # -------------------------
        print('=' * 80)
        print('🤖 AI ADAPTER NODE INITIALIZED - DEBUG MODE')
        print('=' * 80)
        print('📡 SUBSCRIBING TO:')
        print('   • /fc/gps_fix            (sensor_msgs/NavSatFix)            → position [lat,lon,alt]')
        print('   • /fc/attitude           (geometry_msgs/QuaternionStamped)  → orientation quaternion')
        print('   • /fc/attitude_euler     (geometry_msgs/Vector3Stamped)     → orientation Euler [rad]')
        print('   • /fc/imu_raw            (sensor_msgs/Imu)                   → ang. vel + accel (for velocity)')
        print('   • /lidar_distance        (sensor_msgs/Range)                 → LiDAR down ray (normalized)')
        print('   • /ai/action             (std_msgs/Float32MultiArray[4])     → last action + action buffer')
        print('📤 PUBLISHING TO:')
        print('   • /ai/observation        (std_msgs/Float32MultiArray)        → 131-D observation')
        print('   • /ai/observation_debug  (std_msgs/Float32MultiArray)        → debug vector')
        print('🎯 CONFIG:')
        print(f'   • Max ray distance: {self.MAX_RAY_DISTANCE} m (normalization)')
        print(f'   • Goal vector (hardcoded, normalized): {self.STATIC_GOAL_VECTOR.tolist()}')
        print(f'   • Action buffer size: {self.action_buffer.maxlen} steps (zeros when no action this tick)')
        print('🐛 DEBUG MODE: Full 131-D array printed every cycle')
        print('=' * 80)
        self.get_logger().info('AI Adapter Node initialized (131-D observation)')

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _normalize_lidar_distance(self, distance_m: float) -> float:
        d = max(0.0, float(distance_m))
        return float(min(1.0, d / self.MAX_RAY_DISTANCE))

    def _quat_to_rotmat(self, q: np.ndarray) -> np.ndarray:
        """Return 3x3 rotation matrix (world_from_body) from quaternion [x,y,z,w]."""
        x, y, z, w = [float(v) for v in q]
        # normalize to be safe
        n = math.sqrt(x*x + y*y + z*z + w*w)
        if n == 0.0:
            return np.eye(3, dtype=np.float32)
        x, y, z, w = x/n, y/n, z/n, w/n
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        R = np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
            [    2*(xy + wz),  1 - 2*(xx + zz),        2*(yz - wx)],
            [    2*(xz - wy),      2*(yz + wx),     1 - 2*(xx + yy)]
        ], dtype=np.float32)
        return R

    def _prepare_action_for_tick(self) -> np.ndarray:
        """
        Implements: "IF no message is there, set the observation in that instance to 0".
        - If a new /ai/action arrived since the last observation tick: use that (buffer already updated in callback).
        - Otherwise: push a zero action to the buffer and use zeros for this tick's 'last action'.
        """
        if self.action_count == self._last_action_count_seen:
            zero_act = np.zeros(4, dtype=np.float32)
            self.action_buffer.append(zero_act.copy())
            last_action_for_obs = zero_act
        else:
            last_action_for_obs = self.last_action_received.copy()
            # do NOT append here — the callback already appended the received action
        self._last_action_count_seen = self.action_count
        return last_action_for_obs

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        self.position[0] = float(msg.latitude)
        self.position[1] = float(msg.longitude)
        self.position[2] = float(msg.altitude)
        if not self.data_received['gps']:
            self.data_received['gps'] = True
            self.get_logger().info('✓ GPS fix received (using lat, lon, alt as position)')
            print(f'[GPS] lat={self.position[0]:.7f}, lon={self.position[1]:.7f}, alt={self.position[2]:.3f} m')

    def att_quat_callback(self, msg: QuaternionStamped):
        self.quat_att = np.array(
            [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w],
            dtype=np.float32
        )
        if not self.data_received['att_quat']:
            self.data_received['att_quat'] = True
            self.get_logger().info('✓ Attitude quaternion received from /fc/attitude')
            print(f'[ATT-QUAT] q=[{self.quat_att[0]:.6f}, {self.quat_att[1]:.6f}, {self.quat_att[2]:.6f}, {self.quat_att[3]:.6f}]')

    def att_euler_callback(self, msg: Vector3Stamped):
        # Incoming angles assumed radians: vector.x=roll, vector.y=pitch, vector.z=yaw
        self.euler_att = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float32)
        if not self.data_received['att_euler']:
            self.data_received['att_euler'] = True
            self.get_logger().info('✓ Attitude Euler received from /fc/attitude_euler (rad)')
            print(f'[ATT-EULER] rpy=[{self.euler_att[0]:.6f}, {self.euler_att[1]:.6f}, {self.euler_att[2]:.6f}] rad')

    def imu_callback(self, msg: Imu):
        # Angular velocity directly
        self.angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32
        )

        # Linear acceleration (body), likely includes gravity per your example
        accel_body = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            dtype=np.float32
        )

        # Δt for integration from message header time
        t = self._stamp_to_sec(msg.header.stamp)
        if self._prev_imu_time is not None:
            dt = t - self._prev_imu_time
            if dt >= self.MIN_DT and dt <= self.MAX_DT:
                # Use latest attitude quaternion to rotate body accel to world frame
                R_wb = self._quat_to_rotmat(self.quat_att)  # world_from_body
                accel_world = R_wb @ accel_body
                # Gravity compensation: subtract [0, 0, +g] in world frame
                accel_world[2] -= self.GRAVITY
                # Integrate
                self.velocity += accel_world * dt
                self._last_world_accel = accel_world.astype(np.float32)
            else:
                # Skip integration on invalid dt (sensor stall or jump)
                pass
        self._prev_imu_time = t

        if not self.data_received['imu']:
            self.data_received['imu'] = True
            self.get_logger().info('✓ IMU data received (angular vel + accel for velocity integration)')
            print(f'[IMU] ω=[{self.angular_velocity[0]:.5f}, {self.angular_velocity[1]:.5f}, {self.angular_velocity[2]:.5f}] rad/s')

    def lidar_callback(self, msg: Range):
        if np.isfinite(msg.range) and msg.range > 0.0:
            self.lidar_distances[1] = self._normalize_lidar_distance(float(msg.range))
        else:
            self.lidar_distances[1] = 1.0  # treat invalid as no obstacle (max)
            self.get_logger().warn(f'Invalid LiDAR reading: {msg.range}', throttle_duration_sec=2.0)
        if not self.data_received['lidar']:
            self.data_received['lidar'] = True
            self.get_logger().info(f'✓ LiDAR data received (down ray on index 1, others fixed at 1.0)')
            print(f'[LIDAR] down_norm={self.lidar_distances[1]:.4f} (MAX_RAY_DISTANCE={self.MAX_RAY_DISTANCE} m)')

    def action_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn(f'Invalid /ai/action payload (expected 4, got {len(msg.data)})',
                                   throttle_duration_sec=2.0)
            return
        action = np.array(msg.data[:4], dtype=np.float32)
        self.last_action_received = action.copy()
        self.action_buffer.append(action.copy())  # append immediately; compute() will not append again for this tick
        self.action_count += 1

        if not self.data_received['action']:
            self.data_received['action'] = True
            self.get_logger().info('✓ First AI action received — action buffer is now live')
            print(f'[ACTION #1] {action}')

        print(f'[ACTION #{self.action_count}] [{action[0]:.4f}, {action[1]:.4f}, {action[2]:.4f}, {action[3]:.4f}]')

    # -------------------------------------------------------------------------
    # Observation assembly
    # -------------------------------------------------------------------------
    def _compute_base_observation(self, last_action_for_obs: np.ndarray) -> np.ndarray:
        """
        Base 112-D vector layout (matches your original indexing):
          [0:  3] position (lat, lon, alt)
          [3:  7] orientation quaternion (from /fc/attitude)
          [7: 10] orientation Euler rpy (from /fc/attitude_euler) [rad]
          [10:13] velocity (integrated from IMU)
          [13:16] angular velocity (from IMU)
          [16:20] last action (this tick; zeros if no new action)
          [20:100] action buffer (20 × 4)
          [100:112] padding (12 zeros)
        """
        # Flatten action buffer
        action_buffer_flat = np.concatenate(list(self.action_buffer)).astype(np.float32)

        obs = np.concatenate([
            self.position.astype(np.float32),         # 3
            self.quat_att.astype(np.float32),         # 4
            self.euler_att.astype(np.float32),        # 3
            self.velocity.astype(np.float32),         # 3
            self.angular_velocity.astype(np.float32), # 3
            last_action_for_obs.astype(np.float32),   # 4
            action_buffer_flat,                       # 80
            np.zeros(12, dtype=np.float32)            # 12 padding
        ], dtype=np.float32)

        return obs

    def _print_observation_detailed(self, full_obs, base_obs, lidar_obs, goal_vector, last_action_for_obs):
        print('\n' + '=' * 80)
        print(f'🐛 OBSERVATION #{self.obs_count} - DETAILED BREAKDOWN')
        print('=' * 80)

        pos = base_obs[0:3]
        quat = base_obs[3:7]
        euler = base_obs[7:10]
        vel = base_obs[10:13]
        ang_vel = base_obs[13:16]
        last_act = base_obs[16:20]
        action_buf = base_obs[20:100]
        padding = base_obs[100:112]

        print('📍 POSITION [0:3] (3 values):')
        print(f'   {pos}')
        print(f'   lat={pos[0]:.6f}, lon={pos[1]:.6f}, alt={pos[2]:.3f} m')

        print('\n🔄 ORIENTATION QUATERNION [3:7] (4 values) — /fc/attitude:')
        print(f'   {quat}')
        print(f'   qx={quat[0]:.4f}, qy={quat[1]:.4f}, qz={quat[2]:.4f}, qw={quat[3]:.4f}')

        print('\n📐 ORIENTATION EULER [7:10] (3 values, rad) — /fc/attitude_euler:')
        print(f'   {euler}')
        print(f'   roll={euler[0]:.6f} rad, pitch={euler[1]:.6f} rad, yaw={euler[2]:.6f} rad')

        print('\n⚡ VELOCITY [10:13] (3 values) — integrated from /fc/imu_raw:')
        print(f'   {vel}')
        print(f'   vx={vel[0]:.4f}, vy={vel[1]:.4f}, vz={vel[2]:.4f} m/s')
        print(f'   last a_world (gravity-comp.): [{self._last_world_accel[0]:.3f}, {self._last_world_accel[1]:.3f}, {self._last_world_accel[2]:.3f}] m/s²')

        print('\n🌀 ANGULAR VELOCITY [13:16] (3 values) — /fc/imu_raw:')
        print(f'   {ang_vel}')
        print(f'   wx={ang_vel[0]:.4f}, wy={ang_vel[1]:.4f}, wz={ang_vel[2]:.4f} rad/s')

        print('\n🎯 LAST ACTION [16:20] (4 values) — /ai/action (zeros if no new msg this tick):')
        print(f'   {last_act}')
        print(f'   m1={last_act[0]:.4f}, m2={last_act[1]:.4f}, m3={last_act[2]:.4f}, m4={last_act[3]:.4f}')

        print('\n📚 ACTION BUFFER [20:100] (80 values = 20 actions × 4):')
        action_buf_reshaped = action_buf.reshape(20, 4)
        print('   Oldest actions (first 3):')
        for i in range(min(3, 20)):
            print(f'      [{i}] {action_buf_reshaped[i]}')
        if len(action_buf_reshaped) > 6:
            print(f'   ... ({len(action_buf_reshaped)-6} more actions) ...')
        print('   Newest actions (last 3):')
        for i in range(max(0, len(action_buf_reshaped)-3), len(action_buf_reshaped)):
            print(f'      [{i}] {action_buf_reshaped[i]}')

        print('\n⬜ PADDING [100:112] (12 values):')
        print(f'   {padding}')

        print('\n📡 LIDAR DISTANCES [112:128] (16 values, normalized):')
        print(f'   {lidar_obs}')
        print(f'   Ray 0 (Up):    {lidar_obs[0]:.4f}')
        print(f'   Ray 1 (Down):  {lidar_obs[1]:.4f} ⭐ (active sensor)')
        print(f'   Rays 2-15:     {lidar_obs[2:]} (all inactive = 1.0)')

        print('\n🎯 GOAL VECTOR [128:131] (3 values, normalized) — hardcoded:')
        print(f'   {goal_vector}')
        print(f'   gx={goal_vector[0]:.4f}, gy={goal_vector[1]:.4f}, gz={goal_vector[2]:.4f}')
        goal_distance_normalized = np.linalg.norm(goal_vector)
        goal_distance_real = goal_distance_normalized * self.MAX_RAY_DISTANCE
        print(f'   Distance to goal: {goal_distance_real:.2f} m (normalized: {goal_distance_normalized:.4f})')

        print('\n📊 FULL ARRAY STATISTICS:')
        print(f'   Shape: {full_obs.shape}')
        print(f'   Min:   {np.min(full_obs):.6f}')
        print(f'   Max:   {np.max(full_obs):.6f}')
        print(f'   Mean:  {np.mean(full_obs):.6f}')
        print(f'   Std:   {np.std(full_obs):.6f}')

        print('\n🔢 COMPLETE 131-D ARRAY:')
        for i in range(0, len(full_obs), 10):
            end_idx = min(i + 10, len(full_obs))
            values = ', '.join([f'{v:.4f}' for v in full_obs[i:end_idx]])
            print(f'   [{i:3d}:{end_idx:3d}] {values}')

        print('\n📈 OBSERVATION STATUS:')
        print(f'   Actions received (total): {self.action_count}')
        using_action = '✓ YES' if (self.action_count > 0 and not np.allclose(last_action_for_obs, 0.0)) else '✗ NO (zeros this tick)'
        print(f'   Using AI action this tick: {using_action}')
        all_ready = all(self.data_received[k] for k in ['gps', 'att_quat', 'att_euler', 'imu', 'lidar'])
        print(f'   All sensors ready: {"✓ YES" if all_ready else "✗ NO"}')
        print('=' * 80 + '\n')

    def compute_observation(self):
        # Require the exact sources you asked for
        required = ['gps', 'att_quat', 'att_euler', 'imu', 'lidar']
        if not all(self.data_received[k] for k in required):
            missing = [k for k in required if not self.data_received[k]]
            self.get_logger().warn(f'⚠️  Waiting for required data: {missing}', throttle_duration_sec=1.0)
            return

        # Prepare this tick's action (zeros if no new message since last tick)
        last_action_for_obs = self._prepare_action_for_tick()

        try:
            # 1) Base (112-D)
            base_obs = self._compute_base_observation(last_action_for_obs)

            # 2) LiDAR (16-D)
            lidar_obs = self.lidar_distances.copy()

            # 3) Goal (3-D) — hardcoded normalized vector
            goal_vector = self.STATIC_GOAL_VECTOR.copy()

            # Concatenate -> 131-D
            full_obs = np.concatenate([base_obs, lidar_obs, goal_vector]).astype(np.float32)
            assert full_obs.shape == (131,), f'Observation shape mismatch: {full_obs.shape}'

            self.obs_count += 1

            # Debug print
            self._print_observation_detailed(full_obs, base_obs, lidar_obs, goal_vector, last_action_for_obs)

            # Publish observation
            obs_msg = Float32MultiArray()
            obs_msg.data = full_obs.tolist()
            self.obs_pub.publish(obs_msg)

            # Publish compact debug vector (lat,lon,alt, yaw, down_lidar, used_action_flag)
            debug_msg = Float32MultiArray()
            used_action_flag = 0.0 if np.allclose(last_action_for_obs, 0.0) else 1.0
            debug_msg.data = [
                float(self.position[0]), float(self.position[1]), float(self.position[2]),
                float(self.euler_att[2]),                    # yaw [rad]
                float(lidar_obs[1]),                         # down lidar normalized
                used_action_flag
            ]
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'❌ Error computing observation: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    # -------------------------------------------------------------------------
    # Main
    # -------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = AIAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n' + '=' * 80)
        print('🛑 AI ADAPTER NODE SHUTTING DOWN...')
        print('=' * 80)
        print('📊 FINAL STATS:')
        print(f'   • Observations published: {node.obs_count}')
        print(f'   • Actions received: {node.action_count}')
        print(f'   • Action buffer size: {len(node.action_buffer)}/{node.action_buffer.maxlen}')
        print('   • Sensors:')
        for k, v in node.data_received.items():
            status_icon = '✓' if v else '✗'
            print(f'      {status_icon} {k}')
        print('=' * 80)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
