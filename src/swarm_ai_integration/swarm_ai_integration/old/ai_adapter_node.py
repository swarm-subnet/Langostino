#!/usr/bin/env python3
"""
AI Adapter Node - Converts real sensor data to 131-D observation array for Swarm AI model

Updates in this version:
- Subscribes to /fc/waypoint (Float32MultiArray: [wp_no, lat, lon, alt_m, heading, stay, navflag]).
- Treats that waypoint as the "goal": computes a local ENU vector from current GPS position to the goal.
- Normalizes the goal vector by MAX_RAY_DISTANCE (±10 m cap) and places it at the end of the 131-D observation.
- Uses /fc/gps_speed_course to compute horizontal velocity (ENU). IMU is only for angular velocity.
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

        # IMU angular velocity only (no accel integration here)
        self.angular_velocity = np.zeros(3, dtype=np.float32)             # /fc/imu_raw

        # Velocity from GPS speed & COG (ENU)
        self.velocity = np.zeros(3, dtype=np.float32)                     # [vx_east, vy_north, vz(=0)]
        self.speed_mps = 0.0
        self.course_deg = 0.0

        # Goal (geodetic) from /fc/waypoint: [lat, lon, alt]
        self.goal_geodetic = np.zeros(3, dtype=np.float32)                # [lat_deg, lon_deg, alt_m]

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
            'action': False,     # ever received at least one
            'gps_speed': False,  # /fc/gps_speed_course
            'waypoint': False,   # /fc/waypoint
        }

        # -------------------------
        # Constants / config
        # -------------------------
        self.MAX_RAY_DISTANCE = 10.0  # meters, used to normalize goal vector
        # No more static goal: will be computed from /fc/waypoint

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
        self.att_quat_sub = self.create_subscription(QuaternionStamped, '/fc/attitude', self.att_quat_callback, sensor_qos)
        self.att_euler_sub = self.create_subscription(Vector3Stamped, '/fc/attitude_euler', self.att_euler_callback, sensor_qos)

        # IMU (angular velocity only)
        self.imu_sub = self.create_subscription(Imu, '/fc/imu_raw', self.imu_callback, sensor_qos)

        # GPS speed & course
        self.gps_speed_sub = self.create_subscription(Float32MultiArray, '/fc/gps_speed_course', self.gps_speed_callback, sensor_qos)

        # Waypoint (goal) — publisher uses RELIABLE; match it to avoid QoS mismatches
        self.waypoint_sub = self.create_subscription(Float32MultiArray, '/fc/waypoint', self.waypoint_callback, reliable_qos)

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
        print('   • /fc/attitude           (geometry_msgs/QuaternionStamped)  → orientation quaternion (BEST_EFFORT)')
        print('   • /fc/attitude_euler     (geometry_msgs/Vector3Stamped)     → orientation Euler [rad] (BEST_EFFORT)')
        print('   • /fc/imu_raw            (sensor_msgs/Imu)                   → angular velocity (NO accel integration)')
        print('   • /fc/gps_speed_course   (std_msgs/Float32MultiArray[2])     → [speed_mps, course_deg] → vx, vy (ENU)')
        print('   • /fc/waypoint           (std_msgs/Float32MultiArray[7])     → goal [lat,lon,alt] (RELIABLE)')
        print('   • /lidar_distance        (sensor_msgs/Range)                 → LiDAR down ray (normalized)')
        print('   • /ai/action             (std_msgs/Float32MultiArray[4])     → last action + action buffer')
        print('📤 PUBLISHING TO:')
        print('   • /ai/observation        (std_msgs/Float32MultiArray)        → 131-D observation')
        print('   • /ai/observation_debug  (std_msgs/Float32MultiArray)        → debug vector')
        print('🎯 GOAL:')
        print('   • Derived from /fc/waypoint, converted to ENU and normalized by ±10 m')
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

    def _enu_offset_from_latlonalt(self, goal_lat, goal_lon, goal_alt, ref_lat, ref_lon, ref_alt):
        """
        Convert (goal_lat, goal_lon, goal_alt) into ENU offset [east, north, up] (meters)
        relative to (ref_lat, ref_lon, ref_alt). Uses small-angle local tangent approximation.
        """
        lat_rad = math.radians(ref_lat)
        dlat = float(goal_lat) - float(ref_lat)   # degrees
        dlon = float(goal_lon) - float(ref_lon)   # degrees

        # Rough meters-per-degree (good enough for local goals)
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(lat_rad)

        east  = dlon * m_per_deg_lon
        north = dlat * m_per_deg_lat
        up    = float(goal_alt) - float(ref_alt)

        return np.array([east, north, up], dtype=np.float32)

    def _goal_vector_scaled(self):
        """
        Simulation-equivalent goal: (GOAL_POS - pos_w) / MAX_RAY_DISTANCE
        - World frame = ENU (meters), using a local tangent approximation.
        - No clipping.
        """
        # Need both current position and waypoint goal
        if not (self.data_received['waypoint'] and self.data_received['gps']):
            return np.zeros(3, dtype=np.float32)

        lat, lon, alt = self.position.tolist()
        g_lat, g_lon, g_alt = self.goal_geodetic.tolist()

        # ENU offset from current position to goal, in meters (world frame)
        enu = self._enu_offset_from_latlonalt(
            goal_lat=g_lat, goal_lon=g_lon, goal_alt=g_alt,
            ref_lat=lat,     ref_lon=lon,     ref_alt=alt
        )
        # Scale by max ray distance (no clip)
        return (enu / float(self.MAX_RAY_DISTANCE)).astype(np.float32)

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

    def waypoint_callback(self, msg: Float32MultiArray):
        # Expect [wp_no, lat, lon, alt_m, heading, stay, navflag]
        if len(msg.data) < 4:
            self.get_logger().warn(f'Invalid /fc/waypoint payload (expected ≥4, got {len(msg.data)})',
                                   throttle_duration_sec=2.0)
            return
        wp_no = int(msg.data[0])
        lat = float(msg.data[1])
        lon = float(msg.data[2])
        alt = float(msg.data[3])
        self.goal_geodetic[:] = [lat, lon, alt]
        if not self.data_received['waypoint']:
            self.data_received['waypoint'] = True
            self.get_logger().info(f'✓ Waypoint received → goal set to wp#{wp_no} (lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m)')
        else:
            # Throttle noisy logs
            self.get_logger().debug(f'[WAYPOINT] wp#{wp_no} lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m')

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
        # Incoming angles are radians: vector.x=roll, vector.y=pitch, vector.z=yaw
        self.euler_att = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float32)
        if not self.data_received['att_euler']:
            self.data_received['att_euler'] = True
            self.get_logger().info('✓ Attitude Euler received from /fc/attitude_euler (rad)')
            print(f'[ATT-EULER] rpy=[{self.euler_att[0]:.6f}, {self.euler_att[1]:.6f}, {self.euler_att[2]:.6f}] rad')

    def imu_callback(self, msg: Imu):
        # Angular velocity only
        self.angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32
        )
        if not self.data_received['imu']:
            self.data_received['imu'] = True
            self.get_logger().info('✓ IMU data received (angular velocity only)')
            print(f'[IMU] ω=[{self.angular_velocity[0]:.5f}, {self.angular_velocity[1]:.5f}, {self.angular_velocity[2]:.5f}] rad/s')

    def gps_speed_callback(self, msg: Float32MultiArray):
        # Expect [speed_mps, course_deg]
        if len(msg.data) < 2:
            self.get_logger().warn(f'Invalid /fc/gps_speed_course payload (expected 2, got {len(msg.data)})',
                                   throttle_duration_sec=2.0)
            return
        self.speed_mps = float(msg.data[0])
        self.course_deg = float(msg.data[1])

        # COG is degrees clockwise from True North
        # ENU conversion: x=East, y=North
        course_rad = math.radians(self.course_deg)
        vx_east = self.speed_mps * math.sin(course_rad)
        vy_north = self.speed_mps * math.cos(course_rad)

        self.velocity[0] = float(vx_east)
        self.velocity[1] = float(vy_north)
        self.velocity[2] = 0.0  # vz left at 0 as requested

        if not self.data_received['gps_speed']:
            self.data_received['gps_speed'] = True
            self.get_logger().info('✓ GPS speed/course received → vx, vy computed (ENU)')
            print(f'[GPS-SPEED] speed={self.speed_mps:.3f} m/s, course={self.course_deg:.1f}°, '
                  f'vx_east={self.velocity[0]:.3f} m/s, vy_north={self.velocity[1]:.3f} m/s')

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
        Base 112-D vector layout:
          [0:  3] position (lat, lon, alt)
          [3:  7] orientation quaternion (from /fc/attitude)
          [7: 10] orientation Euler rpy (from /fc/attitude_euler) [rad]
          [10:13] velocity (from /fc/gps_speed_course → ENU)
          [13:16] angular velocity (from /fc/imu_raw)
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
            self.velocity.astype(np.float32),         # 3 (vx_east, vy_north, vz=0)
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

        print('\n⚡ VELOCITY [10:13] (3 values) — from /fc/gps_speed_course (ENU):')
        print(f'   {vel}')
        print(f'   speed={self.speed_mps:.4f} m/s, course={self.course_deg:.1f}° → '
              f'vx_east={vel[0]:.4f}, vy_north={vel[1]:.4f}, vz={vel[2]:.4f}')

        print('\n🌀 ANGULAR VELOCITY [13:16] (3 values) — /fc/imu_raw:')
        print(f'   {ang_vel}')
        print(f'   wx={ang_vel[0]:.4f}, wy={ang_vel[1]:.4f}, wz={ang_vel[2]:.4f} rad/s')

        print('\n🎯 GOAL VECTOR [128:131] (3 values) — from /fc/waypoint → ENU / 10 m (no clip):')
        print(f'   {goal_vector}')
        print(f'   gx={goal_vector[0]:.4f}, gy={goal_vector[1]:.4f}, gz={goal_vector[2]:.4f}')
        goal_distance_normalized = np.linalg.norm(goal_vector)
        goal_distance_real = goal_distance_normalized * self.MAX_RAY_DISTANCE
        print(f'   Distance to goal (approx): {goal_distance_real:.2f} m (normalized: {goal_distance_normalized:.4f})')

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
        all_ready = all(self.data_received[k] for k in ['gps', 'att_quat', 'att_euler', 'imu', 'lidar', 'gps_speed', 'waypoint'])
        print(f'   All sensors ready: {"✓ YES" if all_ready else "✗ NO"}')
        print('=' * 80 + '\n')

    def compute_observation(self):
        # Require the exact sources we now depend on
        required = ['gps', 'att_quat', 'att_euler', 'imu', 'lidar', 'gps_speed', 'waypoint']
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

            # 3) Goal (3-D) — from /fc/waypoint → ENU normalized
            goal_vector = self._goal_vector_scaled()

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
