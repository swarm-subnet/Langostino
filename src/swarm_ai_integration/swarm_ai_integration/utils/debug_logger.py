#!/usr/bin/env python3
"""
Debug Logger - Detailed observation logging and debugging utilities

This module handles:
- Formatted observation breakdowns
- Detailed sensor data logging
- Statistics and progress tracking
- Human-readable debug output
"""

import numpy as np
from typing import Dict, Optional
from rclpy.node import Node


class DebugLogger:
    """
    Provides detailed logging and debugging for AI observations.

    Formats observation data in human-readable format with detailed
    breakdowns of each component.
    """

    def __init__(self, node: Node, max_ray_distance: float = 10.0):
        """
        Initialize debug logger.

        Args:
            node: ROS2 node for logging
            max_ray_distance: Maximum distance for denormalization (meters)
        """
        self.node = node
        self.max_ray_distance = max_ray_distance

    def print_observation_detailed(
        self,
        full_obs: np.ndarray,
        breakdown: Dict[str, np.ndarray],
        speed_mps: float,
        course_deg: float,
        goal_geodetic: np.ndarray,
        obs_count: int,
        action_count: int,
        data_status: Dict[str, bool]
    ):
        """
        Print detailed breakdown of observation.

        Args:
            full_obs: Complete 131-D observation array
            breakdown: Dictionary with observation components
            speed_mps: GPS speed in m/s
            course_deg: GPS course in degrees
            goal_geodetic: Goal position [lat, lon, alt]
            obs_count: Observation count
            action_count: Total actions received
            data_status: Data availability flags
        """
        pos = breakdown['rel_pos_enu']
        euler = breakdown['euler_att']
        vel = breakdown['velocity']
        ang_vel = breakdown['angular_velocity']
        action_buf = breakdown['action_buffer']
        lidar_obs = breakdown['lidar_distances']
        goal_vector = breakdown['goal_vector']

        print('\n' + '=' * 80)
        print(f'🐛 OBSERVATION #{obs_count} - DETAILED BREAKDOWN')
        print('=' * 80)

        # Position
        print('📍 RELATIVE POSITION ENU [0:3] (meters):')
        print(f'   {pos}')
        print(f'   E={pos[0]:.3f} m, N={pos[1]:.3f} m, U={pos[2]:.3f} m')

        # Orientation (Euler only - no quaternion)
        print('\n📐 ORIENTATION EULER [3:6] (3 values, rad) — /fc/attitude_euler:')
        print(f'   {euler}')
        print(f'   roll={euler[0]:.6f} rad ({np.degrees(euler[0]):.1f}°), '
              f'pitch={euler[1]:.6f} rad ({np.degrees(euler[1]):.1f}°), '
              f'yaw={euler[2]:.6f} rad ({np.degrees(euler[2]):.1f}°)')

        # Velocity
        print('\n⚡ VELOCITY [6:9] (3 values) — from /fc/gps_speed_course (ENU):')
        print(f'   {vel}')
        print(f'   speed={speed_mps:.4f} m/s, course={course_deg:.1f}° → '
              f'vx_east={vel[0]:.4f}, vy_north={vel[1]:.4f}, vz={vel[2]:.4f}')

        # Angular Velocity
        print('\n🌀 ANGULAR VELOCITY [9:12] (3 values) — /fc/imu_raw:')
        print(f'   {ang_vel}')
        print(f'   wx={ang_vel[0]:.4f}, wy={ang_vel[1]:.4f}, wz={ang_vel[2]:.4f} rad/s')

        # Action Buffer (25 actions now, no separate last_action field)
        print('\n📚 ACTION HISTORY [12:112] (100 values = 25 actions × 4):')
        action_buf_reshaped = action_buf.reshape(25, 4)
        print('   Oldest actions (first 3):')
        for i in range(min(3, 25)):
            print(f'      [{i}] {action_buf_reshaped[i]}')
        if len(action_buf_reshaped) > 6:
            print(f'   ... ({len(action_buf_reshaped)-6} more actions) ...')
        print('   Newest actions (last 3):')
        for i in range(max(0, len(action_buf_reshaped)-3), len(action_buf_reshaped)):
            print(f'      [{i}] {action_buf_reshaped[i]}')
        print('   Most recent action is at the end of the buffer')

        # LiDAR
        print('\n📡 LIDAR DISTANCES [112:128] (16 values, normalized):')
        print(f'   {lidar_obs}')
        print(f'   Ray 0 (Up):    {lidar_obs[0]:.4f}')
        print(f'   Ray 1 (Down):  {lidar_obs[1]:.4f} ⭐ (active sensor, '
              f'{lidar_obs[1] * self.max_ray_distance:.2f}m)')
        print(f'   Rays 2-15:     {lidar_obs[2:]} (all inactive = 1.0)')

        # Goal Vector
        print('\n🎯 GOAL VECTOR [128:131] (3 values) — from /fc/waypoint → ENU / 10 m (no clip):')
        print(f'   {goal_vector}')
        print(f'   gx={goal_vector[0]:.4f}, gy={goal_vector[1]:.4f}, gz={goal_vector[2]:.4f}')
        goal_distance_normalized = np.linalg.norm(goal_vector)
        goal_distance_real = goal_distance_normalized * self.max_ray_distance
        print(f'   Distance to goal (approx): {goal_distance_real:.2f} m '
              f'(normalized: {goal_distance_normalized:.4f})')
        print(f'   Goal geodetic: lat={goal_geodetic[0]:.7f}, '
              f'lon={goal_geodetic[1]:.7f}, alt={goal_geodetic[2]:.1f}m')

        # Array Statistics
        print('\n📊 FULL ARRAY STATISTICS:')
        print(f'   Shape: {full_obs.shape}')
        print(f'   Min:   {np.min(full_obs):.6f}')
        print(f'   Max:   {np.max(full_obs):.6f}')
        print(f'   Mean:  {np.mean(full_obs):.6f}')
        print(f'   Std:   {np.std(full_obs):.6f}')

        # Complete Array
        print('\n🔢 COMPLETE 131-D ARRAY:')
        for i in range(0, len(full_obs), 10):
            end_idx = min(i + 10, len(full_obs))
            values = ', '.join([f'{v:.4f}' for v in full_obs[i:end_idx]])
            print(f'   [{i:3d}:{end_idx:3d}] {values}')

        # Status
        print('\n📈 OBSERVATION STATUS:')
        print(f'   Observations published: {obs_count}')
        print(f'   Actions received (total): {action_count}')
        all_ready = all(data_status.values())
        print(f'   All sensors ready: {"✓ YES" if all_ready else "✗ NO"}')

        # Sensor Status
        print('\n📡 SENSOR DATA STATUS:')
        for key, status in data_status.items():
            status_icon = '✓' if status else '✗'
            print(f'      {status_icon} {key}')

        print('=' * 80 + '\n')

    def print_initial_banner(
        self,
        telemetry_rate: float,
        action_buffer_size: int,
        max_ray_distance: float,
        relative_start_enu: np.ndarray
    ):
        """
        Print initialization banner with configuration.

        Args:
            telemetry_rate: Observation publishing rate (Hz)
            action_buffer_size: Size of action buffer
            max_ray_distance: Maximum ray distance (meters)
            relative_start_enu: Initial relative position
        """
        print('=' * 80)
        print('🤖 AI ADAPTER NODE INITIALIZED - DEBUG MODE')
        print('=' * 80)
        print('📡 SUBSCRIBING TO:')
        print('   • /fc/gps_fix            (sensor_msgs/NavSatFix)            → position [lat,lon,alt]')
        print('   • /fc/attitude_euler     (geometry_msgs/Vector3Stamped)     → orientation Euler [rad]')
        print('   • /fc/imu_raw            (sensor_msgs/Imu)                   → angular velocity')
        print('   • /fc/gps_speed_course   (std_msgs/Float32MultiArray[2])     → [speed_mps, course_deg]')
        print('   • /fc/waypoint           (std_msgs/Float32MultiArray[7])     → goal [lat,lon,alt]')
        print('   • /lidar_distance        (sensor_msgs/Range)                 → LiDAR down ray')
        print('   • /ai/action             (std_msgs/Float32MultiArray[4])     → AI actions')
        print('📤 PUBLISHING TO:')
        print('   • /ai/observation        (std_msgs/Float32MultiArray)        → 131-D observation')
        print('   • /ai/observation_debug  (std_msgs/Float32MultiArray)        → debug vector')
        print('⚙️  CONFIGURATION:')
        print(f'   • Observation rate: ~{telemetry_rate:.1f} Hz')
        print(f'   • Action buffer size: {action_buffer_size} actions')
        print(f'   • Max ray distance: {max_ray_distance} m (for normalization)')
        print(f'   • Relative start ENU: [{relative_start_enu[0]:.1f}, '
              f'{relative_start_enu[1]:.1f}, {relative_start_enu[2]:.1f}] m')
        print('=' * 80)

    def print_shutdown_stats(
        self,
        obs_count: int,
        action_count: int,
        action_buffer_size: int,
        data_status: Dict[str, bool]
    ):
        """
        Print shutdown statistics.

        Args:
            obs_count: Total observations published
            action_count: Total actions received
            action_buffer_size: Size of action buffer
            data_status: Data availability flags
        """
        print('\n' + '=' * 80)
        print('🛑 AI ADAPTER NODE SHUTTING DOWN...')
        print('=' * 80)
        print('📊 FINAL STATS:')
        print(f'   • Observations published: {obs_count}')
        print(f'   • Actions received: {action_count}')
        print(f'   • Action buffer size: {action_buffer_size}')
        print('   • Sensors:')
        for k, v in data_status.items():
            status_icon = '✓' if v else '✗'
            print(f'      {status_icon} {k}')
        print('=' * 80)

    def log_first_data_received(self, data_type: str, details: str = ''):
        """
        Log when first data of a type is received.

        Args:
            data_type: Type of data received
            details: Optional details to log
        """
        message = f'✓ {data_type} received'
        if details:
            message += f' - {details}'
        self.node.get_logger().info(message)
        if details:
            print(f'[{data_type.upper()}] {details}')

    def log_origin_set(self, origin: np.ndarray, initial_rel: np.ndarray, target_rel: np.ndarray):
        """
        Log when relative origin is set.

        Args:
            origin: Origin geodetic [lat, lon, alt]
            initial_rel: Initial relative position achieved
            target_rel: Target relative position
        """
        self.node.get_logger().info(
            f'✓ Relative origin set so initial rel pos ≈ '
            f'[{initial_rel[0]:.3f}, {initial_rel[1]:.3f}, {initial_rel[2]:.3f}] m '
            f'(target {target_rel.tolist()})'
        )
        print(f'[REL-ORIGIN] origin_lat={origin[0]:.7f}, '
              f'origin_lon={origin[1]:.7f}, origin_alt={origin[2]:.3f} m')
