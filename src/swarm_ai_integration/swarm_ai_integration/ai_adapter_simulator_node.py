#!/usr/bin/env python3
"""
AI Adapter Simulated Node
Deterministic, no-noise, perfect velocity tracking harness for AIFlightNode.

Loop:
    /ai/observation  --> AIFlightNode -->  /ai/action [vx, vy, vz, speed]
                                        ^                      |
                                        |______ this node _____|

Behavior (idealized):
- Yaw fixed at 0 rad (body frame == ENU frame) for simplicity.
- Each physics tick (default 10 Hz), position += meters_per_step * speed * [vx, vy, vz].
- No actuator lag, no noise, no dropouts.
- Publishes realistic sensor topics and constructs the exact 131-D observation
  via your real ObservationBuilder/CoordinateTransforms/SensorDataManager.

Do NOT run fc_comms_node at the same time (it publishes the same /fc/* topics).
"""

import math
import time
from typing import Optional

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
    DebugLogger,
)


def quat_from_yaw(yaw_rad: float):
    """Quaternion for yaw-only (roll=pitch=0). Returns (x,y,z,w)."""
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


class AIAdapterSimulatedNode(Node):
    def __init__(self):
        super().__init__('ai_adapter_simulated_node')

        # ─────────────────────────
        # Parameters
        # ─────────────────────────
        self.declare_parameter('telemetry_rate', 30.0)             # Hz for building/publishing observation
        self.declare_parameter('physics_rate', 10.0)               # Hz for state integration (match AIFlightNode 10 Hz)
        self.declare_parameter('meters_per_step', 0.15)            # 15cm per physics tick
        self.declare_parameter('use_speed_scalar', True)           # if True, scale [vx,vy,vz] by 'speed'

        # Start geodetic (approx your logs, adjust as needed)
        self.declare_parameter('start_lat', 37.2486500)
        self.declare_parameter('start_lon', -5.3724274)
        self.declare_parameter('start_alt', 182.0)

        # Desired initial relative ENU (same semantics as your real adapter)
        self.declare_parameter('relative_start_enu', [0.0, 0.0, 3.0])  # meters (hardcoded start)

        # Goal as RELATIVE ENU offset from origin (converted to geodetic here)
        self.declare_parameter('goal_relative_enu', [5.0, 5.0, 3.0])   # E,N,U in meters (default final point)

        # Lidar model
        self.declare_parameter('lidar_min_range', 0.05)
        self.declare_parameter('lidar_max_range', 20.0)
        self.declare_parameter('lidar_fov', 0.035)  # rad
        self.declare_parameter('lidar_frame', 'lidar_link')

        # Frames for other sensors
        self.declare_parameter('gps_frame', 'fc_gps')
        self.declare_parameter('att_frame', 'fc_attitude')
        self.declare_parameter('imu_frame', 'fc_imu')

        # Read params
        telemetry_rate = float(self.get_parameter('telemetry_rate').value)
        physics_rate = float(self.get_parameter('physics_rate').value)
        self.meters_per_step = float(self.get_parameter('meters_per_step').value)
        self.use_speed_scalar = bool(self.get_parameter('use_speed_scalar').value)

        self.start_lat = float(self.get_parameter('start_lat').value)
        self.start_lon = float(self.get_parameter('start_lon').value)
        self.start_alt = float(self.get_parameter('start_alt').value)

        self.relative_start_enu = np.array(
            self.get_parameter('relative_start_enu').get_parameter_value().double_array_value,
            dtype=np.float32,
        )
        self.goal_rel_enu = np.array(
            self.get_parameter('goal_relative_enu').get_parameter_value().double_array_value,
            dtype=np.float32,
        )

        self.lidar_min = float(self.get_parameter('lidar_min_range').value)
        self.lidar_max = float(self.get_parameter('lidar_max_range').value)
        self.lidar_fov = float(self.get_parameter('lidar_fov').value)
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)

        self.gps_frame = str(self.get_parameter('gps_frame').value)
        self.att_frame = str(self.get_parameter('att_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)

        # ─────────────────────────
        # Core modules (reuse your real utils to keep 131-D exact)
        # ─────────────────────────
        self.transforms = CoordinateTransforms()
        self.sensor_manager = SensorDataManager(num_lidar_rays=16, relative_start_enu=self.relative_start_enu)
        self.obs_builder = ObservationBuilder(action_buffer_size=20, max_ray_distance=self.lidar_max)
        self.debug_logger = DebugLogger(node=self, max_ray_distance=self.lidar_max)

        # ─────────────────────────
        # World/state (ENU relative to origin)
        # ─────────────────────────
        # Compute origin so that the initial GPS fix maps to the desired relative_start_enu
        self.origin = self.transforms.compute_origin_for_initial_position(
            current_lat=self.start_lat,
            current_lon=self.start_lon,
            current_alt=self.start_alt,
            desired_enu=self.relative_start_enu,
        )
        self.sensor_manager.set_origin_geodetic(self.origin)

        # Start position: at relative_start_enu, yaw=0, zero angular vel
        self.rel_pos_enu = self.relative_start_enu.astype(np.float32).copy()
        self.yaw = 0.0  # rad (fixed for this simple sim)
        self.vel_enu = np.zeros(3, dtype=np.float32)  # derived from actions per tick

        # Compute goal geodetic from goal relative ENU
        goal_geo = self.transforms.enu_to_geodetic(
            float(self.goal_rel_enu[0]),
            float(self.goal_rel_enu[1]),
            float(self.goal_rel_enu[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )
        self.goal_lat, self.goal_lon, self.goal_alt = float(goal_geo[0]), float(goal_geo[1]), float(goal_geo[2])

        # Initial geodetic (consistent with rel_pos_enu & origin)
        init_geo = self.transforms.enu_to_geodetic(
            float(self.rel_pos_enu[0]),
            float(self.rel_pos_enu[1]),
            float(self.rel_pos_enu[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )
        self.cur_lat, self.cur_lon, self.cur_alt = float(init_geo[0]), float(init_geo[1]), float(init_geo[2])

        # Last course cache for low-speed
        self.last_course_deg: float = 0.0

        # Action from AI (default zeros until messages arrive)
        self.last_action = np.zeros(4, dtype=np.float32)  # [vx, vy, vz, speed]

        # Distance tracking for logging
        self.iteration_count = 0
        self.total_distance_traveled = 0.0
        self.goal_reached = False
        self.goal_tolerance = 0.20  # meters - consider goal reached within 20cm

        # ─────────────────────────
        # QoS and I/O
        # ─────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers (fake sensors)
        self.pub_gps_fix = self.create_publisher(NavSatFix, '/fc/gps_fix', sensor_qos)
        self.pub_gps_spd = self.create_publisher(Float32MultiArray, '/fc/gps_speed_course', sensor_qos)
        self.pub_att_q = self.create_publisher(QuaternionStamped, '/fc/attitude', sensor_qos)
        self.pub_att_e = self.create_publisher(Vector3Stamped, '/fc/attitude_euler', sensor_qos)
        self.pub_imu = self.create_publisher(Imu, '/fc/imu_raw', sensor_qos)
        self.pub_lidar = self.create_publisher(Range, '/lidar_distance', sensor_qos)
        self.pub_wp = self.create_publisher(Float32MultiArray, '/fc/waypoint', reliable_qos)

        # Publishers (observation)
        self.pub_obs = self.create_publisher(Float32MultiArray, '/ai/observation', reliable_qos)
        self.pub_debug = self.create_publisher(Float32MultiArray, '/ai/observation_debug', reliable_qos)

        # Subscriber to AI actions
        self.sub_action = self.create_subscription(Float32MultiArray, '/ai/action', self.cb_action, reliable_qos)

        # Timers
        self.physics_dt = 1.0 / max(1.0, physics_rate)
        self.telemetry_dt = 1.0 / max(1.0, telemetry_rate)
        self.create_timer(self.physics_dt, self._physics_tick)
        self.create_timer(self.telemetry_dt, self._telemetry_tick)
        self.create_timer(1.0, self._publish_waypoint_once_per_sec)

        # Print banner
        self.debug_logger.print_initial_banner(
            telemetry_rate=telemetry_rate,
            action_buffer_size=self.obs_builder.action_buffer_size,
            max_ray_distance=self.lidar_max,
            relative_start_enu=self.relative_start_enu,
        )
        self.get_logger().info(
            f"Sim origin=({self.origin[0]:.7f}, {self.origin[1]:.7f}, {self.origin[2]:.2f}) | "
            f"start_relENU=({self.rel_pos_enu[0]:.2f},{self.rel_pos_enu[1]:.2f},{self.rel_pos_enu[2]:.2f}) | "
            f"goal_relENU=({self.goal_rel_enu[0]:.2f},{self.goal_rel_enu[1]:.2f},{self.goal_rel_enu[2]:.2f})"
        )

        # Mark initial data received in the manager (we immediately have synthetic data)
        self.sensor_manager.data_received.update({
            'gps': True, 'att_quat': True, 'att_euler': True, 'imu': True,
            'lidar': True, 'gps_speed': True, 'waypoint': True, 'action': True
        })

    # ────────────────────────────────────────────────────────────────
    # AI action
    # ────────────────────────────────────────────────────────────────
    def cb_action(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        arr = np.asarray(msg.data[:4], dtype=np.float32).reshape(-1)
        self.last_action = arr
        # Also feed the ObservationBuilder's action buffer (matches real pipeline)
        self.obs_builder.update_action(arr)

    # ────────────────────────────────────────────────────────────────
    # Physics tick (move the drone)
    # ────────────────────────────────────────────────────────────────
    def _physics_tick(self):
        vx, vy, vz, spd = [float(x) for x in self.last_action]
        if not self.use_speed_scalar:
            spd = 1.0  # ignore speed head if disabled
        else:
            # CRITICAL: Speed scalar should be non-negative. The model may output
            # negative values during training artifacts, but we interpret it as magnitude.
            spd = abs(spd)

        # ═══════════════════════════════════════════════════════════════════
        # CRITICAL: Body->ENU coordinate transformation
        # ═══════════════════════════════════════════════════════════════════
        # The AI model was trained in PyBullet using Body Frame coordinates:
        #   vx = forward/backward (along drone's nose, Body-X axis)
        #   vy = right/left (perpendicular to nose, Body-Y axis)
        #   vz = up/down (Body-Z axis)
        #
        # PyBullet's Body Frame convention (when yaw=0):
        #   Body-X (forward) → ENU-North (NOT East!)
        #   Body-Y (right)   → ENU-East  (NOT North!)
        #   Body-Z (up)      → ENU-Up
        #
        # For arbitrary yaw angle, the transformation is:
        #   E = vx * sin(yaw) + vy * cos(yaw)
        #   N = vx * cos(yaw) - vy * sin(yaw)
        #   U = vz
        #
        # When yaw=0: cos(0)=1, sin(0)=0, so:
        #   E = vy  (right → East)
        #   N = vx  (forward → North)
        #   U = vz
        # ═══════════════════════════════════════════════════════════════════

        # Apply body-to-ENU rotation
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        # Transform body velocities to ENU frame
        v_east = vx * sin_yaw + vy * cos_yaw
        v_north = vx * cos_yaw - vy * sin_yaw
        v_up = vz

        # Scale by meters_per_step and speed
        step_scale = float(self.meters_per_step * spd)
        dE = v_east * step_scale
        dN = v_north * step_scale
        dU = v_up * step_scale

        # Store old position for distance calculation
        old_pos = self.rel_pos_enu.copy()

        self.rel_pos_enu[0] += dE
        self.rel_pos_enu[1] += dN
        self.rel_pos_enu[2] += dU

        # Compute "instantaneous" ENU velocity (for GPS speed/course publication)
        self.vel_enu[0] = dE / self.physics_dt
        self.vel_enu[1] = dN / self.physics_dt
        self.vel_enu[2] = dU / self.physics_dt

        # Clamp altitude to >= 0 for sanity (ground at U=0)
        if self.rel_pos_enu[2] < 0.0:
            self.rel_pos_enu[2] = 0.0
            self.vel_enu[2] = 0.0

        # Update geodetic from rel ENU
        geo = self.transforms.enu_to_geodetic(
            float(self.rel_pos_enu[0]),
            float(self.rel_pos_enu[1]),
            float(self.rel_pos_enu[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )
        self.cur_lat, self.cur_lon, self.cur_alt = float(geo[0]), float(geo[1]), float(geo[2])

        # Calculate distances for logging
        step_distance = np.linalg.norm(self.rel_pos_enu - old_pos)
        self.total_distance_traveled += step_distance

        distance_to_goal = np.linalg.norm(self.goal_rel_enu - self.rel_pos_enu)
        distance_from_start = np.linalg.norm(self.rel_pos_enu - self.relative_start_enu)

        # Log progress every iteration
        self.iteration_count += 1
        self.get_logger().info(
            f'🚁 Iter {self.iteration_count:04d} | '
            f'Pos=[{self.rel_pos_enu[0]:6.2f}, {self.rel_pos_enu[1]:6.2f}, {self.rel_pos_enu[2]:6.2f}] | '
            f'→Goal={distance_to_goal:6.2f}m | '
            f'Traveled={self.total_distance_traveled:6.2f}m | '
            f'Action=[{vx:5.2f}, {vy:5.2f}, {vz:5.2f}, spd={spd:4.2f}]'
        )

        # Check if goal reached
        if distance_to_goal <= self.goal_tolerance and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info('=' * 80)
            self.get_logger().info('🎯 GOAL REACHED!')
            self.get_logger().info('=' * 80)
            self.get_logger().info(f'   • Final position: [{self.rel_pos_enu[0]:.2f}, {self.rel_pos_enu[1]:.2f}, {self.rel_pos_enu[2]:.2f}]')
            self.get_logger().info(f'   • Target position: [{self.goal_rel_enu[0]:.2f}, {self.goal_rel_enu[1]:.2f}, {self.goal_rel_enu[2]:.2f}]')
            self.get_logger().info(f'   • Final distance to goal: {distance_to_goal:.3f}m')
            self.get_logger().info(f'   • Total iterations: {self.iteration_count}')
            self.get_logger().info(f'   • Total distance traveled: {self.total_distance_traveled:.2f}m')

            # Calculate efficiency (straight line distance vs actual traveled)
            straight_line_dist = np.linalg.norm(self.goal_rel_enu - self.relative_start_enu)
            efficiency = (straight_line_dist / self.total_distance_traveled * 100) if self.total_distance_traveled > 0 else 0
            self.get_logger().info(f'   • Straight line distance: {straight_line_dist:.2f}m')
            self.get_logger().info(f'   • Path efficiency: {efficiency:.1f}%')
            self.get_logger().info('=' * 80)
            self.get_logger().info('✅ Simulation complete. Shutting down in 2 seconds...')
            self.get_logger().info('=' * 80)

            # Schedule shutdown
            import threading
            def delayed_shutdown():
                import time
                time.sleep(2.0)
                self.get_logger().info('Shutting down simulator node...')
                import os
                os._exit(0)

            shutdown_thread = threading.Thread(target=delayed_shutdown)
            shutdown_thread.daemon = True
            shutdown_thread.start()

    # ────────────────────────────────────────────────────────────────
    # Telemetry tick (publish sensors + build observation)
    # ────────────────────────────────────────────────────────────────
    def _telemetry_tick(self):
        # Sensor: attitude (yaw fixed), IMU (zero rates)
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        # Sensor: GPS speed/course from ENU vel
        speed = float(math.hypot(self.vel_enu[0], self.vel_enu[1]))
        if speed > 1e-6:
            # Course: 0 deg = North, increases clockwise; atan2(vE, vN).
            course_deg = (math.degrees(math.atan2(self.vel_enu[0], self.vel_enu[1])) + 360.0) % 360.0
            self.last_course_deg = course_deg
        else:
            course_deg = self.last_course_deg

        # ═══════════════════════════════════════════════════════════════════════
        # CRITICAL: Simulate all 16 LiDAR rays (not just down ray)
        # ═══════════════════════════════════════════════════════════════════════
        # The model was trained with 16 rays providing 3D obstacle detection.
        # Ray configuration (body frame, from moving_drone.py:116-138):
        #   0: Up [0,0,1]           1: Down [0,0,-1]
        #   2: Forward [1,0,0]      3: Forward-Right [√2/2,√2/2,0]
        #   4: Right [0,1,0]        5: Back-Right [-√2/2,√2/2,0]
        #   6: Back [-1,0,0]        7: Back-Left [-√2/2,-√2/2,0]
        #   8: Left [0,-1,0]        9: Forward-Left [√2/2,-√2/2,0]
        #   10-15: Diagonal rays at ±30° elevation
        #
        # For this simple simulator (no obstacles), we set:
        #   - Down ray (1): actual altitude (distance to ground)
        #   - Up ray (0): large distance (sky is open)
        #   - Horizontal rays (2-9): large distance (open space)
        #   - Diagonal rays (10-15): calculated based on direction and altitude
        # ═══════════════════════════════════════════════════════════════════════

        altitude = float(self.rel_pos_enu[2])  # meters above ground

        # Ray 0: Up - open sky
        self.sensor_manager.update_lidar_ray(0, 1.0)

        # Ray 1: Down - distance to ground
        down_dist = max(self.lidar_min, min(self.lidar_max, altitude))
        norm_down = self.obs_builder.normalize_lidar_distance(down_dist)
        self.sensor_manager.update_lidar_ray(1, norm_down)

        # Rays 2-9: Horizontal rays - open space (no obstacles)
        for ray_idx in range(2, 10):
            self.sensor_manager.update_lidar_ray(ray_idx, 1.0)

        # Rays 10-15: Diagonal rays at ±30° elevation
        # These rays point up/down at 30°, so they may hit ground or be open
        sin_30 = 0.5  # sin(30°)
        cos_30 = 0.866025  # cos(30°)

        # Ray 10: Forward-Up (30°) - open sky
        self.sensor_manager.update_lidar_ray(10, 1.0)

        # Ray 11: Forward-Down (-30°) - may hit ground
        # Distance to ground along this ray: altitude / sin(30°)
        dist_11 = altitude / sin_30 if altitude > 0 else self.lidar_max
        dist_11 = min(self.lidar_max, dist_11)
        norm_11 = self.obs_builder.normalize_lidar_distance(dist_11)
        self.sensor_manager.update_lidar_ray(11, norm_11)

        # Ray 12: Back-Up (30°) - open sky
        self.sensor_manager.update_lidar_ray(12, 1.0)

        # Ray 13: Back-Down (-30°) - may hit ground
        dist_13 = altitude / sin_30 if altitude > 0 else self.lidar_max
        dist_13 = min(self.lidar_max, dist_13)
        norm_13 = self.obs_builder.normalize_lidar_distance(dist_13)
        self.sensor_manager.update_lidar_ray(13, norm_13)

        # Ray 14: Right-Up (30°) - open sky
        self.sensor_manager.update_lidar_ray(14, 1.0)

        # Ray 15: Left-Up (30°) - open sky
        self.sensor_manager.update_lidar_ray(15, 1.0)

        # Feed SensorDataManager with the latest truths (exactly like real callbacks)
        # GPS position
        first_fix = self.sensor_manager.update_gps_position(self.cur_lat, self.cur_lon, self.cur_alt)
        if first_fix:
            # In theory we already set origin; this path won't trigger because we marked gps received True.
            pass

        # GPS velocity
        vel_enu_from_course = self.transforms.velocity_cog_to_enu(speed, course_deg)
        self.sensor_manager.update_velocity_from_gps(speed, course_deg, vel_enu_from_course)

        # Attitude
        self.sensor_manager.update_attitude_quaternion(qx, qy, qz, qw)
        self.sensor_manager.update_attitude_euler(roll=0.0, pitch=0.0, yaw=self.yaw)

        # IMU angular velocity (zero)
        self.sensor_manager.update_angular_velocity(0.0, 0.0, 0.0)

        # Goal (in geodetic)
        self.sensor_manager.update_goal(self.goal_lat, self.goal_lon, self.goal_alt)

        # ── Publish fake sensor topics (optional, nice for dashboards) ──
        # GPS fix
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.gps_frame
        fix.latitude = self.cur_lat
        fix.longitude = self.cur_lon
        fix.altitude = self.cur_alt
        self.pub_gps_fix.publish(fix)

        # GPS speed/course
        spdmsg = Float32MultiArray()
        spdmsg.data = [float(speed), float(course_deg)]
        self.pub_gps_spd.publish(spdmsg)

        # Attitude quaternion
        qs = QuaternionStamped()
        qs.header.stamp = fix.header.stamp
        qs.header.frame_id = self.att_frame
        qs.quaternion.x = qx
        qs.quaternion.y = qy
        qs.quaternion.z = qz
        qs.quaternion.w = qw
        self.pub_att_q.publish(qs)

        # Attitude Euler
        ve = Vector3Stamped()
        ve.header = qs.header
        ve.vector.x = 0.0  # roll
        ve.vector.y = 0.0  # pitch
        ve.vector.z = self.yaw
        self.pub_att_e.publish(ve)

        # IMU (angular velocity only for this sim)
        imu = Imu()
        imu.header = qs.header
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0
        self.pub_imu.publish(imu)

        # LiDAR down
        rng = Range()
        rng.header = qs.header
        rng.header.frame_id = self.lidar_frame
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = self.lidar_fov
        rng.min_range = self.lidar_min
        rng.max_range = self.lidar_max
        rng.range = down_dist
        self.pub_lidar.publish(rng)

        # ── Build and publish the 131-D observation (exactly like your AIAdapterNode) ──
        last_action = self.obs_builder.prepare_action_for_observation()

        # rel_pos_enu from current geodetic (to mirror real pipeline)
        rel_enu = self.transforms.geodetic_to_enu(
            self.cur_lat, self.cur_lon, self.cur_alt,
            float(self.origin[0]), float(self.origin[1]), float(self.origin[2])
        )

        goal_enu = self.transforms.geodetic_to_enu(
            self.goal_lat, self.goal_lon, self.goal_alt,
            float(self.origin[0]), float(self.origin[1]), float(self.origin[2])
        )
        goal_vec = self.obs_builder.compute_goal_vector(goal_enu, rel_enu)

        full_obs = self.obs_builder.build_full_observation(
            rel_pos_enu=rel_enu,
            quat_att=self.sensor_manager.get_quaternion(),
            euler_att=self.sensor_manager.get_euler(),
            velocity=self.sensor_manager.get_velocity(),
            angular_velocity=self.sensor_manager.get_angular_velocity(),
            last_action=last_action,
            lidar_distances=self.sensor_manager.get_lidar_distances(),
            goal_vector=goal_vec
        )

        # Publish obs
        obs_msg = Float32MultiArray()
        obs_msg.data = full_obs.tolist()
        self.pub_obs.publish(obs_msg)

        # Publish a small debug vector [E, N, U, yaw, down_lidar, used_action_flag]
        used_flag = 1.0 if self.obs_builder.is_using_action(last_action) else 0.0
        dbg = Float32MultiArray()
        dbg.data = [
            float(rel_enu[0]), float(rel_enu[1]), float(rel_enu[2]),
            float(self.sensor_manager.get_euler()[2]),
            float(norm_down),
            used_flag
        ]
        self.pub_debug.publish(dbg)

    # ────────────────────────────────────────────────────────────────
    # Waypoint publisher (periodic, like your FC node does)
    # ────────────────────────────────────────────────────────────────
    def _publish_waypoint_once_per_sec(self):
        wp = Float32MultiArray()
        # [wp_no, lat, lon, alt]
        wp.data = [1.0, float(self.goal_lat), float(self.goal_lon), float(self.goal_alt)]
        self.pub_wp.publish(wp)

    # ────────────────────────────────────────────────────────────────
    # Shutdown
    # ────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.debug_logger.print_shutdown_stats(
            obs_count=-1,  # not tracked here
            action_count=self.obs_builder.get_action_count(),
            action_buffer_size=self.obs_builder.action_buffer_size,
            data_status=self.sensor_manager.get_data_status()
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AIAdapterSimulatedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
