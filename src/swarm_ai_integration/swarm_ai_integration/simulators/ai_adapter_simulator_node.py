#!/usr/bin/env python3
"""
AI Adapter Simulated Node
Deterministic, no-noise, perfect velocity tracking harness for AIFlightNode.

Loop:
    /ai/observation  --> AIFlightNode -->  /ai/action [vx, vy, vz, speed]
                                        ^                      |
                                        |______ this node _____|

Behavior (idealized):
- Yaw fixed at 0 rad (body frame aligned with ENU when yaw=0).
- Each physics tick (default 10 Hz), position += meters_per_step * speed * [vx, vy, vz].
- No actuator lag, no noise, no dropouts.
- Publishes realistic sensor topics and constructs the exact 131-D observation
  via your real ObservationBuilder/CoordinateTransforms/SensorDataManager.

NOTE: Model actions are interpreted as direct ENU displacements (vx→East, vy→North, vz→Up).
Though trained in body-frame, the model appears to have learned ENU-aligned outputs.

Do NOT run fc_comms_node at the same time (it publishes the same /fc/* topics).
"""

import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix, Imu, Range
from geometry_msgs.msg import Vector3Stamped

from swarm_ai_integration.utils import (
    CoordinateTransforms,
    SensorDataManager,
    ObservationBuilder,
    DebugLogger,
)


class AIAdapterSimulatedNode(Node):
    def __init__(self):
        super().__init__('ai_adapter_simulated_node')

        # ─────────────────────────
        # Parameters
        # ─────────────────────────
        self.declare_parameter('telemetry_rate', 30.0)             # Hz for publishing observation
        self.declare_parameter('physics_rate', 10.0)               # Hz for state integration (training used 10-30 Hz loop)
        self.declare_parameter('meters_per_step', 0.15)            # scale of one "speed=1.0" step
        self.declare_parameter('use_speed_scalar', True)           # scale direction by |speed|

        # Start geodetic
        self.declare_parameter('start_lat', 37.2486500)
        self.declare_parameter('start_lon', -5.3724274)
        self.declare_parameter('start_alt', 182.0)

        # Desired initial REL ENU
        self.declare_parameter('relative_start_enu', [0.0, 0.0, 3.0])  # meters

        # Goal as REL ENU
        self.declare_parameter('goal_relative_enu', [5.0, 5.0, 3.0])   # E,N,U meters

        # Lidar model
        self.declare_parameter('lidar_min_range', 0.05)
        self.declare_parameter('lidar_max_range', 20.0)
        self.declare_parameter('lidar_fov', 0.035)  # rad
        self.declare_parameter('lidar_frame', 'lidar_link')

        # Frames
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
        # Core modules
        # ─────────────────────────
        self.transforms = CoordinateTransforms()
        # IMPORTANT: action buffer length in the observation should match training.
        # The model was trained with 131-D observations, which requires buffer_size=25.
        # Formula: 12 (kinematics) + 25*4 (actions) + 16 (lidar) + 3 (goal) = 131
        self.obs_builder = ObservationBuilder(action_buffer_size=25, max_ray_distance=self.lidar_max)
        self.sensor_manager = SensorDataManager(num_lidar_rays=16, relative_start_enu=self.relative_start_enu)
        self.debug_logger = DebugLogger(node=self, max_ray_distance=self.lidar_max)

        # ─────────────────────────
        # World/state (ENU relative to origin)
        # ─────────────────────────
        self.origin = self.transforms.compute_origin_for_initial_position(
            current_lat=self.start_lat,
            current_lon=self.start_lon,
            current_alt=self.start_alt,
            desired_enu=self.relative_start_enu,
        )
        self.sensor_manager.set_origin_geodetic(self.origin)

        self.rel_pos_enu = self.relative_start_enu.astype(np.float32).copy()
        self.yaw = 0.0  # rad (fixed)
        self.vel_enu = np.zeros(3, dtype=np.float32)

        # Goal geodetic
        goal_geo = self.transforms.enu_to_geodetic(
            float(self.goal_rel_enu[0]),
            float(self.goal_rel_enu[1]),
            float(self.goal_rel_enu[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )
        self.goal_lat, self.goal_lon, self.goal_alt = float(goal_geo[0]), float(goal_geo[1]), float(goal_geo[2])

        # Initial geodetic
        init_geo = self.transforms.enu_to_geodetic(
            float(self.rel_pos_enu[0]),
            float(self.rel_pos_enu[1]),
            float(self.rel_pos_enu[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )
        self.cur_lat, self.cur_lon, self.cur_alt = float(init_geo[0]), float(init_geo[1]), float(init_geo[2])

        self.last_course_deg: float = 0.0
        self.last_action = np.zeros(4, dtype=np.float32)  # [vx, vy, vz, speed]

        self.iteration_count = 0
        self.total_distance_traveled = 0.0
        self.goal_reached = False
        self.goal_tolerance = 0.50  # m (success threshold)

        # QoS and I/O
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

        # Banner
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

        self.sensor_manager.data_received.update({
            'gps': True, 'att_euler': True, 'imu': True,
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
        self.obs_builder.update_action(arr)

    # ────────────────────────────────────────────────────────────────
    # Physics tick (move the drone)
    # ────────────────────────────────────────────────────────────────
    def _physics_tick(self):
        vx, vy, vz, spd = [float(x) for x in self.last_action]

        # Treat speed as magnitude (not direction)
        if not self.use_speed_scalar:
            spd = 1.0
        else:
            # CRITICAL: Speed scalar should be non-negative. The model may output
            # negative values during training artifacts, but we interpret it as magnitude.
            spd = abs(spd)

        # Direct mapping: model outputs appear to be in ENU-like frame already
        # (despite body-frame training, the model may have learned ENU outputs)
        step_scale = float(self.meters_per_step * spd)
        dE = vx * step_scale
        dN = vy * step_scale
        dU = vz * step_scale

        old_pos = self.rel_pos_enu.copy()
        self.rel_pos_enu[0] += dE
        self.rel_pos_enu[1] += dN
        self.rel_pos_enu[2] = max(0.0, self.rel_pos_enu[2] + dU)  # ground clamp

        # "Instantaneous" velocity for telemetry
        self.vel_enu[0] = dE / self.physics_dt
        self.vel_enu[1] = dN / self.physics_dt
        self.vel_enu[2] = (self.rel_pos_enu[2] - old_pos[2]) / self.physics_dt

        # Update geodetic
        geo = self.transforms.enu_to_geodetic(
            float(self.rel_pos_enu[0]),
            float(self.rel_pos_enu[1]),
            float(self.rel_pos_enu[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )
        self.cur_lat, self.cur_lon, self.cur_alt = float(geo[0]), float(geo[1]), float(geo[2])

        # Logging
        self.iteration_count += 1
        step_distance = np.linalg.norm(self.rel_pos_enu - old_pos)
        self.total_distance_traveled += step_distance
        distance_to_goal = np.linalg.norm(self.goal_rel_enu - self.rel_pos_enu)

        self.get_logger().info(
            f'🚁 Iter {self.iteration_count:04d} | '
            f'Pos=[{self.rel_pos_enu[0]:6.2f}, {self.rel_pos_enu[1]:6.2f}, {self.rel_pos_enu[2]:6.2f}] | '
            f'→Goal={distance_to_goal:6.2f}m | '
            f'Traveled={self.total_distance_traveled:6.2f}m | '
            f'Action=[{vx:5.2f}, {vy:5.2f}, {vz:5.2f}, spd={abs(spd):4.2f}]'
        )

        if not self.goal_reached and distance_to_goal <= self.goal_tolerance:
            self.goal_reached = True
            self._on_goal(distance_to_goal)

    def _on_goal(self, distance_to_goal: float):
        self.get_logger().info('=' * 80)
        self.get_logger().info('🎯 GOAL REACHED!')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'   • Final position: [{self.rel_pos_enu[0]:.2f}, {self.rel_pos_enu[1]:.2f}, {self.rel_pos_enu[2]:.2f}]')
        self.get_logger().info(f'   • Target position: [{self.goal_rel_enu[0]:.2f}, {self.goal_rel_enu[1]:.2f}, {self.goal_rel_enu[2]:.2f}]')
        self.get_logger().info(f'   • Final distance to goal: {distance_to_goal:.3f}m')
        self.get_logger().info(f'   • Total iterations: {self.iteration_count}')
        self.get_logger().info(f'   • Total distance traveled: {self.total_distance_traveled:.2f}m')

        straight_line = float(np.linalg.norm(self.goal_rel_enu - self.relative_start_enu))
        efficiency = (straight_line / self.total_distance_traveled * 100.0) if self.total_distance_traveled > 0 else 0.0
        self.get_logger().info(f'   • Straight line distance: {straight_line:.2f}m')
        self.get_logger().info(f'   • Path efficiency: {efficiency:.1f}%')
        self.get_logger().info('=' * 80)
        self.get_logger().info('✅ Simulation complete. Shutting down in 2 seconds...')
        self.get_logger().info('=' * 80)

        import threading, time as _t, os as _os
        def delayed_shutdown():
            _t.sleep(2.0)
            self.get_logger().info('Shutting down simulator node...')
            _os._exit(0)
        t = threading.Thread(target=delayed_shutdown, daemon=True)
        t.start()

    # ────────────────────────────────────────────────────────────────
    # Telemetry tick (publish sensors + build observation)
    # ────────────────────────────────────────────────────────────────
    def _telemetry_tick(self):
        # GPS speed/course from ENU velocity
        speed_xy = float(math.hypot(self.vel_enu[0], self.vel_enu[1]))
        if speed_xy > 1e-6:
            # Course: 0 deg = North, CW; atan2(vE, vN)
            course_deg = (math.degrees(math.atan2(self.vel_enu[0], self.vel_enu[1])) + 360.0) % 360.0
            self.last_course_deg = course_deg
        else:
            course_deg = self.last_course_deg

        # LiDAR rays (open space; down ray = altitude)
        altitude = float(self.rel_pos_enu[2])
        self.sensor_manager.update_lidar_ray(0, 1.0)  # Up = open
        down_dist = max(self.lidar_min, min(self.lidar_max, altitude))
        norm_down = self.obs_builder.normalize_lidar_distance(down_dist)
        self.sensor_manager.update_lidar_ray(1, norm_down)
        for ray_idx in range(2, 10):  # horizontals
            self.sensor_manager.update_lidar_ray(ray_idx, 1.0)
        sin_30 = 0.5
        # forward-up/open
        self.sensor_manager.update_lidar_ray(10, 1.0)
        # forward-down ground hit distance
        dist_11 = min(self.lidar_max, altitude / sin_30 if altitude > 0 else self.lidar_max)
        self.sensor_manager.update_lidar_ray(11, self.obs_builder.normalize_lidar_distance(dist_11))
        # back-up/open
        self.sensor_manager.update_lidar_ray(12, 1.0)
        # back-down
        dist_13 = min(self.lidar_max, altitude / sin_30 if altitude > 0 else self.lidar_max)
        self.sensor_manager.update_lidar_ray(13, self.obs_builder.normalize_lidar_distance(dist_13))
        # right-down (tilted 30° down, hits ground)
        dist_14 = min(self.lidar_max, altitude / sin_30 if altitude > 0 else self.lidar_max)
        self.sensor_manager.update_lidar_ray(14, self.obs_builder.normalize_lidar_distance(dist_14))
        # left-down (tilted 30° down, hits ground)
        dist_15 = min(self.lidar_max, altitude / sin_30 if altitude > 0 else self.lidar_max)
        self.sensor_manager.update_lidar_ray(15, self.obs_builder.normalize_lidar_distance(dist_15))

        # Update sensor manager (same as real callbacks)
        first_fix = self.sensor_manager.update_gps_position(self.cur_lat, self.cur_lon, self.cur_alt)
        vel_enu_from_course = self.transforms.velocity_cog_to_enu(speed_xy, course_deg)
        self.sensor_manager.update_velocity_from_gps(speed_xy, course_deg, vel_enu_from_course)
        self.sensor_manager.update_attitude_euler(roll=0.0, pitch=0.0, yaw=self.yaw)
        self.sensor_manager.update_angular_velocity(0.0, 0.0, 0.0)
        self.sensor_manager.update_goal(self.goal_lat, self.goal_lon, self.goal_alt)

        # Publish fake sensors
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.gps_frame
        fix.latitude = self.cur_lat
        fix.longitude = self.cur_lon
        fix.altitude = self.cur_alt
        self.pub_gps_fix.publish(fix)

        spdmsg = Float32MultiArray()
        spdmsg.data = [float(speed_xy), float(course_deg)]
        self.pub_gps_spd.publish(spdmsg)

        ve = Vector3Stamped()
        ve.header.stamp = fix.header.stamp
        ve.header.frame_id = self.att_frame
        ve.vector.x = 0.0
        ve.vector.y = 0.0
        ve.vector.z = self.yaw
        self.pub_att_e.publish(ve)

        imu = Imu()
        imu.header = ve.header
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0
        self.pub_imu.publish(imu)

        rng = Range()
        rng.header = fix.header
        rng.header.frame_id = self.lidar_frame
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = self.lidar_fov
        rng.min_range = self.lidar_min
        rng.max_range = self.lidar_max
        rng.range = down_dist
        self.pub_lidar.publish(rng)

        # Build and publish 131-D observation (match training layout)
        last_action = self.obs_builder.prepare_action_for_observation()

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
            euler_att=self.sensor_manager.get_euler(),
            velocity=self.sensor_manager.get_velocity(),
            angular_velocity=self.sensor_manager.get_angular_velocity(),
            lidar_distances=self.sensor_manager.get_lidar_distances(),
            goal_vector=goal_vec
        )

        obs_msg = Float32MultiArray()
        obs_msg.data = full_obs.tolist()
        self.pub_obs.publish(obs_msg)

        used_flag = 1.0 if self.obs_builder.is_using_action(last_action) else 0.0
        dbg = Float32MultiArray()
        dbg.data = [float(rel_enu[0]), float(rel_enu[1]), float(rel_enu[2]), float(self.yaw), float(norm_down), used_flag]
        self.pub_debug.publish(dbg)

    # ────────────────────────────────────────────────────────────────
    def _publish_waypoint_once_per_sec(self):
        wp = Float32MultiArray()
        wp.data = [1.0, float(self.goal_lat), float(self.goal_lon), float(self.goal_alt)]
        self.pub_wp.publish(wp)

    def destroy_node(self):
        self.debug_logger.print_shutdown_stats(
            obs_count=-1,
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
