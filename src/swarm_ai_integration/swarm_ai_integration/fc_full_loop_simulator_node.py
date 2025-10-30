#!/usr/bin/env python3
"""
FC Full Loop Simulator Node

This node simulates the complete control loop including fc_adapter_node:
    AI Adapter → AI Flight → FC Adapter → [THIS NODE] → Sensor feedback → loop

Flow:
1. This node publishes fake sensor data (/fc/gps_fix, /fc/attitude_euler, etc.)
2. AI adapter processes sensors → /ai/observation
3. AI flight generates actions → /ai/action
4. FC adapter converts to RC commands → /fc/rc_override
5. This node receives RC commands, simulates INAV response, updates drone state
6. Loop back to step 1

Key features:
- Simulates INAV ALT HOLD mode (throttle RC → climb rate)
- Motor dynamics (not instant response)
- GPS noise and update rate
- Barometer drift
- Detailed logging to CSV file

This reveals the REAL behavior including PID interactions and mode effects.
"""

import math
import time
import numpy as np
from typing import Optional
from pathlib import Path
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Int32, Float32
from sensor_msgs.msg import NavSatFix, Imu, Range
from geometry_msgs.msg import Vector3Stamped

from swarm_ai_integration.utils import (
    CoordinateTransforms,
    SensorDataManager,
)


class FCFullLoopSimulatorNode(Node):
    """
    Full closed-loop simulator including fc_adapter PID control.

    Simulates:
    - INAV flight controller with ALT HOLD mode
    - Motor dynamics and thrust response
    - GPS noise and update rate (1 Hz)
    - Barometer altitude with drift
    - Realistic sensor delays
    """

    def __init__(self):
        super().__init__('fc_full_loop_simulator_node')

        # ─────────────────────────
        # Parameters
        # ─────────────────────────
        self.declare_parameter('physics_rate', 100.0)              # Hz for physics integration
        self.declare_parameter('gps_publish_rate', 1.5)            # Hz (realistic GPS rate)
        self.declare_parameter('imu_publish_rate', 50.0)           # Hz
        self.declare_parameter('sensor_publish_rate', 10.0)        # Hz for attitude, speed
        self.declare_parameter('lidar_publish_rate', 100.0)        # Hz

        # Start geodetic
        self.declare_parameter('start_lat', 37.2486500)
        self.declare_parameter('start_lon', -5.3724274)
        self.declare_parameter('start_alt', 182.0)

        # Desired initial REL ENU
        self.declare_parameter('relative_start_enu', [0.0, 0.0, 3.0])

        # Goal as REL ENU
        self.declare_parameter('goal_relative_enu', [5.0, 5.0, 3.0])

        # INAV simulation
        self.declare_parameter('enable_althold_simulation', True)  # Simulate INAV ALT HOLD
        self.declare_parameter('althold_kp', 50.0)                 # INAV's ALT HOLD P gain
        self.declare_parameter('max_climb_rate', 3.0)              # m/s (INAV limit)

        # Physics
        self.declare_parameter('motor_time_constant', 0.1)         # seconds (motor lag)
        self.declare_parameter('drag_coefficient', 0.5)            # air resistance
        self.declare_parameter('mass_kg', 1.5)                     # drone mass

        # Noise
        self.declare_parameter('gps_noise_std', 2.0)               # meters
        self.declare_parameter('baro_drift_rate', 0.01)            # m/s drift
        self.declare_parameter('velocity_noise_std', 0.1)          # m/s

        # LiDAR
        self.declare_parameter('lidar_min_range', 0.05)
        self.declare_parameter('lidar_max_range', 20.0)
        self.declare_parameter('lidar_fov', 0.035)
        self.declare_parameter('lidar_frame', 'lidar_link')

        # Frames
        self.declare_parameter('gps_frame', 'fc_gps')
        self.declare_parameter('att_frame', 'fc_attitude')
        self.declare_parameter('imu_frame', 'fc_imu')

        # Logging
        self.declare_parameter('log_directory', '~/swarm-ros/flight-logs/simulator')
        self.declare_parameter('enable_logging', True)

        # Goal
        self.declare_parameter('goal_tolerance', 0.5)              # meters

        # Read params
        physics_rate = float(self.get_parameter('physics_rate').value)
        gps_rate = float(self.get_parameter('gps_publish_rate').value)
        imu_rate = float(self.get_parameter('imu_publish_rate').value)
        sensor_rate = float(self.get_parameter('sensor_publish_rate').value)
        lidar_rate = float(self.get_parameter('lidar_publish_rate').value)

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

        self.althold_enabled = bool(self.get_parameter('enable_althold_simulation').value)
        self.althold_kp = float(self.get_parameter('althold_kp').value)
        self.max_climb_rate = float(self.get_parameter('max_climb_rate').value)

        self.motor_tau = float(self.get_parameter('motor_time_constant').value)
        self.drag_coeff = float(self.get_parameter('drag_coefficient').value)
        self.mass = float(self.get_parameter('mass_kg').value)

        self.gps_noise = float(self.get_parameter('gps_noise_std').value)
        self.baro_drift_rate = float(self.get_parameter('baro_drift_rate').value)
        self.vel_noise = float(self.get_parameter('velocity_noise_std').value)

        self.lidar_min = float(self.get_parameter('lidar_min_range').value)
        self.lidar_max = float(self.get_parameter('lidar_max_range').value)
        self.lidar_fov = float(self.get_parameter('lidar_fov').value)
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)

        self.gps_frame = str(self.get_parameter('gps_frame').value)
        self.att_frame = str(self.get_parameter('att_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)

        log_dir = str(self.get_parameter('log_directory').value)
        self.enable_logging = bool(self.get_parameter('enable_logging').value)

        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # ─────────────────────────
        # Core modules
        # ─────────────────────────
        self.transforms = CoordinateTransforms()

        # ─────────────────────────
        # World state
        # ─────────────────────────
        self.origin = self.transforms.compute_origin_for_initial_position(
            current_lat=self.start_lat,
            current_lon=self.start_lon,
            current_alt=self.start_alt,
            desired_enu=self.relative_start_enu,
        )

        # True state (no noise)
        self.pos_enu = self.relative_start_enu.astype(np.float64).copy()
        self.vel_enu = np.zeros(3, dtype=np.float64)  # [vx_e, vy_n, vz_u] m/s
        self.acc_enu = np.zeros(3, dtype=np.float64)  # m/s²

        # Attitude (body frame)
        self.roll = 0.0   # rad
        self.pitch = 0.0  # rad
        self.yaw = 0.0    # rad (will rotate in flight)
        self.ang_vel = np.zeros(3, dtype=np.float64)  # [wx, wy, wz] rad/s

        # Motor state (simulates lag)
        self.motor_thrust = 0.0     # Current thrust (0-1)
        self.target_thrust = 0.0    # Target thrust from RC

        # Barometer state (drifts over time)
        self.baro_altitude = float(self.pos_enu[2])
        self.baro_drift = 0.0

        # RC inputs (from fc_adapter)
        self.rc_channels = [1500] * 16
        self.rc_received = False

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

        # Stats
        self.iteration_count = 0
        self.total_distance_traveled = 0.0
        self.goal_reached = False
        self.physics_dt = 1.0 / max(1.0, physics_rate)
        self.start_time = time.time()

        # Logging
        self.log_file = None
        self.csv_writer = None
        if self.enable_logging:
            self._init_logging(log_dir)

        # QoS
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

        # Publishers (fake FC sensors)
        self.pub_gps_fix = self.create_publisher(NavSatFix, '/fc/gps_fix', sensor_qos)
        self.pub_gps_sats = self.create_publisher(Int32, '/fc/gps_satellites', sensor_qos)
        self.pub_gps_hdop = self.create_publisher(Float32, '/fc/gps_hdop', sensor_qos)
        self.pub_gps_spd = self.create_publisher(Float32MultiArray, '/fc/gps_speed_course', sensor_qos)
        self.pub_att_e = self.create_publisher(Vector3Stamped, '/fc/attitude_euler', sensor_qos)
        self.pub_imu = self.create_publisher(Imu, '/fc/imu_raw', sensor_qos)
        self.pub_altitude = self.create_publisher(Float32MultiArray, '/fc/altitude', sensor_qos)
        self.pub_lidar = self.create_publisher(Range, '/lidar_distance', sensor_qos)
        self.pub_wp = self.create_publisher(Float32MultiArray, '/fc/waypoint', reliable_qos)

        # Subscriber to RC commands from fc_adapter
        self.sub_rc = self.create_subscription(
            Float32MultiArray, '/fc/rc_override', self.cb_rc_override, reliable_qos
        )

        # Timers
        self.create_timer(self.physics_dt, self._physics_tick)
        self.create_timer(1.0 / gps_rate, self._publish_gps)
        self.create_timer(1.0 / imu_rate, self._publish_imu)
        self.create_timer(1.0 / sensor_rate, self._publish_sensors)
        self.create_timer(1.0 / lidar_rate, self._publish_lidar)
        self.create_timer(1.0, self._publish_waypoint)
        self.create_timer(1.0, self._print_status)

        # Banner
        self.get_logger().info('=' * 80)
        self.get_logger().info('FC FULL LOOP SIMULATOR NODE')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Physics rate: {physics_rate} Hz (dt={self.physics_dt*1000:.1f}ms)')
        self.get_logger().info(f'GPS rate: {gps_rate} Hz (realistic)')
        self.get_logger().info(f'ALT HOLD simulation: {"ENABLED" if self.althold_enabled else "DISABLED"}')
        if self.althold_enabled:
            self.get_logger().info(f'  - INAV ALT HOLD Kp: {self.althold_kp}')
            self.get_logger().info(f'  - Max climb rate: {self.max_climb_rate} m/s')
        self.get_logger().info(f'Motor lag: τ={self.motor_tau}s')
        self.get_logger().info(f'GPS noise: σ={self.gps_noise}m')
        self.get_logger().info(f'Origin: ({self.origin[0]:.7f}, {self.origin[1]:.7f}, {self.origin[2]:.2f})')
        self.get_logger().info(f'Start ENU: [{self.pos_enu[0]:.2f}, {self.pos_enu[1]:.2f}, {self.pos_enu[2]:.2f}]')
        self.get_logger().info(f'Goal ENU: [{self.goal_rel_enu[0]:.2f}, {self.goal_rel_enu[1]:.2f}, {self.goal_rel_enu[2]:.2f}]')
        if self.enable_logging:
            self.get_logger().info(f'Logging to: {self.log_file.name if self.log_file else "N/A"}')
        self.get_logger().info('=' * 80)
        self.get_logger().info('Waiting for RC commands from fc_adapter_node...')
        self.get_logger().info('=' * 80)

    def _init_logging(self, log_dir: str):
        """Initialize CSV logging."""
        log_path = Path(log_dir).expanduser()
        log_path.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_filename = log_path / f'fc_loop_sim_{timestamp}.csv'

        self.log_file = open(log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        # CSV header
        self.csv_writer.writerow([
            'time_sec', 'iteration',
            'pos_e', 'pos_n', 'pos_u',
            'vel_e', 'vel_n', 'vel_u',
            'roll', 'pitch', 'yaw',
            'rc_roll', 'rc_pitch', 'rc_throttle', 'rc_yaw',
            'rc_arm', 'rc_angle', 'rc_althold', 'rc_override', 'rc_rth',
            'motor_thrust', 'target_thrust',
            'baro_alt', 'gps_alt',
            'distance_to_goal', 'distance_traveled'
        ])
        self.log_file.flush()

    # ────────────────────────────────────────────────────────────────
    # RC Override callback (from fc_adapter)
    # ────────────────────────────────────────────────────────────────
    def cb_rc_override(self, msg: Float32MultiArray):
        """Receive RC commands from fc_adapter_node."""
        if len(msg.data) >= 16:
            self.rc_channels = [int(x) for x in msg.data[:16]]
            if not self.rc_received:
                self.rc_received = True
                self.get_logger().info('✅ First RC command received from fc_adapter!')

    # ────────────────────────────────────────────────────────────────
    # Physics simulation (core loop)
    # ────────────────────────────────────────────────────────────────
    def _physics_tick(self):
        """Update physics at high rate (100 Hz)."""
        if not self.rc_received:
            return  # Wait for FC adapter to start sending commands

        dt = self.physics_dt

        # Extract RC channels
        rc_roll = self.rc_channels[0]      # 1000-2000
        rc_pitch = self.rc_channels[1]
        rc_throttle = self.rc_channels[2]
        rc_yaw = self.rc_channels[3]
        rc_althold = self.rc_channels[6]   # AUX3: 1800 = ALT HOLD ON

        # Simulate INAV's interpretation of RC commands
        # =============================================

        # YAW: Simple rate control (for now, keep yaw fixed)
        # In real flight, this would integrate yaw rate
        # yaw_rate = (rc_yaw - 1500) / 500.0 * 180.0  # deg/s
        # self.yaw += math.radians(yaw_rate) * dt

        # THROTTLE: Depends on ALT HOLD mode
        if rc_althold > 1700 and self.althold_enabled:
            # ALT HOLD mode: throttle stick → climb rate command
            # RC range [1000, 2000] → climb rate [-max, +max]
            throttle_normalized = (rc_throttle - 1500) / 500.0  # [-1, +1]
            target_climb_rate = np.clip(
                throttle_normalized * self.max_climb_rate,
                -self.max_climb_rate,
                self.max_climb_rate
            )

            # INAV's ALT HOLD PID: tries to achieve target climb rate
            # Simplified: P-controller on vertical velocity error
            vz_error = target_climb_rate - self.vel_enu[2]
            thrust_correction = self.althold_kp * vz_error

            # Base hover thrust (compensates gravity)
            gravity = 9.81  # m/s²
            hover_thrust = self.mass * gravity

            # Total thrust
            self.target_thrust = hover_thrust + thrust_correction

        else:
            # Manual throttle mode: direct thrust control
            # RC range [1000, 2000] → thrust [0, max_thrust]
            throttle_normalized = (rc_throttle - 1000) / 1000.0  # [0, 1]
            max_thrust = self.mass * 9.81 * 2.0  # 2x gravity for accel
            self.target_thrust = throttle_normalized * max_thrust

        # ROLL & PITCH: Convert to body accelerations
        # Simplified: RC deviation → tilt angle → horizontal thrust
        roll_normalized = (rc_roll - 1500) / 500.0   # [-1, +1]
        pitch_normalized = (rc_pitch - 1500) / 500.0

        max_tilt = math.radians(30.0)  # 30 degrees max
        self.roll = roll_normalized * max_tilt
        self.pitch = pitch_normalized * max_tilt

        # Motor dynamics: first-order lag
        # ================================
        alpha = dt / (self.motor_tau + dt)
        self.motor_thrust = self.motor_thrust + alpha * (self.target_thrust - self.motor_thrust)

        # Forces in body frame
        # ====================
        # Thrust is along body Z (up in body frame)
        thrust_body = np.array([0, 0, self.motor_thrust / self.mass])  # m/s²

        # Tilt creates horizontal components
        # For small angles: ax ≈ g*sin(pitch), ay ≈ g*sin(roll)
        thrust_body[0] += 9.81 * math.sin(self.pitch)   # forward (X)
        thrust_body[1] += -9.81 * math.sin(self.roll)   # right (Y, negative for right)

        # Transform thrust to ENU frame (using yaw)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        thrust_enu = np.array([
            thrust_body[0] * cy - thrust_body[1] * sy,  # East
            thrust_body[0] * sy + thrust_body[1] * cy,  # North
            thrust_body[2]                              # Up
        ])

        # Gravity (ENU frame)
        gravity_enu = np.array([0, 0, -9.81])

        # Drag (opposing velocity)
        drag_enu = -self.drag_coeff * self.vel_enu

        # Total acceleration
        self.acc_enu = thrust_enu + gravity_enu + drag_enu

        # Integrate velocity and position
        old_pos = self.pos_enu.copy()
        self.vel_enu += self.acc_enu * dt
        self.pos_enu += self.vel_enu * dt

        # Ground clamping
        if self.pos_enu[2] < 0:
            self.pos_enu[2] = 0
            self.vel_enu[2] = max(0, self.vel_enu[2])  # Stop downward velocity

        # Update barometer (drifts slowly)
        self.baro_drift += np.random.normal(0, self.baro_drift_rate * dt)
        self.baro_altitude = self.pos_enu[2] + self.baro_drift

        # Stats
        self.iteration_count += 1
        step_distance = np.linalg.norm(self.pos_enu - old_pos)
        self.total_distance_traveled += step_distance

        # Check goal
        distance_to_goal = np.linalg.norm(self.goal_rel_enu - self.pos_enu)
        if not self.goal_reached and distance_to_goal <= self.goal_tolerance:
            self.goal_reached = True
            self._on_goal_reached(distance_to_goal)

        # Log to CSV
        if self.csv_writer:
            elapsed = time.time() - self.start_time
            self.csv_writer.writerow([
                f'{elapsed:.3f}', self.iteration_count,
                f'{self.pos_enu[0]:.3f}', f'{self.pos_enu[1]:.3f}', f'{self.pos_enu[2]:.3f}',
                f'{self.vel_enu[0]:.3f}', f'{self.vel_enu[1]:.3f}', f'{self.vel_enu[2]:.3f}',
                f'{math.degrees(self.roll):.2f}', f'{math.degrees(self.pitch):.2f}', f'{math.degrees(self.yaw):.2f}',
                rc_roll, rc_pitch, rc_throttle, rc_yaw,
                self.rc_channels[4], self.rc_channels[5], self.rc_channels[6],
                self.rc_channels[7], self.rc_channels[8],
                f'{self.motor_thrust:.2f}', f'{self.target_thrust:.2f}',
                f'{self.baro_altitude:.3f}', f'{self.pos_enu[2]:.3f}',
                f'{distance_to_goal:.3f}', f'{self.total_distance_traveled:.3f}'
            ])
            if self.iteration_count % 100 == 0:
                self.log_file.flush()

    # ────────────────────────────────────────────────────────────────
    # Sensor publishers (with noise and realistic rates)
    # ────────────────────────────────────────────────────────────────
    def _publish_gps(self):
        """Publish GPS at realistic rate (1.5 Hz) with noise."""
        # Add GPS noise
        noisy_pos = self.pos_enu + np.random.normal(0, self.gps_noise, 3)

        # Convert to geodetic
        geo = self.transforms.enu_to_geodetic(
            float(noisy_pos[0]),
            float(noisy_pos[1]),
            float(noisy_pos[2]),
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[2]),
        )

        # Publish GPS fix
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.gps_frame
        fix.latitude = geo[0]
        fix.longitude = geo[1]
        fix.altitude = geo[2]
        fix.status.status = fix.status.STATUS_FIX  # 3D fix
        self.pub_gps_fix.publish(fix)

        # GPS quality
        sats = Int32()
        sats.data = 12  # Good satellite count
        self.pub_gps_sats.publish(sats)

        hdop = Float32()
        hdop.data = 0.8  # Good HDOP
        self.pub_gps_hdop.publish(hdop)

    def _publish_sensors(self):
        """Publish attitude, speed, altitude at 10 Hz."""
        # GPS speed/course (with noise)
        noisy_vel = self.vel_enu + np.random.normal(0, self.vel_noise, 3)
        speed_xy = float(math.hypot(noisy_vel[0], noisy_vel[1]))
        if speed_xy > 1e-6:
            course_deg = (math.degrees(math.atan2(noisy_vel[0], noisy_vel[1])) + 360.0) % 360.0
        else:
            course_deg = 0.0

        spdmsg = Float32MultiArray()
        spdmsg.data = [float(speed_xy), float(course_deg)]
        self.pub_gps_spd.publish(spdmsg)

        # Attitude Euler
        ve = Vector3Stamped()
        ve.header.stamp = self.get_clock().now().to_msg()
        ve.header.frame_id = self.att_frame
        ve.vector.x = self.roll
        ve.vector.y = self.pitch
        ve.vector.z = self.yaw
        self.pub_att_e.publish(ve)

        # Altitude (barometer + vertical velocity)
        alt_msg = Float32MultiArray()
        alt_msg.data = [float(self.baro_altitude), float(noisy_vel[2])]
        self.pub_altitude.publish(alt_msg)

    def _publish_imu(self):
        """Publish IMU at 50 Hz."""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame
        imu.angular_velocity.x = self.ang_vel[0]
        imu.angular_velocity.y = self.ang_vel[1]
        imu.angular_velocity.z = self.ang_vel[2]
        # Acceleration in body frame (simplified)
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81  # Feel 1g when hovering
        self.pub_imu.publish(imu)

    def _publish_lidar(self):
        """Publish LiDAR at 100 Hz."""
        altitude = float(self.pos_enu[2])
        down_dist = max(self.lidar_min, min(self.lidar_max, altitude))

        rng = Range()
        rng.header.stamp = self.get_clock().now().to_msg()
        rng.header.frame_id = self.lidar_frame
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = self.lidar_fov
        rng.min_range = self.lidar_min
        rng.max_range = self.lidar_max
        rng.range = down_dist
        self.pub_lidar.publish(rng)

    def _publish_waypoint(self):
        """Publish waypoint at 1 Hz."""
        wp = Float32MultiArray()
        wp.data = [1.0, float(self.goal_lat), float(self.goal_lon), float(self.goal_alt)]
        self.pub_wp.publish(wp)

    # ────────────────────────────────────────────────────────────────
    # Status and logging
    # ────────────────────────────────────────────────────────────────
    def _print_status(self):
        """Print status every second."""
        distance_to_goal = np.linalg.norm(self.goal_rel_enu - self.pos_enu)

        althold_str = "ALT_HOLD" if (self.rc_channels[6] > 1700 and self.althold_enabled) else "MANUAL"

        self.get_logger().info(
            f'[{self.iteration_count:05d}] '
            f'Pos=[{self.pos_enu[0]:6.2f}, {self.pos_enu[1]:6.2f}, {self.pos_enu[2]:6.2f}] | '
            f'Vel=[{self.vel_enu[0]:5.2f}, {self.vel_enu[1]:5.2f}, {self.vel_enu[2]:5.2f}] | '
            f'→Goal={distance_to_goal:6.2f}m | '
            f'RC[R,P,T,Y]=[{self.rc_channels[0]}, {self.rc_channels[1]}, {self.rc_channels[2]}, {self.rc_channels[3]}] | '
            f'Thrust={self.motor_thrust:.1f}N | '
            f'Mode={althold_str}'
        )

    def _on_goal_reached(self, distance_to_goal: float):
        """Called when goal is reached."""
        self.get_logger().info('=' * 80)
        self.get_logger().info('🎯 GOAL REACHED!')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'   • Final position: [{self.pos_enu[0]:.2f}, {self.pos_enu[1]:.2f}, {self.pos_enu[2]:.2f}]')
        self.get_logger().info(f'   • Target position: [{self.goal_rel_enu[0]:.2f}, {self.goal_rel_enu[1]:.2f}, {self.goal_rel_enu[2]:.2f}]')
        self.get_logger().info(f'   • Final distance: {distance_to_goal:.3f}m')
        self.get_logger().info(f'   • Total iterations: {self.iteration_count}')
        self.get_logger().info(f'   • Total distance traveled: {self.total_distance_traveled:.2f}m')

        straight_line = float(np.linalg.norm(self.goal_rel_enu - self.relative_start_enu))
        efficiency = (straight_line / self.total_distance_traveled * 100.0) if self.total_distance_traveled > 0 else 0.0
        self.get_logger().info(f'   • Path efficiency: {efficiency:.1f}%')
        self.get_logger().info('=' * 80)

    def destroy_node(self):
        """Clean shutdown."""
        if self.log_file:
            self.log_file.close()
            self.get_logger().info(f'Log file closed: {self.log_file.name}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FCFullLoopSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
