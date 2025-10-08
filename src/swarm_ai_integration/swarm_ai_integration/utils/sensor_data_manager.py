#!/usr/bin/env python3
"""
Sensor Data Manager - Centralized sensor data storage and management

This module handles:
- Storing and updating sensor data from various sources
- Tracking data availability flags
- Providing unified data access interface
- LiDAR distance management
"""

import numpy as np
from typing import Dict, Optional, Tuple


class SensorDataManager:
    """
    Manages all sensor data for the AI adapter.

    Centralizes data storage and provides a clean interface for
    accessing sensor information.
    """

    def __init__(self, num_lidar_rays: int = 16, relative_start_enu: Optional[np.ndarray] = None):
        """
        Initialize sensor data manager.

        Args:
            num_lidar_rays: Number of LiDAR rays to track
            relative_start_enu: Initial relative position [E, N, U] in meters
        """
        # Position data (geodetic)
        self.position = np.zeros(3, dtype=np.float32)  # [lat, lon, alt]
        self.origin_geodetic: Optional[np.ndarray] = None
        self.relative_start_enu = relative_start_enu if relative_start_enu is not None else np.array([0.0, 0.0, 3.0], dtype=np.float32)

        # GPS quality tracking
        self.gps_satellite_count = 0
        self.gps_fix_type = 0  # 0=NO_FIX, 1=2D, 2+=3D

        # Origin averaging (tiered strategy)
        self.origin_samples = []  # Buffer for averaging initial origin
        self.origin_averaging_window = 0  # Determined by satellite count
        self.origin_is_set = False

        # Orientation data
        self.quat_att = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)  # [x, y, z, w]
        self.euler_att = np.zeros(3, dtype=np.float32)  # [roll, pitch, yaw] radians

        # Velocity data
        self.velocity = np.zeros(3, dtype=np.float32)  # [vx_east, vy_north, vz]
        self.speed_mps = 0.0
        self.course_deg = 0.0

        # Angular velocity (IMU)
        self.angular_velocity = np.zeros(3, dtype=np.float32)  # [wx, wy, wz] rad/s

        # Goal (geodetic)
        self.goal_geodetic = np.zeros(3, dtype=np.float32)  # [lat, lon, alt]

        # LiDAR
        self.lidar_distances = np.ones(num_lidar_rays, dtype=np.float32)  # normalized

        # Data received flags
        self.data_received = {
            'gps': False,
            'att_quat': False,
            'att_euler': False,
            'imu': False,
            'lidar': False,
            'action': False,
            'gps_speed': False,
            'waypoint': False,
        }

    # -------------------------------------------------------------------------
    # Position & Origin Management
    # -------------------------------------------------------------------------

    def update_gps_quality(self, satellite_count: int, fix_type: int):
        """
        Update GPS quality metrics.

        Args:
            satellite_count: Number of satellites
            fix_type: Fix type (0=NO_FIX, 1=2D, 2+=3D)
        """
        self.gps_satellite_count = int(satellite_count)
        self.gps_fix_type = int(fix_type)

    def get_gps_quality(self) -> Tuple[int, int]:
        """Get GPS quality metrics. Returns (satellite_count, fix_type)."""
        return self.gps_satellite_count, self.gps_fix_type

    def is_gps_quality_sufficient(self, min_satellites: int = 6) -> bool:
        """
        Check if GPS quality is sufficient for flight.

        Args:
            min_satellites: Minimum satellite count required (default: 6)

        Returns:
            True if GPS has 3D fix and sufficient satellites
        """
        return self.gps_fix_type >= 2 and self.gps_satellite_count >= min_satellites

    def determine_averaging_window(self) -> int:
        """
        Determine origin averaging window based on satellite count (tiered strategy).

        Strategy:
        - 10+ satellites: Average 10 samples (~7 seconds at 1.5 Hz) - excellent quality
        - 8-9 satellites: Average 20 samples (~14 seconds) - good quality
        - 6-7 satellites: Average 30 samples (~20 seconds) - marginal quality
        - <6 satellites: Return 0 (insufficient)

        Returns:
            Number of samples to average, or 0 if insufficient satellites
        """
        if self.gps_satellite_count >= 10:
            return 10
        elif self.gps_satellite_count >= 8:
            return 20
        elif self.gps_satellite_count >= 6:
            return 30
        else:
            return 0  # Insufficient satellites

    def update_gps_position(self, lat: float, lon: float, alt: float) -> Tuple[bool, bool]:
        """
        Update GPS position with tiered averaging for origin.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude (meters)

        Returns:
            Tuple of (first_fix, origin_ready):
                - first_fix: True if this is the first GPS sample received
                - origin_ready: True if origin has been set (averaging complete)
        """
        self.position[0] = float(lat)
        self.position[1] = float(lon)
        self.position[2] = float(alt)

        first_fix = len(self.origin_samples) == 0

        if not self.data_received['gps']:
            self.data_received['gps'] = True

        # Origin setting with tiered averaging
        if not self.origin_is_set:
            # Determine averaging window if not yet set
            if self.origin_averaging_window == 0:
                self.origin_averaging_window = self.determine_averaging_window()

                if self.origin_averaging_window > 0:
                    # Log the strategy
                    pass  # Will be logged by calling node

            # Collect samples if window is determined
            if self.origin_averaging_window > 0:
                self.origin_samples.append([lat, lon, alt])

                # Check if we have enough samples
                if len(self.origin_samples) >= self.origin_averaging_window:
                    # Compute averaged origin
                    samples_array = np.array(self.origin_samples, dtype=np.float32)
                    averaged_origin = np.mean(samples_array, axis=0)

                    self.origin_is_set = True
                    return first_fix, True  # Origin is ready!

        return first_fix, self.origin_is_set

    def set_origin_geodetic(self, origin: np.ndarray):
        """Set the relative origin in geodetic coordinates."""
        self.origin_geodetic = origin.copy()

    def get_averaged_origin_samples(self) -> Optional[np.ndarray]:
        """
        Get averaged origin from collected samples.

        Returns:
            Averaged origin [lat, lon, alt] if samples collected, else None
        """
        if len(self.origin_samples) == 0:
            return None

        samples_array = np.array(self.origin_samples, dtype=np.float32)
        return np.mean(samples_array, axis=0)

    def get_origin_sample_count(self) -> int:
        """Get number of origin samples collected."""
        return len(self.origin_samples)

    def get_position(self) -> np.ndarray:
        """Get current geodetic position [lat, lon, alt]."""
        return self.position.copy()

    def get_origin(self) -> Optional[np.ndarray]:
        """Get origin geodetic coordinates, or None if not set."""
        return self.origin_geodetic.copy() if self.origin_geodetic is not None else None

    def get_relative_start_enu(self) -> np.ndarray:
        """Get desired initial relative ENU position."""
        return self.relative_start_enu.copy()

    # -------------------------------------------------------------------------
    # Orientation Management
    # -------------------------------------------------------------------------

    def update_attitude_quaternion(self, x: float, y: float, z: float, w: float):
        """Update attitude quaternion."""
        self.quat_att = np.array([x, y, z, w], dtype=np.float32)
        if not self.data_received['att_quat']:
            self.data_received['att_quat'] = True

    def update_attitude_euler(self, roll: float, pitch: float, yaw: float):
        """Update attitude Euler angles (radians)."""
        self.euler_att = np.array([roll, pitch, yaw], dtype=np.float32)
        if not self.data_received['att_euler']:
            self.data_received['att_euler'] = True

    def get_quaternion(self) -> np.ndarray:
        """Get current quaternion [x, y, z, w]."""
        return self.quat_att.copy()

    def get_euler(self) -> np.ndarray:
        """Get current Euler angles [roll, pitch, yaw] in radians."""
        return self.euler_att.copy()

    # -------------------------------------------------------------------------
    # Velocity Management
    # -------------------------------------------------------------------------

    def update_velocity_from_gps(self, speed_mps: float, course_deg: float, velocity_enu: np.ndarray):
        """
        Update velocity from GPS speed and course.

        Args:
            speed_mps: Speed in m/s
            course_deg: Course over ground in degrees
            velocity_enu: Precomputed ENU velocity [vx, vy, vz]
        """
        self.speed_mps = float(speed_mps)
        self.course_deg = float(course_deg)
        self.velocity = velocity_enu.copy()

        if not self.data_received['gps_speed']:
            self.data_received['gps_speed'] = True

    def get_velocity(self) -> np.ndarray:
        """Get current velocity [vx, vy, vz] in m/s."""
        return self.velocity.copy()

    def get_speed_and_course(self) -> Tuple[float, float]:
        """Get GPS speed and course. Returns (speed_mps, course_deg)."""
        return self.speed_mps, self.course_deg

    # -------------------------------------------------------------------------
    # Angular Velocity (IMU)
    # -------------------------------------------------------------------------

    def update_angular_velocity(self, wx: float, wy: float, wz: float):
        """Update angular velocity from IMU."""
        self.angular_velocity = np.array([wx, wy, wz], dtype=np.float32)
        if not self.data_received['imu']:
            self.data_received['imu'] = True

    def get_angular_velocity(self) -> np.ndarray:
        """Get angular velocity [wx, wy, wz] in rad/s."""
        return self.angular_velocity.copy()

    # -------------------------------------------------------------------------
    # Goal (Waypoint) Management
    # -------------------------------------------------------------------------

    def update_goal(self, lat: float, lon: float, alt: float):
        """Update goal position from waypoint."""
        self.goal_geodetic = np.array([lat, lon, alt], dtype=np.float32)
        if not self.data_received['waypoint']:
            self.data_received['waypoint'] = True

    def get_goal(self) -> np.ndarray:
        """Get goal position [lat, lon, alt]."""
        return self.goal_geodetic.copy()

    # -------------------------------------------------------------------------
    # LiDAR Management
    # -------------------------------------------------------------------------

    def update_lidar_ray(self, ray_index: int, normalized_distance: float):
        """
        Update a single LiDAR ray distance.

        Args:
            ray_index: Index of the ray (0-based)
            normalized_distance: Normalized distance [0, 1]
        """
        if 0 <= ray_index < len(self.lidar_distances):
            self.lidar_distances[ray_index] = float(normalized_distance)

        if not self.data_received['lidar']:
            self.data_received['lidar'] = True

    def get_lidar_distances(self) -> np.ndarray:
        """Get all LiDAR distances (normalized)."""
        return self.lidar_distances.copy()

    # -------------------------------------------------------------------------
    # Data Availability
    # -------------------------------------------------------------------------

    def mark_action_received(self):
        """Mark that at least one action has been received."""
        self.data_received['action'] = True

    def is_data_ready(self, required_keys: list) -> bool:
        """
        Check if all required data has been received.

        Args:
            required_keys: List of required data keys

        Returns:
            True if all required data is available
        """
        return all(self.data_received.get(k, False) for k in required_keys)

    def get_missing_data(self, required_keys: list) -> list:
        """
        Get list of missing required data.

        Args:
            required_keys: List of required data keys

        Returns:
            List of missing data keys
        """
        return [k for k in required_keys if not self.data_received.get(k, False)]

    def get_data_status(self) -> Dict[str, bool]:
        """Get copy of data received flags."""
        return self.data_received.copy()

    # -------------------------------------------------------------------------
    # Complete State Access
    # -------------------------------------------------------------------------

    def get_all_sensor_data(self) -> Dict:
        """
        Get all sensor data in a single dictionary.

        Returns:
            Dictionary with all current sensor values
        """
        return {
            'position': self.get_position(),
            'origin': self.get_origin(),
            'quaternion': self.get_quaternion(),
            'euler': self.get_euler(),
            'velocity': self.get_velocity(),
            'angular_velocity': self.get_angular_velocity(),
            'goal': self.get_goal(),
            'lidar': self.get_lidar_distances(),
            'speed_mps': self.speed_mps,
            'course_deg': self.course_deg,
            'data_status': self.get_data_status()
        }
