#!/usr/bin/env python3
"""
Coordinate Transforms - Geodetic to ENU coordinate system conversions

This module handles:
- Geodetic (lat/lon/alt) to ENU (East-North-Up) transformations
- Local tangent plane approximations for small-scale positioning
- Relative origin calculations
- Distance and bearing computations
"""

import math
import numpy as np
from typing import Tuple


class CoordinateTransforms:
    """
    Coordinate transformation utilities for drone positioning.

    Uses local tangent plane approximation for converting between
    geodetic coordinates (latitude, longitude, altitude) and local
    ENU (East-North-Up) Cartesian coordinates.
    """

    # Earth constants
    METERS_PER_DEGREE_LAT = 111320.0  # Approximate meters per degree of latitude

    @staticmethod
    def geodetic_to_enu(
        lat: float, lon: float, alt: float,
        ref_lat: float, ref_lon: float, ref_alt: float
    ) -> np.ndarray:
        """
        Convert geodetic coordinates to ENU offset from a reference point.

        Uses small-angle local tangent plane approximation suitable for
        distances up to a few kilometers.

        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            alt: Target altitude (meters)
            ref_lat: Reference latitude (degrees)
            ref_lon: Reference longitude (degrees)
            ref_alt: Reference altitude (meters)

        Returns:
            numpy array [east, north, up] in meters
        """
        lat_rad = math.radians(ref_lat)
        dlat = float(lat) - float(ref_lat)   # degrees
        dlon = float(lon) - float(ref_lon)   # degrees

        # Compute meters per degree longitude at this latitude
        m_per_deg_lon = CoordinateTransforms.METERS_PER_DEGREE_LAT * math.cos(
            lat_rad if not np.isnan(lat_rad) else 0.0
        )

        east = dlon * m_per_deg_lon
        north = dlat * CoordinateTransforms.METERS_PER_DEGREE_LAT
        up = float(alt) - float(ref_alt)

        return np.array([east, north, up], dtype=np.float32)

    @staticmethod
    def enu_to_geodetic(
        east: float, north: float, up: float,
        ref_lat: float, ref_lon: float, ref_alt: float
    ) -> np.ndarray:
        """
        Convert ENU offset to geodetic coordinates.

        Inverse of geodetic_to_enu, using the same local tangent plane approximation.

        Args:
            east: East offset (meters)
            north: North offset (meters)
            up: Up offset (meters)
            ref_lat: Reference latitude (degrees)
            ref_lon: Reference longitude (degrees)
            ref_alt: Reference altitude (meters)

        Returns:
            numpy array [latitude, longitude, altitude]
        """
        lat_rad = math.radians(ref_lat)

        # Compute meters per degree at reference latitude
        m_per_deg_lat = CoordinateTransforms.METERS_PER_DEGREE_LAT
        m_per_deg_lon = CoordinateTransforms.METERS_PER_DEGREE_LAT * math.cos(
            lat_rad if not np.isnan(lat_rad) else 0.0
        )

        # Invert the transformations
        dlat = north / m_per_deg_lat
        dlon = east / m_per_deg_lon if abs(m_per_deg_lon) > 1e-6 else 0.0

        lat = ref_lat + dlat
        lon = ref_lon + dlon
        alt = ref_alt + up

        return np.array([lat, lon, alt], dtype=np.float32)

    @staticmethod
    def compute_origin_for_initial_position(
        current_lat: float, current_lon: float, current_alt: float,
        desired_enu: np.ndarray
    ) -> np.ndarray:
        """
        Compute a geodetic origin such that the current position appears at desired_enu.

        This is useful for setting up a relative coordinate system where the drone
        starts at a specific ENU position (e.g., [0, 0, 3] meters).

        Args:
            current_lat: Current latitude (degrees)
            current_lon: Current longitude (degrees)
            current_alt: Current altitude (meters)
            desired_enu: Desired [E, N, U] position in meters

        Returns:
            numpy array [origin_lat, origin_lon, origin_alt]
        """
        e0, n0, u0 = [float(x) for x in desired_enu]

        lat_rad = math.radians(current_lat)
        m_per_deg_lat = CoordinateTransforms.METERS_PER_DEGREE_LAT
        m_per_deg_lon = CoordinateTransforms.METERS_PER_DEGREE_LAT * math.cos(
            lat_rad if not np.isnan(lat_rad) else 0.0
        )

        # Solve for origin such that: ENU(current - origin) = desired_enu
        # E = (lon - lon_ref) * m_per_deg_lon  => lon_ref = lon - E / m_per_deg_lon
        # N = (lat - lat_ref) * m_per_deg_lat  => lat_ref = lat - N / m_per_deg_lat
        # U = alt - alt_ref                    => alt_ref = alt - U

        origin_lat = current_lat - (n0 / m_per_deg_lat)
        origin_lon = current_lon - (e0 / m_per_deg_lon) if abs(m_per_deg_lon) > 1e-6 else current_lon
        origin_alt = current_alt - u0

        return np.array([origin_lat, origin_lon, origin_alt], dtype=np.float32)

    @staticmethod
    def distance_2d(point1_enu: np.ndarray, point2_enu: np.ndarray) -> float:
        """
        Compute 2D horizontal distance between two ENU points.

        Args:
            point1_enu: First point [E, N, U]
            point2_enu: Second point [E, N, U]

        Returns:
            Horizontal distance in meters
        """
        de = float(point2_enu[0]) - float(point1_enu[0])
        dn = float(point2_enu[1]) - float(point1_enu[1])
        return math.sqrt(de * de + dn * dn)

    @staticmethod
    def distance_3d(point1_enu: np.ndarray, point2_enu: np.ndarray) -> float:
        """
        Compute 3D distance between two ENU points.

        Args:
            point1_enu: First point [E, N, U]
            point2_enu: Second point [E, N, U]

        Returns:
            3D distance in meters
        """
        diff = point2_enu - point1_enu
        return float(np.linalg.norm(diff))

    @staticmethod
    def bearing_to_target(from_enu: np.ndarray, to_enu: np.ndarray) -> float:
        """
        Compute bearing from one ENU point to another.

        Args:
            from_enu: Starting point [E, N, U]
            to_enu: Target point [E, N, U]

        Returns:
            Bearing in degrees (0-360, where 0 is North, 90 is East)
        """
        de = float(to_enu[0]) - float(from_enu[0])
        dn = float(to_enu[1]) - float(from_enu[1])

        bearing_rad = math.atan2(de, dn)  # atan2(East, North)
        bearing_deg = math.degrees(bearing_rad)

        # Normalize to 0-360
        if bearing_deg < 0:
            bearing_deg += 360.0

        return bearing_deg

    @staticmethod
    def velocity_cog_to_enu(speed_mps: float, course_deg: float) -> np.ndarray:
        """
        Convert GPS speed and course-over-ground to ENU velocity.

        Args:
            speed_mps: Speed in meters per second
            course_deg: Course over ground in degrees (0 = North, 90 = East)

        Returns:
            numpy array [vx_east, vy_north, vz=0] in m/s
        """
        course_rad = math.radians(course_deg)
        vx_east = speed_mps * math.sin(course_rad)
        vy_north = speed_mps * math.cos(course_rad)

        return np.array([vx_east, vy_north, 0.0], dtype=np.float32)
