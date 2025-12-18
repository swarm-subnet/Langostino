#!/usr/bin/env python3
"""
MSP Message Parser - Parses MSP data and converts to ROS messages

This module handles:
- Parsing MSP protocol data payloads
- Converting raw data to ROS message types
- Data validation and error handling
- Coordinate transformations and unit conversions
"""

import numpy as np
from typing import Optional, Dict, Any, Tuple

from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray, String, Int32, Float32

from swarm_ai_integration.msp_protocol import MSPDataTypes, MSPCommand


class MSPMessageParser:
    """
    Parses MSP messages and converts them to ROS message format.

    This class contains all the data parsing logic, keeping the main
    node clean and focused on coordination.
    """

    def __init__(self, imu_scale_accel: float = 9.81 / 512.0, imu_scale_gyro: float = 0.001,
                 battery_voltage_scale: float = 10.0, battery_current_scale: float = 100.0):
        """
        Initialize MSP Message Parser with scale factors.

        Args:
            imu_scale_accel: IMU accelerometer scale factor
            imu_scale_gyro: IMU gyroscope scale factor (radians)
            battery_voltage_scale: Battery voltage divisor
            battery_current_scale: Battery current divisor
        """
        self.imu_scale_accel = imu_scale_accel
        self.imu_scale_gyro = imu_scale_gyro
        self.battery_voltage_scale = battery_voltage_scale
        self.battery_current_scale = battery_current_scale

    def parse_imu_data(self, data: bytes, timestamp, frame_id: str = 'fc_imu') -> Optional[Imu]:
        """
        Parse MSP_RAW_IMU data and create IMU ROS message.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp
            frame_id: TF frame ID

        Returns:
            IMU message or None if parsing fails
        """
        try:
            imu_data = MSPDataTypes.unpack_raw_imu(data)

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = frame_id

            # Convert raw values to proper units
            # Accelerometer: convert to m/s²
            imu_msg.linear_acceleration.x = float(imu_data['acc'][0] * self.imu_scale_accel)
            imu_msg.linear_acceleration.y = float(imu_data['acc'][1] * self.imu_scale_accel)
            imu_msg.linear_acceleration.z = float(imu_data['acc'][2] * self.imu_scale_accel)

            # Gyroscope: convert to rad/s
            gyro_scale = self.imu_scale_gyro * np.pi / 180.0
            imu_msg.angular_velocity.x = float(imu_data['gyro'][0] * gyro_scale)
            imu_msg.angular_velocity.y = float(imu_data['gyro'][1] * gyro_scale)
            imu_msg.angular_velocity.z = float(imu_data['gyro'][2] * gyro_scale)

            # Orientation is not provided by MSP_RAW_IMU, leave as identity quaternion
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

            return imu_msg

        except Exception as e:
            return None

    def parse_gps_data(
        self,
        data: bytes,
        timestamp,
        frame_id: str = 'fc_gps'
    ) -> Tuple[Optional[NavSatFix], Optional[Float32MultiArray], Optional[Int32], Optional[Float32]]:
        """
        Parse MSP_RAW_GPS data and create GPS, speed/course, satellite count, and HDOP messages.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp
            frame_id: TF frame ID

        Returns:
            Tuple of (NavSatFix message, speed/course array, satellite count, HDOP) or (None, None, None, None)
        """
        try:
            # DEBUG: Log raw GPS data bytes
            if len(data) >= 16:
                import struct
                fix_byte = data[0]
                sat_byte = data[1]
                print(f"🐛 GPS DEBUG: fix_byte=0x{fix_byte:02x} ({fix_byte}), satellites={sat_byte}, raw_hex={data[:16].hex()}")

            gps_data = MSPDataTypes.unpack_gps_data(data)

            if not gps_data:
                return None, None, None, None

            # Create NavSatFix message
            gps_msg = NavSatFix()
            gps_msg.header.stamp = timestamp
            gps_msg.header.frame_id = frame_id

            gps_msg.latitude = gps_data['latitude']
            gps_msg.longitude = gps_data['longitude']
            gps_msg.altitude = float(gps_data['altitude'])  # meters

            fix_type_value = gps_data.get('fix_type', 0)
            num_satellites = int(gps_data.get('satellites', 0))

            # Map INAV fix_type to ROS NavSatStatus
            # INAV: 0=NO_FIX, 1=2D, 2=3D
            # ROS: STATUS_NO_FIX=-1, STATUS_FIX=0 (valid 3D), STATUS_SBAS_FIX=1
            if fix_type_value >= 2:
                gps_msg.status.status = gps_msg.status.STATUS_FIX  # 3D fix → status=0 (VALID!)
            elif fix_type_value == 1:
                gps_msg.status.status = gps_msg.status.STATUS_SBAS_FIX  # 2D fix → status=1
            else:
                gps_msg.status.status = gps_msg.status.STATUS_NO_FIX  # No fix → status=-1

            gps_msg.status.service = gps_msg.status.SERVICE_GPS
            gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN

            # Create speed/course message
            speed_cms = int(gps_data.get('speed', 0))  # cm/s from MSP
            speed_mps = float(speed_cms) / 100.0  # -> m/s
            course_deg = float(gps_data.get('ground_course', 0.0))  # deg

            speed_msg = Float32MultiArray()
            speed_msg.data = [speed_mps, course_deg]  # [m/s, deg]

            # Create satellite count message
            sat_msg = Int32()
            sat_msg.data = int(gps_data.get('satellites', 0))

            # Create HDOP message (if available)
            hdop_msg = None
            hdop_value = gps_data.get('hdop')
            if hdop_value is not None:
                hdop_msg = Float32()
                hdop_msg.data = float(hdop_value)

            return gps_msg, speed_msg, sat_msg, hdop_msg

        except Exception as e:
            return None, None, None, None

    def parse_attitude_data(
        self,
        data: bytes,
        timestamp,
        frame_id: str = 'fc_attitude'
    ) -> Tuple[Optional[Vector3Stamped], Optional[Vector3Stamped]]:
        """
        Parse MSP_ATTITUDE data and create Euler angles messages in radians and degrees.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp
            frame_id: TF frame ID

        Returns:
            Tuple of (radians message, degrees message) or (None, None)
        """
        try:
            roll, pitch, yaw = MSPDataTypes.unpack_attitude(data)

            # Convert to radians
            roll_rad = np.radians(roll)
            pitch_rad = np.radians(pitch)
            yaw_rad = np.radians(yaw)

            # Create Euler angles message in radians
            euler_msg = Vector3Stamped()
            euler_msg.header.stamp = timestamp
            euler_msg.header.frame_id = frame_id
            euler_msg.vector.x = roll_rad   # roll
            euler_msg.vector.y = pitch_rad  # pitch
            euler_msg.vector.z = yaw_rad    # yaw

            # Create Euler angles message in degrees
            euler_degrees_msg = Vector3Stamped()
            euler_degrees_msg.header.stamp = timestamp
            euler_degrees_msg.header.frame_id = frame_id
            euler_degrees_msg.vector.x = float(roll)   # roll
            euler_degrees_msg.vector.y = float(pitch)  # pitch
            euler_degrees_msg.vector.z = float(yaw)    # yaw

            return euler_msg, euler_degrees_msg

        except Exception as e:
            return None, None

    def parse_altitude_data(self, data: bytes) -> Optional[Float32MultiArray]:
        """
        Parse MSP_ALTITUDE data and create altitude message.

        Args:
            data: Raw MSP payload

        Returns:
            Float32MultiArray with [altitude_m, vario_m/s] or None
        """
        try:
            altitude_data = MSPDataTypes.unpack_altitude(data)

            if not altitude_data:
                return None

            # Create altitude message
            # Format: [altitude_m, vertical_velocity_m/s]
            altitude_msg = Float32MultiArray()
            altitude_msg.data = [
                float(altitude_data['altitude_m']),
                float(altitude_data['vario'])
            ]

            return altitude_msg

        except Exception as e:
            return None

    def parse_status_data(
        self,
        data: bytes,
        timestamp
    ) -> Tuple[Optional[String], Optional[Float32MultiArray]]:
        """
        Parse MSP_STATUS data and create status messages.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp

        Returns:
            Tuple of (status string, raw status array) or (None, None)
        """
        try:
            status_data = MSPDataTypes.unpack_status(data)

            if not status_data:
                return None, None

            # Human-readable status string
            status_msg = String()
            status_msg.data = (
                f"Cycle: {status_data['cycle_time']}us, "
                f"I2C Errors: {status_data['i2c_errors']}, "
                f"Sensors: 0x{status_data['sensor_mask']:04x}, "
                f"Box Flags: 0x{status_data['box_flags']:08x}"
            )

            # Raw MSP_STATUS data array
            # Format: [cycle_time, i2c_errors, sensor_mask, box_flags, current_setting]
            msp_status_msg = Float32MultiArray()
            msp_status_msg.data = [
                float(status_data['cycle_time']),
                float(status_data['i2c_errors']),
                float(status_data['sensor_mask']),
                float(status_data['box_flags']),
                float(status_data.get('current_setting', 0))
            ]

            return status_msg, msp_status_msg

        except Exception as e:
            return None, None

    def parse_battery_data(
        self,
        data: bytes,
        timestamp,
        frame_id: str = 'fc_battery'
    ) -> Optional[BatteryState]:
        """
        Parse MSP_ANALOG (battery) data and create BatteryState message.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp
            frame_id: TF frame ID

        Returns:
            BatteryState message or None
        """
        try:
            if len(data) < 7:
                return None

            vbat, _, rssi, amperage = data[:4]  # power_meter_sum not used

            battery_msg = BatteryState()
            battery_msg.header.stamp = timestamp
            battery_msg.header.frame_id = frame_id

            battery_msg.voltage = float(vbat / self.battery_voltage_scale)        # volts
            battery_msg.current = float(amperage / self.battery_current_scale)   # amps
            battery_msg.power_supply_status = battery_msg.POWER_SUPPLY_STATUS_UNKNOWN
            battery_msg.power_supply_technology = battery_msg.POWER_SUPPLY_TECHNOLOGY_LIPO

            return battery_msg

        except Exception as e:
            return None

    def parse_motor_data(self, data: bytes) -> Optional[Float32MultiArray]:
        """
        Parse MSP_MOTOR data and create motor values array.

        Args:
            data: Raw MSP payload

        Returns:
            Float32MultiArray with motor values or None
        """
        try:
            if len(data) < 8:  # 4 motors × 2 bytes each
                return None

            motor_values = []
            for i in range(0, min(8, len(data)), 2):
                motor_value = int.from_bytes(data[i:i+2], 'little')
                motor_values.append(float(motor_value))

            motor_msg = Float32MultiArray()
            motor_msg.data = motor_values
            return motor_msg

        except Exception as e:
            return None

    def parse_waypoint_data(self, data: bytes) -> Optional[Float32MultiArray]:
        """
        Parse MSP_WP (waypoint) data and create waypoint array.

        Args:
            data: Raw MSP payload

        Returns:
            Float32MultiArray: [wp_no, lat, lon, alt_m, heading, stay, navflag] or None
        """
        try:
            wp = MSPDataTypes.unpack_waypoint(data)
            if not wp:
                return None

            # Format: [wp_no, lat_deg, lon_deg, alt_m, heading_deg, staytime_s, navflag]
            msg = Float32MultiArray()
            msg.data = [
                float(wp['wp_no']),
                float(wp['latitude']),
                float(wp['longitude']),
                float(wp['altitude_m']),
                float(wp['heading_deg']),
                float(wp['staytime_s']),
                float(wp['navflag']),
            ]
            return msg

        except Exception as e:
            return None

    def parse_rc_data(self, data: bytes) -> Optional[list]:
        """
        Parse MSP_RC data and return channel values.

        Args:
            data: Raw MSP payload

        Returns:
            List of RC channel values or None
        """
        try:
            channels = MSPDataTypes.unpack_rc_channels(data)
            if not channels:
                return None
            return channels

        except Exception as e:
            return None
