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
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray, String

from swarm_ai_integration.msp_protocol import MSPDataTypes, MSPCommand


class MSPMessageParser:
    """
    Parses MSP messages and converts them to ROS message format.

    This class contains all the data parsing logic, keeping the main
    node clean and focused on coordination.
    """

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert Euler angles (radians) to quaternion (x, y, z, w) using ZYX order.

        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians

        Returns:
            Tuple of (qx, qy, qz, qw)
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return (qx, qy, qz, qw)

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
            acc_scale = 9.81 / 512.0  # Typical scale for INAV
            imu_msg.linear_acceleration.x = float(imu_data['acc'][0] * acc_scale)
            imu_msg.linear_acceleration.y = float(imu_data['acc'][1] * acc_scale)
            imu_msg.linear_acceleration.z = float(imu_data['acc'][2] * acc_scale)

            # Gyroscope: convert to rad/s
            gyro_scale = 0.001 * np.pi / 180.0  # Typical scale
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
    ) -> Tuple[Optional[NavSatFix], Optional[Float32MultiArray]]:
        """
        Parse MSP_RAW_GPS data and create GPS and speed/course messages.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp
            frame_id: TF frame ID

        Returns:
            Tuple of (NavSatFix message, speed/course array) or (None, None)
        """
        try:
            gps_data = MSPDataTypes.unpack_gps_data(data)

            if not gps_data:
                return None, None

            # Create NavSatFix message
            gps_msg = NavSatFix()
            gps_msg.header.stamp = timestamp
            gps_msg.header.frame_id = frame_id

            gps_msg.latitude = gps_data['latitude']
            gps_msg.longitude = gps_data['longitude']
            gps_msg.altitude = float(gps_data['altitude'])  # meters

            fix_type_value = gps_data.get('fix_type', 0)
            # Map: 0=NO_FIX, 1=2D, 2=3D (treat >=3 as 3D as well)
            if fix_type_value >= 2:
                gps_msg.status.status = gps_msg.status.STATUS_FIX
            elif fix_type_value == 1:
                gps_msg.status.status = gps_msg.status.STATUS_SBAS_FIX
            else:
                gps_msg.status.status = gps_msg.status.STATUS_NO_FIX

            gps_msg.status.service = gps_msg.status.SERVICE_GPS
            gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN

            # Create speed/course message
            speed_cms = int(gps_data.get('speed', 0))  # cm/s from MSP
            speed_mps = float(speed_cms) / 100.0  # -> m/s
            course_deg = float(gps_data.get('ground_course', 0.0))  # deg

            speed_msg = Float32MultiArray()
            speed_msg.data = [speed_mps, course_deg]  # [m/s, deg]

            return gps_msg, speed_msg

        except Exception as e:
            return None, None

    def parse_attitude_data(
        self,
        data: bytes,
        timestamp,
        frame_id: str = 'fc_attitude'
    ) -> Tuple[Optional[QuaternionStamped], Optional[Vector3Stamped]]:
        """
        Parse MSP_ATTITUDE data and create quaternion and Euler messages.

        Args:
            data: Raw MSP payload
            timestamp: ROS timestamp
            frame_id: TF frame ID

        Returns:
            Tuple of (QuaternionStamped, Vector3Stamped) or (None, None)
        """
        try:
            roll, pitch, yaw = MSPDataTypes.unpack_attitude(data)

            # Convert to radians
            roll_rad = np.radians(roll)
            pitch_rad = np.radians(pitch)
            yaw_rad = np.radians(yaw)

            # Create Euler angles message
            euler_msg = Vector3Stamped()
            euler_msg.header.stamp = timestamp
            euler_msg.header.frame_id = frame_id
            euler_msg.vector.x = roll_rad   # roll
            euler_msg.vector.y = pitch_rad  # pitch
            euler_msg.vector.z = yaw_rad    # yaw

            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

            # Create quaternion message
            quat_msg = QuaternionStamped()
            quat_msg.header.stamp = timestamp
            quat_msg.header.frame_id = frame_id
            quat_msg.quaternion.x = qx
            quat_msg.quaternion.y = qy
            quat_msg.quaternion.z = qz
            quat_msg.quaternion.w = qw

            return quat_msg, euler_msg

        except Exception as e:
            return None, None

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

            battery_msg.voltage = float(vbat / 10.0)        # volts
            battery_msg.current = float(amperage / 100.0)   # amps
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

    def parse_ident_data(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse MSP_IDENT data and return flight controller identification info.

        Based on MSP protocol:
        - byte 0: version
        - byte 1: multitype (aircraft type)
        - byte 2: MSP version
        - bytes 3-6: capability flags (uint32, little-endian)

        Capabilities:
        - Bit 0: BIND - Binding support
        - Bit 2: DYNBAL - Dynamic balancing
        - Bit 3: FLAP - Flap support
        - Bit 4: NAVCAP - Navigation capability
        - Bit 5: EXTAUX - Extended AUX channels

        Args:
            data: Raw MSP_IDENT payload (7 bytes expected)

        Returns:
            Dictionary with parsed ident data or None if parsing fails
        """
        try:
            if len(data) < 7:
                return None

            # Parse basic fields
            version = data[0]
            multitype = data[1]
            msp_version = data[2]

            # Parse capability flags (uint32, little-endian)
            capability = int.from_bytes(data[3:7], byteorder='little', signed=False)

            # Decode capability bits
            capabilities = []
            if capability & (1 << 0):
                capabilities.append('BIND')
            if capability & (1 << 2):
                capabilities.append('DYNBAL')
            if capability & (1 << 3):
                capabilities.append('FLAP')
            if capability & (1 << 4):
                capabilities.append('NAVCAP')
            if capability & (1 << 5):
                capabilities.append('EXTAUX')

            # Multitype mapping (common INAV/Betaflight types)
            multitype_names = {
                1: 'TRI',
                2: 'QUADP',
                3: 'QUADX',
                4: 'BI',
                5: 'GIMBAL',
                6: 'Y6',
                7: 'HEX6',
                8: 'FLYING_WING',
                9: 'Y4',
                10: 'HEX6X',
                11: 'OCTOX8',
                12: 'OCTOFLATX',
                13: 'OCTOFLATP',
                14: 'AIRPLANE',
                15: 'HELI_120_CCPM',
                16: 'HELI_90_DEG',
                17: 'VTAIL4',
                18: 'HEX6H',
                19: 'PPM_TO_SERVO',
                20: 'DUALCOPTER',
                21: 'SINGLECOPTER',
            }

            return {
                'version': version,
                'multitype': multitype,
                'multitype_name': multitype_names.get(multitype, f'UNKNOWN_{multitype}'),
                'msp_version': msp_version,
                'capability_raw': capability,
                'capabilities': capabilities,
                'has_bind': 'BIND' in capabilities,
                'has_dynbal': 'DYNBAL' in capabilities,
                'has_flap': 'FLAP' in capabilities,
                'has_navcap': 'NAVCAP' in capabilities,
                'has_extaux': 'EXTAUX' in capabilities,
            }

        except Exception as e:
            return None
