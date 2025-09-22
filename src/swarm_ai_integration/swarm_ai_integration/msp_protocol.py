#!/usr/bin/env python3
"""
MSP Protocol Implementation for INAV 7 Communication

This module implements the MultiWii Serial Protocol (MSP) for communication
with INAV 7 flight controllers. It handles message encoding, decoding,
and provides high-level interfaces for common operations.

MSP Protocol Format:
$M<direction><size><command><data><checksum>

Where:
- direction: '<' for request, '>' for response, '!' for error
- size: payload size (0-255 bytes)
- command: MSP command code
- data: command payload
- checksum: XOR checksum of size, command, and data
"""

import struct
import time
from enum import IntEnum
from typing import Optional, Union, List, Tuple, Dict, Any
import logging


class MSPDirection(IntEnum):
    """MSP message direction indicators"""
    REQUEST = ord('<')
    RESPONSE = ord('>')
    ERROR = ord('!')


class MSPCommand(IntEnum):
    """INAV MSP Command Codes"""
    # Status and identification
    MSP_IDENT = 100
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_SERVO = 103
    MSP_MOTOR = 104
    MSP_RC = 105
    MSP_RAW_GPS = 106
    MSP_COMP_GPS = 107
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110
    MSP_ARMING_CONFIG = 111

    # Control commands
    MSP_SET_RAW_RC = 200
    MSP_SET_RAW_GPS = 201
    MSP_SET_PID = 202
    MSP_SET_BOX = 203
    MSP_SET_RC_TUNING = 204
    MSP_ACC_CALIBRATION = 205
    MSP_MAG_CALIBRATION = 206
    MSP_SET_MISC = 207
    MSP_RESET_CONF = 208
    MSP_SET_WP = 209
    MSP_SELECT_SETTING = 210
    MSP_SET_HEAD = 211
    MSP_SET_SERVO_CONFIGURATION = 212
    MSP_SET_MOTOR = 214

    # INAV specific commands
    MSP_NAV_STATUS = 121
    MSP_NAV_CONFIG = 122
    MSP_SET_NAV_CONFIG = 123
    MSP_WP = 118
    MSP_SET_WP = 209
    MSP_BOXNAMES = 116
    MSP_PIDNAMES = 117
    MSP_BOXIDS = 119

    # Custom velocity control (if supported by firmware)
    MSP_SET_VELOCITY = 250


class MSPMessage:
    """MSP message container"""

    def __init__(self, command: int, data: bytes = b'', direction: MSPDirection = MSPDirection.REQUEST):
        self.command = command
        self.data = data
        self.direction = direction
        self.size = len(data)

    def encode(self) -> bytes:
        """Encode MSP message to wire format"""
        # Header: $M + direction
        header = b'$M' + bytes([self.direction])

        # Payload: size + command + data
        payload = struct.pack('<BB', self.size, self.command) + self.data

        # Checksum: XOR of size, command, and data
        checksum = 0
        for byte in payload:
            checksum ^= byte

        return header + payload + bytes([checksum])

    @staticmethod
    def decode(data: bytes) -> Optional['MSPMessage']:
        """Decode MSP message from wire format"""
        if len(data) < 6:  # Minimum message size
            return None

        if data[:2] != b'$M':
            return None

        direction = data[2]
        size = data[3]
        command = data[4]

        if len(data) < 6 + size:
            return None

        payload = data[5:5 + size]
        received_checksum = data[5 + size]

        # Verify checksum
        calculated_checksum = size ^ command
        for byte in payload:
            calculated_checksum ^= byte
        calculated_checksum &= 0xFF

        if received_checksum != calculated_checksum:
            return None

        return MSPMessage(command, payload, MSPDirection(direction))


class MSPDataTypes:
    """Data type handlers for MSP messages"""

    @staticmethod
    def pack_attitude(roll: float, pitch: float, yaw: float) -> bytes:
        """Pack attitude data (roll, pitch, yaw in decidegrees)"""
        roll_dd = int(roll * 10)
        pitch_dd = int(pitch * 10)
        yaw_dd = int(yaw * 10)
        return struct.pack('<hhh', roll_dd, pitch_dd, yaw_dd)

    @staticmethod
    def unpack_attitude(data: bytes) -> Tuple[float, float, float]:
        """Unpack attitude data to degrees"""
        if len(data) < 6:
            return 0.0, 0.0, 0.0
        roll_dd, pitch_dd, yaw_dd = struct.unpack('<hhh', data[:6])
        return roll_dd / 10.0, pitch_dd / 10.0, yaw_dd / 10.0

    @staticmethod
    def pack_rc_channels(channels: List[int]) -> bytes:
        """Pack RC channel data (1000-2000 range)"""
        return struct.pack('<' + 'H' * len(channels), *channels)

    @staticmethod
    def unpack_rc_channels(data: bytes) -> List[int]:
        """Unpack RC channel data"""
        num_channels = len(data) // 2
        if num_channels == 0:
            return []
        return list(struct.unpack('<' + 'H' * num_channels, data))

    @staticmethod
    def pack_raw_imu(acc_x: int, acc_y: int, acc_z: int,
                     gyro_x: int, gyro_y: int, gyro_z: int,
                     mag_x: int, mag_y: int, mag_z: int) -> bytes:
        """Pack raw IMU data"""
        return struct.pack('<hhhhhhhhh',
                          acc_x, acc_y, acc_z,
                          gyro_x, gyro_y, gyro_z,
                          mag_x, mag_y, mag_z)

    @staticmethod
    def unpack_raw_imu(data: bytes) -> Dict[str, Tuple[int, int, int]]:
        """Unpack raw IMU data"""
        if len(data) < 18:
            return {'acc': (0, 0, 0), 'gyro': (0, 0, 0), 'mag': (0, 0, 0)}

        values = struct.unpack('<hhhhhhhhh', data[:18])
        return {
            'acc': values[:3],
            'gyro': values[3:6],
            'mag': values[6:9]
        }

    @staticmethod
    def pack_gps_data(lat: int, lon: int, alt: int, speed: int,
                     ground_course: int, satellites: int) -> bytes:
        """Pack GPS data"""
        return struct.pack('<iiHHHB', lat, lon, alt, speed, ground_course, satellites)

    @staticmethod
    def unpack_gps_data(data: bytes) -> Dict[str, Union[int, float]]:
        """Unpack GPS data"""
        if len(data) < 17:
            return {}

        lat, lon, alt, speed, ground_course, satellites = struct.unpack('<iiHHHB', data[:17])
        return {
            'latitude': lat / 1e7,  # Convert to degrees
            'longitude': lon / 1e7,
            'altitude': alt,        # centimeters
            'speed': speed,         # cm/s
            'ground_course': ground_course / 10.0,  # degrees
            'satellites': satellites
        }

    @staticmethod
    def pack_motor_commands(motors: List[int]) -> bytes:
        """Pack motor command data (1000-2000 range)"""
        return struct.pack('<' + 'H' * len(motors), *motors)

    @staticmethod
    def unpack_status(data: bytes) -> Dict[str, Any]:
        """Unpack flight controller status"""
        if len(data) < 11:
            return {}

        cycle_time, i2c_errors, sensor_flags, flight_mode_flags, profile = struct.unpack('<HHHHI', data[:11])

        return {
            'cycle_time': cycle_time,
            'i2c_errors': i2c_errors,
            'sensor_flags': sensor_flags,
            'flight_mode_flags': flight_mode_flags,
            'profile': profile
        }


class MSPVelocityController:
    """High-level velocity control interface for INAV"""

    def __init__(self):
        self.rc_mid = 1500  # RC channel middle value
        self.rc_range = 400  # RC channel range (+/- from middle)

        # Velocity to RC channel mapping parameters
        self.max_velocity_ms = 5.0  # Maximum velocity in m/s
        self.max_yaw_rate_dps = 180.0  # Maximum yaw rate in degrees/s

    def velocity_to_rc_channels(self, vx: float, vy: float, vz: float, yaw_rate: float) -> List[int]:
        """
        Convert velocity commands to RC channel values for INAV velocity control

        Args:
            vx: Forward velocity (m/s, positive forward)
            vy: Right velocity (m/s, positive right)
            vz: Up velocity (m/s, positive up)
            yaw_rate: Yaw rate (rad/s, positive clockwise)

        Returns:
            List of RC channel values [roll, pitch, throttle, yaw]
        """
        # Convert velocities to RC channel values
        # INAV uses RC channels for velocity control in specific flight modes

        # Roll channel (controls lateral velocity)
        roll_cmd = int(self.rc_mid + (vy / self.max_velocity_ms) * self.rc_range)
        roll_cmd = max(1000, min(2000, roll_cmd))

        # Pitch channel (controls forward velocity, inverted)
        pitch_cmd = int(self.rc_mid - (vx / self.max_velocity_ms) * self.rc_range)
        pitch_cmd = max(1000, min(2000, pitch_cmd))

        # Throttle channel (controls vertical velocity)
        throttle_cmd = int(self.rc_mid + (vz / self.max_velocity_ms) * self.rc_range)
        throttle_cmd = max(1000, min(2000, throttle_cmd))

        # Yaw channel (controls yaw rate)
        yaw_rate_dps = yaw_rate * 180.0 / 3.14159  # Convert rad/s to deg/s
        yaw_cmd = int(self.rc_mid + (yaw_rate_dps / self.max_yaw_rate_dps) * self.rc_range)
        yaw_cmd = max(1000, min(2000, yaw_cmd))

        return [roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd]

    def create_rc_command(self, vx: float, vy: float, vz: float, yaw_rate: float) -> MSPMessage:
        """Create MSP_SET_RAW_RC command for velocity control"""
        # Standard 8-channel RC setup
        channels = [1500] * 8  # Initialize all channels to middle

        # Set primary control channels
        velocity_channels = self.velocity_to_rc_channels(vx, vy, vz, yaw_rate)
        channels[0] = velocity_channels[0]  # Roll
        channels[1] = velocity_channels[1]  # Pitch
        channels[2] = velocity_channels[2]  # Throttle
        channels[3] = velocity_channels[3]  # Yaw

        # Arm channel (channel 5) - keep armed if needed
        channels[4] = 2000  # Armed state

        # Mode switches can be set on channels 6-8 as needed
        # For velocity control, ensure appropriate flight mode is selected

        payload = MSPDataTypes.pack_rc_channels(channels)
        return MSPMessage(MSPCommand.MSP_SET_RAW_RC, payload)


class MSPProtocolError(Exception):
    """MSP protocol specific errors"""
    pass


class MSPTimeoutError(MSPProtocolError):
    """MSP communication timeout"""
    pass


class MSPChecksumError(MSPProtocolError):
    """MSP message checksum error"""
    pass