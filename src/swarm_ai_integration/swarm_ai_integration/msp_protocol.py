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

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('MSP_Protocol')


class MSPDirection(IntEnum):
    """MSP message direction indicators"""
    REQUEST = ord('<')
    RESPONSE = ord('>')
    ERROR = ord('!')


class MSPCommand(IntEnum):
    """INAV MSP Command Codes (corregidos según documentación oficial)"""
    # API and firmware info
    MSP_API_VERSION = 1
    MSP_FC_VARIANT = 2
    MSP_FC_VERSION = 3
    MSP_BOARD_INFO = 4
    MSP_BUILD_INFO = 5
    MSP_BF_BUILD_INFO = 69
    
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
    MSP_RC_TUNING = 111
    MSP_PID = 112
    MSP_BOX = 113
    MSP_MISC = 114
    MSP_MOTOR_PINS = 115
    MSP_BOXNAMES = 116
    MSP_PIDNAMES = 117
    MSP_WP = 118
    MSP_BOXIDS = 119
    MSP_SERVO_CONF = 120
    
    # Navigation
    MSP_NAV_STATUS = 121
    MSP_NAV_CONFIG = 122
    
    # Battery and sensors
    MSP_CELLS = 130
    MSP_BATTERY_STATE = 130
    MSP_VOLTAGE_METERS = 128
    MSP_AMPERAGE_METERS = 129
    
    # Configuration
    MSP_BATTERY_CONFIG = 32
    MSP_SET_BATTERY_CONFIG = 33
    MSP_MODE_RANGES = 34
    MSP_SET_MODE_RANGE = 35
    MSP_FEATURE = 36
    MSP_SET_FEATURE = 37
    MSP_BOARD_ALIGNMENT = 38
    MSP_SET_BOARD_ALIGNMENT = 39
    MSP_ARMING_CONFIG = 61
    MSP_SET_ARMING_CONFIG = 62
    MSP_RX_MAP = 64
    MSP_SET_RX_MAP = 65
    
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
    MSP_SET_SERVO_CONF = 212
    MSP_SET_MOTOR = 214
    MSP_SET_NAV_CONFIG = 215
    
    # System commands
    MSP_REBOOT = 68
    MSP_EEPROM_WRITE = 250
    MSP_DEBUGMSG = 253
    MSP_DEBUG = 254
    MSP_UID = 160


class MSPMessage:
    """MSP message container"""

    def __init__(self, command: int, data: bytes = b'', direction: MSPDirection = MSPDirection.REQUEST):
        self.command = command
        self.data = data
        self.direction = direction
        self.size = len(data)
        
        # Log message creation
        cmd_name = self._get_command_name(command)
        logger.debug(f"Creating MSP message: {cmd_name} (code={command}), direction={'REQ' if direction == MSPDirection.REQUEST else 'RESP' if direction == MSPDirection.RESPONSE else 'ERR'}, size={self.size}")

    @staticmethod
    def _get_command_name(command: int) -> str:
        """Get command name from code"""
        try:
            return MSPCommand(command).name
        except ValueError:
            return f"UNKNOWN_{command}"

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

        encoded = header + payload + bytes([checksum])
        logger.debug(f"Encoded message: {encoded.hex()}")
        return encoded

    @staticmethod
    def decode(data: bytes) -> Optional['MSPMessage']:
        """Decode MSP message from wire format"""
        logger.debug(f"Attempting to decode message: {data.hex()}")
        
        if len(data) < 6:  # Minimum message size
            logger.warning(f"Message too short: {len(data)} bytes (minimum 6)")
            return None

        if data[:2] != b'$M':
            logger.warning(f"Invalid header: {data[:2].hex()} (expected $M)")
            return None

        direction = data[2]
        size = data[3]
        command = data[4]

        if len(data) < 6 + size:
            logger.warning(f"Incomplete message: expected {6 + size} bytes, got {len(data)}")
            return None

        payload = data[5:5 + size]
        received_checksum = data[5 + size]

        # Verify checksum
        calculated_checksum = size ^ command
        for byte in payload:
            calculated_checksum ^= byte
        calculated_checksum &= 0xFF

        if received_checksum != calculated_checksum:
            logger.error(f"Checksum mismatch: received={received_checksum:02x}, calculated={calculated_checksum:02x}")
            return None

        msg = MSPMessage(command, payload, MSPDirection(direction))
        cmd_name = msg._get_command_name(command)
        dir_str = 'RESPONSE' if direction == MSPDirection.RESPONSE else 'REQUEST' if direction == MSPDirection.REQUEST else 'ERROR'
        logger.info(f"✓ Decoded {dir_str}: {cmd_name} (code={command}), size={size}, payload={payload.hex() if payload else 'empty'}")
        
        return msg


class MSPDataTypes:
    """Data type handlers for MSP messages"""

    @staticmethod
    def pack_attitude(roll: float, pitch: float, yaw: float) -> bytes:
        """Pack attitude data (roll, pitch, yaw in decidegrees)"""
        roll_dd = int(roll * 10)
        pitch_dd = int(pitch * 10)
        yaw_dd = int(yaw * 10)
        logger.debug(f"Packing attitude: roll={roll}°, pitch={pitch}°, yaw={yaw}°")
        return struct.pack('<hhh', roll_dd, pitch_dd, yaw_dd)

    @staticmethod
    def unpack_attitude(data: bytes) -> Tuple[float, float, float]:
        """Unpack attitude data to degrees"""
        if len(data) < 6:
            logger.warning(f"Insufficient data for attitude: {len(data)} bytes")
            return 0.0, 0.0, 0.0
        roll_dd, pitch_dd, yaw_dd = struct.unpack('<hhh', data[:6])
        result = (roll_dd / 10.0, pitch_dd / 10.0, yaw_dd / 10.0)
        logger.debug(f"Unpacked attitude: roll={result[0]}°, pitch={result[1]}°, yaw={result[2]}°")
        return result

    @staticmethod
    def pack_rc_channels(channels: List[int]) -> bytes:
        """Pack RC channel data (1000-2000 range)"""
        logger.debug(f"Packing {len(channels)} RC channels: {channels}")
        return struct.pack('<' + 'H' * len(channels), *channels)

    @staticmethod
    def unpack_rc_channels(data: bytes) -> List[int]:
        """Unpack RC channel data"""
        num_channels = len(data) // 2
        if num_channels == 0:
            return []
        channels = list(struct.unpack('<' + 'H' * num_channels, data))
        logger.debug(f"Unpacked {num_channels} RC channels: {channels}")
        return channels

    @staticmethod
    def pack_raw_imu(acc_x: int, acc_y: int, acc_z: int,
                     gyro_x: int, gyro_y: int, gyro_z: int,
                     mag_x: int, mag_y: int, mag_z: int) -> bytes:
        """Pack raw IMU data"""
        logger.debug(f"Packing IMU: acc=({acc_x},{acc_y},{acc_z}), gyro=({gyro_x},{gyro_y},{gyro_z}), mag=({mag_x},{mag_y},{mag_z})")
        return struct.pack('<hhhhhhhhh',
                          acc_x, acc_y, acc_z,
                          gyro_x, gyro_y, gyro_z,
                          mag_x, mag_y, mag_z)

    @staticmethod
    def unpack_raw_imu(data: bytes) -> Dict[str, Tuple[int, int, int]]:
        """Unpack raw IMU data"""
        if len(data) < 18:
            logger.warning(f"Insufficient data for IMU: {len(data)} bytes")
            return {'acc': (0, 0, 0), 'gyro': (0, 0, 0), 'mag': (0, 0, 0)}

        values = struct.unpack('<hhhhhhhhh', data[:18])
        result = {
            'acc': values[:3],
            'gyro': values[3:6],
            'mag': values[6:9]
        }
        logger.debug(f"Unpacked IMU: {result}")
        return result

    @staticmethod
    def pack_gps_data(lat: int, lon: int, alt: int, speed: int,
                     ground_course: int, satellites: int) -> bytes:
        """Pack GPS data"""
        logger.debug(f"Packing GPS: lat={lat}, lon={lon}, alt={alt}m, speed={speed}cm/s, course={ground_course}°, sats={satellites}")
        return struct.pack('<iiHHHB', lat, lon, alt, speed, ground_course, satellites)

    @staticmethod
    def unpack_gps_data(data: bytes) -> Dict[str, Union[int, float]]:
        """Unpack GPS data - INAV 7 format with fix byte and HDOP"""
        if len(data) < 18:
            logger.warning(f"Insufficient data for GPS: {len(data)} bytes (expected 18)")
            return {}

        # INAV MSP_RAW_GPS: lat(4), lon(4), alt(2), speed(2), ground_course(2), sats(1), fix(1), hdop(2)
        lat, lon, alt, speed, ground_course, satellites, fix_type, hdop = struct.unpack('<iiHHHBBH', data[:18])
        result = {
            'latitude': lat / 1e7,
            'longitude': lon / 1e7,
            'altitude': alt,
            'speed': speed,
            'ground_course': ground_course / 10.0,
            'satellites': satellites,
            'fix_type': fix_type,
            'hdop': hdop
        }
        logger.debug(f"Unpacked GPS: lat={result['latitude']}°, lon={result['longitude']}°, sats={satellites}, fix={fix_type}, hdop={hdop}")
        return result

    @staticmethod
    def pack_motor_commands(motors: List[int]) -> bytes:
        """Pack motor command data (1000-2000 range)"""
        logger.debug(f"Packing {len(motors)} motor commands: {motors}")
        return struct.pack('<' + 'H' * len(motors), *motors)

    @staticmethod
    def unpack_status(data: bytes) -> Dict[str, Any]:
        """Unpack flight controller status"""
        if len(data) < 11:
            logger.warning(f"Insufficient data for status: {len(data)} bytes")
            return {}

        # INAV MSP_STATUS format: cycle_time(2), i2c_errors(2), sensor(2), flag(4), current_profile(1)
        cycle_time, i2c_errors, sensor_flags, flight_mode_flags, profile = struct.unpack('<HHHIB', data[:11])

        result = {
            'cycle_time': cycle_time,
            'i2c_errors': i2c_errors,
            'sensor_flags': sensor_flags,
            'flight_mode_flags': flight_mode_flags,
            'profile': profile
        }
        logger.info(f"Status: cycle_time={cycle_time}µs, i2c_errors={i2c_errors}, flight_mode={flight_mode_flags:08x}")
        return result


class MSPVelocityController:
    """High-level velocity control interface for INAV"""

    def __init__(self):
        self.rc_mid = 1500
        self.rc_range = 400
        self.max_velocity_ms = 5.0
        self.max_yaw_rate_dps = 180.0
        logger.info("Velocity controller initialized: rc_mid=1500, rc_range=±400, max_vel=5.0m/s, max_yaw=180°/s")

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
        logger.debug(f"Converting velocities: vx={vx:.2f}m/s, vy={vy:.2f}m/s, vz={vz:.2f}m/s, yaw={yaw_rate:.2f}rad/s")
        
        roll_cmd = int(self.rc_mid + (vy / self.max_velocity_ms) * self.rc_range)
        roll_cmd = max(1000, min(2000, roll_cmd))

        pitch_cmd = int(self.rc_mid - (vx / self.max_velocity_ms) * self.rc_range)
        pitch_cmd = max(1000, min(2000, pitch_cmd))

        throttle_cmd = int(self.rc_mid + (vz / self.max_velocity_ms) * self.rc_range)
        throttle_cmd = max(1000, min(2000, throttle_cmd))

        yaw_rate_dps = yaw_rate * 180.0 / 3.14159
        yaw_cmd = int(self.rc_mid + (yaw_rate_dps / self.max_yaw_rate_dps) * self.rc_range)
        yaw_cmd = max(1000, min(2000, yaw_cmd))

        result = [roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd]
        logger.debug(f"RC commands: roll={roll_cmd}, pitch={pitch_cmd}, throttle={throttle_cmd}, yaw={yaw_cmd}")
        return result

    def create_rc_command(self, vx: float, vy: float, vz: float, yaw_rate: float) -> MSPMessage:
        """Create MSP_SET_RAW_RC command for velocity control"""
        channels = [1500] * 8
        velocity_channels = self.velocity_to_rc_channels(vx, vy, vz, yaw_rate)
        channels[0] = velocity_channels[0]  # Roll
        channels[1] = velocity_channels[1]  # Pitch
        channels[2] = velocity_channels[2]  # Throttle
        channels[3] = velocity_channels[3]  # Yaw
        channels[4] = 2000  # Armed

        logger.info(f"Creating RC command for velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")
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