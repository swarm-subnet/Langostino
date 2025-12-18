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
        """
        Pack attitude data for sending to FC.

        INAV expects mixed units when SENDING commands:
        - Roll/Pitch: degrees (no conversion)
        - Yaw: decidegrees (multiply by 10)
        """
        roll_dd = int(roll)      # degrees
        pitch_dd = int(pitch)    # degrees
        yaw_dd = int(yaw * 10)   # decidegrees
        logger.debug(f"Packing attitude: roll={roll}°, pitch={pitch}°, yaw={yaw}°")
        return struct.pack('<hhh', roll_dd, pitch_dd, yaw_dd)

    @staticmethod
    def unpack_attitude(data: bytes) -> Tuple[float, float, float]:
        """
        Unpack attitude data to degrees.

        INAV sends mixed units:
        - Roll/Pitch: decidegrees (IMU-based, need ÷10)
        - Yaw: degrees (magnetometer-based, already correct)
        """
        if len(data) < 6:
            logger.warning(f"Insufficient data for attitude: {len(data)} bytes")
            return 0.0, 0.0, 0.0
        roll_dd, pitch_dd, yaw_deg = struct.unpack('<hhh', data[:6])

        # Convert roll/pitch from decidegrees to degrees, yaw is already in degrees
        roll = float(roll_dd) / 10.0
        pitch = float(pitch_dd) / 10.0
        yaw = float(yaw_deg)

        result = (roll, pitch, yaw)
        logger.debug(f"Unpacked attitude: roll={result[0]:.1f}°, pitch={result[1]:.1f}°, yaw={result[2]:.1f}°")
        return result
    
    @staticmethod
    def unpack_status(data: bytes) -> Dict[str, Any]:
        """
        Unpack MSP_STATUS (code=101).

        Layout:
        0..1  : uint16 time_us
        2..3  : uint16 errors
        4..5  : uint16 sensors_mask
        6..9  : uint32 box_flags
        10    : uint8  current_setting (optional on some targets)

        Total 11 bytes when current_setting is present; some targets send 10 bytes.
        """
        import struct
        res: Dict[str, Any] = {}
        try:
            if len(data) >= 11:
                time_us, errors, sensors_u16, flags_u32, current_setting = struct.unpack_from('<HHHIB', data, 0)
                res.update({
                    'cycle_time': time_us,      # keep legacy keys used by callers
                    'i2c_errors': errors,
                    'sensor_mask': sensors_u16,
                    'box_flags': flags_u32,
                    'current_setting': current_setting,
                })
            elif len(data) >= 10:
                time_us, errors, sensors_u16, flags_u32 = struct.unpack_from('<HHHI', data, 0)
                res.update({
                    'cycle_time': time_us,
                    'i2c_errors': errors,
                    'sensor_mask': sensors_u16,
                    'box_flags': flags_u32,
                    'current_setting': None,
                })
            else:
                try:
                    logger.warning(f"MSP_STATUS: insufficient length {len(data)}")
                except Exception:
                    pass
                return {}
        except struct.error as e:
            try:
                logger.error(f"MSP_STATUS unpack error: {e}; data={data.hex()}")
            except Exception:
                pass
            return {}

        # Convenience booleans per spec bits:
        # bit0=Accelerometer, bit1=Barometer, bit2=Magnetometer, bit3=GPS, bit4=Sonar
        s = res['sensor_mask']
        res['accelerometer'] = bool(s & (1 << 0))
        res['barometer']     = bool(s & (1 << 1))
        res['magnetometer']  = bool(s & (1 << 2))
        res['gps']           = bool(s & (1 << 3))
        res['sonar']         = bool(s & (1 << 4))

        # Do NOT infer "armed" here — MSP_STATUS does not carry an armed bit.
        return res

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
        """
        Unpack GPS data for MSP_RAW_GPS (code=106) per MSP spec:

        Byte layout (length = 16 bytes):
            0   : uint8   fix
            1   : uint8   numSat
            2-5 : int32   lat       (° * 1e7, SIGNED for negative latitudes)
            6-9 : int32   lon       (° * 1e7, SIGNED for negative longitudes)
            10-11: uint16 altitude  (meters)
            12-13: uint16 speed     (cm/s)
            14-15: uint16 ground_course (deg * 10)

        NOTE: MSP docs show uint32 for lat/lon but it's actually int32 (signed)
        to support negative coordinates (southern hemisphere, western longitude).

        Returns keys:
            latitude (float, deg), longitude (float, deg),
            altitude (int, m), speed (int, cm/s),
            ground_course (float, deg),
            satellites (int), fix_type (int)
        """
        if len(data) < 16:
            logger.warning(f"Insufficient data for GPS: {len(data)} bytes (expected 16+)")
            return {}

        # INAV sometimes sends 18 bytes instead of 16
        # Parse first 16 bytes: BB = two uint8, ii = two int32 (signed), HHH = three uint16
        fix, num_sat, lat_i, lon_i, alt_m, speed_cms, course_10deg = struct.unpack('<BBiiHHH', data[:16])

        # If 18 bytes, check what the extra bytes contain
        hdop = None
        extra_bytes_hex = ""
        if len(data) >= 18:
            extra_bytes = data[16:18]
            extra_bytes_hex = extra_bytes.hex()
            # Try as uint16 HDOP
            hdop = struct.unpack('<H', extra_bytes)[0]
            logger.info(f"📡 GPS Extended (18 bytes): extra_bytes=0x{extra_bytes_hex}, interpreted_as_hdop={hdop}")

        # Debug: Log all raw values
        logger.info(f"🐛 GPS FULL DEBUG: fix_byte={fix} (0x{fix:02x}), sats={num_sat}, "
                   f"lat_raw={lat_i}, lon_raw={lon_i}, hdop={hdop}, "
                   f"full_hex={data[:18].hex() if len(data)>=18 else data.hex()}")

        return {
            'latitude':  lat_i / 1e7,
            'longitude': lon_i / 1e7,
            'altitude':  alt_m,                       # meters
            'speed':     speed_cms,                   # cm/s
            'ground_course': course_10deg / 10.0,     # degrees
            'satellites': num_sat,
            'fix_type':  fix,
            'hdop':      hdop / 100.0 if hdop is not None else None  # HDOP in meters (100 = 1.00m)
        }

    @staticmethod
    def pack_waypoint_request(wp_no: int) -> bytes:
        """Payload for MSP_WP request: single byte with waypoint index."""
        return struct.pack('<B', wp_no & 0xFF)

    @staticmethod
    def unpack_altitude(data: bytes) -> Dict[str, float]:
        """
        Unpack MSP_ALTITUDE (code=109) data.

        INAV sends 10 bytes (extended format):
            0..3  : int32  estimated_altitude (cm, relative to arming point)
            4..5  : int16  vertical_velocity (cm/s, vario)
            6..9  : int32  estimated_z_velocity (cm/s, possibly for navigation)

        Returns:
            {
                'altitude_m': float (meters),
                'vario': float (m/s, vertical velocity)
            }
        """
        # DEBUG: Always log raw data
        logger.info(f"🐛 MSP_ALTITUDE RAW: len={len(data)} bytes, hex={data.hex() if data else 'empty'}")

        if len(data) < 6:
            logger.warning(f"Insufficient data for MSP_ALTITUDE: {len(data)} bytes (expected 6-10)")
            return {'altitude_m': 0.0, 'vario': 0.0}

        try:
            # Parse first 6 bytes (always present)
            altitude_cm, vario_cms = struct.unpack('<ih', data[:6])

            # Parse extended format if available (bytes 6-9)
            z_vel_cms = 0
            if len(data) >= 10:
                z_vel_cms = struct.unpack('<i', data[6:10])[0]
                logger.info(f"🐛 MSP_ALTITUDE EXTENDED: altitude_cm={altitude_cm}, vario_cms={vario_cms}, z_vel_cms={z_vel_cms}")
            else:
                logger.info(f"🐛 MSP_ALTITUDE UNPACKED: altitude_cm={altitude_cm}, vario_cms={vario_cms}")

            result = {
                'altitude_m': float(altitude_cm) / 100.0,  # cm to meters
                'vario': float(vario_cms) / 100.0          # cm/s to m/s
            }
            logger.info(f"✓ MSP_ALTITUDE RESULT: altitude={result['altitude_m']:.2f}m, vario={result['vario']:+.2f}m/s")
            return result

        except struct.error as e:
            logger.error(f"MSP_ALTITUDE unpack error: {e}; data={data.hex()}")
            return {'altitude_m': 0.0, 'vario': 0.0}

    @staticmethod
    def unpack_waypoint(data: bytes) -> Dict[str, Any]:
        """
        Unpack MSP_WP (code=118) payload.

        NOTE: INAV adds 1 byte after wp_no, so fields start one byte later than
        some older docs. Verified on payload like:
        00 04 04 4a 56 19 60 5d e8 00 23 91 01 00 00 00 00 00 00 00
        |  \____lat____/ \____lon____/ \___altHold___/ \hd/ \st/  nav
        wp

        Layout we observe (len ~= 20):
            0        : uint8   wp_no
            2..5     : int32   lat        (deg * 1e7, signed)
            6..9     : int32   lon        (deg * 1e7, signed)
            10..13   : uint32  altHold    (centimeters)  → meters below
            14..15   : uint16  heading    (deci-degrees) → degrees below
            16..17   : uint16  staytime   (seconds)
            19       : uint8   navflag    (if present; default 0)

        Returns:
            {
            'wp_no': int,
            'latitude': float (deg),
            'longitude': float (deg),
            'altitude_m': float,
            'heading_deg': float,
            'staytime_s': int,
            'navflag': int
            }
        """
        if len(data) < 20:
            logger.warning(f"Insufficient data for MSP_WP: {len(data)} bytes (expected >=20)")
            return {}

        wp_no = data[0]

        # Shifted offsets (+1 vs. older docs)
        lat_i  = struct.unpack_from('<i', data, 2)[0]
        lon_i  = struct.unpack_from('<i', data, 6)[0]
        alt_u  = struct.unpack_from('<I', data, 10)[0]
        head_u = struct.unpack_from('<H', data, 14)[0]
        stay_u = struct.unpack_from('<H', data, 16)[0]

        # navflag may be at byte 19 when len==20; guard for variable tails
        navflag = int(data[19]) if len(data) > 19 else 0

        # Conversions
        lat_deg = lat_i / 1e7
        lon_deg = lon_i / 1e7
        altitude_m = float(alt_u) / 100.0      # cm → m (absolute alt)
        heading_deg = float(head_u) / 10.0     # deci-deg → deg
        staytime_s = int(stay_u)

        result = {
            'wp_no': int(wp_no),
            'latitude': lat_deg,
            'longitude': lon_deg,
            'altitude_m': altitude_m,
            'heading_deg': heading_deg,
            'staytime_s': staytime_s,
            'navflag': navflag,
        }
        logger.debug(f"Unpacked WP: {result}")
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
