#!/usr/bin/env python3
"""
Flight Controller Communications Node - MSP Interface for INAV 7

This node manages bidirectional communication with INAV 7 flight controllers
using the MultiWii Serial Protocol (MSP). It handles:
- Sending control commands to the FC
- Receiving telemetry data (IMU, GPS, status)
- Managing connection health and recovery
- Protocol-level error handling

The node maintains persistent connection with the flight controller and
provides ROS2 interfaces for high-level control and monitoring.
"""

import serial
import threading
import time
import queue
from typing import Optional, Dict, Any, List
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String, Float32MultiArray, Header
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, QuaternionStamped
from nav_msgs.msg import Odometry

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes,
    MSPProtocolError, MSPTimeoutError, MSPChecksumError
)


class FCCommsNode(Node):
    """
    Flight Controller Communication Node using MSP protocol for INAV 7.

    Subscribers:
        /fc/msp_command (std_msgs/Float32MultiArray): Raw MSP commands
        /fc/rc_override (std_msgs/Float32MultiArray): RC channel overrides

    Publishers:
        /fc/imu_raw (sensor_msgs/Imu): IMU data from flight controller
        /fc/gps_fix (sensor_msgs/NavSatFix): GPS data from flight controller
        /fc/attitude (geometry_msgs/QuaternionStamped): Attitude quaternion (from roll, pitch, yaw)
        /fc/attitude_euler (geometry_msgs/Vector3Stamped): Attitude Euler angles (roll, pitch, yaw)
        /fc/status (std_msgs/String): Flight controller status
        /fc/battery (sensor_msgs/BatteryState): Battery status
        /fc/connected (std_msgs/Bool): Connection status
        /fc/motor_rpm (std_msgs/Float32MultiArray): Motor RPM data
    """

    def __init__(self):
        super().__init__('fc_comms_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('telemetry_rate', 10.0)  # Hz
        self.declare_parameter('heartbeat_rate', 1.0)   # Hz

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.telemetry_rate = self.get_parameter('telemetry_rate').get_parameter_value().double_value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').get_parameter_value().double_value

        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        self.last_heartbeat = time.time()

        # Threading
        self.comm_thread: Optional[threading.Thread] = None
        self.running = False
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()

        # Data storage
        self.last_telemetry = {
            'imu': None,
            'gps': None,
            'attitude': None,
            'attitude_euler': None,
            'status': None,
            'battery': None
        }

        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'errors': 0,
            'reconnects': 0
        }

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.msp_cmd_sub = self.create_subscription(
            Float32MultiArray, '/fc/msp_command', self.msp_command_callback, reliable_qos)
        self.rc_override_sub = self.create_subscription(
            Float32MultiArray, '/fc/rc_override', self.rc_override_callback, reliable_qos)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/fc/imu_raw', sensor_qos)
        self.gps_pub = self.create_publisher(NavSatFix, '/fc/gps_fix', sensor_qos)
        self.attitude_pub = self.create_publisher(QuaternionStamped, '/fc/attitude', sensor_qos)
        self.attitude_euler_pub = self.create_publisher(Vector3Stamped, '/fc/attitude_euler', sensor_qos)
        self.status_pub = self.create_publisher(String, '/fc/status', reliable_qos)
        self.battery_pub = self.create_publisher(BatteryState, '/fc/battery', sensor_qos)
        self.connected_pub = self.create_publisher(Bool, '/fc/connected', reliable_qos)
        self.motor_rpm_pub = self.create_publisher(Float32MultiArray, '/fc/motor_rpm', sensor_qos)

        # Timers
        self.telemetry_timer = self.create_timer(1.0 / self.telemetry_rate, self.request_telemetry)
        self.heartbeat_timer = self.create_timer(1.0 / self.heartbeat_rate, self.send_heartbeat)
        self.status_timer = self.create_timer(1.0, self.publish_connection_status)

        # Start communication thread
        self.start_communication()

        self.get_logger().info(f'🚀 FC Communications Node initialized')
        self.get_logger().info(f'📡 Serial: {self.serial_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'📊 Telemetry rate: {self.telemetry_rate} Hz')

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> tuple:
        """
        Convert Euler angles (roll, pitch, yaw) in radians to quaternion (x, y, z, w)
        Using the aerospace sequence (ZYX): yaw -> pitch -> roll
        
        Args:
            roll: rotation around x-axis in radians
            pitch: rotation around y-axis in radians
            yaw: rotation around z-axis in radians
            
        Returns:
            tuple: (qx, qy, qz, qw)
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

    def start_communication(self):
        """Start the communication thread"""
        self.running = True
        self.comm_thread = threading.Thread(target=self.communication_worker)
        self.comm_thread.daemon = True
        self.comm_thread.start()
        self.get_logger().info('🔄 Communication thread started')

    def stop_communication(self):
        """Stop the communication thread"""
        self.get_logger().info('⏸️  Stopping communication thread...')
        self.running = False
        if self.comm_thread and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=2.0)
        self.get_logger().info('✅ Communication thread stopped')

    def connect_serial(self) -> bool:
        """Establish serial connection to flight controller"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

            self.get_logger().info(f'🔌 Connecting to {self.serial_port}...')
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )

            self.connected = True
            self.last_heartbeat = time.time()
            self.get_logger().info(f'✅ Connected to flight controller on {self.serial_port}')
            return True

        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to flight controller: {e}')
            self.connected = False
            return False

    def disconnect_serial(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                self.get_logger().info('🔌 Serial connection closed')
            except Exception as e:
                self.get_logger().debug(f'Error closing serial connection: {e}')

        self.connected = False

    def communication_worker(self):
        """Main communication thread worker"""
        last_reconnect_attempt = 0

        while self.running:
            try:
                # Attempt connection if not connected
                if not self.connected:
                    current_time = time.time()
                    if current_time - last_reconnect_attempt > self.reconnect_interval:
                        if self.connect_serial():
                            self.stats['reconnects'] += 1
                            self.get_logger().info(f'♻️  Reconnect #{self.stats["reconnects"]}')
                        last_reconnect_attempt = current_time
                    else:
                        time.sleep(0.1)
                        continue

                # Process outgoing commands
                self.process_command_queue()

                # Read incoming data
                self.read_incoming_data()

                # Check connection health
                if time.time() - self.last_heartbeat > 10.0:  # 10 second timeout
                    self.get_logger().warn('⚠️  Flight controller communication timeout')
                    self.disconnect_serial()

            except Exception as e:
                self.get_logger().error(f'❌ Communication worker error: {e}')
                self.stats['errors'] += 1
                self.disconnect_serial()
                time.sleep(1.0)

        self.disconnect_serial()

    def process_command_queue(self):
        """Process outgoing MSP commands"""
        try:
            while not self.command_queue.empty():
                message = self.command_queue.get_nowait()
                self.send_msp_message(message)
        except queue.Empty:
            pass

    def send_msp_message(self, message: MSPMessage) -> bool:
        """Send MSP message to flight controller"""
        if not self.connected or not self.serial_conn:
            return False

        try:
            encoded = message.encode()
            self.serial_conn.write(encoded)
            self.serial_conn.flush()
            self.stats['messages_sent'] += 1
            
            cmd_name = self._get_command_name(message.command)
            self.get_logger().debug(f'📤 Sent: {cmd_name} (#{self.stats["messages_sent"]})')
            return True

        except Exception as e:
            self.get_logger().debug(f'Failed to send MSP message: {e}')
            self.stats['errors'] += 1
            return False

    def read_incoming_data(self):
        """Read and process incoming MSP messages"""
        if not self.connected or not self.serial_conn:
            return

        try:
            if self.serial_conn.in_waiting > 0:
                # Read available data
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self.process_incoming_data(data)

        except Exception as e:
            self.get_logger().debug(f'Error reading serial data: {e}')
            self.stats['errors'] += 1

    def process_incoming_data(self, data: bytes):
        """Process incoming MSP data"""
        # Simple message extraction - in production, implement proper buffering
        for i in range(len(data) - 5):
            if data[i:i+2] == b'$M':
                try:
                    # Try to decode message starting at this position
                    remaining = data[i:]
                    message = MSPMessage.decode(remaining)
                    if message:
                        self.handle_msp_response(message)
                        self.stats['messages_received'] += 1
                        self.last_heartbeat = time.time()
                except Exception as e:
                    self.get_logger().debug(f'Error decoding MSP message: {e}')

    def handle_msp_response(self, message: MSPMessage):
        """Handle incoming MSP response messages"""
        cmd_name = self._get_command_name(message.command)
        self.get_logger().info(f'📥 Received: {cmd_name} (code={message.command}, size={message.size} bytes)')
        
        try:
            if message.command == MSPCommand.MSP_RAW_IMU:
                self.handle_imu_data(message.data)
            elif message.command == MSPCommand.MSP_RAW_GPS:
                self.handle_gps_data(message.data)
            elif message.command == MSPCommand.MSP_ATTITUDE:
                self.handle_attitude_data(message.data)
            elif message.command == MSPCommand.MSP_STATUS:
                self.handle_status_data(message.data)
            elif message.command == MSPCommand.MSP_ANALOG:
                self.handle_battery_data(message.data)
            elif message.command == MSPCommand.MSP_MOTOR:
                self.handle_motor_data(message.data)
            else:
                self.get_logger().debug(f'   ℹ️  Unhandled message type: {cmd_name}')

        except Exception as e:
            self.get_logger().error(f'❌ Error handling {cmd_name}: {e}')

    def handle_imu_data(self, data: bytes):
        """Handle IMU data from flight controller"""
        imu_data = MSPDataTypes.unpack_raw_imu(data)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'fc_imu'

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

        # Don't set covariances (leave as zeros - unknown)
        # Arrays are already initialized to zeros by default

        self.imu_pub.publish(imu_msg)
        self.last_telemetry['imu'] = imu_msg
        
        self.get_logger().info(f'   ➜ Published to /fc/imu_raw | '
                              f'acc=[{imu_msg.linear_acceleration.x:.2f}, {imu_msg.linear_acceleration.y:.2f}, {imu_msg.linear_acceleration.z:.2f}] m/s² | '
                              f'gyro=[{imu_msg.angular_velocity.x:.2f}, {imu_msg.angular_velocity.y:.2f}, {imu_msg.angular_velocity.z:.2f}] rad/s')

    def handle_gps_data(self, data: bytes):
        """Handle GPS data from flight controller"""
        gps_data = MSPDataTypes.unpack_gps_data(data)

        if not gps_data:
            self.get_logger().warn('   ⚠️  Empty GPS data received')
            return

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'fc_gps'

        gps_msg.latitude = gps_data['latitude']
        gps_msg.longitude = gps_data['longitude']
        gps_msg.altitude = gps_data['altitude']  # meters (no /100)

        fix_type_value = gps_data.get('fix_type', 0)
        # Map: 0=NO_FIX, 1=2D, 2=3D (treat >=3 as 3D as well)
        if fix_type_value >= 2:
            gps_msg.status.status = gps_msg.status.STATUS_FIX
            fix_type = '3D_FIX'
        elif fix_type_value == 1:
            gps_msg.status.status = gps_msg.status.STATUS_SBAS_FIX
            fix_type = '2D_FIX'
        else:
            gps_msg.status.status = gps_msg.status.STATUS_NO_FIX
            fix_type = 'NO_FIX'

        gps_msg.status.service = gps_msg.status.SERVICE_GPS

        # Don't set covariances (leave as zeros - unknown)
        gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(gps_msg)
        self.last_telemetry['gps'] = gps_msg
        
        self.get_logger().info(
            f'   ➜ Published to /fc/gps_fix | '
            f'lat={gps_msg.latitude:.6f}° lon={gps_msg.longitude:.6f}° | '
            f'alt={gps_msg.altitude:.1f}m | '
            f'sats={gps_data["satellites"]} ({fix_type})'
        )

    def handle_attitude_data(self, data: bytes):
        """Handle attitude data from flight controller"""
        roll, pitch, yaw = MSPDataTypes.unpack_attitude(data)

        # Convert to radians
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)

        # Create timestamp
        stamp = self.get_clock().now().to_msg()

        # Publish Euler angles
        euler_msg = Vector3Stamped()
        euler_msg.header.stamp = stamp
        euler_msg.header.frame_id = 'fc_attitude'
        euler_msg.vector.x = roll_rad   # roll
        euler_msg.vector.y = pitch_rad  # pitch
        euler_msg.vector.z = yaw_rad    # yaw

        self.attitude_euler_pub.publish(euler_msg)
        self.last_telemetry['attitude_euler'] = euler_msg

        # Convert to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # Publish quaternion
        quat_msg = QuaternionStamped()
        quat_msg.header.stamp = stamp
        quat_msg.header.frame_id = 'fc_attitude'
        quat_msg.quaternion.x = qx
        quat_msg.quaternion.y = qy
        quat_msg.quaternion.z = qz
        quat_msg.quaternion.w = qw

        self.attitude_pub.publish(quat_msg)
        self.last_telemetry['attitude'] = quat_msg
        
        self.get_logger().info(f'   ➜ Published to /fc/attitude (quaternion) | '
                              f'q=[x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}]')
        self.get_logger().info(f'   ➜ Published to /fc/attitude_euler | '
                              f'roll={roll:.1f}° pitch={pitch:.1f}° yaw={yaw:.1f}°')

    def handle_status_data(self, data: bytes):
        """Handle flight controller status"""
        status_data = MSPDataTypes.unpack_status(data)

        if status_data:
            status_msg = String()
            status_msg.data = f"Cycle: {status_data['cycle_time']}us, " \
                             f"I2C Errors: {status_data['i2c_errors']}, " \
                             f"Sensors: 0x{status_data['sensor_flags']:04x}, " \
                             f"Mode: 0x{status_data['flight_mode_flags']:04x}"

            self.status_pub.publish(status_msg)
            self.last_telemetry['status'] = status_msg
            
            self.get_logger().info(f'   ➜ Published to /fc/status | '
                                  f'cycle={status_data["cycle_time"]}µs | '
                                  f'i2c_err={status_data["i2c_errors"]} | '
                                  f'sensors=0x{status_data["sensor_flags"]:04x} | '
                                  f'mode=0x{status_data["flight_mode_flags"]:04x}')

    def handle_battery_data(self, data: bytes):
        """Handle battery/analog data"""
        if len(data) >= 7:
            vbat, power_meter_sum, rssi, amperage = data[:4]

            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.header.frame_id = 'fc_battery'

            battery_msg.voltage = float(vbat / 10.0)  # Convert to volts
            battery_msg.current = float(amperage / 100.0)  # Convert to amps
            battery_msg.power_supply_status = battery_msg.POWER_SUPPLY_STATUS_UNKNOWN
            battery_msg.power_supply_technology = battery_msg.POWER_SUPPLY_TECHNOLOGY_LIPO

            self.battery_pub.publish(battery_msg)
            self.last_telemetry['battery'] = battery_msg
            
            power = battery_msg.voltage * battery_msg.current
            self.get_logger().info(f'   ➜ Published to /fc/battery | '
                                  f'voltage={battery_msg.voltage:.2f}V | '
                                  f'current={battery_msg.current:.2f}A | '
                                  f'power={power:.2f}W | '
                                  f'rssi={rssi}')
        else:
            self.get_logger().warn(f'   ⚠️  Insufficient battery data: {len(data)} bytes')

    def handle_motor_data(self, data: bytes):
        """Handle motor RPM/command data"""
        if len(data) >= 8:  # 4 motors × 2 bytes each
            motor_values = []
            for i in range(0, min(8, len(data)), 2):
                motor_value = int.from_bytes(data[i:i+2], 'little')
                motor_values.append(float(motor_value))

            motor_msg = Float32MultiArray()
            motor_msg.data = motor_values
            self.motor_rpm_pub.publish(motor_msg)
            
            motors_str = ', '.join([f'M{i+1}={int(v)}' for i, v in enumerate(motor_values)])
            self.get_logger().info(f'   ➜ Published to /fc/motor_rpm | {motors_str}')
        else:
            self.get_logger().warn(f'   ⚠️  Insufficient motor data: {len(data)} bytes')

    def msp_command_callback(self, msg: Float32MultiArray):
        """Handle raw MSP command requests"""
        if len(msg.data) < 1:
            self.get_logger().warn('⚠️  Received empty MSP command')
            return

        command = int(msg.data[0])
        payload = bytes([int(x) for x in msg.data[1:]])
        cmd_name = self._get_command_name(command)

        msp_msg = MSPMessage(command, payload)
        self.command_queue.put(msp_msg)
        
        self.get_logger().info(f'🎮 Queued MSP command from /fc/msp_command: {cmd_name} (code={command}, payload={len(payload)} bytes)')

    def rc_override_callback(self, msg: Float32MultiArray):
        """Handle RC channel override commands"""
        if len(msg.data) >= 4:
            channels = [int(x) for x in msg.data]
            # Pad to 8 channels if needed
            while len(channels) < 8:
                channels.append(1500)

            payload = MSPDataTypes.pack_rc_channels(channels)
            msp_msg = MSPMessage(MSPCommand.MSP_SET_RAW_RC, payload)
            self.command_queue.put(msp_msg)
            
            ch_str = f'[{channels[0]}, {channels[1]}, {channels[2]}, {channels[3]}]'
            self.get_logger().info(f'🎮 Queued RC override from /fc/rc_override: RPYT={ch_str}')
        else:
            self.get_logger().warn(f'⚠️  Invalid RC override data: {len(msg.data)} values (need at least 4)')

    def request_telemetry(self):
        """Request telemetry data from flight controller"""
        if not self.connected:
            return

        # Request different telemetry data in rotation
        commands = [
            MSPCommand.MSP_RAW_IMU,
            MSPCommand.MSP_RAW_GPS,
            MSPCommand.MSP_ATTITUDE,
            MSPCommand.MSP_STATUS,
            MSPCommand.MSP_ANALOG,
            MSPCommand.MSP_MOTOR
        ]

        # Send one command per cycle to avoid overwhelming the FC
        current_time = int(time.time() * self.telemetry_rate)
        command = commands[current_time % len(commands)]

        msp_msg = MSPMessage(command)
        self.command_queue.put(msp_msg)

    def send_heartbeat(self):
        """Send heartbeat to maintain connection"""
        if self.connected:
            # Request FC status as heartbeat
            msp_msg = MSPMessage(MSPCommand.MSP_IDENT)
            self.command_queue.put(msp_msg)
            self.get_logger().debug('💓 Heartbeat sent')

    def publish_connection_status(self):
        """Publish connection status"""
        conn_msg = Bool()
        conn_msg.data = self.connected
        self.connected_pub.publish(conn_msg)
        
        status = '🟢 CONNECTED' if self.connected else '🔴 DISCONNECTED'
        self.get_logger().debug(f'{status} | Sent: {self.stats["messages_sent"]} | '
                               f'Received: {self.stats["messages_received"]} | '
                               f'Errors: {self.stats["errors"]}')

    def _get_command_name(self, command: int) -> str:
        """Get command name from code"""
        try:
            return MSPCommand(command).name
        except ValueError:
            return f"UNKNOWN_{command}"

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info('🛑 Shutting down FC Communications Node...')
        self.get_logger().info(f'📊 Final stats - Sent: {self.stats["messages_sent"]}, '
                              f'Received: {self.stats["messages_received"]}, '
                              f'Errors: {self.stats["errors"]}, '
                              f'Reconnects: {self.stats["reconnects"]}')
        self.stop_communication()
        self.disconnect_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FCCommsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()