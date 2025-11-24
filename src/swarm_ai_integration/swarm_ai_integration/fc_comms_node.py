#!/usr/bin/env python3
"""
Flight Controller Communications Node (Refactored)

Manages bidirectional MSP communication with INAV 7 flight controller.
This node coordinates between modular components for clean separation of concerns:
- MSPSerialHandler: Serial communication & threading
- MSPMessageParser: Data parsing & conversion
- TelemetryPublisher: ROS message publishing

Architecture:
    FC (Serial/MSP) ←→ MSPSerialHandler ←→ FCCommsNode ←→ MSPMessageParser ←→ TelemetryPublisher ←→ ROS Topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from swarm_ai_integration.msp_protocol import MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
from swarm_ai_integration.utils import MSPSerialHandler, MSPMessageParser, TelemetryPublisher


class FCCommsNode(Node):
    """
    Flight Controller Communications Node.

    Coordinates MSP communication by orchestrating modular components:
    - Serial handler for I/O operations
    - Message parser for data conversion
    - Telemetry publisher for ROS integration
    """

    def __init__(self):
        super().__init__('fc_comms_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('heartbeat_timeout', 10.0)
        self.declare_parameter('connection_timeout', 10.0)
        self.declare_parameter('waypoint_poll_mode', 'first')  # 'first' or 'all' or 'none'
        self.declare_parameter('max_waypoint_cycle', 10)
        self.declare_parameter('command_qos_depth', 10)
        self.declare_parameter('max_rx_buffer_size', 1024)

        # Data processing scale factors
        self.declare_parameter('imu_scale_accel', 9.81 / 512.0)  # IMU accelerometer scale
        self.declare_parameter('imu_scale_gyro', 0.001)  # IMU gyroscope scale
        self.declare_parameter('battery_voltage_scale', 10.0)  # Battery voltage divisor
        self.declare_parameter('battery_current_scale', 100.0)  # Battery current divisor

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.telemetry_rate = self.get_parameter('telemetry_rate').get_parameter_value().double_value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').get_parameter_value().double_value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.wp_poll_mode = self.get_parameter('waypoint_poll_mode').get_parameter_value().string_value
        self.max_waypoint_cycle = self.get_parameter('max_waypoint_cycle').get_parameter_value().integer_value
        self.command_qos_depth = self.get_parameter('command_qos_depth').get_parameter_value().integer_value
        self.max_rx_buffer_size = self.get_parameter('max_rx_buffer_size').get_parameter_value().integer_value

        # Get scale factors
        self.imu_scale_accel = self.get_parameter('imu_scale_accel').get_parameter_value().double_value
        self.imu_scale_gyro = self.get_parameter('imu_scale_gyro').get_parameter_value().double_value
        self.battery_voltage_scale = self.get_parameter('battery_voltage_scale').get_parameter_value().double_value
        self.battery_current_scale = self.get_parameter('battery_current_scale').get_parameter_value().double_value

        # Initialize modular components
        self.serial_handler = MSPSerialHandler(
            node=self,
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
            reconnect_interval=self.reconnect_interval,
            heartbeat_timeout=self.heartbeat_timeout,
            max_rx_buffer_size=self.max_rx_buffer_size
        )

        self.parser = MSPMessageParser(
            imu_scale_accel=self.imu_scale_accel,
            imu_scale_gyro=self.imu_scale_gyro,
            battery_voltage_scale=self.battery_voltage_scale,
            battery_current_scale=self.battery_current_scale
        )
        self.publisher = TelemetryPublisher(self)

        # Set callback for incoming MSP messages
        self.serial_handler.set_message_callback(self.handle_msp_response)

        # Telemetry request sequence
        self.telemetry_commands = [
            MSPCommand.MSP_RAW_IMU,
            MSPCommand.MSP_RAW_GPS,
            MSPCommand.MSP_ATTITUDE,
            MSPCommand.MSP_ALTITUDE,
            MSPCommand.MSP_STATUS,
            MSPCommand.MSP_ANALOG,
            MSPCommand.MSP_MOTOR
        ]
        self.current_telemetry_index = 0

        # Waypoint polling
        self.wp_poll_index = 1  # Start at WP #1

        # Statistics tracking
        self.stats = {
            'unknown_commands': {}
        }

        # Create timers
        telemetry_period = 1.0 / self.telemetry_rate
        self.telemetry_timer = self.create_timer(telemetry_period, self.request_telemetry)

        heartbeat_period = 1.0 / self.heartbeat_rate
        self.heartbeat_timer = self.create_timer(heartbeat_period, self.send_heartbeat)

        # Subscribers for outgoing commands
        self.create_subscription(
            Float32MultiArray,
            '/fc/msp_command',
            self.msp_command_callback,
            self.command_qos_depth
        )

        self.create_subscription(
            Float32MultiArray,
            '/fc/rc_override',
            self.rc_override_callback,
            self.command_qos_depth
        )

        # Start serial communication
        self.serial_handler.start()

        self.get_logger().info('═' * 60)
        self.get_logger().info(f'🚀 FC Communications Node Started')
        self.get_logger().info(f'   Serial Port: {self.serial_port}')
        self.get_logger().info(f'   Baud Rate: {self.baud_rate}')
        self.get_logger().info(f'   Telemetry Rate: {self.telemetry_rate} Hz')
        self.get_logger().info(f'   Waypoint Polling: {self.wp_poll_mode}')
        self.get_logger().info('═' * 60)

    # ═══════════════════════════════════════════════════════════════════
    # Timer Callbacks - Telemetry Requests
    # ═══════════════════════════════════════════════════════════════════

    def request_telemetry(self):
        """Request telemetry data from flight controller (round-robin)"""
        if not self.serial_handler.is_connected():
            return

        # Round-robin through telemetry commands
        command = self.telemetry_commands[self.current_telemetry_index]
        self.current_telemetry_index = (self.current_telemetry_index + 1) % len(self.telemetry_commands)

        # Send request
        message = MSPMessage(command, b'', MSPDirection.REQUEST)
        self.serial_handler.send_message(message)

        # Also request waypoint data
        self.request_waypoint()

    def request_waypoint(self):
        """Request waypoint data based on polling mode"""
        if self.wp_poll_mode == 'first':
            # Always poll WP #1 (first mission waypoint)
            wp_payload = MSPDataTypes.pack_waypoint_request(1)
        elif self.wp_poll_mode == 'all':
            # Cycle through WP #0 to #N (would need to know max from mission)
            wp_payload = MSPDataTypes.pack_waypoint_request(self.wp_poll_index)
            self.wp_poll_index = (self.wp_poll_index % self.max_waypoint_cycle) + 1  # Cycle 1-max
        else:
            return

        wp_message = MSPMessage(MSPCommand.MSP_WP, wp_payload, MSPDirection.REQUEST)
        self.serial_handler.send_message(wp_message)

    def send_heartbeat(self):
        """
        Send periodic heartbeat to flight controller.

        Note: MSP_IDENT was removed in INAV 5.x, so we just publish
        connection status based on serial handler state.
        """
        if not self.serial_handler.is_connected():
            return

        # Publish connection status (no MSP_IDENT needed for INAV 8.x)
        self.publisher.publish_connection_status(True)

    # ═══════════════════════════════════════════════════════════════════
    # Incoming Message Handling
    # ═══════════════════════════════════════════════════════════════════

    def handle_msp_response(self, message: MSPMessage):
        """
        Handle incoming MSP response messages.

        This is the main routing function that delegates to specific handlers
        based on message type.
        """
        cmd_name = self._get_command_name(message.command)

        # Log MSP_SET_RAW_RC at DEBUG level to reduce log noise
        if message.command == MSPCommand.MSP_SET_RAW_RC:
            self.get_logger().debug(f'📥 Received: {cmd_name} (code={message.command}, size={message.size} bytes)')
        else:
            self.get_logger().info(f'📥 Received: {cmd_name} (code={message.command}, size={message.size} bytes)')

        try:
            # Route to appropriate handler
            if message.command == MSPCommand.MSP_RAW_IMU:
                self._handle_imu(message.data)

            elif message.command == MSPCommand.MSP_RAW_GPS:
                self._handle_gps(message.data)

            elif message.command == MSPCommand.MSP_ATTITUDE:
                self._handle_attitude(message.data)

            elif message.command == MSPCommand.MSP_ALTITUDE:
                self._handle_altitude(message.data)

            elif message.command == MSPCommand.MSP_STATUS:
                self._handle_status(message.data)

            elif message.command == MSPCommand.MSP_ANALOG:
                self._handle_battery(message.data)

            elif message.command == MSPCommand.MSP_MOTOR:
                self._handle_motor(message.data)

            elif message.command == MSPCommand.MSP_WP:
                self._handle_waypoint(message.data)

            elif message.command == MSPCommand.MSP_RC:
                self._handle_rc(message.data)

            elif message.command == MSPCommand.MSP_SET_RAW_RC:
                # RC command ACK - just log at debug level (no data to process)
                self.get_logger().debug(f'✓ RC command acknowledged')

            else:
                # Unknown or unhandled command
                self._handle_unknown_command(message)

        except Exception as e:
            self.get_logger().error(f'❌ Error handling {cmd_name}: {e}')
            import traceback
            self.get_logger().error(f'   Traceback: {traceback.format_exc()}')

    # ═══════════════════════════════════════════════════════════════════
    # Specific Message Handlers (using Parser & Publisher)
    # ═══════════════════════════════════════════════════════════════════

    def _handle_imu(self, data: bytes):
        """Handle IMU data"""
        imu_msg = self.parser.parse_imu_data(
            data,
            self.get_clock().now().to_msg(),
            'fc_imu'
        )
        if imu_msg:
            self.publisher.publish_imu(imu_msg)

    def _handle_gps(self, data: bytes):
        """Handle GPS data"""
        gps_msg, speed_msg, sat_msg, hdop_msg = self.parser.parse_gps_data(
            data,
            self.get_clock().now().to_msg(),
            'fc_gps'
        )
        if gps_msg:
            self.publisher.publish_gps(gps_msg, speed_msg, sat_msg, hdop_msg)

    def _handle_attitude(self, data: bytes):
        """Handle attitude data"""
        euler_msg = self.parser.parse_attitude_data(
            data,
            self.get_clock().now().to_msg(),
            'fc_attitude'
        )
        if euler_msg:
            self.publisher.publish_attitude(euler_msg)

    def _handle_altitude(self, data: bytes):
        """Handle altitude data"""
        altitude_msg = self.parser.parse_altitude_data(data)
        if altitude_msg:
            self.publisher.publish_altitude(altitude_msg)

    def _handle_status(self, data: bytes):
        """Handle status data"""
        status_msg, msp_status_msg = self.parser.parse_status_data(
            data,
            self.get_clock().now().to_msg()
        )
        if status_msg:
            self.publisher.publish_status(status_msg, msp_status_msg)

    def _handle_battery(self, data: bytes):
        """Handle battery data"""
        battery_msg = self.parser.parse_battery_data(
            data,
            self.get_clock().now().to_msg(),
            'fc_battery'
        )
        if battery_msg:
            self.publisher.publish_battery(battery_msg)

    def _handle_motor(self, data: bytes):
        """Handle motor data"""
        motor_msg = self.parser.parse_motor_data(data)
        if motor_msg:
            self.publisher.publish_motor(motor_msg)

    def _handle_waypoint(self, data: bytes):
        """Handle waypoint data"""
        waypoint_msg = self.parser.parse_waypoint_data(data)
        if waypoint_msg:
            self.publisher.publish_waypoint(waypoint_msg)

    def _handle_rc(self, data: bytes):
        """Handle RC channel data"""
        channels = self.parser.parse_rc_data(data)
        if channels:
            self.get_logger().debug(f'   RC Channels: {channels}')

    def _handle_unknown_command(self, message: MSPMessage):
        """
        Handle unknown or unhandled MSP commands with comprehensive logging.

        This helps identify:
        - New MSP commands from firmware updates
        - Responses to commands we sent but didn't expect
        - Protocol errors or unexpected messages
        """
        cmd_name = self._get_command_name(message.command)

        # Check if this is a known MSP command from the protocol but not handled
        try:
            _ = MSPCommand(message.command)  # Check if it exists in enum
            # It's a known command code but we don't have a handler for it
            self.get_logger().warn(
                f'⚠️  Unhandled MSP command: {cmd_name} (code={message.command})\n'
                f'   Direction: {self._get_direction_name(message.direction)}\n'
                f'   Size: {message.size} bytes\n'
                f'   Raw data (hex): {message.data.hex() if message.data else "empty"}\n'
                f'   Note: This is a known MSP command but no handler is implemented.'
            )
        except ValueError:
            # Unknown command code - not in our MSPCommand enum
            self.get_logger().warn(
                f'⚠️  Unknown MSP command code: {message.command} (0x{message.command:02x})\n'
                f'   Direction: {self._get_direction_name(message.direction)}\n'
                f'   Size: {message.size} bytes\n'
                f'   Raw data (hex): {message.data.hex() if message.data else "empty"}\n'
                f'   Note: This command code is not defined in MSPCommand enum.\n'
                f'   This could be:\n'
                f'     - A new INAV 7 command not yet in our protocol definition\n'
                f'     - A custom flight controller command\n'
                f'     - Corrupted data or protocol error'
            )

        # Log to statistics for monitoring
        cmd_key = f'{message.command}_{cmd_name}'
        self.stats['unknown_commands'][cmd_key] = self.stats['unknown_commands'].get(cmd_key, 0) + 1

        # Periodically report statistics on unknown commands
        total_unknown = sum(self.stats['unknown_commands'].values())
        if total_unknown % 10 == 1:  # Report every 10 unknown messages
            self.get_logger().info(
                f'📊 Unknown command statistics (total: {total_unknown}):\n' +
                '\n'.join([f'   {k}: {v} occurrences' for k, v in self.stats['unknown_commands'].items()])
            )

    # ═══════════════════════════════════════════════════════════════════
    # Outgoing Command Subscriptions
    # ═══════════════════════════════════════════════════════════════════

    def msp_command_callback(self, msg: Float32MultiArray):
        """Handle raw MSP command requests"""
        if len(msg.data) < 1:
            return

        command_code = int(msg.data[0])
        payload = bytes([int(x) for x in msg.data[1:]]) if len(msg.data) > 1 else b''

        message = MSPMessage(command_code, payload, MSPDirection.REQUEST)
        self.serial_handler.send_message(message)

        self.get_logger().info(f'📤 Sent custom MSP command: {command_code}')

    def rc_override_callback(self, msg: Float32MultiArray):
        """Handle RC override commands"""
        if len(msg.data) < 4:
            self.get_logger().warn('RC override requires at least 4 channels')
            return

        channels = [int(x) for x in msg.data]
        payload = MSPDataTypes.pack_rc_channels(channels)

        message = MSPMessage(MSPCommand.MSP_SET_RAW_RC, payload, MSPDirection.REQUEST)
        self.serial_handler.send_message(message)

        self.get_logger().debug(f'📤 Sent RC override: {channels}')

    # ═══════════════════════════════════════════════════════════════════
    # Helper Methods
    # ═══════════════════════════════════════════════════════════════════

    def _get_command_name(self, command: int) -> str:
        """Get command name from code"""
        try:
            return MSPCommand(command).name
        except ValueError:
            return f'UNKNOWN_{command}'

    def _get_direction_name(self, direction: int) -> str:
        """Get human-readable direction name"""
        if direction == MSPDirection.REQUEST:
            return 'REQUEST (<)'
        elif direction == MSPDirection.RESPONSE:
            return 'RESPONSE (>)'
        elif direction == MSPDirection.ERROR:
            return 'ERROR (!)'
        else:
            return f'UNKNOWN (0x{direction:02x})'

    # ═══════════════════════════════════════════════════════════════════
    # Node Lifecycle
    # ═══════════════════════════════════════════════════════════════════

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('🛑 Shutting down FC Communications Node...')
        self.serial_handler.stop()

        # Print final statistics
        stats = self.serial_handler.get_stats()
        self.get_logger().info(
            f'📊 Final Statistics:\n'
            f'   Messages Sent: {stats["messages_sent"]}\n'
            f'   Messages Received: {stats["messages_received"]}\n'
            f'   Errors: {stats["errors"]}\n'
            f'   Reconnects: {stats["reconnects"]}'
        )

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
