#!/usr/bin/env python3
"""
MSP Serial Handler - Manages serial communication with flight controller

This module handles:
- Serial port connection and reconnection
- Thread-safe serial I/O operations
- Message buffering and decoding
- Command queue management
"""

import serial
import threading
import time
import queue
from typing import Optional, Callable
from collections.abc import Callable as CallableABC

import rclpy
from rclpy.node import Node

from swarm_ai_integration.msp_protocol import MSPMessage, MSPDirection


class MSPSerialHandler:
    """
    Handles serial communication with flight controller using MSP protocol.

    Features:
    - Auto-reconnection on connection loss
    - Thread-safe serial operations
    - Message buffering for fragmented data
    - Command queue for outgoing messages
    """

    def __init__(
        self,
        node: Node,
        port: str,
        baudrate: int,
        timeout: float = 1.0,
        reconnect_interval: float = 5.0,
        heartbeat_timeout: float = 10.0,
        max_rx_buffer_size: int = 1024
    ):
        """
        Initialize MSP serial handler.

        Args:
            node: ROS2 node for logging
            port: Serial port path (e.g., '/dev/ttyAMA0')
            baudrate: Serial baud rate
            timeout: Serial read/write timeout
            reconnect_interval: Time between reconnection attempts
            heartbeat_timeout: Timeout for heartbeat/connection health
            max_rx_buffer_size: Maximum receive buffer size (bytes)
        """
        self.node = node
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.reconnect_interval = reconnect_interval
        self.heartbeat_timeout = heartbeat_timeout
        self.max_rx_buffer_size = max_rx_buffer_size

        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        self.last_heartbeat = time.time()

        # Threading
        self.comm_thread: Optional[threading.Thread] = None
        self.running = False
        self.command_queue = queue.Queue()

        # Message buffering
        self.rx_buffer = bytearray()

        # Callback for incoming messages
        self.message_callback: Optional[Callable[[MSPMessage], None]] = None

        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'errors': 0,
            'reconnects': 0
        }

    def set_message_callback(self, callback: Callable[[MSPMessage], None]):
        """Set callback function for incoming messages"""
        self.message_callback = callback

    def start(self):
        """Start the communication thread"""
        if self.running:
            self.node.get_logger().warn('Serial handler already running')
            return

        self.running = True
        self.comm_thread = threading.Thread(target=self._communication_worker, daemon=True)
        self.comm_thread.start()
        self.node.get_logger().info('🔄 MSP serial communication thread started')

    def stop(self):
        """Stop the communication thread"""
        self.node.get_logger().info('⏸️  Stopping MSP serial communication...')
        self.running = False

        if self.comm_thread and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=2.0)

        self.disconnect()
        self.node.get_logger().info('✅ MSP serial communication stopped')

    def connect(self) -> bool:
        """Establish serial connection to flight controller"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

            self.node.get_logger().info(f'🔌 Connecting to {self.port}...')
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )

            self.connected = True
            self.last_heartbeat = time.time()
            self.node.get_logger().info(f'✅ Connected to flight controller on {self.port}')
            return True

        except Exception as e:
            self.node.get_logger().error(f'❌ Failed to connect to {self.port}: {e}')
            self.connected = False
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                self.node.get_logger().info('🔌 Serial connection closed')
            except Exception as e:
                self.node.get_logger().debug(f'Error closing serial connection: {e}')

        self.connected = False

    def send_message(self, message: MSPMessage) -> bool:
        """
        Queue MSP message for sending.

        Args:
            message: MSP message to send

        Returns:
            True if queued successfully
        """
        self.command_queue.put(message)
        return True

    def _communication_worker(self):
        """Main communication thread worker"""
        last_reconnect_attempt = 0

        while self.running:
            try:
                # Attempt connection if not connected
                if not self.connected:
                    current_time = time.time()
                    if current_time - last_reconnect_attempt > self.reconnect_interval:
                        if self.connect():
                            self.stats['reconnects'] += 1
                            self.node.get_logger().info(f'♻️  Reconnect #{self.stats["reconnects"]}')
                        last_reconnect_attempt = current_time
                    else:
                        time.sleep(0.1)
                        continue

                # Process outgoing commands
                self._process_command_queue()

                # Read incoming data
                self._read_and_process_data()

                # Check connection health
                if time.time() - self.last_heartbeat > self.heartbeat_timeout:
                    self.node.get_logger().warn('⚠️  Flight controller communication timeout')
                    self.disconnect()

            except Exception as e:
                self.node.get_logger().error(f'❌ Communication worker error: {e}')
                self.stats['errors'] += 1
                self.disconnect()
                time.sleep(1.0)

        self.disconnect()

    def _process_command_queue(self):
        """Process outgoing MSP commands"""
        try:
            while not self.command_queue.empty():
                message = self.command_queue.get_nowait()
                self._send_msp_message(message)
        except queue.Empty:
            pass

    def _send_msp_message(self, message: MSPMessage) -> bool:
        """Send MSP message to flight controller"""
        if not self.connected or not self.serial_conn:
            return False

        try:
            encoded = message.encode()
            self.serial_conn.write(encoded)
            self.serial_conn.flush()
            self.stats['messages_sent'] += 1

            self.node.get_logger().debug(
                f'📤 Sent MSP message #{self.stats["messages_sent"]}'
            )
            return True

        except Exception as e:
            self.node.get_logger().debug(f'Failed to send MSP message: {e}')
            self.stats['errors'] += 1
            return False

    def _read_and_process_data(self):
        """Read and process incoming MSP messages"""
        if not self.connected or not self.serial_conn:
            return

        try:
            if self.serial_conn.in_waiting > 0:
                # Read available data
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self._process_incoming_data(data)

        except Exception as e:
            self.node.get_logger().debug(f'Error reading serial data: {e}')
            self.stats['errors'] += 1

    def _process_incoming_data(self, data: bytes):
        """
        Process incoming MSP data with proper buffering.

        Handles fragmented messages by accumulating data in a buffer
        and attempting to decode complete messages.
        """
        # Append new data to buffer
        self.rx_buffer.extend(data)

        # Try to extract complete messages from buffer
        while len(self.rx_buffer) >= 6:  # Minimum MSP message size
            # Look for message start marker
            header_idx = self.rx_buffer.find(b'$M')

            if header_idx == -1:
                # No message start found, clear buffer
                self.rx_buffer.clear()
                break

            # Remove any junk before the header
            if header_idx > 0:
                self.node.get_logger().debug(f'Discarding {header_idx} bytes before MSP header')
                self.rx_buffer = self.rx_buffer[header_idx:]

            # Check if we have enough data for the header
            if len(self.rx_buffer) < 6:
                # Not enough data yet, wait for more
                break

            # Parse header to determine message length
            # Format: $M + direction(1) + size(1) + command(1) + data(size) + checksum(1)
            _ = self.rx_buffer[2]  # direction (not used here)
            size = self.rx_buffer[3]
            expected_length = 6 + size  # header(3) + size(1) + command(1) + data(size) + checksum(1)

            if len(self.rx_buffer) < expected_length:
                # Incomplete message, wait for more data
                self.node.get_logger().debug(
                    f'Buffering partial MSP message: have {len(self.rx_buffer)} bytes, '
                    f'need {expected_length} bytes (size={size})'
                )
                break

            # Extract complete message from buffer
            message_bytes = bytes(self.rx_buffer[:expected_length])

            try:
                # Try to decode the message
                message = MSPMessage.decode(message_bytes)

                if message:
                    # Successfully decoded, handle it
                    if self.message_callback:
                        self.message_callback(message)

                    self.stats['messages_received'] += 1
                    self.last_heartbeat = time.time()

                    # Remove processed message from buffer
                    self.rx_buffer = self.rx_buffer[expected_length:]
                else:
                    # Decoding failed (checksum error, etc.)
                    # Skip past the bad header and try again
                    self.node.get_logger().debug('Failed to decode message, skipping header')
                    self.rx_buffer = self.rx_buffer[3:]

            except Exception as e:
                self.node.get_logger().debug(f'Error decoding MSP message: {e}')
                # Skip past the bad header and try again
                self.rx_buffer = self.rx_buffer[3:]

        # Prevent buffer from growing indefinitely
        if len(self.rx_buffer) > self.max_rx_buffer_size:
            self.node.get_logger().warn(
                f'RX buffer too large ({len(self.rx_buffer)} bytes), clearing. '
                'This may indicate communication problems.'
            )
            self.rx_buffer.clear()

    def get_stats(self) -> dict:
        """Get communication statistics"""
        return self.stats.copy()

    def is_connected(self) -> bool:
        """Check if connected to flight controller"""
        return self.connected
