#!/usr/bin/env python3
"""
LiDAR Reader Node - NLink_TOFSense Protocol Implementation

This node reads distance data from NLink TOFSense LiDAR sensors using the
NLink_TOFSense_Frame0 protocol over UART. It supports multiple sensor instances
for different mounting positions (front, down, etc.).

Protocol Format:
- Header: 0x57 0x00
- ID: sensor identifier
- System Time: 4 bytes (ms)
- Distance: 4 bytes (raw value / 256 / 1000 = meters)
- Status: 1 byte
- Signal Strength: 2 bytes
- Range Precision: 1 byte
- Checksum: 1 byte (sum of bytes 0-14)

Total frame size: 16 bytes

The node publishes sensor_msgs/Range messages containing distance measurements
and provides diagnostic information about sensor health and data quality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Range
from std_msgs.msg import Header, Float32MultiArray, String, Bool
from geometry_msgs.msg import PointStamped

import serial
import struct
import time
import threading
from typing import Optional, Tuple, Dict, Any
from collections import deque
import numpy as np


class LidarReaderNode(Node):
    """
    LiDAR Reader Node for NLink TOFSense sensors.

    This node handles serial communication with NLink TOFSense LiDAR sensors,
    parses the proprietary protocol, and publishes ROS2 Range messages.

    Publishers:
        /lidar_distance (sensor_msgs/Range): Distance measurements
        /lidar_raw (std_msgs/Float32MultiArray): Raw sensor data
        /lidar_status (std_msgs/String): Sensor status and diagnostics
        /lidar_point (geometry_msgs/PointStamped): 3D point in sensor frame
    """

    def __init__(self):
        super().__init__('lidar_reader_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('sensor_position', 'front')  # front, down, etc.
        self.declare_parameter('timeout', 0.1)  # Serial timeout
        self.declare_parameter('max_range', 50.0)  # Maximum valid range (meters)
        self.declare_parameter('min_range', 0.05)  # Minimum valid range (meters)
        self.declare_parameter('field_of_view', 0.035)  # FOV in radians (~2 degrees)
        self.declare_parameter('enable_filtering', True)  # Enable distance filtering
        self.declare_parameter('filter_window_size', 5)  # Moving average filter size
        self.declare_parameter('max_invalid_readings', 10)  # Max consecutive invalid readings

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.sensor_position = self.get_parameter('sensor_position').get_parameter_value().string_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.field_of_view = self.get_parameter('field_of_view').get_parameter_value().double_value
        self.enable_filtering = self.get_parameter('enable_filtering').get_parameter_value().bool_value
        self.filter_window_size = self.get_parameter('filter_window_size').get_parameter_value().integer_value
        self.max_invalid_readings = self.get_parameter('max_invalid_readings').get_parameter_value().integer_value

        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()

        # Data processing
        self.buffer = bytearray()
        self.distance_filter = deque(maxlen=self.filter_window_size)
        self.last_valid_distance = 0.0
        self.consecutive_invalid_readings = 0

        # Statistics and monitoring
        self.stats = {
            'total_frames': 0,
            'valid_frames': 0,
            'invalid_frames': 0,
            'checksum_errors': 0,
            'out_of_range_readings': 0,
            'last_distance': 0.0,
            'last_signal_strength': 0,
            'last_status': 0,
            'connection_errors': 0
        }

        # Status tracking
        self.sensor_healthy = False
        self.last_reading_time = time.time()

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.range_pub = self.create_publisher(Range, 'lidar_distance', sensor_qos)
        self.raw_pub = self.create_publisher(Float32MultiArray, 'lidar_raw', sensor_qos)
        self.status_pub = self.create_publisher(String, 'lidar_status', reliable_qos)
        self.point_pub = self.create_publisher(PointStamped, 'lidar_point', sensor_qos)
        self.healthy_pub = self.create_publisher(Bool, 'lidar_healthy', reliable_qos)

        # Initialize serial connection
        self.connect_serial()

        # Create timer for reading serial data
        if self.publish_rate > 0:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'LiDAR Reader Node initialized - Port: {self.serial_port}, '
            f'Position: {self.sensor_position}, Rate: {self.publish_rate} Hz'
        )

    def connect_serial(self) -> bool:
        """Initialize serial connection to LiDAR sensor"""
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()

                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )

                # Clear input/output buffers
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()

                self.sensor_healthy = True
                self.get_logger().info(f'Serial port {self.serial_port} opened at {self.baud_rate} baud')
                return True

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.sensor_healthy = False
            self.stats['connection_errors'] += 1
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error opening serial port: {e}')
            self.sensor_healthy = False
            return False

    def parse_frame(self, frame: bytes) -> Optional[Tuple[float, int, int, int, int]]:
        """
        Parse NLink_TOFSense_Frame0 protocol frame.

        Frame format (16 bytes):
        - Bytes 0-1: Header (0x57, 0x00)
        - Byte 2: Sensor ID
        - Bytes 3-6: System time (4 bytes, little endian, milliseconds)
        - Bytes 7-10: Distance (4 bytes, little endian, raw units)
        - Byte 11: Status
        - Bytes 12-13: Signal strength (2 bytes, little endian)
        - Byte 14: Range precision
        - Byte 15: Checksum (sum of bytes 0-14)

        Args:
            frame: 16-byte frame data

        Returns:
            Tuple of (distance_m, system_time_ms, status, signal_strength, range_precision)
            or None if parsing fails
        """
        if len(frame) != 16:
            return None

        # Check header
        if frame[0] != 0x57 or frame[1] != 0x00:
            return None

        # Calculate and verify checksum
        calculated_checksum = sum(frame[:15]) & 0xFF
        received_checksum = frame[15]

        if calculated_checksum != received_checksum:
            self.get_logger().debug(
                f'Checksum mismatch: calculated 0x{calculated_checksum:02x}, '
                f'received 0x{received_checksum:02x}'
            )
            self.stats['checksum_errors'] += 1
            return None

        try:
            # Parse fields
            sensor_id = frame[2]

            # System time (4 bytes, little endian)
            system_time = struct.unpack('<I', frame[3:7])[0]

            # Distance (4 bytes, little endian) - convert from raw to meters
            distance_raw = struct.unpack('<I', frame[7:11])[0]
            distance_m = distance_raw / 256.0 / 1000.0

            # Status byte
            status = frame[11]

            # Signal strength (2 bytes, little endian)
            signal_strength = struct.unpack('<H', frame[12:14])[0]

            # Range precision
            range_precision = frame[14]

            return (distance_m, system_time, status, signal_strength, range_precision)

        except struct.error as e:
            self.get_logger().debug(f'Error unpacking frame data: {e}')
            return None

    def find_frame_in_buffer(self) -> Optional[bytes]:
        """
        Look for a valid frame in the buffer using sliding window approach.

        Returns:
            16-byte frame if found and valid, None otherwise
        """
        # Need at least 16 bytes for a complete frame
        while len(self.buffer) >= 16:
            # Look for frame header (0x57 0x00)
            if self.buffer[0] == 0x57 and self.buffer[1] == 0x00:
                # Extract potential frame
                potential_frame = bytes(self.buffer[:16])

                # Verify checksum quickly
                checksum = sum(potential_frame[:15]) & 0xFF
                if checksum == potential_frame[15]:
                    # Valid frame found, remove from buffer
                    del self.buffer[:16]
                    return potential_frame
                else:
                    # Invalid checksum, skip first byte and continue searching
                    del self.buffer[0]
            else:
                # No header at current position, skip first byte
                del self.buffer[0]

        return None

    def apply_distance_filter(self, distance: float) -> float:
        """
        Apply moving average filter to distance readings.

        Args:
            distance: Raw distance measurement

        Returns:
            Filtered distance measurement
        """
        if not self.enable_filtering:
            return distance

        # Add to filter window
        self.distance_filter.append(distance)

        # Return moving average
        return float(np.mean(self.distance_filter))

    def validate_distance(self, distance: float, status: int) -> bool:
        """
        Validate distance measurement against range and status criteria.

        Args:
            distance: Distance measurement in meters
            status: Sensor status byte

        Returns:
            True if distance is valid, False otherwise
        """
        # Check range bounds
        if not (self.min_range <= distance <= self.max_range):
            self.stats['out_of_range_readings'] += 1
            return False

        # Check sensor status (implementation depends on sensor documentation)
        # For now, assume status 0 is good, others may indicate issues
        if status != 0:
            self.get_logger().debug(f'Sensor status warning: {status}')

        return True

    def timer_callback(self):
        """Read serial data and publish parsed distance measurements."""
        if not self.sensor_healthy:
            # Try to reconnect periodically
            if time.time() - self.last_reading_time > 5.0:  # Every 5 seconds
                self.connect_serial()
            return

        try:
            with self.serial_lock:
                if not self.serial_conn or not self.serial_conn.is_open:
                    self.sensor_healthy = False
                    return

                # Read available data
                if self.serial_conn.in_waiting > 0:
                    new_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.buffer.extend(new_data)

            # Try to find and parse frames
            frame = self.find_frame_in_buffer()
            if frame:
                self.stats['total_frames'] += 1

                parsed_data = self.parse_frame(frame)
                if parsed_data:
                    distance_m, system_time, status, signal_strength, range_precision = parsed_data

                    # Validate distance
                    if self.validate_distance(distance_m, status):
                        # Apply filtering
                        filtered_distance = self.apply_distance_filter(distance_m)

                        # Update statistics
                        self.stats['valid_frames'] += 1
                        self.stats['last_distance'] = filtered_distance
                        self.stats['last_signal_strength'] = signal_strength
                        self.stats['last_status'] = status
                        self.last_valid_distance = filtered_distance
                        self.last_reading_time = time.time()
                        self.consecutive_invalid_readings = 0

                        # Publish Range message
                        self.publish_range_message(filtered_distance)

                        # Publish raw data
                        self.publish_raw_data(distance_m, system_time, status, signal_strength, range_precision)

                        # Publish 3D point
                        self.publish_point_message(filtered_distance)

                    else:
                        self.stats['invalid_frames'] += 1
                        self.consecutive_invalid_readings += 1

                        # If too many consecutive invalid readings, mark sensor as unhealthy
                        if self.consecutive_invalid_readings > self.max_invalid_readings:
                            self.sensor_healthy = False
                            self.get_logger().warn(
                                f'Too many consecutive invalid readings ({self.consecutive_invalid_readings}), '
                                'marking sensor as unhealthy'
                            )
                else:
                    self.stats['invalid_frames'] += 1

        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
            self.sensor_healthy = False
            self.stats['connection_errors'] += 1
        except Exception as e:
            self.get_logger().error(f'Unexpected error in timer callback: {e}')

    def publish_range_message(self, distance: float):
        """Publish sensor_msgs/Range message"""
        try:
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED  # TOF sensors typically use infrared
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = distance

            self.range_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing range message: {e}')

    def publish_raw_data(self, distance: float, system_time: int, status: int,
                        signal_strength: int, range_precision: int):
        """Publish raw sensor data for debugging"""
        try:
            raw_msg = Float32MultiArray()
            raw_msg.data = [
                float(distance),
                float(system_time),
                float(status),
                float(signal_strength),
                float(range_precision),
                float(time.time())  # ROS timestamp
            ]

            self.raw_pub.publish(raw_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing raw data: {e}')

    def publish_point_message(self, distance: float):
        """Publish 3D point message in sensor frame"""
        try:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = self.frame_id

            # Convert distance to 3D point based on sensor position
            if self.sensor_position == 'front':
                # Forward-facing sensor: point along positive X axis
                point_msg.point.x = distance
                point_msg.point.y = 0.0
                point_msg.point.z = 0.0
            elif self.sensor_position == 'down':
                # Downward-facing sensor: point along negative Z axis
                point_msg.point.x = 0.0
                point_msg.point.y = 0.0
                point_msg.point.z = -distance
            else:
                # Default: forward-facing
                point_msg.point.x = distance
                point_msg.point.y = 0.0
                point_msg.point.z = 0.0

            self.point_pub.publish(point_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing point message: {e}')

    def publish_status(self):
        """Publish sensor status and diagnostics"""
        try:
            # Sensor health status
            health_msg = Bool()
            health_msg.data = self.sensor_healthy
            self.healthy_pub.publish(health_msg)

            # Detailed status message
            if self.stats['total_frames'] > 0:
                success_rate = (self.stats['valid_frames'] / self.stats['total_frames']) * 100
            else:
                success_rate = 0.0

            time_since_last = time.time() - self.last_reading_time

            status_parts = [
                f"Health: {'OK' if self.sensor_healthy else 'ERROR'}",
                f"Position: {self.sensor_position}",
                f"Distance: {self.stats['last_distance']:.3f}m",
                f"Success: {success_rate:.1f}%",
                f"Signal: {self.stats['last_signal_strength']}",
                f"LastRead: {time_since_last:.1f}s ago",
                f"Frames: {self.stats['total_frames']}",
                f"Errors: {self.stats['invalid_frames']}"
            ]

            status_msg = String()
            status_msg.data = " | ".join(status_parts)
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                    self.get_logger().info(f'Serial port {self.serial_port} closed')
        except Exception as e:
            self.get_logger().debug(f'Error closing serial port: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LidarReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()