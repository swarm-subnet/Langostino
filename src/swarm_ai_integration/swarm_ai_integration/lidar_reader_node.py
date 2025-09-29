#!/usr/bin/env python3
"""
LiDAR Reader Node - I2C Protocol Implementation

This node reads distance data from LiDAR sensors via I2C communication.
It supports multiple sensor instances for different mounting positions (front, down, etc.).

I2C Protocol:
- I2C Address: 0x08 (default)
- Distance Register: 0x24 (base address for distance data)
- Data Format: 4 bytes, little-endian uint32 representing distance in millimeters
- Conversion: distance_mm / 1000.0 = distance in meters

The node publishes sensor_msgs/Range messages containing distance measurements
and provides diagnostic information about sensor health and data quality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Range
from std_msgs.msg import Header, Float32MultiArray, String, Bool
from geometry_msgs.msg import PointStamped

from smbus2 import SMBus
import struct
import time
import threading
from typing import Optional, Tuple, Dict, Any
from collections import deque
import numpy as np


class LidarReaderNode(Node):
    """
    LiDAR Reader Node for I2C-based LiDAR sensors.

    This node handles I2C communication with LiDAR sensors,
    reads distance data, and publishes ROS2 Range messages.

    Publishers:
        /lidar_distance (sensor_msgs/Range): Distance measurements
        /lidar_raw (std_msgs/Float32MultiArray): Raw sensor data
        /lidar_status (std_msgs/String): Sensor status and diagnostics
        /lidar_point (geometry_msgs/PointStamped): 3D point in sensor frame
    """

    def __init__(self):
        super().__init__('lidar_reader_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)  # I2C bus number
        self.declare_parameter('i2c_address', 0x08)  # I2C device address
        self.declare_parameter('distance_register', 0x24)  # Distance data register
        self.declare_parameter('publish_rate', 100.0)  # Hz (max 100Hz per specs)
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('sensor_position', 'front')  # front, down, etc.
        self.declare_parameter('max_range', 50.0)  # Maximum valid range (meters)
        self.declare_parameter('min_range', 0.05)  # Minimum valid range (meters)
        self.declare_parameter('field_of_view', 0.035)  # FOV in radians (~2 degrees)
        self.declare_parameter('enable_filtering', True)  # Enable distance filtering
        self.declare_parameter('filter_window_size', 5)  # Moving average filter size
        self.declare_parameter('max_invalid_readings', 10)  # Max consecutive invalid readings

        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.distance_register = self.get_parameter('distance_register').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.sensor_position = self.get_parameter('sensor_position').get_parameter_value().string_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.field_of_view = self.get_parameter('field_of_view').get_parameter_value().double_value
        self.enable_filtering = self.get_parameter('enable_filtering').get_parameter_value().bool_value
        self.filter_window_size = self.get_parameter('filter_window_size').get_parameter_value().integer_value
        self.max_invalid_readings = self.get_parameter('max_invalid_readings').get_parameter_value().integer_value

        # I2C connection
        self.i2c_bus_conn: Optional[SMBus] = None
        self.i2c_lock = threading.Lock()

        # Data processing
        self.distance_filter = deque(maxlen=self.filter_window_size)
        self.last_valid_distance = 0.0
        self.consecutive_invalid_readings = 0

        # Statistics and monitoring
        self.stats = {
            'total_readings': 0,
            'valid_readings': 0,
            'invalid_readings': 0,
            'i2c_errors': 0,
            'out_of_range_readings': 0,
            'last_distance': 0.0,
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

        # Initialize I2C connection
        self.connect_i2c()

        # Create timer for reading I2C data
        if self.publish_rate > 0:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'LiDAR Reader Node initialized - I2C Bus: {self.i2c_bus}, Address: 0x{self.i2c_address:02x}, '
            f'Position: {self.sensor_position}, Rate: {self.publish_rate} Hz'
        )

    def connect_i2c(self) -> bool:
        """Initialize I2C connection to LiDAR sensor"""
        try:
            with self.i2c_lock:
                if self.i2c_bus_conn:
                    self.i2c_bus_conn.close()

                self.i2c_bus_conn = SMBus(self.i2c_bus)

                # Test connection by trying to read a byte
                _ = self.i2c_bus_conn.read_byte(self.i2c_address)

                self.sensor_healthy = True
                self.get_logger().info(f'I2C bus {self.i2c_bus} connected to address 0x{self.i2c_address:02x}')
                return True

        except Exception as e:
            self.get_logger().error(f'Failed to connect to I2C device 0x{self.i2c_address:02x} on bus {self.i2c_bus}: {e}')
            self.sensor_healthy = False
            self.stats['connection_errors'] += 1
            if self.i2c_bus_conn:
                try:
                    self.i2c_bus_conn.close()
                except:
                    pass
                self.i2c_bus_conn = None
            return False

    def read_distance_i2c(self) -> Optional[float]:
        """
        Read distance data from I2C sensor.

        Reads 4 bytes from the distance register (0x24) which contains
        distance in millimeters as a little-endian uint32.

        Returns:
            Distance in meters, or None if reading fails
        """
        try:
            with self.i2c_lock:
                if not self.i2c_bus_conn:
                    return None

                # Read 4 bytes from distance register
                data = self.i2c_bus_conn.read_i2c_block_data(self.i2c_address, self.distance_register, 4)

                # Convert to uint32 little-endian and then to meters
                distance_mm = struct.unpack('<I', bytes(data))[0]
                distance_m = distance_mm / 1000.0

                return distance_m

        except Exception as e:
            self.get_logger().debug(f'I2C read error: {e}')
            self.stats['i2c_errors'] += 1
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

    def validate_distance(self, distance: float) -> bool:
        """
        Validate distance measurement against range criteria.

        Args:
            distance: Distance measurement in meters

        Returns:
            True if distance is valid, False otherwise
        """
        # Check range bounds
        if not (self.min_range <= distance <= self.max_range):
            self.stats['out_of_range_readings'] += 1
            return False

        return True

    def timer_callback(self):
        """Read I2C data and publish distance measurements."""
        if not self.sensor_healthy:
            # Try to reconnect periodically
            if time.time() - self.last_reading_time > 5.0:  # Every 5 seconds
                self.connect_i2c()
            return

        try:
            # Read distance from I2C sensor
            distance_m = self.read_distance_i2c()

            if distance_m is not None:
                self.stats['total_readings'] += 1

                # Validate distance
                if self.validate_distance(distance_m):
                    # Apply filtering
                    filtered_distance = self.apply_distance_filter(distance_m)

                    # Update statistics
                    self.stats['valid_readings'] += 1
                    self.stats['last_distance'] = filtered_distance
                    self.last_valid_distance = filtered_distance
                    self.last_reading_time = time.time()
                    self.consecutive_invalid_readings = 0

                    # Publish Range message
                    self.publish_range_message(filtered_distance)

                    # Publish raw data
                    self.publish_raw_data(distance_m)

                    # Publish 3D point
                    self.publish_point_message(filtered_distance)

                else:
                    self.stats['invalid_readings'] += 1
                    self.consecutive_invalid_readings += 1

                    # If too many consecutive invalid readings, mark sensor as unhealthy
                    if self.consecutive_invalid_readings > self.max_invalid_readings:
                        self.sensor_healthy = False
                        self.get_logger().warn(
                            f'Too many consecutive invalid readings ({self.consecutive_invalid_readings}), '
                            'marking sensor as unhealthy'
                        )
            else:
                self.stats['invalid_readings'] += 1
                self.consecutive_invalid_readings += 1

        except Exception as e:
            self.get_logger().error(f'Unexpected error in timer callback: {e}')
            self.sensor_healthy = False
            self.stats['connection_errors'] += 1

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

    def publish_raw_data(self, distance: float):
        """Publish raw sensor data for debugging"""
        try:
            raw_msg = Float32MultiArray()
            raw_msg.data = [
                float(distance),  # Distance in meters
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
            if self.stats['total_readings'] > 0:
                success_rate = (self.stats['valid_readings'] / self.stats['total_readings']) * 100
            else:
                success_rate = 0.0

            time_since_last = time.time() - self.last_reading_time

            status_parts = [
                f"Health: {'OK' if self.sensor_healthy else 'ERROR'}",
                f"Position: {self.sensor_position}",
                f"I2C: 0x{self.i2c_address:02x}",
                f"Distance: {self.stats['last_distance']:.3f}m",
                f"Success: {success_rate:.1f}%",
                f"LastRead: {time_since_last:.1f}s ago",
                f"Readings: {self.stats['total_readings']}",
                f"I2C_Errors: {self.stats['i2c_errors']}"
            ]

            status_msg = String()
            status_msg.data = " | ".join(status_parts)
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        try:
            with self.i2c_lock:
                if self.i2c_bus_conn:
                    self.i2c_bus_conn.close()
                    self.get_logger().info(f'I2C bus {self.i2c_bus} closed')
        except Exception as e:
            self.get_logger().debug(f'Error closing I2C bus: {e}')

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