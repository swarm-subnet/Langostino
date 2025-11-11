#!/usr/bin/env python3
"""
Black Box Recorder Node - Simplified Flight Data Recorder

This node logs all critical system data every 0.1 seconds to JSON files.
Each log entry contains a snapshot of the latest values from all topics.

Structure:
- observation_1:
    timestamp: ...
    ai_observation: [...]
    ai_action: [...]
    fc_gps: {...}
    ...
- observation_2:
    ...

Features:
- Time-based logging (10 Hz)
- Simple JSON format (no compression)
- Latest-value snapshot approach
- File rotation at max size
"""

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import Imu, NavSatFix, BatteryState, Range
from geometry_msgs.msg import Vector3Stamped, PointStamped


class BlackBoxRecorderNode(Node):
    """
    Simplified Black Box Flight Data Recorder.
    Logs system state snapshots every 0.1 seconds.
    """

    def __init__(self):
        super().__init__('black_box_recorder_node')

        # Declare parameters
        self.declare_parameter('log_directory', '~/swarm-ros/flight-logs')
        self.declare_parameter('max_log_file_size_mb', 100)
        self.declare_parameter('log_rate_hz', 10.0)  # Logging frequency
        self.declare_parameter('sensor_qos_depth', 1)
        self.declare_parameter('reliable_qos_depth', 10)

        # Get parameters
        log_dir_str = self.get_parameter('log_directory').get_parameter_value().string_value
        self.log_directory = Path(os.path.expanduser(log_dir_str))
        self.max_file_size_mb = self.get_parameter('max_log_file_size_mb').get_parameter_value().integer_value
        self.log_rate = self.get_parameter('log_rate_hz').get_parameter_value().double_value
        self.sensor_qos_depth = self.get_parameter('sensor_qos_depth').get_parameter_value().integer_value
        self.reliable_qos_depth = self.get_parameter('reliable_qos_depth').get_parameter_value().integer_value

        # Initialize logging system
        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.file_handle = None
        self.current_file_size = 0
        self.file_counter = 0
        self.observation_counter = 0

        # Storage for latest values from each topic
        self.latest_data = {}

        # Performance tracking
        self.stats = {
            'observations_logged': 0,
            'bytes_written': 0,
            'files_created': 0,
            'errors': 0
        }

        # Setup log directory and initial file
        self.setup_logging()

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.reliable_qos_depth
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.sensor_qos_depth
        )

        # Subscribe to all topics and store latest values
        self._setup_subscriptions(reliable_qos, sensor_qos)

        # Create timer for periodic snapshot logging
        log_period = 1.0 / self.log_rate
        self.log_timer = self.create_timer(log_period, self.log_snapshot)

        # Create timer for statistics
        self.stats_timer = self.create_timer(60.0, self.log_statistics)

        self.get_logger().info(f'Black Box Recorder initialized - Session: {self.session_id}')
        self.get_logger().info(f'Logging to: {self.log_directory}')
        self.get_logger().info(f'Log rate: {self.log_rate} Hz')

    def setup_logging(self):
        """Initialize the logging system"""
        try:
            # Create log directory
            self.log_directory.mkdir(parents=True, exist_ok=True)

            # Test write permissions
            test_file = self.log_directory / '.write_test'
            test_file.write_text('test')
            test_file.unlink()

            # Create first log file
            self.create_new_log_file()

        except PermissionError:
            self.get_logger().warn(f'No write permission for {self.log_directory}, falling back to /tmp')
            self.log_directory = Path('/tmp/swarm_blackbox')
            self.log_directory.mkdir(parents=True, exist_ok=True)
            self.create_new_log_file()

        except Exception as e:
            self.get_logger().error(f'Failed to setup logging: {e}')
            raise

    def create_new_log_file(self):
        """Create a new log file"""
        try:
            # Close previous file if open
            if self.file_handle:
                self.file_handle.close()

            # Create new filename
            filename = f"blackbox_{self.session_id}_{self.file_counter:03d}.jsonl"
            filepath = self.log_directory / filename

            # Open new file
            self.file_handle = open(filepath, 'w')
            self.current_file_size = 0
            self.stats['files_created'] += 1

            # Write header
            header = {
                'type': 'HEADER',
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'session_id': self.session_id,
                'file_number': self.file_counter,
                'log_rate_hz': self.log_rate,
                'system_info': {
                    'log_directory': str(self.log_directory),
                    'max_file_size_mb': self.max_file_size_mb
                }
            }

            self.file_handle.write(json.dumps(header) + '\n')
            self.file_handle.flush()

            self.get_logger().info(f'Created new log file: {filename}')

        except Exception as e:
            self.get_logger().error(f'Failed to create new log file: {e}')
            self.stats['errors'] += 1

    def _setup_subscriptions(self, reliable_qos, sensor_qos):
        """Setup all topic subscriptions - store latest values"""

        # AI System Topics
        self.create_subscription(
            Float32MultiArray, '/ai/observation',
            lambda msg: self._update_latest('ai_observation', self._msg_to_dict(msg)), reliable_qos)

        self.create_subscription(
            Float32MultiArray, '/ai/action',
            lambda msg: self._update_latest('ai_action', self._msg_to_dict(msg)), reliable_qos)

        self.create_subscription(
            Float32MultiArray, '/ai/status',
            lambda msg: self._update_latest('ai_status', self._msg_to_dict(msg)), reliable_qos)

        self.create_subscription(
            Bool, '/ai/model_ready',
            lambda msg: self._update_latest('ai_model_ready', msg.data), reliable_qos)

        # Flight Controller Topics
        self.create_subscription(
            Imu, '/fc/imu_raw',
            lambda msg: self._update_latest('fc_imu', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            NavSatFix, '/fc/gps_fix',
            lambda msg: self._update_latest('fc_gps', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            Vector3Stamped, '/fc/attitude_euler',
            lambda msg: self._update_latest('fc_attitude', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            Float32MultiArray, '/fc/altitude',
            lambda msg: self._update_latest('fc_altitude', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            BatteryState, '/fc/battery',
            lambda msg: self._update_latest('fc_battery', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            String, '/fc/status',
            lambda msg: self._update_latest('fc_status', msg.data), reliable_qos)

        self.create_subscription(
            Float32MultiArray, '/fc/msp_status',
            lambda msg: self._update_latest('fc_msp_status', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            Bool, '/fc/connected',
            lambda msg: self._update_latest('fc_connected', msg.data), reliable_qos)

        self.create_subscription(
            Float32MultiArray, '/fc/motor_rpm',
            lambda msg: self._update_latest('fc_motor_rpm', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            Float32MultiArray, '/fc/gps_speed_course',
            lambda msg: self._update_latest('fc_gps_speed_course', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            Float32MultiArray, '/fc/waypoint',
            lambda msg: self._update_latest('fc_waypoint', self._msg_to_dict(msg)), reliable_qos)

        # FC Adapter Topics
        self.create_subscription(
            String, '/fc_adapter/status',
            lambda msg: self._update_latest('fc_adapter_status', msg.data), reliable_qos)

        self.create_subscription(
            Vector3Stamped, '/fc_adapter/velocity_error',
            lambda msg: self._update_latest('fc_adapter_velocity_error', self._msg_to_dict(msg)), sensor_qos)

        # LiDAR Topics
        self.create_subscription(
            Range, 'lidar_distance',
            lambda msg: self._update_latest('lidar_distance', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            Float32MultiArray, 'lidar_raw',
            lambda msg: self._update_latest('lidar_raw', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            PointStamped, 'lidar_point',
            lambda msg: self._update_latest('lidar_point', self._msg_to_dict(msg)), sensor_qos)

        self.create_subscription(
            String, 'lidar_status',
            lambda msg: self._update_latest('lidar_status', msg.data), reliable_qos)

        self.create_subscription(
            Bool, 'lidar_health',
            lambda msg: self._update_latest('lidar_health', msg.data), sensor_qos)

        # Safety System Topics
        self.create_subscription(
            Bool, '/safety/override',
            lambda msg: self._update_latest('safety_override', msg.data), reliable_qos)

        self.create_subscription(
            Bool, '/safety/rth_command',
            lambda msg: self._update_latest('safety_rth_command', msg.data), reliable_qos)

        self.create_subscription(
            String, '/safety/status',
            lambda msg: self._update_latest('safety_status', msg.data), reliable_qos)

    def _update_latest(self, topic_name: str, value: Any):
        """Update the latest value for a topic"""
        self.latest_data[topic_name] = value

    def _msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS message to dictionary"""
        result = {}

        # Handle Float32MultiArray
        if hasattr(msg, 'data') and isinstance(msg.data, (list, tuple)):
            return {'data': list(msg.data)}

        # Handle ROS2 messages using get_fields_and_field_types()
        if hasattr(msg, 'get_fields_and_field_types'):
            try:
                fields = msg.get_fields_and_field_types()
                for field_name in fields.keys():
                    if field_name.startswith('_'):
                        continue

                    value = getattr(msg, field_name, None)

                    # Skip None values
                    if value is None:
                        continue

                    # Recursively handle nested messages
                    if hasattr(value, 'get_fields_and_field_types'):
                        result[field_name] = self._msg_to_dict(value)
                    # Handle arrays/lists
                    elif isinstance(value, (list, tuple)):
                        # Check if it's a list of messages
                        if len(value) > 0 and hasattr(value[0], 'get_fields_and_field_types'):
                            result[field_name] = [self._msg_to_dict(item) for item in value]
                        else:
                            result[field_name] = list(value)
                    # Handle primitive types
                    else:
                        result[field_name] = value

            except Exception:
                # Fallback to __slots__ if get_fields_and_field_types fails
                pass

        # Fallback: Handle messages using __slots__ (older method)
        if not result and hasattr(msg, '__slots__'):
            for slot in msg.__slots__:
                if slot.startswith('_'):
                    continue
                value = getattr(msg, slot, None)
                if value is None:
                    continue
                if hasattr(value, '__slots__') or hasattr(value, 'get_fields_and_field_types'):
                    result[slot] = self._msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[slot] = list(value)
                else:
                    result[slot] = value

        return result

    def log_snapshot(self):
        """Log a snapshot of all latest values (called every 0.1s)"""
        try:
            self.observation_counter += 1

            # Create snapshot entry
            entry = {
                'observation_number': self.observation_counter,
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'data': dict(self.latest_data)  # Copy of all latest values
            }

            # Write to file
            line = json.dumps(entry) + '\n'
            self.file_handle.write(line)
            self.file_handle.flush()

            # Update stats
            line_bytes = len(line.encode('utf-8'))
            self.current_file_size += line_bytes
            self.stats['observations_logged'] += 1
            self.stats['bytes_written'] += line_bytes

            # Check if we need to rotate file
            if self.current_file_size >= self.max_file_size_mb * 1024 * 1024:
                self.get_logger().info(f'File size limit reached, rotating...')
                self.file_counter += 1
                self.create_new_log_file()

        except Exception as e:
            self.get_logger().error(f'Error logging snapshot: {e}')
            self.stats['errors'] += 1

    def log_statistics(self):
        """Log periodic statistics"""
        try:
            stats_entry = {
                'type': 'STATISTICS',
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'stats': dict(self.stats),
                'current_file_size_mb': self.current_file_size / (1024 * 1024),
                'topics_tracked': len(self.latest_data)
            }

            self.file_handle.write(json.dumps(stats_entry) + '\n')
            self.file_handle.flush()

            self.get_logger().info(
                f'Stats: {self.stats["observations_logged"]} observations, '
                f'{self.stats["bytes_written"] / 1024 / 1024:.1f} MB written, '
                f'{len(self.latest_data)} topics tracked'
            )

        except Exception as e:
            self.get_logger().error(f'Error logging statistics: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        try:
            if self.file_handle:
                # Write shutdown event
                shutdown_entry = {
                    'type': 'SYSTEM_SHUTDOWN',
                    'timestamp': datetime.now(timezone.utc).isoformat(),
                    'final_stats': dict(self.stats)
                }
                self.file_handle.write(json.dumps(shutdown_entry) + '\n')
                self.file_handle.flush()
                self.file_handle.close()

            self.get_logger().info('Black Box Recorder shutdown complete')

        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlackBoxRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
