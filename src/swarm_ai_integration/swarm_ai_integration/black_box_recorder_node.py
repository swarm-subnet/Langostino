#!/usr/bin/env python3
"""
Black Box Recorder Node - Flight Data Recorder for Swarm AI System

This node acts as a black box flight data recorder, logging all critical system data
including sensor inputs, AI model outputs, flight controller communications, and
safety system status. Data is persisted to files that survive system restarts.

Data Logged:
- Sensor data: LiDAR, IMU, GPS, battery
- AI system: observations, actions, model status
- Flight controller: commands sent, telemetry received, MSP communications
- Safety system: override status, emergency actions
- System health: node status, performance metrics
"""

import json
import os
import time
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Any, Optional
import gzip
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String, Float32MultiArray, Header
from sensor_msgs.msg import Imu, NavSatFix, BatteryState, Range
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped, PointStamped


class BlackBoxRecorderNode(Node):
    """
    Black Box Flight Data Recorder for comprehensive system logging.

    This node subscribes to all critical topics in the swarm AI system and logs
    the data to persistent JSON files with automatic rotation and compression.
    """

    def __init__(self):
        super().__init__('black_box_recorder_node')

        # Declare parameters
        self.declare_parameter('log_directory', '/var/log/swarm_blackbox')
        self.declare_parameter('max_file_size_mb', 100)
        self.declare_parameter('max_files_per_session', 10)
        self.declare_parameter('compress_old_files', True)
        self.declare_parameter('log_level', 'INFO')  # DEBUG, INFO, WARN, ERROR
        self.declare_parameter('buffer_size', 1000)  # Messages to buffer before write
        self.declare_parameter('flush_interval', 5.0)  # Seconds between forced flushes

        # Get parameters
        self.log_directory = Path(self.get_parameter('log_directory').get_parameter_value().string_value)
        self.max_file_size_mb = self.get_parameter('max_file_size_mb').get_parameter_value().integer_value
        self.max_files_per_session = self.get_parameter('max_files_per_session').get_parameter_value().integer_value
        self.compress_old_files = self.get_parameter('compress_old_files').get_parameter_value().bool_value
        self.log_level = self.get_parameter('log_level').get_parameter_value().string_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.flush_interval = self.get_parameter('flush_interval').get_parameter_value().double_value

        # Initialize logging system
        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.log_buffer = []
        self.buffer_lock = threading.Lock()
        self.file_handle = None
        self.current_file_size = 0
        self.file_counter = 0

        # Data caches for correlation
        self.last_data = {
            'ai_observation': None,
            'ai_action': None,
            'fc_attitude': None,
            'fc_imu': None,
            'lidar_distance': None,
            'safety_status': None
        }

        # Performance tracking - MUST be initialized before setup_logging()
        self.stats = {
            'messages_logged': 0,
            'bytes_written': 0,
            'files_created': 0,
            'dropped_messages': 0,
            'errors': 0
        }

        # Setup log directory and initial file
        self.setup_logging()

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

        # Subscribe to all critical topics
        self._setup_subscriptions(reliable_qos, sensor_qos)

        # Create timer for periodic flush
        self.flush_timer = self.create_timer(self.flush_interval, self.flush_buffer)

        # Create timer for statistics logging
        self.stats_timer = self.create_timer(60.0, self.log_statistics)  # Every minute

        # Log system startup
        self.log_system_event('SYSTEM_START', {
            'session_id': self.session_id,
            'node_name': self.get_name(),
            'log_directory': str(self.log_directory),
            'parameters': {
                'max_file_size_mb': self.max_file_size_mb,
                'max_files_per_session': self.max_files_per_session,
                'compress_old_files': self.compress_old_files,
                'log_level': self.log_level
            }
        })

        self.get_logger().info(f'Black Box Recorder initialized - Session: {self.session_id}')
        self.get_logger().info(f'Logging to: {self.log_directory}')

    def setup_logging(self):
        """Initialize the logging system with directory and file creation"""
        try:
            # Create log directory if it doesn't exist
            self.log_directory.mkdir(parents=True, exist_ok=True)

            # Create session subdirectory
            self.session_dir = self.log_directory / f"session_{self.session_id}"
            self.session_dir.mkdir(exist_ok=True)

            # Create initial log file
            self.create_new_log_file()

            self.get_logger().info(f'Log directory created: {self.session_dir}')

        except Exception as e:
            self.get_logger().error(f'Failed to setup logging directory: {e}')
            # Fallback to /tmp
            self.log_directory = Path('/tmp/swarm_blackbox')
            self.log_directory.mkdir(parents=True, exist_ok=True)
            self.session_dir = self.log_directory / f"session_{self.session_id}"
            self.session_dir.mkdir(exist_ok=True)
            self.create_new_log_file()

    def create_new_log_file(self):
        """Create a new log file with proper naming and headers"""
        try:
            # Close existing file if open
            if self.file_handle:
                self.file_handle.close()

            # Compress old file if enabled
            if self.compress_old_files and self.file_counter > 0:
                self.compress_previous_file()

            # Create new filename
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            filename = f"blackbox_{timestamp}_{self.file_counter:03d}.jsonl"
            filepath = self.session_dir / filename

            # Open new file
            self.file_handle = open(filepath, 'w', encoding='utf-8')
            self.current_file_size = 0
            self.file_counter += 1
            self.stats['files_created'] += 1

            # Write file header
            header = {
                'log_type': 'FILE_HEADER',
                'timestamp': time.time(),
                'utc_time': datetime.now(timezone.utc).isoformat(),
                'session_id': self.session_id,
                'file_number': self.file_counter,
                'node_name': self.get_name(),
                'ros_version': 'ROS2',
                'system_info': {
                    'log_directory': str(self.log_directory),
                    'max_file_size_mb': self.max_file_size_mb
                }
            }

            self._write_log_entry(header)

            self.get_logger().info(f'Created new log file: {filename}')

        except Exception as e:
            self.get_logger().error(f'Failed to create new log file: {e}')
            self.stats['errors'] += 1

    def compress_previous_file(self):
        """Compress the previous log file to save space"""
        try:
            if self.file_counter > 0:
                # Find the previous file
                pattern = f"blackbox_*_{self.file_counter-1:03d}.jsonl"
                for filepath in self.session_dir.glob(pattern):
                    compressed_path = filepath.with_suffix('.jsonl.gz')

                    with open(filepath, 'rb') as f_in:
                        with gzip.open(compressed_path, 'wb') as f_out:
                            f_out.writelines(f_in)

                    # Remove original file
                    filepath.unlink()
                    self.get_logger().debug(f'Compressed log file: {compressed_path.name}')
                    break

        except Exception as e:
            self.get_logger().debug(f'Error compressing previous file: {e}')

    def _setup_subscriptions(self, reliable_qos, sensor_qos):
        """Setup all topic subscriptions for data logging"""

        # AI System Topics
        self.ai_obs_sub = self.create_subscription(
            Float32MultiArray, '/ai/observation',
            lambda msg: self.log_message('AI_OBSERVATION', msg, 'ai_observation'), reliable_qos)

        self.ai_action_sub = self.create_subscription(
            Float32MultiArray, '/ai/action',
            lambda msg: self.log_message('AI_ACTION', msg, 'ai_action'), reliable_qos)

        self.ai_status_sub = self.create_subscription(
            Float32MultiArray, '/ai/status',
            lambda msg: self.log_message('AI_STATUS', msg), reliable_qos)

        self.ai_ready_sub = self.create_subscription(
            Bool, '/ai/model_ready',
            lambda msg: self.log_message('AI_MODEL_READY', msg), reliable_qos)

        # Flight Controller Topics
        self.fc_imu_sub = self.create_subscription(
            Imu, '/fc/imu_raw',
            lambda msg: self.log_message('FC_IMU', msg, 'fc_imu'), sensor_qos)

        self.fc_gps_sub = self.create_subscription(
            NavSatFix, '/fc/gps_fix',
            lambda msg: self.log_message('FC_GPS', msg), sensor_qos)

        self.fc_attitude_sub = self.create_subscription(
            Vector3Stamped, '/fc/attitude_euler',
            lambda msg: self.log_message('FC_ATTITUDE_EULER', msg, 'fc_attitude'), sensor_qos)

        self.fc_battery_sub = self.create_subscription(
            BatteryState, '/fc/battery',
            lambda msg: self.log_message('FC_BATTERY', msg), sensor_qos)

        self.fc_status_sub = self.create_subscription(
            String, '/fc/status',
            lambda msg: self.log_message('FC_STATUS', msg), reliable_qos)

        self.fc_msp_status_sub = self.create_subscription(
            Float32MultiArray, '/fc/msp_status',
            lambda msg: self.log_message('FC_MSP_STATUS', msg), sensor_qos)

        self.fc_connected_sub = self.create_subscription(
            Bool, '/fc/connected',
            lambda msg: self.log_message('FC_CONNECTED', msg), reliable_qos)

        self.fc_motor_rpm_sub = self.create_subscription(
            Float32MultiArray, '/fc/motor_rpm',
            lambda msg: self.log_message('FC_MOTOR_RPM', msg), sensor_qos)

        self.fc_gps_speed_course_sub = self.create_subscription(
            Float32MultiArray, '/fc/gps_speed_course',
            lambda msg: self.log_message('FC_GPS_SPEED_COURSE', msg), sensor_qos)

        self.fc_waypoint_sub = self.create_subscription(
            Float32MultiArray, '/fc/waypoint',
            lambda msg: self.log_message('FC_WAYPOINT', msg), reliable_qos)

        # FC Command Topics
        self.fc_rc_override_sub = self.create_subscription(
            Float32MultiArray, '/fc/rc_override',
            lambda msg: self.log_message('FC_RC_OVERRIDE', msg), reliable_qos)

        self.fc_msp_command_sub = self.create_subscription(
            Float32MultiArray, '/fc/msp_command',
            lambda msg: self.log_message('FC_MSP_COMMAND', msg), reliable_qos)

        # FC Adapter Topics
        self.fc_adapter_status_sub = self.create_subscription(
            String, '/fc_adapter/status',
            lambda msg: self.log_message('FC_ADAPTER_STATUS', msg), reliable_qos)

        self.fc_adapter_vel_error_sub = self.create_subscription(
            Vector3Stamped, '/fc_adapter/velocity_error',
            lambda msg: self.log_message('FC_ADAPTER_VELOCITY_ERROR', msg), sensor_qos)

        # LiDAR Topics
        self.lidar_distance_sub = self.create_subscription(
            Range, '/lidar_distance',
            lambda msg: self.log_message('LIDAR_DISTANCE', msg, 'lidar_distance'), sensor_qos)

        self.lidar_raw_sub = self.create_subscription(
            Float32MultiArray, '/lidar_raw',
            lambda msg: self.log_message('LIDAR_RAW', msg), sensor_qos)

        self.lidar_point_sub = self.create_subscription(
            PointStamped, '/lidar_point',
            lambda msg: self.log_message('LIDAR_POINT', msg), sensor_qos)

        # Safety System Topics
        self.safety_override_sub = self.create_subscription(
            Bool, '/safety/override',
            lambda msg: self.log_message('SAFETY_OVERRIDE', msg, 'safety_status'), reliable_qos)

        self.safety_rth_sub = self.create_subscription(
            Bool, '/safety/rth_command',
            lambda msg: self.log_message('SAFETY_RTH_COMMAND', msg, 'safety_status'), reliable_qos)

        self.safety_status_sub = self.create_subscription(
            String, '/safety/status',
            lambda msg: self.log_message('SAFETY_STATUS', msg, 'safety_status'), reliable_qos)

    def log_message(self, message_type: str, msg, cache_key: str = None):
        """Log a ROS message with timestamp and metadata"""
        try:
            # Create base log entry
            log_entry = {
                'log_type': message_type,
                'timestamp': time.time(),
                'utc_time': datetime.now(timezone.utc).isoformat(),
                'ros_timestamp': self._get_ros_timestamp(msg),
                'data': self._extract_message_data(msg)
            }

            # Cache data for correlation if requested
            if cache_key:
                self.last_data[cache_key] = log_entry['data']

            # Add correlation data for key messages
            if message_type == 'AI_ACTION' and self.last_data['ai_observation']:
                log_entry['correlated_observation'] = self.last_data['ai_observation']

            # Buffer the log entry
            with self.buffer_lock:
                if len(self.log_buffer) < self.buffer_size:
                    self.log_buffer.append(log_entry)
                    self.stats['messages_logged'] += 1
                else:
                    self.stats['dropped_messages'] += 1
                    if self.log_level == 'DEBUG':
                        self.get_logger().debug('Log buffer full, dropping message')

        except Exception as e:
            self.get_logger().error(f'Error logging message {message_type}: {e}')
            self.stats['errors'] += 1

    def log_system_event(self, event_type: str, data: Dict[str, Any]):
        """Log system-level events like startup, shutdown, errors"""
        log_entry = {
            'log_type': 'SYSTEM_EVENT',
            'event_type': event_type,
            'timestamp': time.time(),
            'utc_time': datetime.now(timezone.utc).isoformat(),
            'data': data
        }

        with self.buffer_lock:
            self.log_buffer.append(log_entry)

    def _extract_message_data(self, msg) -> Dict[str, Any]:
        """Extract data from ROS message into JSON-serializable format"""
        try:
            if hasattr(msg, 'data'):
                # Standard data messages
                if isinstance(msg.data, (list, tuple)):
                    return {'data': list(msg.data)}
                else:
                    return {'data': msg.data}

            elif isinstance(msg.data, (list, tuple)) and hasattr(msg, 'data'):
                # Float32MultiArray (already handled above, but ensure it's first)
                return {'data': list(msg.data)}

            elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                # Twist messages
                return {
                    'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
                    'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z}
                }

            elif hasattr(msg, 'orientation'):
                # IMU messages
                return {
                    'orientation': {
                        'x': msg.orientation.x, 'y': msg.orientation.y,
                        'z': msg.orientation.z, 'w': msg.orientation.w
                    },
                    'angular_velocity': {
                        'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y,
                        'z': msg.angular_velocity.z
                    },
                    'linear_acceleration': {
                        'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y,
                        'z': msg.linear_acceleration.z
                    }
                }

            elif hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                # GPS messages
                return {
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'status': {'status': msg.status.status, 'service': msg.status.service}
                }

            elif hasattr(msg, 'voltage'):
                # Battery messages
                return {
                    'voltage': msg.voltage,
                    'current': msg.current if hasattr(msg, 'current') else None,
                    'percentage': msg.percentage if hasattr(msg, 'percentage') else None
                }

            elif hasattr(msg, 'range'):
                # Range sensor messages
                return {
                    'range': msg.range,
                    'min_range': msg.min_range,
                    'max_range': msg.max_range,
                    'field_of_view': msg.field_of_view
                }

            elif hasattr(msg, 'vector'):
                # Vector3Stamped messages
                return {
                    'vector': {'x': msg.vector.x, 'y': msg.vector.y, 'z': msg.vector.z}
                }

            elif hasattr(msg, 'point'):
                # PointStamped messages
                return {
                    'point': {'x': msg.point.x, 'y': msg.point.y, 'z': msg.point.z}
                }

            else:
                # Fallback: try to convert to dict
                return {'raw': str(msg)}

        except Exception as e:
            self.get_logger().debug(f'Error extracting message data: {e}')
            return {'error': f'Failed to extract data: {str(e)}'}

    def _get_ros_timestamp(self, msg) -> Optional[float]:
        """Extract ROS timestamp from message header if available"""
        try:
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            return None
        except:
            return None

    def flush_buffer(self):
        """Flush buffered log entries to file"""
        try:
            with self.buffer_lock:
                if not self.log_buffer:
                    return

                entries_to_write = self.log_buffer.copy()
                self.log_buffer.clear()

            # Write entries to file
            for entry in entries_to_write:
                self._write_log_entry(entry)

            # Flush file handle
            if self.file_handle:
                self.file_handle.flush()
                os.fsync(self.file_handle.fileno())  # Force write to disk

            # Check if file rotation is needed
            if self.current_file_size > self.max_file_size_mb * 1024 * 1024:
                if self.file_counter < self.max_files_per_session:
                    self.create_new_log_file()
                else:
                    self.get_logger().warn('Maximum files per session reached, continuing with current file')

        except Exception as e:
            self.get_logger().error(f'Error flushing buffer: {e}')
            self.stats['errors'] += 1

    def _write_log_entry(self, entry: Dict[str, Any]):
        """Write a single log entry to the current file"""
        try:
            if not self.file_handle:
                return

            # Convert to JSON and write
            json_line = json.dumps(entry, default=str, separators=(',', ':')) + '\n'
            self.file_handle.write(json_line)

            # Update file size
            self.current_file_size += len(json_line.encode('utf-8'))
            self.stats['bytes_written'] += len(json_line.encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f'Error writing log entry: {e}')
            self.stats['errors'] += 1

    def log_statistics(self):
        """Log periodic statistics about the recorder performance"""
        stats_data = {
            'messages_logged': self.stats['messages_logged'],
            'bytes_written': self.stats['bytes_written'],
            'files_created': self.stats['files_created'],
            'dropped_messages': self.stats['dropped_messages'],
            'errors': self.stats['errors'],
            'current_file_size': self.current_file_size,
            'buffer_size': len(self.log_buffer),
            'uptime': time.time() - self.get_clock().now().seconds_nanoseconds()[0]
        }

        self.log_system_event('STATISTICS', stats_data)

        if self.log_level in ['DEBUG', 'INFO']:
            self.get_logger().info(
                f'Black Box Stats - Messages: {self.stats["messages_logged"]}, '
                f'Files: {self.stats["files_created"]}, '
                f'Errors: {self.stats["errors"]}, '
                f'Dropped: {self.stats["dropped_messages"]}'
            )

    def destroy_node(self):
        """Clean shutdown with final log flush"""
        try:
            self.get_logger().info('Black Box Recorder shutting down...')

            # Log shutdown event
            self.log_system_event('SYSTEM_SHUTDOWN', {
                'final_stats': self.stats,
                'session_duration': time.time() - self.get_clock().now().seconds_nanoseconds()[0]
            })

            # Final buffer flush
            self.flush_buffer()

            # Close file handle
            if self.file_handle:
                self.file_handle.close()
                self.get_logger().info('Log file closed successfully')

            # Compress final file if enabled
            if self.compress_old_files:
                self.compress_previous_file()

        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = BlackBoxRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Black Box Recorder error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()