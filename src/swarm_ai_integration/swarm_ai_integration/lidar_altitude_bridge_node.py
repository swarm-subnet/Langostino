#!/usr/bin/env python3
"""
LiDAR to Altitude Bridge Node

This node bridges LiDAR distance readings to the /fc/altitude topic,
replacing the barometer altitude data when the barometer sensor is not available.

The node:
- Subscribes to /lidar_distance (from downward-facing LiDAR)
- Converts distance readings to altitude format
- Calculates vertical velocity (vario) from successive readings
- Publishes to /fc/altitude in the same format as barometer: [altitude_m, vario_m/s]

This allows the system to continue operating when the barometer is unavailable
due to hardware failure or other issues.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Bool, String
from collections import deque
import time
import numpy as np


class LidarAltitudeBridgeNode(Node):
    """
    Bridge node that converts LiDAR distance to barometer-compatible altitude data.

    Subscribes:
        /lidar_distance (sensor_msgs/Range): LiDAR distance measurements

    Publishes:
        /fc/altitude (std_msgs/Float32MultiArray): [altitude_m, vario_m/s]
        /lidar_altitude_status (std_msgs/String): Bridge status information
        /lidar_altitude_healthy (std_msgs/Bool): Health status
    """

    def __init__(self):
        super().__init__('lidar_altitude_bridge_node')

        # Declare parameters
        self.declare_parameter('lidar_topic', '/lidar_distance')
        self.declare_parameter('altitude_topic', '/fc/altitude')
        self.declare_parameter('vario_window_size', 5)  # Samples for vario calculation
        self.declare_parameter('vario_time_threshold', 0.5)  # Seconds
        self.declare_parameter('max_vario', 10.0)  # m/s (sanity check)
        self.declare_parameter('publish_status', True)
        self.declare_parameter('status_interval', 1.0)  # seconds

        # Get parameters
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.altitude_topic = self.get_parameter('altitude_topic').get_parameter_value().string_value
        self.vario_window_size = self.get_parameter('vario_window_size').get_parameter_value().integer_value
        self.vario_time_threshold = self.get_parameter('vario_time_threshold').get_parameter_value().double_value
        self.max_vario = self.get_parameter('max_vario').get_parameter_value().double_value
        self.publish_status_flag = self.get_parameter('publish_status').get_parameter_value().bool_value
        self.status_interval = self.get_parameter('status_interval').get_parameter_value().double_value

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

        # Data storage for vario calculation
        # Each entry: (timestamp, altitude)
        self.altitude_history = deque(maxlen=self.vario_window_size)

        # Current state
        self.current_altitude = 0.0
        self.current_vario = 0.0
        self.last_lidar_time = 0.0
        self.healthy = False

        # Statistics
        self.stats = {
            'messages_received': 0,
            'messages_published': 0,
            'last_altitude': 0.0,
            'last_vario': 0.0,
            'avg_update_rate': 0.0
        }

        # Publishers
        self.altitude_pub = self.create_publisher(
            Float32MultiArray,
            self.altitude_topic,
            sensor_qos
        )

        self.status_pub = self.create_publisher(
            String,
            '/lidar_altitude_status',
            reliable_qos
        )

        self.healthy_pub = self.create_publisher(
            Bool,
            '/lidar_altitude_healthy',
            reliable_qos
        )

        # Subscriber
        self.lidar_sub = self.create_subscription(
            Range,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos
        )

        # Status publishing timer
        if self.publish_status_flag:
            self.status_timer = self.create_timer(
                self.status_interval,
                self.publish_status
            )

        self.get_logger().info('═' * 60)
        self.get_logger().info('🔗 LiDAR-to-Altitude Bridge Node Started')
        self.get_logger().info(f'   LiDAR Input: {self.lidar_topic}')
        self.get_logger().info(f'   Altitude Output: {self.altitude_topic}')
        self.get_logger().info(f'   Vario Window: {self.vario_window_size} samples')
        self.get_logger().info('   ⚠️  Replacing barometer with LiDAR altitude')
        self.get_logger().info('═' * 60)

    def lidar_callback(self, msg: Range):
        """
        Process incoming LiDAR distance measurements and publish as altitude.

        Args:
            msg: Range message from LiDAR sensor
        """
        try:
            # Get current time
            current_time = time.time()

            # Extract distance (this is altitude for downward-facing LiDAR)
            altitude = msg.range

            # Validate range
            if not (msg.min_range <= altitude <= msg.max_range):
                self.get_logger().warn(
                    f'LiDAR reading out of valid range: {altitude:.3f}m '
                    f'(valid: {msg.min_range:.2f}-{msg.max_range:.2f}m)'
                )
                return

            # Update statistics
            self.stats['messages_received'] += 1
            self.current_altitude = altitude
            self.last_lidar_time = current_time
            self.healthy = True

            # Add to history for vario calculation
            self.altitude_history.append((current_time, altitude))

            # Calculate vario (vertical velocity)
            vario = self._calculate_vario()
            self.current_vario = vario

            # Publish altitude message in barometer format: [altitude_m, vario_m/s]
            altitude_msg = Float32MultiArray()
            altitude_msg.data = [float(altitude), float(vario)]

            self.altitude_pub.publish(altitude_msg)

            # Update statistics
            self.stats['messages_published'] += 1
            self.stats['last_altitude'] = altitude
            self.stats['last_vario'] = vario

            # Log at INFO level (same as barometer)
            self.get_logger().info(
                f'   ➜ Published to {self.altitude_topic} | '
                f'altitude={altitude:.2f}m | '
                f'vario={vario:+.2f}m/s '
                f'[LiDAR source]'
            )

        except Exception as e:
            self.get_logger().error(f'Error in lidar_callback: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self.healthy = False

    def _calculate_vario(self) -> float:
        """
        Calculate vertical velocity (vario) from altitude history.

        Uses linear regression on recent altitude samples to estimate
        the rate of change (vertical velocity).

        Returns:
            Vertical velocity in m/s (positive = ascending, negative = descending)
        """
        # Need at least 2 samples
        if len(self.altitude_history) < 2:
            return 0.0

        # Get time span
        times = np.array([sample[0] for sample in self.altitude_history])
        altitudes = np.array([sample[1] for sample in self.altitude_history])

        time_span = times[-1] - times[0]

        # Need sufficient time span for reliable calculation
        if time_span < self.vario_time_threshold:
            return 0.0

        # Simple linear regression to find rate of change
        # vario = d(altitude)/dt
        try:
            # Fit line: altitude = vario * time + offset
            coefficients = np.polyfit(times, altitudes, 1)
            vario = coefficients[0]  # Slope = rate of change

            # Sanity check: limit vario to reasonable values
            if abs(vario) > self.max_vario:
                self.get_logger().warn(
                    f'Calculated vario {vario:.2f}m/s exceeds max {self.max_vario}m/s, clamping'
                )
                vario = np.clip(vario, -self.max_vario, self.max_vario)

            return float(vario)

        except Exception as e:
            self.get_logger().debug(f'Error calculating vario: {e}')
            return 0.0

    def publish_status(self):
        """Publish bridge status information"""
        try:
            # Calculate uptime and data rate
            if self.stats['messages_received'] > 0:
                uptime = time.time() - (self.altitude_history[0][0] if self.altitude_history else time.time())
                if uptime > 0:
                    self.stats['avg_update_rate'] = self.stats['messages_received'] / uptime

            # Health status
            health_msg = Bool()
            health_msg.data = self.healthy
            self.healthy_pub.publish(health_msg)

            # Detailed status
            status_parts = [
                f"Health: {'OK' if self.healthy else 'ERROR'}",
                f"Altitude: {self.stats['last_altitude']:.3f}m",
                f"Vario: {self.stats['last_vario']:+.3f}m/s",
                f"Msgs: {self.stats['messages_published']}",
                f"Rate: {self.stats['avg_update_rate']:.1f}Hz",
                f"Source: LiDAR→Baro"
            ]

            status_msg = String()
            status_msg.data = " | ".join(status_parts)
            self.status_pub.publish(status_msg)

            # Check for stale data
            if self.last_lidar_time > 0:
                time_since_last = time.time() - self.last_lidar_time
                if time_since_last > 2.0:  # 2 seconds without data
                    self.healthy = False
                    self.get_logger().warn(
                        f'No LiDAR data received for {time_since_last:.1f}s - marking unhealthy'
                    )

        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('🛑 Shutting down LiDAR-to-Altitude Bridge Node...')
        self.get_logger().info(
            f'📊 Final Statistics:\n'
            f'   Messages Received: {self.stats["messages_received"]}\n'
            f'   Messages Published: {self.stats["messages_published"]}\n'
            f'   Last Altitude: {self.stats["last_altitude"]:.3f}m\n'
            f'   Last Vario: {self.stats["last_vario"]:+.3f}m/s'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarAltitudeBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
