#!/usr/bin/env python3
"""
Telemetry Publisher - Manages ROS topic publishing for flight controller telemetry

This module handles:
- Creating and managing ROS publishers
- Publishing telemetry messages
- Logging publication events
- Tracking last published values
"""

from typing import Dict, Any, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray, String, Bool


class TelemetryPublisher:
    """
    Manages ROS publishers for flight controller telemetry data.

    This class encapsulates all ROS publishing logic, keeping it
    separate from protocol handling and data parsing.
    """

    def __init__(self, node: Node):
        """
        Initialize telemetry publishers.

        Args:
            node: ROS2 node instance
        """
        self.node = node

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Storage for last published values
        self.last_telemetry: Dict[str, Any] = {
            'imu': None,
            'gps': None,
            'attitude': None,
            'attitude_euler': None,
            'status': None,
            'battery': None
        }

        # Create publishers
        self._create_publishers()

    def _create_publishers(self):
        """Create all ROS publishers"""
        # IMU data
        self.imu_pub = self.node.create_publisher(
            Imu, '/fc/imu_raw', self.sensor_qos
        )

        # GPS data
        self.gps_pub = self.node.create_publisher(
            NavSatFix, '/fc/gps_fix', self.sensor_qos
        )
        self.gps_speed_course_pub = self.node.create_publisher(
            Float32MultiArray, '/fc/gps_speed_course', self.sensor_qos
        )

        # Attitude data
        self.attitude_pub = self.node.create_publisher(
            QuaternionStamped, '/fc/attitude', self.reliable_qos
        )
        self.attitude_euler_pub = self.node.create_publisher(
            Vector3Stamped, '/fc/attitude_euler', self.reliable_qos
        )

        # Status
        self.status_pub = self.node.create_publisher(
            String, '/fc/status', self.reliable_qos
        )
        self.msp_status_pub = self.node.create_publisher(
            Float32MultiArray, '/fc/msp_status', self.reliable_qos
        )

        # Battery
        self.battery_pub = self.node.create_publisher(
            BatteryState, '/fc/battery', self.sensor_qos
        )

        # Connection status
        self.connected_pub = self.node.create_publisher(
            Bool, '/fc/connected', self.reliable_qos
        )

        # Motor data
        self.motor_pub = self.node.create_publisher(
            Float32MultiArray, '/fc/motor_rpm', self.sensor_qos
        )

        # Waypoint data
        self.waypoint_pub = self.node.create_publisher(
            Float32MultiArray, '/fc/waypoint', self.reliable_qos
        )

        self.node.get_logger().info('✅ Telemetry publishers created')

    def publish_imu(self, imu_msg: Imu):
        """Publish IMU data"""
        self.imu_pub.publish(imu_msg)
        self.last_telemetry['imu'] = imu_msg

        self.node.get_logger().info(
            f'   ➜ Published to /fc/imu_raw | '
            f'acc=[{imu_msg.linear_acceleration.x:.2f}, '
            f'{imu_msg.linear_acceleration.y:.2f}, '
            f'{imu_msg.linear_acceleration.z:.2f}] m/s² | '
            f'gyro=[{imu_msg.angular_velocity.x:.3f}, '
            f'{imu_msg.angular_velocity.y:.3f}, '
            f'{imu_msg.angular_velocity.z:.3f}] rad/s'
        )

    def publish_gps(self, gps_msg: NavSatFix, speed_course_msg: Optional[Float32MultiArray] = None):
        """Publish GPS data and optional speed/course"""
        self.gps_pub.publish(gps_msg)
        self.last_telemetry['gps'] = gps_msg

        if speed_course_msg:
            self.gps_speed_course_pub.publish(speed_course_msg)
            speed_mps, course_deg = speed_course_msg.data[0], speed_course_msg.data[1]
            speed_info = f'speed={speed_mps:.2f}m/s | course={course_deg:.1f}° | '
        else:
            speed_info = ''

        self.node.get_logger().info(
            f'   ➜ Published to /fc/gps_fix | '
            f'{speed_info}'
            f'lat={gps_msg.latitude:.7f} | lon={gps_msg.longitude:.7f} | '
            f'alt={gps_msg.altitude:.1f}m | fix={gps_msg.status.status}'
        )

    def publish_attitude(
        self,
        quat_msg: QuaternionStamped,
        euler_msg: Optional[Vector3Stamped] = None
    ):
        """Publish attitude data (quaternion and Euler angles)"""
        self.attitude_pub.publish(quat_msg)
        self.last_telemetry['attitude'] = quat_msg

        if euler_msg:
            self.attitude_euler_pub.publish(euler_msg)
            self.last_telemetry['attitude_euler'] = euler_msg

            import numpy as np
            self.node.get_logger().info(
                f'   ➜ Published to /fc/attitude | '
                f'roll={np.degrees(euler_msg.vector.x):.1f}° | '
                f'pitch={np.degrees(euler_msg.vector.y):.1f}° | '
                f'yaw={np.degrees(euler_msg.vector.z):.1f}°'
            )

    def publish_status(self, status_msg: String, msp_status_msg: Optional[Float32MultiArray] = None):
        """Publish status data"""
        self.status_pub.publish(status_msg)
        self.last_telemetry['status'] = status_msg

        if msp_status_msg:
            self.msp_status_pub.publish(msp_status_msg)

        self.node.get_logger().info(f'   ➜ Published to /fc/status | {status_msg.data}')

    def publish_battery(self, battery_msg: BatteryState):
        """Publish battery data"""
        self.battery_pub.publish(battery_msg)
        self.last_telemetry['battery'] = battery_msg

        power = battery_msg.voltage * battery_msg.current
        self.node.get_logger().info(
            f'   ➜ Published to /fc/battery | '
            f'voltage={battery_msg.voltage:.2f}V | '
            f'current={battery_msg.current:.2f}A | '
            f'power={power:.2f}W'
        )

    def publish_motor(self, motor_msg: Float32MultiArray):
        """Publish motor data"""
        self.motor_pub.publish(motor_msg)

        motors = motor_msg.data
        motor_str = ', '.join([f'M{i+1}={int(m)}' for i, m in enumerate(motors)])
        self.node.get_logger().info(f'   ➜ Published to /fc/motor_rpm | {motor_str}')

    def publish_waypoint(self, waypoint_msg: Float32MultiArray):
        """Publish waypoint data"""
        self.waypoint_pub.publish(waypoint_msg)

        wp = waypoint_msg.data
        self.node.get_logger().info(
            f'   ➜ Published to /fc/waypoint | '
            f'wp#{int(wp[0])} lat={wp[1]:.7f} lon={wp[2]:.7f} | '
            f'alt={wp[3]:.1f}m | heading={wp[4]:.1f}° | '
            f'stay={int(wp[5])}s | navflag=0x{int(wp[6]):02x}'
        )

    def publish_connection_status(self, connected: bool):
        """Publish connection status"""
        msg = Bool()
        msg.data = connected
        self.connected_pub.publish(msg)

    def get_last_telemetry(self, key: str) -> Optional[Any]:
        """Get last published telemetry value"""
        return self.last_telemetry.get(key)

    def get_all_telemetry(self) -> Dict[str, Any]:
        """Get all last published telemetry values"""
        return self.last_telemetry.copy()
