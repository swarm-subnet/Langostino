#!/usr/bin/env python3
"""
FC Adapter Node - Simple Joystick Mode

Translates /ai/action vectors directly to RC values like a joystick.
No PID, no sensor feedback - pure open-loop control.

Publishes RC values to /fc/rc_override topic for fc_comms_node to send via MSP.

ENU Coordinate System:
  X = forward (East)
  Y = right (North)
  Z = up

RC Channel Mapping (AETR + AUX):
  CH1: ROLL    (vy control - right/left)
  CH2: PITCH   (vx control - forward/back)
  CH3: THROTTLE (vz control - up/down in ALT HOLD)
  CH4: YAW     (always 1500 - no rotation)
  CH5: ARM     (AUX1 - always 1800 when armed)
  CH6: ANGLE   (AUX2 - always 1500 for angle mode)
  CH7: ALT_HOLD (AUX3 - 1000 during warmup, 1800 after)
  CH8: MSP_OVERRIDE (AUX4 - always 1800)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool, String


class FCAdapterNode(Node):
    """
    FC Adapter Node - Joystick Mode
    Subscriptions:
      /ai/action                 (std_msgs/Float32MultiArray) : [vx, vy, vz, speed]
      /safety/override           (std_msgs/Bool)

    Publications:
      /fc_adapter/status         (std_msgs/String)
      /fc/rc_override            (std_msgs/Float32MultiArray) : [ch1, ch2, ..., ch9]
    """

    def __init__(self):
        super().__init__('fc_adapter_node')

        # ------------ Parameters ------------
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('rc_mid_value', 1500)
        self.declare_parameter('rc_min_value', 1450)
        self.declare_parameter('rc_max_value', 1550)

        # Mapping gains (how much RC deflection per unit of velocity command)
        self.declare_parameter('vx_to_pitch_gain', 50.0)   # RC units per m/s
        self.declare_parameter('vy_to_roll_gain', 50.0)    # RC units per m/s
        self.declare_parameter('vz_to_throttle_gain', 100.0)  # RC units per m/s

        # Warmup and safety
        self.declare_parameter('warmup_duration_sec', 10.0)
        self.declare_parameter('command_timeout', 1.0)

        # Get parameter values
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.rc_mid = int(self.get_parameter('rc_mid_value').value)
        self.rc_min = int(self.get_parameter('rc_min_value').value)
        self.rc_max = int(self.get_parameter('rc_max_value').value)

        self.vx_gain = float(self.get_parameter('vx_to_pitch_gain').value)
        self.vy_gain = float(self.get_parameter('vy_to_roll_gain').value)
        self.vz_gain = float(self.get_parameter('vz_to_throttle_gain').value)

        self.warmup_duration = float(self.get_parameter('warmup_duration_sec').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout').value)

        # ------------ State ------------
        self.last_action = [0.0, 0.0, 0.0, 0.0]  # [vx, vy, vz, speed]
        self.last_cmd_time = time.time()
        self.safety_override = False

        # Warmup state
        self.warmup_complete = False
        self.warmup_start_time = time.time()

        # ------------ QoS & ROS I/O ------------
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.create_subscription(Float32MultiArray, '/ai/action', self.cb_ai_action, control_qos)
        self.create_subscription(Bool, '/safety/override', self.cb_safety, control_qos)

        # Publications
        self.status_pub = self.create_publisher(String, '/fc_adapter/status', control_qos)
        self.rc_override_pub = self.create_publisher(Float32MultiArray, '/fc/rc_override', control_qos)

        # Timer
        self.control_period = 1.0 / max(1.0, self.control_rate)
        self.create_timer(self.control_period, self.control_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'FC Adapter Node (Joystick Mode) started @ {self.control_rate}Hz\n'
            f'  RC range: [{self.rc_min}, {self.rc_mid}, {self.rc_max}]\n'
            f'  Gains: vx→pitch={self.vx_gain}, vy→roll={self.vy_gain}, vz→throttle={self.vz_gain}\n'
            f'  Warmup: {self.warmup_duration}s\n'
            f'  Publishing to: /fc/rc_override'
        )

    # ------------ Callbacks ------------
    def cb_ai_action(self, msg: Float32MultiArray):
        """Receive AI action command"""
        data = list(msg.data)
        if len(data) < 4:
            return

        self.last_action = [
            float(data[0]),  # vx (forward)
            float(data[1]),  # vy (right)
            float(data[2]),  # vz (up)
            float(data[3])   # speed (0-1 scale)
        ]
        self.last_cmd_time = time.time()

    def cb_safety(self, msg: Bool):
        """Safety override - emergency hover"""
        self.safety_override = bool(msg.data)
        if self.safety_override:
            self.get_logger().warn('⚠️ Safety override ON → emergency hover')

    # ------------ Control Loop ------------
    def control_loop(self):
        """Main control loop - translates action to RC values"""
        now = time.time()

        # Check warmup status
        if not self.warmup_complete:
            if (now - self.warmup_start_time) >= self.warmup_duration:
                self.warmup_complete = True
                self.get_logger().info(f'✅ Warmup complete ({self.warmup_duration}s) - switching to ALT HOLD mode')
            else:
                # During warmup: send neutral position with throttle low
                self._send_warmup_command()
                return

        # Check for command timeout or safety
        if (now - self.last_cmd_time) > self.cmd_timeout:
            self._send_hover_command(reason='timeout')
            return

        if self.safety_override:
            self._send_hover_command(reason='safety')
            return

        # Normal operation: translate action to RC
        self._send_action_command()

    def _send_warmup_command(self):
        """Send warmup RC values (armed, low throttle, no ALT HOLD)"""
        channels = [
            self.rc_mid,  # CH1: Roll (neutral)
            self.rc_mid,  # CH2: Pitch (neutral)
            1000,         # CH3: Throttle (low for warmup)
            self.rc_mid,  # CH4: Yaw (neutral)
            1800,         # CH5: ARM (high)
            1500,         # CH6: ANGLE mode (high)
            1000,         # CH7: ALT HOLD (off during warmup)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]
        self._publish_rc_override(channels)

    def _send_hover_command(self, reason: str):
        """Send hover RC values (neutral with ALT HOLD active)"""
        channels = [
            self.rc_mid,  # CH1: Roll (neutral)
            self.rc_mid,  # CH2: Pitch (neutral)
            self.rc_mid,  # CH3: Throttle (neutral in ALT HOLD = maintain altitude)
            self.rc_mid,  # CH4: Yaw (neutral)
            1800,         # CH5: ARM (high)
            1500,         # CH6: ANGLE mode (high)
            1800,         # CH7: ALT HOLD (high)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]
        self._publish_rc_override(channels)
        self.get_logger().info(f'Hover command sent ({reason})', throttle_duration_sec=2.0)

    def _send_action_command(self):
        """Translate action vector to RC channels"""
        vx, vy, vz, speed = self.last_action

        # Scale horizontal commands by speed
        vx_scaled = vx * speed
        vy_scaled = vy * speed
        vz_scaled = vz * speed

        # Map to RC values
        # vx (forward) → PITCH (CH2): positive vx = higher pitch
        pitch_rc = self.rc_mid + int(vx_scaled * self.vx_gain)

        # vy (right) → ROLL (CH1): negative vy = higher roll (inverted for correct direction)
        roll_rc = self.rc_mid - int(vy_scaled * self.vy_gain)

        # vz (up) → THROTTLE (CH3): positive vz = higher throttle
        throttle_rc = self.rc_mid + int(vz_scaled * self.vz_gain)

        # Clamp to RC limits
        roll_rc = max(self.rc_min, min(self.rc_max, roll_rc))
        pitch_rc = max(self.rc_min, min(self.rc_max, pitch_rc))
        throttle_rc = max(self.rc_min, min(self.rc_max, throttle_rc))

        # Build channel array
        channels = [
            roll_rc,      # CH1: ROLL
            pitch_rc,     # CH2: PITCH
            throttle_rc,  # CH3: THROTTLE
            self.rc_mid,  # CH4: YAW (always neutral)
            1800,         # CH5: ARM (high)
            1500,         # CH6: ANGLE mode (high)
            1800,         # CH7: ALT HOLD (high)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]

        # Publish to topic
        self._publish_rc_override(channels)

        # Log action
        self.get_logger().info(
            f'Action=[{vx:+.2f}, {vy:+.2f}, {vz:+.2f}, s={speed:.2f}] → '
            f'RC[R={roll_rc}, P={pitch_rc}, T={throttle_rc}, Y={self.rc_mid}]'
        )

    # ------------ RC Publishing ------------
    def _publish_rc_override(self, channels: list):
        """Publish RC override values to topic for fc_comms_node"""
        rc_msg = Float32MultiArray()
        rc_msg.data = [float(ch) for ch in channels]
        self.rc_override_pub.publish(rc_msg)

    # ------------ Status Publishing ------------
    def publish_status(self):
        """Publish node status"""
        flags = []
        now = time.time()

        if not self.warmup_complete:
            elapsed = now - self.warmup_start_time
            remaining = max(0, self.warmup_duration - elapsed)
            flags.append(f'WARMUP({remaining:.1f}s)')
        else:
            flags.append('READY')

        if self.safety_override:
            flags.append('SAFETY_OVERRIDE')

        if (now - self.last_cmd_time) > self.cmd_timeout:
            flags.append('CMD_TIMEOUT')

        flags.append('JOYSTICK_MODE')

        s = String()
        s.data = ' | '.join(flags)
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = FCAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down FC Adapter Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
