#!/usr/bin/env python3
"""
FC Adapter Node - Simple Joystick Mode with Yaw Alignment

Translates /ai/action vectors directly to RC values like a joystick.
No PID, no sensor feedback - pure open-loop control.

Publishes RC values to /fc/rc_override topic for fc_comms_node to send via MSP.

Startup Sequence:
  1. Arming phase: Send ARM=1800 with throttle=1000 (2s default)
  2. Warmup phase: Remain armed with throttle=1000, ALT HOLD disabled (10s default)
  3. Yaw alignment phase: Rise drone (2s @ throttle 1550), then align to north (0°)
     - If heading < 350°: turn right (yaw 1520) for 0.5s
     - If heading > 10°: turn left (yaw 1480) for 0.5s
     - Repeat until heading is within 350-360° or 0-10° range
  4. AI control phase: Normal operation with AI model commands

ENU Coordinate System:
  X = forward (East)
  Y = right (North)
  Z = up

RC Channel Mapping (AERT + AUX):
  CH1: ROLL    (vy control - right/left)
  CH2: PITCH   (vx control - forward/back)
  CH3: YAW     (1500 during AI control, variable during alignment)
  CH4: THROTTLE (vz control - up/down in ALT HOLD)
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
from geometry_msgs.msg import Vector3Stamped

from swarm_ai_integration.utils import YawAlignmentController, normalize_heading_to_360


class FCAdapterNode(Node):
    """
    FC Adapter Node - Joystick Mode with Yaw Alignment

    Subscriptions:
      /ai/action                 (std_msgs/Float32MultiArray) : [vx, vy, vz, speed]
      /safety/override           (std_msgs/Bool)
      /fc/attitude_euler         (geometry_msgs/Vector3Stamped) : [roll, pitch, yaw] in radians

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

        # Arming, warmup and safety
        self.declare_parameter('arming_duration_sec', 2.0)
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

        self.arming_duration = float(self.get_parameter('arming_duration_sec').value)
        self.warmup_duration = float(self.get_parameter('warmup_duration_sec').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout').value)

        # ------------ State ------------
        self.last_action = [0.0, 0.0, 0.0, 0.0]  # [vx, vy, vz, speed]
        self.last_cmd_time = time.time()
        self.safety_override = False

        # Arming state
        self.arming_complete = False
        self.arming_start_time = time.time()

        # Warmup state
        self.warmup_complete = False
        self.warmup_start_time = None  # Will be set after arming complete

        # Yaw alignment state
        self.yaw_alignment_complete = False
        self.current_heading_rad = 0.0  # Current heading in radians from /fc/attitude_euler

        # ------------ QoS & ROS I/O ------------
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.create_subscription(Float32MultiArray, '/ai/action', self.cb_ai_action, control_qos)
        self.create_subscription(Bool, '/safety/override', self.cb_safety, control_qos)
        self.create_subscription(Vector3Stamped, '/fc/attitude_euler', self.cb_attitude, sensor_qos)

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
            f'  Arming: {self.arming_duration}s, Warmup: {self.warmup_duration}s\n'
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

    def cb_attitude(self, msg: Vector3Stamped):
        """Receive attitude data (roll, pitch, yaw in radians)"""
        self.current_heading_rad = float(msg.vector.z)  # yaw in radians

    # ------------ Yaw Alignment Helper Methods ------------
    def get_current_heading_degrees(self) -> float:
        """
        Get current heading in degrees (0-360).

        Returns:
            Current heading in degrees
        """
        return normalize_heading_to_360(self.current_heading_rad)

    def send_rc_command_for_yaw_alignment(self, roll: int, pitch: int, throttle: int, yaw: int):
        """
        Send RC command during yaw alignment sequence.

        This method is called by YawAlignmentController.

        Args:
            roll: Roll channel value (1000-2000)
            pitch: Pitch channel value (1000-2000)
            throttle: Throttle channel value (1000-2000)
            yaw: Yaw channel value (1000-2000)
        """
        channels = [
            roll,        # CH1: ROLL
            pitch,       # CH2: PITCH
            yaw,         # CH3: YAW
            throttle,    # CH4: THROTTLE
            1800,        # CH5: ARM (high)
            1500,        # CH6: ANGLE mode (high)
            1800,        # CH7: ALT HOLD (high - active during yaw alignment)
            1800,        # CH8: MSP RC OVERRIDE (high)
        ]
        self._publish_rc_override(channels)

    def _execute_yaw_alignment(self):
        """
        Execute yaw alignment sequence.

        This method is called once after warmup to align drone heading to north.
        It blocks the control loop until alignment is complete.
        """
        self.get_logger().info('🎯 Starting yaw alignment sequence...')

        # Create YawAlignmentController instance
        yaw_controller = YawAlignmentController(
            node=self,
            send_rc_command_callback=self.send_rc_command_for_yaw_alignment,
            get_heading_callback=self.get_current_heading_degrees,
            rise_duration=2.0,           # Rise for 2 seconds
            rise_throttle=1550,          # Throttle at 1550
            yaw_turn_duration=0.5,       # Turn for 0.5 seconds per correction
            yaw_right_value=1520,        # Right turn value
            yaw_left_value=1480,         # Left turn value
            heading_tolerance_low=350.0, # Lower tolerance (350°)
            heading_tolerance_high=10.0  # Upper tolerance (10°)
        )

        # Execute the full yaw alignment sequence (blocking)
        success = yaw_controller.execute_full_sequence()

        # Mark alignment as complete regardless of success
        # (to avoid getting stuck in alignment loop)
        self.yaw_alignment_complete = True

        if success:
            self.get_logger().info('✅ Yaw alignment complete - ready for AI control')
        else:
            self.get_logger().warn('⚠️ Yaw alignment incomplete - proceeding to AI control anyway')

    # ------------ Control Loop ------------
    def control_loop(self):
        """Main control loop - translates action to RC values"""
        now = time.time()

        # Phase 0: Arming (send ARM channel high with throttle at 1000)
        if not self.arming_complete:
            if (now - self.arming_start_time) >= self.arming_duration:
                self.arming_complete = True
                self.warmup_start_time = now  # Start warmup timer
                self.get_logger().info(f'✅ Arming complete ({self.arming_duration}s) - proceeding to warmup')
            else:
                # During arming: send ARM high with throttle at 1000
                self._send_arming_command()
                return

        # Phase 1: Warmup
        if not self.warmup_complete:
            if (now - self.warmup_start_time) >= self.warmup_duration:
                self.warmup_complete = True
                self.get_logger().info(f'✅ Warmup complete ({self.warmup_duration}s) - proceeding to yaw alignment')
            else:
                # During warmup: send neutral position with throttle low
                self._send_warmup_command()
                return

        # Phase 2: Yaw Alignment (after warmup, before AI control)
        if not self.yaw_alignment_complete:
            self._execute_yaw_alignment()
            return

        # Check for command timeout or safety
        if (now - self.last_cmd_time) > self.cmd_timeout:
            self._send_hover_command(reason='timeout')
            return

        if self.safety_override:
            self._send_hover_command(reason='safety')
            return

        # Phase 3: Normal AI control operation
        self._send_action_command()

    def _send_arming_command(self):
        """
        Send arming RC values (ARM high, throttle at 1000).

        AERT1234 channel order:
        - A (Aileron/Roll) - CH1
        - E (Elevator/Pitch) - CH2
        - R (Rudder/Yaw) - CH3
        - T (Throttle) - CH4
        - AUX1 (ARM) - CH5
        - AUX2 (ANGLE) - CH6
        - AUX3 (ALT HOLD) - CH7
        - AUX4 (MSP Override) - CH8
        """
        channels = [
            self.rc_mid,  # CH1: ROLL (neutral)
            self.rc_mid,  # CH2: PITCH (neutral)
            self.rc_mid,  # CH3: YAW (neutral)
            1000,         # CH4: THROTTLE (1000 for arming)
            1800,         # CH5: ARM (high - arming)
            1500,         # CH6: ANGLE mode (high)
            1000,         # CH7: ALT HOLD (off during arming)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]
        self._publish_rc_override(channels)

    def _send_warmup_command(self):
        """Send warmup RC values (armed, low throttle, no ALT HOLD)"""
        channels = [
            self.rc_mid,  # CH1: ROLL (neutral)
            self.rc_mid,  # CH2: PITCH (neutral)
            self.rc_mid,  # CH3: YAW (neutral)
            1000,         # CH4: THROTTLE (1000 for low throttle)
            1800,         # CH5: ARM (high)
            1500,         # CH6: ANGLE mode (high)
            1000,         # CH7: ALT HOLD (off during warmup)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]
        self._publish_rc_override(channels)

    def _send_hover_command(self, reason: str):
        """Send hover RC values (neutral with ALT HOLD active)"""
        channels = [
            self.rc_mid,  # CH1: ROLL (neutral)
            self.rc_mid,  # CH2: PITCH (neutral)
            self.rc_mid,  # CH3: YAW (neutral)
            self.rc_mid,  # CH4: THROTTLE (neutral in ALT HOLD = maintain altitude)
            1800,         # CH5: ARM (high)
            1500,         # CH6: ANGLE mode (high)
            1800,         # CH7: ALT HOLD (high)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]
        self._publish_rc_override(channels)
        self.get_logger().info(f'Hover command sent ({reason})', throttle_duration_sec=2.0)

    def _send_action_command(self):
        """
        Translate action vector to RC channels.

        The vector (vx, vy, vz, speed) is interpreted as:
        - Direction: (vx, vy, vz) defines the direction (will be normalized)
        - Speed: scalar (0-1) defines the magnitude

        Normalization approach:
        - Find max absolute component in (vx, vy, vz)
        - Normalize each component by this max value
        - Scale by speed
        - Map to RC range: -1 → rc_min (1450), 0 → rc_mid (1500), +1 → rc_max (1550)

        Examples:
        - (1, 1, 1, 1) and (0.25, 0.25, 0.25, 1) produce the same output (same direction & speed)
        - (1, -1, 0, 1) → pitch=1550, roll=1450, throttle=1500
        """
        vx, vy, vz, speed = self.last_action

        # Find max absolute component for normalization
        max_component = max(abs(vx), abs(vy), abs(vz))

        if max_component > 1e-6:
            # Normalize direction by max component
            vx_norm = vx / max_component
            vy_norm = vy / max_component
            vz_norm = vz / max_component

            # Scale by speed (0-1)
            vx_scaled = vx_norm * speed
            vy_scaled = vy_norm * speed
            vz_scaled = vz_norm * speed
        else:
            # No direction specified, stay neutral
            vx_scaled = 0.0
            vy_scaled = 0.0
            vz_scaled = 0.0

        # Calculate half range for mapping
        half_range = self.rc_max - self.rc_mid  # 50 units

        # Map scaled components to RC values
        # vx (forward) → PITCH (CH2): positive vx = higher pitch
        pitch_rc = self.rc_mid + int(vx_scaled * half_range)

        # vy (right) → ROLL (CH1): positive vy = higher roll
        roll_rc = self.rc_mid + int(vy_scaled * half_range)

        # vz (up) → THROTTLE (CH3): positive vz = higher throttle
        throttle_rc = self.rc_mid + int(vz_scaled * half_range)

        # Clamp to RC limits
        roll_rc = max(self.rc_min, min(self.rc_max, roll_rc))
        pitch_rc = max(self.rc_min, min(self.rc_max, pitch_rc))
        throttle_rc = max(self.rc_min, min(self.rc_max, throttle_rc))

        # Build channel array (AERT order)
        channels = [
            roll_rc,      # CH1: ROLL
            pitch_rc,     # CH2: PITCH
            self.rc_mid,  # CH3: YAW (always neutral)
            throttle_rc,  # CH4: THROTTLE
            1800,         # CH5: ARM (high)
            1500,         # CH6: ANGLE mode (high)
            1800,         # CH7: ALT HOLD (high)
            1800,         # CH8: MSP RC OVERRIDE (high)
        ]

        # Publish to topic
        self._publish_rc_override(channels)

        # Log action with normalized values
        self.get_logger().info(
            f'Action=[{vx:+.2f}, {vy:+.2f}, {vz:+.2f}, s={speed:.2f}] '
            f'Norm=[{vx_scaled:+.2f}, {vy_scaled:+.2f}, {vz_scaled:+.2f}] → '
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

        if not self.arming_complete:
            elapsed = now - self.arming_start_time
            remaining = max(0, self.arming_duration - elapsed)
            flags.append(f'ARMING({remaining:.1f}s)')
        elif not self.warmup_complete:
            elapsed = now - self.warmup_start_time
            remaining = max(0, self.warmup_duration - elapsed)
            flags.append(f'WARMUP({remaining:.1f}s)')
        elif not self.yaw_alignment_complete:
            heading = self.get_current_heading_degrees()
            flags.append(f'YAW_ALIGN({heading:.1f}°)')
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
