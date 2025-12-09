#!/usr/bin/env python3
"""
Simple Test Node for FC Adapter (Joystick Mode)

Publishes action vectors to /ai/action and displays the resulting RC values from /fc/rc_override.
No physics simulation - just pure input/output testing.

Test Sequence:
  1. Hover
  2. Forward (50%)
  3. Forward (25%)
  4. Hover
  5. Backward (50%)
  6. Right (50%)
  7. Left (50%)
  8. Climb (50%)
  9. Descend (50%)
  10. Diagonal forward-right (70%)
  11. Final hover
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray


class TestMotionSimNode(Node):
    """Simple test node for fc_adapter joystick mode"""

    def __init__(self):
        super().__init__('test_motion_sim_node')

        # Parameters
        self.declare_parameter('update_rate_hz', 2.0)  # Slower for easier reading
        self.declare_parameter('test_sequence_enabled', True)
        self.declare_parameter('hold_duration_sec', 5.0)

        self.update_rate = float(self.get_parameter('update_rate_hz').value)
        self.test_sequence_enabled = bool(self.get_parameter('test_sequence_enabled').value)
        self.hold_duration = float(self.get_parameter('hold_duration_sec').value)

        # State
        self.test_index = 0
        self.hold_counter = 0
        self.hold_cycles = int(self.hold_duration * self.update_rate)
        self.last_rc_override = None

        # Test sequence: (action, description, duration_sec)
        # Format: [vx, vy, vz, speed], description, duration
        self.test_sequence = [
            ([0.0, 0.0, 0.0, 0.0], "Hover - no movement", 20.0),
            ([0.0, 0.0, 1.0, 1.0], "Up at 100% speed", 3.0),
            ([0.0, 0.0, 0.0, 0.0], "Hover - no movement", 3.0),
            ([1.0, 0.0, 0.0, 0.25], "Nort at 25% speed", 2.0),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop", 3.0),
            ([0.0, -1.0, 0.0, 0.25], "East at 25% speed", 2.0),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop", 3.0),
            ([-1.0, 0.0, 0.0, 0.25], "South at 25% speed", 2.0),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop", 3.0),
            ([0.0, 1.0, 0.0, 0.25], "West at 25% speed", 2.0),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop", 3.0),
            ([0.0, 0.0, -1.0, 1.0], "Down at 100% speed", 3.0),
            ([0.0, 0.0, 0.0, 0.0], "Hover - stop", 5.0),
        ]

        # Current action
        self.current_action = [0.0, 0.0, 0.0, 0.0]

        # QoS
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.action_pub = self.create_publisher(
            Float32MultiArray, '/ai/action', control_qos)

        # Subscribers
        self.create_subscription(
            Float32MultiArray, '/fc/rc_override', self.cb_rc_override, control_qos)

        # Timer
        self.update_period = 1.0 / self.update_rate
        self.create_timer(self.update_period, self.update_loop)

        self.get_logger().info(
            f'Test Motion Simulator started @ {self.update_rate}Hz\n'
            f'  Test sequence: {"ENABLED" if self.test_sequence_enabled else "DISABLED"}\n'
            f'  Hold duration: {self.hold_duration}s per test\n'
            f'  Total tests: {len(self.test_sequence)}'
        )

    def cb_rc_override(self, msg: Float32MultiArray):
        """Receive RC override output from fc_adapter"""
        self.last_rc_override = list(msg.data)

    def update_loop(self):
        """Main loop - publish action and display RC output"""

        # Update test sequence
        if self.test_sequence_enabled:
            self._update_test_sequence()

        # Publish action
        action_msg = Float32MultiArray()
        action_msg.data = [float(x) for x in self.current_action]
        self.action_pub.publish(action_msg)

        # Display current state
        self._print_state()

    def _update_test_sequence(self):
        """Update action based on test sequence"""
        self.hold_counter += 1

        # Get current test duration
        _, _, current_duration = self.test_sequence[self.test_index]
        current_hold_cycles = int(current_duration * self.update_rate)

        if self.hold_counter >= current_hold_cycles:
            # Move to next test
            self.hold_counter = 0
            self.test_index = (self.test_index + 1) % len(self.test_sequence)

            action, description, duration = self.test_sequence[self.test_index]
            self.current_action = list(action)

            self.get_logger().info(
                f'\n{"="*80}\n'
                f'  TEST {self.test_index + 1}/{len(self.test_sequence)}: {description} ({duration}s)\n'
                f'  Action: [vx={action[0]:+.1f}, vy={action[1]:+.1f}, vz={action[2]:+.1f}, speed={action[3]:.1f}]\n'
                f'{"="*80}'
            )

    def _print_state(self):
        """Print current action and RC output"""
        vx, vy, vz, speed = self.current_action

        # Format RC output
        if self.last_rc_override and len(self.last_rc_override) >= 4:
            rc_str = (
                f"[R={int(self.last_rc_override[0])}, "
                f"P={int(self.last_rc_override[1])}, "
                f"T={int(self.last_rc_override[2])}, "
                f"Y={int(self.last_rc_override[3])}]"
            )
            if len(self.last_rc_override) >= 8:
                rc_str += (
                    f" AUX[ARM={int(self.last_rc_override[4])}, "
                    f"ANG={int(self.last_rc_override[5])}, "
                    f"ALT={int(self.last_rc_override[6])}, "
                    f"MSP={int(self.last_rc_override[7])}]"
                )
        else:
            rc_str = "[waiting for fc_adapter...]"

        self.get_logger().info(
            f'Action=[{vx:+.2f}, {vy:+.2f}, {vz:+.2f}, s={speed:.2f}] → RC_OUT={rc_str}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestMotionSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Test Motion Simulator')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
