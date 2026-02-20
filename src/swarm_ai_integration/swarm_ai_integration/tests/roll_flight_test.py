#!/usr/bin/env python3
"""
Roll Flight Test Node - Hardcoded flight sequence for testing drone capabilities

This node executes a predefined flight sequence to test basic drone movements:
1. Arm and setup flight modes (Angle mode + Alt Hold)
2. Throttle up to lift off (2 seconds)
3. Roll right and return to center (2 seconds each)
4. Roll left and return to center (2 seconds each)
5. Throttle down to land (2 seconds)

All RC commands are published to /fc/rc_override and automatically logged
by the black_box_recorder_node.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
import time
from enum import Enum


class FlightPhase(Enum):
    """Flight test sequence phases"""
    INIT = 0
    ARMED = 1
    THROTTLE_UP = 2
    ROLL_RIGHT = 3
    ROLL_CENTER_1 = 4
    ROLL_LEFT = 5
    ROLL_CENTER_2 = 6
    THROTTLE_DOWN = 7
    COMPLETE = 8


class RollFlightTestNode(Node):
    """
    Node that executes a hardcoded flight test sequence.

    RC Channel Mapping (AETR + AUX):
    - CH1 (index 0): Roll
    - CH2 (index 1): Pitch
    - CH3 (index 2): Throttle (MUST be 1000 when arming!)
    - CH4 (index 3): Yaw
    - CH5 (index 4): Arm/Disarm (1800 = armed, >1700)
    - CH6 (index 5): Angle Mode (1500 = enabled)
    - CH7 (index 6): Alt Hold (1800 = enabled, >1700)
    - CH8 (index 7): MSP Override (1800 = enabled, MUST be >1700)
    """

    def __init__(self):
        super().__init__('flight_test_node')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for RC override commands
        self.rc_pub = self.create_publisher(
            Float32MultiArray,
            '/fc/rc_override',
            qos_profile
        )

        # Flight test parameters
        self.declare_parameter('publish_rate_hz', 40.0)  # Match FC adapter rate
        self.declare_parameter('phase_duration_sec', 2.0)  # Duration for each flight action
        self.declare_parameter('arm_duration_sec', 10.0)  # Duration for ARM phase (must be longer!)
        self.declare_parameter('startup_delay_sec', 3.0)  # Delay before starting sequence

        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.phase_duration = self.get_parameter('phase_duration_sec').value
        self.arm_duration = self.get_parameter('arm_duration_sec').value
        self.startup_delay = self.get_parameter('startup_delay_sec').value

        # RC values for the test (based on working old tests)
        self.RC_NEUTRAL = 1500
        self.RC_THROTTLE_LOW = 1000  # CRITICAL: Must be low when arming!
        self.RC_THROTTLE_UP = 1520
        self.RC_THROTTLE_DOWN = 1480
        self.RC_ROLL_RIGHT = 1520
        self.RC_ROLL_LEFT = 1480
        self.RC_ARM = 1800  # >1700 to arm (was 2000, but tests use 1800)
        self.RC_DISARM = 1000
        self.RC_ANGLE_MODE = 1500
        self.RC_ALT_HOLD_OFF = 1500  # Alt Hold disabled during arming
        self.RC_ALT_HOLD_ON = 1800   # Alt Hold enabled during flight (>1700)
        self.RC_MSP_OVERRIDE = 1800

        # State tracking
        self.current_phase = FlightPhase.INIT
        self.phase_start_time = None
        self.test_start_time = None

        # Create timer for publishing RC commands
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.control_loop
        )

        self.get_logger().info('='*60)
        self.get_logger().info('Flight Test Node Initialized')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Flight action duration: {self.phase_duration} sec')
        self.get_logger().info(f'ARM duration: {self.arm_duration} sec')
        self.get_logger().info(f'Startup delay: {self.startup_delay} sec')
        self.get_logger().info('')
        self.get_logger().info('Test Sequence:')
        self.get_logger().info(f'  1. Startup delay ({self.startup_delay} sec)')
        self.get_logger().info(f'  2. ARM with throttle LOW ({self.arm_duration} sec) - CRITICAL!')
        self.get_logger().info(f'  3. Throttle UP to 1520 ({self.phase_duration} sec)')
        self.get_logger().info(f'  4. Roll RIGHT to 1520 ({self.phase_duration} sec)')
        self.get_logger().info(f'  5. Roll CENTER to 1500 ({self.phase_duration} sec)')
        self.get_logger().info(f'  6. Roll LEFT to 1480 ({self.phase_duration} sec)')
        self.get_logger().info(f'  7. Roll CENTER to 1500 ({self.phase_duration} sec)')
        self.get_logger().info(f'  8. Throttle DOWN to 1480 ({self.phase_duration} sec)')
        self.get_logger().info('  9. Disarm and complete')
        self.get_logger().info('')
        self.get_logger().info('Black box will automatically record all commands')
        self.get_logger().info('='*60)

    def control_loop(self):
        """Main control loop - called at publish_rate Hz"""
        current_time = time.time()

        # Initialize test start time
        if self.test_start_time is None:
            self.test_start_time = current_time
            self.get_logger().info(f'Starting flight test sequence in {self.startup_delay} seconds...')
            return

        # Wait for startup delay
        elapsed_since_start = current_time - self.test_start_time
        if elapsed_since_start < self.startup_delay:
            # Publish disarmed neutral commands during startup
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_NEUTRAL,
                yaw=self.RC_NEUTRAL,
                armed=False
            )
            return

        # Transition from INIT to ARMED after startup delay
        if self.current_phase == FlightPhase.INIT:
            self.transition_to_phase(FlightPhase.ARMED)

        # Check if current phase duration has elapsed
        if self.phase_start_time is not None:
            phase_elapsed = current_time - self.phase_start_time

            # Auto-advance phases based on duration
            if self.current_phase == FlightPhase.ARMED:
                # ARMED phase uses longer arm_duration
                if phase_elapsed >= self.arm_duration:
                    self.advance_to_next_phase()
            elif self.current_phase != FlightPhase.COMPLETE:
                # Other phases use standard phase_duration
                if phase_elapsed >= self.phase_duration:
                    self.advance_to_next_phase()

        # Execute current phase
        self.execute_current_phase()

    def execute_current_phase(self):
        """Execute RC commands for the current flight phase"""

        if self.current_phase == FlightPhase.ARMED:
            # Armed with THROTTLE LOW (required for arming), Alt Hold OFF
            # This matches the working old tests: [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_LOW,  # CRITICAL: Must be 1000 to arm!
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=False  # Keep Alt Hold off during arming
            )

        elif self.current_phase == FlightPhase.THROTTLE_UP:
            # Throttle up to lift off with Alt Hold enabled
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_UP,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True  # Enable Alt Hold for flight
            )

        elif self.current_phase == FlightPhase.ROLL_RIGHT:
            # Roll right
            self.publish_rc_command(
                roll=self.RC_ROLL_RIGHT,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_UP,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True
            )

        elif self.current_phase == FlightPhase.ROLL_CENTER_1:
            # Return to center after right roll
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_UP,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True
            )

        elif self.current_phase == FlightPhase.ROLL_LEFT:
            # Roll left
            self.publish_rc_command(
                roll=self.RC_ROLL_LEFT,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_UP,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True
            )

        elif self.current_phase == FlightPhase.ROLL_CENTER_2:
            # Return to center after left roll
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_UP,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True
            )

        elif self.current_phase == FlightPhase.THROTTLE_DOWN:
            # Throttle down to land
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_DOWN,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True
            )

        elif self.current_phase == FlightPhase.COMPLETE:
            # Test complete - disarm
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_NEUTRAL,
                yaw=self.RC_NEUTRAL,
                armed=False
            )

    def publish_rc_command(self, roll, pitch, throttle, yaw, armed, alt_hold=False):
        """
        Publish RC override command to flight controller

        Args:
            roll: RC value for roll (1000-2000)
            pitch: RC value for pitch (1000-2000)
            throttle: RC value for throttle (1000-2000)
            yaw: RC value for yaw (1000-2000)
            armed: Boolean indicating if drone should be armed
            alt_hold: Boolean indicating if alt hold should be enabled
        """
        msg = Float32MultiArray()

        # Initialize 16 channels (standard RC frame size)
        channels = [float(self.RC_NEUTRAL)] * 16

        # Set primary control channels
        channels[0] = float(roll)        # CH1 - Roll
        channels[1] = float(pitch)       # CH2 - Pitch
        channels[2] = float(throttle)    # CH3 - Throttle
        channels[3] = float(yaw)         # CH4 - Yaw

        # Set mode and arming channels
        channels[4] = float(self.RC_ARM if armed else self.RC_DISARM)  # CH5 - Arm
        channels[5] = float(self.RC_ANGLE_MODE)                        # CH6 - Angle Mode
        channels[6] = float(self.RC_ALT_HOLD_ON if alt_hold else self.RC_ALT_HOLD_OFF)  # CH7 - Alt Hold
        channels[7] = float(self.RC_MSP_OVERRIDE)                      # CH8 - MSP Override

        msg.data = channels
        self.rc_pub.publish(msg)

    def transition_to_phase(self, new_phase):
        """Transition to a new flight phase"""
        self.current_phase = new_phase
        self.phase_start_time = time.time()

        # Log phase transition
        phase_names = {
            FlightPhase.ARMED: 'ARMED - Ready to fly',
            FlightPhase.THROTTLE_UP: 'THROTTLE UP (1520)',
            FlightPhase.ROLL_RIGHT: 'ROLL RIGHT (1520)',
            FlightPhase.ROLL_CENTER_1: 'ROLL CENTER (1500)',
            FlightPhase.ROLL_LEFT: 'ROLL LEFT (1480)',
            FlightPhase.ROLL_CENTER_2: 'ROLL CENTER (1500)',
            FlightPhase.THROTTLE_DOWN: 'THROTTLE DOWN (1480)',
            FlightPhase.COMPLETE: 'TEST COMPLETE'
        }

        if new_phase in phase_names:
            self.get_logger().info(f'[Phase] {phase_names[new_phase]}')

    def advance_to_next_phase(self):
        """Advance to the next phase in the sequence"""
        phase_sequence = [
            FlightPhase.ARMED,
            FlightPhase.THROTTLE_UP,
            FlightPhase.ROLL_RIGHT,
            FlightPhase.ROLL_CENTER_1,
            FlightPhase.ROLL_LEFT,
            FlightPhase.ROLL_CENTER_2,
            FlightPhase.THROTTLE_DOWN,
            FlightPhase.COMPLETE
        ]

        current_index = phase_sequence.index(self.current_phase)
        if current_index < len(phase_sequence) - 1:
            next_phase = phase_sequence[current_index + 1]
            self.transition_to_phase(next_phase)

            # Stop publishing after test is complete
            if next_phase == FlightPhase.COMPLETE:
                self.get_logger().info('='*60)
                self.get_logger().info('Flight test sequence completed successfully!')
                self.get_logger().info('All data has been logged by black_box_recorder_node')
                self.get_logger().info('Check ~/swarm-ros/flight-logs/ for recorded data')
                self.get_logger().info('='*60)
                # Keep node running to maintain final disarmed state


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = RollFlightTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Flight test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
