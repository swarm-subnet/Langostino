#!/usr/bin/env python3
"""
Simple Flight Test Node - Basic altitude test sequence

This node executes a simplified flight sequence to test basic altitude control:
1. Arm in Angle mode (Alt Hold disabled at 900)
2. Wait 10 seconds after arming
3. Throttle up to 1510 in Angle mode to rise
4. Switch to Alt Hold (1500) and hover for 15 seconds
5. Throttle down to 1480 to land (Alt Hold maintained)

All RC commands are published to /fc/rc_override and automatically logged
by the black_box_recorder_node.

Author: ROS2 Swarm Project
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
    HOVER = 3
    THROTTLE_DOWN = 4
    COMPLETE = 5


class SimpleFlightTestNode(Node):
    """
    Node that executes a simple altitude test sequence.

    RC Channel Mapping (AETR + AUX):
    - CH1 (index 0): Roll
    - CH2 (index 1): Pitch
    - CH3 (index 2): Throttle (MUST be 1000 when arming!)
    - CH4 (index 3): Yaw
    - CH5 (index 4): Arm/Disarm (1800 = armed, >1700)
    - CH6 (index 5): Angle Mode (1500 = enabled)
    - CH7 (index 6): Alt Hold (1500 = enabled/hover, 900 = disabled)
    - CH8 (index 7): MSP Override (1800 = enabled, MUST be >1700)
    """

    def __init__(self):
        super().__init__('simple_flight_test_node')

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
        self.declare_parameter('arm_duration_sec', 10.0)  # Wait after arming
        self.declare_parameter('throttle_up_duration_sec', 1.0)  # Rise duration
        self.declare_parameter('hover_duration_sec', 15.0)  # Hover duration
        self.declare_parameter('landing_duration_sec', 5.0)  # Landing duration
        self.declare_parameter('startup_delay_sec', 3.0)  # Delay before starting

        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.arm_duration = self.get_parameter('arm_duration_sec').value
        self.throttle_up_duration = self.get_parameter('throttle_up_duration_sec').value
        self.hover_duration = self.get_parameter('hover_duration_sec').value
        self.landing_duration = self.get_parameter('landing_duration_sec').value
        self.startup_delay = self.get_parameter('startup_delay_sec').value

        # RC values for the test
        self.RC_NEUTRAL = 1500
        self.RC_THROTTLE_LOW = 1000  # CRITICAL: Must be low when arming!
        self.RC_THROTTLE_UP = 1300   # Rise throttle value (reduced from 1520 for gentler climb)
        self.RC_THROTTLE_DOWN = 1300  # Landing throttle value
        self.RC_ARM = 1800  # >1700 to arm
        self.RC_DISARM = 1000
        self.RC_ANGLE_MODE = 1500
        self.RC_ALT_HOLD_OFF = 900   # Alt Hold disabled
        self.RC_ALT_HOLD_ON = 1400   # Alt Hold enabled for hovering
        self.RC_MSP_OVERRIDE = 1800

        # State tracking
        self.current_phase = FlightPhase.INIT
        self.phase_start_time = None
        self.test_start_time = None
        self.last_print_time = None  # For periodic status updates

        # Create timer for publishing RC commands
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.control_loop
        )

        print('='*60)
        print('Simple Flight Test Node Initialized')
        print('='*60)
        print(f'Publish rate: {self.publish_rate} Hz')
        print(f'ARM duration: {self.arm_duration} sec')
        print(f'Throttle up duration: {self.throttle_up_duration} sec')
        print(f'Hover duration: {self.hover_duration} sec')
        print(f'Landing duration: {self.landing_duration} sec')
        print(f'Startup delay: {self.startup_delay} sec')
        print('')
        print('Test Sequence:')
        print(f'  1. Startup delay ({self.startup_delay} sec)')
        print(f'  2. ARM with throttle LOW ({self.arm_duration} sec)')
        print(f'  3. Throttle UP to {self.RC_THROTTLE_UP} ({self.throttle_up_duration} sec)')
        print(f'  4. Hover at neutral 1500 ({self.hover_duration} sec)')
        print(f'  5. Throttle DOWN to 1480 ({self.landing_duration} sec)')
        print('  6. Disarm and complete')
        print('')
        print('Black box will automatically record all commands')
        print(f'Logs will be saved to: ~/swarm-ros/flight-logs/')
        print('='*60)

        self.get_logger().info('Simple Flight Test Node Initialized')

    def control_loop(self):
        """Main control loop - called at publish_rate Hz"""
        current_time = time.time()

        # Initialize test start time
        if self.test_start_time is None:
            self.test_start_time = current_time
            print(f'\n[INIT] Starting simple flight test in {self.startup_delay} seconds...')
            self.get_logger().info(f'Starting simple flight test in {self.startup_delay} seconds...')
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

        # Print periodic status updates (every 2 seconds)
        if self.last_print_time is None or (current_time - self.last_print_time) >= 2.0:
            self.last_print_time = current_time
            if self.phase_start_time is not None:
                phase_elapsed = current_time - self.phase_start_time
                remaining = self._get_phase_duration() - phase_elapsed
                if remaining > 0:
                    print(f'[STATUS] Phase: {self.current_phase.name}, Elapsed: {phase_elapsed:.1f}s, Remaining: {remaining:.1f}s')

        # Check if current phase duration has elapsed
        if self.phase_start_time is not None:
            phase_elapsed = current_time - self.phase_start_time

            # Auto-advance phases based on duration
            if self.current_phase == FlightPhase.ARMED:
                if phase_elapsed >= self.arm_duration:
                    self.advance_to_next_phase()
            elif self.current_phase == FlightPhase.THROTTLE_UP:
                if phase_elapsed >= self.throttle_up_duration:
                    self.advance_to_next_phase()
            elif self.current_phase == FlightPhase.HOVER:
                if phase_elapsed >= self.hover_duration:
                    self.advance_to_next_phase()
            elif self.current_phase == FlightPhase.THROTTLE_DOWN:
                if phase_elapsed >= self.landing_duration:
                    self.advance_to_next_phase()

        # Execute current phase
        self.execute_current_phase()

    def _get_phase_duration(self):
        """Get the duration for the current phase"""
        if self.current_phase == FlightPhase.ARMED:
            return self.arm_duration
        elif self.current_phase == FlightPhase.THROTTLE_UP:
            return self.throttle_up_duration
        elif self.current_phase == FlightPhase.HOVER:
            return self.hover_duration
        elif self.current_phase == FlightPhase.THROTTLE_DOWN:
            return self.landing_duration
        else:
            return 0.0

    def execute_current_phase(self):
        """Execute RC commands for the current flight phase"""

        if self.current_phase == FlightPhase.ARMED:
            # Armed with THROTTLE LOW (required for arming), Alt Hold OFF
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_LOW,  # CRITICAL: Must be 1000 to arm!
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=False  # Keep Alt Hold off during arming
            )

        elif self.current_phase == FlightPhase.THROTTLE_UP:
            # Throttle up to rise in Angle mode (Alt Hold disabled)
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_THROTTLE_UP,
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=False  # Rise in Angle mode only
            )

        elif self.current_phase == FlightPhase.HOVER:
            # Switch to Alt Hold at 1500 to hover
            self.publish_rc_command(
                roll=self.RC_NEUTRAL,
                pitch=self.RC_NEUTRAL,
                throttle=self.RC_NEUTRAL,  # Neutral throttle with Alt Hold = hover
                yaw=self.RC_NEUTRAL,
                armed=True,
                alt_hold=True  # Enable Alt Hold at 1500 to maintain altitude
            )

        elif self.current_phase == FlightPhase.THROTTLE_DOWN:
            # Throttle down to 1480 to land
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
            FlightPhase.ARMED: f'ARMED - Ready to fly (waiting {self.arm_duration} sec)',
            FlightPhase.THROTTLE_UP: f'THROTTLE UP to {self.RC_THROTTLE_UP} (rising for {self.throttle_up_duration} sec)',
            FlightPhase.HOVER: f'HOVERING at neutral {self.RC_NEUTRAL} ({self.hover_duration} sec)',
            FlightPhase.THROTTLE_DOWN: f'THROTTLE DOWN to {self.RC_THROTTLE_DOWN} (landing for {self.landing_duration} sec)',
            FlightPhase.COMPLETE: 'TEST COMPLETE'
        }

        if new_phase in phase_names:
            print(f'\n[PHASE] {phase_names[new_phase]}')
            self.get_logger().info(f'[Phase] {phase_names[new_phase]}')

    def advance_to_next_phase(self):
        """Advance to the next phase in the sequence"""
        phase_sequence = [
            FlightPhase.ARMED,
            FlightPhase.THROTTLE_UP,
            FlightPhase.HOVER,
            FlightPhase.THROTTLE_DOWN,
            FlightPhase.COMPLETE
        ]

        current_index = phase_sequence.index(self.current_phase)
        if current_index < len(phase_sequence) - 1:
            next_phase = phase_sequence[current_index + 1]
            self.transition_to_phase(next_phase)

            # Stop publishing after test is complete
            if next_phase == FlightPhase.COMPLETE:
                print('\n' + '='*60)
                print('Simple flight test completed successfully!')
                print('All data has been logged by black_box_recorder_node')
                print('Check ~/swarm-ros/flight-logs/ for recorded data')
                print('='*60 + '\n')

                self.get_logger().info('='*60)
                self.get_logger().info('Simple flight test completed successfully!')
                self.get_logger().info('All data has been logged by black_box_recorder_node')
                self.get_logger().info('Check ~/swarm-ros/flight-logs/ for recorded data')
                self.get_logger().info('='*60)
                # Keep node running to maintain final disarmed state


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = SimpleFlightTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simple flight test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
