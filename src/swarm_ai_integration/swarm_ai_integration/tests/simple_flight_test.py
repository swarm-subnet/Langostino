#!/usr/bin/env python3
"""
Simple Flight Test Node - Direct MSP Control

This script executes a simple flight test sequence using direct MSP communication:
1. ARM for 3 seconds (throttle low, Angle mode, Pos Hold ON)
2. RISE for 0.5 seconds (throttle 1600, Angle mode, Pos Hold ON)
3. HOVER for 2 seconds (throttle 1500, Angle mode, Pos Hold ON at 1800)
4. PITCH FORWARD for 15 seconds (throttle 1500, Pitch 1800, Angle mode, Pos Hold ON)
5. HOVER for 2 seconds (throttle 1500, Angle mode, Pos Hold ON at 1800)
6. LAND for 5 seconds (throttle 1400, Angle mode, Pos Hold ON)
7. DISARM

RC Channel Mapping (AETR + AUX):
- CH1 (index 0): Roll (1500 = neutral)
- CH2 (index 1): Pitch (1500 = neutral)
- CH3 (index 2): Throttle (1000 = low, 1500 = hover, 1300 = rise/land)
- CH4 (index 3): Yaw (1500 = neutral)
- CH5 (index 4): Arm/Disarm (1800 = armed, 1000 = disarmed)
- CH6 (index 5): Angle Mode (1500 = enabled)
- CH7 (index 6): Pos Hold (1800 = enabled, 1000 = disabled)
- CH8 (index 7): MSP Override (1800 = enabled)

"""

import time
import serial
from typing import List, Optional

import rclpy
from rclpy.node import Node

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)


class SimpleFlightTestNode(Node):
    """Simple flight test node with direct MSP control"""

    def __init__(self):
        super().__init__('simple_flight_test_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('stream_hz', 40)

        # Phase durations (seconds)
        self.declare_parameter('arm_duration', 3.0)
        self.declare_parameter('rise_duration', 0.5)
        self.declare_parameter('hover_duration', 2.0)
        self.declare_parameter('pitch_forward', 15.0)
        self.declare_parameter('land_duration', 5.0)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.stream_hz = self.get_parameter('stream_hz').value

        self.arm_duration = self.get_parameter('arm_duration').value
        self.rise_duration = self.get_parameter('rise_duration').value
        self.hover_duration = self.get_parameter('hover_duration').value
        self.pitch_forward = self.get_parameter('pitch_forward').value
        self.land_duration = self.get_parameter('land_duration').value

        # RC channel constants
        self.RC_NEUTRAL = 1500
        self.RC_THROTTLE_ARM = 900
        self.RC_THROTTLE_RISE = 1600
        self.RC_THROTTLE_HOVER = 1500
        self.RC_THROTTLE_LAND = 1400
        self.RC_PITCH_FORWARD = 1800
        self.RC_ARM = 1800
        self.RC_DISARM = 1000
        self.RC_ANGLE_MODE = 1500      # CH6 - Angle mode enabled
        self.RC_POS_HOLD_OFF = 1000    # CH7 - Pos Hold disabled
        self.RC_POS_HOLD_ON = 1500    # CH7 - Pos Hold enabled (=1500)
        self.RC_MSP_OVERRIDE = 2000  # CH8 - MSP Override enabled

        # Serial connection
        self.ser: Optional[serial.Serial] = None

        self.get_logger().info('═' * 60)
        self.get_logger().info('Simple Flight Test Node Initialized')
        self.get_logger().info('═' * 60)
        self.get_logger().info(f'Serial Port: {self.serial_port}')
        self.get_logger().info(f'Baud Rate: {self.baud_rate}')
        self.get_logger().info(f'Stream Rate: {self.stream_hz} Hz')
        self.get_logger().info('')
        self.get_logger().info('Test Sequence:')
        self.get_logger().info(f'  1. ARM ({self.arm_duration}s) - Angle mode, Pos Hold OFF')
        self.get_logger().info(f'  2. RISE ({self.rise_duration}s) - Throttle {self.RC_THROTTLE_RISE}, Angle mode, Pos Hold ON')
        self.get_logger().info(f'  3. HOVER ({self.hover_duration}s) - Pos Hold ON at 1800')
        self.get_logger().info(f'  4. PITCH FORWARD ({self.pitch_forward}s) - Pitch {self.RC_PITCH_FORWARD}')
        self.get_logger().info(f'  5. HOVER ({self.hover_duration}s) - Pos Hold ON at 1800')
        self.get_logger().info(f'  6. LAND ({self.land_duration}s) - Throttle {self.RC_THROTTLE_LAND}')
        self.get_logger().info(f'  7. DISARM')
        self.get_logger().info('═' * 60)

    def connect(self) -> bool:
        """Connect to flight controller"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.02)
            time.sleep(2.0)  # Allow FC to boot
            self.get_logger().info(f'✓ Connected to {self.serial_port} @ {self.baud_rate}')
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Serial connection failed: {e}')
            return False

    def disconnect(self):
        """Disconnect from flight controller"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('✓ Disconnected from flight controller')

    def send_rc(self, channels: List[int]):
        """Send RC channels via MSP_SET_RAW_RC"""
        if not self.ser or not self.ser.is_open:
            return

        payload = MSPDataTypes.pack_rc_channels(channels)
        msg = MSPMessage(MSPCommand.MSP_SET_RAW_RC, payload, MSPDirection.REQUEST)

        try:
            self.ser.write(msg.encode())
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'✗ Send error: {e}')

    def stream_for(self, channels: List[int], duration: float, phase_name: str):
        """Stream RC frame at configured Hz for specified duration"""
        period = 1.0 / float(self.stream_hz)
        t_end = time.time() + duration

        self.get_logger().info(f'▶ {phase_name} - {duration}s')
        self.get_logger().info(f'  Channels: {channels}')

        while time.time() < t_end:
            t0 = time.time()
            self.send_rc(channels)

            dt = time.time() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)

        self.get_logger().info(f'✓ {phase_name} complete')

    def run_flight_test(self):
        """Execute the complete flight test sequence"""
        if not self.connect():
            return False

        try:
            # PHASE 1: ARM
            arm_frame = [
                self.RC_NEUTRAL,  # CH1 - Roll
                self.RC_NEUTRAL,  # CH2 - Pitch
                self.RC_THROTTLE_ARM,  # CH3 - Throttle
                self.RC_NEUTRAL,  # CH4 - Yaw
                self.RC_ARM,  # CH5 - Arm
                self.RC_ANGLE_MODE,  # CH6 - Angle Mode
                self.RC_POS_HOLD_OFF,  # CH7 - NAV modes off
                self.RC_MSP_OVERRIDE,  # CH8 - MSP Override
                self.RC_NEUTRAL,  # CH9
                self.RC_NEUTRAL,  # CH10
                self.RC_NEUTRAL,  # CH11
                self.RC_NEUTRAL,  # CH12
                self.RC_NEUTRAL,  # CH13
                self.RC_NEUTRAL,  # CH14
                self.RC_NEUTRAL,  # CH15
                self.RC_NEUTRAL   # CH16
            ]
            self.stream_for(arm_frame, self.arm_duration, 'PHASE 1: ARM')

            # PHASE 2: RISE
            rise_frame = [
                self.RC_NEUTRAL,  # CH1 - Roll
                self.RC_NEUTRAL,  # CH2 - Pitch
                self.RC_THROTTLE_RISE,  # CH3 - Throttle
                self.RC_NEUTRAL,  # CH4 - Yaw
                self.RC_ARM,  # CH5 - Arm
                self.RC_ANGLE_MODE,  # CH6 - Angle Mode
                self.RC_POS_HOLD_ON,  # CH7 - NAV POSHOLD
                self.RC_MSP_OVERRIDE,  # CH8 - MSP Override
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL
            ]
            self.stream_for(rise_frame, self.rise_duration, 'PHASE 2: RISE')

            # PHASE 3: HOVER (2 seconds)
            hover_frame = [
                self.RC_NEUTRAL,  # CH1 - Roll
                self.RC_NEUTRAL,  # CH2 - Pitch
                self.RC_THROTTLE_HOVER,  # CH3 - Throttle
                self.RC_NEUTRAL,  # CH4 - Yaw
                self.RC_ARM,  # CH5 - Arm
                self.RC_ANGLE_MODE,  # CH6 - Angle Mode
                self.RC_POS_HOLD_ON,  # CH7 - NAV POSHOLD
                self.RC_MSP_OVERRIDE,  # CH8 - MSP Override
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL
            ]
            self.stream_for(hover_frame, self.hover_duration, 'PHASE 3: HOVER (2s)')

            # PHASE 4: PITCH FORWARD (3 seconds)
            nudge_frame = [
                self.RC_NEUTRAL,  # CH1 - Roll
                self.RC_PITCH_FORWARD,  # CH2 - Pitch
                self.RC_THROTTLE_HOVER,  # CH3 - Throttle
                self.RC_NEUTRAL,  # CH4 - Yaw
                self.RC_ARM,  # CH5 - Arm
                self.RC_ANGLE_MODE,  # CH6 - Angle Mode
                self.RC_POS_HOLD_ON,  # CH7 - NAV POSHOLD
                self.RC_MSP_OVERRIDE,  # CH8 - MSP Override
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL
            ]
            self.stream_for(nudge_frame, self.pitch_forward, 'PHASE 4: PITCH FORWARD (3s)')

            # PHASE 5: HOVER (2 seconds)
            self.stream_for(hover_frame, self.hover_duration, 'PHASE 5: HOVER (2s)')

            # PHASE 6: LAND (Throttle 1450)
            land_frame = [
                self.RC_NEUTRAL,  # CH1 - Roll
                self.RC_NEUTRAL,  # CH2 - Pitch
                self.RC_THROTTLE_LAND,  # CH3 - Throttle
                self.RC_NEUTRAL,  # CH4 - Yaw
                self.RC_ARM,  # CH5 - Disarm
                self.RC_ANGLE_MODE,  # CH6 - Angle Mode
                self.RC_POS_HOLD_ON,  # CH7 - NAV POSHOLD
                self.RC_MSP_OVERRIDE,  # CH8 - MSP Override
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL
            ]
            self.stream_for(land_frame, self.land_duration, 'PHASE 6: LAND')

            # PHASE 7: DISARM
            disarm_frame = [
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_THROTTLE_ARM,
                self.RC_NEUTRAL,
                self.RC_DISARM,         # CH5 - Disarm
                self.RC_ANGLE_MODE,
                self.RC_POS_HOLD_OFF,
                self.RC_MSP_OVERRIDE,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL
            ]
            self.stream_for(disarm_frame, 1.0, 'PHASE 7: DISARM')

            self.get_logger().info('═' * 60)
            self.get_logger().info('✓ Flight test completed successfully!')
            self.get_logger().info('═' * 60)
            return True

        except KeyboardInterrupt:
            self.get_logger().warn('✗ Flight test interrupted by user')
            return False
        except Exception as e:
            self.get_logger().error(f'✗ Flight test failed: {e}')
            return False
        finally:
            self.disconnect()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = SimpleFlightTestNode()

    try:
        # Run the flight test
        node.run_flight_test()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
