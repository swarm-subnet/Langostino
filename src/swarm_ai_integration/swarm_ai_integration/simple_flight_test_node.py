#!/usr/bin/env python3
"""
Simple Flight Test Node - Direct MSP Control

This script executes a simple flight test sequence using direct MSP communication:
1. ARM for 3 seconds (throttle low, Angle mode, Alt Hold OFF)
2. RISE for 2 seconds (throttle 1300, Angle mode, Alt Hold OFF)
3. HOVER for 5 seconds (throttle 1500, Angle mode, Alt Hold ON at 1800)
4. LAND for 3 seconds (throttle 1300, Angle mode, Alt Hold OFF)
5. DISARM

RC Channel Mapping (AETR + AUX):
- CH1 (index 0): Roll (1500 = neutral)
- CH2 (index 1): Pitch (1500 = neutral)
- CH3 (index 2): Throttle (1000 = low, 1500 = hover, 1300 = rise/land)
- CH4 (index 3): Yaw (1500 = neutral)
- CH5 (index 4): Arm/Disarm (1800 = armed, 1000 = disarmed)
- CH6 (index 5): Angle Mode (1500 = enabled)
- CH7 (index 6): Alt Hold (1800 = enabled, 1000 = disabled)
- CH8 (index 7): MSP Override (1800 = enabled)

Safety: TEST WITH PROPS OFF FIRST
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
        self.declare_parameter('rise_duration', 2.0)
        self.declare_parameter('hover_duration', 5.0)
        self.declare_parameter('land_duration', 10.0)

        # RC values
        self.declare_parameter('throttle_rise', 1300)
        self.declare_parameter('throttle_hover', 1500)
        self.declare_parameter('throttle_land', 1300)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.stream_hz = self.get_parameter('stream_hz').value

        self.arm_duration = self.get_parameter('arm_duration').value
        self.rise_duration = self.get_parameter('rise_duration').value
        self.hover_duration = self.get_parameter('hover_duration').value
        self.land_duration = self.get_parameter('land_duration').value

        self.throttle_rise = self.get_parameter('throttle_rise').value
        self.throttle_hover = self.get_parameter('throttle_hover').value
        self.throttle_land = self.get_parameter('throttle_land').value

        # RC channel constants
        self.RC_NEUTRAL = 1500
        self.RC_THROTTLE_LOW = 1000
        self.RC_ARM = 1800
        self.RC_DISARM = 1000
        self.RC_ANGLE_MODE = 1500      # CH6 - Angle mode enabled
        self.RC_ALT_HOLD_ON = 1800     # CH7 - Alt Hold enabled (>1700)
        self.RC_ALT_HOLD_OFF = 1000    # CH7 - Alt Hold disabled
        self.RC_MSP_OVERRIDE = 1800

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
        self.get_logger().info(f'  1. ARM ({self.arm_duration}s) - Angle mode, Alt Hold OFF')
        self.get_logger().info(f'  2. RISE ({self.rise_duration}s) - Throttle {self.throttle_rise}, Angle mode')
        self.get_logger().info(f'  3. HOVER ({self.hover_duration}s) - Alt Hold ON at 1800')
        self.get_logger().info(f'  4. LAND ({self.land_duration}s) - Throttle {self.throttle_land}')
        self.get_logger().info(f'  5. DISARM')
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
            # AETR + AUX (16 channels total - INAV standard)
            arm_frame = [
                self.RC_NEUTRAL,        # CH1 - Roll
                self.RC_NEUTRAL,        # CH2 - Pitch
                self.RC_THROTTLE_LOW,   # CH3 - Throttle (MUST be low to arm)
                self.RC_NEUTRAL,        # CH4 - Yaw
                self.RC_ARM,            # CH5 - Arm
                self.RC_ANGLE_MODE,     # CH6 - Angle Mode ON
                self.RC_ALT_HOLD_OFF,   # CH7 - Alt Hold OFF
                self.RC_MSP_OVERRIDE,   # CH8 - MSP Override
                self.RC_NEUTRAL,        # CH9
                self.RC_NEUTRAL,        # CH10
                self.RC_NEUTRAL,        # CH11
                self.RC_NEUTRAL,        # CH12
                self.RC_NEUTRAL,        # CH13
                self.RC_NEUTRAL,        # CH14
                self.RC_NEUTRAL,        # CH15
                self.RC_NEUTRAL         # CH16
            ]
            self.stream_for(arm_frame, self.arm_duration, 'PHASE 1: ARM')

            # PHASE 2: RISE (Angle mode, Alt Hold OFF)
            rise_frame = [
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.throttle_rise,     # CH3 - Throttle up to rise
                self.RC_NEUTRAL,
                self.RC_ARM,
                self.RC_ANGLE_MODE,     # CH6 - Angle Mode ON
                self.RC_ALT_HOLD_OFF,   # CH7 - Alt Hold OFF
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
            self.stream_for(rise_frame, self.rise_duration, 'PHASE 2: RISE')

            # PHASE 3: HOVER (Alt Hold ON)
            hover_frame = [
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.throttle_hover,    # CH3 - Neutral throttle
                self.RC_NEUTRAL,
                self.RC_ARM,
                self.RC_ANGLE_MODE,     # CH6 - Angle Mode ON
                self.RC_ALT_HOLD_ON,    # CH7 - Alt Hold ON at 1800
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
            self.stream_for(hover_frame, self.hover_duration, 'PHASE 3: HOVER')

            # PHASE 4: LAND (Alt Hold OFF, gentle descent)
            land_frame = [
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.throttle_land,     # CH3 - Throttle down
                self.RC_NEUTRAL,
                self.RC_ARM,
                self.RC_ANGLE_MODE,     # CH6 - Angle Mode ON
                self.RC_ALT_HOLD_ON,   # CH7 - Alt Hold OFF
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
            self.stream_for(land_frame, self.land_duration, 'PHASE 4: LAND')

            # PHASE 5: DISARM
            disarm_frame = [
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_NEUTRAL,
                self.RC_DISARM,         # CH5 - Disarm
                self.RC_ANGLE_MODE,
                self.RC_ALT_HOLD_OFF,
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
            self.stream_for(disarm_frame, 1.0, 'PHASE 5: DISARM')

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
