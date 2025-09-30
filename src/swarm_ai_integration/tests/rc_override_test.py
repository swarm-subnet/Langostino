#!/usr/bin/env python3
"""
Flight Controller Test Tool
Tool to diagnose and control FC via MSP
"""

import time
import argparse
import serial
from typing import Optional, Dict
from swarm_ai_integration.msp_protocol import MSPMessage, MSPCommand, MSPDirection, MSPDataTypes

class FlightController:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None

    def connect(self):
        """Connect to FC"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for initialization
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from FC"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    def send_message(self, command: int, data: bytes = b'') -> bool:
        """Send MSP message"""
        msg = MSPMessage(command, data, MSPDirection.REQUEST)
        try:
            self.serial.write(msg.encode())
            self.serial.flush()
            return True
        except Exception as e:
            print(f"❌ Error sending message: {e}")
            return False

    def receive_message(self, timeout: float = 0.5) -> Optional[MSPMessage]:
        """Receive MSP message"""
        start_time = time.time()
        buffer = b''

        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                buffer += self.serial.read(self.serial.in_waiting)

                # Find start of message
                if b'$M' in buffer:
                    start_idx = buffer.index(b'$M')
                    buffer = buffer[start_idx:]

                    # Check if we have a full message
                    if len(buffer) >= 6:
                        size = buffer[3]
                        expected_length = 6 + size

                        if len(buffer) >= expected_length:
                            msg = MSPMessage.decode(buffer[:expected_length])
                            return msg

            time.sleep(0.01)

        return None

    def send_and_receive(self, command: int, data: bytes = b'') -> Optional[Dict]:
        """Send command and wait for response"""
        if not self.send_message(command, data):
            return None

        response = self.receive_message()
        if response and response.direction == MSPDirection.RESPONSE:
            return self._parse_response(response)

        return None

    def _parse_response(self, msg: MSPMessage) -> Dict:
        """Parse MSP response according to command"""
        if msg.command == MSPCommand.MSP_STATUS:
            if len(msg.data) >= 11:
                import struct
                cycle_time, i2c_errors, sensor, flag, _ = struct.unpack('<HHIBB', msg.data[:11])
                return {
                    'cycle_time': cycle_time,
                    'i2c_errors': i2c_errors,
                    'sensor': sensor,
                    'flag': flag,
                    'armed': bool(flag & 1)
                }

        elif msg.command == MSPCommand.MSP_RC:
            channels = MSPDataTypes.unpack_rc_channels(msg.data)
            return {'channels': channels}

        elif msg.command == MSPCommand.MSP_RAW_GPS:
            return MSPDataTypes.unpack_gps_data(msg.data)

        return {'raw_data': msg.data}

    def get_status(self):
        """Get basic FC status"""
        response = self.send_and_receive(MSPCommand.MSP_STATUS)
        if response:
            return {
                'armed': response.get('armed', False),
                'cycle_time': response.get('cycle_time', 0),
                'i2c_errors': response.get('i2c_errors', 0),
                'sensor': response.get('sensor', 0),
                'flag': response.get('flag', 0),
            }
        return None

    def check_arming_conditions(self):
        """Check arming conditions"""
        status = self.get_status()
        if not status:
            return {}

        sensor = status['sensor']
        conditions = {
            'accelerometer': bool(sensor & 1),
            'gyroscope': bool(sensor & 2),
            'magnetometer': bool(sensor & 4),
            'barometer': bool(sensor & 8),
            'gps': bool(sensor & 16),
            'level': bool(sensor & 64),
        }
        return conditions

    def set_raw_rc(self, channels):
        """Send RC channels to FC"""
        payload = MSPDataTypes.pack_rc_channels(channels)
        return self.send_message(MSPCommand.MSP_SET_RAW_RC, payload)

    def arm_drone(self, force=False):
        """Arm the drone"""
        print("=== ATTEMPTING TO ARM THE DRONE ===")

        if not force:
            print("Checking arming conditions...")
            conditions = self.check_arming_conditions()
            failed = []
            for condition, passed in conditions.items():
                status_icon = "✅" if passed else "❌"
                status_text = "OK" if passed else "FAIL"
                print(f"  {status_icon} {condition.capitalize()}: {status_text}")
                if not passed:
                    failed.append(condition)

            if failed:
                print(f"\n❌ Cannot arm. Failed conditions: {', '.join(failed)}")
                print("Use --force to arm anyway (DANGEROUS!)")
                return False
        else:
            print("⚠️  FORCED MODE - Skipping safety checks")

        print("Sending arm command...")

        # Low throttle, AUX1 high to arm
        channels = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1000]

        for attempt in range(5):
            self.set_raw_rc(channels)
            time.sleep(0.1)

            status = self.get_status()
            if status and status['armed']:
                print("✅ Drone armed successfully")
                return True

        print("❌ Failed to arm the drone")
        return False

    def disarm_drone(self):
        """Disarm the drone"""
        print("=== DISARMING DRONE ===")

        # AUX1 low to disarm
        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]

        for attempt in range(5):
            self.set_raw_rc(channels)
            time.sleep(0.1)

            status = self.get_status()
            if status and not status['armed']:
                print("✅ Drone disarmed successfully")
                return True

        print("❌ Failed to disarm the drone")
        return False

    def emergency_disarm(self):
        """Emergency disarm"""
        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        for _ in range(10):
            self.set_raw_rc(channels)
            time.sleep(0.05)
        return True

    def check_msp_rc_support(self):
        """Check if MSP RC is working"""
        print("\n=== CHECKING MSP RC SUPPORT ===")

        # Send test channels and verify they are received
        print("\n📡 Sending test channels...")
        test_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]

        if self.set_raw_rc(test_channels):
            time.sleep(0.1)

            # Read current RC
            response = self.send_and_receive(MSPCommand.MSP_RC)
            if response and 'channels' in response:
                channels = response['channels']
                print(f"   Channels sent: {test_channels[:4]}")
                print(f"   Channels read: {channels[:4]}")

                # Check if they match (±10 margin)
                matches = all(abs(channels[i] - test_channels[i]) < 10 for i in range(min(4, len(channels))))
                if matches:
                    print("   ✅ MSP RC is working correctly")
                    return True
                else:
                    print("   ❌ Channels do not match - MSP RC is NOT active")
                    print("\n💡 SOLUTION:")
                    print("   1. Connect to the FC with Betaflight Configurator")
                    print("   2. Go to CLI and run:")
                    print("      set serialrx_provider = MSP")
                    print("      save")
                    return False

        print("❌ Could not verify MSP RC")
        return False

    def get_detailed_status(self):
        """Detailed status including RC and GPS"""
        status = self.get_status()
        if not status:
            return None

        # Read RC channels
        rc_response = self.send_and_receive(MSPCommand.MSP_RC)
        if rc_response and 'channels' in rc_response:
            status['rc_channels'] = rc_response['channels'][:8]

        # Read GPS
        gps_response = self.send_and_receive(MSPCommand.MSP_RAW_GPS)
        if gps_response:
            status['gps'] = {
                'fix': gps_response.get('fix_type', 0),
                'numSat': gps_response.get('satellites', 0),
                'lat': gps_response.get('latitude', 0),
                'lon': gps_response.get('longitude', 0),
            }

        return status

    def test_rc_channels(self, duration=5):
        """Test by sending different RC values"""
        print(f"\n=== RC CHANNELS TEST ({duration}s) ===")
        print("Sending varied values to each channel...")

        start_time = time.time()
        iteration = 0

        while time.time() - start_time < duration:
            # Vary roll between 1000-2000
            roll = 1000 + int(500 * (1 + abs((iteration % 100) - 50) / 50))
            channels = [roll, 1500, 1000, 1500, 1000, 1000, 1000, 1000]

            self.set_raw_rc(channels)

            if iteration % 10 == 0:
                # Read and display every 10 iterations
                response = self.send_and_receive(MSPCommand.MSP_RC)
                if response and 'channels' in response:
                    received = response['channels']
                    print(f"   Sent ROLL={roll} -> Received ROLL={received[0]}")

            iteration += 1
            time.sleep(0.05)

        print("✅ Test completed")


def main():
    parser = argparse.ArgumentParser(description='Flight Controller Test Tool')
    parser.add_argument('action', choices=['status', 'arm', 'disarm', 'emergency', 'check-msp', 'detailed', 'test-rc'],
                       help='Action to perform')
    parser.add_argument('--force', action='store_true',
                       help='Force arming without checking conditions')
    parser.add_argument('--port', default='/dev/ttyAMA0',
                       help='Serial port (default: /dev/ttyAMA0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')
    parser.add_argument('--duration', type=int, default=5,
                       help='Duration for test-rc in seconds (default: 5)')

    args = parser.parse_args()

    controller = FlightController(port=args.port, baudrate=args.baudrate)

    if not controller.connect():
        return 1

    try:
        if args.action == 'status':
            status = controller.get_status()
            if status:
                print("\n=== DRONE STATUS ===")
                armed_status = "🔴 ARMED" if status['armed'] else "🟢 DISARMED"
                print(f"Status: {armed_status}")
                print(f"Cycle time: {status['cycle_time']}μs")
                print(f"I2C errors: {status['i2c_errors']}")

                conditions = controller.check_arming_conditions()
                print("\nArming conditions:")
                for condition, passed in conditions.items():
                    status_icon = "✅" if passed else "❌"
                    print(f"  {status_icon} {condition.capitalize()}")
            else:
                print("❌ Could not get status")

        elif args.action == 'arm':
            success = controller.arm_drone(args.force)
            return 0 if success else 1

        elif args.action == 'disarm':
            success = controller.disarm_drone()
            return 0 if success else 1

        elif args.action == 'emergency':
            success = controller.emergency_disarm()
            if success:
                print("✅ Emergency disarm successful")
            else:
                print("❌ Emergency disarm failed")
            return 0 if success else 1

        elif args.action == 'check-msp':
            success = controller.check_msp_rc_support()
            return 0 if success else 1

        elif args.action == 'detailed':
            status = controller.get_detailed_status()
            if status:
                print("\n=== DETAILED STATUS ===")
                armed_status = "🔴 ARMED" if status['armed'] else "🟢 DISARMED"
                print(f"Status: {armed_status}")
                print(f"Cycle time: {status['cycle_time']}μs")
                print(f"I2C errors: {status['i2c_errors']}")

                if 'rc_channels' in status:
                    print(f"\n📡 Current RC channels:")
                    labels = ['ROLL', 'PITCH', 'THR', 'YAW', 'AUX1', 'AUX2', 'AUX3', 'AUX4']
                    for label, value in zip(labels, status['rc_channels']):
                        bar = '█' * int(value / 50)
                        print(f"  {label:6s}: {value:4d} {bar}")

                if 'gps' in status:
                    gps = status['gps']
                    fix_status = ['NO FIX', '2D FIX', '3D FIX'][min(gps['fix'], 2)]
                    print(f"\n🛰️  GPS:")
                    print(f"  Fix: {fix_status}")
                    print(f"  Satellites: {gps['numSat']}")
                    if gps['fix'] > 0:
                        print(f"  Position: {gps['lat']:.6f}, {gps['lon']:.6f}")

                conditions = controller.check_arming_conditions()
                print("\n🔒 Arming conditions:")
                for condition, passed in conditions.items():
                    status_icon = "✅" if passed else "❌"
                    print(f"  {status_icon} {condition.capitalize()}")
            else:
                print("❌ Could not get detailed status")

        elif args.action == 'test-rc':
            controller.test_rc_channels(duration=args.duration)

    except KeyboardInterrupt:
        print("\n🚨 Interrupt detected - Emergency disarm")
        controller.emergency_disarm()

    finally:
        controller.disconnect()

    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())
