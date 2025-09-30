#!/usr/bin/env python3
"""
Flight Controller Test Tool
Tool to diagnose and control FC via MSP

SAFETY:
- Test with props OFF first.
- When using `throttle` action, the script will ARM (AUX1 high),
  then stream RC with a throttle bump for the requested duration.
- Ensure your mode ranges make AUX1=HIGH arm the craft, and your failsafe is safe.
"""

import time
import argparse
import serial
from typing import Optional, Dict, List, Any

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ_DEFAULT = 40  # steady RC stream rate


class FlightController:
    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()

    # -------------------- Connection --------------------

    def connect(self) -> bool:
        """Connect to FC"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
            time.sleep(2)  # Wait for FC to boot its MSP UART
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

    # -------------------- MSP I/O --------------------

    def send_message(self, command: int, data: bytes = b'') -> bool:
        """Send one MSP message"""
        msg = MSPMessage(command, data, MSPDirection.REQUEST)
        try:
            self.serial.write(msg.encode())
            self.serial.flush()
            return True
        except Exception as e:
            print(f"❌ Error sending message: {e}")
            return False

    def _read_next_frame(self, timeout: float = 0.5) -> Optional[MSPMessage]:
        """Buffered frame reader that returns the next valid MSP frame (any command)."""
        end = time.time() + timeout
        while time.time() < end:
            # Pull any available bytes
            if self.serial.in_waiting:
                self.rx_buffer.extend(self.serial.read(self.serial.in_waiting))

            # Find header
            start = self.rx_buffer.find(b'$M')
            if start == -1:
                # No header yet; discard junk and wait a bit
                if len(self.rx_buffer) > 0:
                    self.rx_buffer.clear()
                time.sleep(0.005)
                continue

            # Need minimum 6 bytes ($M dir size cmd ... checksum)
            if len(self.rx_buffer) - start < 6:
                time.sleep(0.002)
                continue

            size = self.rx_buffer[start + 3]
            frame_len = 6 + size
            if len(self.rx_buffer) - start < frame_len:
                time.sleep(0.002)
                continue

            frame = bytes(self.rx_buffer[start:start + frame_len])
            # Drop consumed region (and any noise before it)
            del self.rx_buffer[:start + frame_len]

            msg = MSPMessage.decode(frame)
            if msg:
                return msg
            # If decode failed due to checksum, loop to try next bytes quickly
        return None

    def send_and_receive(
        self,
        command: int,
        data: bytes = b'',
        expect_command: Optional[int] = None,
        timeout: float = 0.5
    ) -> Optional[Dict[str, Any]]:
        """
        Send a command and wait until we see a RESPONSE whose command matches `expect_command`.
        If `expect_command` is None, it defaults to the same command (request/response pair).
        """
        if expect_command is None:
            expect_command = command

        # Flush stale input so we don't read an old ACK
        try:
            self.serial.reset_input_buffer()
        except Exception:
            pass

        if not self.send_message(command, data):
            return None

        end = time.time() + timeout
        while time.time() < end:
            msg = self._read_next_frame(timeout=end - time.time())
            if not msg:
                continue
            if msg.direction != MSPDirection.RESPONSE:
                continue
            if msg.command != expect_command:
                # Ignore unrelated frames (e.g., the empty ACK to MSP_SET_RAW_RC when we're expecting MSP_RC)
                continue
            return self._parse_response(msg)

        return None

    # -------------------- Response parsing --------------------

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

    # -------------------- High level helpers --------------------

    def get_status(self) -> Optional[Dict[str, int]]:
        """Get basic FC status"""
        response = self.send_and_receive(MSPCommand.MSP_STATUS, expect_command=MSPCommand.MSP_STATUS)
        if response:
            return {
                'armed': response.get('armed', False),
                'cycle_time': response.get('cycle_time', 0),
                'i2c_errors': response.get('i2c_errors', 0),
                'sensor': response.get('sensor', 0),
                'flag': response.get('flag', 0),
            }
        return None

    def check_arming_conditions(self) -> Dict[str, bool]:
        """Check arming-related sensor flags (coarse)."""
        status = self.get_status()
        if not status:
            return {}
        sensor = status['sensor']
        return {
            'accelerometer': bool(sensor & 1),
            'gyroscope': bool(sensor & 2),
            'magnetometer': bool(sensor & 4),
            'barometer': bool(sensor & 8),
            'gps': bool(sensor & 16),
            'level': bool(sensor & 64),
        }

    def set_raw_rc(self, channels: List[int]) -> bool:
        """Send RC channels to FC (one frame)."""
        payload = MSPDataTypes.pack_rc_channels(channels)
        return self.send_message(MSPCommand.MSP_SET_RAW_RC, payload)

    def stream_raw_rc(self, channels: List[int], seconds: float, hz: int = STREAM_HZ_DEFAULT):
        """Continuously stream RC frames for `seconds` at `hz` (FC expects steady updates)."""
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.set_raw_rc(channels)
            # It's fine to occasionally read responses (optional); we keep TX cadence tight.
            time.sleep(period)

    # -------------------- Arm / Disarm --------------------

    def arm_drone(self, force: bool = False) -> bool:
        """Arm the drone (AUX1 high). Assumes AUX1 high selects ARM in your Mode Ranges."""
        print("=== ATTEMPTING TO ARM THE DRONE ===")

        if not force:
            print("Checking arming conditions...")
            failed = [name for name, ok in self.check_arming_conditions().items() if not ok]
            for name, ok in self.check_arming_conditions().items():
                print(f"  {'✅' if ok else '❌'} {name.capitalize()}")
            if failed:
                print(f"\n❌ Cannot arm. Failed conditions: {', '.join(failed)}")
                print("Use --force to arm anyway (DANGEROUS!)")
                return False
        else:
            print("⚠️  FORCED MODE - Skipping safety checks")

        print("Sending arm command (AUX1 HIGH, throttle low)...")

        # MSP order: [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        channels = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1000]

        # Stream for a short window so the FC sees a stable switch state
        self.stream_raw_rc(channels, seconds=1.0, hz=STREAM_HZ_DEFAULT)

        # Verify armed
        status = self.get_status()
        if status and status['armed']:
            print("✅ Drone armed successfully")
            return True

        print("❌ Failed to arm the drone")
        return False

    def disarm_drone(self) -> bool:
        """Disarm (AUX1 low)"""
        print("=== DISARMING DRONE ===")
        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.stream_raw_rc(channels, seconds=0.8, hz=STREAM_HZ_DEFAULT)

        status = self.get_status()
        if status and not status['armed']:
            print("✅ Drone disarmed successfully")
            return True
        print("❌ Failed to disarm the drone")
        return False

    def emergency_disarm(self) -> bool:
        """Emergency disarm: AUX1 low + low throttle, streamed briefly."""
        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.stream_raw_rc(channels, seconds=0.5, hz=STREAM_HZ_DEFAULT)
        return True

    # -------------------- Diagnostics --------------------

    def check_msp_rc_support(self) -> bool:
        """Check that MSP RC override is applied by reading MSP_RC after sending."""
        print("\n=== CHECKING MSP RC SUPPORT ===")
        # Send known values
        sent = [1600, 1400, 1000, 1500, 1000, 1000, 1000, 1000]  # R=1600, P=1400, THR=1000, YAW=1500
        self.set_raw_rc(sent)
        time.sleep(0.06)

        # Explicitly wait for MSP_RC response (105)
        resp = self.send_and_receive(MSPCommand.MSP_RC, expect_command=MSPCommand.MSP_RC, timeout=0.5)
        if not resp or 'channels' not in resp:
            print("❌ Could not verify MSP RC (no MSP_RC response)")
            return False

        got = resp['channels']
        print(f"   Channels sent (first 4): {sent[:4]}")
        print(f"   Channels read (first 4): {got[:4]}")

        # Allow small deadband
        margin = 12
        match = all(abs(got[i] - sent[i]) <= margin for i in range(4))
        if match:
            print("   ✅ MSP RC override appears active")
            return True

        print("   ❌ Channels differ; check:")
        print("     - `set msp_override_channels = 31` (you did this) and `save`")
        print("     - Mode Ranges (AUX1 for ARM if you arm via code)")
        print("     - That you are streaming frames when trying to control")
        return False

    def get_detailed_status(self) -> Optional[Dict[str, Any]]:
        """Detailed status including RC and GPS"""
        status = self.get_status()
        if not status:
            return None

        rc = self.send_and_receive(MSPCommand.MSP_RC, expect_command=MSPCommand.MSP_RC)
        if rc and 'channels' in rc:
            status['rc_channels'] = rc['channels'][:8]

        gps = self.send_and_receive(MSPCommand.MSP_RAW_GPS, expect_command=MSPCommand.MSP_RAW_GPS)
        if gps:
            status['gps'] = {
                'fix': gps.get('fix_type', 0),
                'numSat': gps.get('satellites', 0),
                'lat': gps.get('latitude', 0),
                'lon': gps.get('longitude', 0),
            }
        return status

    def test_rc_channels(self, duration: float = 5.0):
        """Sweep ROLL while streaming, printing the received value periodically."""
        print(f"\n=== RC CHANNELS TEST ({duration}s) ===")
        start = time.time()
        iteration = 0
        while time.time() - start < duration:
            roll = 1000 + int(1000 * ((iteration % 100) / 100.0))  # 1000..2000
            channels = [roll, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
            self.set_raw_rc(channels)

            if iteration % 10 == 0:
                resp = self.send_and_receive(MSPCommand.MSP_RC, expect_command=MSPCommand.MSP_RC)
                if resp and 'channels' in resp:
                    got = resp['channels']
                    print(f"   Sent ROLL={roll} -> Received ROLL={got[0]}")
            iteration += 1
            time.sleep(0.05)
        print("✅ Test completed")

    # -------------------- Movement via throttle --------------------

    def bump_throttle(self, throttle: int = 1600, seconds: float = 3.0, hz: int = STREAM_HZ_DEFAULT, auto_arm: bool = True) -> bool:
        """
        ARM (if requested) and then stream RC with a throttle bump.
        MSP order is [ROLL, PITCH, THROTTLE, YAW, AUX1...], so throttle is index 2.
        """
        print(f"\n=== THROTTLE BUMP: {throttle} for {seconds}s @ {hz}Hz ===")

        if auto_arm:
            if not self.arm_drone(force=False):
                print("⚠️  Trying forced arm...")
                if not self.arm_drone(force=True):
                    print("❌ Could not arm; aborting throttle bump.")
                    return False

        # Compose steady frame with desired throttle and AUX1 high to stay armed
        channels = [1500, 1500, throttle, 1500, 1800, 1000, 1000, 1000]

        print("Streaming RC frames with throttle bump...")
        self.stream_raw_rc(channels, seconds=seconds, hz=hz)

        # After bump, drop throttle back to low but keep AUX1 high briefly
        print("Dropping throttle to idle...")
        idle = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1000]
        self.stream_raw_rc(idle, seconds=0.6, hz=hz)

        # Optional disarm for safety
        self.disarm_drone()
        return True


def main():
    parser = argparse.ArgumentParser(description='Flight Controller Test Tool')
    parser.add_argument('action',
                        choices=['status', 'arm', 'disarm', 'emergency', 'check-msp', 'detailed', 'test-rc', 'throttle'],
                        help='Action to perform')
    parser.add_argument('--force', action='store_true', help='Force arming without checking conditions')
    parser.add_argument('--port', default='/dev/ttyAMA0', help='Serial port (default: /dev/ttyAMA0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate (default: 115200)')
    parser.add_argument('--duration', type=float, default=5.0, help='Duration for test-rc in seconds (default: 5)')
    # Throttle bump parameters
    parser.add_argument('--throttle', type=int, default=1600, help='Throttle value (1000-2000) for throttle action')
    parser.add_argument('--seconds', type=float, default=3.0, help='Seconds to hold throttle value (default: 3)')
    parser.add_argument('--hz', type=int, default=STREAM_HZ_DEFAULT, help='Streaming rate for RC frames')

    args = parser.parse_args()
    fc = FlightController(port=args.port, baudrate=args.baudrate)

    if not fc.connect():
        return 1

    try:
        if args.action == 'status':
            status = fc.get_status()
            if status:
                print("\n=== DRONE STATUS ===")
                print(f"Status: {'🔴 ARMED' if status['armed'] else '🟢 DISARMED'}")
                print(f"Cycle time: {status['cycle_time']}μs")
                print(f"I2C errors: {status['i2c_errors']}")
                print("\nArming conditions:")
                for condition, ok in fc.check_arming_conditions().items():
                    print(f"  {'✅' if ok else '❌'} {condition.capitalize()}")
            else:
                print("❌ Could not get status")

        elif args.action == 'arm':
            return 0 if fc.arm_drone(args.force) else 1

        elif args.action == 'disarm':
            return 0 if fc.disarm_drone() else 1

        elif args.action == 'emergency':
            ok = fc.emergency_disarm()
            print("✅ Emergency disarm successful" if ok else "❌ Emergency disarm failed")
            return 0 if ok else 1

        elif args.action == 'check-msp':
            return 0 if fc.check_msp_rc_support() else 1

        elif args.action == 'detailed':
            status = fc.get_detailed_status()
            if status:
                print("\n=== DETAILED STATUS ===")
                print(f"Status: {'🔴 ARMED' if status['armed'] else '🟢 DISARMED'}")
                print(f"Cycle time: {status['cycle_time']}μs")
                print(f"I2C errors: {status['i2c_errors']}")
                if 'rc_channels' in status:
                    print("\n📡 Current RC channels (first 8):")
                    labels = ['ROLL', 'PITCH', 'THR', 'YAW', 'AUX1', 'AUX2', 'AUX3', 'AUX4']
                    for label, value in zip(labels, status['rc_channels']):
                        bar = '█' * int(value / 50)
                        print(f"  {label:6s}: {value:4d} {bar}")
                if 'gps' in status:
                    gps = status['gps']
                    fix_status = ['NO FIX', '2D FIX', '3D FIX'][min(gps['fix'], 2)]
                    print("\n🛰️  GPS:")
                    print(f"  Fix: {fix_status}")
                    print(f"  Satellites: {gps['numSat']}")
                    if gps['fix'] > 0:
                        print(f"  Position: {gps['lat']:.6f}, {gps['lon']:.6f}")
            else:
                print("❌ Could not get detailed status")

        elif args.action == 'test-rc':
            fc.test_rc_channels(duration=args.duration)

        elif args.action == 'throttle':
            # This will ARM, bump throttle, then DISARM (unless arming fails)
            ok = fc.bump_throttle(throttle=args.throttle, seconds=args.seconds, hz=args.hz, auto_arm=True)
            return 0 if ok else 1

    except KeyboardInterrupt:
        print("\n🚨 Interrupt detected - Emergency disarm")
        fc.emergency_disarm()

    finally:
        fc.disconnect()

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
