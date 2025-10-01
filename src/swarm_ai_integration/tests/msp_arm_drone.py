#!/usr/bin/env python3
"""
Arm-Only Tool for INAV via MSP

What it does:
- Connects to the FC over MSP
- Checks that MSP RC override is taking effect (so the FC will accept our RC frames)
- Arms by setting AUX1 (CH5) HIGH while keeping throttle LOW
- Verifies the armed state and exits

IMPORTANT (matches your INAV configurator info):
- Channel map is AETR: [ROLL, PITCH, THROTTLE, YAW] = CH1..CH4
- ARM switch is CH5 (AUX1) -> arm when >1700
- MSP RC OVERRIDE is CH8 (AUX4) -> must be >1700 for MSP control to work
  You must flip your radio's CH8 override switch HIGH before running this.
- CH6 around 1500 keeps ANGLE mode ON (fine for arming)

Safety:
- Test with props OFF first.
"""

import time
import argparse
import serial
from typing import Optional, Dict, Any, List

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ = 40  # steady RC stream cadence expected by FC


class FlightController:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()

    # -------------------- Connection --------------------

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
            time.sleep(2.0)  # give MSP UART time to settle
            print(f"Connected to {self.port} @ {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    # -------------------- MSP I/O --------------------

    def send_message(self, command: int, data: bytes = b'') -> bool:
        msg = MSPMessage(command, data, MSPDirection.REQUEST)
        try:
            self.serial.write(msg.encode())
            self.serial.flush()
            return True
        except Exception as e:
            print(f"❌ Error sending MSP message: {e}")
            return False

    def _read_next_frame(self, timeout: float = 0.5) -> Optional[MSPMessage]:
        end = time.time() + timeout
        while time.time() < end:
            if self.serial.in_waiting:
                self.rx_buffer.extend(self.serial.read(self.serial.in_waiting))

            start = self.rx_buffer.find(b"$M")
            if start == -1:
                if self.rx_buffer:
                    self.rx_buffer.clear()
                time.sleep(0.003)
                continue

            # minimum: $ M dir size cmd ... checksum  (6 bytes + payload)
            if len(self.rx_buffer) - start < 6:
                time.sleep(0.002)
                continue

            size = self.rx_buffer[start + 3]
            frame_len = 6 + size
            if len(self.rx_buffer) - start < frame_len:
                time.sleep(0.002)
                continue

            frame = bytes(self.rx_buffer[start:start + frame_len])
            del self.rx_buffer[:start + frame_len]

            msg = MSPMessage.decode(frame)
            if msg:
                return msg
        return None

    def send_and_receive(
        self,
        command: int,
        data: bytes = b'',
        expect_command: Optional[int] = None,
        timeout: float = 0.5
    ) -> Optional[Dict[str, Any]]:
        if expect_command is None:
            expect_command = command

        # drop any stale input
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
                continue
            return self._parse_response(msg)
        return None

    # -------------------- Response parsing --------------------

    def _parse_response(self, msg: MSPMessage) -> Dict[str, Any]:
        if msg.command == MSPCommand.MSP_STATUS:
            if len(msg.data) >= 11:
                import struct
                cycle_time, i2c_errors, sensor, flag, _ = struct.unpack('<HHIBB', msg.data[:11])
                return {
                    "cycle_time": cycle_time,
                    "i2c_errors": i2c_errors,
                    "sensor": sensor,
                    "flag": flag,
                    "armed": bool(flag & 1),
                }

        elif msg.command == MSPCommand.MSP_RC:
            return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}

        return {"raw_data": msg.data}

    # -------------------- Helpers --------------------

    def get_status(self) -> Optional[Dict[str, Any]]:
        resp = self.send_and_receive(MSPCommand.MSP_STATUS, expect_command=MSPCommand.MSP_STATUS)
        if not resp:
            return None
        return {
            "armed": resp.get("armed", False),
            "cycle_time": resp.get("cycle_time", 0),
            "i2c_errors": resp.get("i2c_errors", 0),
            "sensor": resp.get("sensor", 0),
            "flag": resp.get("flag", 0),
        }

    def set_raw_rc(self, channels: List[int]) -> bool:
        payload = MSPDataTypes.pack_rc_channels(channels)
        return self.send_message(MSPCommand.MSP_SET_RAW_RC, payload)

    def stream_raw_rc(self, channels: List[int], seconds: float, hz: int = STREAM_HZ):
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.set_raw_rc(channels)
            time.sleep(period)

    # -------------------- Checks --------------------

    def verify_msp_rc_effective(self, margin: int = 12) -> bool:
        """
        Confirm that MSP RC frames actually affect the FC (requires CH8 override ON via radio).
        Sends a known set and reads back MSP_RC to compare first four channels (AETR).
        """
        print("\n=== Checking MSP RC override ===")
        test = [1600, 1400, 1000, 1500, 1000, 1500, 1500, 1500]  # R=1600, P=1400, THR=1000, YAW=1500
        self.set_raw_rc(test)
        time.sleep(0.06)
        rc = self.send_and_receive(MSPCommand.MSP_RC, expect_command=MSPCommand.MSP_RC, timeout=0.5)
        if not rc or "channels" not in rc:
            print("❌ No MSP_RC response. Cannot verify RC override.")
            return False
        got = rc["channels"]
        print(f"   Sent (first 4): {test[:4]}")
        print(f"   Read (first 4): {got[:4]}")
        ok = all(abs(got[i] - test[i]) <= margin for i in range(4))
        if ok:
            print("   ✅ MSP RC override appears active")
        else:
            print("   ❌ RC values not following MSP input")
        return ok

    # -------------------- Arm only --------------------

    def arm_only(self, hold_seconds: float = 1.0, hz: int = STREAM_HZ) -> bool:
        """
        Arms by setting AUX1 (CH5) HIGH with throttle LOW.
        Assumes CH8 (MSP RC OVERRIDE) is already HIGH on your radio so MSP RC frames take effect.
        """
        print("\n=== ARMING (AUX1 HIGH, throttle LOW) ===")
        # Keep sticks centered, throttle LOW, ARM switch HIGH (AUX1), others neutral.
        # AETR order -> [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        channels_arm = [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500]
        self.stream_raw_rc(channels_arm, seconds=hold_seconds, hz=hz)

        status = self.get_status()
        if status and status.get("armed"):
            print("✅ Drone is ARMED")
            return True
        print("❌ FC did not report ARMED")
        return False


def main():
    parser = argparse.ArgumentParser(description="Arm-Only Tool (INAV MSP)")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--skip-check", action="store_true",
                        help="Skip MSP RC override verification (NOT recommended)")
    parser.add_argument("--hold", type=float, default=1.0,
                        help="Seconds to hold AUX1 HIGH while arming (default: 1.0)")
    parser.add_argument("--hz", type=int, default=STREAM_HZ, help=f"RC stream rate (default: {STREAM_HZ}Hz)")
    args = parser.parse_args()

    print("SAFETY: Test with props OFF. Ensure CH8 (MSP RC OVERRIDE) is HIGH on your radio.\n")

    fc = FlightController(port=args.port, baudrate=args.baudrate)
    if not fc.connect():
        return 1

    try:
        if not args.skip_check:
            if not fc.verify_msp_rc_effective():
                print("\n➡️  Make sure your radio sets **CH8 > 1700** (MSP RC OVERRIDE ON),")
                print("    then run this again. Aborting to be safe.")
                return 1

        ok = fc.arm_only(hold_seconds=args.hold, hz=args.hz)
        return 0 if ok else 1

    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    finally:
        fc.disconnect()


if __name__ == "__main__":
    import sys
    sys.exit(main())
