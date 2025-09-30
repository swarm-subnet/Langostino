#!/usr/bin/env python3
"""
msp_override_verify.py
Verifies MSP RC override by sending MSP_SET_RAW_RC and reading MSP_RC.

Usage:
  python3 msp_override_verify.py --port /dev/ttyAMA0 --baudrate 115200
"""

import time
import argparse
import serial
from typing import Optional, Dict, Any

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

class MSPClient:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.rx = bytearray()

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            time.sleep(2.0)  # give FC time to boot MSP UART
            print(f"Connected to {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"❌ Serial open failed: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")

    def send(self, cmd: int, data: bytes = b'') -> bool:
        try:
            msg = MSPMessage(cmd, data, MSPDirection.REQUEST)
            self.ser.write(msg.encode())
            self.ser.flush()
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            return False

    def _read_frame(self, timeout: float) -> Optional[MSPMessage]:
        end = time.time() + timeout
        while time.time() < end:
            if self.ser.in_waiting:
                self.rx.extend(self.ser.read(self.ser.in_waiting))

            start = self.rx.find(b"$M")
            if start == -1:
                if self.rx:
                    self.rx.clear()
                time.sleep(0.003)
                continue

            if len(self.rx) - start < 6:
                time.sleep(0.002)
                continue

            size = self.rx[start + 3]
            frame_len = 6 + size
            if len(self.rx) - start < frame_len:
                time.sleep(0.002)
                continue

            frame = bytes(self.rx[start:start + frame_len])
            del self.rx[:start + frame_len]

            msg = MSPMessage.decode(frame)
            if msg:
                return msg
        return None

    def req(self, cmd: int, expect: Optional[int] = None, data: bytes = b'', timeout: float = 0.6) -> Optional[Dict[str, Any]]:
        if expect is None:
            expect = cmd
        # Drop stale input before this transaction
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        if not self.send(cmd, data):
            return None

        end = time.time() + timeout
        while time.time() < end:
            msg = self._read_frame(timeout=end - time.time())
            if not msg:
                continue
            if msg.direction != MSPDirection.RESPONSE:
                continue
            if msg.command != expect:
                # ignore unrelated frames (e.g., ACK to a different cmd)
                continue
            return self._parse(msg)
        return None

    @staticmethod
    def _parse(msg: MSPMessage) -> Dict[str, Any]:
        if msg.command == MSPCommand.MSP_RC:
            return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}
        if msg.command == MSPCommand.MSP_STATUS:
            import struct
            if len(msg.data) >= 11:
                cycle_time, i2c_errors, sensor, flag, _ = struct.unpack('<HHIBB', msg.data[:11])
                return {"cycle_time": cycle_time, "i2c_errors": i2c_errors, "sensor": sensor, "flag": flag, "armed": bool(flag & 1)}
        return {"raw_data": msg.data}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--margin", type=int, default=12, help="allowed difference when comparing channels")
    args = ap.parse_args()

    cli = MSPClient(args.port, args.baudrate)
    if not cli.connect():
        return 1

    try:
        print("\n=== Verifying MSP RC override ===")
        # MSP order: [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        sent = [1600, 1400, 1200, 1500, 1000, 1000, 1000, 1000]
        payload = MSPDataTypes.pack_rc_channels(sent)

        # Send the override frame (we expect only an empty ACK to cmd 200)
        cli.send(MSPCommand.MSP_SET_RAW_RC, payload)
        time.sleep(0.06)

        # Now explicitly query MSP_RC (105)
        resp = cli.req(MSPCommand.MSP_RC, expect=MSPCommand.MSP_RC, timeout=0.8)
        if not resp or "channels" not in resp:
            print("❌ No MSP_RC response received.")
            print("Checklist:")
            print("  • Ensure MSP is enabled on this UART in the Ports tab")
            print("  • You already set: set msp_override_channels = 31; save")
            print("  • Keep receiver_type = SERIAL (DO NOT set to MSP for override)")
            return 2

        got = resp["channels"]
        print(f"Sent (first 5): {sent[:5]}")
        print(f"Got  (first 5): {got[:5]}")

        match = True
        for i in range(5):
            if i >= len(got):
                match = False
                break
            if abs(got[i] - sent[i]) > args.margin:
                match = False

        if match:
            print("✅ MSP RC override appears ACTIVE.")
            return 0
        else:
            print("❌ Channels differ beyond margin.")
            print("  • Double-check your mode mapping and RC deadband")
            print("  • Confirm the UART truly runs MSP (no conflicting smartport/CRSF TX on same pins)")
            return 3

    finally:
        cli.close()

if __name__ == "__main__":
    import sys
    sys.exit(main())
