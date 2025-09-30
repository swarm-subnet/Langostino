#!/usr/bin/env python3
"""
msp_arm_and_throttle.py
Arms via AUX1 HIGH (Mode Ranges must map ARM to AUX1 high), then bumps THROTTLE.

Usage:
  python3 msp_arm_and_throttle.py throttle --throttle 1600 --seconds 3 \
      --port /dev/ttyAMA0 --baudrate 115200

SAFETY:
- Test with PROPS OFF first; restrain craft; use in a safe area.
- Ensure Mode Ranges: ARM on AUX1 HIGH.
"""

import time
import argparse
import serial
from typing import Optional, Dict, Any, List

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ = 40  # steady stream rate for RC frames

class MSPClient:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.rx = bytearray()

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            time.sleep(2.0)
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
                continue
            return self._parse(msg)
        return None

    @staticmethod
    def _parse(msg: MSPMessage) -> Dict[str, Any]:
        if msg.command == MSPCommand.MSP_STATUS:
            pass
        if msg.command == MSPCommand.MSP_RC:
            return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}
        return {"raw_data": msg.data}

    # ------------ High-level helpers ------------

    def stream_rc(self, channels: List[int], seconds: float, hz: int = STREAM_HZ):
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        payload = MSPDataTypes.pack_rc_channels(channels)
        while time.time() < t_end:
            # Re-pack is cheap, but we keep the pre-packed payload for cadence
            self.send(MSPCommand.MSP_SET_RAW_RC, payload)
            time.sleep(period)

    def get_status(self) -> Optional[Dict[str, Any]]:
        return self.req(MSPCommand.MSP_STATUS, expect=MSPCommand.MSP_STATUS, timeout=0.5)

    def arm(self, force: bool = False) -> bool:
        print("=== ARM (AUX1 HIGH) ===")
        if not force:
            st = self.get_status()
            if not st:
                print("⚠️  No STATUS; proceeding with caution.")
            elif not st.get("armed", False):
                # You can add detailed arming checks here if desired
                pass

        # MSP order: [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        # Arm with THR low and AUX1 high
        arm_frame = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1000]
        self.stream_rc(arm_frame, seconds=1.0, hz=STREAM_HZ)

        st2 = self.get_status()
        if st2 and st2.get("armed", False):
            print("✅ Armed")
            return True
        print("❌ Did not arm. Check Mode Ranges (ARM on AUX1 high) and arming flags.")
        return False

    def disarm(self):
        print("=== DISARM (AUX1 LOW) ===")
        disarm_frame = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.stream_rc(disarm_frame, seconds=0.8, hz=STREAM_HZ)
        st = self.get_status()
        if st and not st.get("armed", True):
            print("✅ Disarmed")
        else:
            print("⚠️  Disarm not confirmed; check arming flags.")

    def throttle_bump(self, throttle: int, seconds: float, hz: int = STREAM_HZ, auto_arm: bool = True) -> bool:
        print(f"=== THROTTLE BUMP: {throttle} for {seconds}s @ {hz}Hz ===")

        if auto_arm:
            if not self.arm(force=False):
                print("⚠️  Trying forced arm...")
                if not self.arm(force=True):
                    print("❌ Could not arm; aborting.")
                    return False

        # Hold AUX1 high while bumping throttle (index 2)
        frame = [1500, 1500, throttle, 1500, 1800, 1000, 1000, 1000]
        print("Streaming throttle bump...")
        self.stream_rc(frame, seconds=seconds, hz=hz)

        print("Dropping to idle throttle (still armed briefly)...")
        idle = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1000]
        self.stream_rc(idle, seconds=0.6, hz=hz)

        # Disarm for safety
        self.disarm()
        return True


def main():
    ap = argparse.ArgumentParser(description="Arm and throttle via MSP override")
    ap.add_argument("action", choices=["arm", "disarm", "throttle"])
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--throttle", type=int, default=1600, help="1000-2000")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--hz", type=int, default=STREAM_HZ)
    ap.add_argument("--force", action="store_true", help="skip arming checks")
    args = ap.parse_args()

    cli = MSPClient(args.port, args.baudrate)
    if not cli.connect():
        return 1

    try:
        if args.action == "arm":
            ok = cli.arm(force=args.force)
            return 0 if ok else 2
        elif args.action == "disarm":
            cli.disarm()
            return 0
        elif args.action == "throttle":
            ok = cli.throttle_bump(throttle=args.throttle, seconds=args.seconds, hz=args.hz, auto_arm=True)
            return 0 if ok else 3
    finally:
        cli.close()

if __name__ == "__main__":
    import sys
    sys.exit(main())
