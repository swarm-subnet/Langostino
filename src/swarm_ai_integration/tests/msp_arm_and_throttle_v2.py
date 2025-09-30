#!/usr/bin/env python3
"""
msp_arm_and_throttle.py
Arms via AUX1 HIGH (Mode Ranges must map ARM to AUX1 high), then bumps THROTTLE.
Prints Tx/Rx each cycle (like the verifier). AERT ordering:
  CH1=Aileron (Roll), CH2=Elevator (Pitch), CH3=Rudder (Yaw), CH4=Throttle, CH5=AUX1, ...

STATUS:
- This script does NOT parse MSP_STATUS in the protocol. If you pass --status-every > 0,
  it will *safely* fetch MSP_STATUS and extract only the raw flags from bytes 6..9
  (little-endian), printing them as "(xx)". Otherwise no STATUS is requested.

Usage:
  python3 msp_arm_and_throttle.py throttle --throttle 1600 --seconds 3 \
      --port /dev/ttyAMA0 --baudrate 115200 --echo-every 1 --status-every 0

SAFETY:
- Props OFF for bench tests. Use a safe area when spinning motors.
- Modes: ARM on AUX1 HIGH; "MSP RC Override" mode ON.
- `msp_override_channels` must include A,E,R,T + AUX1 (e.g., 31).
"""

import time
import argparse
import serial
from typing import Optional, Dict, Any, List

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ = 40  # steady stream rate for RC frames

# AERT indices (0-based)
IDX_ROLL = 0      # Aileron
IDX_PITCH = 1     # Elevator
IDX_YAW = 2       # Rudder
IDX_THR = 3       # Throttle
IDX_AUX1 = 4
IDX_AUX2 = 5
IDX_AUX3 = 6
IDX_AUX4 = 7

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
        """Generic request/response helper (used for MSP_RC echo; STATUS optional)."""
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
        if msg.command == MSPCommand.MSP_RC:
            return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}
        # For STATUS we intentionally return raw bytes only; no struct unpack dependency.
        return {"raw_data": msg.data}

    # ------------ Logging helpers ------------

    @staticmethod
    def _fmt_line(prefix: str, values: List[int]) -> str:
        return f"{prefix}: " + " ".join(str(v) for v in values)

    def _status_flags_hex(self, raw: Optional[bytes]) -> str:
        """
        Safely extract 4-byte box_flags (bytes 6..9 little-endian) if present.
        Returns "(xx)" like the verifier; "(--)" if unavailable.
        """
        if not raw or len(raw) < 10:
            return "(--)"
        flags = int.from_bytes(raw[6:10], byteorder="little", signed=False)
        return f" ({flags:02x})"

    # ------------ High-level helpers ------------

    def stream_rc_echo(
        self,
        channels: List[int],
        seconds: float,
        hz: int,
        echo_every: int = 1,
        status_every: int = 0,
        maxchan_print: int = 16
    ):
        """
        Stream an RC frame and print Tx/Rx lines. Optionally fetch STATUS flags.
        - echo_every: query/print MSP_RC every N frames (1 = every frame)
        - status_every: fetch MSP_STATUS every N frames (0 = never)
        """
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        cycle = 0
        payload = MSPDataTypes.pack_rc_channels(channels)

        while time.time() < t_end:
            t0 = time.time()
            self.send(MSPCommand.MSP_SET_RAW_RC, payload)

            do_echo = (echo_every > 0 and (cycle % echo_every) == 0)
            do_status = (status_every > 0 and (cycle % status_every) == 0)

            status_suffix = ""
            if do_status:
                st = self.req(MSPCommand.MSP_STATUS, expect=MSPCommand.MSP_STATUS, timeout=0.2)
                status_suffix = self._status_flags_hex(st.get("raw_data") if st else None)

            if do_echo:
                rc = self.req(MSPCommand.MSP_RC, expect=MSPCommand.MSP_RC, timeout=0.25)
                tx_line = self._fmt_line("Tx", channels[:maxchan_print])
                if rc and "channels" in rc:
                    rx_vals = rc["channels"][:maxchan_print]
                    rx_line = self._fmt_line("Rx", rx_vals) + (status_suffix if status_suffix else "")
                else:
                    rx_line = "Rx: <no MSP_RC>" + (status_suffix if status_suffix else "")
                print(tx_line)
                print(rx_line)

            cycle += 1
            # maintain cadence
            sleep_left = period - (time.time() - t0)
            if sleep_left > 0:
                time.sleep(sleep_left)

    def arm(self, echo_every: int, status_every: int, hz: int) -> bool:
        """Send AUX1 HIGH with THROTTLE low for ~1s; prints Tx/Rx if echo enabled."""
        print("=== ARM (AUX1 HIGH, THR LOW; no status parsing) ===")
        ch = [1500] * 8
        ch[IDX_THR] = 1000
        ch[IDX_AUX1] = 1800
        self.stream_rc_echo(ch, seconds=1.0, hz=hz, echo_every=echo_every, status_every=status_every)
        print("Sent arming pulse.")
        return True

    def disarm(self, echo_every: int, status_every: int, hz: int):
        """Send AUX1 LOW for ~0.8s; prints Tx/Rx if echo enabled."""
        print("=== DISARM (AUX1 LOW; no status parsing) ===")
        ch = [1500] * 8
        ch[IDX_THR] = 1000
        ch[IDX_AUX1] = 1000
        self.stream_rc_echo(ch, seconds=0.8, hz=hz, echo_every=echo_every, status_every=status_every)
        print("Sent disarm pulse.")

    def throttle_bump(
        self,
        throttle: int,
        seconds: float,
        hz: int,
        auto_arm: bool,
        echo_every: int,
        status_every: int
    ) -> bool:
        """Hold AUX1 high and stream THROTTLE (AERT index 3) at `throttle` for `seconds`, with Tx/Rx prints."""
        print(f"=== THROTTLE BUMP: {throttle} for {seconds}s @ {hz}Hz (AERT order) ===")

        if auto_arm:
            self.arm(echo_every=echo_every, status_every=status_every, hz=hz)

        # Bump throttle while keeping AUX1 HIGH
        ch = [1500] * 8
        ch[IDX_THR] = throttle
        ch[IDX_AUX1] = 1800
        print("Streaming throttle bump...")
        self.stream_rc_echo(ch, seconds=seconds, hz=hz, echo_every=echo_every, status_every=status_every)

        # Idle throttle briefly (still armed)
        print("Dropping to idle throttle...")
        ch[IDX_THR] = 1000
        self.stream_rc_echo(ch, seconds=0.6, hz=hz, echo_every=echo_every, status_every=status_every)

        # Disarm for safety
        self.disarm(echo_every=echo_every, status_every=status_every, hz=hz)
        return True


def main():
    ap = argparse.ArgumentParser(description="Arm and throttle via MSP override (AERT order) with Tx/Rx echo")
    ap.add_argument("action", choices=["arm", "disarm", "throttle"])
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--throttle", type=int, default=1600, help="1000-2000")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--hz", type=int, default=STREAM_HZ)
    ap.add_argument("--no-arm", action="store_true", help="do not auto-arm before throttle bump")
    ap.add_argument("--echo-every", type=int, default=1, help="print Tx/Rx every N frames (1 = every frame)")
    ap.add_argument("--status-every", type=int, default=0, help="fetch STATUS every N frames (0 = never)")
    args = ap.parse_args()

    cli = MSPClient(args.port, args.baudrate)
    if not cli.connect():
        return 1

    try:
        if args.action == "arm":
            ok = cli.arm(echo_every=args.echo_every, status_every=args.status_every, hz=args.hz)
            return 0 if ok else 2
        elif args.action == "disarm":
            cli.disarm(echo_every=args.echo_every, status_every=args.status_every, hz=args.hz)
            return 0
        elif args.action == "throttle":
            ok = cli.throttle_bump(
                throttle=args.throttle,
                seconds=args.seconds,
                hz=args.hz,
                auto_arm=not args.no_arm,
                echo_every=args.echo_every,
                status_every=args.status_every
            )
            return 0 if ok else 3
    finally:
        cli.close()

if __name__ == "__main__":
    import sys
    sys.exit(main())
