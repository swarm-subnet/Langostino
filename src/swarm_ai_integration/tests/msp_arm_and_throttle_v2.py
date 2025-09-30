#!/usr/bin/env python3
"""
msp_arm_and_throttle.py
Hardcoded: 2s ARM (AUX1 high), 8s THROTTLE bump, 2s DISARM.
- Prints Tx/Rx every frame.
- Channel order assumed RPYT (Roll, Pitch, Yaw, Throttle), then AUX:
    CH1=Roll, CH2=Pitch, CH3=Yaw, CH4=Throttle, CH5=AUX1, CH6=AUX2, ...

No MSP_STATUS requests at all.

Usage:
  python3 msp_arm_and_throttle.py --port /dev/ttyAMA0 --baudrate 115200
"""

import time
import argparse
import serial
from typing import Optional, Dict, Any, List

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ = 40  # Hz
THROTTLE_VAL = 1600  # hardcoded throttle during 8s phase

# RPYT indices (0-based)
IDX_ROLL = 0
IDX_PITCH = 1
IDX_YAW = 2
IDX_THR = 3
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
                time.sleep(0.002); continue
            size = self.rx[start + 3]
            frame_len = 6 + size
            if len(self.rx) - start < frame_len:
                time.sleep(0.002); continue
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
            if not msg or msg.direction != MSPDirection.RESPONSE or msg.command != expect:
                continue
            return self._parse(msg)
        return None

    @staticmethod
    def _parse(msg: MSPMessage) -> Dict[str, Any]:
        if msg.command == MSPCommand.MSP_RC:
            return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}
        return {"raw_data": msg.data}

    # ---------- echo streaming ----------

    @staticmethod
    def _fmt_line(prefix: str, values: List[int]) -> str:
        return f"{prefix}: " + " ".join(str(v) for v in values)

    def stream_rc_echo(self, channels: List[int], seconds: float, hz: int = STREAM_HZ):
        """Stream one RC frame for `seconds` at `hz`, printing Tx/Rx each frame."""
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        payload = MSPDataTypes.pack_rc_channels(channels)
        n = len(channels)

        while time.time() < t_end:
            t0 = time.time()
            self.send(MSPCommand.MSP_SET_RAW_RC, payload)
            rc = self.req(MSPCommand.MSP_RC, expect=MSPCommand.MSP_RC, timeout=0.25)

            print(self._fmt_line("Tx", channels))
            if rc and "channels" in rc:
                rx_vals = rc["channels"][:n]
                print(self._fmt_line("Rx", rx_vals))
            else:
                print("Rx: <no MSP_RC>")

            sleep_left = period - (time.time() - t0)
            if sleep_left > 0:
                time.sleep(sleep_left)

    # ---------- hardcoded phases ----------

    def phase_arm_2s(self):
        print("=== ARM (2s) — AUX1 HIGH (CH5=1800), THR LOW (CH4=1000) ===")
        ch = [1500] * 8
        ch[IDX_ROLL] = 1500
        ch[IDX_PITCH] = 1500
        ch[IDX_YAW] = 1500
        ch[IDX_THR] = 1000
        ch[IDX_AUX1] = 1800
        self.stream_rc_echo(ch, seconds=2.0, hz=STREAM_HZ)

    def phase_throttle_8s(self):
        print(f"=== THROTTLE (8s) — THR={THROTTLE_VAL}, AUX1 HIGH ===")
        ch = [1500] * 8
        ch[IDX_ROLL] = 1500
        ch[IDX_PITCH] = 1500
        ch[IDX_YAW] = 1500
        ch[IDX_THR] = THROTTLE_VAL
        ch[IDX_AUX1] = 1800
        self.stream_rc_echo(ch, seconds=8.0, hz=STREAM_HZ)

    def phase_disarm_2s(self):
        print("=== DISARM (2s) — AUX1 LOW (CH5=1000), THR LOW (CH4=1000) ===")
        ch = [1500] * 8
        ch[IDX_ROLL] = 1500
        ch[IDX_PITCH] = 1500
        ch[IDX_YAW] = 1500
        ch[IDX_THR] = 1000
        ch[IDX_AUX1] = 1000
        self.stream_rc_echo(ch, seconds=2.0, hz=STREAM_HZ)


def main():
    ap = argparse.ArgumentParser(description="Hardcoded MSP RPYT arm/throttle/disarm with Tx/Rx echo")
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baudrate", type=int, default=115200)
    args = ap.parse_args()

    cli = MSPClient(args.port, args.baudrate)
    if not cli.connect():
        return 1

    try:
        cli.phase_arm_2s()        # 2 seconds
        cli.phase_throttle_8s()   # 8 seconds at THROTTLE_VAL
        cli.phase_disarm_2s()     # 2 seconds
    finally:
        cli.close()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
