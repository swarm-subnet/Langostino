#!/usr/bin/env python3
"""
msp_override_verify.py
Continuously verifies MSP RC override by STREAMING MSP_SET_RAW_RC at a fixed cadence
and reading MSP_RC each cycle, printing Tx/Rx lines similar to the Codeberg example.

Usage examples:
  # Stream at 10 Hz (every 100ms), override AUX1 (channel 5) to 1800, others at 1567:
  python3 msp_override_verify.py --port /dev/ttyAMA0 --baudrate 115200 --every 100 5=1800

  # Override channel 14 to 1234 like in the readme example, send 18 channels:
  python3 msp_override_verify.py --port /dev/ttyAMA0 --baudrate 115200 --every 100 --maxchan 18 14=1234

  # Free-run (no timer), press Ctrl+C to stop:
  python3 msp_override_verify.py --every 0 5=1800

Notes:
- Channel numbers are 1-based by default (so "5" means AUX1). Use --zero-based if you prefer 0-based.
- Keep receiver_type = SERIAL (or NONE). Do NOT set it to MSP for override.
- You must have set: set msp_override_channels = <mask>; save
- Maintain >= ~5 Hz stream; slower cadences may fall back to RX/failsafe.
"""

import time
import argparse
import serial
from typing import Optional, Dict, Any, List

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
            time.sleep(2.0)  # allow FC UART bring-up
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
                time.sleep(0.002)
                continue

            if len(self.rx) - start < 6:
                time.sleep(0.0015)
                continue

            size = self.rx[start + 3]
            frame_len = 6 + size
            if len(self.rx) - start < frame_len:
                time.sleep(0.0015)
                continue

            frame = bytes(self.rx[start:start + frame_len])
            del self.rx[:start + frame_len]

            msg = MSPMessage.decode(frame)
            if msg:
                return msg
        return None

    def req(self, cmd: int, expect: Optional[int] = None, data: bytes = b'', timeout: float = 0.5) -> Optional[Dict[str, Any]]:
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
        if msg.command == MSPCommand.MSP_STATUS:
            #return MSPDataTypes.unpack_status(msg.data)
            pass
        return {"raw_data": msg.data}

def build_tx_frame(maxchan: int, defval: int, overrides: Dict[int, int]) -> List[int]:
    """
    Build a channel list of length `maxchan`, pre-filled with `defval`,
    with any `overrides` (0-based indices) applied.
    """
    tx = [defval for _ in range(maxchan)]
    for idx, val in overrides.items():
        if 0 <= idx < maxchan:
            tx[idx] = val
    return tx

def parse_overrides(assignments: List[str], zero_based: bool) -> Dict[int, int]:
    """
    Parse ['14=1234', '5=1800', ...] into {index0: value, ...}
    Interprets the left side as 1-based unless zero_based=True.
    """
    overrides: Dict[int, int] = {}
    for a in assignments:
        if '=' not in a:
            raise ValueError(f'Bad channel assignment "{a}". Use CH=VALUE, e.g., 5=1800')
        chs, vals = a.split('=', 1)
        try:
            ch = int(chs.strip())
            val = int(vals.strip())
        except ValueError:
            raise ValueError(f'Bad channel/value in "{a}"')
        if not zero_based:
            ch -= 1  # convert to 0-based index
        if ch < 0:
            raise ValueError(f'Channel index must be >= 0 (was {ch})')
        if not (800 <= val <= 2200):
            raise ValueError(f'PWM value out of range (800..2200): {val}')
        overrides[ch] = val
    return overrides

def main():
    ap = argparse.ArgumentParser(description="Stream MSP_SET_RAW_RC and print Tx/Rx each cycle.")
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--every", type=int, default=100, help="refresh period in ms (0 = free-run)")
    ap.add_argument("--defval", type=int, default=1567, help="default PWM for non-overridden channels")
    ap.add_argument("--maxchan", type=int, default=16, help="how many channels to send (e.g., 16 or 18)")
    ap.add_argument("--zero-based", action="store_true", help="treat channel numbers as 0-based (default is 1-based)")
    ap.add_argument("--status-every", type=int, default=5, help="query MSP_STATUS every N cycles (0 = never)")
    ap.add_argument("assign", nargs="*", help="channel=value pairs, e.g. 5=1800 (1-based by default)")
    args = ap.parse_args()

    try:
        overrides = parse_overrides(args.assign, zero_based=args.zero_based)
    except ValueError as e:
        print(f"❌ {e}")
        return 2

    cli = MSPClient(args.port, args.baudrate)
    if not cli.connect():
        return 1

    try:
        print("\n=== Streaming MSP RC override (Ctrl+C to stop) ===")
        if args.every != 0 and args.every < 200:
            print(f"(Info) Using {args.every} ms cadence (~{int(1000/max(1,args.every))} Hz). "
                  f"Keep ≥ 5 Hz; slower may fail over to RX/failsafe.")
        elif args.every == 0:
            print("(Info) Free-running (no delay).")

        cycle = 0
        period = args.every / 1000.0 if args.every > 0 else 0.0

        while True:
            t0 = time.time()

            # Build current Tx frame
            tx = build_tx_frame(args.maxchan, args.defval, overrides)
            payload = MSPDataTypes.pack_rc_channels(tx)

            # Send override for this cycle
            cli.send(MSPCommand.MSP_SET_RAW_RC, payload)

            # Immediately ask for MSP_RC and (optionally) MSP_STATUS
            rc = cli.req(MSPCommand.MSP_RC, expect=MSPCommand.MSP_RC, timeout=0.25)
            status_str = ""
            if args.status_every > 0 and (cycle % max(1, args.status_every) == 0):
                st = cli.req(MSPCommand.MSP_STATUS, expect=MSPCommand.MSP_STATUS, timeout=0.2)
                if st:
                    status_str = f" ({st.get('box_flags', 0):02x})" if st else ""


            # Print Tx / Rx similar to the example
            if args.maxchan <= 18:
                # keep lines compact like the example
                print("Tx: " + " ".join(f"{v}" for v in tx))
            else:
                print(f"Tx({args.maxchan}): " + " ".join(f"{v}" for v in tx))

            if rc and "channels" in rc:
                rx = rc["channels"]
                # Pad/truncate to match display width
                if len(rx) < args.maxchan:
                    rx = rx + [0] * (args.maxchan - len(rx))
                else:
                    rx = rx[:args.maxchan]
                print("Rx: " + " ".join(f"{v}" for v in rx) + (f" {status_str}" if status_str else ""))
            else:
                print("Rx: <no MSP_RC>")

            cycle += 1

            # sleep to maintain cadence
            if period > 0:
                elapsed = time.time() - t0
                to_sleep = period - elapsed
                if to_sleep > 0:
                    time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("\n^C")
    finally:
        cli.close()

    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())
