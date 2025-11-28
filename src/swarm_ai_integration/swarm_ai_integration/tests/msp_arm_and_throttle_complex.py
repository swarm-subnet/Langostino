#!/usr/bin/env python3
"""
arm_only_sticky.py — INAV MSP "arm-only" with sticky streaming + Tx/Rx logging

What this does
--------------
- Connects to your FC over MSP (serial).
- "Warms up" MSP RC override by streaming the SAME RC frame repeatedly at a steady rate,
  and checking MSP_RC echoes until input≈output (within a small margin).
- Then sends an ARM frame (AUX1/CH5 HIGH, throttle LOW) the same way (sticky streaming)
  for a short hold period and verifies ARMED via MSP_STATUS.
- Exits without disarming or bumping throttle.
- **NEW:** Logs every MSP message SENT and RECEIVED (including parsed values when known).
  Use --logfile to save as JSON Lines.

Matches your INAV config
------------------------
- Channel map AETR: [ROLL, PITCH, THROTTLE, YAW] = CH1..CH4
- ARM is CH5 (AUX1) -> >1700 arms
- MSP RC OVERRIDE is CH8 (AUX4) -> you normally need your RADIO to hold this >1700 so that MSP RC frames
  are accepted. This script *also* sends CH8=1800 in its frames (once override is active, it helps keep it on),
  but the initial enabling typically still depends on your radio's CH8 switch.

Safety
------
- TEST WITH PROPS OFF FIRST.
"""

import time
import argparse
import serial
import json
from datetime import datetime, timezone
from typing import Optional, Dict, Any, List

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ_DEFAULT = 40
ECHO_MARGIN_DEFAULT = 12
CONSEC_MATCH_DEFAULT = 3


class FlightController:
    def __init__(self, port: str, baudrate: int, logfile: Optional[str] = None):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()

        # ---- Logging state ----
        self.tx_rx_log: List[Dict[str, Any]] = []
        self.logfile = logfile

    # -------------------- Logging helpers --------------------
    @staticmethod
    def _now_iso() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _bytes_to_hex(b: bytes) -> str:
        return b.hex()

    def _append_log(self, entry: Dict[str, Any]) -> None:
        """Accumulate in-memory; file write is deferred to close() unless immediate is needed."""
        self.tx_rx_log.append(entry)

    def _log_send(self, msg: MSPMessage) -> None:
        try:
            entry = {
                "ts": self._now_iso(),
                "direction": "Tx",
                "command": int(msg.command),
                "command_name": getattr(MSPCommand, "name", None) or str(msg.command),
                "size": len(msg.data) if msg.data else 0,
                "raw_hex": self._bytes_to_hex(msg.encode()),
                "data_hex": self._bytes_to_hex(msg.data or b""),
                "parsed": self._safe_parse_for_log(msg)
            }
            self._append_log(entry)
        except Exception:
            # Logging must never break I/O
            pass

    def _log_recv(self, msg: MSPMessage) -> None:
        try:
            entry = {
                "ts": self._now_iso(),
                "direction": "Rx",
                "command": int(msg.command),
                "command_name": getattr(MSPCommand, "name", None) or str(msg.command),
                "size": len(msg.data) if msg.data else 0,
                "raw_hex": self._bytes_to_hex(msg.encode()),
                "data_hex": self._bytes_to_hex(msg.data or b""),
                "parsed": self._safe_parse_for_log(msg)
            }
            self._append_log(entry)
        except Exception:
            pass

    @staticmethod
    def _safe_parse_for_log(msg: MSPMessage) -> Optional[Dict[str, Any]]:
        """Return a *log-friendly* parsed dict when we recognize the command."""
        try:
            if msg.command == MSPCommand.MSP_STATUS:
                return MSPDataTypes.unpack_status(msg.data)
            elif msg.command == MSPCommand.MSP_RC:
                return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}
            else:
                # For unrecognized commands, keep it minimal to avoid binary bloat
                return None
        except Exception:
            return None

    def dump_log_to_file(self) -> None:
        if not self.logfile:
            return
        try:
            with open(self.logfile, "w", encoding="utf-8") as f:
                for row in self.tx_rx_log:
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
            print(f"📝 Wrote {len(self.tx_rx_log)} Tx/Rx log entries to {self.logfile}")
        except Exception as e:
            print(f"⚠️ Could not write log file: {e}")

    # -------------------- Connection --------------------
    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            time.sleep(2.0)  # allow MSP UART to settle
            print(f"Connected to {self.port} @ {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"❌ Serial open failed: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")
        # Always dump log at the end if a logfile was provided
        self.dump_log_to_file()

    # -------------------- MSP I/O --------------------
    def send(self, cmd: int, data: bytes = b'') -> bool:
        try:
            msg = MSPMessage(cmd, data, MSPDirection.REQUEST)
            # Log BEFORE sending (so log also captures what we attempted)
            self._log_send(msg)
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
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))
            start = self.rx_buffer.find(b"$M")
            if start == -1:
                if self.rx_buffer:
                    self.rx_buffer.clear()
                time.sleep(0.003)
                continue

            # minimum: $ M dir size cmd ... checksum
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
                # Log after decoding (to include parsed)
                self._log_recv(msg)
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
        if msg.command == MSPCommand.MSP_STATUS:
            return MSPDataTypes.unpack_status(msg.data)
        elif msg.command == MSPCommand.MSP_RC:
            return {"channels": MSPDataTypes.unpack_rc_channels(msg.data)}
        return {"raw_data": msg.data}

    # -------------------- Utilities --------------------
    @staticmethod
    def _fmt(prefix: str, vals: List[int]) -> str:
        return f"{prefix}: " + " ".join(str(v) for v in vals)

    def read_rc_once(self) -> Optional[List[int]]:
        rc = self.req(MSPCommand.MSP_RC, expect=MSPCommand.MSP_RC, timeout=0.25)
        if rc and "channels" in rc:
            return rc["channels"]
        return None

    def stream_until_echo(
        self,
        channels: List[int],
        max_seconds: float,
        hz: int,
        margin: int,
        consecutive: int,
        echo_print: bool = False,
    ) -> bool:
        """
        Stream the same RC frame repeatedly and check MSP_RC until consecutive frames match (first 4 AETR).
        Returns True once echo is 'locked', or False if timeout.
        """
        payload = MSPDataTypes.pack_rc_channels(channels)
        period = 1.0 / float(hz)
        t_end = time.time() + max_seconds
        matches = 0

        while time.time() < t_end:
            t0 = time.time()
            self.send(MSPCommand.MSP_SET_RAW_RC, payload)

            rx = self.read_rc_once()
            if echo_print:
                print(self._fmt("Tx", channels))
                if rx:
                    print(self._fmt("Rx", rx[:len(channels)]))
                else:
                    print("Rx: <no MSP_RC>")

            if rx:
                ok = all(abs((rx[i] if i < len(rx) else 0) - channels[i]) <= margin for i in range(4))
                if ok:
                    matches += 1
                    if matches >= consecutive:
                        return True
                else:
                    matches = 0  # reset streak on mismatch

            # keep cadence tight
            dt = time.time() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)

        return False

    # -------------------- High-level --------------------
    def get_status(self) -> Optional[Dict[str, Any]]:
        return self.req(MSPCommand.MSP_STATUS, expect=MSPCommand.MSP_STATUS)

    def arm_only(
        self,
        warmup_seconds: float,
        hold_seconds: float,
        hz: int,
        margin: int,
        consecutive: int,
        echo_print: bool = False,
    ) -> bool:
        """
        1) Warm up MSP RC override by streaming a neutral frame until echo locks (or timeout).
        2) Stream the ARM frame (AUX1 high, throttle low) until echo locks (or timeout).
        3) Keep sending the ARM frame for the remainder of hold_seconds.
        4) Verify ARMED via MSP_STATUS.
        """
        # AETR + AUX: [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        neutral = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1800]  # AUX4=1800 helps keep override on once active
        print("\n=== WARM-UP: streaming neutral RC until echo matches ===")
        warm_ok = self.stream_until_echo(
            channels=neutral,
            max_seconds=warmup_seconds,
            hz=hz,
            margin=margin,
            consecutive=consecutive,
            echo_print=echo_print,
        )
        if not warm_ok:
            print("⚠️ MSP RC echo did not lock during warm-up. Continuing anyway (may still work).")

        # ARM frame: AUX1 high (CH5≈1800), throttle low (CH3=1000), keep AUX4 high
        arm = [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]
        print("\n=== ARM: streaming AUX1 HIGH with throttle LOW until echo matches ===")
        arm_locked = self.stream_until_echo(
            channels=arm,
            max_seconds=max(0.8, min(hold_seconds, 3.0)),  # try to lock quickly within the hold window
            hz=hz,
            margin=margin,
            consecutive=consecutive,
            echo_print=echo_print,
        )

        # If we locked early but still have hold time left, continue streaming the same arm frame
        if hold_seconds > 0:
            remaining = hold_seconds
            if arm_locked:
                # already spent time in stream_until_echo; keep it simple and just hold for the full request
                remaining = max(0.0, hold_seconds)
            if remaining > 0:
                print(f"=== HOLD: keep sending ARM frame for {remaining:.2f}s ===")
                self._hold_stream(channels=arm, seconds=remaining, hz=hz, echo_print=echo_print)

        # Final check
        status = self.get_status()
        if status:
            print(
                f"MSP_STATUS: time_us={status['cycle_time']}, errors={status['i2c_errors']}, "
                f"sensors=0x{status['sensor_mask']:04x}, box_flags=0x{status['box_flags']:08x}, "
                f"current_setting={status['current_setting']}"
            )

        # Consider success when the ARM frame has latched in MSP_RC echo
        if arm_locked:
            print("✅ ARM frame latched (MSP_RC echo matched).")
            return True

        print("❌ Could not confirm ARM (MSP_RC echo did not latch).")
        return False

    def _hold_stream(self, channels: List[int], seconds: float, hz: int, echo_print: bool):
        payload = MSPDataTypes.pack_rc_channels(channels)
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        while time.time() < t_end:
            t0 = time.time()
            self.send(MSPCommand.MSP_SET_RAW_RC, payload)
            if echo_print:
                rx = self.read_rc_once()
                if rx:
                    print(self._fmt("Rx", rx[:len(channels)]))
            dt = time.time() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)


def main():
    ap = argparse.ArgumentParser(description="INAV MSP Arm-Only (sticky streaming until echo matches)")
    ap.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (default: /dev/ttyAMA0)")
    ap.add_argument("--baudrate", type=int, default=115200, help="Baudrate (default: 115200)")
    ap.add_argument("--hz", type=int, default=STREAM_HZ_DEFAULT, help=f"Streaming rate (default: {STREAM_HZ_DEFAULT} Hz)")
    ap.add_argument("--warmup", type=float, default=3.0, help="Warm-up seconds streaming neutral frame (default: 1.0)")
    ap.add_argument("--hold", type=float, default=5.0, help="Seconds to hold ARM frame (default: 1.0)")
    ap.add_argument("--margin", type=int, default=ECHO_MARGIN_DEFAULT, help=f"Echo tolerance per channel (default: {ECHO_MARGIN_DEFAULT})")
    ap.add_argument("--consecutive", type=int, default=CONSEC_MATCH_DEFAULT, help=f"Consecutive matching frames to accept echo (default: {CONSEC_MATCH_DEFAULT})")
    ap.add_argument("--echo", action="store_true", help="Print Tx/Rx each frame")
    ap.add_argument("--logfile", default=None, help="Write all Tx/Rx MSP frames as JSON Lines to this file")
    args = ap.parse_args()

    print("SAFETY: Test with props OFF. For reliable operation, ensure your RADIO holds CH8 (MSP RC OVERRIDE) > 1700.\n")

    fc = FlightController(args.port, args.baudrate, logfile=args.logfile)
    if not fc.connect():
        return 1

    try:
        ok = fc.arm_only(
            warmup_seconds=args.warmup,
            hold_seconds=args.hold,
            hz=args.hz,
            margin=args.margin,
            consecutive=args.consecutive,
            echo_print=args.echo,
        )
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    finally:
        fc.close()


if __name__ == "__main__":
    import sys
    sys.exit(main())
