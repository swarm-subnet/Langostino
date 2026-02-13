#!/usr/bin/env python3
"""
arm_and_throttle_simple.py — INAV MSP "arm, then throttle"

What it does
------------
- Connects to FC over MSP.
- Phase 1: streams ARM frame (AUX1 high, throttle low) for 3 seconds.
- Phase 2: keeps AUX1 high and sets throttle to 1500 for a set duration (default 5 seconds).
- Streams frames at a steady rate (default 40 Hz).
- Does NOT depend on MSP_STATUS or MSP_RC echo; it just streams the frames.

Matches your INAV config
------------------------
- Channel map AETR: [ROLL, PITCH, THROTTLE, YAW] = CH1..CH4
- ARM is CH5 (AUX1) -> >1700 arms
- MSP RC OVERRIDE is CH8 (AUX4) -> should be >1700 (radio or code) so MSP RC control is accepted.

Safety
------
- TEST WITH PROPS OFF FIRST.
"""

import time
import argparse
import serial
from typing import Optional, List

from swarm_ai_integration.msp_protocol import (
    MSPMessage, MSPCommand, MSPDirection, MSPDataTypes
)

STREAM_HZ_DEFAULT = 40
ARM_SECONDS_DEFAULT = 3.0
THROTTLE_VALUE_DEFAULT = 1500
THROTTLE_SECONDS_DEFAULT = 2.5


class FlightController:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None

    # -------------------- Connection --------------------

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            time.sleep(2.0)  # allow FC to boot its MSP UART
            print(f"Connected to {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"❌ Serial open failed: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")

    # -------------------- MSP send --------------------

    def send_rc(self, channels: List[int]):
        """Send RC channels once via MSP_SET_RAW_RC"""
        payload = MSPDataTypes.pack_rc_channels(channels)
        msg = MSPMessage(MSPCommand.MSP_SET_RAW_RC, payload, MSPDirection.REQUEST)
        try:
            self.ser.write(msg.encode())
            self.ser.flush()
        except Exception as e:
            print(f"❌ Send error: {e}")

    # -------------------- Helpers --------------------

    def _stream_for(self, channels: List[int], seconds: float, hz: int):
        """Stream given RC frame at hz for seconds."""
        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        while time.time() < t_end:
            t0 = time.time()
            self.send_rc(channels)
            dt = time.time() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)

    # -------------------- Actions --------------------

    def arm_then_throttle(
        self,
        arm_seconds: float = ARM_SECONDS_DEFAULT,
        throttle_value: int = THROTTLE_VALUE_DEFAULT,
        throttle_seconds: float = THROTTLE_SECONDS_DEFAULT,
        hz: int = STREAM_HZ_DEFAULT,
    ):
        """
        Phase 1: ARM for arm_seconds (AUX1 high, throttle low)
        Phase 2: THROTTLE to throttle_value for throttle_seconds (AUX1 stays high)
        """
        # Clamp throttle for safety
        throttle_value = max(1000, min(2000, throttle_value))

        print(f"=== PHASE 1: ARM for {arm_seconds:.1f}s @ {hz}Hz ===")
        # AETR + AUX: [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        arm_frame = [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]
        self._stream_for(arm_frame, arm_seconds, hz)

        print(f"=== PHASE 2: THROTTLE={1100} for {throttle_seconds:.1f}s @ {hz}Hz (AUX1 stays HIGH) ===")
        throttle_frame = [1500, 1500, 1100, 1500, 1800, 1500, 1500, 1800]
        self._stream_for(throttle_frame, throttle_seconds, hz)

        print(f"=== PHASE 3: THROTTLE={1200} for {throttle_seconds:.1f}s @ {hz}Hz (AUX1 stays HIGH) ===")
        throttle_frame = [1500, 1500, 1200, 1500, 1800, 1500, 1500, 1800]
        self._stream_for(throttle_frame, throttle_seconds, hz)

        print(f"=== PHASE 4: THROTTLE={1300} for {throttle_seconds:.1f}s @ {hz}Hz (AUX1 stays HIGH) ===")
        throttle_frame = [1500, 1500, 1300, 1500, 1800, 1500, 1500, 1800]
        self._stream_for(throttle_frame, throttle_seconds, hz)

        print(f"=== PHASE 5: THROTTLE={1400} for {throttle_seconds:.1f}s @ {hz}Hz (AUX1 stays HIGH) ===")
        throttle_frame = [1500, 1500, 1400, 1500, 1800, 1500, 1500, 1800]
        self._stream_for(throttle_frame, throttle_seconds, hz)

        print(f"=== PHASE 5: THROTTLE={1600} for {throttle_seconds:.1f}s @ {hz}Hz (AUX1 stays HIGH) ===")
        throttle_frame = [1500, 1500, 1600, 1500, 1800, 1500, 1500, 1800]
        self._stream_for(throttle_frame, throttle_seconds, hz)

        print("✅ Completed arm + throttle sequence.")


def main():
    parser = argparse.ArgumentParser(description="INAV MSP Arm → Throttle")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--arm-seconds", type=float, default=ARM_SECONDS_DEFAULT, help="Seconds to stream ARM (default: 3)")
    parser.add_argument("--throttle", type=int, default=THROTTLE_VALUE_DEFAULT, help="Throttle value 1000..2000 (default: 1500)")
    parser.add_argument("--throttle-seconds", type=float, default=THROTTLE_SECONDS_DEFAULT, help="Seconds to hold throttle (default: 5)")
    parser.add_argument("--hz", type=int, default=STREAM_HZ_DEFAULT, help=f"Streaming rate Hz (default: {STREAM_HZ_DEFAULT})")
    args = parser.parse_args()

    print("SAFETY: Test with props OFF. Ensure CH8 (MSP RC OVERRIDE) >1700 on your radio.")

    fc = FlightController(args.port, args.baudrate)
    if not fc.connect():
        return 1

    try:
        fc.arm_then_throttle(
            arm_seconds=args.arm_seconds,
            throttle_value=args.throttle,
            throttle_seconds=args.throttle_seconds,
            hz=args.hz,
        )
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    finally:
        fc.close()


if __name__ == "__main__":
    import sys
    sys.exit(main())
