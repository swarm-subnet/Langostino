#!/usr/bin/env python3
"""
arm_only_simple.py — INAV MSP "arm-only"

What it does
------------
- Connects to FC over MSP.
- Sends AUX1 high (CH5 ~1800) and throttle low (CH3 ~1000) for 5 seconds.
- Streams frames at 40 Hz so the FC sees a stable ARM command.
- Does NOT disarm afterwards, does NOT depend on MSP_STATUS or echo latch.

Matches your INAV config
------------------------
- Channel map AETR: [ROLL, PITCH, THROTTLE, YAW] = CH1..CH4
- ARM is CH5 (AUX1) -> >1700 arms
- MSP RC OVERRIDE is CH8 (AUX4) -> must be >1700 (radio or code) so MSP RC control is accepted.

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

STREAM_HZ = 40  # Hz
ARM_SECONDS = 5.0


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

    # -------------------- Actions --------------------

    def arm_for_seconds(self, seconds: float = ARM_SECONDS, hz: int = STREAM_HZ):
        """
        Stream ARM frame (AUX1 high, throttle low) for N seconds at hz.
        """
        print(f"=== ARMING for {seconds:.1f}s at {hz}Hz ===")
        # AETR + AUX: [ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4]
        channels = [1500, 1500, 1000, 1500, 1800, 1500, 1500, 1800]

        period = 1.0 / float(hz)
        t_end = time.time() + seconds
        while time.time() < t_end:
            t0 = time.time()
            self.send_rc(channels)
            dt = time.time() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)
        print("✅ ARM command streaming finished (FC should be armed if override was active).")


def main():
    parser = argparse.ArgumentParser(description="INAV MSP Arm-Only")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--seconds", type=float, default=ARM_SECONDS, help="Seconds to stream ARM (default: 5)")
    parser.add_argument("--hz", type=int, default=STREAM_HZ, help="Streaming rate Hz (default: 40)")
    args = parser.parse_args()

    print("SAFETY: Test with props OFF. Ensure CH8 (MSP RC OVERRIDE) >1700 on your radio.")

    fc = FlightController(args.port, args.baudrate)
    if not fc.connect():
        return 1

    try:
        fc.arm_for_seconds(seconds=args.seconds, hz=args.hz)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    finally:
        fc.close()


if __name__ == "__main__":
    import sys
    sys.exit(main())
