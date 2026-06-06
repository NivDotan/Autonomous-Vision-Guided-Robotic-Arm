#!/usr/bin/env python3
"""
Standalone motor diagnostic — NO ROS2 required.

Reads the current position of each SO-101 motor using raw pyserial.
Run this BEFORE touching any ROS2 code to confirm serial communication works.

Usage:
    python3 read_motors.py
    python3 read_motors.py --port /dev/ttyUSB0
    python3 read_motors.py --port /dev/ttyACM0 --baud 1000000
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed.  pip install pyserial")
    sys.exit(1)

# ── Feetech STS3215 register map ──────────────────────────────────────────────
REG_PRESENT_POSITION = 56   # 2 bytes, unsigned 0-4096
REG_PRESENT_LOAD     = 60   # 2 bytes, signed
REG_PRESENT_CURRENT  = 69   # 2 bytes, signed

MOTOR_NAMES = ['base', 'shoulder', 'elbow', 'palm', 'wrist', 'gripper']
MOTOR_IDS   = [1,      2,          3,       4,      5,       6       ]


def checksum(body: list) -> int:
    return (~sum(body)) & 0xFF


def read_register(ser: serial.Serial, motor_id: int, reg: int, n_bytes: int) -> int | None:
    """Send a read request, skip the echo, return the register value or None."""
    # Build request: 0xFF 0xFF ID LEN(=4) 0x02 REG N_BYTES CHECKSUM
    body = [motor_id, 4, 0x02, reg, n_bytes]
    body.append(checksum(body))
    pkt = bytes([0xFF, 0xFF]) + bytes(body)

    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()

    # Note: SO-101 USB-RS485 adapter does NOT echo — read response directly.
    # Read response: 0xFF 0xFF ID LEN ERROR DATA... CHECKSUM
    resp = ser.read(n_bytes + 6)
    if len(resp) < n_bytes + 6:
        return None
    if resp[0] != 0xFF or resp[1] != 0xFF:
        return None
    error = resp[4]
    if error:
        return None

    data = resp[5:5 + n_bytes]
    if n_bytes == 1:
        return data[0]
    elif n_bytes == 2:
        val = data[0] | (data[1] << 8)
        return val
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=1_000_000)
    ap.add_argument('--loop', action='store_true', help='Keep reading every second')
    args = ap.parse_args()

    print(f"\nOpening {args.port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"ERROR: {e}")
        print("Check that /dev/ttyACM0 exists and you have permission: sudo chmod a+rw /dev/ttyACM0")
        sys.exit(1)

    time.sleep(0.2)
    print("Port open. Reading motors...\n")

    def read_once():
        any_ok = False
        print(f"{'Motor':<12} {'ID':>3}  {'Ticks':>6}  {'~Degrees':>10}  {'Load':>6}  {'Current':>8}")
        print("-" * 60)
        for name, mid in zip(MOTOR_NAMES, MOTOR_IDS):
            pos  = read_register(ser, mid, REG_PRESENT_POSITION, 2)
            load = read_register(ser, mid, REG_PRESENT_LOAD,     2)
            curr = read_register(ser, mid, REG_PRESENT_CURRENT,  2)

            if pos is None:
                print(f"  {name:<10} {mid:>3}  {'NO RESPONSE':>6}")
                continue

            any_ok = True
            deg = (pos - 2048) * 360.0 / 4096.0
            if load is not None and load > 32767:
                load -= 65536
            if curr is not None and curr > 32767:
                curr -= 65536

            load_str = f"{load:>6}" if load is not None else "  N/A"
            curr_str = f"{curr:>8}" if curr is not None else "    N/A"
            print(f"  {name:<10} {mid:>3}  {pos:>6}  {deg:>+10.1f}°  {load_str}  {curr_str}")

        print()
        return any_ok

    if args.loop:
        while True:
            read_once()
            time.sleep(1.0)
    else:
        ok = read_once()
        if not ok:
            print("No motors responded. Check:")
            print("  1. Are motors powered? (servo board LED on?)")
            print("  2. Is the USB-RS485 adapter plugged in?")
            print("  3. Is the port correct? Run: ls /dev/ttyACM* /dev/ttyUSB*")
            print("  4. Try --port /dev/ttyUSB0 if /dev/ttyACM0 gives no response")
        ser.close()


if __name__ == '__main__':
    main()
