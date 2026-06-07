#!/usr/bin/env python3
"""
Raw move test — NO daemon, NO ROS2, NO app.

Enables torque and commands a small move on each motor, then reads back to
confirm the servo physically responded. Run this with the daemon and app
STOPPED so this script owns /dev/ttyACM0.

Usage:
    python3 tools/move_test.py            # moves base +150 then back
    python3 tools/move_test.py --id 2     # test motor 2 (shoulder)
"""
import argparse, sys, time
try:
    import serial
except ImportError:
    sys.exit("pip install pyserial")

REG_TORQUE_ENABLE    = 40
REG_GOAL_POSITION    = 42
REG_PRESENT_POSITION = 56


def chk(b): return (~sum(b)) & 0xFF

def read_pos(ser, mid):
    body = [mid, 4, 0x02, REG_PRESENT_POSITION, 2]; body.append(chk(body))
    ser.reset_input_buffer(); ser.write(bytes([0xFF,0xFF])+bytes(body)); ser.flush()
    r = ser.read(8)
    if len(r) < 8 or r[0] != 0xFF: return None
    return r[5] | (r[6] << 8)

def write_reg(ser, mid, reg, val, n):
    data = [val & 0xFF, (val>>8) & 0xFF] if n==2 else [val & 0xFF]
    body = [mid, 3+n, 0x03, reg] + data; body.append(chk(body))
    ser.write(bytes([0xFF,0xFF])+bytes(body)); ser.flush(); ser.read(6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--delta", type=int, default=150)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 1_000_000, timeout=0.2); time.sleep(0.2)

    p0 = read_pos(ser, args.id)
    print(f"motor {args.id} present position: {p0}")
    if p0 is None:
        sys.exit("No response — wrong port, or daemon/app still holding it?")

    print("enabling torque…")
    write_reg(ser, args.id, REG_TORQUE_ENABLE, 1, 1)
    time.sleep(0.1)

    target = p0 + args.delta
    print(f"commanding {p0} -> {target}")
    write_reg(ser, args.id, REG_GOAL_POSITION, target, 2)
    time.sleep(1.2)

    p1 = read_pos(ser, args.id)
    print(f"position after move: {p1}  (moved {p1 - p0:+d} ticks)")

    # move back
    write_reg(ser, args.id, REG_GOAL_POSITION, p0, 2)
    time.sleep(1.2)
    p2 = read_pos(ser, args.id)
    print(f"returned to: {p2}")

    if abs((p1 or p0) - p0) > 30:
        print("\nRESULT: ✅ motor MOVES from raw serial. Torque + write path OK.")
        print("        => the problem is in the daemon/app layer, not the hardware.")
    else:
        print("\nRESULT: ❌ motor did NOT move even with torque enabled.")
        print("        => check: servo power supply ON? correct motor ID? cable?")
    ser.close()


if __name__ == "__main__":
    main()
