"""
feetech_direct.py — minimal Feetech STS3215 driver using only pyserial.

No lerobot, no SDK required. Implements the SCServo half-duplex protocol
directly so the driver works on Linux out of the box with:
    pip install pyserial

Protocol reference: Feetech SCS/STS serial servo protocol
  Header:    0xFF 0xFF
  Packet:    [ID] [LEN] [INSTR] [PARAMS...] [CHECKSUM]
  Checksum:  ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF
  2-byte values: little-endian (low byte first)

Half-duplex note:
  The USB-RS485 adapter used with the SO-101 echoes transmitted bytes back on
  RX. We flush the input buffer before sending and skip the echo bytes
  (same count as sent) before reading the actual response.
"""
from __future__ import annotations

import time
from typing import Optional

try:
    import serial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

# ── STS3215 register addresses ─────────────────────────────────────────────
REG_TORQUE_ENABLE    = 40   # 1 byte
REG_GOAL_POSITION    = 42   # 2 bytes
REG_GOAL_VELOCITY    = 46   # 2 bytes
REG_PRESENT_POSITION = 56   # 2 bytes, unsigned
REG_PRESENT_LOAD     = 60   # 2 bytes, signed (bit15 = direction)
REG_PRESENT_CURRENT  = 69   # 2 bytes, signed int16

MOTOR_NAMES = ('base', 'shoulder', 'elbow', 'palm', 'wrist', 'gripper')
MOTOR_IDS   = (1, 2, 3, 4, 5, 6)


def _checksum(body: list[int]) -> int:
    return (~sum(body)) & 0xFF


class FeetechBus:
    """Low-level Feetech SCS/STS serial bus over pyserial."""

    def __init__(self, port: str, baud: int = 1_000_000) -> None:
        if not _SERIAL_OK:
            raise ImportError('pyserial not installed: pip install pyserial')
        self._ser = serial.Serial(
            port, baud,
            timeout=0.1,            # 100 ms read timeout per byte
            write_timeout=0.1,
        )
        time.sleep(0.1)             # let the port settle

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    # ── packet helpers ────────────────────────────────────────────────────

    def _send(self, motor_id: int, instr: int, params: list[int]) -> None:
        """Build and send a packet."""
        length = len(params) + 2    # instr + params + checksum
        body   = [motor_id, length, instr] + params
        body.append(_checksum(body))
        pkt = bytes([0xFF, 0xFF]) + bytes(body)
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        self._ser.flush()

    def _recv(self, n_params: int) -> Optional[bytes]:
        """Read response. Returns param bytes or None on error.
        Note: the SO-101 USB adapter does NOT echo TX bytes, so no skip needed."""
        # response: 0xFF 0xFF ID LEN ERROR PARAMS... CHECKSUM
        resp_total = n_params + 6   # header(2)+id+len+error+checksum
        raw = self._ser.read(resp_total)
        if len(raw) < 6 or raw[0] != 0xFF or raw[1] != 0xFF:
            return None
        error = raw[4]
        if error:
            return None
        return raw[5:5 + n_params]

    # ── public read / write ───────────────────────────────────────────────

    def read_u16(self, motor_id: int, reg: int) -> Optional[int]:
        self._send(motor_id, 0x02, [reg, 2])
        raw = self._recv(2)
        if raw is None or len(raw) < 2:
            return None
        return raw[0] | (raw[1] << 8)

    def write_u16(self, motor_id: int, reg: int, value: int) -> None:
        self._send(motor_id, 0x03, [reg, value & 0xFF, (value >> 8) & 0xFF])
        self._recv(0)               # consume ack

    def write_u8(self, motor_id: int, reg: int, value: int) -> None:
        self._send(motor_id, 0x03, [reg, value & 0xFF])
        self._recv(0)

    def sync_write_u16(self, reg: int, id_value_pairs: list[tuple[int, int]]) -> None:
        """Write the same register on multiple motors in one packet (no response)."""
        params = [reg, 2]           # reg, data_len
        for mid, val in id_value_pairs:
            params += [mid, val & 0xFF, (val >> 8) & 0xFF]
        body = [0xFE, len(params) + 2, 0x83] + params
        body.append(_checksum(body))
        pkt  = bytes([0xFF, 0xFF]) + bytes(body)
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        self._ser.flush()


class FeetechDirectHardware:
    """
    Drop-in replacement for robot_sam2_app_v2's FeetechHardware using only pyserial.

    Compatible interface with DaemonHardware / FeetechHardware:
      connect() / disconnect()
      read_ticks() / write_ticks()
      set_torque() / read_gripper_state() / gripper_load_detected()

    Install dep: pip install pyserial
    """

    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 1_000_000) -> None:
        self._port = port
        self._baud = baud
        self._bus: Optional[FeetechBus] = None
        self.connected = False

    def connect(self) -> bool:
        try:
            self._bus = FeetechBus(self._port, self._baud)
            # Sanity-check: read motor 1 position
            pos = self._bus.read_u16(1, REG_PRESENT_POSITION)
            if pos is None:
                raise RuntimeError(
                    f'No response from motor ID=1 on {self._port}. '
                    'Check cable, port, and that motors are powered.'
                )
            self.connected = True
            print(f'[FeetechDirect] Connected on {self._port} — motor 1 pos={pos}')
            return True
        except Exception as exc:
            print(f'[FeetechDirect] Connect failed: {exc}')
            self._bus = None
            self.connected = False
            return False

    def disconnect(self) -> None:
        if self._bus is not None:
            self._bus.close()
        self.connected = False

    def read_ticks(self) -> Optional[dict[str, int]]:
        if not self.connected or self._bus is None:
            return None
        ticks: dict[str, int] = {}
        for name, mid in zip(MOTOR_NAMES, MOTOR_IDS):
            v = self._bus.read_u16(mid, REG_PRESENT_POSITION)
            ticks[name] = v if v is not None else 2048
        return ticks

    def write_ticks(self, ticks: dict[str, int]) -> None:
        """Use sync-write when all 6 joints are present; fallback to individual writes."""
        if not self.connected or self._bus is None:
            return
        pairs = [
            (mid, int(ticks[name]))
            for name, mid in zip(MOTOR_NAMES, MOTOR_IDS)
            if name in ticks
        ]
        if len(pairs) == 6:
            self._bus.sync_write_u16(REG_GOAL_POSITION, pairs)
        else:
            for mid, val in pairs:
                self._bus.write_u16(mid, REG_GOAL_POSITION, val)

    def set_torque(self, enabled: bool, motor_ids: Optional[list[int]] = None) -> None:
        if not self.connected or self._bus is None:
            return
        ids = motor_ids if motor_ids is not None else list(MOTOR_IDS)
        val = 1 if enabled else 0
        for mid in ids:
            self._bus.write_u8(mid, REG_TORQUE_ENABLE, val)

    def read_gripper_state(self) -> tuple[Optional[int], Optional[int]]:
        """Returns (load, current) for gripper (motor 6)."""
        if not self.connected or self._bus is None:
            return None, None
        raw_load = self._bus.read_u16(6, REG_PRESENT_LOAD)
        raw_curr = self._bus.read_u16(6, REG_PRESENT_CURRENT)
        # Convert to signed int16
        load = (raw_load - 65536 if raw_load is not None and raw_load > 32767
                else raw_load)
        curr = (raw_curr - 65536 if raw_curr is not None and raw_curr > 32767
                else raw_curr)
        return load, curr

    def read_gripper_load(self) -> Optional[int]:
        load, _ = self.read_gripper_state()
        return load

    def read_gripper_current(self) -> Optional[int]:
        _, curr = self.read_gripper_state()
        return curr

    def gripper_load_detected(self) -> bool:
        # The driver_node handles detection via its rolling current buffer.
        return False

    def load_home(self, path) -> dict[int, int]:
        import json
        from pathlib import Path
        try:
            data = json.loads(Path(path).read_text())
            return {int(k): int(v) for k, v in data.items()}
        except Exception:
            return {}

    def write_home(self, home: dict[int, int]) -> None:
        ticks = {
            name: home.get(mid, 2048)
            for name, mid in zip(MOTOR_NAMES, MOTOR_IDS)
        }
        self.write_ticks(ticks)

    def write_goal_velocity(self, motor_name: str, velocity: int) -> None:
        if not self.connected or self._bus is None:
            return
        try:
            idx = list(MOTOR_NAMES).index(motor_name)
            mid = MOTOR_IDS[idx]
            self._bus.write_u16(mid, REG_GOAL_VELOCITY, velocity)
        except (ValueError, Exception):
            pass

    def reset_gripper_current_buffer(self) -> None:
        pass  # no internal buffer — driver_node owns this
