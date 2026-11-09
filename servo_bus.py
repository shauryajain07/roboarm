"""Feetech / Waveshare ST3215 (SMS_STS) bus driver for the ENGG1101 robot arm.

Byte-for-byte compatible with the course web controller
(activelearning-web/app/engg1101/robotic-arm-controller/webserial/scservo.ts):

    packet   = FF FF id len instr params... chk
    len      = len(params) + 2
    chk      = ~(sum(bytes[2:-1])) & 0xFF

Additions over the web tool:
  * sync_write_positions(): moves all 5 joints with ONE packet (no 5 round-trips)
  * read_load(): used to check that the gripper is actually holding a block
  * MockTransport: a simulated arm so every sequence can be tested without hardware
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

# --- instructions -----------------------------------------------------------
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

# --- SMS_STS registers (same numbers as scservo.ts) ---------------------------
ADDR_TORQUE_ENABLE = 40   # 1 on, 0 off, 128 = "set current position as middle (2048)"
ADDR_ACC = 41             # goal block: acc, posL, posH, timeL, timeH, spdL, spdH
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
ADDR_MOVING = 66

POS_MIN, POS_MAX = 0, 4095
SPEED_MAX = 4095
ACC_MAX = 255

# status-reply error bits (same as the official scservo_sdk). The servo still did the
# READ/WRITE - these are warnings that stay set while the condition lasts.
FAULT_BITS = {0x01: "voltage", 0x02: "angle limit", 0x04: "overheat",
              0x08: "over-current", 0x20: "overload"}
STRAIN_BITS = 0x04 | 0x08 | 0x20     # the motor is fighting something


def describe_fault(err: int) -> str:
    return ", ".join(n for b, n in FAULT_BITS.items() if err & b) or f"error 0x{err:02x}"


DEFAULT_TIMEOUT_S = 0.25   # same as the web tool
PING_TIMEOUT_S = 0.5
FLUSH_AFTER_TIMEOUT_S = 0.05


class ServoError(Exception):
    pass


class ServoTimeout(ServoError):
    pass


def checksum(body) -> int:
    return (~sum(body)) & 0xFF


def build_packet(servo_id: int, instr: int, params=()) -> bytes:
    params = list(params)
    body = [servo_id & 0xFF, len(params) + 2, instr, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def _check_range(name: str, v, lo: int, hi: int) -> int:
    v = int(round(v))
    if not lo <= v <= hi:
        raise ValueError(f"{name} {v} outside {lo}-{hi}")
    return v


def goal_block(position: int, speed: int, acc: int) -> list[int]:
    """7 data bytes written at ADDR_ACC: acc, pos(L,H), time(L,H)=0, speed(L,H)."""
    p = _check_range("Goal position", position, POS_MIN, POS_MAX)
    s = _check_range("Speed", speed, 0, SPEED_MAX)
    a = _check_range("Acceleration", acc, 0, ACC_MAX)
    return [a, p & 0xFF, p >> 8, 0, 0, s & 0xFF, s >> 8]


def build_ping_packet(servo_id: int) -> bytes:
    return build_packet(servo_id, INST_PING)


def build_write_position_packet(servo_id: int, position: int, speed: int, acc: int) -> bytes:
    return build_packet(servo_id, INST_WRITE, [ADDR_ACC, *goal_block(position, speed, acc)])


def build_sync_write_positions_packet(targets: dict[int, tuple[int, int, int]]) -> bytes:
    """targets = {id: (position, speed, acc)} -> one broadcast packet, no reply."""
    params = [ADDR_ACC, 7]
    for sid, (pos, spd, acc) in sorted(targets.items()):
        params += [sid, *goal_block(pos, spd, acc)]
    return build_packet(BROADCAST_ID, INST_SYNC_WRITE, params)


def parse_status(buf: bytearray):
    """Pull one status packet out of buf. Returns (packet|None, remaining).
    Resyncs on FF FF and drops packets with a bad checksum (like scservo.ts)."""
    while True:
        i = buf.find(b"\xff\xff")
        if i < 0:
            return None, bytearray(buf[-1:]) if buf[-1:] == b"\xff" else bytearray()
        buf = buf[i:]
        if len(buf) < 6:
            return None, buf
        sid, length = buf[2], buf[3]
        if sid == 0xFF or length < 2:
            buf = buf[1:]
            continue
        total = length + 4
        if len(buf) < total:
            return None, buf
        pkt = buf[:total]
        if checksum(pkt[2:-1]) != pkt[-1]:
            buf = buf[1:]
            continue
        return {"id": sid, "instr_or_err": pkt[4], "error": pkt[4],
                "params": bytes(pkt[5:-1]), "raw": bytes(pkt)}, buf[total:]


def open_serial(port: str, baud: int, settle_s: float = 1.5):
    """Open the port WITHOUT pulsing DTR/RTS. On ESP32 / Arduino-style driver boards
    those lines are wired to reset, so a normal open reboots the board and the first
    pings are lost while it boots. We hold them low, then wait and drop boot noise."""
    import serial  # pyserial
    ser = serial.Serial()
    ser.port, ser.baudrate = port, baud
    ser.timeout, ser.write_timeout = 0.01, 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    if settle_s:
        time.sleep(settle_s)
        ser.reset_input_buffer()
    return ser


# ============================================================================
class ServoBus:
    """One-request-in-flight serial client. Thread-safe."""

    def __init__(self, port: str, baud: int = 115200, write_replies: bool = True,
                 transport=None, settle_s: float = 1.5):
        if transport is None:
            transport = open_serial(port, baud, settle_s)
        self.ser = transport
        self.write_replies = write_replies  # web tool waits for a status after every WRITE
        self._lock = threading.Lock()
        self._rx = bytearray()
        self.faults: dict[int, int] = {}    # servo id -> error bits in its latest reply (0 = ok)

    def close(self):
        self.ser.close()

    # -- low level --------------------------------------------------------
    def _flush(self):
        self._rx.clear()
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def _request(self, pkt: bytes, expect_id, n_params, timeout: float = DEFAULT_TIMEOUT_S):
        with self._lock:
            self._flush()
            self.ser.write(pkt)
            if expect_id is None:
                return None
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                # take whatever has arrived (at least 1 byte) - read(64) would sit out the
                # whole 10 ms port timeout on every reply, capping recording at ~16 Hz
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                self._rx += chunk
                while True:
                    status, self._rx = parse_status(self._rx)
                    if status is None:
                        break
                    if status["raw"] == pkt:          # our own echo on a half-duplex bus
                        continue
                    if status["id"] != expect_id:
                        continue
                    if n_params is not None and len(status["params"]) != n_params:
                        continue
                    # a set error bit is a warning, not a failed command: record it for
                    # the GUI instead of raising (raising made every read fail while
                    # e.g. the gripper was overloaded, so the whole readout died)
                    self.faults[expect_id] = status["error"]
                    return status
            time.sleep(FLUSH_AFTER_TIMEOUT_S)
            self._flush()
            raise ServoTimeout(f"no reply from servo {expect_id}")

    # -- commands ---------------------------------------------------------
    def ping(self, sid: int) -> bool:
        try:
            self._request(build_ping_packet(sid), sid, 0, PING_TIMEOUT_S)
            return True
        except ServoTimeout:
            return False

    def read(self, sid: int, addr: int, n: int) -> bytes:
        return self._request(build_packet(sid, INST_READ, [addr, n]), sid, n)["params"]

    def write(self, sid: int, addr: int, data):
        self._request(build_packet(sid, INST_WRITE, [addr, *data]),
                      sid if self.write_replies else None, 0)

    def read_position(self, sid: int) -> int:
        b = self.read(sid, ADDR_PRESENT_POSITION, 2)
        return b[0] | (b[1] << 8)

    def read_load(self, sid: int) -> int:
        """Load magnitude 0..1000 (0.1 % units); sign = direction bit 10."""
        b = self.read(sid, ADDR_PRESENT_LOAD, 2)
        raw = b[0] | (b[1] << 8)
        mag = raw & 0x3FF
        return -mag if raw & 0x400 else mag

    def is_moving(self, sid: int) -> bool:
        return bool(self.read(sid, ADDR_MOVING, 1)[0])

    def set_torque(self, sid: int, on: bool):
        self.write(sid, ADDR_TORQUE_ENABLE, [1 if on else 0])

    def write_position(self, sid: int, pos: int, speed: int, acc: int):
        self.write(sid, ADDR_ACC, goal_block(pos, speed, acc))

    def sync_read_positions(self, ids, timeout: float = DEFAULT_TIMEOUT_S) -> dict[int, int]:
        """All positions with ONE request (SYNC READ): every servo answers in turn, so a full
        5-joint sample takes one round trip instead of five. Raises ServoTimeout if any
        servo is missing (e.g. the bridge doesn't pass SYNC READ) - callers fall back."""
        ids = list(ids)
        pkt = build_packet(BROADCAST_ID, INST_SYNC_READ, [ADDR_PRESENT_POSITION, 2, *ids])
        out: dict[int, int] = {}
        with self._lock:
            self._flush()
            self.ser.write(pkt)
            deadline = time.monotonic() + timeout
            while len(out) < len(ids) and time.monotonic() < deadline:
                # take whatever has arrived (at least 1 byte) - read(64) would sit out the
                # whole 10 ms port timeout on every reply, capping recording at ~16 Hz
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                self._rx += chunk
                while True:
                    status, self._rx = parse_status(self._rx)
                    if status is None:
                        break
                    if status["raw"] == pkt or status["id"] not in ids or len(status["params"]) != 2:
                        continue
                    self.faults[status["id"]] = status["error"]
                    b = status["params"]
                    out[status["id"]] = b[0] | (b[1] << 8)
            if len(out) < len(ids):
                time.sleep(FLUSH_AFTER_TIMEOUT_S)
                self._flush()
                raise ServoTimeout(f"sync read: no reply from {sorted(set(ids) - set(out))}")
        return out

    def sync_write_positions(self, targets: dict[int, tuple[int, int, int]]):
        """Broadcast — servos never reply to SYNC WRITE."""
        self._request(build_sync_write_positions_packet(targets), None, None)


# ============================================================================
@dataclass
class _SimServo:
    pos: float = 2048
    goal: float = 2048
    speed: float = 300
    torque: bool = False
    load: int = 0
    err: int = 0          # error bits sent in every reply (tests simulate overload with it)


class MockTransport:
    """Pretends to be a serial port with 5 ST3215s on it. Positions move toward
    goals at `speed` ticks/s (scaled by time_scale) so arrival logic is exercised."""

    def __init__(self, ids=(1, 2, 3, 4, 5), time_scale: float = 50.0):
        self.servos = {i: _SimServo() for i in ids}
        self.time_scale = time_scale
        self._t = time.monotonic()
        self._out = bytearray()
        self.log: list[bytes] = []
        self.gripper_id = 5
        self.gripper_block_at = None   # set to a tick value to simulate a block in the jaws

    def _tick(self):
        now = time.monotonic()
        dt = (now - self._t) * self.time_scale
        self._t = now
        for sid, s in self.servos.items():
            if not s.torque:
                continue
            step = max(s.speed, 1) * dt
            d = s.goal - s.pos
            s.pos = s.goal if abs(d) <= step else s.pos + step * (1 if d > 0 else -1)
            s.load = 0
            if sid == self.gripper_id and self.gripper_block_at is not None:
                if s.goal > self.gripper_block_at and s.pos >= self.gripper_block_at:
                    s.pos, s.load = self.gripper_block_at, 400   # jaws stall on block

    def _reply(self, sid, params=b""):
        body = [sid, len(params) + 2, self.servos[sid].err if sid in self.servos else 0, *params]
        self._out += bytes([0xFF, 0xFF, *body, checksum(body)])

    def write(self, pkt: bytes):
        self._tick()
        self.log.append(bytes(pkt))
        assert pkt[:2] == b"\xff\xff" and checksum(pkt[2:-1]) == pkt[-1], "bad packet"
        sid, instr, p = pkt[2], pkt[4], pkt[5:-1]
        if instr == INST_SYNC_WRITE:
            addr, n = p[0], p[1]
            for k in range(2, len(p), n + 1):
                self._apply_write(p[k], addr, p[k + 1:k + 1 + n])
            return
        if instr == INST_SYNC_READ:
            addr, n = p[0], p[1]
            for i in p[2:]:
                if i in self.servos:
                    pos = int(round(self.servos[i].pos))
                    mem = {56: pos & 0xFF, 57: pos >> 8}
                    self._reply(i, bytes(mem.get(addr + k, 0) for k in range(n)))
            return
        if sid not in self.servos:
            return
        s = self.servos[sid]
        if instr == INST_PING:
            self._reply(sid)
        elif instr == INST_WRITE:
            self._apply_write(sid, p[0], p[1:])
            self._reply(sid)
        elif instr == INST_READ:
            addr, n = p[0], p[1]
            pos = int(round(s.pos))
            mem = {56: pos & 0xFF, 57: pos >> 8,
                   60: abs(s.load) & 0xFF, 61: (abs(s.load) >> 8) & 0x3,
                   66: int(abs(s.goal - s.pos) > 1 and s.torque)}
            self._reply(sid, bytes(mem.get(addr + i, 0) for i in range(n)))

    def _apply_write(self, sid, addr, data):
        s = self.servos.get(sid)
        if s is None:
            return
        if addr == ADDR_TORQUE_ENABLE:
            if data[0] == 128:
                s.pos = s.goal = 2048
            else:
                s.torque = bool(data[0])
                s.goal = s.pos
        elif addr == ADDR_ACC and s.torque:
            s.goal = data[1] | (data[2] << 8)
            s.speed = data[5] | (data[6] << 8)

    @property
    def in_waiting(self) -> int:
        return len(self._out)

    def read(self, n: int) -> bytes:
        self._tick()
        out, self._out = bytes(self._out[:n]), self._out[n:]
        if not out:
            time.sleep(0.0005)
        return out

    def reset_input_buffer(self):
        pass

    def close(self):
        pass

    def hand_move(self, positions: dict[int, int]):
        """Test helper: physically push joints while torque is off."""
        for sid, v in positions.items():
            self.servos[sid].pos = self.servos[sid].goal = v


def mock_bus(**kw):
    t = MockTransport(**kw)
    return ServoBus("mock", transport=t), t
