"""Minimal Modbus RTU implementation (master + slave framing).

Written by hand (instead of using pymodbus for the serial side) so that the
gateway can tell *why* a poll failed -- CRC error vs. timeout vs. exception
vs. malformed frame -- which pymodbus collapses into a generic "no response".
The slave half mirrors firmware/vibration-node/src/modbus_rtu_slave.c line for
line so the emulator and the ESP32 firmware answer identically.

Frame (RTU):  [addr:1][fc:1][data:N][crc_lo:1][crc_hi:1]
CRC-16/MODBUS: poly 0xA001 (reflected 0x8005), init 0xFFFF, sent low byte first.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

# ---------------------------------------------------------------- CRC ------

def _make_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return table


_CRC_TABLE = _make_table()


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def add_crc(pdu_with_addr: bytes) -> bytes:
    return pdu_with_addr + struct.pack("<H", crc16(pdu_with_addr))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    return crc16(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]


def t35_seconds(baud: int) -> float:
    """Inter-frame silence (3.5 char times, 11 bits/char); fixed 1.75 ms above 19200 baud."""
    if baud > 19200:
        return 0.00175
    return 3.5 * 11.0 / baud


# ------------------------------------------------------------ requests -----

FC_READ_COILS = 0x01
FC_READ_HOLDING = 0x03
FC_READ_INPUT = 0x04
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10

EX_ILLEGAL_FUNCTION = 0x01
EX_ILLEGAL_ADDRESS = 0x02
EX_ILLEGAL_VALUE = 0x03


def build_read(addr: int, fc: int, start: int, count: int) -> bytes:
    return add_crc(struct.pack(">BBHH", addr, fc, start, count))


def build_write_single(addr: int, reg: int, value: int) -> bytes:
    return add_crc(struct.pack(">BBHH", addr, FC_WRITE_SINGLE, reg, value & 0xFFFF))


def build_write_multiple(addr: int, start: int, values: list[int]) -> bytes:
    body = struct.pack(">BBHHB", addr, FC_WRITE_MULTIPLE, start, len(values), 2 * len(values))
    body += b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
    return add_crc(body)


def expected_response_len(request: bytes) -> int:
    fc = request[1]
    if fc in (FC_READ_HOLDING, FC_READ_INPUT):
        count = struct.unpack(">H", request[4:6])[0]
        return 5 + 2 * count
    if fc in (FC_WRITE_SINGLE, FC_WRITE_MULTIPLE):
        return 8
    raise ValueError(f"unsupported fc {fc}")


# --------------------------------------------------------------- master -----

class PollResult(str, Enum):
    OK = "ok"
    TIMEOUT = "timeout"          # nothing (or too little) came back in time
    CRC_ERROR = "crc_error"      # right length, CRC mismatch
    EXCEPTION = "exception"      # slave answered with fc|0x80
    MALFORMED = "malformed"      # wrong address/fc/byte-count


@dataclass
class PollStats:
    polls: int = 0
    ok: int = 0
    timeout: int = 0
    crc_error: int = 0
    exception: int = 0
    malformed: int = 0
    latency_s: list[float] = field(default_factory=list, repr=False)

    def record(self, res: PollResult, dt: float) -> None:
        self.polls += 1
        setattr(self, res.value, getattr(self, res.value) + 1)
        if res is PollResult.OK and len(self.latency_s) < 200_000:
            self.latency_s.append(dt)

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in ("polls", "ok", "timeout", "crc_error", "exception", "malformed")}


class RtuMaster:
    """Blocking RTU master over any object with read(n)/write(b)/reset_input_buffer()
    (pyserial Serial, or a test double)."""

    def __init__(self, port, baud: int = 19200, timeout_s: float = 0.25):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.t35 = t35_seconds(baud)
        self.stats = PollStats()

    def _transact(self, request: bytes) -> tuple[PollResult, bytes]:
        exp = expected_response_len(request)
        self.port.reset_input_buffer()
        self.port.write(request)
        deadline = time.monotonic() + self.timeout_s
        buf = b""
        while time.monotonic() < deadline:
            chunk = self.port.read(max(1, exp - len(buf)))
            if chunk:
                buf += chunk
                # exception responses are 5 bytes
                if len(buf) >= 2 and buf[1] & 0x80 and len(buf) >= 5:
                    break
                if len(buf) >= exp:
                    break
        if len(buf) == 0:
            return PollResult.TIMEOUT, buf
        if len(buf) >= 2 and buf[1] & 0x80 and len(buf) >= 5:
            frame = buf[:5]
            if not check_crc(frame):
                return PollResult.CRC_ERROR, frame
            return PollResult.EXCEPTION, frame
        if len(buf) < exp:
            # partial frame: either a byte got lost or the slave timed out
            return PollResult.TIMEOUT, buf
        frame = buf[:exp]
        if not check_crc(frame):
            return PollResult.CRC_ERROR, frame
        if frame[0] != request[0] or frame[1] != request[1]:
            return PollResult.MALFORMED, frame
        if request[1] in (FC_READ_HOLDING, FC_READ_INPUT) and frame[2] != exp - 5:
            return PollResult.MALFORMED, frame
        return PollResult.OK, frame

    def read_registers(self, addr: int, fc: int, start: int, count: int) -> tuple[PollResult, list[int] | None]:
        req = build_read(addr, fc, start, count)
        t0 = time.monotonic()
        res, frame = self._transact(req)
        dt = time.monotonic() - t0
        self.stats.record(res, dt)
        if res is not PollResult.OK:
            # let the line go quiet before the next request (t3.5) and drop junk
            time.sleep(max(self.t35, 0.005))
            self.port.reset_input_buffer()
            return res, None
        values = list(struct.unpack(f">{count}H", frame[3:3 + 2 * count]))
        return res, values

    def write_register(self, addr: int, reg: int, value: int) -> PollResult:
        req = build_write_single(addr, reg, value)
        t0 = time.monotonic()
        res, _ = self._transact(req)
        self.stats.record(res, time.monotonic() - t0)
        return res


# ---------------------------------------------------------------- slave -----

class RtuSlave:
    """Pure request->response handler. Transport/framing (t3.5 silence) is done
    by the caller. Mirrors modbus_rtu_slave.c."""

    def __init__(self, unit_id: int,
                 read_input: Callable[[int, int], list[int] | None],
                 read_holding: Callable[[int, int], list[int] | None],
                 write_holding: Callable[[int, list[int]], bool]):
        self.unit_id = unit_id
        self.read_input = read_input
        self.read_holding = read_holding
        self.write_holding = write_holding
        self.frames_ok = 0
        self.frames_bad_crc = 0
        self.frames_other_unit = 0

    def _exception(self, fc: int, code: int) -> bytes:
        return add_crc(bytes([self.unit_id, fc | 0x80, code]))

    def handle(self, frame: bytes) -> bytes | None:
        """Return the response frame, or None when the slave must stay silent
        (bad CRC, other unit, broadcast)."""
        if len(frame) < 4 or not check_crc(frame):
            self.frames_bad_crc += 1
            return None  # spec: discard silently, master will time out
        if frame[0] != self.unit_id:
            self.frames_other_unit += 1
            return None
        self.frames_ok += 1
        fc = frame[1]
        if fc in (FC_READ_HOLDING, FC_READ_INPUT):
            if len(frame) != 8:
                return self._exception(fc, EX_ILLEGAL_VALUE)
            start, count = struct.unpack(">HH", frame[2:6])
            if not 1 <= count <= 125:
                return self._exception(fc, EX_ILLEGAL_VALUE)
            fn = self.read_input if fc == FC_READ_INPUT else self.read_holding
            vals = fn(start, count)
            if vals is None:
                return self._exception(fc, EX_ILLEGAL_ADDRESS)
            body = bytes([self.unit_id, fc, 2 * count]) + b"".join(struct.pack(">H", v & 0xFFFF) for v in vals)
            return add_crc(body)
        if fc == FC_WRITE_SINGLE:
            if len(frame) != 8:
                return self._exception(fc, EX_ILLEGAL_VALUE)
            reg, val = struct.unpack(">HH", frame[2:6])
            if not self.write_holding(reg, [val]):
                return self._exception(fc, EX_ILLEGAL_ADDRESS)
            return frame  # echo
        if fc == FC_WRITE_MULTIPLE:
            if len(frame) < 9:
                return self._exception(fc, EX_ILLEGAL_VALUE)
            start, count, nbytes = struct.unpack(">HHB", frame[2:7])
            if nbytes != 2 * count or len(frame) != 9 + nbytes or not 1 <= count <= 123:
                return self._exception(fc, EX_ILLEGAL_VALUE)
            vals = list(struct.unpack(f">{count}H", frame[7:7 + nbytes]))
            if not self.write_holding(start, vals):
                return self._exception(fc, EX_ILLEGAL_ADDRESS)
            return add_crc(frame[:6])
        return self._exception(fc, EX_ILLEGAL_FUNCTION)
