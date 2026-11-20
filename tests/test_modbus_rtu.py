import struct
from iiot.modbus_rtu import (RtuMaster, RtuSlave, PollResult, add_crc, build_read, check_crc, crc16,
                             FC_READ_INPUT, FC_READ_HOLDING, build_write_single, build_write_multiple, t35_seconds)


def test_crc_known_vector():
    # Classic example: read 10 holding regs from unit 1 at 0 -> CRC bytes C5 CD
    frame = build_read(1, 3, 0, 10)
    assert frame.hex() == "01030000000ac5cd"
    assert check_crc(frame)
    assert crc16(b"123456789") == 0x4B37   # CRC-16/MODBUS check value


def test_t35():
    assert abs(t35_seconds(9600) - 0.00401) < 1e-4
    assert t35_seconds(115200) == 0.00175


def make_slave():
    ir = list(range(100, 116))
    hr = [450, 280, 3, 10]
    def rd(start, count, regs):
        return None if start + count > len(regs) else regs[start:start + count]
    def wr(start, vals):
        if start + len(vals) > len(hr) or 3 in range(start, start + len(vals)):
            return False
        hr[start:start + len(vals)] = vals
        return True
    return RtuSlave(10, lambda s, c: rd(s, c, ir), lambda s, c: rd(s, c, hr), wr), hr


def test_slave_read_input():
    s, _ = make_slave()
    resp = s.handle(build_read(10, FC_READ_INPUT, 2, 3))
    assert check_crc(resp) and resp[:3] == bytes([10, 4, 6])
    assert struct.unpack(">3H", resp[3:9]) == (102, 103, 104)


def test_slave_exceptions_and_silence():
    s, _ = make_slave()
    r = s.handle(build_read(10, FC_READ_INPUT, 15, 2))       # past the end
    assert r[1] == 0x84 and r[2] == 0x02
    r = s.handle(add_crc(bytes([10, 0x2B, 0, 0])))           # unsupported fc
    assert r[1] == 0xAB and r[2] == 0x01
    bad = bytearray(build_read(10, FC_READ_INPUT, 0, 1)); bad[3] ^= 0x01
    assert s.handle(bytes(bad)) is None and s.frames_bad_crc == 1   # CRC error -> silent
    assert s.handle(build_read(11, FC_READ_INPUT, 0, 1)) is None    # other unit -> silent


def test_slave_writes():
    s, hr = make_slave()
    req = build_write_single(10, 0, 500)
    assert s.handle(req) == req and hr[0] == 500
    r = s.handle(build_write_single(10, 3, 1))                # read-only unit id
    assert r[1] == 0x86
    r = s.handle(build_write_multiple(10, 0, [400, 300]))
    assert check_crc(r) and r[1] == 0x10 and hr[:2] == [400, 300]


class LoopPort:
    """In-memory serial port wired to a slave, with optional corruption."""
    def __init__(self, slave, corrupt=None):
        self.slave, self.corrupt, self.rx = slave, corrupt, b""
    def reset_input_buffer(self): self.rx = b""
    def write(self, data):
        resp = self.slave.handle(data)
        if resp:
            self.rx = self.corrupt(resp) if self.corrupt else resp
    def read(self, n):
        out, self.rx = self.rx[:n], self.rx[n:]
        return out


def test_master_classifies_results():
    s, _ = make_slave()
    m = RtuMaster(LoopPort(s), timeout_s=0.01)
    res, regs = m.read_registers(10, FC_READ_INPUT, 0, 16)
    assert res is PollResult.OK and regs[0] == 100
    m2 = RtuMaster(LoopPort(s, corrupt=lambda b: b[:5] + bytes([b[5] ^ 0x40]) + b[6:]), timeout_s=0.01)
    assert m2.read_registers(10, FC_READ_INPUT, 0, 16)[0] is PollResult.CRC_ERROR
    m3 = RtuMaster(LoopPort(s), timeout_s=0.01)
    assert m3.read_registers(99, FC_READ_INPUT, 0, 16)[0] is PollResult.TIMEOUT
    assert m3.read_registers(10, FC_READ_HOLDING, 0, 16)[0] is PollResult.EXCEPTION
    assert m3.stats.timeout == 1 and m3.stats.exception == 1
