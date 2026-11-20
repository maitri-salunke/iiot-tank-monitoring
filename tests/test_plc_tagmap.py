"""The gateway decoder (iiot/telemetry.py) and the PLC program must agree on the tag map."""
import os
import re

from iiot.telemetry import COIL_NAMES, PLC_COIL_COUNT, PLC_HR_COUNT, PLC_HR_MAP

ST = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "plc", "tank_control.st")).read()
CODE = re.sub(r"\(\*.*?\*\)", "", ST, flags=re.S)      # strip comments


def test_holding_registers_contiguous_and_typed():
    decl = {int(a): t for a, t in re.findall(r"\w+\s+AT\s+%QW(\d+)\s*:\s*(\w+)", CODE) if int(a) < 100}
    assert sorted(decl) == list(range(PLC_HR_COUNT)), "gateway block must cover exactly %QW0..N-1"
    for reg, name, _scale, signed in PLC_HR_MAP:
        assert decl[reg] == ("INT" if signed else "UINT"), (reg, name, decl[reg])
    mapped = {r for r, *_ in PLC_HR_MAP} | {7, 8, 27, 28}   # the two 32-bit hi/lo pairs
    assert mapped == set(range(PLC_HR_COUNT))


def test_coils_contiguous():
    bits = sorted(int(b) * 8 + int(i) for b, i in re.findall(r"\w+\s+AT\s+%QX(\d+)\.(\d+)\s*:\s*BOOL", CODE)
                  if int(b) < 100)
    assert bits == list(range(PLC_COIL_COUNT)) and len(COIL_NAMES) == PLC_COIL_COUNT


def test_scan_constant_matches_task_interval():
    scan = int(re.search(r"SCAN_MS\s*:\s*UINT\s*:=\s*(\d+)", CODE).group(1))
    task = int(re.search(r"TASK\s+\w+\s*\(\s*INTERVAL\s*:=\s*T#(\d+)ms", CODE).group(1))
    assert scan == task


def test_request_sizes_fit_one_pdu():
    assert PLC_HR_COUNT <= 125 and PLC_COIL_COUNT <= 2000      # FC03 / FC01 limits
