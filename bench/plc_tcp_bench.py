"""Modbus TCP poll benchmark against the OpenPLC server (host port 5502).

Same request the gateway issues each cycle: FC03 HR 0..15 + FC01 coils 0..9.
    .venv/bin/python bench/plc_tcp_bench.py --n 10000
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pymodbus.client import ModbusTcpClient  # noqa: E402

from bench.common import pct, save  # noqa: E402


from iiot.telemetry import PLC_COIL_COUNT, PLC_COIL_START, PLC_HR_COUNT, PLC_HR_START  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10000)
    a = ap.parse_args()
    c = ModbusTcpClient("localhost", port=5502, timeout=0.5, retries=0)
    c.connect()
    ok = err = exc = 0
    lat = []
    t0 = time.monotonic()
    for _ in range(a.n):
        t = time.monotonic()
        try:
            r1 = c.read_holding_registers(PLC_HR_START, count=PLC_HR_COUNT, slave=1)
            r2 = c.read_coils(PLC_COIL_START, count=PLC_COIL_COUNT, slave=1)
            if r1.isError() or r2.isError():
                exc += 1
            else:
                ok += 1
                lat.append((time.monotonic() - t) * 1000)
        except Exception:
            err += 1
            c.close(); c.connect()
    dt = time.monotonic() - t0
    res = {"n": a.n, "ok": ok, "timeouts_or_conn_errors": err, "exceptions": exc,
           "cycles_per_s": round(a.n / dt, 1), "cycle_latency_ms": {"p50": round(pct(lat, 50), 2),
                                                                   "p99": round(pct(lat, 99), 2), "max": round(max(lat), 2)},
           "hr": PLC_HR_COUNT, "coils": PLC_COIL_COUNT,
           "method": f"back-to-back FC03({PLC_HR_COUNT} regs)+FC01({PLC_COIL_COUNT} coils) from macOS host -> "
                     "Docker Desktop port 5502 -> OpenPLC"}
    print(res)
    save("plc_tcp_bench.json", res)


if __name__ == "__main__":
    main()
