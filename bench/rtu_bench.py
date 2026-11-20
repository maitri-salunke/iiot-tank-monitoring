"""Modbus RTU poll benchmark -- runs INSIDE the gateway container
(it needs the gateway's serial port /tmp/ttyRS485):

    docker exec iiot-gateway python bench/rtu_bench.py --n 10000 --ber 0
    docker exec iiot-gateway python bench/rtu_bench.py --n 10000 --ber 1e-4

Pauses the gateway's own RTU polling (single master on the bus), sets the
simulated wire's bit-error rate on the vibration emulator, then issues N
back-to-back FC04 reads of the full input-register block (IR_COUNT registers) and classifies every
result (ok / crc_error / timeout / exception / malformed).
"""
import argparse
import json
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import serial  # noqa: E402

from iiot import vib_regmap as R  # noqa: E402
from iiot.modbus_rtu import FC_READ_INPUT, RtuMaster  # noqa: E402


def post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=5).read())


def pct(v, p):
    s = sorted(v)
    return s[max(0, min(len(s) - 1, int(round(p / 100 * len(s) + 0.5)) - 1))] if s else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10000)
    ap.add_argument("--ber", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=0.2)   # same as the gateway (RTU_TIMEOUT_S)
    ap.add_argument("--port", default="/tmp/ttyRS485")
    ap.add_argument("--vib", default="http://vibnode:8082/control")
    a = ap.parse_args()

    post("http://127.0.0.1:9100/", {"paused": True})
    try:
        before = post(a.vib, {"ber": a.ber})
        ser = serial.Serial(a.port, R.BAUD, timeout=0.01)
        m = RtuMaster(ser, baud=R.BAUD, timeout_s=a.timeout)
        time.sleep(0.2)
        ser.reset_input_buffer()
        t0 = time.monotonic()
        for _ in range(a.n):
            m.read_registers(R.UNIT_ID, FC_READ_INPUT, 0, R.IR_COUNT)
        dt = time.monotonic() - t0
        after = post(a.vib, {"ber": 0.0})
    finally:
        post("http://127.0.0.1:9100/", {"paused": False})
    st = m.stats.as_dict()
    lat = [x * 1000 for x in m.stats.latency_s]
    req_bytes, resp_bytes = 8, 5 + 2 * R.IR_COUNT
    out = {
        "n": a.n, "ber": a.ber, "baud": R.BAUD, "regs": R.IR_COUNT, "timeout_s": a.timeout, "duration_s": round(dt, 2),
        "polls_per_s": round(a.n / dt, 2), "results": st,
        "errors_per_10k": round((a.n - st["ok"]) / a.n * 10000, 1),
        "crc_per_10k": round(st["crc_error"] / a.n * 10000, 1),
        "timeout_per_10k": round(st["timeout"] / a.n * 10000, 1),
        "ok_latency_ms": {"p50": round(pct(lat, 50), 2), "p99": round(pct(lat, 99), 2), "max": round(max(lat), 2) if lat else None},
        "wire_time_ms_theoretical": round((req_bytes + resp_bytes) * 10 / R.BAUD * 1000, 2),
        "wire_bytes_corrupted": after["wire"]["bytes_corrupted"] - before["wire"]["bytes_corrupted"],
        "slave_bad_crc_frames": after["slave"]["bad_crc"] - before["slave"]["bad_crc"],
    }
    print(json.dumps(out))


if __name__ == "__main__":
    main()
