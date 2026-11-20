"""PID step-response measurement on the RUNNING OpenPLC loop (SIMULATED plant).

Writes the setpoint into the PLC (HR 1024 = %MW0) over Modbus TCP, samples the
PLC's level tag (HR 0) at 20 Hz, and computes settling time, overshoot, rise
time and the steady-state band for each step.

    .venv/bin/python bench/step_response.py --cycles 3 --low 50 --high 70
"""
import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pymodbus.client import ModbusTcpClient  # noqa: E402

from bench.common import RESULTS, http_json, save  # noqa: E402
from iiot.pid import step_metrics  # noqa: E402

HR_SP, HR_CMD = 1024, 1025


def read_level(c):
    r = c.read_holding_registers(0, count=3, slave=1)
    return r.registers[0] / 100.0, r.registers[1] / 100.0, r.registers[2] / 100.0


def record(c, seconds, hz=20.0):
    t, y, u = [], [], []
    t0 = time.monotonic()
    nxt = t0
    while time.monotonic() - t0 < seconds:
        lv, _sp, spd = read_level(c)
        t.append(time.monotonic() - t0); y.append(lv); u.append(spd)
        nxt += 1 / hz
        time.sleep(max(0, nxt - time.monotonic()))
    return t, y, u


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5502)
    ap.add_argument("--low", type=float, default=50.0)
    ap.add_argument("--high", type=float, default=70.0)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--hold", type=float, default=120.0, help="seconds recorded per step")
    a = ap.parse_args()

    http_json("http://localhost:8081/control", {"valve_pct": 50, "dry_run": False, "estop": False})
    c = ModbusTcpClient(a.host, port=a.port)
    c.connect()
    c.write_register(HR_CMD, 0x0004, slave=1)            # RESET latched trips
    time.sleep(0.5)
    c.write_register(HR_CMD, 0x0001, slave=1)            # START
    c.write_register(HR_SP, int(a.low * 100), slave=1)
    print(f"pre-settling at {a.low}% ...")
    record(c, 90)

    os.makedirs(RESULTS, exist_ok=True)
    steps, all_rows = [], []
    for cyc in range(a.cycles):
        for y0, y1 in ((a.low, a.high), (a.high, a.low)):
            c.write_register(HR_SP, int(y1 * 100), slave=1)
            t, y, u = record(c, a.hold)
            m = step_metrics(t, y, 0.0, y0, y1, band_frac=0.02, ss_window_s=30.0)
            m_span = step_metrics(t, y, 0.0, y0, y1, band_frac=2.0 / abs(y1 - y0), ss_window_s=30.0)
            m["settling_time_2pct_span_s"] = m_span["settling_time_s"]
            m.update({"cycle": cyc, "from": y0, "to": y1, "u_max": max(u)})
            steps.append(m)
            all_rows += [(cyc, y0, y1, ti, yi, ui) for ti, yi, ui in zip(t, y, u)]
            print(f"step {y0}->{y1}: settle(2% of step)={m['settling_time_s']:.1f}s "
                  f"settle(+/-2% span)={m['settling_time_2pct_span_s']:.1f}s overshoot={m['overshoot_pct']:.2f}% "
                  f"ss=[{m['ss_min']:+.2f},{m['ss_max']:+.2f}] std={m['ss_std']:.3f}")
    c.write_register(HR_SP, int(a.low * 100), slave=1)
    with open(os.path.join(RESULTS, "step_response.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cycle", "from", "to", "t_s", "level_pct", "speed_pct"])
        w.writerows(all_rows)
    save("step_response.json", {
        "method": "SP written to OpenPLC %MW0 via Modbus TCP; PLC level tag sampled at 20 Hz; "
                  "settling = last exit from band (2% of step size, i.e. +/-0.4 % level, and +/-2 % of span); "
                  "overshoot as % of step; steady band over last 30 s of each 120 s step. SIMULATED tank.",
        "gains": {"kp": 15.0, "ti_s": 12.0, "td_s": 0.0}, "task_ms": 50, "steps": steps,
    })
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
        for cyc, y0, y1 in {(r[0], r[1], r[2]) for r in all_rows if r[0] == 0}:
            rows = [r for r in all_rows if r[0] == cyc and r[1] == y0]
            ax[0].plot([r[3] for r in rows], [r[4] for r in rows], label=f"{y0:.0f}->{y1:.0f} %")
            ax[1].plot([r[3] for r in rows], [r[5] for r in rows])
        ax[0].set_ylabel("level % (simulated)"); ax[0].legend(); ax[0].grid(alpha=.3)
        ax[1].set_ylabel("pump speed ref %"); ax[1].set_xlabel("s after setpoint step"); ax[1].grid(alpha=.3)
        fig.suptitle("OpenPLC ST PID, Kp=15 Ti=12 s, 50 ms task -- software-in-the-loop")
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(RESULTS), "docs", "img", "step_response.png"), dpi=110)
    except Exception as e:  # plotting is optional
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
