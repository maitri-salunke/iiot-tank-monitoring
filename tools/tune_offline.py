"""Offline PI(D) tuning sweep on the SIMULATED tank (Python port of the ST PID).

Models the 50 ms PLC task plus one extra scan of I/O delay (OpenPLC polls the
plant slave every 50 ms). The gains chosen here are only a starting point; the
reported step-response numbers come from bench/step_response.py on the real
OpenPLC loop.
"""
import itertools, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from iiot.plant_model import TankModel, TankState
from iiot.pid import FbPid, step_metrics


def simulate(kp, ti, td=0.0, sp0=50.0, sp1=70.0, valve=50.0, t_end=120.0, seed=1):
    m = TankModel(state=TankState(h_m=sp0 / 100, valve_pct=valve), seed=seed)
    pid = FbPid()
    ts, dt = 0.05, 0.01
    # pre-settle at sp0
    t, out, delayed = 0.0, 0.0, [sp0]
    m.s.run_cmd = True
    log_t, log_y = [], []
    steps = int((t_end + 60) / dt)
    sp = sp0
    for i in range(steps):
        if i % int(ts / dt) == 0:
            y_meas = delayed.pop(0)
            delayed.append(m.level_pct_meas())
            sp = sp1 if t >= 60.0 else sp0
            out = pid(True, y_meas, sp, kp, ti, td, ts, track=out)
            m.s.speed_ref_pct = out
            m.set_heartbeat(i)
            if t >= 60.0:
                log_t.append(t); log_y.append(m.level_pct_meas())
        m.step(dt)
        t += dt
    return step_metrics(log_t, log_y, 60.0, sp0, sp1)


if __name__ == "__main__":
    best = []
    for kp, ti in itertools.product([5, 8, 10, 15, 20, 30], [20, 40, 60, 100, 150]):
        r = simulate(kp, ti)
        best.append((r["settling_time_s"], r["overshoot_pct"], kp, ti, r))
    for st, ov, kp, ti, r in sorted(best, key=lambda x: (x[1] > 5, x[0]))[:12]:
        print(f"Kp={kp:5.1f} Ti={ti:5.0f}  settle(2%)={st:6.1f}s  overshoot={ov:5.2f}%  ss_std={r['ss_std']:.3f}")
