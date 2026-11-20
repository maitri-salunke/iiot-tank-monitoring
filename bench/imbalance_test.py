"""Imbalance-fault detection test (SIMULATED vibration signal).

N trials. In each, an imbalance of severity d (extra 1x velocity, mm/s RMS,
drawn from a seeded uniform distribution that deliberately spans values below
and above the 2.8 mm/s alarm level) is injected into the emulator for HOLD s,
then removed for REST s. Detection is judged end-to-end from InfluxDB: the
node's debounced `imbalance` status bit, as stored by the pipeline.

    .venv/bin/python bench/imbalance_test.py --trials 20
"""
import argparse
import datetime
import os
import random
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bench.common import flux, http_json, save  # noqa: E402

VIB = "http://localhost:8082/control"
BASE_1X = 1.2           # healthy 1x velocity in the synthetic signal
THRESH = 2.8            # HR1 default


def rfc3339(s):
    base, _, frac = s.rstrip("Z").partition(".")
    return datetime.datetime.fromisoformat(base + "+00:00").timestamp() + (float("0." + frac) if frac else 0.0)


def alarm_rows(t0, t1):
    rows = flux(f'''from(bucket: "iiot") |> range(start: {int(t0)}, stop: {int(t1) + 1})
      |> filter(fn: (r) => r._measurement == "vibration" and (r._field == "imbalance" or r._field == "vel_1x_mms"))
      |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")''')
    out = []
    for r in rows:
        out.append((rfc3339(r["_time"]), r["imbalance"] == "true", float(r["vel_1x_mms"])))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--hold", type=float, default=12.0)
    ap.add_argument("--rest", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--lo", type=float, default=0.5)
    ap.add_argument("--hi", type=float, default=4.0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    sev = [round(rng.uniform(a.lo, a.hi), 2) for _ in range(a.trials)]
    http_json(VIB, {"imbalance_mms": 0.0})
    time.sleep(a.rest)
    trials = []
    for i, d in enumerate(sev):
        t_inj = time.time()
        http_json(VIB, {"imbalance_mms": d})
        time.sleep(a.hold)
        t_clr = time.time()
        http_json(VIB, {"imbalance_mms": 0.0})
        time.sleep(a.rest)
        trials.append((d, t_inj, t_clr, time.time()))
        print(f"trial {i + 1}/{a.trials}: +{d} mm/s", flush=True)
    time.sleep(5)
    res_trials = []
    fp_windows = 0
    healthy_samples = 0
    for d, t_inj, t_clr, t_end in trials:
        rows = alarm_rows(t_inj, t_end)
        during = [r for r in rows if t_inj <= r[0] <= t_clr + 1.0]
        first = next((r[0] for r in during if r[1]), None)
        healthy = [r for r in rows if r[0] >= t_clr + 6.0]          # after clear-debounce
        fp = sum(1 for r in healthy if r[1])
        fp_windows += fp
        healthy_samples += len(healthy)
        res_trials.append({"severity_mms": d, "expected_1x_mms": round(BASE_1X + d, 2),
                           "should_alarm": BASE_1X + d > THRESH, "detected": first is not None,
                           "time_to_detect_s": round(first - t_inj, 2) if first else None,
                           "max_1x_seen": max((r[2] for r in during), default=None),
                           "false_alarm_samples_after": fp})
    above = [t for t in res_trials if t["should_alarm"]]
    below = [t for t in res_trials if not t["should_alarm"]]
    ttd = sorted(t["time_to_detect_s"] for t in res_trials if t["detected"])
    res = {
        "trials": a.trials, "hold_s": a.hold, "rest_s": a.rest, "seed": a.seed, "severity_range_mms": [a.lo, a.hi],
        "detected_total": sum(t["detected"] for t in res_trials),
        "above_threshold": len(above), "detected_above_threshold": sum(t["detected"] for t in above),
        "below_threshold": len(below), "detected_below_threshold": sum(t["detected"] for t in below),
        "time_to_detect_s": {"min": ttd[0] if ttd else None, "median": ttd[len(ttd) // 2] if ttd else None,
                             "max": ttd[-1] if ttd else None},
        "false_alarm_samples": fp_windows, "healthy_samples_checked": healthy_samples,
        "method": "seeded severities injected into the vibration emulator; detection = debounced node imbalance "
                  "bit (1x > 2.8 mm/s and 1x >= 0.7*overall, 3 windows) observed in InfluxDB; SIMULATED signal",
        "detail": res_trials,
    }
    print({k: v for k, v in res.items() if k != "detail"})
    save("imbalance.json", res)


if __name__ == "__main__":
    main()
