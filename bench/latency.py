"""End-to-end latency, read back from InfluxDB.

For every telemetry message the ingestor writes, after InfluxDB acknowledged
the write, a `pipeline_latency` point with
    e2e_ms     = write-ack time - plant sample time (stamped by the SIMULATED plant I/O,
                 carried through the OpenPLC registers %IW104/105 -> %QW7/8)
    gw_ms      = write-ack time - gateway poll time
    src_age_ms = gateway poll time - plant sample time
All containers share the Docker VM clock, so no cross-host clock sync error.
Grafana's own refresh interval (5 s) is NOT included -- "dashboard" here means
"queryable in InfluxDB".

    .venv/bin/python bench/latency.py --minutes 15     # waits, then analyses that window
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bench.common import flux, http_json, pct, save  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=15)
    ap.add_argument("--no-wait", action="store_true", help="analyse the last N minutes immediately")
    ap.add_argument("--out", default="latency.json")
    a = ap.parse_args()
    g0 = http_json("http://localhost:9100/")
    t0 = time.time()
    if not a.no_wait:
        print(f"collecting {a.minutes} min of live data ...")
        time.sleep(a.minutes * 60)
        g1 = http_json("http://localhost:9100/")
        if g1["disconnects"] != g0["disconnects"]:
            print("WARNING: MQTT disconnect during window -- result includes backlog")
    rows = flux(f'''from(bucket: "iiot") |> range(start: -{int(a.minutes * 60)}s)
      |> filter(fn: (r) => r._measurement == "pipeline_latency")
      |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")''')
    out = {"window_min": a.minutes, "n": len(rows),
           "method": "ingestor writes pipeline_latency after each Influx write ack; percentiles nearest-rank; SIMULATED plant timestamps; "
                     "same clock (Docker VM); excludes Grafana refresh"}
    # gw_ms is taken over the messages that carry a plant timestamp (PLC messages), so
    # e2e = src_age + gw holds per message.  Before v2 every message carried PLC data;
    # now vibration-only messages exist and are reported separately (their gw_ms
    # includes the ~55 ms RS-485 transaction, because ts is taken before the request).
    plc_rows = [r for r in rows if r.get("e2e_ms") not in (None, "")]
    vib_rows = [r for r in rows if r.get("e2e_ms") in (None, "")]
    out["n_plc"], out["n_vib_only"] = len(plc_rows), len(vib_rows)
    for k, src in (("e2e_ms", plc_rows), ("gw_ms", plc_rows), ("src_age_ms", plc_rows), ("gw_ms_vib", vib_rows)):
        v = [float(r[k.replace("_vib", "")]) for r in src if r.get(k.replace("_vib", "")) not in (None, "")]
        out[k] = {"p50": pct(v, 50), "p95": pct(v, 95), "p99": pct(v, 99), "max": max(v) if v else None,
                  "min": min(v) if v else None}
        print(k, out[k])
    g = http_json("http://localhost:9100/")
    dt = time.time() - t0
    out["gateway_poll_hz"] = g["poll_hz"]
    out["vib_poll_hz"] = g.get("vib_poll_hz")
    out["tags"] = g.get("tags")
    out["rbe_on"] = g.get("rbe_on")
    out["integrity_s"] = g.get("integrity_s")
    if not a.no_wait and "rbe" in g and "rbe" in g0:
        r = {}
        for src in ("plc", "vib"):
            d = {k: g["rbe"][src][k] - g0["rbe"][src][k] for k in g["rbe"][src]}
            d["msgs_per_s"] = round((d["integrity"] + d["change"]) / dt, 2)
            d["polls_per_s"] = round(d["polls"] / dt, 2)
            d["fields_sent_frac"] = round(d["fields_sent"] / d["fields_polled"], 4) if d["fields_polled"] else None
            r[src] = d
        out["rbe_window"] = r
        out["poll_overruns"] = g["poll_overruns"] - g0["poll_overruns"]
    save(a.out, out)


if __name__ == "__main__":
    main()
