"""Broker / WAN outage test for store-and-forward.

Modes
  net     docker network disconnect <it_net> iiot-gateway   (gateway keeps polling the OT side;
          its TCP session to the broker just goes dark -- detected by MQTT keepalive)
  broker  docker stop iiot-mosquitto / docker start            (graceful stop: broker persists
          its queue for the ingestor's persistent session)
  kill    docker kill iiot-mosquitto (SIGKILL) / docker start   (ungraceful: anything the broker
          PUBACKed but had not yet delivered or autosaved can be lost)

For each outage: record the gateway seq before, cut, wait D seconds, restore,
then wait until the gateway reports the backlog drained (its backfill timer
runs from MQTT CONNACK until every row that existed at reconnect is PUBACKed).

At the end: for the whole seq range covered, query InfluxDB for
    rows    = count of `poll.seq` points           (one per message)
    distinct= count of distinct seq values
    lost    = expected - distinct
    duplicate rows in Influx = rows - distinct     (idempotent writes => 0)
plus duplicate *deliveries* counted by the ingestor (QoS 1 at-least-once).

    .venv/bin/python bench/outage_test.py --cycles 6 --duration 120 --mode alternate
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bench.common import docker, flux, http_json, save  # noqa: E402
from iiot.telemetry import PLC_FIELDS, VIB_FIELDS  # noqa: E402

GW = "http://localhost:9100/"
ING = "http://localhost:9101/"
NET = os.environ.get("IT_NET", "iiot-tank-monitoring_it_net")


def gw():
    """Gateway metrics via docker exec: the published port rides on it_net, which
    is exactly the network this test disconnects."""
    for _ in range(30):
        try:
            out = docker("exec", "iiot-gateway", "python", "-c",
                         "import urllib.request;print(urllib.request.urlopen('http://127.0.0.1:9100/').read().decode())")
            return json.loads(out)
        except Exception:
            time.sleep(1)
    raise RuntimeError("gateway metrics unreachable")


def cut(mode):
    if mode == "net":
        docker("network", "disconnect", NET, "iiot-gateway")
    elif mode == "kill":               # ungraceful: SIGKILL, no final persistence save
        docker("kill", "iiot-mosquitto")
    else:
        docker("stop", "-t", "10", "iiot-mosquitto")


def restore(mode):
    if mode == "net":
        docker("network", "connect", NET, "iiot-gateway")
    else:
        docker("start", "iiot-mosquitto")


def wait_drained(n_drains_before, restore_ms, timeout=900):
    t0 = time.time()
    while time.time() - t0 < timeout:
        g = gw()
        new = [d for d in g["drains"] if d["reconnect_ms"] >= restore_ms - 2000]
        if len(g["drains"]) > n_drains_before and new and g["outbox_depth"] < 50:
            return new[-1], g
        time.sleep(1)
    raise RuntimeError("backlog did not drain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--duration", type=float, default=120)
    ap.add_argument("--mode", choices=["net", "broker", "kill", "alternate"], default="alternate")
    ap.add_argument("--gap", type=float, default=20, help="healthy seconds between outages")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()

    ing0 = http_json(ING)
    g0 = gw()
    seq_first = g0["seq_max"] + 1
    t_start = time.time()
    cycles = []
    for i in range(a.cycles):
        mode = a.mode if a.mode != "alternate" else ("net" if i % 2 == 0 else "broker")
        g = gw()
        seq_before = g["seq_max"]
        cut(mode)
        t_cut = time.time()
        print(f"[{i + 1}/{a.cycles}] {mode} outage for {a.duration:.0f}s (seq {seq_before})", flush=True)
        time.sleep(a.duration)
        g_mid = gw()
        restore(mode)
        restore_ms = int(time.time() * 1000)
        d, g_after = wait_drained(len(g_mid["drains"]), restore_ms)
        c = {"mode": mode, "duration_s": a.duration, "seq_before": seq_before, "seq_at_restore": g_mid["seq_max"],
             "outbox_depth_at_restore": g_mid["outbox_depth"], "backlog_at_reconnect": d["backlog"],
             "backfill_s": d["backfill_s"],
             "reconnect_delay_s": round(d["reconnect_ms"] / 1000 - restore_ms / 1000, 2)}
        cycles.append(c)
        print("   ", c, flush=True)
        time.sleep(a.gap)

    time.sleep(10)                      # let the last live messages land
    g_end = gw()
    seq_last = g_end["seq_max"] - 20    # exclude the newest few still in flight
    ing1 = http_json(ING)
    start = int(t_start - 120)
    base = (f'from(bucket: "iiot") |> range(start: {start}) '
            f'|> filter(fn: (r) => r._measurement == "poll" and r._field == "seq") '
            f'|> filter(fn: (r) => r._value >= {seq_first} and r._value <= {seq_last})')
    rows = flux(base + ' |> group() |> count()')
    distinct = flux(base + ' |> group() |> distinct() |> count()')
    n_rows = int(rows[0]["_value"]) if rows else 0
    n_distinct = int(distinct[0]["_value"]) if distinct else 0
    expected = seq_last - seq_first + 1
    missing = []
    if n_distinct != expected:
        got = {int(r["_value"]) for r in flux(base + ' |> keep(columns: ["_value"])')}
        missing = sorted(set(range(seq_first, seq_last + 1)) - got)[:200]
    tank_fields = len(PLC_FIELDS)
    vib_fields = len(VIB_FIELDS)
    res = {
        "tag": a.tag, "mode": a.mode, "cycles": a.cycles, "outage_s_each": a.duration,
        "total_outage_s": a.cycles * a.duration, "wall_time_s": round(time.time() - t_start, 1),
        "seq_range": [seq_first, seq_last], "expected_msgs": expected,
        "influx_rows": n_rows, "influx_distinct_seq": n_distinct,
        "lost_msgs": expected - n_distinct, "duplicate_rows_in_influx": n_rows - n_distinct,
        "duplicate_deliveries_at_ingestor": ing1["duplicates"] - ing0["duplicates"],
        "missing_seq_sample": missing,
        "field_values_per_msg": tank_fields + vib_fields,     # tags polled (RBE sends a subset per msg)
        "tags": tank_fields + vib_fields, "rbe_on": g_end.get("rbe_on"), "plc_poll_hz": g_end.get("poll_hz"),
        "backfill_s": [c["backfill_s"] for c in cycles],
        "backlog_msgs": [c["backlog_at_reconnect"] for c in cycles],
        "gateway_outbox_dropped": g_end["outbox_dropped"],
        "cycles_detail": cycles,
        "method": "docker network disconnect (it_net) / docker stop mosquitto; Influx counts by seq; SIMULATED data",
    }
    print({k: v for k, v in res.items() if k != "cycles_detail"})
    save(f"outage_{a.tag or a.mode}.json", res)


if __name__ == "__main__":
    main()
