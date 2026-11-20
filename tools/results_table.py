"""Render results/*.json into the README results table (markdown)."""
import glob
import json
import os
import statistics as st

R = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")


def load(name):
    p = os.path.join(R, name)
    return json.load(open(p)) if os.path.exists(p) else None


rows = []
s = load("step_response.json")
if s:
    up = [x for x in s["steps"] if x["to"] > x["from"]]
    dn = [x for x in s["steps"] if x["to"] < x["from"]]
    for label, grp in (("50→70 %", up), ("70→50 %", dn)):
        rows.append((f"PID step {label}: settling, ±2 % of span", ", ".join(f"{x['settling_time_2pct_span_s']:.1f}" for x in grp) + " s",
                     f"{len(grp)} steps on OpenPLC loop, 20 Hz sampling"))
        rows.append((f"PID step {label}: settling, ±2 % of step (±0.4 % level)", ", ".join(f"{x['settling_time_s']:.1f}" for x in grp) + " s", "same runs"))
        rows.append((f"PID step {label}: overshoot", ", ".join(f"{x['overshoot_pct']:.2f}" for x in grp) + " % of step", "same runs"))
        rows.append((f"PID step {label}: steady-state error band (last 30 s)",
                     ", ".join(f"[{x['ss_min']:+.2f}, {x['ss_max']:+.2f}] σ={x['ss_std']:.2f}" for x in grp) + " % level",
                     "includes 0.10 % σ simulated transmitter noise"))
for line in open(os.path.join(R, "rtu_bench.jsonl")) if os.path.exists(os.path.join(R, "rtu_bench.jsonl")) else []:
    b = json.loads(line)
    rows.append((f"Modbus RTU, simulated BER {b['ber']:g}: errors per 10k polls",
                 f"{b['errors_per_10k']:.0f} (CRC {b['crc_per_10k']:.0f}, timeout {b['timeout_per_10k']:.0f})",
                 f"{b['n']} back-to-back FC04×{b.get('regs', 16)} regs @19200 8N1 emulated wire timing, {b['timeout_s'] * 1000:.0f} ms timeout" + (f"; {b['note']}" if b.get("note") else "")))
    rows.append((f"Modbus RTU, BER {b['ber']:g}: max poll rate / OK latency p50, p99",
                 f"{b['polls_per_s']:.1f} polls/s; {b['ok_latency_ms']['p50']:.1f}, {b['ok_latency_ms']['p99']:.1f} ms",
                 f"theoretical wire time {b['wire_time_ms_theoretical']} ms"))
t = load("plc_tcp_bench.json")
if t:
    rows.append(("Modbus TCP to OpenPLC: poll cycles/s, errors per 10k",
                 f"{t['cycles_per_s']} cycles/s, {(t['n'] - t['ok']) / t['n'] * 1e4:.0f} errors",
                 f"{t['n']} × (FC03 {t.get('hr', 16)} regs + FC01 {t.get('coils', 10)} coils), host→Docker; p50 {t['cycle_latency_ms']['p50']} ms, p99 {t['cycle_latency_ms']['p99']} ms"))
for fname, label in (("latency_rbe_off.json", "RBE off, every poll published"), ("latency.json", "")):
    l = load(fname)
    if not l:
        continue
    mode = label or ("RBE on (default)" if l.get("rbe_on") else "")
    tags = f", {l['tags']} tags" if l.get("tags") else ""
    rows.append((f"End-to-end latency plant I/O sample → InfluxDB write ack, p50 / p99{' — ' + mode if mode else ''}",
                 f"{l['e2e_ms']['p50']:.0f} / {l['e2e_ms']['p99']:.0f} ms (max {l['e2e_ms']['max']:.0f})",
                 f"{l.get('n_plc', l['n'])} PLC messages over {l['window_min']} min at {l['gateway_poll_hz']} Hz polling{tags}"))
    rows.append(("  of which gateway poll → InfluxDB ack, p50 / p99",
                 f"{l['gw_ms']['p50']:.0f} / {l['gw_ms']['p99']:.0f} ms", "MQTT/TLS + broker + ingestor + Influx write"))
    rows.append(("  of which sample age at gateway poll, p50 / p99",
                 f"{l['src_age_ms']['p50']:.0f} / {l['src_age_ms']['p99']:.0f} ms",
                 "plant I/O refresh + OpenPLC slave poll + scan"))
    rw = l.get("rbe_window")
    if rw:
        p_, v_ = rw["plc"], rw["vib"]
        rows.append(("  messages/s sent (PLC, vibration) / polls/s",
                     f"{p_['msgs_per_s']}, {v_['msgs_per_s']} / {p_['polls_per_s']}, {v_['polls_per_s']}",
                     f"tag values sent / polled: PLC {p_['fields_sent_frac'] * 100:.1f} %, vibration {v_['fields_sent_frac'] * 100:.1f} %"))
for p in sorted(glob.glob(os.path.join(R, "outage_*.json"))):
    o = json.load(open(p))
    bf = o["backfill_s"]
    rows.append((f"Outage test `{o['tag'] or o['mode']}`: {o['cycles']} × {o['outage_s_each']:.0f} s ({o['mode']})",
                 f"lost {o['lost_msgs']}, duplicate rows {o['duplicate_rows_in_influx']}, duplicate deliveries {o['duplicate_deliveries_at_ingestor']}",
                 f"{o['expected_msgs']} msgs checked by seq in InfluxDB"))
    rows.append((f"  backfill time / backlog per outage",
                 ", ".join(f"{b:.1f} s/{n}" for b, n in zip(bf, o["backlog_msgs"])),
                 f"CONNACK → last pre-reconnect row PUBACKed by broker; {o['field_values_per_msg']} field values per msg"))
    if o.get("influx_backfill_s"):
        rows.append(("  backlog fully written to InfluxDB after reconnect", ", ".join(f"{b:.2f} s" for b in o["influx_backfill_s"]),
                     "CONNACK → Influx write-ack of last queued msg (pipeline_latency)"))
i = load("imbalance.json")
if i:
    rows.append(("Imbalance detection, faults above 2.8 mm/s threshold",
                 f"{i['detected_above_threshold']} / {i['above_threshold']}",
                 f"{i['trials']} seeded faults (+{i['severity_range_mms'][0]}…{i['severity_range_mms'][1]} mm/s at 1x), judged in InfluxDB"))
    rows.append(("Imbalance detection, faults below threshold (should not alarm)",
                 f"{i['detected_below_threshold']} / {i['below_threshold']} alarmed", "same trials"))
    rows.append(("Imbalance time-to-detect min / median / max",
                 f"{i['time_to_detect_s']['min']} / {i['time_to_detect_s']['median']} / {i['time_to_detect_s']['max']} s",
                 "3-window debounce (0.64 s windows) + 2 Hz polling"))
    rows.append(("Imbalance false alarms in healthy periods", f"{i['false_alarm_samples']} / {i['healthy_samples_checked']} samples", ""))
il = load("interlocks.json")
if il:
    for c in il["cases"]:
        rows.append((f"Interlock: {c['case']}", ("PASS" if c["tripped"] else "FAIL") + (f", {c['t_s']} s" if c["t_s"] is not None else ""),
                     "fault injected via plant test API"))

print("| Metric (all SIMULATED plant/sensor) | Result | Method |")
print("|---|---|---|")
for m, v, how in rows:
    print(f"| {m} | {v} | {how} |")
