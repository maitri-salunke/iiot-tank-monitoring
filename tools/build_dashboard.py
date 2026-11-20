"""Generate grafana/provisioning/dashboards/tank.json (kept as code so it is reviewable)."""
import json, os

DS = {"type": "influxdb", "uid": "influx"}


def q(meas, fields, fn="mean", extra=""):
    flt = " or ".join(f'r._field == "{f}"' for f in fields)
    return (f'from(bucket: "iiot") |> range(start: v.timeRangeStart, stop: v.timeRangeStop)\n'
            f'  |> filter(fn: (r) => r._measurement == "{meas}" and ({flt})){extra}\n'
            f'  |> aggregateWindow(every: v.windowPeriod, fn: {fn}, createEmpty: false)')


def last_q(meas, field, extra="", rng="-1m"):
    return (f'from(bucket: "iiot") |> range(start: {rng})\n'
            f'  |> filter(fn: (r) => r._measurement == "{meas}" and r._field == "{field}") |> last(){extra}')


pid = [0]


def panel(title, kind, x, y, w, h, targets, unit=None, thresholds=None, extra=None, desc=None):
    pid[0] += 1
    fc = {"defaults": {"custom": {}}, "overrides": []}
    if unit:
        fc["defaults"]["unit"] = unit
    if thresholds:
        fc["defaults"]["thresholds"] = {"mode": "absolute", "steps": thresholds}
    p = {"id": pid[0], "title": title, "type": kind, "datasource": DS,
         "gridPos": {"x": x, "y": y, "w": w, "h": h}, "fieldConfig": fc,
         "targets": [{"refId": chr(65 + i), "datasource": DS, "query": t} for i, t in enumerate(targets)],
         "options": {}}
    if desc:
        p["description"] = desc
    if extra:
        p.update(extra)
    return p


green, red, amber = "green", "red", "orange"
bool_steps = [{"color": green, "value": None}, {"color": red, "value": 1}]
stat_opts = {"options": {"reduceOptions": {"calcs": ["lastNotNull"]}, "colorMode": "background", "graphMode": "none"}}
to_int = " |> toInt()"

panels = [
    panel("Tank level LT-101 vs setpoint (SIMULATED plant)", "timeseries", 0, 0, 12, 8,
          [q("tank", ["level_pct", "sp_pct"])], unit="percent",
          desc="Level from the Python tank model, controlled by the ST PID running in OpenPLC."),
    panel("Pump speed reference / flows", "timeseries", 12, 0, 12, 8,
          [q("tank", ["speed_pct"]), q("tank", ["fin_lpm", "fout_lpm"])]),
    panel("Pump running", "stat", 0, 8, 3, 4, [last_q("tank", "pump_run", to_int)],
          thresholds=[{"color": "gray", "value": None}, {"color": green, "value": 1}], extra=stat_opts),
    panel("HH alarm", "stat", 3, 8, 3, 4, [last_q("tank", "al_hh", to_int)], thresholds=bool_steps, extra=stat_opts),
    panel("Dry-run trip", "stat", 6, 8, 3, 4, [last_q("tank", "dry_trip", to_int)], thresholds=bool_steps, extra=stat_opts),
    panel("E-stop active", "stat", 9, 8, 3, 4, [last_q("tank", "estop", to_int)], thresholds=bool_steps, extra=stat_opts,
          desc="Hardwired in the field; the dashboard only displays it. Nothing on MQTT can reset it."),
    panel("Imbalance alarm (node)", "stat", 12, 8, 4, 4, [last_q("vibration", "imbalance", to_int)],
          thresholds=bool_steps, extra=stat_opts),
    panel("Gateway outbox depth", "stat", 16, 8, 4, 4, [last_q("gateway_health", "outbox_depth")],
          thresholds=[{"color": green, "value": None}, {"color": amber, "value": 10}], extra=stat_opts),
    panel("Gateway online", "stat", 20, 8, 4, 4, [last_q("gateway_status", "online", rng="-30d")],
          thresholds=[{"color": red, "value": None}, {"color": green, "value": 1}], extra=stat_opts),
    panel("Pump vibration velocity RMS (SIMULATED ADXL345)", "timeseries", 0, 12, 12, 8,
          [q("vibration", ["vel_rms_mms", "vel_1x_mms"])], unit="suffix: mm/s",
          thresholds=[{"color": green, "value": None}, {"color": amber, "value": 2.8}, {"color": red, "value": 4.5}],
          extra={"fieldConfig": {"defaults": {"unit": "suffix: mm/s", "custom": {"thresholdsStyle": {"mode": "line"}},
                                              "thresholds": {"mode": "absolute", "steps": [
                                                  {"color": green, "value": None}, {"color": amber, "value": 2.8},
                                                  {"color": red, "value": 4.5}]}}, "overrides": []}}),
    panel("FFT peak frequency / running speed", "timeseries", 12, 12, 12, 8,
          [q("vibration", ["peak_hz", "run_hz"])], unit="hertz"),
    panel("Pipeline latency: plant I/O sample -> Influx write ack", "timeseries", 0, 20, 12, 8,
          [q("pipeline_latency", ["e2e_ms", "gw_ms"], fn="max")], unit="ms"),
    panel("Gateway outbox depth / RTU errors", "timeseries", 12, 20, 12, 8,
          [q("gateway_health", ["outbox_depth"], fn="max"), q("gateway_health", ["rtu_crc_error", "rtu_timeout"], fn="max")]),
    # ---- v2 tags (report-by-exception: sparse points, so series use lastNotNull-style windows) ----
    panel("PID terms LIC-101 (P, I, D, output)", "timeseries", 0, 28, 12, 8,
          [q("tank", ["pid_p_pct", "pid_i_pct", "pid_d_pct", "pid_out_pct"], fn="last")], unit="percent",
          desc="Diagnostic outputs of FB_PID; P = Kp*(SP-PV), I is preloaded for bumpless transfer."),
    panel("Level filtered / rate of change, net flow", "timeseries", 12, 28, 12, 8,
          [q("tank", ["level_rate_pct_min", "net_flow_lpm"], fn="last")]),
    panel("First-out trip", "stat", 0, 36, 4, 4, [last_q("tank", "first_out")],
          thresholds=[{"color": green, "value": None}, {"color": red, "value": 1}],
          extra={"options": {"reduceOptions": {"calcs": ["lastNotNull"]}, "colorMode": "background", "graphMode": "none"},
                 "fieldConfig": {"defaults": {"mappings": [{"type": "value", "options": {
                     "0": {"text": "none"}, "1": {"text": "E-stop"}, "2": {"text": "HH level"}, "3": {"text": "dry run"},
                     "4": {"text": "fail to start"}, "5": {"text": "I/O fault"}}}],
                     "thresholds": {"mode": "absolute", "steps": [{"color": green, "value": None}, {"color": red, "value": 1}]}},
                     "overrides": []}}),
    panel("Sensor quality word", "stat", 4, 36, 4, 4, [last_q("tank", "quality")],
          thresholds=[{"color": green, "value": None}, {"color": amber, "value": 1}], extra=stat_opts,
          desc="b0 LT range, b1 LT frozen, b2 FT-101, b3 FT-102, b4 IT-101, b5 LSHH disc., b6 LSLL disc., b7 I/O stale"),
    panel("Active alarms", "stat", 8, 36, 4, 4, [last_q("tank", "active_alarms")],
          thresholds=[{"color": green, "value": None}, {"color": amber, "value": 1}], extra=stat_opts),
    panel("Pump starts", "stat", 12, 36, 4, 4, [last_q("tank", "pump_starts", rng="-10m")], extra=stat_opts),
    panel("Pump runtime", "stat", 16, 36, 4, 4, [last_q("tank", "pump_runtime_s", rng="-10m")], unit="s", extra=stat_opts),
    panel("Trips (dry / FTS / HH / E-stop)", "stat", 20, 36, 4, 4,
          [last_q("tank", f, rng="-10m") for f in ("dry_trips", "fts_trips", "hh_trips", "estop_count")], extra=stat_opts),
    panel("Vibration: crest factor / kurtosis (X)", "timeseries", 0, 40, 12, 8,
          [q("vibration", ["crest_x", "kurt_x", "kurt_y"], fn="last")],
          desc="Sine: crest 1.41, kurtosis 1.5; Gaussian: kurtosis 3; impacting bearing: > 4 (SIMULATED)."),
    panel("Vibration: X acceleration band RMS", "timeseries", 12, 40, 12, 8,
          [q("vibration", ["band1_mg", "band2_mg", "band3_mg", "band4_mg"], fn="last")], unit="suffix: mg"),
    panel("Vibration: harmonics 1x/2x/3x/vane-pass, per-axis velocity", "timeseries", 0, 48, 12, 8,
          [q("vibration", ["vel_1x_mms", "vel_2x_mms", "vel_3x_mms", "vel_vane_mms", "vel_y_mms", "vel_z_mms"], fn="last")],
          unit="suffix: mm/s"),
    panel("Bearing temperature (SIMULATED NTC) / ISO 10816 zone", "timeseries", 12, 48, 12, 8,
          [q("vibration", ["temp_c"], fn="last"), q("vibration", ["iso_zone"], fn="last")]),
    panel("Report-by-exception: messages per window", "timeseries", 0, 56, 24, 6,
          [q("poll", ["seq"], fn="count")], desc="One poll point per MQTT message; PLC polled at 20 Hz, RTU at 2 Hz."),
]

dash = {"uid": "iiot-tank", "title": "Tank control & pump condition (SIL testbed)", "schemaVersion": 39,
        "version": 2, "editable": False, "refresh": "5s", "time": {"from": "now-15m", "to": "now"},
        "tags": ["iiot", "simulated"], "panels": panels}
out = os.path.join(os.path.dirname(__file__), "..", "grafana", "provisioning", "dashboards", "tank.json")
json.dump(dash, open(out, "w"), indent=1)
print("wrote", out)
