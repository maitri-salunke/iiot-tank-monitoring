"""Register decoding (gateway side) and Influx point mapping (ingestor side).

Pure functions -- no I/O -- so the decoding and the idempotency property are
unit-testable (tests/test_telemetry_idempotent.py).

The PLC tag map is table-driven (PLC_HR_MAP / COIL_NAMES). tests/test_plc_tagmap.py
parses plc/tank_control.st and fails if the %QW / %QX declarations drift from it.
"""
from __future__ import annotations

from . import vib_regmap as R

# ------------------------------------------------------------------ PLC ------
# (register, field name, scale divisor, signed).  Registers 7/8 (sample time) and
# 27/28 (pump runtime) are 32-bit hi/lo pairs and are decoded separately.
PLC_HR_MAP: list[tuple[int, str, float, bool]] = [
    (0, "level_pct", 100, False),
    (1, "sp_pct", 100, False),          # active (clamped) setpoint
    (2, "speed_pct", 100, False),
    (3, "fin_lpm", 10, False),
    (4, "fout_lpm", 10, False),
    (5, "current_a", 100, False),
    (6, "valve_pct", 100, False),
    # 7, 8: plant sample time ms (hi, lo) -> src_ts_ms
    (9, "src_cnt", 1, False),
    (10, "status", 1, False),
    (11, "scans", 1, False),
    (12, "err_pct", 100, True),
    (13, "mode", 1, False),
    (14, "kp", 100, False),
    (15, "ti_s", 10, False),
    (16, "td_s", 100, False),
    (17, "pid_out_pct", 100, False),
    (18, "pid_p_pct", 100, True),
    (19, "pid_i_pct", 100, True),
    (20, "pid_d_pct", 100, True),
    (21, "hmi_sp_pct", 100, False),     # operator entry before the 90 % clamp
    (22, "man_speed_pct", 100, False),
    (23, "lim_h_pct", 100, False),
    (24, "lim_l_pct", 100, False),
    (25, "lim_hh_pct", 100, False),
    (26, "lim_ll_pct", 100, False),
    # 27, 28: pump runtime s (hi, lo) -> pump_runtime_s
    (29, "pump_starts", 1, False),
    (30, "dry_trips", 1, False),
    (31, "fts_trips", 1, False),
    (32, "hh_trips", 1, False),
    (33, "estop_count", 1, False),
    (34, "io_fault_count", 1, False),
    (35, "first_out", 1, False),        # 0 none, 1 E-stop, 2 HH, 3 dry run, 4 fail-to-start, 5 I/O fault
    (36, "dry_tmr_ms", 1, False),
    (37, "fts_tmr_ms", 1, False),
    (38, "stale_tmr_ms", 1, False),
    (39, "quality", 1, False),          # per-sensor quality word, see QUALITY_BITS
    (40, "interlocks", 1, False),       # permissive-chain contacts, 1 = healthy
    (41, "di_word", 1, False),          # raw field DIs
    (42, "net_flow_lpm", 10, True),
    (43, "level_filt_pct", 100, False),
    (44, "level_rate_pct_min", 100, True),
    (45, "heartbeat", 1, False),
    (46, "run_since_start_s", 1, False),
    (47, "active_alarms", 1, False),
    (48, "scan_ms", 1, False),
    (49, "prog_version", 1, False),
]
PLC_HR_START, PLC_HR_COUNT = 0, 50      # %QW0..%QW49  -> one FC03 request
PLC_COIL_START = 0
COIL_NAMES = ["pump_run", "al_h", "al_l", "al_hh", "al_ll", "dry_trip", "estop", "permissive",
              "fts_trip", "io_fault",
              "manual", "pid_sat", "di_pump_fb", "di_lshh", "di_lsll", "run_cmd",
              "dry_timing", "fts_timing", "stale_timing",
              "q_lt_bad", "q_ft101_bad", "q_ft102_bad", "q_it101_bad", "q_lshh_disc", "q_lsll_disc",
              "any_trip"]
PLC_COIL_COUNT = len(COIL_NAMES)        # %QX0.0..%QX3.1 -> one FC01 request
QUALITY_BITS = ["lt101_range", "lt101_frozen", "ft101_range", "ft102_range", "it101_bad",
                "lshh_discrepancy", "lsll_discrepancy", "io_stale"]
PLC_FIELDS = [n for _, n, _, _ in PLC_HR_MAP] + ["src_ts_ms", "pump_runtime_s"] + COIL_NAMES


def _s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


def _scale(v: int, scale) -> int | float:
    """int scale 1 -> Influx integer field; anything else (incl. 1.0) -> float field.
    Field types must never change for an existing field (Influx rejects the write)."""
    return v if (scale == 1 and isinstance(scale, int)) else v / float(scale)


def unwrap_ms32(v32: int, now_ms: int, max_future_ms: int = 60_000) -> int:
    """Rebuild a full epoch-ms timestamp from its low 32 bits (wraps every 49.7 days).

    The sample may legitimately be slightly *newer* than `now_ms` (the poll
    timestamp is taken before the Modbus request goes out), so the reference is
    shifted forward by `max_future_ms` -- without this, a sample 1 ms in the
    future decoded as ~49.7 days in the past (found in the first latency run).
    """
    ref = now_ms + max_future_ms
    return ref - ((ref - v32) % (1 << 32))


def decode_plc(hr: list[int], coils: list[bool], now_ms: int) -> dict:
    d: dict = {}
    for reg, name, scale, signed in PLC_HR_MAP:
        v = _s16(hr[reg]) if signed else hr[reg]
        d[name] = _scale(v, scale)
    d["src_ts_ms"] = unwrap_ms32((hr[7] << 16) | hr[8], now_ms)
    d["pump_runtime_s"] = (hr[27] << 16) | hr[28]
    for name, bit in zip(COIL_NAMES, coils[:PLC_COIL_COUNT]):
        d[name] = bool(bit)
    return d


# ------------------------------------------------------------ vibration ------
# (register, field name, scale divisor, signed)
VIB_IR_MAP: list[tuple[int, str, float, bool]] = [
    (R.IR_STATUS, "status", 1, False),
    (R.IR_ACC_RMS_RADIAL, "acc_rms_mg", 1.0, False),
    (R.IR_VEL_RMS, "vel_rms_mms", 100, False),
    (R.IR_PEAK_FREQ, "peak_hz", 10, False),
    (R.IR_PEAK_AMP, "peak_mg", 1.0, False),
    (R.IR_VEL_1X, "vel_1x_mms", 100, False),
    (R.IR_RUN_SPEED, "run_hz", 10, False),
    (R.IR_WINDOW_COUNT, "window", 1, False),
    # IR_UPTIME_HI/LO -> uptime_s
    (R.IR_ACC_RMS_X, "acc_x_mg", 1.0, False),
    (R.IR_ACC_RMS_Y, "acc_y_mg", 1.0, False),
    (R.IR_ACC_RMS_Z, "acc_z_mg", 1.0, False),
    (R.IR_FW_VERSION, "fw_version", 1, False),
    (R.IR_VEL_RMS_Y, "vel_y_mms", 100, False),
    (R.IR_VEL_RMS_Z, "vel_z_mms", 100, False),
    (R.IR_ACC_PEAK_X, "acc_pk_x_mg", 1.0, False),
    (R.IR_ACC_PEAK_Y, "acc_pk_y_mg", 1.0, False),
    (R.IR_ACC_PEAK_Z, "acc_pk_z_mg", 1.0, False),
    (R.IR_CREST_X, "crest_x", 100, False),
    (R.IR_CREST_Y, "crest_y", 100, False),
    (R.IR_CREST_Z, "crest_z", 100, False),
    (R.IR_KURT_X, "kurt_x", 100, False),
    (R.IR_KURT_Y, "kurt_y", 100, False),
    (R.IR_KURT_Z, "kurt_z", 100, False),
    (R.IR_BAND1_ACC, "band1_mg", 10, False),
    (R.IR_BAND2_ACC, "band2_mg", 10, False),
    (R.IR_BAND3_ACC, "band3_mg", 10, False),
    (R.IR_BAND4_ACC, "band4_mg", 10, False),
    (R.IR_VEL_2X, "vel_2x_mms", 100, False),
    (R.IR_VEL_3X, "vel_3x_mms", 100, False),
    (R.IR_VEL_VANE, "vel_vane_mms", 100, False),
    (R.IR_DC_X, "dc_x_mg", 1.0, True),
    (R.IR_DC_Y, "dc_y_mg", 1.0, True),
    (R.IR_DC_Z, "dc_z_mg", 1.0, True),
    (R.IR_TEMP, "temp_c", 100, True),
    (R.IR_CLIP_COUNT, "clip_count", 1, False),
    (R.IR_FIFO_OVR, "fifo_overruns", 1, False),
    (R.IR_I2C_ERR, "i2c_errors", 1, False),
    (R.IR_VA_CNT, "va_debounce", 1, False),
    (R.IR_IM_CNT, "im_debounce", 1, False),
    (R.IR_ISO_ZONE, "iso_zone", 1, False),
]
VIB_FIELDS = ([n for _, n, _, _ in VIB_IR_MAP] + ["uptime_s", "sensor_ok", "window_valid", "vel_alert",
                                                   "imbalance"])


def decode_vib(ir: list[int]) -> dict:
    d: dict = {}
    for reg, name, scale, signed in VIB_IR_MAP:
        v = _s16(ir[reg]) if signed else ir[reg]
        d[name] = _scale(v, scale)
    st = ir[R.IR_STATUS]
    d["uptime_s"] = (ir[R.IR_UPTIME_HI] << 16) | ir[R.IR_UPTIME_LO]
    d["sensor_ok"] = bool(st & R.STATUS_SENSOR_OK)
    d["window_valid"] = bool(st & R.STATUS_WINDOW_VALID)
    d["vel_alert"] = bool(st & R.STATUS_VEL_ALERT)
    d["imbalance"] = bool(st & R.STATUS_IMBALANCE)
    return d


TAG_COUNT = len(PLC_FIELDS) + len(VIB_FIELDS)

# ------------------------------------------------- report-by-exception -------
# Deadband per field (absolute, engineering units).  0 = publish on any change;
# None = never triggers a publish by itself (counters, timers, clocks) -- such a
# field is still sent with the integrity snapshot.  Fields in *_ALWAYS ride on
# every message (the sample timestamp is needed for latency accounting).
# Deadbands are set from the measured noise of the SIMULATED signals: e.g. level
# noise is 0.10 % (1 sigma), so 0.25 % is ~1.8 sigma of the difference of two samples.
PLC_DEADBAND: dict[str, float | None] = {
    "level_pct": 0.25, "level_filt_pct": 0.2, "level_rate_pct_min": 3.0,
    "sp_pct": 0, "hmi_sp_pct": 0, "man_speed_pct": 0,
    # Kp = 15 turns 0.1 % level noise into ~1.5 % (1 sigma) of P-term / output jitter
    "speed_pct": 3.0, "pid_out_pct": 3.0, "pid_p_pct": 5.0, "pid_i_pct": 1.0, "pid_d_pct": 3.0,
    "err_pct": 0.3,
    "fin_lpm": 1.0, "fout_lpm": 1.0, "net_flow_lpm": 1.5, "current_a": 0.1, "valve_pct": 0.5,
    "mode": 0, "kp": 0, "ti_s": 0, "td_s": 0, "status": 0, "quality": 0, "interlocks": 0, "di_word": 0,
    "first_out": 0, "active_alarms": 0,
    "lim_h_pct": 0, "lim_l_pct": 0, "lim_hh_pct": 0, "lim_ll_pct": 0, "scan_ms": 0, "prog_version": 0,
    "pump_starts": 0, "dry_trips": 0, "fts_trips": 0, "hh_trips": 0, "estop_count": 0, "io_fault_count": 0,
    # free-running counters / clocks: integrity only
    "src_cnt": None, "scans": None, "heartbeat": None, "src_ts_ms": None, "pump_runtime_s": None,
    "run_since_start_s": None, "dry_tmr_ms": None, "fts_tmr_ms": None, "stale_tmr_ms": None,
    **{c: 0 for c in COIL_NAMES},
}
PLC_ALWAYS = ("src_ts_ms",)
VIB_DEADBAND: dict[str, float | None] = {
    "vel_rms_mms": 0.05, "vel_1x_mms": 0.05, "vel_y_mms": 0.05, "vel_z_mms": 0.05,
    "vel_2x_mms": 0.05, "vel_3x_mms": 0.05, "vel_vane_mms": 0.05,
    "acc_rms_mg": 2.0, "acc_x_mg": 2.0, "acc_y_mg": 2.0, "acc_z_mg": 2.0,
    "acc_pk_x_mg": 10.0, "acc_pk_y_mg": 10.0, "acc_pk_z_mg": 10.0,
    "crest_x": 0.2, "crest_y": 0.2, "crest_z": 0.2, "kurt_x": 0.2, "kurt_y": 0.2, "kurt_z": 0.2,
    "band1_mg": 2.0, "band2_mg": 2.0, "band3_mg": 1.0, "band4_mg": 1.0,
    "peak_hz": 1.0, "peak_mg": 3.0, "run_hz": 0.5,
    "dc_x_mg": 10.0, "dc_y_mg": 10.0, "dc_z_mg": 10.0, "temp_c": 0.2,
    "status": 0, "sensor_ok": 0, "window_valid": 0, "vel_alert": 0, "imbalance": 0, "iso_zone": 0,
    "clip_count": 0, "fifo_overruns": 0, "i2c_errors": 0, "va_debounce": 0, "im_debounce": 0,
    "fw_version": 0,
    "window": None, "uptime_s": None,
}
VIB_ALWAYS = ("window",)


# --------------------------------------------------------------- Influx ------

def _fmt_field(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return f"{v}i"
    if isinstance(v, float):
        return repr(v)
    s = str(v).replace("\\", "\\\\").replace('"', '\\"')
    return f'"{s}"'


def _line(measurement: str, tags: dict, fields: dict, ts_ms: int) -> str:
    tag_s = ",".join(f"{k}={v}" for k, v in sorted(tags.items()))
    field_s = ",".join(f"{k}={_fmt_field(v)}" for k, v in sorted(fields.items()))
    return f"{measurement},{tag_s} {field_s} {ts_ms}"


def to_lines(msg: dict) -> list[str]:
    """Map one telemetry message to Influx line protocol (ms precision).

    The point timestamp is the gateway poll time carried *inside* the message,
    never the arrival time, so re-delivering the same message produces
    byte-identical points -> Influx overwrites instead of duplicating.
    With report-by-exception a message may carry only the fields that changed;
    Influx stores sparse fields natively (each field is its own series).
    """
    ts = int(msg["ts"])
    seq = int(msg["seq"])
    base = {"site": msg.get("site", "site1")}
    lines = []
    if msg.get("plc"):
        f = dict(msg["plc"])
        f["seq"] = seq
        lines.append(_line("tank", {**base, "asset": msg.get("asset", "tank1")}, f, ts))
    if msg.get("vib"):
        f = dict(msg["vib"])
        f["seq"] = seq
        lines.append(_line("vibration", {**base, "asset": "pump1"}, f, ts))
    q = msg.get("q", {})
    pf = {"seq": seq, "plc": q.get("plc", "n/a"), "vib": q.get("vib", "n/a")}
    if "kind" in msg:
        pf["kind"] = msg["kind"]
    lines.append(_line("poll", {**base, "gateway": msg.get("gw", "gw1")}, pf, ts))
    return lines
