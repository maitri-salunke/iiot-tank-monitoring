import json
import sqlite3
from iiot import vib_regmap as R
from iiot.telemetry import (PLC_COIL_COUNT, PLC_FIELDS, PLC_HR_COUNT, TAG_COUNT, VIB_FIELDS, decode_plc,
                            decode_vib, to_lines, unwrap_ms32)
from iiot.outbox import Outbox, wire_payload


def sample_msg(seq=42):
    hr = [5012, 5000, 4870, 291, 300, 245, 5000, 0x0190, 0x1234, 77, 0x0081, 9, 0xFFF4, 1, 1500, 120]
    hr += [0, 4870, 0xFFE2, 4900, 0, 5000, 0, 8500, 1500, 9500, 500, 0x0001, 0x0002, 3, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0x003F, 0x0013, 0xFF9C, 5010, 0xFFCE, 77, 120, 0, 50, 0x0200]
    coils = [True, False, False, False, False, False, False, True, False, False] + [False] * 16
    now = 1_763_600_000_000
    plc = decode_plc(hr, coils, now)
    ir = [3, 30, 132, 1237, 39, 120, 247, 5, 0, 99] + [30, 24, 9, 0x0101, 800, 512] + [0] * (R.IR_COUNT - 16)
    ir[R.IR_DC_Z] = 1000
    ir[R.IR_DC_X] = 0xFFF6        # -10 mg
    ir[R.IR_TEMP] = 3712
    ir[R.IR_KURT_X] = 152
    vib = decode_vib(ir)
    return {"seq": seq, "ts": now, "site": "site1", "asset": "tank1", "gw": "gw1",
            "plc": plc, "vib": vib, "q": {"plc": "ok", "vib": "ok"}}


def test_decode_scaling_and_signed():
    m = sample_msg()
    assert m["plc"]["level_pct"] == 50.12 and m["plc"]["err_pct"] == -0.12
    assert m["plc"]["pump_run"] is True and m["plc"]["permissive"] is True
    assert m["vib"]["sensor_ok"] and not m["vib"]["imbalance"]
    assert m["vib"]["vel_rms_mms"] == 1.32 and m["vib"]["peak_hz"] == 123.7


def test_decode_extended_tags():
    m = sample_msg()
    p, v = m["plc"], m["vib"]
    assert len(p) == len(PLC_FIELDS) and len(v) == len(VIB_FIELDS) and TAG_COUNT >= 100
    assert p["pid_p_pct"] == -0.3 and p["net_flow_lpm"] == -10.0 and p["level_rate_pct_min"] == -0.5
    assert p["pump_runtime_s"] == 65538 and p["pump_starts"] == 3 and p["first_out"] == 0
    assert p["lim_hh_pct"] == 95.0 and p["prog_version"] == 0x0200
    assert v["dc_x_mg"] == -10.0 and v["dc_z_mg"] == 1000.0 and v["temp_c"] == 37.12 and v["kurt_x"] == 1.52
    assert v["window_valid"] is True and v["fw_version"] == 0x0101


def test_influx_field_types_stay_stable():
    """Influx rejects a write that changes a field's type; the original 40 fields keep theirs."""
    m = sample_msg()
    assert isinstance(m["plc"]["level_pct"], float) and isinstance(m["plc"]["scans"], int)
    assert isinstance(m["vib"]["acc_x_mg"], float) and isinstance(m["vib"]["peak_mg"], float)
    assert isinstance(m["vib"]["window"], int) and isinstance(m["vib"]["uptime_s"], int)


def test_sparse_rbe_message_maps_to_lines():
    msg = {"seq": 7, "ts": 1000, "plc": {"level_pct": 50.3, "src_ts_ms": 990}, "q": {"plc": "ok"}, "kind": "change"}
    lines = to_lines(msg)
    assert lines[0] == "tank,asset=tank1,site=site1 level_pct=50.3,seq=7i,src_ts_ms=990i 1000"
    assert lines[1] == 'poll,gateway=gw1,site=site1 kind="change",plc="ok",seq=7i,vib="n/a" 1000'


def test_unwrap_ms32_handles_wrap():
    now = (5 << 32) + 1000
    assert unwrap_ms32(500, now) == (5 << 32) + 500
    assert unwrap_ms32(0xFFFFFF00, now) == (4 << 32) + 0xFFFFFF00   # sample from before the wrap
    assert unwrap_ms32((now + 3) & 0xFFFFFFFF, now) == now + 3          # sample a few ms newer than poll ts


def test_redelivery_produces_identical_points():
    """QoS 1 may deliver a message twice; the Influx points must be byte-identical
    (same measurement, tags, timestamp, fields) so the second write is an overwrite."""
    m = sample_msg()
    wire = json.dumps(m)
    first = to_lines(json.loads(wire))
    second = to_lines(json.loads(wire))
    assert first == second
    assert all(l.endswith(f" {m['ts']}") for l in first)      # poll time, not arrival time
    assert any(l.startswith("tank,") for l in first) and any(l.startswith("vibration,") for l in first)


def test_end_to_end_replay_is_idempotent(tmp_path):
    """Outbox replay -> wire payload -> line protocol, stored in a key/value table
    keyed like Influx (series key + time). Duplicates must not add rows."""
    ob = Outbox(str(tmp_path / "ob.db"))
    for i in range(20):
        body = sample_msg(0); body.pop("seq"); body["ts"] += i * 500
        ob.enqueue("t", body["ts"], body)
    db = sqlite3.connect(":memory:")
    db.execute("CREATE TABLE pts (k TEXT PRIMARY KEY, v TEXT)")

    def ingest(rows):
        for seq, _ts, _topic, body in rows:
            for line in to_lines(json.loads(wire_payload(seq, body))):
                series, fields, ts = line.rsplit(" ", 2)
                db.execute("INSERT OR REPLACE INTO pts VALUES (?, ?)", (series + "@" + ts, fields))
    rows = ob.fetch_after(0, 100)
    ingest(rows[:15])
    ingest(rows[10:])            # overlap: 10..14 delivered twice
    ingest(rows)                 # full replay
    assert db.execute("SELECT COUNT(*) FROM pts").fetchone()[0] == 20 * 3   # tank + vibration + poll
