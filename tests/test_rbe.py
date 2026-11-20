from iiot.rbe import RbeFilter
from iiot.telemetry import PLC_DEADBAND, PLC_FIELDS, VIB_DEADBAND, VIB_FIELDS

DB = {"level": 0.25, "run": 0, "cnt": None, "ts": None}


def f(level=50.0, run=True, cnt=1, ts=1000):
    return {"level": level, "run": run, "cnt": cnt, "ts": ts}


def test_first_poll_is_full_integrity_snapshot():
    r = RbeFilter(DB, ("ts",), integrity_s=5)
    out, kind = r.update(f(), 0.0)
    assert kind == "integrity" and out == f()


def test_inside_deadband_is_suppressed_and_counters_never_trigger():
    r = RbeFilter(DB, ("ts",), integrity_s=5)
    r.update(f(), 0.0)
    out, kind = r.update(f(level=50.2, cnt=2, ts=1050), 0.05)
    assert out is None and kind == "none"


def test_change_sends_only_moved_tags_plus_always():
    r = RbeFilter(DB, ("ts",), integrity_s=5)
    r.update(f(), 0.0)
    out, kind = r.update(f(level=50.3, cnt=3, ts=1100), 0.1)
    assert kind == "change" and out == {"level": 50.3, "ts": 1100}
    out, _ = r.update(f(level=50.3, run=False, ts=1150), 0.15)
    assert out == {"run": False, "ts": 1150}


def test_deadband_is_against_last_published_not_last_polled():
    """A slow ramp of 0.1 per poll must be reported once it has moved > deadband in total."""
    r = RbeFilter(DB, (), integrity_s=100)
    r.update(f(level=50.0), 0.0)
    sent = [r.update(f(level=50.0 + 0.1 * i), 0.1 * i)[0] for i in range(1, 8)]
    assert sent[:2] == [None, None] and sent[2] == {"level": 50.0 + 0.1 * 3}
    assert sent[3:5] == [None, None] and sent[5] == {"level": 50.0 + 0.1 * 6}


def test_periodic_integrity_and_reset():
    r = RbeFilter(DB, ("ts",), integrity_s=5)
    r.update(f(), 0.0)
    assert r.update(f(cnt=9), 4.9)[0] is None
    out, kind = r.update(f(cnt=10), 5.0)
    assert kind == "integrity" and out["cnt"] == 10
    r.reset()                                  # e.g. after a failed poll
    assert r.update(f(cnt=11), 5.1)[1] == "integrity"


def test_disabled_publishes_every_poll():
    r = RbeFilter(DB, (), integrity_s=5, enabled=False)
    assert all(r.update(f(), t * 0.05)[1] == "integrity" for t in range(5))


def test_every_tag_has_a_deadband_entry():
    assert set(PLC_FIELDS) == set(PLC_DEADBAND)
    assert set(VIB_FIELDS) == set(VIB_DEADBAND)
    assert len(PLC_FIELDS) + len(VIB_FIELDS) >= 100
