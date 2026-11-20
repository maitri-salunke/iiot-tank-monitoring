import json
from iiot.outbox import Outbox, wire_payload


def test_seq_monotonic_and_survives_restart(tmp_path):
    p = str(tmp_path / "ob.db")
    ob = Outbox(p)
    seqs = [ob.enqueue("t", 1000 + i, {"i": i}) for i in range(5)]
    assert seqs == [1, 2, 3, 4, 5]
    ob.ack([1, 2, 3, 4, 5])
    assert ob.depth() == 0
    ob.close()
    ob = Outbox(p)                         # restart: seq must not be reused after deletes
    assert ob.enqueue("t", 2000, {}) == 6
    assert ob.max_seq() == 6


def test_unacked_rows_replay_in_order(tmp_path):
    ob = Outbox(str(tmp_path / "ob.db"))
    for i in range(10):
        ob.enqueue("t", i, {"i": i})
    ob.ack([1, 2, 5])                       # out-of-order PUBACKs are fine
    rows = ob.fetch_after(0, 100)
    assert [r[0] for r in rows] == [3, 4, 6, 7, 8, 9, 10]
    assert [r[0] for r in ob.fetch_after(6, 2)] == [7, 8]
    assert ob.oldest_seq() == 3


def test_crash_between_publish_and_ack_replays(tmp_path):
    """Simulate: rows handed to MQTT, process dies before PUBACK -> rows still there."""
    p = str(tmp_path / "ob.db")
    ob = Outbox(p)
    for i in range(3):
        ob.enqueue("t", i, {"v": i})
    handed = ob.fetch_after(0, 3)          # published, but no ack recorded
    ob.close()
    ob2 = Outbox(p)
    again = ob2.fetch_after(0, 10)
    assert [r[0] for r in again] == [r[0] for r in handed]   # at-least-once: re-sent
    # identical payload bytes on replay (needed for idempotent downstream writes)
    assert [wire_payload(r[0], r[3]) for r in again] == [wire_payload(r[0], r[3]) for r in handed]


def test_cap_drops_oldest_and_counts(tmp_path):
    ob = Outbox(str(tmp_path / "ob.db"), max_rows=500)
    for i in range(2000):
        ob.enqueue("t", i, {})
    assert ob.depth() <= 500 + 1000        # cap enforced every 1000 inserts
    assert ob.dropped > 0
    assert ob.oldest_seq() > 1


def test_wire_payload_deterministic():
    a = wire_payload(7, json.dumps({"b": 1, "a": 2}))
    b = wire_payload(7, json.dumps({"a": 2, "b": 1}))
    assert a == b and json.loads(a)["seq"] == 7
