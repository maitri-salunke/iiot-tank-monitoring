"""SQLite store-and-forward outbox.

Every poll snapshot is committed here *before* any network I/O. The row id is
the message sequence number:

  * INTEGER PRIMARY KEY AUTOINCREMENT -> strictly increasing, never reused, even
    after rows are deleted or the process restarts (SQLite keeps the high-water
    mark in sqlite_sequence).
  * a row is deleted only after the broker's PUBACK for it (QoS 1).
  * on restart, every remaining row is re-published in seq order.  Some may
    already have reached the broker (PUBACK lost) -> the subscriber sees a
    duplicate, which is harmless because the Influx write is idempotent
    (same measurement + tags + timestamp overwrites).

WAL + synchronous=NORMAL: survives process crashes; a power cut may lose the
last few transactions (acceptable for telemetry; use FULL for stricter needs).
"""
from __future__ import annotations

import json
import sqlite3
import threading
from typing import Iterable


class Outbox:
    def __init__(self, path: str, max_rows: int = 2_000_000):
        self.path = path
        self.max_rows = max_rows
        self.dropped = 0
        self._lock = threading.Lock()
        self._db = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=NORMAL")
        self._db.execute(
            "CREATE TABLE IF NOT EXISTS outbox ("
            " seq INTEGER PRIMARY KEY AUTOINCREMENT,"
            " ts_ms INTEGER NOT NULL,"
            " topic TEXT NOT NULL,"
            " body TEXT NOT NULL)"
        )

    def enqueue(self, topic: str, ts_ms: int, body: dict) -> int:
        with self._lock:
            cur = self._db.execute("INSERT INTO outbox (ts_ms, topic, body) VALUES (?, ?, ?)",
                                   (ts_ms, topic, json.dumps(body, separators=(",", ":"))))
            seq = cur.lastrowid
            if self.max_rows and seq % 1000 == 0:
                self._enforce_cap()
            return seq

    def _enforce_cap(self) -> None:
        n = self._db.execute("SELECT COUNT(*) FROM outbox").fetchone()[0]
        if n > self.max_rows:
            excess = n - self.max_rows
            self._db.execute("DELETE FROM outbox WHERE seq IN (SELECT seq FROM outbox ORDER BY seq LIMIT ?)",
                             (excess,))
            self.dropped += excess  # oldest-first drop policy, counted and reported

    def fetch_after(self, last_seq: int, limit: int) -> list[tuple[int, int, str, str]]:
        with self._lock:
            return self._db.execute(
                "SELECT seq, ts_ms, topic, body FROM outbox WHERE seq > ? ORDER BY seq LIMIT ?",
                (last_seq, limit)).fetchall()

    def ack(self, seqs: Iterable[int]) -> int:
        seqs = list(seqs)
        if not seqs:
            return 0
        with self._lock:
            self._db.execute("BEGIN")
            self._db.executemany("DELETE FROM outbox WHERE seq = ?", [(s,) for s in seqs])
            self._db.execute("COMMIT")
        return len(seqs)

    def depth(self) -> int:
        with self._lock:
            return self._db.execute("SELECT COUNT(*) FROM outbox").fetchone()[0]

    def oldest_seq(self) -> int | None:
        with self._lock:
            return self._db.execute("SELECT MIN(seq) FROM outbox").fetchone()[0]

    def max_seq(self) -> int:
        """Highest seq ever issued (survives deletes)."""
        with self._lock:
            row = self._db.execute("SELECT seq FROM sqlite_sequence WHERE name='outbox'").fetchone()
            return row[0] if row else 0

    def close(self) -> None:
        with self._lock:
            self._db.close()


def wire_payload(seq: int, body_json: str) -> bytes:
    """Final MQTT payload: the stored body plus its sequence number (deterministic)."""
    body = json.loads(body_json)
    body["seq"] = seq
    return json.dumps(body, separators=(",", ":"), sort_keys=True).encode()
