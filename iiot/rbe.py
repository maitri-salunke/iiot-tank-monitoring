"""Report-by-exception (publish on change) with per-tag deadbands and a periodic
integrity snapshot.  Pure logic, no I/O (tests/test_rbe.py).

Per poll, `RbeFilter.update(fields, now_s)` returns what to publish:

  * the first good sample after start-up or after a bad-quality poll -> full
    snapshot ("integrity"), so a consumer never holds a stale partial picture;
  * every `integrity_s` seconds -> full snapshot, even if nothing moved
    (proves liveness, refreshes counters/clocks, bounds how stale any tag can be);
  * otherwise only the tags whose value moved by MORE than their deadband from
    the last *published* value (not the last polled value -- comparing against
    the last poll lets a slow ramp creep forever without being reported);
  * tags with deadband None never trigger on their own (free-running counters,
    clocks); tags in `always` ride along on every message (e.g. the sample
    timestamp, needed for latency accounting);
  * nothing (None) when no tag exceeded its deadband.

Booleans / enums / words use deadband 0 = any change.  Unknown fields default to 0.
"""
from __future__ import annotations


class RbeFilter:
    def __init__(self, deadbands: dict[str, float | None], always: tuple[str, ...] = (),
                 integrity_s: float = 5.0, enabled: bool = True):
        self.deadbands = deadbands
        self.always = always
        self.integrity_s = integrity_s
        self.enabled = enabled
        self.last: dict | None = None      # last published value per tag
        self.last_full_s = float("-inf")
        self.stats = {"polls": 0, "integrity": 0, "change": 0, "suppressed": 0,
                      "fields_polled": 0, "fields_sent": 0}

    def reset(self) -> None:
        """Forget published state (after a bad poll): next good poll is a full snapshot."""
        self.last = None

    def _moved(self, name: str, new, old) -> bool:
        db = self.deadbands.get(name, 0)
        if db is None:
            return False
        if isinstance(new, bool) or isinstance(old, bool) or not isinstance(new, (int, float)):
            return new != old
        return abs(new - old) > db if db > 0 else new != old

    def update(self, fields: dict, now_s: float) -> tuple[dict | None, str]:
        st = self.stats
        st["polls"] += 1
        st["fields_polled"] += len(fields)
        if (not self.enabled or self.last is None or now_s - self.last_full_s >= self.integrity_s):
            self.last = dict(fields)
            self.last_full_s = now_s
            st["integrity"] += 1
            st["fields_sent"] += len(fields)
            return dict(fields), "integrity"
        out = {}
        for k, v in fields.items():
            if k not in self.last or self._moved(k, v, self.last[k]):
                out[k] = v
        if not out:
            st["suppressed"] += 1
            return None, "none"
        self.last.update(out)
        for k in self.always:
            if k in fields:
                out[k] = fields[k]
        st["change"] += 1
        st["fields_sent"] += len(out)
        return out, "change"
