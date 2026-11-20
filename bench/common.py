"""Helpers shared by the measurement scripts (run from the host venv)."""
from __future__ import annotations

import csv
import io
import json
import os
import subprocess
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")
INFLUX = os.environ.get("INFLUX_URL", "http://localhost:8086")
TOKEN = os.environ.get("INFLUX_TOKEN", "iiot-lab-token-not-a-secret")
ORG = "iiot"


def http_json(url: str, body: dict | None = None, timeout: float = 5.0) -> dict:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def flux(query: str) -> list[dict]:
    req = urllib.request.Request(
        f"{INFLUX}/api/v2/query?org={ORG}", data=json.dumps({"query": query, "type": "flux"}).encode(),
        headers={"Authorization": f"Token {TOKEN}", "Content-Type": "application/json",
                 "Accept": "application/csv"})
    with urllib.request.urlopen(req, timeout=120) as r:
        text = r.read().decode()
    rows = []
    for block in text.strip().split("\r\n\r\n"):
        lines = [l for l in block.splitlines() if l and not l.startswith("#")]
        if len(lines) < 2:
            continue
        rows += list(csv.DictReader(io.StringIO("\n".join(lines))))
    return rows


def docker(*args: str, check: bool = True) -> str:
    return subprocess.run(["docker", *args], check=check, capture_output=True, text=True).stdout


def save(name: str, obj: dict) -> str:
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, name)
    obj = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), **obj}
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
    print("saved", path)
    return path


def pct(values: list[float], p: float) -> float:
    """Nearest-rank percentile."""
    if not values:
        return float("nan")
    s = sorted(values)
    k = max(0, min(len(s) - 1, int(round(p / 100.0 * len(s) + 0.5)) - 1))
    return s[k]
