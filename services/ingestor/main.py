"""MQTT -> InfluxDB 2 ingestor (the "IT side" subscriber).

  * persistent session (clean_session=False, fixed client id): the broker queues
    QoS 1 messages for us while we are away.
  * manual acks: PUBACK is sent only *after* InfluxDB confirmed the write, so a
    crash between receive and write causes redelivery, never loss.
  * idempotent writes: point time = poll timestamp inside the message, so a
    redelivered message overwrites the identical point.
  * duplicate accounting: every seq is recorded in SQLite; a seq seen twice is
    counted as a duplicate *delivery* (expected under QoS 1, harmless).
  * latency: after each successful write, e2e = write-ack time - plant sample
    time (stamped at the simulated I/O and passed through the PLC registers).
  * batching without a batching delay: the writer blocks for the first message,
    then takes whatever else is *already* queued (up to BATCH_MAX) and writes at
    once.  Live traffic -> batches of 1, ~1 Influx round trip of delay; a
    reconnect backlog -> large batches automatically.  (The earlier version
    waited a fixed 50 ms for more messages: pure added latency at 2 msg/s.)
  * the pipeline_latency points are handed to a second thread, so their write
    is not in front of the next telemetry message.
"""
from __future__ import annotations

import json
import logging
import os
import queue
import socket
import sqlite3
import ssl
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from iiot.telemetry import to_lines  # noqa: E402

log = logging.getLogger("ingestor")
E = os.environ.get
SITE = E("SITE", "site1")
BUCKET, ORG = E("INFLUX_BUCKET", "iiot"), E("INFLUX_ORG", "iiot")
BATCH_MAX, BATCH_WAIT_S = int(E("BATCH_MAX", "500")), float(E("BATCH_WAIT_S", "0"))

inbox: "queue.Queue[mqtt.MQTTMessage]" = queue.Queue()
lat_q: "queue.Queue[list[str]]" = queue.Queue()
stats = {"received": 0, "written_msgs": 0, "duplicates": 0, "write_errors": 0, "bad_payload": 0,
         "last_seq": None, "batches": 0, "mqtt_connects": 0, "mqtt_disconnects": 0}


class SeenStore:
    def __init__(self, path: str):
        self.db = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=NORMAL")   # dup accounting only; no fsync per batch
        self.db.execute("CREATE TABLE IF NOT EXISTS seen (seq INTEGER PRIMARY KEY, first_ms INTEGER, n INTEGER)")

    def mark(self, seqs: list[int], now_ms: int) -> list[bool]:
        """Return per-seq True if it is a duplicate delivery."""
        dup = []
        self.db.execute("BEGIN")
        for s in seqs:
            cur = self.db.execute("INSERT OR IGNORE INTO seen (seq, first_ms, n) VALUES (?, ?, 1)", (s, now_ms))
            if cur.rowcount == 0:
                self.db.execute("UPDATE seen SET n = n + 1 WHERE seq = ?", (s,))
                dup.append(True)
            else:
                dup.append(False)
        self.db.execute("COMMIT")
        return dup


def make_client() -> mqtt.Client:
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ingestor-1", protocol=mqtt.MQTTv311,
                    clean_session=False, manual_ack=True)
    c.tls_set(ca_certs=E("MQTT_CA", "/certs/ca.crt"), certfile=E("MQTT_CERT", "/certs/ingestor.crt"),
              keyfile=E("MQTT_KEY", "/certs/ingestor.key"), tls_version=ssl.PROTOCOL_TLS_CLIENT)
    c.reconnect_delay_set(min_delay=1, max_delay=4)

    def on_connect(client, userdata, flags, reason_code, properties):
        stats["mqtt_connects"] += 1
        log.info("MQTT connected session_present=%s", flags.session_present)
        try:   # PUBACKs are tiny; do not let Nagle hold them back
            client.socket().setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, AttributeError):
            pass
        client.subscribe([(f"iiot/{SITE}/+/telemetry", 1), (f"iiot/{SITE}/+/status", 1),
                          (f"iiot/{SITE}/+/health", 0)])

    def on_message(client, userdata, msg):
        inbox.put(msg)

    def on_disconnect(client, userdata, flags, reason_code, properties):
        stats["mqtt_disconnects"] += 1
        log.warning("MQTT disconnected: %s (inbox=%d)", reason_code, inbox.qsize())

    c.on_connect, c.on_message, c.on_disconnect = on_connect, on_message, on_disconnect
    return c


def health_lines(topic: str, payload: dict) -> list[str]:
    gw = payload.get("gw", "gw1")
    if topic.endswith("/status"):
        v = 1 if payload.get("state") == "online" else 0
        return [f"gateway_status,site={SITE},gateway={gw} online={v}i {int(time.time() * 1000)}"]
    ts = int(payload.get("ts", time.time() * 1000))
    f = {"outbox_depth": payload.get("outbox_depth", 0), "seq_max": payload.get("seq_max", 0),
         "published": payload.get("published", 0), "acked": payload.get("acked", 0),
         "rtu_crc_error": payload.get("rtu", {}).get("crc_error", 0),
         "rtu_timeout": payload.get("rtu", {}).get("timeout", 0), "rtu_polls": payload.get("rtu", {}).get("polls", 0)}
    fs = ",".join(f"{k}={int(v)}i" for k, v in f.items())
    return [f"gateway_health,site={SITE},gateway={gw} {fs} {ts}"]


def writer_loop(client: mqtt.Client, write_api, seen: SeenStore) -> None:
    while True:
        batch = [inbox.get()]
        deadline = time.monotonic() + BATCH_WAIT_S
        while len(batch) < BATCH_MAX:
            try:
                wait = deadline - time.monotonic()
                batch.append(inbox.get(timeout=wait) if wait > 0 else inbox.get_nowait())
            except queue.Empty:
                break
        lines, tele = [], []
        for m in batch:
            try:
                p = json.loads(m.payload)
            except json.JSONDecodeError:
                stats["bad_payload"] += 1
                continue
            if m.topic.endswith("/telemetry"):
                lines += to_lines(p)
                tele.append(p)
            else:
                lines += health_lines(m.topic, p)
        stats["received"] += len(batch)
        while lines:
            try:
                write_api.write(bucket=BUCKET, org=ORG, record=lines, write_precision="ms")
                break
            except Exception as e:  # Influx down: keep batch, do not ack -> no loss
                stats["write_errors"] += 1
                log.warning("influx write failed (%s); retrying", e)
                time.sleep(1.0)
        ack_ms = int(time.time() * 1000)
        if tele:
            dups = seen.mark([int(p["seq"]) for p in tele], ack_ms)
            lat = []
            for p, d in zip(tele, dups):
                if d:
                    stats["duplicates"] += 1
                    continue
                f = {"seq": f"{int(p['seq'])}i", "gw_ms": f"{ack_ms - int(p['ts'])}i"}
                age = int(p["ts"]) - int(p["plc"]["src_ts_ms"]) if p.get("plc") else None
                # Guard: right after a plant/PLC restart the PLC passes through a zeroed
                # timestamp (registers not yet filled) -> nonsense "ages" of days. Skip those.
                if age is not None and -1_000 <= age <= 60_000:
                    f["e2e_ms"] = f"{ack_ms - int(p['plc']['src_ts_ms'])}i"
                    f["src_age_ms"] = f"{age}i"
                fs = ",".join(f"{k}={v}" for k, v in f.items())
                lat.append(f"pipeline_latency,site={SITE} {fs} {int(p['ts'])}")
            if lat:
                lat_q.put(lat)
            stats["written_msgs"] += len(tele)
            stats["last_seq"] = int(tele[-1]["seq"])
        stats["batches"] += 1
        for m in batch:
            client.ack(m.mid, m.qos)


def latency_writer(write_api) -> None:
    """Writes pipeline_latency points (measurement metadata) off the critical path."""
    while True:
        lines = lat_q.get()
        while len(lines) < 5000:
            try:
                lines += lat_q.get_nowait()
            except queue.Empty:
                break
        try:
            write_api.write(bucket=BUCKET, org=ORG, record=lines, write_precision="ms")
        except Exception as e:
            log.warning("latency write failed: %s", e)


class Api(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        body = json.dumps({**stats, "inbox": inbox.qsize()}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    influx = InfluxDBClient(url=E("INFLUX_URL", "http://influxdb:8086"), token=E("INFLUX_TOKEN"), org=ORG,
                            timeout=10_000)
    write_api = influx.write_api(write_options=SYNCHRONOUS)
    seen = SeenStore(E("SEEN_PATH", "/data/seen.db"))
    client = make_client()
    client.connect_async(E("MQTT_HOST", "mosquitto"), int(E("MQTT_PORT", "8883")), keepalive=10)
    client.loop_start()
    threading.Thread(target=writer_loop, args=(client, write_api, seen), daemon=True).start()
    # separate client (own HTTP connection pool) for the metadata writes
    lat_api = InfluxDBClient(url=E("INFLUX_URL", "http://influxdb:8086"), token=E("INFLUX_TOKEN"), org=ORG,
                             timeout=10_000).write_api(write_options=SYNCHRONOUS)
    threading.Thread(target=latency_writer, args=(lat_api,), daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", int(E("METRICS_PORT", "9101"))), Api).serve_forever()


if __name__ == "__main__":
    main()
