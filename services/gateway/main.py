"""Edge gateway: Modbus TCP (OpenPLC) + Modbus RTU (vibration node) -> SQLite
outbox -> MQTT/TLS (QoS 1, LWT, retained status).

Threads
  plc poller   fixed-rate Modbus TCP poll of OpenPLC (PLC_POLL_HZ, default 20 Hz):
               FC03 HR 0..49 + FC01 coils 0..25 = 2 requests per cycle
  vib poller   fixed-rate Modbus RTU poll of the vibration node (VIB_POLL_HZ, default 2 Hz):
               FC04 IR 0..43 = 1 request per cycle.  Its own thread, so the ~55 ms
               RS-485 transaction never delays the PLC path.
  Each poll goes through a report-by-exception filter (iiot/rbe.py: per-tag
  deadbands + full integrity snapshot every INTEGRITY_S).  Only polls with
  something to report become an outbox row = one seq.
  drainer      moves outbox rows to paho in seq order, bounded in-flight window,
               deletes rows on PUBACK, measures backfill time after reconnects.
               Woken by an Event on enqueue / PUBACK (no polling sleep in the path).
  paho       network loop (reconnects automatically)
  http       GET /metrics (JSON), POST /rtu {"paused": true} for the RTU bench

Read-only by design: the gateway never writes to the PLC. Commands (and above
all the E-stop) stay in the PLC / hardwired layer.
"""
from __future__ import annotations

import collections
import json
import logging
import os
import ssl
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import socket

import paho.mqtt.client as mqtt
import serial
from pymodbus.client import ModbusTcpClient

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from iiot import vib_regmap as R  # noqa: E402
from iiot.modbus_rtu import FC_READ_INPUT, PollResult, PollStats, RtuMaster  # noqa: E402
from iiot.outbox import Outbox, wire_payload  # noqa: E402
from iiot.rbe import RbeFilter  # noqa: E402
from iiot.telemetry import (PLC_ALWAYS, PLC_COIL_COUNT, PLC_COIL_START, PLC_DEADBAND,  # noqa: E402
                            PLC_HR_COUNT, PLC_HR_START, TAG_COUNT, VIB_ALWAYS, VIB_DEADBAND, decode_plc,
                            decode_vib)

log = logging.getLogger("gateway")
E = os.environ.get
SITE, ASSET, GW = E("SITE", "site1"), E("ASSET", "tank1"), E("GATEWAY_ID", "gw1")
TOPIC_TELEMETRY = f"iiot/{SITE}/{ASSET}/telemetry"
TOPIC_STATUS = f"iiot/{SITE}/{GW}/status"
TOPIC_HEALTH = f"iiot/{SITE}/{GW}/health"
PLC_POLL_HZ = float(E("PLC_POLL_HZ", E("POLL_HZ", "20")))
VIB_POLL_HZ = float(E("VIB_POLL_HZ", "2"))
RBE_ON = E("RBE", "1") == "1"
INTEGRITY_S = float(E("INTEGRITY_S", "5"))
MAX_INFLIGHT = int(E("MQTT_MAX_INFLIGHT", "100"))

outbox = Outbox(E("OUTBOX_PATH", "/data/outbox.db"))
rtu_paused = threading.Event()
ack_q: collections.deque[int] = collections.deque()
wake = threading.Event()          # set on enqueue and on PUBACK -> drainer runs at once
state = {
    "connected": False, "connects": 0, "disconnects": 0, "last_connect_ms": None,
    "drains": [], "poll_cycles": 0, "poll_overruns": 0, "cycle_ms_max": 0.0,
    "vib_cycles": 0, "vib_overruns": 0, "enqueued": 0,
    "published": 0, "acked": 0, "started_ms": int(time.time() * 1000),
}
rbe = {"plc": RbeFilter(PLC_DEADBAND, PLC_ALWAYS, INTEGRITY_S, RBE_ON),
       "vib": RbeFilter(VIB_DEADBAND, VIB_ALWAYS, INTEGRITY_S, RBE_ON)}
plc_stats = PollStats()
drain_mark: dict = {}


# ------------------------------------------------------------------ MQTT -----
def make_client() -> mqtt.Client:
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"gateway-{GW}",
                    protocol=mqtt.MQTTv311, clean_session=True)
    c.tls_set(ca_certs=E("MQTT_CA", "/certs/ca.crt"), certfile=E("MQTT_CERT", "/certs/gateway.crt"),
              keyfile=E("MQTT_KEY", "/certs/gateway.key"), tls_version=ssl.PROTOCOL_TLS_CLIENT)
    c.will_set(TOPIC_STATUS, json.dumps({"state": "offline", "gw": GW, "reason": "lwt"}), qos=1, retain=True)
    c.max_inflight_messages_set(MAX_INFLIGHT)
    c.max_queued_messages_set(0)
    c.reconnect_delay_set(min_delay=1, max_delay=4)

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log.warning("connect refused: %s", reason_code)
            return
        now = int(time.time() * 1000)
        state["connected"] = True
        state["connects"] += 1
        state["last_connect_ms"] = now
        try:   # paho does not disable Nagle; small QoS 1 PUBLISHes must not wait for an ACK
            client.socket().setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, AttributeError):
            pass
        client.publish(TOPIC_STATUS, json.dumps({"state": "online", "gw": GW, "ts": now}), qos=1, retain=True)
        depth = outbox.depth()
        drain_mark.clear()
        drain_mark.update({"t0": time.monotonic(), "t0_ms": now, "mark_seq": outbox.max_seq(), "backlog": depth})
        log.info("MQTT connected (TLS); backlog=%d", depth)

    def on_disconnect(client, userdata, flags, reason_code, properties):
        state["connected"] = False
        state["disconnects"] += 1
        log.warning("MQTT disconnected: %s", reason_code)

    def on_publish(client, userdata, mid, reason_code, properties):
        ack_q.append(mid)   # never take our locks here (paho holds its own)
        wake.set()          # Event has its own private lock; it never waits on paho

    c.on_connect, c.on_disconnect, c.on_publish = on_connect, on_disconnect, on_publish
    return c


def drainer(client: mqtt.Client) -> None:
    inflight: dict[int, int] = {}       # mid -> seq (touched by this thread only)
    last_handed = 0
    last_health = 0.0
    while True:
        wake.clear()        # clear BEFORE looking for work: a set() after this is never lost
        did = False
        acked = []
        while ack_q:
            seq = inflight.pop(ack_q.popleft(), None)
            if seq is not None:
                acked.append(seq)
        if acked:
            outbox.ack(acked)
            state["acked"] += len(acked)
            did = True
        if state["connected"] and len(inflight) < MAX_INFLIGHT:
            rows = outbox.fetch_after(last_handed, MAX_INFLIGHT - len(inflight))
            for seq, _ts, topic, body in rows:
                info = client.publish(topic, wire_payload(seq, body), qos=1)
                inflight[info.mid] = seq
                last_handed = seq
                state["published"] += 1
            did = did or bool(rows)
        # backfill accounting: all rows that existed at reconnect are acked
        if drain_mark and drain_mark.get("backlog", 0) > 0:
            oldest = outbox.oldest_seq()
            if oldest is None or oldest > drain_mark["mark_seq"]:
                d = {"reconnect_ms": drain_mark["t0_ms"], "backlog": drain_mark["backlog"],
                     "backfill_s": round(time.monotonic() - drain_mark["t0"], 3)}
                state["drains"] = (state["drains"] + [d])[-50:]
                log.info("backfill done: %s", d)
                drain_mark.clear()
        if time.monotonic() - last_health > 5 and state["connected"]:
            last_health = time.monotonic()
            client.publish(TOPIC_HEALTH, json.dumps(health()), qos=0, retain=True)
        if not did:
            wake.wait(0.05)


# ------------------------------------------------------------------ polling --
def open_rtu() -> RtuMaster | None:
    try:
        ser = serial.Serial(E("RTU_PORT", "/tmp/ttyRS485"), R.BAUD, timeout=0.02)
        return RtuMaster(ser, baud=R.BAUD, timeout_s=float(E("RTU_TIMEOUT_S", "0.2")))
    except serial.SerialException as e:
        log.warning("RTU port: %s", e)
        return None


rtu_holder: dict = {"m": None, "stats": PollStats()}


def poll_vib():
    m = rtu_holder["m"]
    if m is None:
        m = open_rtu()
        if m is None:
            return "no_port", None
        m.stats = rtu_holder["stats"]
        rtu_holder["m"] = m
    try:
        res, regs = m.read_registers(R.UNIT_ID, FC_READ_INPUT, 0, R.IR_COUNT)
    except (serial.SerialException, OSError) as e:
        log.warning("RTU error %s", e)
        rtu_holder["m"] = None
        return "port_error", None
    return res.value, (decode_vib(regs) if res is PollResult.OK else None)


def poll_plc(cli: ModbusTcpClient, now_ms: int):
    t0 = time.monotonic()
    try:
        if not cli.connected:
            cli.connect()
        hr = cli.read_holding_registers(PLC_HR_START, count=PLC_HR_COUNT, slave=1)
        co = cli.read_coils(PLC_COIL_START, count=PLC_COIL_COUNT, slave=1)
        if hr.isError() or co.isError():
            plc_stats.record(PollResult.EXCEPTION, 0)
            return "exception", None
        plc_stats.record(PollResult.OK, time.monotonic() - t0)
        return "ok", decode_plc(hr.registers, co.bits, now_ms)
    except Exception as e:  # pymodbus raises ConnectionException / ModbusIOException
        plc_stats.record(PollResult.TIMEOUT, 0)
        log.debug("PLC poll failed: %s", e)
        cli.close()
        return "timeout", None


def emit(src: str, q: str, fields: dict | None, now_ms: int, now_s: float, t0: float, last_q: dict) -> None:
    """Report-by-exception for one source; enqueue a row only if there is news."""
    body = {"ts": now_ms, "site": SITE, "asset": ASSET, "gw": GW, "q": {src: q}}
    if fields is None:
        # bad poll: publish the quality change at once (and repeat it at the integrity
        # rate); forget published state so the first good poll is a full snapshot
        rbe[src].reset()
        if last_q.get("q") == q and now_s - last_q.get("t", 0) < INTEGRITY_S:
            return
        body["kind"] = "quality"
    else:
        out, kind = rbe[src].update(fields, now_s)
        if out is None and last_q.get("q") == q:
            return
        body[src], body["kind"] = (out or {}), kind
    last_q.update(q=q, t=now_s)
    body["poll_ms"] = round((time.monotonic() - t0) * 1000, 2)
    outbox.enqueue(TOPIC_TELEMETRY, now_ms, body)
    state["enqueued"] += 1
    wake.set()


def fixed_rate(hz: float, fn, overrun_key: str, cycles_key: str) -> None:
    period = 1.0 / hz
    next_t = time.monotonic()
    while True:
        fn()
        state[cycles_key] += 1
        next_t += period
        delay = next_t - time.monotonic()
        if delay < 0:
            state[overrun_key] += 1
            next_t = time.monotonic()
        else:
            time.sleep(delay)


def plc_poller() -> None:
    cli = ModbusTcpClient(E("PLC_HOST", "openplc"), port=int(E("PLC_PORT", "502")), timeout=0.5, retries=0)
    last_q: dict = {}

    def cycle():
        t0 = time.monotonic()
        now_ms = int(time.time() * 1000)
        q, plc = poll_plc(cli, now_ms)
        emit("plc", q, plc, now_ms, t0, t0, last_q)
        state["cycle_ms_max"] = max(state["cycle_ms_max"], (time.monotonic() - t0) * 1000)
    fixed_rate(PLC_POLL_HZ, cycle, "poll_overruns", "poll_cycles")


def vib_poller() -> None:
    last_q: dict = {}

    def cycle():
        t0 = time.monotonic()
        now_ms = int(time.time() * 1000)
        if rtu_paused.is_set():
            q, vib = "paused", None
        else:
            q, vib = poll_vib()
        emit("vib", q, vib, now_ms, t0, t0, last_q)
    fixed_rate(VIB_POLL_HZ, cycle, "vib_overruns", "vib_cycles")


# ------------------------------------------------------------------ http -----
def health() -> dict:
    return {
        "gw": GW, "ts": int(time.time() * 1000), "connected": state["connected"],
        "outbox_depth": outbox.depth(), "seq_max": outbox.max_seq(), "outbox_dropped": outbox.dropped,
        "published": state["published"], "acked": state["acked"],
        "connects": state["connects"], "disconnects": state["disconnects"],
        "poll_cycles": state["poll_cycles"], "poll_overruns": state["poll_overruns"],
        "cycle_ms_max": round(state["cycle_ms_max"], 2), "poll_hz": PLC_POLL_HZ,
        "vib_poll_hz": VIB_POLL_HZ, "vib_cycles": state["vib_cycles"], "vib_overruns": state["vib_overruns"],
        "enqueued": state["enqueued"], "tags": TAG_COUNT, "rbe_on": RBE_ON, "integrity_s": INTEGRITY_S,
        "rbe": {k: dict(v.stats) for k, v in rbe.items()},
        "rtu": rtu_holder["stats"].as_dict(), "plc": plc_stats.as_dict(),
    }


class Api(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        h = health()
        h["drains"] = state["drains"]
        h["rtu_paused"] = rtu_paused.is_set()
        self._send(h)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        req = json.loads(self.rfile.read(n) or b"{}")
        if "paused" in req:
            if req["paused"]:
                rtu_paused.set()
                time.sleep(0.6)       # let an in-progress RTU transaction finish
                if rtu_holder["m"] is not None:
                    try:
                        rtu_holder["m"].port.close()
                    except Exception:
                        pass
                    rtu_holder["m"] = None
            else:
                rtu_paused.clear()
        self._send({"rtu_paused": rtu_paused.is_set()})


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("pymodbus").setLevel(logging.CRITICAL)   # we count failures ourselves
    client = make_client()
    client.connect_async(E("MQTT_HOST", "mosquitto"), int(E("MQTT_PORT", "8883")), keepalive=10)
    client.loop_start()
    threading.Thread(target=drainer, args=(client,), daemon=True).start()
    threading.Thread(target=plc_poller, daemon=True).start()
    threading.Thread(target=vib_poller, daemon=True).start()
    log.info("gateway %s: PLC %.1f Hz, RTU %.1f Hz, %d tags, RBE=%s integrity=%.1fs; outbox seq=%d depth=%d",
             GW, PLC_POLL_HZ, VIB_POLL_HZ, TAG_COUNT, RBE_ON, INTEGRITY_S, outbox.max_seq(), outbox.depth())
    ThreadingHTTPServer(("0.0.0.0", int(E("METRICS_PORT", "9100"))), Api).serve_forever()


if __name__ == "__main__":
    main()
