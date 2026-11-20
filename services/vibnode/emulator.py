"""SIMULATED ESP32 vibration node (Modbus RTU slave) + simulated RS-485 wire.

What is emulated, honestly:
  * the ADXL345 signal is SYNTHETIC: 1x running speed (24.75 Hz, ~1485 rpm 4-pole
    motor on 50 Hz), 2x, 5x vane-pass, white noise, 3.9 mg quantisation.
    An "imbalance fault" adds velocity at 1x (POST /control {"imbalance_mms": x}).
    A "bearing fault" adds a train of decaying 350 Hz impacts at 3.1x
    (POST /control {"bearing_g": 0.4}) -> kurtosis, crest factor and band 4 rise.
    Bearing-housing temperature is a first-order model (35 degC + heating from
    vibration severity and bearing fault, tau 60 s) -- also SIMULATED.
  * features are computed with iiot/vib_dsp.py, the host-tested twin of the
    firmware's vib_dsp.c, and served with the same register map (iiot/vib_regmap.py).
  * the serial link is a socat virtual serial pair:
        emulator  <->  /tmp/ttyNODE ==socat pty pair== /tmp/ttyBUS  <->  wire thread
    The wire thread (stands in for the RS-485 cable / a serial device server)
    exposes the bus on TCP :7000, delays bytes to 19200 baud 8N1 timing (10 bits/byte)
    and can flip bits at a configurable bit-error rate (POST /control {"ber": 1e-4}).
    The gateway attaches its own socat pty (/tmp/ttyRS485) to that TCP port.
"""
from __future__ import annotations

import json
import logging
import math
import os
import random
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from iiot import vib_regmap as R  # noqa: E402
from iiot.modbus_rtu import RtuSlave, t35_seconds  # noqa: E402
from iiot.vib_dsp import AlarmLogic, compute, iso_zone  # noqa: E402

log = logging.getLogger("vibnode")
NODE_TTY = os.environ.get("NODE_TTY", "/tmp/ttyNODE")
BUS_TTY = os.environ.get("BUS_TTY", "/tmp/ttyBUS")
WIRE_PORT = int(os.environ.get("WIRE_PORT", "7000"))
HTTP_PORT = int(os.environ.get("VIB_HTTP_PORT", "8082"))
G = 9.80665

lock = threading.Lock()
ir = [0] * R.IR_COUNT
hr = list(R.HR_DEFAULTS)
cfg = {"imbalance_mms": 0.0, "bearing_g": 0.0, "ber": float(os.environ.get("WIRE_BER", "0")), "sensor_fault": False,
       "baud": R.BAUD}
wire_stats = {"bytes_m2s": 0, "bytes_s2m": 0, "bytes_corrupted": 0}
boot = time.monotonic()


# ---------------------------------------------------------------- signal ------
class SignalSynth:
    def __init__(self, seed: int = 7):
        self.rng = np.random.default_rng(seed)
        self.t0 = 0.0
        self.f1 = 24.75
        self.t_imp0 = 0.0

    @staticmethod
    def _acc_g(v_rms_mms: float, f: float) -> float:
        return v_rms_mms * math.sqrt(2) / 1000.0 * 2 * math.pi * f / G

    def window(self, n: int, fs: float, imbalance_mms: float, bearing_g: float = 0.0):
        t = self.t0 + np.arange(n) / fs
        self.t0 += n / fs
        f1 = self.f1 + self.rng.normal(0, 0.05)
        v1 = 1.2 + imbalance_mms
        comps = [(v1, f1, 0.0), (0.45, 2 * f1, 0.7), (0.35, 5 * f1, 1.9)]
        x = sum(self._acc_g(v, f) * np.sin(2 * np.pi * f * t + ph) for v, f, ph in comps)
        y = sum(0.8 * self._acc_g(v, f) * np.sin(2 * np.pi * f * t + ph + np.pi / 2) for v, f, ph in comps)
        if bearing_g > 0:                                      # outer-race-like impact train
            bp = 1.0 / (3.1 * f1)
            tau = t - self.t_imp0
            phase = np.mod(tau, bp)
            ring = bearing_g * np.exp(-phase * 300.0) * np.sin(2 * np.pi * 350.0 * phase)
            x = x + ring
            y = y + 0.6 * ring
        z = 1.0 + 0.3 * x
        out = []
        for s in (x, y, z):
            s = s + self.rng.normal(0, 0.004, n)              # sensor + mechanical noise
            s = np.round(s / 0.0039) * 0.0039                  # ADXL345 full-res LSB
            out.append(s)
        return out


def dsp_loop() -> None:
    synth = SignalSynth()
    alarm = AlarmLogic()
    n, fs = R.WINDOW_LEN, float(R.SAMPLE_RATE_HZ)
    period = n / fs
    next_t = time.monotonic() + period
    win = 0
    temp_c = 35.0
    with lock:
        ir[R.IR_FW_VERSION] = R.FW_VERSION
        ir[R.IR_SAMPLE_RATE] = R.SAMPLE_RATE_HZ
        ir[R.IR_WINDOW_LEN] = R.WINDOW_LEN
    while True:
        time.sleep(max(0.0, next_t - time.monotonic()))
        next_t += period
        with lock:
            imb, sensor_fault, brg = cfg["imbalance_mms"], cfg["sensor_fault"], cfg["bearing_g"]
            thr = list(hr)
        x, y, z = synth.window(n, fs, imb, brg)
        f = compute(x, y, z, fs)
        target = 35.0 + 0.8 * f.vel_rms_mms + 25.0 * brg
        temp_c += (target - temp_c) * (1 - math.exp(-period / 60.0))
        temp_meas = temp_c + synth.rng.normal(0, 0.03)
        va, im = alarm.update(f, thr[R.HR_VEL_ALERT], thr[R.HR_VEL_1X_ALARM], thr[R.HR_DEBOUNCE])
        win = (win + 1) & 0xFFFF
        up = int(time.monotonic() - boot)
        st = 0 if sensor_fault else (R.STATUS_SENSOR_OK | R.STATUS_WINDOW_VALID
                                     | (R.STATUS_VEL_ALERT if va else 0) | (R.STATUS_IMBALANCE if im else 0))
        u = lambda v: max(0, min(65535, int(round(v))))  # noqa: E731
        s16 = lambda v: max(-32768, min(32767, int(round(v)))) & 0xFFFF  # noqa: E731
        with lock:
            ir[R.IR_STATUS] = st
            ir[R.IR_ACC_RMS_RADIAL] = u(f.acc_rms_x_mg)
            ir[R.IR_VEL_RMS] = u(f.vel_rms_mms * 100)
            ir[R.IR_PEAK_FREQ] = u(f.peak_freq_hz * 10)
            ir[R.IR_PEAK_AMP] = u(f.peak_amp_mg)
            ir[R.IR_VEL_1X] = u(f.vel_1x_mms * 100)
            ir[R.IR_RUN_SPEED] = u(f.run_speed_hz * 10)
            ir[R.IR_WINDOW_COUNT] = win
            ir[R.IR_UPTIME_HI] = (up >> 16) & 0xFFFF
            ir[R.IR_UPTIME_LO] = up & 0xFFFF
            ir[R.IR_ACC_RMS_X] = u(f.acc_rms_x_mg)
            ir[R.IR_ACC_RMS_Y] = u(f.acc_rms_y_mg)
            ir[R.IR_ACC_RMS_Z] = u(f.acc_rms_z_mg)
            ir[R.IR_VEL_RMS_Y] = u(f.vel_rms_y_mms * 100)
            ir[R.IR_VEL_RMS_Z] = u(f.vel_rms_z_mms * 100)
            ir[R.IR_ACC_PEAK_X] = u(f.acc_peak_x_mg)
            ir[R.IR_ACC_PEAK_Y] = u(f.acc_peak_y_mg)
            ir[R.IR_ACC_PEAK_Z] = u(f.acc_peak_z_mg)
            ir[R.IR_CREST_X] = u(f.crest_x * 100)
            ir[R.IR_CREST_Y] = u(f.crest_y * 100)
            ir[R.IR_CREST_Z] = u(f.crest_z * 100)
            ir[R.IR_KURT_X] = u(f.kurt_x * 100)
            ir[R.IR_KURT_Y] = u(f.kurt_y * 100)
            ir[R.IR_KURT_Z] = u(f.kurt_z * 100)
            ir[R.IR_BAND1_ACC] = u(f.band1_mg * 10)
            ir[R.IR_BAND2_ACC] = u(f.band2_mg * 10)
            ir[R.IR_BAND3_ACC] = u(f.band3_mg * 10)
            ir[R.IR_BAND4_ACC] = u(f.band4_mg * 10)
            ir[R.IR_VEL_2X] = u(f.vel_2x_mms * 100)
            ir[R.IR_VEL_3X] = u(f.vel_3x_mms * 100)
            ir[R.IR_VEL_VANE] = u(f.vel_5x_mms * 100)
            ir[R.IR_DC_X] = s16(f.dc_x_mg)
            ir[R.IR_DC_Y] = s16(f.dc_y_mg)
            ir[R.IR_DC_Z] = s16(f.dc_z_mg)
            ir[R.IR_TEMP] = s16(temp_meas * 100)
            ir[R.IR_CLIP_COUNT] = u(f.clip_count)
            ir[R.IR_FIFO_OVR] = 0          # no FIFO / I2C in the emulator: these stay 0
            ir[R.IR_I2C_ERR] = 0
            ir[R.IR_VA_CNT] = alarm._va_cnt
            ir[R.IR_IM_CNT] = alarm._im_cnt
            ir[R.IR_ISO_ZONE] = iso_zone(f.vel_rms_mms, R.ISO_ZONE_LIMITS_MMS)


# ---------------------------------------------------------------- slave -------
def _read_input(start, count):
    if start + count > R.IR_COUNT:
        return None
    with lock:
        return ir[start:start + count]


def _read_holding(start, count):
    if start + count > R.HR_COUNT:
        return None
    with lock:
        return hr[start:start + count]


def _write_holding(start, values):
    if start + len(values) > R.HR_COUNT:
        return False
    for i, v in enumerate(values):
        reg = start + i
        if reg == R.HR_UNIT_ID or (reg == R.HR_DEBOUNCE and not 1 <= v <= 20):
            return False
    with lock:
        hr[start:start + len(values)] = values
    return True


slave = RtuSlave(R.UNIT_ID, _read_input, _read_holding, _write_holding)


def slave_loop() -> None:
    gap = max(t35_seconds(R.BAUD), 0.004)   # pty scheduling jitter > t3.5, use >= 4 ms
    while True:
        try:
            ser = serial.Serial(NODE_TTY, R.BAUD, timeout=0)
        except serial.SerialException:
            time.sleep(0.5)
            continue
        log.info("RTU slave unit %d on %s", R.UNIT_ID, NODE_TTY)
        try:
            while True:
                # RTU framing rule: a frame ends after >= t3.5 of line silence.
                # (pyserial's posix read() ignores inter_byte_timeout, so use select.)
                r, _, _ = select.select([ser.fileno()], [], [], 1.0)
                if not r:
                    continue
                frame = ser.read(256)
                while True:
                    r, _, _ = select.select([ser.fileno()], [], [], gap)
                    if not r:
                        break
                    frame += ser.read(256)
                resp = slave.handle(frame)
                if resp:
                    ser.write(resp)
        except (serial.SerialException, OSError) as e:
            log.warning("serial error %s; reopening", e)
            time.sleep(0.5)


# ---------------------------------------------------------------- wire --------
def _corrupt(data: bytes, ber: float) -> bytes:
    if ber <= 0:
        return data
    out = bytearray(data)
    for i in range(len(out)):
        hit = False
        for bit in range(8):                 # model data-bit flips only (no framing errors)
            if random.random() < ber:
                out[i] ^= 1 << bit
                hit = True
        if hit:
            wire_stats["bytes_corrupted"] += 1
    return bytes(out)


def wire_loop() -> None:
    """Bridge /tmp/ttyBUS <-> TCP client with 19200-baud byte timing and noise."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", WIRE_PORT))
    srv.listen(1)
    while True:
        try:
            bus = serial.Serial(BUS_TTY, R.BAUD, timeout=0)
            break
        except serial.SerialException:
            time.sleep(0.2)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        log.info("wire: master attached from %s", addr)
        try:
            while True:
                r, _, _ = select.select([conn, bus.fileno()], [], [], 1.0)
                with lock:
                    ber, baud = cfg["ber"], cfg["baud"]
                byte_t = 10.0 / baud
                if conn in r:
                    data = conn.recv(512)
                    if not data:
                        break
                    time.sleep(len(data) * byte_t)            # time on the wire
                    wire_stats["bytes_m2s"] += len(data)
                    bus.write(_corrupt(data, ber))
                if bus.fileno() in r:
                    data = bus.read(512)
                    if data:
                        time.sleep(len(data) * byte_t)
                        wire_stats["bytes_s2m"] += len(data)
                        conn.sendall(_corrupt(data, ber))
        except OSError as e:
            log.warning("wire: %s", e)
        finally:
            conn.close()
            log.info("wire: master detached")


# ---------------------------------------------------------------- control -----
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

    def _state(self):
        with lock:
            return {"cfg": dict(cfg), "ir": list(ir), "hr": list(hr), "wire": dict(wire_stats),
                    "slave": {"ok": slave.frames_ok, "bad_crc": slave.frames_bad_crc,
                              "other_unit": slave.frames_other_unit}}

    def do_GET(self):
        self._send(self._state())

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        req = json.loads(self.rfile.read(n) or b"{}")
        with lock:
            for k in ("imbalance_mms", "ber", "bearing_g"):
                if k in req:
                    cfg[k] = float(req[k])
            if "sensor_fault" in req:
                cfg["sensor_fault"] = bool(req["sensor_fault"])
        log.info("control %s", req)
        self._send(self._state())


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    log.info("SIMULATED vibration node (emulates ESP32 firmware register map)")
    for fn in (dsp_loop, slave_loop, wire_loop):
        threading.Thread(target=fn, daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Api).serve_forever()


if __name__ == "__main__":
    main()
