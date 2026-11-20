"""SIMULATED plant I/O: tank model exposed as a Modbus TCP slave (pymodbus).

OpenPLC polls this as a "slave device" (software-in-the-loop). A small HTTP
API on :8081 is the *test harness*: it moves the outlet valve, presses the
field E-stop, or makes the pump run dry. It is not part of the control path.

Register map (unit 1, port 5020) -- see README "Plant I/O map":
  DI 0 ESTOP_OK, 1 PUMP_FB, 2 LSHH, 3 LSLL
  IR 0 level %x100, 1 inflow L/min x10, 2 outflow L/min x10, 3 motor current A x100,
     4/5 sample time ms (hi/lo of uint32), 6 sample counter, 7 valve % x100
  CO 0 pump run command          (written by PLC)
  HR 0 pump speed ref % x100, 1 PLC heartbeat   (written by PLC)
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext  # noqa: E402
from pymodbus.server import StartAsyncTcpServer  # noqa: E402

from iiot.plant_model import TankModel, TankState  # noqa: E402

log = logging.getLogger("plant")
DT = float(os.environ.get("PLANT_DT_S", "0.02"))          # 50 Hz physics
MB_PORT = int(os.environ.get("PLANT_MODBUS_PORT", "5020"))
HTTP_PORT = int(os.environ.get("PLANT_HTTP_PORT", "8081"))

lock = threading.Lock()
model = TankModel(state=TankState(h_m=float(os.environ.get("PLANT_INIT_LEVEL", "0.30"))))
counter = 0


def _blk():
    return ModbusSequentialDataBlock(0, [0] * 32)


slave = ModbusSlaveContext(di=_blk(), co=_blk(), hr=_blk(), ir=_blk())
context = ModbusServerContext(slaves=slave, single=True)


def u16(x: float) -> int:
    return max(0, min(65535, int(round(x))))


async def physics_loop() -> None:
    global counter
    next_t = time.monotonic()
    while True:
        with lock:
            # outputs written by the PLC (Modbus master)
            model.s.run_cmd = bool(slave.getValues(1, 0, 1)[0])
            hr = slave.getValues(3, 0, 2)
            model.s.speed_ref_pct = hr[0] / 100.0
            model.set_heartbeat(hr[1])
            model.step(DT)
            counter = (counter + 1) & 0xFFFF
            ts_ms = int(time.time() * 1000) & 0xFFFFFFFF
            ir = [
                u16(model.level_pct_meas() * 100),
                u16(model.flow_in_lpm() * 10),
                u16(model.flow_out_lpm() * 10),
                u16(model.motor_current_a() * 100),
                (ts_ms >> 16) & 0xFFFF,
                ts_ms & 0xFFFF,
                counter,
                u16(model.s.valve_pct * 100),
            ]
            di = [not model.s.estop_pressed, model.pump_running_fb(), model.lshh(), model.lsll()]
            # single setValues per table keeps hi/lo timestamp words consistent for a reader
            slave.setValues(4, 0, ir)
            slave.setValues(2, 0, di)
        next_t += DT
        await asyncio.sleep(max(0.0, next_t - time.monotonic()))


def state_dict() -> dict:
    s = model.s
    return {
        "level_pct": round(model.level_pct_true(), 3), "valve_pct": s.valve_pct,
        "speed_pct": round(s.speed_pct, 2), "speed_ref_pct": s.speed_ref_pct, "run_cmd": s.run_cmd,
        "pump_fb": model.pump_running_fb(), "estop_pressed": s.estop_pressed,
        "dry_run_fault": s.dry_run_fault, "hb_age_s": round(s.hb_age_s, 3),
        "qin_lps": round(s.qin_lps, 4), "qout_lps": round(s.qout_lps, 4), "counter": counter,
    }


class Api(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code: int, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        with lock:
            self._send(200, state_dict())

    def do_POST(self):
        """POST /control {"valve_pct": 60, "estop": true, "dry_run": true, "level_pct": 40}"""
        n = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return self._send(400, {"error": "bad json"})
        with lock:
            if "valve_pct" in req:
                model.s.valve_pct = float(min(max(req["valve_pct"], 0), 100))
            if "estop" in req:
                model.s.estop_pressed = bool(req["estop"])
            if "dry_run" in req:
                model.s.dry_run_fault = bool(req["dry_run"])
            if "level_pct" in req:   # test-harness reset of the physical level
                model.s.h_m = float(req["level_pct"]) / 100.0 * model.p.height_m
            log.info("control %s", req)
            self._send(200, state_dict())


async def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Api)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log.info("SIMULATED tank plant: Modbus TCP :%d, test API :%d, dt=%.3fs", MB_PORT, HTTP_PORT, DT)
    asyncio.create_task(physics_loop())
    await StartAsyncTcpServer(context=context, address=("0.0.0.0", MB_PORT))


if __name__ == "__main__":
    asyncio.run(main())
