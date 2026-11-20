"""Functional test of the PLC interlocks on the running system (SIMULATED plant).

For each case: start the pump, inject the condition through the plant's test
API, and time how long until the PLC drops PUMP_RUN and sets the expected bit.
    .venv/bin/python bench/interlock_test.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pymodbus.client import ModbusTcpClient  # noqa: E402

from bench.common import http_json, save  # noqa: E402

PLANT = "http://localhost:8081/control"
BITS = {"run": 0x1, "hh": 0x8, "dry": 0x20, "estop": 0x40, "permit": 0x80, "io": 0x200}
c = ModbusTcpClient("localhost", port=5502)


def status():
    return c.read_holding_registers(10, count=1, slave=1).registers[0]


def cmd(bit):
    c.write_register(1025, bit, slave=1)


def start_running(level=None):
    http_json(PLANT, {"estop": False, "dry_run": False, "valve_pct": 50, **({"level_pct": level} if level else {})})
    time.sleep(0.5)
    cmd(0x4); time.sleep(0.3); cmd(0x1)
    t0 = time.time()
    while not status() & BITS["run"]:
        if time.time() - t0 > 5:
            raise RuntimeError(f"pump did not start, status={status():#x}")
        time.sleep(0.05)
    time.sleep(2)


def wait_trip(expect_bit, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = status()
        if not s & BITS["run"] and s & expect_bit:
            return round(time.time() - t0, 2), s
        time.sleep(0.02)
    return None, status()


def main():
    c.connect()
    results = []

    start_running()
    http_json(PLANT, {"estop": True}); t, s = wait_trip(BITS["estop"])
    results.append({"case": "field E-stop pressed", "tripped": t is not None, "t_s": t, "status": hex(s),
                    "note": "contactor drops in the field; PLC sees ESTOP_OK=0 and drops seal-in"})
    http_json(PLANT, {"estop": False}); time.sleep(0.5)
    results.append({"case": "no auto-restart after E-stop released", "tripped": not status() & BITS["run"],
                    "t_s": None, "status": hex(status())})

    start_running()
    http_json(PLANT, {"dry_run": True}); t, s = wait_trip(BITS["dry"])
    results.append({"case": "dry run (suction lost)", "tripped": t is not None, "t_s": t, "status": hex(s),
                    "note": "TON 3 s on run+fb+speed>=20%+inflow<2 L/min, latched"})
    http_json(PLANT, {"dry_run": False}); time.sleep(0.5)
    still = bool(status() & BITS["dry"])
    cmd(0x4); time.sleep(0.5)
    results.append({"case": "dry-run latch holds until RESET", "tripped": still and not status() & BITS["dry"],
                    "t_s": None, "status": hex(status())})

    start_running()
    c.write_register(1024, 9000, slave=1)                 # SP 90 % (max allowed)
    http_json(PLANT, {"level_pct": 96.0}); t, s = wait_trip(BITS["hh"])
    results.append({"case": "high-high level (LSHH / >=95 %)", "tripped": t is not None, "t_s": t, "status": hex(s)})
    c.write_register(1024, 5000, slave=1)
    http_json(PLANT, {"level_pct": 50.0})

    start_running()
    os.system("docker pause iiot-plant >/dev/null")
    t, s = wait_trip(BITS["io"])
    os.system("docker unpause iiot-plant >/dev/null")
    results.append({"case": "plant I/O frozen (docker pause plant)", "tripped": t is not None, "t_s": t, "status": hex(s),
                    "note": "sample counter stale 2 s -> IO_FAULT -> permissive lost"})

    start_running(level=50.0)
    for r in results:
        print(r)
    save("interlocks.json", {"method": "faults injected via plant test API / docker pause; PLC status word polled "
                                       "at ~50 Hz over Modbus TCP; t_s = injection -> PUMP_RUN false with expected bit",
                             "cases": results, "all_passed": all(r["tripped"] for r in results)})


if __name__ == "__main__":
    main()
