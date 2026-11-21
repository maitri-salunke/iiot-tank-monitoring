"""Headless OpenPLC v3 provisioning through its own web UI endpoints.

Same HTTP requests a user's browser makes:
  POST /login                    (default openplc/openplc -- lab only)
  POST /upload-program           multipart .st upload -> server-side file name
  POST /upload-program-action    register program in the DB
  GET  /compile-program?file=    matiec iec2c + g++ build
  GET  /compilation-logs         poll until "Compilation finished successfully!"
  POST /add-modbus-device        simulated plant as Modbus TCP slave device
  POST /settings                 Modbus server :502, start-in-run-mode, polling 50 ms
  GET  /start_plc
Then sends START (HR 1025 bit0) to the PLC over Modbus TCP so the loop runs.
Idempotent: skips upload/compile if this exact program + config is already active.
Stdlib only (runs in OpenPLC's own venv).
"""
import hashlib
import http.cookiejar
import os
import re
import socket
import struct
import sys
import time
import urllib.parse
import urllib.request

BASE = "http://127.0.0.1:8080"
ST_PATH = "/opt/iiot/tank_control.st"
MARKER = "/docker_persistent/iiot_provisioned"
PLANT_HOST = os.environ.get("PLANT_HOST", "plant")
PLANT_PORT = os.environ.get("PLANT_MODBUS_PORT", "5020")
SLAVE_POLL_MS = os.environ.get("OPENPLC_SLAVE_POLL_MS", "50")
AUTO_START = os.environ.get("PLC_AUTO_START", "1") == "1"

jar = http.cookiejar.CookieJar()
opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))


def log(*a):
    print("[provision]", *a, flush=True)


def get(path, timeout=10):
    with opener.open(BASE + path, timeout=timeout) as r:
        return r.read().decode(errors="replace")


def post_form(path, fields):
    data = urllib.parse.urlencode(fields).encode()
    with opener.open(urllib.request.Request(BASE + path, data=data), timeout=30) as r:
        return r.read().decode(errors="replace")


def post_multipart(path, fields, files):
    boundary = "----iiot" + hashlib.md5(str(time.time()).encode()).hexdigest()
    parts = []
    for k, v in fields.items():
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{k}"\r\n\r\n{v}\r\n'.encode())
    for k, (fname, content) in files.items():
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{k}"; filename="{fname}"\r\n'
                     f"Content-Type: application/octet-stream\r\n\r\n".encode() + content + b"\r\n")
    parts.append(f"--{boundary}--\r\n".encode())
    req = urllib.request.Request(BASE + path, data=b"".join(parts),
                                 headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
    with opener.open(req, timeout=30) as r:
        return r.read().decode(errors="replace")


def wait_http(deadline_s=180):
    t0 = time.time()
    while time.time() - t0 < deadline_s:
        try:
            get("/login", timeout=2)
            return
        except Exception:
            time.sleep(1)
    raise RuntimeError("OpenPLC webserver did not come up")


def modbus_write_reg(host, port, reg, value, unit=1):
    """Raw Modbus TCP FC06 (no library): MBAP + PDU."""
    pdu = struct.pack(">BHH", 6, reg, value)
    mbap = struct.pack(">HHHB", 1, 0, len(pdu) + 1, unit)
    with socket.create_connection((host, port), timeout=3) as s:
        s.sendall(mbap + pdu)
        resp = s.recv(256)
    return resp[7] == 6


def plc_modbus_up(timeout_s):
    """True once the PLC's Modbus server on :502 answers a write, within timeout_s."""
    for _ in range(timeout_s):
        try:
            if modbus_write_reg("127.0.0.1", 502, 1024 + 1, 0):
                return True
        except OSError:
            pass
        time.sleep(1)
    return False


def main():
    wait_http()
    out = post_form("/login", {"username": "openplc", "password": "openplc"})
    if "Dashboard" not in out and "dashboard" not in out:
        raise RuntimeError("login failed")
    log("logged in")

    st = open(ST_PATH, "rb").read()
    plant_ip = socket.gethostbyname(PLANT_HOST)  # libmodbus modbus_new_tcp needs an IP literal
    fingerprint = hashlib.sha256(st + plant_ip.encode() + SLAVE_POLL_MS.encode()).hexdigest()
    already = os.path.exists(MARKER) and open(MARKER).read().strip() == fingerprint

    if not already:
        # 1) upload + register + compile
        html = post_multipart("/upload-program", {}, {"file": ("tank_control.st", st)})
        m = re.search(r"value='(\d+\.st)' id='prog_file'", html)
        if not m:
            raise RuntimeError("could not find uploaded file name")
        prog_file = m.group(1)
        post_form("/upload-program-action", {"prog_name": "tank_control", "epoch_time": str(int(time.time())),
                                             "prog_descr": "Tank level PID + interlocks (iiot-tank-monitoring)",
                                             "prog_file": prog_file})
        get(f"/compile-program?file={prog_file}")
        log("compiling", prog_file)
        t0 = time.time()
        while True:
            logs = get("/compilation-logs")
            if "Compilation finished successfully!" in logs:
                log(f"compiled OK in {time.time() - t0:.1f}s")
                break
            if "Compilation finished with errors!" in logs or time.time() - t0 > 600:
                print(logs)
                raise RuntimeError("compilation failed")
            time.sleep(2)

        # 2) slave device: the simulated plant
        modbus_page = get("/modbus")
        if "tank_plant" not in modbus_page:
            post_form("/add-modbus-device", {
                "device_name": "tank_plant", "device_protocol": "TCP", "device_id": "1",
                "device_ip": plant_ip, "device_port": PLANT_PORT,
                "device_cport": "", "device_baud": "115200", "device_parity": "None", "device_data": "8",
                "device_stop": "1", "device_pause": "0",
                "di_start": "0", "di_size": "4",       # %IX100.0..3
                "do_start": "0", "do_size": "1",       # %QX100.0
                "ai_start": "0", "ai_size": "8",       # %IW100..107  (input registers)
                "aor_start": "0", "aor_size": "0",     # holding read: none
                "aow_start": "0", "aow_size": "2",     # %QW100..101  (holding write)
            })
            log("slave device added:", plant_ip, PLANT_PORT)

        # 3) runtime settings
        post_form("/settings", {
            "modbus_server_port": "502", "auto_run_text": "true", "snap7_run_text": "false",
            "slave_polling_period": SLAVE_POLL_MS, "slave_timeout": "200",
            "device_hostname": socket.gethostname(),
        })
        log("settings applied")
        with open(MARKER, "w") as f:
            f.write(fingerprint)
    else:
        log("already provisioned with this program/config")

    # On a restarted container the runtime auto-starts in RUN mode; calling /start_plc
    # again would restart it and drop the Modbus server, so only start it if it is down.
    if not (already and plc_modbus_up(20)):
        get("/start_plc")
        # wait for the PLC Modbus server to come up and the program to scan
        if not plc_modbus_up(60):
            raise RuntimeError("PLC Modbus server not reachable")
    log("PLC running, Modbus server on :502")
    if AUTO_START:
        time.sleep(2)  # let the slave poll fill %IW / %IX
        modbus_write_reg("127.0.0.1", 502, 1024 + 1, 0x0001)
        log("START pulse sent (HR 1025 bit0)")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        log("ERROR", e)
        sys.exit(1)
