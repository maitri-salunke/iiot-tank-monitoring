# Design notes: every component, and why it is built this way

This is a **software-in-the-loop (SIL)** testbed. Nothing here touches real
hardware. The tank, the pump, the field switches, the ADXL345 accelerometer and
the RS-485 cable are all simulated. The PLC runtime (OpenPLC v3), the IEC
61131-3 program, the broker, the database and the dashboards are the real
software, running in Docker on one laptop. The ESP32 firmware is compiled but
has never been flashed.

```
 OT side (ot_net 172.28.10.0/24)                         IT side (it_net 172.28.20.0/24)
 ┌──────────────┐ Modbus TCP ┌──────────────┐ Modbus TCP ┌──────────┐ MQTT/TLS ┌───────────┐   ┌──────────┐   ┌──────────┐
 │ plant (sim)  │◄──────────►│ OpenPLC v3   │◄───────────│          │  QoS 1   │ Mosquitto │──►│ ingestor │──►│ InfluxDB │──► Grafana
 │ tank+pump    │  master:PLC│ ST PID +     │  read-only │ gateway  │─────────►│ mTLS, ACL │   │ manual   │   │ 2.7      │   (alerts)
 │ :5020        │  50 ms     │ interlocks   │            │ (dual-   │          │ persist.  │   │ ack, dedup│  └──────────┘
 └──────────────┘            └──────────────┘            │  homed)  │          └───────────┘   └──────────┘
 ┌──────────────┐  RTU over socat pty pair + "wire"      │ SQLite   │
 │ vibnode (sim)│◄───────────────────────────────────────│ outbox   │
 │ ESP32 regmap │   19200 8N1 timing, injectable BER      └──────────┘
 └──────────────┘
```

## 1. Plant model (`iiot/plant_model.py`, `services/plant/main.py`)

* Mass balance: `A·dh/dt = Qin − Qout`. `Qin = Qmax·n/100` (pump speed `n`, set
  to zero when the suction is dry). `Qout = Cv·valve·√h` (a gravity drain).
  `A = 0.05 m²`, `H = 1 m` (50 L), `Qmax = 1 L/s`, and `Cv` is chosen so the valve at
  50% holds 50% level when the pump runs at 50%.
* The pump speed follows its reference through a 1 s first-order lag, which
  stands in for the VFD ramp. The level transmitter has 0.10% (1σ) Gaussian noise
  and is quantised to 0.01%.
* The model is self-regulating (outflow rises with √h). Linearised around 50%,
  it is a first-order process with τ ≈ 100 s, and it behaves almost like an
  integrator over the PID's time scale.
* **Hardwired behaviour is modelled in the plant itself, not in the PLC**:
  * Pressing the field E-stop de-energises the contactor directly, whatever the
    PLC commands.
  * If the PLC heartbeat register stops changing for 2 s, the "VFD" drops the
    pump (comms-loss fault).
  * The LSHH and LSLL float switches are separate from the level transmitter.
* pymodbus serves the I/O as a Modbus TCP slave. The physics runs at 100 Hz
  (`PLANT_DT_S`, 50 Hz before v2). Each
  table is updated with a single `setValues` call, so the timestamp hi/lo words
  are always consistent for a reader.
* `:8081` is a *test-harness* HTTP API (valve position, E-stop, dry-run fault).
  It is deliberately outside the control path.

## 2. PLC: OpenPLC v3 + IEC 61131-3 (`plc/`)

* **Image**: OpenPLC has no arm64 image, so it is built from source
  (`thiagoralves/OpenPLC_v3` pinned at `b5d4135`) on `debian:trixie`.
* **Headless provisioning** (`plc/provision.py`) drives the same HTTP endpoints
  a browser would: login, upload the `.st` file, register it, compile it
  (matiec `iec2c` → C → g++, about 8 s), add the plant as a "slave device",
  enable the Modbus server and start-in-run-mode, then start. Afterwards it
  sends a START pulse over raw Modbus TCP (FC06). `docker compose up` therefore
  brings up a programmed, running PLC.
* **Program** (`plc/tank_control.st`):
  * `FB_PID`, written in ST:
    * positional PID with derivative on measurement (no derivative kick on setpoint
      steps) and a D filter with Tf = Td/10;
    * conditional-integration anti-windup;
    * bumpless transfer: while disabled, the I term is preloaded so that the output
      equals the current actuator value.
  * Interlocks, written one rung per equation (each rung's ASCII ladder is in
    the comments): dry-run trip, fail-to-start, run permissive, and start/stop
    seal-in.
  * Alarms with hysteresis: H 85/83, L 15/17, HH 95/93 (OR LSHH), LL 5/7 (OR LSLL).
  * I/O watchdog: if the plant's sample counter stays the same for 2 s, the PLC
    raises IO_FAULT and drops the permissive (fail-safe).
* **Ladder vs ST**: the interlocks are **ST, not LD**. OpenPLC's runtime compiler
  (matiec) only takes textual languages. Converting LD (PLCopen XML) to ST is
  done by the OpenPLC Editor, a GUI built on Beremiz, and that step cannot be
  scripted in this container. The rungs are written so that each boolean
  equation is a one-to-one transcription of the ladder rung drawn above it.
* matiec gotchas hit while building this:
  * `PID` is a standard-library FB name, so the instance is called `LIC101`.
  * Located (`AT %…`) and plain variables must sit in separate `VAR` blocks.
  * Bitwise `OR` needs `WORD`, not `UINT`.
* Address map (OpenPLC conventions):
  * `%IX100.x` / `%IW100…` and `%QX100.x` / `%QW100…` are the slave-device I/O (plant).
  * `%QW0…49` → holding registers 0–49 (tag map read by the gateway).
  * `%QX0.0…%QX3.1` → coils 0–25.
  * `%MW0…` → holding registers 1024+ (HMI setpoint, commands, gains).
* Scan: the task interval is 20 ms (50 ms before v2). OpenPLC polls the plant
  every 10 ms plus the transaction time. Each scan reads inputs, runs logic and
  writes outputs. A separate Modbus thread serves the buffers.
  * `SCAN_MS` in the program feeds the PID sample time and the diagnostic timers.
    `tests/test_plc_tagmap.py` fails if it differs from the task interval.
* **v2 diagnostics in the program** (tag map in the README):
  * the PID's P, I and D terms and its output;
  * alarm limits, and the operator setpoint before its 90 % clamp;
  * pump runtime (s) and a start counter;
  * trip counters for dry run, fail-to-start, HH, E-stop and I/O fault;
  * a first-out trip code;
  * elapsed time of each interlock timer;
  * a per-sensor quality word: range checks, frozen transmitter, float-switch vs
    transmitter discrepancy, missing motor current;
  * interlock-chain and raw-DI words;
  * net flow, filtered level and level rate.

  All of it is diagnostic. No trip path reads any of it.

## 3. Vibration node

### Firmware (`firmware/vibration-node`, PlatformIO, Arduino-ESP32)

* ADXL345 over I2C at 400 kHz: 800 Hz ODR, ±4 g full-resolution (3.9 mg/LSB),
  FIFO in stream mode, drained every 10 ms (the FIFO is 32 samples deep, which
  is 40 ms at 800 Hz).
* 512-sample windows (0.64 s, 1.5625 Hz bins). The sampler task runs on core 0.
* `vib_dsp.c` (portable C):
  * mean removal and AC RMS per axis;
  * Hann window and radix-2 FFT;
  * peak with parabolic interpolation;
  * velocity spectrum `v = a/(2πf)`;
  * velocity RMS (10–400 Hz) via Parseval, with the Hann ENBW correction of 1.5;
  * "1x" = RMS of ±1 bin around the velocity peak in 10–60 Hz.
* Alarms: an alarm sets after 3 consecutive windows over the threshold and
  clears after 3 consecutive windows under 90% of it.
  * Imbalance requires the 1x component to dominate (≥ 0.7 × overall).
* `modbus_rtu_slave.c` handles FC03, FC04, FC06 and FC16, returns exceptions
  01/02/03, and stays silent on a bad CRC or a different unit id. The main loop
  ends a frame after t3.5 of silence (2.005 ms at 19200 baud).
* RS-485: MAX3485 with DE/RE on GPIO4. The firmware drives DE high, writes,
  calls `flush()` (which waits for the shift register to empty) and drops DE.
* **Status: it compiles (RAM 11.2%, flash 22.9%). It has never been run on
  hardware.** The DE timing, the I2C error paths and the FIFO overrun behaviour
  are all unverified.
* The *same* `vib_dsp.c` and `modbus_rtu_slave.c` are compiled on the host in
  `tests/test_vib_dsp.py` and compared, via ctypes, against the Python twin the
  emulator uses. They agree to within 1e-3 relative.

### Emulator (`services/vibnode/emulator.py`)

* It uses the identical register map (`iiot/vib_regmap.py`;
  `tests/test_regmap_sync.py` parses `regmap.h` and fails on any drift) and the
  identical DSP.
* The signal is synthetic:
  * 1x at 24.75 Hz (1485 rpm, a 4-pole motor on the Indian 50 Hz grid), plus 2x
    and 5x (vane pass);
  * 4 mg white noise;
  * 3.9 mg quantisation.
* An imbalance fault adds velocity at 1x.
* The serial path is a real tty all the way:
  * **Emulator side**: the emulator opens `/tmp/ttyNODE`. A socat virtual serial
    pair joins it to `/tmp/ttyBUS`, which is read by the "wire" thread.
  * **Wire thread**: it serves TCP :7000 and delays every byte by 10 bits/19200 s.
    It can also flip data bits at a configured BER.
  * **Gateway side**: the gateway's own socat PTY `/tmp/ttyRS485` connects to
    TCP :7000. This is like a serial device server in raw mode.

## 4. Gateway (`services/gateway/main.py`)

* **Two poll loops**, each in its own thread, so the slow serial bus never
  delays the fast TCP data:
  * **PLC**, at `PLC_POLL_HZ` (20 Hz): FC03 HR 0–49 and FC01 coils 0–25 to OpenPLC.
  * **Vibration node**, at `VIB_POLL_HZ` (2 Hz): FC04 IR 0–43. This is one request
    of about 55 ms on the wire.

  Together that is 3 requests and 120 raw registers/bits. They decode into 74 tank
  fields and 45 vibration fields, 119 tags in all. Before v2: 40 tags at 2 Hz, with
  the RTU read in series in front of every publish.
* **Report-by-exception** (`iiot/rbe.py`, deadbands in `iiot/telemetry.py`):
  * A poll becomes a message (one outbox row, one seq) only if some tag moved by
    more than its deadband since its last published value, or if quality changed.
  * Every `INTEGRITY_S` (5 s), and after a bad poll, a full snapshot is sent.
  * A change message carries only the moved tags, plus the plant sample timestamp.
  * `RBE=0` publishes every poll in full; the latency bench uses it for sample count.
* The RTU master is hand-written (`iiot/modbus_rtu.py`) so every poll can be
  classified as ok, crc_error, timeout, exception or malformed.
* **Store-and-forward**:
  * The snapshot is committed to SQLite (WAL) *before* any network I/O.
  * `INTEGER PRIMARY KEY AUTOINCREMENT` provides seq numbers that never repeat
    across deletes and restarts.
  * The drainer thread hands rows to paho in seq order, with a bounded in-flight
    window of 100. It is woken by a `threading.Event` on every enqueue and every
    PUBACK. Before v2 it polled with a 20 ms sleep, which added up to 20 ms.
  * A row is deleted only when its PUBACK arrives.
  * paho resends its own unacknowledged messages on reconnect (DUP=1). The
    outbox covers process restarts.
* **Deadlock avoided**: paho calls `on_publish` while it holds its internal
  message mutex. The callback therefore only appends the mid to a deque. All
  the bookkeeping happens in the drainer thread.
* **MQTT**:
  * TLS 1.2+ with client certificates (mTLS).
  * LWT `{"state":"offline"}` retained on `iiot/site1/gw1/status`.
  * Retained `online` status published on connect.
  * Health stats published every 5 s, retained, QoS 0.
* **Backfill timer**: runs from CONNACK until every row that existed at
  reconnect has been PUBACKed.
* **The gateway never writes to the PLC.** It is read-only.

## 5. Broker, ingestor, database, dashboards

* **Mosquitto 2.0**:
  * TLS-only listener with `require_certificate` and `use_identity_as_username`
    (the certificate CN becomes the username), plus a per-CN ACL.
  * `persistence true`.
  * `max_queued_messages 0`. The default of 1000 would silently drop QoS 1
    messages for the ingestor during a large backfill.
  * `set_tcp_nodelay true`. Nagle's algorithm otherwise holds a small PUBLISH
    while an earlier segment is unacknowledged. The gateway and ingestor set
    TCP_NODELAY on their own sockets for the same reason (paho does not).
* **Ingestor**:
  * Persistent session (`clean_session=False`).
  * Batching without a batching delay: it blocks for the first message, then takes
    whatever is *already* queued (up to 500) and writes once. Live traffic gives
    batches of 1. A reconnect backlog gives large batches automatically. Before v2
    it waited a fixed 50 ms for more messages, which was pure latency at 2 msg/s.
  * `manual_ack=True`: the PUBACK goes out only after InfluxDB confirms the
    write. If Influx is down, the ingestor retries and does not ack, so nothing
    is lost.
  * Records every seq in SQLite to count duplicate deliveries.
  * Writes a `pipeline_latency` point per message, from a second thread, so this
    write does not sit in front of the next telemetry message.
* **Idempotency**:
  * Each point's timestamp is the gateway poll time carried inside the message,
    and its tags are fixed.
  * A redelivered message therefore produces a byte-identical point, and
    InfluxDB overwrites it (same series key and timestamp).
  * Duplicates are harmless.
* **Grafana 11**: the datasource, dashboard and 4 alert rules are provisioned
  from files:
  * level > 85%;
  * dry-run trip;
  * imbalance (the node's debounced 1x alarm bit);
  * gateway offline / no telemetry.

## 6. Security boundaries (lab, not production)

* **Two networks**: the OT network holds the plant, PLC and node; the IT network
  holds the broker, DB and UI. Only the gateway spans both. This is a
  Purdue-model DMZ in miniature.
* Lab-only secrets, all stated plainly:
  * OpenPLC keeps its default `openplc/openplc` login.
  * The InfluxDB token and Grafana password are fixed lab values.
  * `certs/` comes from `scripts/gen_certs.sh` and is gitignored.

---
