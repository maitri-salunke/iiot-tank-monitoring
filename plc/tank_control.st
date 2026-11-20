(*
  tank_control.st -- IEC 61131-3 program for OpenPLC v3 runtime.

  POUs
    FB_PID        Structured Text PID (derivative on measurement, filtered D,
                  conditional-integration anti-windup, bumpless auto/manual).
    TANK_CONTROL  Main program: I/O scaling, alarms, interlocks, PID, tag map.

  Interlocks are written as one boolean equation per ladder rung (see the ASCII
  rungs in the comments). OpenPLC's runtime compiler (matiec iec2c) accepts
  only textual languages; ladder (LD) needs the OpenPLC Editor GUI to convert
  PLCopen XML to ST, which cannot be driven headlessly, so the rungs are kept
  as rung-equivalent ST.

  I/O comes from the SIMULATED plant (Python, Modbus TCP slave) that OpenPLC
  polls as a "slave device":  %IX100.x / %IW100.. in,  %QX100.x / %QW100.. out.
  The gateway / HMI see the PLC's own Modbus server:
      holding regs 0..49  = %QW0..%QW49 (tag map, read-only by convention;
                            0..15 process, 16..49 PID terms, limits, counters,
                            interlock timers, quality, diagnostics)
      coils 0..25         = %QX0.0..%QX3.1 (status, raw DIs, quality bits)
  The gateway reads both blocks in 2 requests (FC03 x50, FC01 x26).
  iiot/telemetry.py PLC_HR_MAP / COIL_NAMES is the decoder; tests/test_plc_tagmap.py
  checks it against the AT %QW / %QX declarations below.
      holding regs 1024.. = %MW0.. (HMI writes: setpoint, commands, gains)
*)

FUNCTION TO_I100 : INT
  (* REAL -> INT x100 with saturation (a %-term can exceed +/-327.67) *)
  VAR_INPUT
    X : REAL;
  END_VAR
  VAR
    Y : REAL;
  END_VAR
  Y := X * 100.0;
  IF Y > 32767.0 THEN Y := 32767.0; END_IF;
  IF Y < -32768.0 THEN Y := -32768.0; END_IF;
  TO_I100 := REAL_TO_INT(Y);
END_FUNCTION


FUNCTION_BLOCK FB_PID
  VAR_INPUT
    ENABLE : BOOL;
    PV : REAL;
    SP : REAL;
    KP : REAL;          (* %out per %level *)
    TI : REAL;          (* integral time, s; <= 0 disables I *)
    TD : REAL;          (* derivative time, s; <= 0 disables D *)
    TS : REAL;          (* sample time, s (= task interval) *)
    OUT_MIN : REAL;
    OUT_MAX : REAL;
    TRACK : REAL;       (* value to follow while disabled (bumpless transfer) *)
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
    ERR : REAL;
    SATURATED : BOOL;
    P_OUT : REAL;       (* diagnostics: the three terms, % output *)
    I_OUT : REAL;
    D_OUT : REAL;
  END_VAR
  VAR
    I_TERM : REAL;
    D_TERM : REAL;
    P_TERM : REAL;
    PV_PREV : REAL;
    U : REAL;
    TF : REAL;
    INIT : BOOL;
  END_VAR

  ERR := SP - PV;
  IF NOT INIT THEN
    PV_PREV := PV;
    INIT := TRUE;
  END_IF;

  IF NOT ENABLE THEN
    (* track: next enable starts from the current actuator value *)
    OUT := TRACK;
    I_TERM := TRACK - KP * ERR;
    D_TERM := 0.0;
    PV_PREV := PV;
    SATURATED := FALSE;
  ELSE
    P_TERM := KP * ERR;

    (* derivative on measurement (no kick on setpoint steps), filter Tf = Td/10 *)
    IF TD > 0.0 THEN
      TF := TD / 10.0;
      D_TERM := (TF * D_TERM - KP * TD * (PV - PV_PREV)) / (TF + TS);
    ELSE
      D_TERM := 0.0;
    END_IF;
    PV_PREV := PV;

    U := P_TERM + I_TERM + D_TERM;

    (* conditional integration: freeze I while saturated in the direction of the error *)
    IF TI > 0.0 THEN
      IF NOT (((U >= OUT_MAX) AND (ERR > 0.0)) OR ((U <= OUT_MIN) AND (ERR < 0.0))) THEN
        I_TERM := I_TERM + KP * TS / TI * ERR;
      END_IF;
      IF I_TERM > OUT_MAX THEN I_TERM := OUT_MAX; END_IF;
      IF I_TERM < OUT_MIN THEN I_TERM := OUT_MIN; END_IF;
    ELSE
      I_TERM := 0.0;
    END_IF;

    U := P_TERM + I_TERM + D_TERM;
    SATURATED := FALSE;
    IF U > OUT_MAX THEN U := OUT_MAX; SATURATED := TRUE; END_IF;
    IF U < OUT_MIN THEN U := OUT_MIN; SATURATED := TRUE; END_IF;
    OUT := U;
  END_IF;
  P_OUT := KP * ERR;
  I_OUT := I_TERM;
  D_OUT := D_TERM;
END_FUNCTION_BLOCK


PROGRAM TANK_CONTROL
  VAR
    (* ---- field inputs from the simulated plant (OpenPLC slave device 0) ---- *)
    ESTOP_OK  AT %IX100.0 : BOOL;   (* hardwired NC E-stop chain healthy *)
    PUMP_FB   AT %IX100.1 : BOOL;   (* contactor aux / VFD running feedback *)
    LSHH      AT %IX100.2 : BOOL;   (* independent high-high float switch *)
    LSLL      AT %IX100.3 : BOOL;   (* independent low-low float switch *)
    LT101_RAW AT %IW100 : UINT;     (* level 0..10000 = 0..100.00 % *)
    FT101_RAW AT %IW101 : UINT;     (* inflow  L/min x10 *)
    FT102_RAW AT %IW102 : UINT;     (* outflow L/min x10 *)
    IT101_RAW AT %IW103 : UINT;     (* pump motor current A x100 *)
    SRC_TS_HI AT %IW104 : UINT;     (* plant sample time, ms, high word *)
    SRC_TS_LO AT %IW105 : UINT;     (* plant sample time, ms, low word *)
    SRC_CNT   AT %IW106 : UINT;     (* plant sample counter *)
    VALVE_RAW AT %IW107 : UINT;     (* outlet valve position % x100 (disturbance) *)

    (* ---- field outputs to the simulated plant ---- *)
    PUMP_RUN_CMD   AT %QX100.0 : BOOL;
    PUMP_SPEED_REF AT %QW100 : UINT;  (* VFD speed reference 0..10000 = 0..100 % *)
    PLC_HEARTBEAT  AT %QW101 : UINT;  (* plant drops the pump if this stalls (comm-loss) *)

    (* ---- tag map for gateway / SCADA (PLC Modbus server) ---- *)
    TAG_LEVEL   AT %QW0 : UINT;
    TAG_SP      AT %QW1 : UINT;
    TAG_SPEED   AT %QW2 : UINT;
    TAG_FIN     AT %QW3 : UINT;
    TAG_FOUT    AT %QW4 : UINT;
    TAG_CURRENT AT %QW5 : UINT;
    TAG_VALVE   AT %QW6 : UINT;
    TAG_TS_HI   AT %QW7 : UINT;
    TAG_TS_LO   AT %QW8 : UINT;
    TAG_CNT     AT %QW9 : UINT;
    TAG_STATUS  AT %QW10 : UINT;
    TAG_SCANS   AT %QW11 : UINT;
    TAG_ERR     AT %QW12 : INT;     (* SP - PV, % x100, signed *)
    TAG_MODE    AT %QW13 : UINT;    (* 0 stopped, 1 auto, 2 manual *)
    TAG_KP      AT %QW14 : UINT;
    TAG_TI      AT %QW15 : UINT;

    C_RUN       AT %QX0.0 : BOOL;
    C_AL_H      AT %QX0.1 : BOOL;
    C_AL_L      AT %QX0.2 : BOOL;
    C_AL_HH     AT %QX0.3 : BOOL;
    C_AL_LL     AT %QX0.4 : BOOL;
    C_DRY_TRIP  AT %QX0.5 : BOOL;
    C_ESTOP     AT %QX0.6 : BOOL;
    C_PERMIT    AT %QX0.7 : BOOL;
    C_FTS_TRIP  AT %QX1.0 : BOOL;
    C_IO_FAULT  AT %QX1.1 : BOOL;

    (* ---- extended tag map (v2): PID terms, limits, counters, timers, quality ---- *)
    TAG_TD        AT %QW16 : UINT;  (* Td s x100 *)
    TAG_PID_OUT   AT %QW17 : UINT;  (* PID output % x100 *)
    TAG_P_TERM    AT %QW18 : INT;   (* % x100 *)
    TAG_I_TERM    AT %QW19 : INT;
    TAG_D_TERM    AT %QW20 : INT;
    TAG_HMI_SP    AT %QW21 : UINT;  (* operator setpoint entry, before the 90 % clamp *)
    TAG_MAN_SPEED AT %QW22 : UINT;
    TAG_LIM_H     AT %QW23 : UINT;  (* alarm limits % x100 *)
    TAG_LIM_L     AT %QW24 : UINT;
    TAG_LIM_HH    AT %QW25 : UINT;
    TAG_LIM_LL    AT %QW26 : UINT;
    TAG_RUN_HI    AT %QW27 : UINT;  (* pump runtime s, uint32 hi *)
    TAG_RUN_LO    AT %QW28 : UINT;  (*                      lo *)
    TAG_STARTS    AT %QW29 : UINT;  (* pump start counter *)
    TAG_DRY_TRIPS AT %QW30 : UINT;
    TAG_FTS_TRIPS AT %QW31 : UINT;
    TAG_HH_TRIPS  AT %QW32 : UINT;
    TAG_ESTOPS    AT %QW33 : UINT;
    TAG_IOF_CNT   AT %QW34 : UINT;
    TAG_FIRST_OUT AT %QW35 : UINT;  (* 0 none 1 E-stop 2 HH 3 dry run 4 fail-to-start 5 I/O fault *)
    TAG_DRY_MS    AT %QW36 : UINT;  (* interlock timers: elapsed ms toward trip *)
    TAG_FTS_MS    AT %QW37 : UINT;
    TAG_STALE_MS  AT %QW38 : UINT;
    TAG_QUALITY   AT %QW39 : UINT;  (* per-sensor quality bits *)
    TAG_ILK       AT %QW40 : UINT;  (* permissive chain contacts, 1 = healthy *)
    TAG_DI        AT %QW41 : UINT;  (* raw field DIs *)
    TAG_NET_FLOW  AT %QW42 : INT;   (* inflow - outflow, L/min x10 *)
    TAG_LEVEL_F   AT %QW43 : UINT;  (* level, 2 s filter, % x100 *)
    TAG_RATE      AT %QW44 : INT;   (* level rate of change, %/min x100 *)
    TAG_HB        AT %QW45 : UINT;  (* heartbeat sent to the plant *)
    TAG_SINCE     AT %QW46 : UINT;  (* s since last start *)
    TAG_NALARM    AT %QW47 : UINT;  (* active alarms/trips *)
    TAG_SCAN_MS   AT %QW48 : UINT;
    TAG_VERSION   AT %QW49 : UINT;

    C_MANUAL      AT %QX1.2 : BOOL;
    C_PID_SAT     AT %QX1.3 : BOOL;
    C_DI_PUMP_FB  AT %QX1.4 : BOOL;
    C_DI_LSHH     AT %QX1.5 : BOOL;
    C_DI_LSLL     AT %QX1.6 : BOOL;
    C_RUN_CMD     AT %QX1.7 : BOOL;
    C_DRY_TIMING  AT %QX2.0 : BOOL;
    C_FTS_TIMING  AT %QX2.1 : BOOL;
    C_STALE_TIMING AT %QX2.2 : BOOL;
    C_Q_LT        AT %QX2.3 : BOOL;   (* LT-101 out of range or frozen *)
    C_Q_FT101     AT %QX2.4 : BOOL;
    C_Q_FT102     AT %QX2.5 : BOOL;
    C_Q_IT101     AT %QX2.6 : BOOL;
    C_Q_LSHH      AT %QX2.7 : BOOL;   (* float switch disagrees with transmitter *)
    C_Q_LSLL      AT %QX3.0 : BOOL;
    C_ANY_TRIP    AT %QX3.1 : BOOL;

    (* ---- HMI writes (holding registers 1024..) ---- *)
    HMI_SP        AT %MW0 : UINT;   (* setpoint % x100; 0 at boot -> default 50 % *)
    HMI_CMD       AT %MW1 : WORD;   (* bit0 START, bit1 STOP, bit2 RESET, bit3 MANUAL (level) *)
    HMI_MAN_SPEED AT %MW2 : UINT;   (* manual speed % x100 *)
    HMI_KP        AT %MW3 : UINT;   (* Kp x100 *)
    HMI_TI        AT %MW4 : UINT;   (* Ti s x10 *)
    HMI_TD        AT %MW5 : UINT;   (* Td s x100 *)
  END_VAR
  VAR
    (* ---- internal (matiec wants located and plain declarations in separate blocks) ---- *)
    INIT_DONE : BOOL;
    LEVEL : REAL;
    FLOW_IN : REAL;
    SP : REAL;
    SPEED : REAL;
    START_PB : BOOL;
    STOP_PB : BOOL;
    RESET_PB : BOOL;
    MANUAL : BOOL;
    PUMP_RUN : BOOL;
    PERMISSIVE : BOOL;
    AL_H : BOOL;
    AL_L : BOOL;
    AL_HH : BOOL;
    AL_LL : BOOL;
    DRY_TRIP : BOOL;
    FTS_TRIP : BOOL;
    IO_FAULT : BOOL;
    CNT_SAME : BOOL;
    LAST_CNT : UINT;
    SCANS : UINT;
    HB : UINT;
    STATUS : WORD;      (* bit operations need ANY_BIT types *)
    DRY_TON : TON;
    FTS_TON : TON;
    STALE_TON : TON;
    LIC101 : FB_PID;   (* level indicating controller; "PID" is reserved by the matiec std library *)
    (* ---- v2 diagnostics ---- *)
    SCAN_MS : UINT := 20;   (* MUST equal the TASK interval at the bottom of this file *)
    TS_S : REAL;
    SEC_ACC : UINT;
    SEC_TICK : BOOL;
    RUN_S : UDINT;
    SINCE_S : UINT;
    RT_START : R_TRIG;
    RT_DRY : R_TRIG;
    RT_FTS : R_TRIG;
    RT_HH : R_TRIG;
    RT_ESTOP : R_TRIG;
    RT_IOF : R_TRIG;
    N_STARTS : UINT;
    N_DRY : UINT;
    N_FTS : UINT;
    N_HH : UINT;
    N_ESTOP : UINT;
    N_IOF : UINT;
    DRY_MS : UINT;
    FTS_MS : UINT;
    STALE_MS : UINT;
    LT_LAST : UINT;
    LT_SAME_MS : UINT;
    Q_LT_RANGE : BOOL;
    Q_LT_FROZEN : BOOL;
    Q_FT101 : BOOL;
    Q_FT102 : BOOL;
    Q_IT101 : BOOL;
    Q_LSHH : BOOL;
    Q_LSLL : BOOL;
    IT_TON : TON;
    QUAL : WORD;
    ILK : WORD;
    DIW : WORD;
    TRIP_NOW : UINT;
    FIRST_OUT : UINT;
    NALARM : UINT;
    F_INIT : BOOL;
    LEVEL_F : REAL;
    LEVEL_F_PREV : REAL;
    RATE_F : REAL;
  END_VAR

  (* ================= first scan: defaults for non-retentive %MW ============ *)
  IF NOT INIT_DONE THEN
    IF HMI_SP = 0 THEN HMI_SP := 5000; END_IF;
    IF HMI_KP = 0 THEN HMI_KP := 1500; END_IF;     (* Kp = 15.0 %/% *)
    IF HMI_TI = 0 THEN HMI_TI := 120; END_IF;      (* Ti = 12.0 s (tools/tune_offline.py) *)
    INIT_DONE := TRUE;
  END_IF;

  (* ================= input scaling ========================================= *)
  TS_S    := UINT_TO_REAL(SCAN_MS) / 1000.0;
  LEVEL   := UINT_TO_REAL(LT101_RAW) / 100.0;
  FLOW_IN := UINT_TO_REAL(FT101_RAW) / 10.0;
  SP      := UINT_TO_REAL(HMI_SP) / 100.0;
  IF SP > 90.0 THEN SP := 90.0; END_IF;            (* setpoint limited below the H alarm band *)

  START_PB := (HMI_CMD AND 16#0001) <> 0;
  STOP_PB  := (HMI_CMD AND 16#0002) <> 0;
  RESET_PB := (HMI_CMD AND 16#0004) <> 0;
  MANUAL   := (HMI_CMD AND 16#0008) <> 0;
  HMI_CMD  := HMI_CMD AND 16#0008;                 (* pushbuttons are pulses: clear after read *)

  (* ================= alarms (with hysteresis) ============================== *)
  IF LEVEL >= 85.0 THEN AL_H := TRUE;  ELSIF LEVEL <= 83.0 THEN AL_H := FALSE; END_IF;
  IF LEVEL <= 15.0 THEN AL_L := TRUE;  ELSIF LEVEL >= 17.0 THEN AL_L := FALSE; END_IF;
  IF LSHH OR (LEVEL >= 95.0) THEN AL_HH := TRUE; ELSIF LEVEL <= 93.0 THEN AL_HH := FALSE; END_IF;
  IF LSLL OR (LEVEL <= 5.0) THEN AL_LL := TRUE; ELSIF LEVEL >= 7.0 THEN AL_LL := FALSE; END_IF;

  (* ================= I/O comms watchdog ===================================== *)
  (* Plant sample counter must keep changing; stale > 2 s = I/O fault (fail-safe). *)
  CNT_SAME := SRC_CNT = LAST_CNT;
  LAST_CNT := SRC_CNT;
  STALE_TON(IN := CNT_SAME, PT := T#2s);
  IO_FAULT := STALE_TON.Q;

  (* ================= interlock rungs (ladder-equivalent) =================== *)
  (* Rung 1  dry-run trip (latched)
       |--[PUMP_RUN]--[PUMP_FB]--[SPEED>=20]--[FLOW_IN<2]--(TON 3s)--+--(S DRY_TRIP)--|
       |--[RESET_PB]--[/DRY_TON.IN]----------------------------------+--(R DRY_TRIP)--|   *)
  DRY_TON(IN := PUMP_RUN AND PUMP_FB AND (SPEED >= 20.0) AND (FLOW_IN < 2.0), PT := T#3s);
  IF DRY_TON.Q THEN DRY_TRIP := TRUE; END_IF;
  IF RESET_PB AND NOT DRY_TON.IN THEN DRY_TRIP := FALSE; END_IF;

  (* Rung 2  fail-to-start (latched): command on, no running feedback for 2 s
       |--[PUMP_RUN_CMD]--[/PUMP_FB]--(TON 2s)--(S FTS_TRIP)--|   reset as rung 1 *)
  FTS_TON(IN := PUMP_RUN_CMD AND NOT PUMP_FB, PT := T#2s);
  IF FTS_TON.Q THEN FTS_TRIP := TRUE; END_IF;
  IF RESET_PB AND NOT FTS_TON.IN THEN FTS_TRIP := FALSE; END_IF;

  (* Rung 3  run permissive (series chain of NC contacts)
       |--[ESTOP_OK]--[/AL_HH]--[/DRY_TRIP]--[/FTS_TRIP]--[/IO_FAULT]--(PERMISSIVE)--|   *)
  PERMISSIVE := ESTOP_OK AND NOT AL_HH AND NOT DRY_TRIP AND NOT FTS_TRIP AND NOT IO_FAULT;

  (* Rung 4  start/stop seal-in
       |--+--[START_PB]--+--[/STOP_PB]--[PERMISSIVE]--(PUMP_RUN)--|
       |  +--[PUMP_RUN]--+                                         |   *)
  PUMP_RUN := (START_PB OR PUMP_RUN) AND NOT STOP_PB AND PERMISSIVE;

  (* ================= level PID (Structured Text) =========================== *)
  LIC101(ENABLE := PUMP_RUN AND NOT MANUAL,
      PV := LEVEL, SP := SP,
      KP := UINT_TO_REAL(HMI_KP) / 100.0,
      TI := UINT_TO_REAL(HMI_TI) / 10.0,
      TD := UINT_TO_REAL(HMI_TD) / 100.0,
      TS := TS_S,
      OUT_MIN := 0.0, OUT_MAX := 100.0,
      TRACK := SPEED);

  IF NOT PUMP_RUN THEN
    SPEED := 0.0;
  ELSIF MANUAL THEN
    SPEED := UINT_TO_REAL(HMI_MAN_SPEED) / 100.0;
    IF SPEED > 100.0 THEN SPEED := 100.0; END_IF;
  ELSE
    SPEED := LIC101.OUT;
  END_IF;

  (* ================= outputs to plant ====================================== *)
  PUMP_RUN_CMD := PUMP_RUN;
  PUMP_SPEED_REF := REAL_TO_UINT(SPEED * 100.0);
  HB := HB + 1;
  PLC_HEARTBEAT := HB;

  (* ================= diagnostics (v2) ====================================== *)
  (* 1 s tick from the scan clock (OpenPLC sleeps to absolute tick times, so no drift) *)
  SEC_ACC := SEC_ACC + SCAN_MS;
  SEC_TICK := SEC_ACC >= 1000;
  IF SEC_TICK THEN SEC_ACC := SEC_ACC - 1000; END_IF;
  IF SEC_TICK AND PUMP_FB THEN RUN_S := RUN_S + 1; END_IF;
  IF NOT PUMP_RUN THEN SINCE_S := 0; ELSIF SEC_TICK AND (SINCE_S < 65535) THEN SINCE_S := SINCE_S + 1; END_IF;

  (* event counters on rising edges *)
  RT_START(CLK := PUMP_RUN);   IF RT_START.Q THEN N_STARTS := N_STARTS + 1; END_IF;
  RT_DRY(CLK := DRY_TRIP);     IF RT_DRY.Q THEN N_DRY := N_DRY + 1; END_IF;
  RT_FTS(CLK := FTS_TRIP);     IF RT_FTS.Q THEN N_FTS := N_FTS + 1; END_IF;
  RT_HH(CLK := AL_HH);         IF RT_HH.Q THEN N_HH := N_HH + 1; END_IF;
  RT_ESTOP(CLK := NOT ESTOP_OK); IF RT_ESTOP.Q THEN N_ESTOP := N_ESTOP + 1; END_IF;
  RT_IOF(CLK := IO_FAULT);     IF RT_IOF.Q THEN N_IOF := N_IOF + 1; END_IF;

  (* interlock timers: how far each trip timer has run (ms, saturating) *)
  IF DRY_TON.IN THEN IF DRY_MS < 60000 THEN DRY_MS := DRY_MS + SCAN_MS; END_IF; ELSE DRY_MS := 0; END_IF;
  IF FTS_TON.IN THEN IF FTS_MS < 60000 THEN FTS_MS := FTS_MS + SCAN_MS; END_IF; ELSE FTS_MS := 0; END_IF;
  IF STALE_TON.IN THEN IF STALE_MS < 60000 THEN STALE_MS := STALE_MS + SCAN_MS; END_IF; ELSE STALE_MS := 0; END_IF;

  (* per-sensor quality (diagnostic only -- the trips above do not depend on it) *)
  Q_LT_RANGE := LT101_RAW > 10000;
  IF LT101_RAW = LT_LAST THEN
    IF LT_SAME_MS < 60000 THEN LT_SAME_MS := LT_SAME_MS + SCAN_MS; END_IF;
  ELSE
    LT_SAME_MS := 0;
  END_IF;
  LT_LAST := LT101_RAW;
  Q_LT_FROZEN := (LT_SAME_MS >= 5000) AND (LEVEL > 1.0) AND (LEVEL < 99.0);   (* a live transmitter is never noise-free *)
  Q_FT101 := FT101_RAW > 900;                        (* > 90 L/min: beyond 1.5x pump capacity *)
  Q_FT102 := FT102_RAW > 900;
  IT_TON(IN := PUMP_FB AND (SPEED >= 30.0) AND (IT101_RAW = 0), PT := T#1s);
  Q_IT101 := (IT101_RAW > 1000) OR IT_TON.Q;         (* > 10 A, or no current while running *)
  Q_LSHH := (LSHH AND (LEVEL < 90.0)) OR ((NOT LSHH) AND (LEVEL > 98.0));
  Q_LSLL := (LSLL AND (LEVEL > 10.0)) OR ((NOT LSLL) AND (LEVEL < 2.0));
  QUAL := 16#0000;
  IF Q_LT_RANGE THEN QUAL := QUAL OR 16#0001; END_IF;
  IF Q_LT_FROZEN THEN QUAL := QUAL OR 16#0002; END_IF;
  IF Q_FT101 THEN QUAL := QUAL OR 16#0004; END_IF;
  IF Q_FT102 THEN QUAL := QUAL OR 16#0008; END_IF;
  IF Q_IT101 THEN QUAL := QUAL OR 16#0010; END_IF;
  IF Q_LSHH THEN QUAL := QUAL OR 16#0020; END_IF;
  IF Q_LSLL THEN QUAL := QUAL OR 16#0040; END_IF;
  IF IO_FAULT THEN QUAL := QUAL OR 16#0080; END_IF;

  (* permissive chain contacts and raw DIs as words *)
  ILK := 16#0000;
  IF ESTOP_OK THEN ILK := ILK OR 16#0001; END_IF;
  IF NOT AL_HH THEN ILK := ILK OR 16#0002; END_IF;
  IF NOT DRY_TRIP THEN ILK := ILK OR 16#0004; END_IF;
  IF NOT FTS_TRIP THEN ILK := ILK OR 16#0008; END_IF;
  IF NOT IO_FAULT THEN ILK := ILK OR 16#0010; END_IF;
  IF PERMISSIVE THEN ILK := ILK OR 16#0020; END_IF;
  DIW := 16#0000;
  IF ESTOP_OK THEN DIW := DIW OR 16#0001; END_IF;
  IF PUMP_FB THEN DIW := DIW OR 16#0002; END_IF;
  IF LSHH THEN DIW := DIW OR 16#0004; END_IF;
  IF LSLL THEN DIW := DIW OR 16#0008; END_IF;
  IF PUMP_RUN_CMD THEN DIW := DIW OR 16#0010; END_IF;

  (* first-out annunciation: the first trip cause is held until RESET/START with no trip present *)
  TRIP_NOW := 0;
  IF NOT ESTOP_OK THEN TRIP_NOW := 1;
  ELSIF AL_HH THEN TRIP_NOW := 2;
  ELSIF DRY_TRIP THEN TRIP_NOW := 3;
  ELSIF FTS_TRIP THEN TRIP_NOW := 4;
  ELSIF IO_FAULT THEN TRIP_NOW := 5;
  END_IF;
  IF (FIRST_OUT = 0) AND (TRIP_NOW <> 0) THEN FIRST_OUT := TRIP_NOW; END_IF;
  IF (RESET_PB OR START_PB) AND (TRIP_NOW = 0) THEN FIRST_OUT := 0; END_IF;

  NALARM := 0;
  IF AL_H THEN NALARM := NALARM + 1; END_IF;
  IF AL_L THEN NALARM := NALARM + 1; END_IF;
  IF AL_HH THEN NALARM := NALARM + 1; END_IF;
  IF AL_LL THEN NALARM := NALARM + 1; END_IF;
  IF DRY_TRIP THEN NALARM := NALARM + 1; END_IF;
  IF FTS_TRIP THEN NALARM := NALARM + 1; END_IF;
  IF IO_FAULT THEN NALARM := NALARM + 1; END_IF;
  IF NOT ESTOP_OK THEN NALARM := NALARM + 1; END_IF;

  (* filtered level (tau 2 s) and its rate of change (tau 2 s), %/min *)
  IF NOT F_INIT THEN LEVEL_F := LEVEL; LEVEL_F_PREV := LEVEL; F_INIT := TRUE; END_IF;
  LEVEL_F := LEVEL_F + (LEVEL - LEVEL_F) * TS_S / 2.0;
  RATE_F := RATE_F + ((LEVEL_F - LEVEL_F_PREV) / TS_S * 60.0 - RATE_F) * TS_S / 2.0;
  LEVEL_F_PREV := LEVEL_F;

  (* ================= tag map ============================================== *)
  SCANS := SCANS + 1;
  STATUS := 16#0000;
  IF PUMP_RUN THEN STATUS := STATUS OR 16#0001; END_IF;
  IF AL_H THEN STATUS := STATUS OR 16#0002; END_IF;
  IF AL_L THEN STATUS := STATUS OR 16#0004; END_IF;
  IF AL_HH THEN STATUS := STATUS OR 16#0008; END_IF;
  IF AL_LL THEN STATUS := STATUS OR 16#0010; END_IF;
  IF DRY_TRIP THEN STATUS := STATUS OR 16#0020; END_IF;
  IF NOT ESTOP_OK THEN STATUS := STATUS OR 16#0040; END_IF;
  IF PERMISSIVE THEN STATUS := STATUS OR 16#0080; END_IF;
  IF FTS_TRIP THEN STATUS := STATUS OR 16#0100; END_IF;
  IF IO_FAULT THEN STATUS := STATUS OR 16#0200; END_IF;
  IF MANUAL THEN STATUS := STATUS OR 16#0400; END_IF;
  IF LIC101.SATURATED THEN STATUS := STATUS OR 16#0800; END_IF;

  TAG_LEVEL := LT101_RAW;
  TAG_SP := REAL_TO_UINT(SP * 100.0);
  TAG_SPEED := PUMP_SPEED_REF;
  TAG_FIN := FT101_RAW;
  TAG_FOUT := FT102_RAW;
  TAG_CURRENT := IT101_RAW;
  TAG_VALVE := VALVE_RAW;
  TAG_TS_HI := SRC_TS_HI;
  TAG_TS_LO := SRC_TS_LO;
  TAG_CNT := SRC_CNT;
  TAG_STATUS := WORD_TO_UINT(STATUS);
  TAG_SCANS := SCANS;
  TAG_ERR := REAL_TO_INT(LIC101.ERR * 100.0);
  IF NOT PUMP_RUN THEN TAG_MODE := 0; ELSIF MANUAL THEN TAG_MODE := 2; ELSE TAG_MODE := 1; END_IF;
  TAG_KP := HMI_KP;
  TAG_TI := HMI_TI;

  C_RUN := PUMP_RUN;
  C_AL_H := AL_H;
  C_AL_L := AL_L;
  C_AL_HH := AL_HH;
  C_AL_LL := AL_LL;
  C_DRY_TRIP := DRY_TRIP;
  C_ESTOP := NOT ESTOP_OK;
  C_PERMIT := PERMISSIVE;
  C_FTS_TRIP := FTS_TRIP;
  C_IO_FAULT := IO_FAULT;

  TAG_TD := HMI_TD;
  TAG_PID_OUT := REAL_TO_UINT(LIC101.OUT * 100.0);
  TAG_P_TERM := TO_I100(LIC101.P_OUT);
  TAG_I_TERM := TO_I100(LIC101.I_OUT);
  TAG_D_TERM := TO_I100(LIC101.D_OUT);
  TAG_HMI_SP := HMI_SP;
  TAG_MAN_SPEED := HMI_MAN_SPEED;
  TAG_LIM_H := 8500;
  TAG_LIM_L := 1500;
  TAG_LIM_HH := 9500;
  TAG_LIM_LL := 500;
  TAG_RUN_HI := UDINT_TO_UINT(RUN_S / 65536);
  TAG_RUN_LO := UDINT_TO_UINT(RUN_S MOD 65536);
  TAG_STARTS := N_STARTS;
  TAG_DRY_TRIPS := N_DRY;
  TAG_FTS_TRIPS := N_FTS;
  TAG_HH_TRIPS := N_HH;
  TAG_ESTOPS := N_ESTOP;
  TAG_IOF_CNT := N_IOF;
  TAG_FIRST_OUT := FIRST_OUT;
  TAG_DRY_MS := DRY_MS;
  TAG_FTS_MS := FTS_MS;
  TAG_STALE_MS := STALE_MS;
  TAG_QUALITY := WORD_TO_UINT(QUAL);
  TAG_ILK := WORD_TO_UINT(ILK);
  TAG_DI := WORD_TO_UINT(DIW);
  TAG_NET_FLOW := UINT_TO_INT(FT101_RAW) - UINT_TO_INT(FT102_RAW);
  TAG_LEVEL_F := REAL_TO_UINT(LEVEL_F * 100.0);
  TAG_RATE := TO_I100(RATE_F);
  TAG_HB := HB;
  TAG_SINCE := SINCE_S;
  TAG_NALARM := NALARM;
  TAG_SCAN_MS := SCAN_MS;
  TAG_VERSION := 16#0200;

  C_MANUAL := MANUAL;
  C_PID_SAT := LIC101.SATURATED;
  C_DI_PUMP_FB := PUMP_FB;
  C_DI_LSHH := LSHH;
  C_DI_LSLL := LSLL;
  C_RUN_CMD := PUMP_RUN_CMD;
  C_DRY_TIMING := DRY_TON.IN;
  C_FTS_TIMING := FTS_TON.IN;
  C_STALE_TIMING := STALE_TON.IN;
  C_Q_LT := Q_LT_RANGE OR Q_LT_FROZEN;
  C_Q_FT101 := Q_FT101;
  C_Q_FT102 := Q_FT102;
  C_Q_IT101 := Q_IT101;
  C_Q_LSHH := Q_LSHH;
  C_Q_LSLL := Q_LSLL;
  C_ANY_TRIP := DRY_TRIP OR FTS_TRIP OR AL_HH OR IO_FAULT OR NOT ESTOP_OK;
END_PROGRAM


CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK MainTask(INTERVAL := T#20ms, PRIORITY := 0);   (* keep SCAN_MS in TANK_CONTROL equal *)
    PROGRAM Inst0 WITH MainTask : TANK_CONTROL;
  END_RESOURCE
END_CONFIGURATION
