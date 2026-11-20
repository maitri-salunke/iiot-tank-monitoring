import math
from iiot.plant_model import TankModel, TankState, TankParams
from iiot.pid import FbPid, step_metrics


def test_mass_balance_steady_state():
    m = TankModel(state=TankState(h_m=0.5, valve_pct=50, run_cmd=True, speed_ref_pct=50.0), seed=0)
    m.set_heartbeat(1)
    for i in range(int(600 / 0.02)):
        m.set_heartbeat(i)
        m.step(0.02)
    assert abs(m.s.qin_lps - m.s.qout_lps) < 1e-3
    # Qout = Cv*0.5*sqrt(h) = 0.5 L/s  ->  h = 0.5 m
    assert abs(m.level_pct_true() - 50.0) < 0.5


def test_estop_and_comm_loss_stop_pump():
    m = TankModel(state=TankState(run_cmd=True, speed_ref_pct=80), seed=0)
    m.set_heartbeat(1); m.step(0.02)
    assert m.pump_energised()
    m.s.estop_pressed = True
    assert not m.pump_energised()            # hardwired: independent of PLC command
    m.s.estop_pressed = False
    for _ in range(150):                     # 3 s without heartbeat change
        m.step(0.02)
    assert not m.pump_energised()


def test_dry_run_no_flow_low_current():
    m = TankModel(state=TankState(run_cmd=True, speed_ref_pct=80), seed=0)
    for i in range(500):
        m.set_heartbeat(i); m.step(0.02)
    wet_i = m.motor_current_a()
    m.s.dry_run_fault = True
    m.step(0.02)
    assert m.flow_in_lpm() == 0.0 and m.motor_current_a() < 0.6 * wet_i


def test_noise_statistics():
    m = TankModel(params=TankParams(level_noise_pct=0.1), state=TankState(h_m=0.5), seed=3)
    xs = [m.level_pct_meas() for _ in range(5000)]
    mean = sum(xs) / len(xs); sd = (sum((x - mean) ** 2 for x in xs) / len(xs)) ** 0.5
    assert abs(mean - 50) < 0.01 and 0.09 < sd < 0.11


def test_pid_anti_windup_and_bumpless():
    pid = FbPid()
    for _ in range(2000):                    # huge error, saturated output
        pid(True, 0.0, 100.0, kp=2.0, ti=5.0, td=0.0, ts=0.05)
    assert pid.out == 100.0 and pid.i_term <= 100.0
    # error reverses: with anti-windup the output must leave saturation immediately
    out = pid(True, 60.0, 50.0, kp=2.0, ti=5.0, td=0.0, ts=0.05)
    assert out < 100.0
    p2 = FbPid()
    p2(False, 40.0, 50.0, 2.0, 5.0, 0.0, 0.05, track=37.0)
    assert abs(p2(True, 40.0, 50.0, 2.0, 5.0, 0.0, 0.05) - 37.0) <= 0.2 + 1e-9   # bumpless: only one I-step (Kp*Ts/Ti*e = 0.2) away


def test_closed_loop_step_meets_spec_in_simulation():
    """Offline (Python twin) check only -- the README numbers come from OpenPLC."""
    from tools.tune_offline import simulate
    r = simulate(15.0, 12.0)
    assert r["overshoot_pct"] < 5.0
    assert r["settling_time_s"] < 40.0


def test_step_metrics_on_first_order():
    t = [i * 0.1 for i in range(1200)]
    y = [50 + 20 * (1 - math.exp(-(ti - 10) / 5)) if ti >= 10 else 50 for ti in t]
    r = step_metrics(t, y, 10.0, 50.0, 70.0)
    assert abs(r["settling_time_s"] - 5 * math.log(50)) < 0.2     # 2 % band: t = tau*ln(50)
    assert r["overshoot_pct"] == 0.0


def test_running_feedback_at_zero_speed_ref():
    """Regression: PID output 0 % must not look like a failed pump."""
    m = TankModel(state=TankState(run_cmd=True, speed_ref_pct=0.0), seed=0)
    for i in range(200):
        m.set_heartbeat(i); m.step(0.02)
    assert m.pump_running_fb()
