"""SIMULATED process: one open tank, a variable-speed fill pump, a manual
outlet valve (disturbance).  Not a model of any specific real plant.

    A * dh/dt = Qin - Qout
    Qin  = Qmax * n_pump/100 * wet          (wet = 0 when the suction runs dry)
    Qout = Cv * valve/100 * sqrt(h)          (gravity drain through the valve)
    pump speed follows its reference through a first-order lag (VFD ramp)

Numbers chosen so a 20 % level step takes tens of seconds (lab-scale 50 L tank).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


@dataclass
class TankParams:
    area_m2: float = 0.05          # 50 L over 1 m height
    height_m: float = 1.0
    qmax_lps: float = 1.0          # pump at 100 % (60 L/min)
    cv_lps_sqrtm: float = 1.414    # valve 50 % at h=0.5 m drains 0.5 L/s
    pump_tau_s: float = 1.0        # VFD/pump speed lag
    level_noise_pct: float = 0.10  # 1-sigma transmitter noise, % of span
    flow_noise_frac: float = 0.005
    lshh_pct: float = 95.0         # physical float switches
    lsll_pct: float = 5.0
    heartbeat_timeout_s: float = 2.0


@dataclass
class TankState:
    h_m: float = 0.30
    speed_pct: float = 0.0         # actual pump speed
    valve_pct: float = 50.0
    run_cmd: bool = False
    speed_ref_pct: float = 0.0
    estop_pressed: bool = False    # field E-stop (hardwired: cuts contactor directly)
    dry_run_fault: bool = False    # supply lost -> pump runs dry
    hb_last: int = -1
    hb_age_s: float = 0.0
    t_s: float = 0.0
    qin_lps: float = 0.0
    qout_lps: float = 0.0


class TankModel:
    def __init__(self, params: TankParams | None = None, state: TankState | None = None, seed: int | None = None):
        self.p = params or TankParams()
        self.s = state or TankState()
        self.rng = random.Random(seed)

    # ----- what the motor contactor actually does --------------------------------
    def pump_energised(self) -> bool:
        s = self.s
        comm_ok = s.hb_age_s < self.p.heartbeat_timeout_s
        return s.run_cmd and not s.estop_pressed and comm_ok

    def set_heartbeat(self, hb: int) -> None:
        if hb != self.s.hb_last:
            self.s.hb_last = hb
            self.s.hb_age_s = 0.0

    def step(self, dt: float) -> None:
        p, s = self.p, self.s
        s.t_s += dt
        s.hb_age_s += dt
        target = s.speed_ref_pct if self.pump_energised() else 0.0
        target = min(max(target, 0.0), 100.0)
        s.speed_pct += (target - s.speed_pct) * (1.0 - math.exp(-dt / p.pump_tau_s))
        wet = 0.0 if s.dry_run_fault else 1.0
        s.qin_lps = p.qmax_lps * s.speed_pct / 100.0 * wet
        s.qout_lps = p.cv_lps_sqrtm * s.valve_pct / 100.0 * math.sqrt(max(s.h_m, 0.0))
        s.h_m += (s.qin_lps - s.qout_lps) / 1000.0 / p.area_m2 * dt
        s.h_m = min(max(s.h_m, 0.0), p.height_m)  # overflow weir / empty

    # ----- sensors ------------------------------------------------------------------
    def level_pct_true(self) -> float:
        return self.s.h_m / self.p.height_m * 100.0

    def level_pct_meas(self) -> float:
        v = self.level_pct_true() + self.rng.gauss(0.0, self.p.level_noise_pct)
        return min(max(v, 0.0), 100.0)

    def flow_in_lpm(self) -> float:
        return max(0.0, self.s.qin_lps * 60.0 * (1 + self.rng.gauss(0, self.p.flow_noise_frac)))

    def flow_out_lpm(self) -> float:
        return max(0.0, self.s.qout_lps * 60.0 * (1 + self.rng.gauss(0, self.p.flow_noise_frac)))

    def motor_current_a(self) -> float:
        n = self.s.speed_pct / 100.0
        if n <= 0.01:
            return 0.0
        load = 0.3 if self.s.dry_run_fault else 1.0   # dry pump draws far less power
        return 1.2 * n + 3.3 * n ** 3 * load

    def pump_running_fb(self) -> bool:
        # VFD "running" status: true while the drive is enabled, even at 0 % speed
        # (an earlier version used speed > 1 %, which made the PLC's fail-to-start
        # trip whenever the PID asked for zero flow -- found by bench/step_response.py)
        return self.pump_energised()

    def lshh(self) -> bool:
        return self.level_pct_true() >= self.p.lshh_pct

    def lsll(self) -> bool:
        return self.level_pct_true() <= self.p.lsll_pct
