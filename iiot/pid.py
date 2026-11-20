"""Line-for-line Python port of FB_PID in plc/tank_control.st.

Used for offline tuning (tools/tune_offline.py) and unit tests. The PID that
actually controls the running system is the ST one inside OpenPLC.
"""
from __future__ import annotations


class FbPid:
    def __init__(self):
        self.i_term = 0.0
        self.d_term = 0.0
        self.pv_prev = 0.0
        self.init = False
        self.out = 0.0
        self.err = 0.0
        self.saturated = False

    def __call__(self, enable: bool, pv: float, sp: float, kp: float, ti: float, td: float,
                 ts: float, out_min: float = 0.0, out_max: float = 100.0, track: float = 0.0) -> float:
        self.err = sp - pv
        if not self.init:
            self.pv_prev = pv
            self.init = True
        if not enable:
            self.out = track
            self.i_term = track - kp * self.err
            self.d_term = 0.0
            self.pv_prev = pv
            self.saturated = False
            return self.out
        p_term = kp * self.err
        if td > 0.0:
            tf = td / 10.0
            self.d_term = (tf * self.d_term - kp * td * (pv - self.pv_prev)) / (tf + ts)
        else:
            self.d_term = 0.0
        self.pv_prev = pv
        u = p_term + self.i_term + self.d_term
        if ti > 0.0:
            if not ((u >= out_max and self.err > 0.0) or (u <= out_min and self.err < 0.0)):
                self.i_term += kp * ts / ti * self.err
            self.i_term = min(max(self.i_term, out_min), out_max)
        else:
            self.i_term = 0.0
        u = p_term + self.i_term + self.d_term
        self.saturated = False
        if u > out_max:
            u, self.saturated = out_max, True
        if u < out_min:
            u, self.saturated = out_min, True
        self.out = u
        return u


def step_metrics(t: list[float], y: list[float], t_step: float, y0: float, y1: float,
                 band_frac: float = 0.02, ss_window_s: float = 20.0) -> dict:
    """Step-response metrics.

    settling time: last time |y - y1| > band (band = band_frac * |y1 - y0|) after the step,
                   measured from t_step.
    overshoot:     max excursion past y1, % of step size.
    steady band:   min/max and std of (y - y1) over the final ss_window_s seconds.
    rise time:     10 % -> 90 % of the step.
    """
    step = y1 - y0
    band = abs(step) * band_frac
    sign = 1.0 if step >= 0 else -1.0
    after = [(ti, yi) for ti, yi in zip(t, y) if ti >= t_step]
    last_out = t_step
    for ti, yi in after:
        if abs(yi - y1) > band:
            last_out = ti
    peak = max((sign * (yi - y1) for _, yi in after), default=0.0)
    overshoot_pct = max(0.0, peak) / abs(step) * 100.0 if step else 0.0
    t_end = after[-1][0] if after else t_step
    tail = [yi - y1 for ti, yi in after if ti >= t_end - ss_window_s]
    t10 = next((ti for ti, yi in after if sign * (yi - y0) >= 0.1 * abs(step)), float("nan"))
    t90 = next((ti for ti, yi in after if sign * (yi - y0) >= 0.9 * abs(step)), float("nan"))
    mean = sum(tail) / len(tail) if tail else float("nan")
    std = (sum((v - mean) ** 2 for v in tail) / len(tail)) ** 0.5 if tail else float("nan")
    return {
        "settling_time_s": last_out - t_step,
        "settled": (t_end - last_out) >= ss_window_s / 2,
        "overshoot_pct": overshoot_pct,
        "rise_time_s": t90 - t10,
        "ss_min": min(tail) if tail else float("nan"),
        "ss_max": max(tail) if tail else float("nan"),
        "ss_mean": mean,
        "ss_std": std,
        "band_abs": band,
    }
