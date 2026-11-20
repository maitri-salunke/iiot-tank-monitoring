"""Vibration feature extraction -- Python twin of firmware vib_dsp.c.

Per window of N samples (X, Y, Z in g) at fs:
  * AC RMS per axis (mean removed), mg
  * Hann-windowed FFT of X -> single-sided peak-amplitude spectrum
  * dominant peak (5 Hz .. fs/2) with parabolic interpolation
  * velocity spectrum v_k = a_k / (2*pi*f_k); velocity RMS over 10 Hz .. fs/2
    via Parseval, corrected by the Hann ENBW (1.5 bins)
  * running speed = velocity peak in 10..60 Hz, "1x" = RMS of +/-1 bin around it
  * v1.1: per-axis velocity RMS, peak, crest factor, kurtosis and mean (DC) per
    axis; X acceleration RMS in 4 bands; velocity at 2x, 3x and 5x (vane pass);
    count of clipped samples
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

G = 9.80665
HANN_ENBW = 1.5


@dataclass
class Features:
    acc_rms_x_mg: float
    acc_rms_y_mg: float
    acc_rms_z_mg: float
    peak_freq_hz: float
    peak_amp_mg: float
    vel_rms_mms: float
    vel_1x_mms: float
    run_speed_hz: float
    # v1.1 -- order must match vib_features_t in firmware vib_dsp.h
    vel_rms_y_mms: float = 0.0
    vel_rms_z_mms: float = 0.0
    acc_peak_x_mg: float = 0.0
    acc_peak_y_mg: float = 0.0
    acc_peak_z_mg: float = 0.0
    crest_x: float = 0.0
    crest_y: float = 0.0
    crest_z: float = 0.0
    kurt_x: float = 0.0
    kurt_y: float = 0.0
    kurt_z: float = 0.0
    band1_mg: float = 0.0
    band2_mg: float = 0.0
    band3_mg: float = 0.0
    band4_mg: float = 0.0
    vel_2x_mms: float = 0.0
    vel_3x_mms: float = 0.0
    vel_5x_mms: float = 0.0
    dc_x_mg: float = 0.0
    dc_y_mg: float = 0.0
    dc_z_mg: float = 0.0
    clip_count: float = 0.0


def hann(n: int) -> np.ndarray:
    k = np.arange(n)
    return 0.5 * (1.0 - np.cos(2.0 * math.pi * k / n))  # periodic Hann


def amplitude_spectrum(x: np.ndarray) -> np.ndarray:
    """Single-sided peak amplitude spectrum (same units as x), bins 0..N/2-1."""
    n = len(x)
    w = hann(n)
    X = np.fft.fft((x - x.mean()) * w)
    amp = 2.0 * np.abs(X[: n // 2]) / (n * 0.5)  # 0.5 = Hann coherent gain
    amp[0] = 0.0
    return amp


BANDS_HZ = [(5.0, 50.0), (50.0, 150.0), (150.0, 300.0), (300.0, 400.0)]
CLIP_G = 3.98


def velocity_spectrum(amp: np.ndarray, df: float) -> tuple[np.ndarray, int]:
    """mm/s peak per bin from a g peak spectrum; bins below 10 Hz zeroed."""
    n2 = len(amp)
    k10 = int(math.ceil(10.0 / df))
    f = np.arange(n2) * df
    vel = np.zeros(n2)
    vel[k10:] = amp[k10:] * G / (2 * math.pi * f[k10:]) * 1000.0
    return vel, k10


def _band_rms(spec: np.ndarray, k0: int, k1: int) -> float:
    """RMS of bins k0..k1 inclusive (peak spectrum, Hann ENBW corrected)."""
    k0, k1 = max(k0, 1), min(k1, len(spec) - 1)
    if k1 < k0:
        return 0.0
    return math.sqrt(float(np.sum(spec[k0:k1 + 1] ** 2) / 2.0) / HANN_ENBW)


def time_stats(s: np.ndarray) -> tuple[float, float, float, float]:
    """(mean mg, peak |s-mean| mg, crest factor, kurtosis m4/m2^2)."""
    m = float(s.mean())
    d = s - m
    m2 = float(np.mean(d * d))
    m4 = float(np.mean(d ** 4))
    pk = float(np.max(np.abs(d)))
    rms = math.sqrt(m2)
    crest = pk / rms if rms > 0 else 0.0
    kurt = m4 / (m2 * m2) if m2 > 0 else 0.0
    return m * 1000.0, pk * 1000.0, crest, kurt


def compute(x: np.ndarray, y: np.ndarray, z: np.ndarray, fs: float) -> Features:
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    n = len(x)
    df = fs / n

    def ac_rms(s):
        s = s - s.mean()
        return float(np.sqrt(np.mean(s * s))) * 1000.0

    amp = amplitude_spectrum(x)  # g
    k_lo = max(1, int(math.ceil(5.0 / df)))
    k_hi = n // 2 - 1
    kp = k_lo + int(np.argmax(amp[k_lo:k_hi]))
    a, b, c = amp[kp - 1], amp[kp], amp[kp + 1]
    den = a - 2 * b + c
    delta = 0.5 * (a - c) / den if den != 0 else 0.0
    peak_freq = (kp + delta) * df
    peak_amp = float(b) * 1000.0

    # velocity spectrum (mm/s peak) for bins >= 10 Hz
    k10 = int(math.ceil(10.0 / df))
    f = np.arange(n // 2) * df
    vel = np.zeros(n // 2)
    vel[k10:] = amp[k10:] * G / (2 * math.pi * f[k10:]) * 1000.0
    vel_rms = math.sqrt(float(np.sum(vel[k10:] ** 2) / 2.0) / HANN_ENBW)

    k60 = int(math.floor(60.0 / df))
    k1 = k10 + int(np.argmax(vel[k10:k60 + 1]))
    band = vel[k1 - 1:k1 + 2]
    vel_1x = math.sqrt(float(np.sum(band ** 2) / 2.0) / HANN_ENBW)

    n2 = n // 2
    harm = []
    for m in (2, 3, 5):
        km = m * k1
        harm.append(_band_rms(vel, km - 1, km + 1) if km + 1 < n2 else 0.0)

    def vel_rms_of(s):
        v, kk = velocity_spectrum(amplitude_spectrum(s), df)
        return math.sqrt(float(np.sum(v[kk:] ** 2) / 2.0) / HANN_ENBW)

    bands = []
    for lo, hi in BANDS_HZ:
        k0 = int(math.ceil(lo / df))
        k1b = min(n2 - 1, int(math.ceil(hi / df)) - 1)
        bands.append(_band_rms(amp, k0, k1b) * 1000.0)
    st = [time_stats(s) for s in (x, y, z)]
    clip = float(sum(int(np.sum(np.abs(s) >= CLIP_G)) for s in (x, y, z)))

    return Features(ac_rms(x), ac_rms(y), ac_rms(z), float(peak_freq), peak_amp,
                    vel_rms, vel_1x, k1 * df,
                    vel_rms_of(y), vel_rms_of(z),
                    st[0][1], st[1][1], st[2][1], st[0][2], st[1][2], st[2][2], st[0][3], st[1][3], st[2][3],
                    *bands, *harm, st[0][0], st[1][0], st[2][0], clip)


def iso_zone(vel_rms_mms: float, limits=(1.4, 2.8, 4.5)) -> int:
    """ISO 10816-3 evaluation zone (group 2 machines, rigid foundation): 1=A .. 4=D."""
    return 1 + sum(vel_rms_mms >= lim for lim in limits)


class AlarmLogic:
    """Debounced alarms (identical to firmware): set after `debounce` consecutive
    windows above threshold, clear after `debounce` consecutive windows below
    90 % of the threshold."""

    def __init__(self):
        self.vel_alert = False
        self.imbalance = False
        self._va_cnt = 0
        self._im_cnt = 0

    @staticmethod
    def _step(active: bool, cnt: int, above: bool, below: bool, debounce: int):
        if not active:
            cnt = cnt + 1 if above else 0
            if cnt >= debounce:
                return True, 0
            return False, cnt
        cnt = cnt + 1 if below else 0
        if cnt >= debounce:
            return False, 0
        return True, cnt

    def update(self, feat: Features, vel_alert_x100: int, vel_1x_x100: int, debounce: int):
        va_thr = vel_alert_x100 / 100.0
        im_thr = vel_1x_x100 / 100.0
        dominant = feat.vel_1x_mms >= 0.7 * feat.vel_rms_mms
        self.vel_alert, self._va_cnt = self._step(
            self.vel_alert, self._va_cnt, feat.vel_rms_mms > va_thr, feat.vel_rms_mms < 0.9 * va_thr, debounce)
        self.imbalance, self._im_cnt = self._step(
            self.imbalance, self._im_cnt, dominant and feat.vel_1x_mms > im_thr,
            feat.vel_1x_mms < 0.9 * im_thr, debounce)
        return self.vel_alert, self.imbalance
