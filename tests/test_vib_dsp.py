import ctypes, dataclasses, math, os, shutil, subprocess
import numpy as np
import pytest
from iiot.vib_dsp import compute, AlarmLogic, Features, G, iso_zone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware", "vibration-node", "src")
FS, N = 800.0, 512


def tone(v_rms_mms, f, n=N, fs=FS):
    t = np.arange(n) / fs
    a = v_rms_mms * math.sqrt(2) / 1000 * 2 * math.pi * f / G
    return a * np.sin(2 * math.pi * f * t)


def test_velocity_rms_of_pure_tone():
    x = tone(3.0, 25.0)
    f = compute(x, x * 0, x * 0 + 1.0, FS)
    assert abs(f.vel_rms_mms - 3.0) / 3.0 < 0.05
    assert abs(f.vel_1x_mms - 3.0) / 3.0 < 0.05
    assert abs(f.run_speed_hz - 25.0) < 1.6
    assert abs(f.peak_freq_hz - 25.0) < 0.5           # parabolic interpolation


def test_alarm_debounce():
    al = AlarmLogic()
    hi = compute(tone(4.0, 25.0), np.zeros(N), np.ones(N), FS)
    lo = compute(tone(1.0, 25.0), np.zeros(N), np.ones(N), FS)
    assert al.update(hi, 450, 280, 3)[1] is False
    assert al.update(hi, 450, 280, 3)[1] is False
    assert al.update(hi, 450, 280, 3)[1] is True       # 3rd window sets
    al.update(lo, 450, 280, 3); al.update(lo, 450, 280, 3)
    assert al.update(lo, 450, 280, 3)[1] is False      # 3 windows below 90 % clears


FEATURE_NAMES = [f.name for f in dataclasses.fields(Features)]


class Feat(ctypes.Structure):
    # vib_features_t is all floats, in the same order as the Python dataclass
    _fields_ = [(n, ctypes.c_float) for n in FEATURE_NAMES]


def test_crest_kurtosis_dc_of_pure_tone():
    x = tone(3.0, 25.0)
    f = compute(x, x * 0, x * 0 + 1.0, FS)
    assert abs(f.crest_x - math.sqrt(2)) < 0.02          # sine: crest sqrt(2)
    assert abs(f.kurt_x - 1.5) < 0.02                    # sine: kurtosis 1.5
    assert abs(f.dc_z_mg - 1000.0) < 0.5 and abs(f.dc_x_mg) < 1.0
    assert f.clip_count == 0


def test_impacts_raise_kurtosis_and_crest():
    rng = np.random.default_rng(3)
    base = tone(1.5, 24.75) + rng.normal(0, 0.004, N)
    imp = base.copy()
    t = np.arange(N) / FS
    for t0 in np.arange(0.01, 0.64, 1 / 76.7):            # ~3.1x BPFO-like impact train
        k = t >= t0
        imp[k] += 0.4 * np.exp(-(t[k] - t0) * 300) * np.sin(2 * math.pi * 350 * (t[k] - t0))
    fb, fi = compute(base, base * 0, base * 0 + 1, FS), compute(imp, imp * 0, imp * 0 + 1, FS)
    assert fi.kurt_x > fb.kurt_x + 1.0 and fi.crest_x > fb.crest_x + 1.0
    assert fi.band4_mg > 3 * fb.band4_mg                  # 300-400 Hz resonance band lights up


def test_band_energy_and_harmonics_land_in_the_right_bins():
    x = tone(2.0, 24.75) + tone(1.0, 2 * 24.75) + tone(0.5, 5 * 24.75)
    f = compute(x, x * 0, x * 0 + 1, FS)
    assert abs(f.vel_2x_mms - 1.0) < 0.1 and abs(f.vel_5x_mms - 0.5) < 0.06 and f.vel_3x_mms < 0.1
    tot = math.sqrt(f.band1_mg ** 2 + f.band2_mg ** 2 + f.band3_mg ** 2 + f.band4_mg ** 2)
    assert abs(tot - f.acc_rms_x_mg) / f.acc_rms_x_mg < 0.05   # Parseval across the bands
    assert f.band3_mg < 0.05 * tot and f.band4_mg < 0.05 * tot


def test_per_axis_velocity_and_iso_zone():
    x = tone(3.0, 25.0)
    f = compute(x, 0.5 * x, 0.25 * x + 1.0, FS)
    assert abs(f.vel_rms_y_mms - 1.5) < 0.08 and abs(f.vel_rms_z_mms - 0.75) < 0.05
    assert [iso_zone(v) for v in (0.5, 1.4, 2.79, 3.0, 4.5, 9.0)] == [1, 2, 2, 3, 4, 4]


def test_clipping_counted():
    x = np.clip(tone(3.0, 25.0) * 0 + 4.2 * np.sin(np.arange(N) / 5), -4.0, 4.0)
    f = compute(x, x * 0, x * 0 + 1, FS)
    assert f.clip_count > 0


@pytest.fixture(scope="module")
def clib(tmp_path_factory):
    cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        pytest.skip("no C compiler")
    out = str(tmp_path_factory.mktemp("c") / "libvib.so")
    subprocess.check_call([cc, "-O2", "-shared", "-fPIC", "-o", out,
                           os.path.join(FW, "vib_dsp.c"), os.path.join(FW, "modbus_rtu_slave.c"), "-lm"])
    return ctypes.CDLL(out)


def test_firmware_dsp_matches_python_twin(clib):
    rng = np.random.default_rng(1)
    for v1 in (1.2, 2.5, 5.0):
        t = np.arange(N) / FS
        x = tone(v1, 24.75) + tone(0.4, 49.5) + tone(0.35, 123.75) + rng.normal(0, 0.004, N)
        y, z = 0.8 * x, 1.0 + 0.3 * x
        py = compute(x, y, z, FS)
        arr = lambda a: (ctypes.c_float * N)(*a.astype(np.float32))
        f = Feat()
        clib.vib_compute(arr(x), arr(y), arr(z), ctypes.c_float(FS), ctypes.byref(f))
        for name in FEATURE_NAMES:
            c_val, p_val = getattr(f, name), getattr(py, name)
            assert abs(c_val - p_val) <= 1e-3 * max(1.0, abs(p_val)), (name, c_val, p_val)


def test_firmware_modbus_crc_matches_python(clib):
    from iiot.modbus_rtu import crc16
    clib.mb_crc16.restype = ctypes.c_uint16
    for data in (b"123456789", bytes([1, 3, 0, 0, 0, 10]), bytes(range(200))):
        assert clib.mb_crc16(data, len(data)) == crc16(data)
