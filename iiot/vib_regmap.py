"""Vibration node register map -- single source of truth for Python.

Kept in sync with firmware/vibration-node/include/regmap.h; tests/test_regmap_sync.py
parses that header and fails if the two drift apart.

Link: Modbus RTU, RS-485 half duplex, 19200 baud 8N1, unit id 10.
All registers are 16-bit unsigned unless noted; scaling in the comment.
"""

UNIT_ID = 10
BAUD = 19200

# ---- Input registers (FC 0x04, read-only) ----
IR_STATUS = 0          # bit field, see STATUS_*
IR_ACC_RMS_RADIAL = 1  # mg, AC RMS of X (radial horizontal)
IR_VEL_RMS = 2         # mm/s x100, velocity RMS 10..400 Hz (ISO 10816 style)
IR_PEAK_FREQ = 3       # Hz x10, dominant FFT peak (5..400 Hz) of X
IR_PEAK_AMP = 4        # mg, amplitude of that peak
IR_VEL_1X = 5          # mm/s x100, velocity RMS in +/-1 bin around running speed
IR_RUN_SPEED = 6       # Hz x10, running speed estimate (velocity peak 10..60 Hz)
IR_WINDOW_COUNT = 7    # increments once per completed FFT window (wraps)
IR_UPTIME_HI = 8       # seconds since boot, high word
IR_UPTIME_LO = 9       # seconds since boot, low word
IR_ACC_RMS_X = 10      # mg
IR_ACC_RMS_Y = 11      # mg
IR_ACC_RMS_Z = 12      # mg (gravity removed: AC RMS)
IR_FW_VERSION = 13     # 0xMMmm
IR_SAMPLE_RATE = 14    # Hz
IR_WINDOW_LEN = 15     # samples per FFT window
# ---- v1.1 extension: per-axis / statistical / spectral features, diagnostics ----
IR_VEL_RMS_Y = 16      # mm/s x100, velocity RMS 10..400 Hz, Y (radial vertical)
IR_VEL_RMS_Z = 17      # mm/s x100, velocity RMS 10..400 Hz, Z (axial)
IR_ACC_PEAK_X = 18     # mg, max |a - mean| in the window
IR_ACC_PEAK_Y = 19     # mg
IR_ACC_PEAK_Z = 20     # mg
IR_CREST_X = 21        # x100, crest factor = peak / AC RMS (sine 1.41, impacts > 3)
IR_CREST_Y = 22        # x100
IR_CREST_Z = 23        # x100
IR_KURT_X = 24         # x100, kurtosis m4/m2^2 (sine 1.5, Gaussian 3, impacting bearing > 4)
IR_KURT_Y = 25         # x100
IR_KURT_Z = 26         # x100
IR_BAND1_ACC = 27      # mg x10, X acceleration RMS 5..50 Hz    (1x/2x: imbalance, misalignment)
IR_BAND2_ACC = 28      # mg x10, X acceleration RMS 50..150 Hz  (harmonics, vane pass)
IR_BAND3_ACC = 29      # mg x10, X acceleration RMS 150..300 Hz (bearing defect frequencies)
IR_BAND4_ACC = 30      # mg x10, X acceleration RMS 300..400 Hz (bearing resonance / impacts)
IR_VEL_2X = 31         # mm/s x100, velocity RMS +/-1 bin at 2x running speed (misalignment)
IR_VEL_3X = 32         # mm/s x100, at 3x (looseness / misalignment)
IR_VEL_VANE = 33       # mm/s x100, at 5x = vane-pass of a 5-vane impeller
IR_DC_X = 34           # mg, signed int16: window mean (gravity / mounting orientation)
IR_DC_Y = 35           # mg, signed
IR_DC_Z = 36           # mg, signed
IR_TEMP = 37           # degC x100, signed: bearing-housing NTC on ADC1 (SIMULATED in the emulator)
IR_CLIP_COUNT = 38     # samples at +/- full scale in the last window (sensor saturation)
IR_FIFO_OVR = 39       # ADXL345 FIFO overruns since boot (wraps)
IR_I2C_ERR = 40        # I2C transaction errors since boot (wraps)
IR_VA_CNT = 41         # velocity-alert debounce counter (windows toward set/clear)
IR_IM_CNT = 42         # imbalance debounce counter
IR_ISO_ZONE = 43       # ISO 10816-3 zone from vel RMS (group 2, rigid): 1 A <1.4, 2 B <2.8, 3 C <4.5, 4 D
IR_COUNT = 44

# ---- Holding registers (FC 0x03 read, 0x06/0x10 write) ----
HR_VEL_ALERT = 0       # mm/s x100, overall velocity alert threshold (default 450 = 4.5 mm/s)
HR_VEL_1X_ALARM = 1    # mm/s x100, imbalance (1x) alarm threshold (default 280 = 2.8 mm/s)
HR_DEBOUNCE = 2        # windows above/below threshold before set/clear (default 3)
HR_UNIT_ID = 3         # read-only mirror of unit id (writes rejected)
HR_COUNT = 4

HR_DEFAULTS = [450, 280, 3, UNIT_ID]

# ---- Status bits ----
STATUS_SENSOR_OK = 1 << 0      # ADXL345 DEVID read back 0xE5 and FIFO is delivering
STATUS_WINDOW_VALID = 1 << 1   # at least one full window computed
STATUS_VEL_ALERT = 1 << 2      # overall velocity above HR_VEL_ALERT (debounced)
STATUS_IMBALANCE = 1 << 3      # 1x dominant and above HR_VEL_1X_ALARM (debounced)

FW_VERSION = 0x0101
SAMPLE_RATE_HZ = 800           # ADXL345 BW_RATE = 0x0D
WINDOW_LEN = 512               # 0.64 s window, 1.5625 Hz bins
CLIP_G = 3.98                  # |a| at or above this counts as clipped (+/-4 g range)
BANDS_HZ = [(5.0, 50.0), (50.0, 150.0), (150.0, 300.0), (300.0, 400.0)]
ISO_ZONE_LIMITS_MMS = (1.4, 2.8, 4.5)
