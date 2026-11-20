/*
 * ESP32 vibration node -- Modbus RTU slave over RS-485.
 *
 *   ADXL345 (I2C, 800 Hz, +/-4 g full-res, FIFO stream) --> 512-sample window
 *   --> vib_compute() (RMS, Hann FFT, velocity RMS, 1x) --> input registers
 *   --> Modbus RTU slave (unit 10, 19200 8N1) on UART2 via MAX3485.
 *
 * STATUS: compiles with PlatformIO (espressif32 / Arduino). It has NOT been
 * flashed or run on real hardware -- treat timings and the RS-485 DE handling
 * as untested. The Python emulator (services/vibnode) implements the same
 * register map and the same DSP (vib_dsp.c is unit-tested against it on the host).
 *
 * Wiring (assumed):
 *   ADXL345  SDA=GPIO21 SCL=GPIO22 VCC=3V3 CS=3V3 (I2C mode) SDO=GND (addr 0x53)
 *   MAX3485  RO=GPIO16(RX2) DI=GPIO17(TX2) DE+/RE=GPIO4, A/B to bus, 120R at ends
 *   NTC 10k (B=3950) bearing-housing probe: 3V3 -- 10k -- GPIO34 (ADC1_CH6) -- NTC -- GND
 */
#include <Arduino.h>
#include <Wire.h>
#include "regmap.h"
#include "vib_dsp.h"
#include "modbus_rtu_slave.h"

#ifndef VIB_FW_VERSION
#define VIB_FW_VERSION 0x0101
#endif

static const int PIN_SDA = 21, PIN_SCL = 22;
static const int PIN_RS485_RX = 16, PIN_RS485_TX = 17, PIN_RS485_DE = 4;
static const uint8_t ADXL_ADDR = 0x53;
static const float ADXL_G_PER_LSB = 0.0039f;   /* full resolution: 3.9 mg/LSB */
static const int PIN_NTC = 34;
static uint16_t g_i2c_errors = 0, g_fifo_overruns = 0;

/* ---- register banks (shared between sampler task and Modbus loop) ---- */
static volatile uint16_t g_ir[IR_COUNT];
static volatile uint16_t g_hr[HR_COUNT] = {450, 280, 3, VIB_UNIT_ID};
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static mb_slave_t g_mb;
static uint8_t g_rx[MB_MAX_FRAME], g_tx[MB_MAX_FRAME];
static size_t g_rx_len = 0;
static uint32_t g_last_rx_us = 0;
static uint32_t g_t35_us = 2005;  /* 3.5 * 11 bits / 19200 baud */

/* ------------------------------------------------------------ ADXL345 --- */
static bool adxl_write(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(ADXL_ADDR); Wire.write(reg); Wire.write(v);
    return Wire.endTransmission() == 0;
}
static bool adxl_read(uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(ADXL_ADDR); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { g_i2c_errors++; return false; }
    if (Wire.requestFrom((int)ADXL_ADDR, (int)n) != (int)n) { g_i2c_errors++; return false; }
    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}
static bool adxl_init() {
    uint8_t id = 0;
    if (!adxl_read(0x00, &id, 1) || id != 0xE5) return false;   /* DEVID */
    adxl_write(0x2D, 0x00);   /* POWER_CTL: standby while configuring */
    adxl_write(0x2C, 0x0D);   /* BW_RATE: 800 Hz ODR */
    adxl_write(0x31, 0x09);   /* DATA_FORMAT: FULL_RES, +/-4 g */
    adxl_write(0x38, 0x80);   /* FIFO_CTL: stream mode */
    adxl_write(0x2D, 0x08);   /* POWER_CTL: measure */
    return true;
}

/* --------------------------------------------------------- sampler task --- */
static float s_x[VIB_DSP_N], s_y[VIB_DSP_N], s_z[VIB_DSP_N];

/* bearing-housing temperature, degC; NaN if the probe is open/shorted */
static float read_ntc_c() {
    const float R_FIX = 10000.0f, R0 = 10000.0f, BETA = 3950.0f, T0 = 298.15f;
    uint32_t mv = analogReadMilliVolts(PIN_NTC);
    if (mv < 50 || mv > 3250) return NAN;
    float r = R_FIX * (float)mv / (3300.0f - (float)mv);
    return 1.0f / (1.0f / T0 + logf(r / R0) / BETA) - 273.15f;
}

static uint16_t u16r(float v) { return v <= 0 ? 0 : v >= 65535.0f ? 65535 : (uint16_t)lroundf(v); }
static uint16_t s16r(float v) {
    long x = lroundf(v); if (x < -32768) x = -32768; if (x > 32767) x = 32767;
    return (uint16_t)(int16_t)x;
}
static uint16_t iso_zone(float v) { return v < 1.4f ? 1 : v < 2.8f ? 2 : v < 4.5f ? 3 : 4; }

static void publish(const vib_features_t &f, const vib_alarm_t &al, uint16_t status, uint16_t window_count) {
    uint32_t up = millis() / 1000;
    float t = read_ntc_c();
    portENTER_CRITICAL(&g_mux);
    g_ir[IR_STATUS] = status;
    g_ir[IR_ACC_RMS_RADIAL] = (uint16_t)lroundf(f.acc_rms_x_mg);
    g_ir[IR_VEL_RMS] = (uint16_t)lroundf(f.vel_rms_mms * 100.0f);
    g_ir[IR_PEAK_FREQ] = (uint16_t)lroundf(f.peak_freq_hz * 10.0f);
    g_ir[IR_PEAK_AMP] = (uint16_t)lroundf(f.peak_amp_mg);
    g_ir[IR_VEL_1X] = (uint16_t)lroundf(f.vel_1x_mms * 100.0f);
    g_ir[IR_RUN_SPEED] = (uint16_t)lroundf(f.run_speed_hz * 10.0f);
    g_ir[IR_WINDOW_COUNT] = window_count;
    g_ir[IR_UPTIME_HI] = (uint16_t)(up >> 16);
    g_ir[IR_UPTIME_LO] = (uint16_t)(up & 0xFFFF);
    g_ir[IR_ACC_RMS_X] = (uint16_t)lroundf(f.acc_rms_x_mg);
    g_ir[IR_ACC_RMS_Y] = (uint16_t)lroundf(f.acc_rms_y_mg);
    g_ir[IR_ACC_RMS_Z] = (uint16_t)lroundf(f.acc_rms_z_mg);
    g_ir[IR_VEL_RMS_Y] = u16r(f.vel_rms_y_mms * 100.0f);
    g_ir[IR_VEL_RMS_Z] = u16r(f.vel_rms_z_mms * 100.0f);
    g_ir[IR_ACC_PEAK_X] = u16r(f.acc_peak_x_mg);
    g_ir[IR_ACC_PEAK_Y] = u16r(f.acc_peak_y_mg);
    g_ir[IR_ACC_PEAK_Z] = u16r(f.acc_peak_z_mg);
    g_ir[IR_CREST_X] = u16r(f.crest_x * 100.0f);
    g_ir[IR_CREST_Y] = u16r(f.crest_y * 100.0f);
    g_ir[IR_CREST_Z] = u16r(f.crest_z * 100.0f);
    g_ir[IR_KURT_X] = u16r(f.kurt_x * 100.0f);
    g_ir[IR_KURT_Y] = u16r(f.kurt_y * 100.0f);
    g_ir[IR_KURT_Z] = u16r(f.kurt_z * 100.0f);
    for (int b = 0; b < 4; b++) g_ir[IR_BAND1_ACC + b] = u16r(f.band_mg[b] * 10.0f);
    g_ir[IR_VEL_2X] = u16r(f.vel_2x_mms * 100.0f);
    g_ir[IR_VEL_3X] = u16r(f.vel_3x_mms * 100.0f);
    g_ir[IR_VEL_VANE] = u16r(f.vel_5x_mms * 100.0f);
    g_ir[IR_DC_X] = s16r(f.dc_x_mg);
    g_ir[IR_DC_Y] = s16r(f.dc_y_mg);
    g_ir[IR_DC_Z] = s16r(f.dc_z_mg);
    g_ir[IR_TEMP] = isnan(t) ? 0x8000 : s16r(t * 100.0f);   /* 0x8000 = probe fault */
    g_ir[IR_CLIP_COUNT] = u16r(f.clip_count);
    g_ir[IR_FIFO_OVR] = g_fifo_overruns;
    g_ir[IR_I2C_ERR] = g_i2c_errors;
    g_ir[IR_VA_CNT] = al.va_cnt;
    g_ir[IR_IM_CNT] = al.im_cnt;
    g_ir[IR_ISO_ZONE] = iso_zone(f.vel_rms_mms);
    portEXIT_CRITICAL(&g_mux);
}

static void sampler_task(void *) {
    bool sensor_ok = adxl_init();
    vib_alarm_t alarm = {0, 0, 0, 0};
    uint16_t window_count = 0;
    size_t idx = 0;
    uint8_t raw[6];
    for (;;) {
        if (!sensor_ok) {
            portENTER_CRITICAL(&g_mux);
            g_ir[IR_STATUS] = 0;
            portEXIT_CRITICAL(&g_mux);
            vTaskDelay(pdMS_TO_TICKS(1000));
            sensor_ok = adxl_init();
            continue;
        }
        uint8_t fifo = 0;
        if (!adxl_read(0x39, &fifo, 1)) { sensor_ok = false; continue; }
        uint8_t entries = fifo & 0x3F;
        if (entries >= 32) g_fifo_overruns++;   /* FIFO full: stream mode discarded the oldest samples */
        for (uint8_t e = 0; e < entries; e++) {
            if (!adxl_read(0x32, raw, 6)) { sensor_ok = false; break; }
            s_x[idx] = (int16_t)(raw[1] << 8 | raw[0]) * ADXL_G_PER_LSB;
            s_y[idx] = (int16_t)(raw[3] << 8 | raw[2]) * ADXL_G_PER_LSB;
            s_z[idx] = (int16_t)(raw[5] << 8 | raw[4]) * ADXL_G_PER_LSB;
            if (++idx == VIB_DSP_N) {
                idx = 0;
                vib_features_t f;
                vib_compute(s_x, s_y, s_z, (float)VIB_SAMPLE_RATE_HZ, &f);
                uint16_t thr_va, thr_im, deb;
                portENTER_CRITICAL(&g_mux);
                thr_va = g_hr[HR_VEL_ALERT]; thr_im = g_hr[HR_VEL_1X_ALARM]; deb = g_hr[HR_DEBOUNCE];
                portEXIT_CRITICAL(&g_mux);
                vib_alarm_update(&alarm, &f, thr_va, thr_im, deb);
                uint16_t st = STATUS_SENSOR_OK | STATUS_WINDOW_VALID;
                if (alarm.vel_alert) st |= STATUS_VEL_ALERT;
                if (alarm.imbalance) st |= STATUS_IMBALANCE;
                publish(f, alarm, st, ++window_count);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));   /* 32-deep FIFO fills in 40 ms at 800 Hz */
    }
}

/* -------------------------------------------------------------- Modbus --- */
static int on_holding_write(uint16_t reg, uint16_t value) {
    if (reg == HR_UNIT_ID) return 0;                       /* read-only */
    if (reg == HR_DEBOUNCE && (value < 1 || value > 20)) return 0;
    g_hr[reg] = value;     /* caller (loop) already holds g_mux */
    return 1;
}

static void rs485_send(const uint8_t *buf, size_t n) {
    digitalWrite(PIN_RS485_DE, HIGH);              /* driver enable */
    Serial2.write(buf, n);
    Serial2.flush();                               /* wait until TX shift register empty */
    digitalWrite(PIN_RS485_DE, LOW);               /* back to receive */
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);
    Serial2.begin(VIB_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
    Serial2.setRxFIFOFull(1);                      /* deliver bytes promptly so t3.5 timing is meaningful */
    Wire.begin(PIN_SDA, PIN_SCL, 400000);

    g_ir[IR_FW_VERSION] = VIB_FW_VERSION;
    g_ir[IR_SAMPLE_RATE] = VIB_SAMPLE_RATE_HZ;
    g_ir[IR_WINDOW_LEN] = VIB_WINDOW_LEN;

    g_mb.unit_id = VIB_UNIT_ID;
    g_mb.input_regs = g_ir;   g_mb.n_input = IR_COUNT;
    g_mb.holding_regs = g_hr; g_mb.n_holding = HR_COUNT;
    g_mb.on_holding_write = on_holding_write;

    xTaskCreatePinnedToCore(sampler_task, "sampler", 8192, nullptr, 5, nullptr, 0);
    Serial.println("vibration node up (UNTESTED ON HARDWARE)");
}

void loop() {
    while (Serial2.available()) {
        int c = Serial2.read();
        if (g_rx_len < sizeof(g_rx)) g_rx[g_rx_len++] = (uint8_t)c;
        g_last_rx_us = micros();
    }
    if (g_rx_len > 0 && (uint32_t)(micros() - g_last_rx_us) > g_t35_us) {
        /* copy register snapshot consistently while handling */
        portENTER_CRITICAL(&g_mux);
        size_t n = mb_slave_handle(&g_mb, g_rx, g_rx_len, g_tx);
        portEXIT_CRITICAL(&g_mux);
        g_rx_len = 0;
        if (n > 0) rs485_send(g_tx, n);
    }
}
