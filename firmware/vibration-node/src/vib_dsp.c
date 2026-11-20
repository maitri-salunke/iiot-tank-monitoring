/* Vibration feature extraction. Portable C (no Arduino deps) so the exact
 * same file is compiled for the ESP32 *and* on the host for tests
 * (tests/test_vib_dsp.py loads it via ctypes and compares with iiot/vib_dsp.py). */
#include "vib_dsp.h"
#include <math.h>
#include <string.h>

#define G_MS2 9.80665f
#define HANN_ENBW 1.5f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float s_re[VIB_DSP_N];
static float s_im[VIB_DSP_N];
static float s_hann[VIB_DSP_N];
static float s_amp[VIB_DSP_N / 2];
static float s_vel[VIB_DSP_N / 2];
static int s_init = 0;

static void dsp_init(void) {
    for (int i = 0; i < VIB_DSP_N; i++)
        s_hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)VIB_DSP_N));
    s_init = 1;
}

/* in-place iterative radix-2 Cooley-Tukey FFT */
static void fft(float *re, float *im, int n) {
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / len;
        float wr = (float)cos(ang), wi = (float)sin(ang);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                int a = i + k, b = i + k + len / 2;
                float tr = re[b] * cr - im[b] * ci;
                float ti = re[b] * ci + im[b] * cr;
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr; im[a] += ti;
                float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

static float mean(const float *x, int n) {
    double s = 0; for (int i = 0; i < n; i++) s += x[i]; return (float)(s / n);
}

static float ac_rms_mg(const float *x, int n) {
    float m = mean(x, n); double s = 0;
    for (int i = 0; i < n; i++) { double d = x[i] - m; s += d * d; }
    return (float)sqrt(s / n) * 1000.0f;
}

/* Hann-windowed single-sided peak amplitude spectrum (g) of x into s_amp */
static void amp_spectrum(const float *x, int n) {
    float m = mean(x, n);
    for (int i = 0; i < n; i++) { s_re[i] = (x[i] - m) * s_hann[i]; s_im[i] = 0.0f; }
    fft(s_re, s_im, n);
    for (int k = 0; k < n / 2; k++)
        s_amp[k] = 2.0f * sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]) / ((float)n * 0.5f);
    s_amp[0] = 0.0f;
}

/* RMS of bins k0..k1 of a peak spectrum, Hann ENBW corrected */
static float band_rms(const float *spec, int k0, int k1, int n2) {
    if (k0 < 1) k0 = 1;
    if (k1 > n2 - 1) k1 = n2 - 1;
    if (k1 < k0) return 0.0f;
    double s = 0;
    for (int k = k0; k <= k1; k++) s += (double)spec[k] * spec[k];
    return (float)sqrt(s / 2.0 / HANN_ENBW);
}

/* velocity spectrum (mm/s peak) from s_amp into s_vel; returns k10 */
static int vel_spectrum(float df, int n2) {
    int k10 = (int)ceilf(10.0f / df);
    for (int k = 0; k < n2; k++) {
        if (k < k10) { s_vel[k] = 0.0f; continue; }
        float f = (float)k * df;
        s_vel[k] = s_amp[k] * G_MS2 / (2.0f * (float)M_PI * f) * 1000.0f;
    }
    return k10;
}

static float vel_rms_axis(const float *x, int n, float df) {
    amp_spectrum(x, n);
    int k10 = vel_spectrum(df, n / 2);
    double vs = 0;
    for (int k = k10; k < n / 2; k++) vs += (double)s_vel[k] * s_vel[k];
    return (float)sqrt(vs / 2.0 / HANN_ENBW);
}

/* mean (mg), peak |x-mean| (mg), crest factor, kurtosis m4/m2^2 */
static void time_stats(const float *x, int n, float *dc_mg, float *pk_mg, float *crest, float *kurt) {
    float m = mean(x, n);
    double m2 = 0, m4 = 0, pk = 0;
    for (int i = 0; i < n; i++) {
        double d = (double)x[i] - m, d2 = d * d;
        m2 += d2; m4 += d2 * d2;
        if (fabs(d) > pk) pk = fabs(d);
    }
    m2 /= n; m4 /= n;
    double rms = sqrt(m2);
    *dc_mg = m * 1000.0f;
    *pk_mg = (float)(pk * 1000.0);
    *crest = rms > 0 ? (float)(pk / rms) : 0.0f;
    *kurt = m2 > 0 ? (float)(m4 / (m2 * m2)) : 0.0f;
}

static const float k_bands[4][2] = {{5.0f, 50.0f}, {50.0f, 150.0f}, {150.0f, 300.0f}, {300.0f, 400.0f}};

void vib_compute(const float *x, const float *y, const float *z, float fs, vib_features_t *out) {
    const int n = VIB_DSP_N;
    if (!s_init) dsp_init();
    const float df = fs / (float)n;

    out->acc_rms_x_mg = ac_rms_mg(x, n);
    out->acc_rms_y_mg = ac_rms_mg(y, n);
    out->acc_rms_z_mg = ac_rms_mg(z, n);

    /* Y and Z first: they reuse the spectrum buffers that X needs afterwards */
    out->vel_rms_y_mms = vel_rms_axis(y, n, df);
    out->vel_rms_z_mms = vel_rms_axis(z, n, df);
    time_stats(x, n, &out->dc_x_mg, &out->acc_peak_x_mg, &out->crest_x, &out->kurt_x);
    time_stats(y, n, &out->dc_y_mg, &out->acc_peak_y_mg, &out->crest_y, &out->kurt_y);
    time_stats(z, n, &out->dc_z_mg, &out->acc_peak_z_mg, &out->crest_z, &out->kurt_z);
    int clip = 0;
    for (int i = 0; i < n; i++)
        clip += (fabsf(x[i]) >= VIB_CLIP_G) + (fabsf(y[i]) >= VIB_CLIP_G) + (fabsf(z[i]) >= VIB_CLIP_G);
    out->clip_count = (float)clip;

    amp_spectrum(x, n);
    const int n2 = n / 2;
    for (int b = 0; b < 4; b++) {
        int k0 = (int)ceilf(k_bands[b][0] / df);
        int k1b = (int)ceilf(k_bands[b][1] / df) - 1;
        if (k1b > n2 - 1) k1b = n2 - 1;
        out->band_mg[b] = band_rms(s_amp, k0, k1b, n2) * 1000.0f;
    }

    int k_lo = (int)ceilf(5.0f / df); if (k_lo < 1) k_lo = 1;
    int k_hi = n / 2 - 1;
    int kp = k_lo;
    for (int k = k_lo; k < k_hi; k++) if (s_amp[k] > s_amp[kp]) kp = k;
    float a = s_amp[kp - 1], b = s_amp[kp], c = s_amp[kp + 1];
    float den = a - 2.0f * b + c;
    float delta = (den != 0.0f) ? 0.5f * (a - c) / den : 0.0f;
    out->peak_freq_hz = ((float)kp + delta) * df;
    out->peak_amp_mg = b * 1000.0f;

    int k10 = vel_spectrum(df, n2);
    double vsum = 0;
    for (int k = k10; k < n2; k++) vsum += (double)s_vel[k] * s_vel[k];
    out->vel_rms_mms = (float)sqrt(vsum / 2.0 / HANN_ENBW);

    int k60 = (int)floorf(60.0f / df);
    int k1 = k10;
    for (int k = k10; k <= k60; k++) if (s_vel[k] > s_vel[k1]) k1 = k;
    double b1 = 0;
    for (int k = k1 - 1; k <= k1 + 1; k++) b1 += (double)s_vel[k] * s_vel[k];
    out->vel_1x_mms = (float)sqrt(b1 / 2.0 / HANN_ENBW);
    out->run_speed_hz = (float)k1 * df;
    const int mult[3] = {2, 3, 5};
    float *harm[3] = {&out->vel_2x_mms, &out->vel_3x_mms, &out->vel_5x_mms};
    for (int i = 0; i < 3; i++) {
        int km = mult[i] * k1;
        *harm[i] = (km + 1 < n2) ? band_rms(s_vel, km - 1, km + 1, n2) : 0.0f;
    }
}

static void alarm_step(uint8_t *active, uint16_t *cnt, int above, int below, uint16_t debounce) {
    if (!*active) {
        *cnt = above ? (uint16_t)(*cnt + 1) : 0;
        if (*cnt >= debounce) { *active = 1; *cnt = 0; }
    } else {
        *cnt = below ? (uint16_t)(*cnt + 1) : 0;
        if (*cnt >= debounce) { *active = 0; *cnt = 0; }
    }
}

void vib_alarm_update(vib_alarm_t *st, const vib_features_t *f,
                      uint16_t vel_alert_x100, uint16_t vel_1x_x100, uint16_t debounce) {
    float va = vel_alert_x100 / 100.0f, im = vel_1x_x100 / 100.0f;
    int dominant = f->vel_1x_mms >= 0.7f * f->vel_rms_mms;
    alarm_step(&st->vel_alert, &st->va_cnt, f->vel_rms_mms > va, f->vel_rms_mms < 0.9f * va, debounce);
    alarm_step(&st->imbalance, &st->im_cnt, dominant && f->vel_1x_mms > im, f->vel_1x_mms < 0.9f * im, debounce);
}
