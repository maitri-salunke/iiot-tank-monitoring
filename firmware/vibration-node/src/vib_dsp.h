#ifndef VIB_DSP_H
#define VIB_DSP_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define VIB_DSP_N 512

typedef struct {
    float acc_rms_x_mg, acc_rms_y_mg, acc_rms_z_mg;
    float peak_freq_hz, peak_amp_mg;
    float vel_rms_mms, vel_1x_mms, run_speed_hz;
    /* v1.1 -- order mirrored by iiot/vib_dsp.py Features and tests/test_vib_dsp.py */
    float vel_rms_y_mms, vel_rms_z_mms;
    float acc_peak_x_mg, acc_peak_y_mg, acc_peak_z_mg;
    float crest_x, crest_y, crest_z;
    float kurt_x, kurt_y, kurt_z;
    float band_mg[4];          /* X accel RMS 5-50, 50-150, 150-300, 300-400 Hz */
    float vel_2x_mms, vel_3x_mms, vel_5x_mms;
    float dc_x_mg, dc_y_mg, dc_z_mg;
    float clip_count;          /* samples with |a| >= VIB_CLIP_G, all axes */
} vib_features_t;

#define VIB_CLIP_G 3.98f

typedef struct {
    uint8_t vel_alert, imbalance;
    uint16_t va_cnt, im_cnt;
} vib_alarm_t;

/* x, y, z: VIB_DSP_N samples each, in g. */
void vib_compute(const float *x, const float *y, const float *z, float fs, vib_features_t *out);
void vib_alarm_update(vib_alarm_t *st, const vib_features_t *f,
                      uint16_t vel_alert_x100, uint16_t vel_1x_x100, uint16_t debounce);

#ifdef __cplusplus
}
#endif
#endif
