/* Vibration node Modbus register map. Keep in sync with iiot/vib_regmap.py
 * (tests/test_regmap_sync.py enforces it). Unit id 10, 19200 8N1, RS-485. */
#ifndef REGMAP_H
#define REGMAP_H

#define VIB_UNIT_ID        10
#define VIB_BAUD           19200

/* Input registers (FC 0x04) */
#define IR_STATUS          0   /* bit field */
#define IR_ACC_RMS_RADIAL  1   /* mg */
#define IR_VEL_RMS         2   /* mm/s x100, 10..400 Hz */
#define IR_PEAK_FREQ       3   /* Hz x10 */
#define IR_PEAK_AMP        4   /* mg */
#define IR_VEL_1X          5   /* mm/s x100 */
#define IR_RUN_SPEED       6   /* Hz x10 */
#define IR_WINDOW_COUNT    7
#define IR_UPTIME_HI       8
#define IR_UPTIME_LO       9
#define IR_ACC_RMS_X       10  /* mg */
#define IR_ACC_RMS_Y       11  /* mg */
#define IR_ACC_RMS_Z       12  /* mg */
#define IR_FW_VERSION      13
#define IR_SAMPLE_RATE     14  /* Hz */
#define IR_WINDOW_LEN      15
/* v1.1 extension */
#define IR_VEL_RMS_Y       16  /* mm/s x100 */
#define IR_VEL_RMS_Z       17  /* mm/s x100 */
#define IR_ACC_PEAK_X      18  /* mg, max |a - mean| */
#define IR_ACC_PEAK_Y      19
#define IR_ACC_PEAK_Z      20
#define IR_CREST_X         21  /* x100, peak / AC RMS */
#define IR_CREST_Y         22
#define IR_CREST_Z         23
#define IR_KURT_X          24  /* x100, m4 / m2^2 */
#define IR_KURT_Y          25
#define IR_KURT_Z          26
#define IR_BAND1_ACC       27  /* mg x10, X 5..50 Hz */
#define IR_BAND2_ACC       28  /* mg x10, X 50..150 Hz */
#define IR_BAND3_ACC       29  /* mg x10, X 150..300 Hz */
#define IR_BAND4_ACC       30  /* mg x10, X 300..400 Hz */
#define IR_VEL_2X          31  /* mm/s x100 */
#define IR_VEL_3X          32  /* mm/s x100 */
#define IR_VEL_VANE        33  /* mm/s x100, 5x */
#define IR_DC_X            34  /* mg, int16 */
#define IR_DC_Y            35
#define IR_DC_Z            36
#define IR_TEMP            37  /* degC x100, int16, NTC on ADC1 */
#define IR_CLIP_COUNT      38
#define IR_FIFO_OVR        39
#define IR_I2C_ERR         40
#define IR_VA_CNT          41
#define IR_IM_CNT          42
#define IR_ISO_ZONE        43  /* 1..4 = A..D */
#define IR_COUNT           44

/* Holding registers (FC 0x03 / 0x06 / 0x10) */
#define HR_VEL_ALERT       0   /* mm/s x100, default 450 */
#define HR_VEL_1X_ALARM    1   /* mm/s x100, default 280 */
#define HR_DEBOUNCE        2   /* windows, default 3 */
#define HR_UNIT_ID         3   /* read-only */
#define HR_COUNT           4

#define STATUS_SENSOR_OK     (1u << 0)
#define STATUS_WINDOW_VALID  (1u << 1)
#define STATUS_VEL_ALERT     (1u << 2)
#define STATUS_IMBALANCE     (1u << 3)

#define VIB_SAMPLE_RATE_HZ 800
#define VIB_WINDOW_LEN     512

#endif
