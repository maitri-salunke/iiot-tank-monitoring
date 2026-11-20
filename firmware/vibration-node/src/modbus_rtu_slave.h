#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MB_MAX_FRAME 256

typedef struct {
    uint8_t unit_id;
    volatile uint16_t *input_regs;  uint16_t n_input;
    volatile uint16_t *holding_regs; uint16_t n_holding;
    /* returns 0 if the write must be rejected (illegal address/value) */
    int (*on_holding_write)(uint16_t reg, uint16_t value);
    uint32_t frames_ok, frames_bad_crc, frames_other_unit;
} mb_slave_t;

uint16_t mb_crc16(const uint8_t *buf, size_t len);

/* Handle one complete request frame (already delimited by t3.5 silence).
 * Writes the response into resp and returns its length; 0 = stay silent. */
size_t mb_slave_handle(mb_slave_t *s, const uint8_t *req, size_t len, uint8_t *resp);

#ifdef __cplusplus
}
#endif
#endif
