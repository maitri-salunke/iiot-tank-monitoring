/* Modbus RTU slave PDU handler. Portable C; mirrors iiot/modbus_rtu.py RtuSlave.
 * Supported: 0x03 read holding, 0x04 read input, 0x06 write single, 0x10 write multiple. */
#include "modbus_rtu_slave.h"

uint16_t mb_crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static size_t finish(uint8_t *resp, size_t n) {
    uint16_t crc = mb_crc16(resp, n);
    resp[n] = (uint8_t)(crc & 0xFF);
    resp[n + 1] = (uint8_t)(crc >> 8);
    return n + 2;
}

static size_t exception(mb_slave_t *s, uint8_t fc, uint8_t code, uint8_t *resp) {
    resp[0] = s->unit_id; resp[1] = fc | 0x80; resp[2] = code;
    return finish(resp, 3);
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

size_t mb_slave_handle(mb_slave_t *s, const uint8_t *req, size_t len, uint8_t *resp) {
    if (len < 4) { s->frames_bad_crc++; return 0; }
    uint16_t crc = mb_crc16(req, len - 2);
    if ((uint8_t)(crc & 0xFF) != req[len - 2] || (uint8_t)(crc >> 8) != req[len - 1]) {
        s->frames_bad_crc++;
        return 0;                          /* silent: master times out */
    }
    if (req[0] != s->unit_id) { s->frames_other_unit++; return 0; }
    s->frames_ok++;
    uint8_t fc = req[1];

    if (fc == 0x03 || fc == 0x04) {
        if (len != 8) return exception(s, fc, 0x03, resp);
        uint16_t start = be16(req + 2), count = be16(req + 4);
        if (count < 1 || count > 125) return exception(s, fc, 0x03, resp);
        volatile uint16_t *regs = (fc == 0x04) ? s->input_regs : s->holding_regs;
        uint16_t nregs = (fc == 0x04) ? s->n_input : s->n_holding;
        if ((uint32_t)start + count > nregs) return exception(s, fc, 0x02, resp);
        resp[0] = s->unit_id; resp[1] = fc; resp[2] = (uint8_t)(2 * count);
        for (uint16_t i = 0; i < count; i++) {
            uint16_t v = regs[start + i];
            resp[3 + 2 * i] = (uint8_t)(v >> 8);
            resp[4 + 2 * i] = (uint8_t)(v & 0xFF);
        }
        return finish(resp, 3 + 2u * count);
    }
    if (fc == 0x06) {
        if (len != 8) return exception(s, fc, 0x03, resp);
        uint16_t reg = be16(req + 2), val = be16(req + 4);
        if (reg >= s->n_holding || !s->on_holding_write(reg, val)) return exception(s, fc, 0x02, resp);
        for (int i = 0; i < 8; i++) resp[i] = req[i];       /* echo request */
        return 8;
    }
    if (fc == 0x10) {
        if (len < 9) return exception(s, fc, 0x03, resp);
        uint16_t start = be16(req + 2), count = be16(req + 4);
        uint8_t nbytes = req[6];
        if (nbytes != 2 * count || len != 9u + nbytes || count < 1 || count > 123)
            return exception(s, fc, 0x03, resp);
        if ((uint32_t)start + count > s->n_holding) return exception(s, fc, 0x02, resp);
        for (uint16_t i = 0; i < count; i++)
            if (!s->on_holding_write(start + i, be16(req + 7 + 2 * i))) return exception(s, fc, 0x02, resp);
        for (int i = 0; i < 6; i++) resp[i] = req[i];
        return finish(resp, 6);
    }
    return exception(s, fc, 0x01, resp);
}
