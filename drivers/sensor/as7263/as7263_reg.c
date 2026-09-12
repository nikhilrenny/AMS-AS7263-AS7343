/**
 * @file as7263_reg.c
 * @brief Polled virtual-register read/write for AS7263 over i2c_dt_spec.
 *
 * Only this file touches i2c_dt_spec. Implements the AS726x indirection
 * protocol: poll STATUS.TX_VALID==0, write vreg addr (OR 0x80 for a
 * pending write) to WRITE, poll again, then write/read the data byte.
 * No sensor-API awareness here.
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include "as7263.h"

#define AS7263_POLL_TIMEOUT_MS  1000
#define AS7263_POLL_DELAY_US    500

static int as7263_wait_tx_ready(const struct i2c_dt_spec *bus)
{
    int64_t deadline = k_uptime_get() + AS7263_POLL_TIMEOUT_MS;
    uint8_t status;
    int ret;

    do {
        ret = i2c_reg_read_byte_dt(bus, AS7263_PHYS_STATUS, &status);
        if (ret) {
            return ret;
        }
        if (!(status & AS7263_STATUS_TX_VALID)) {
            return 0;
        }
        k_usleep(AS7263_POLL_DELAY_US);
    } while (k_uptime_get() < deadline);

    return -ETIMEDOUT;
}

static int as7263_wait_rx_valid(const struct i2c_dt_spec *bus)
{
    int64_t deadline = k_uptime_get() + AS7263_POLL_TIMEOUT_MS;
    uint8_t status;
    int ret;

    do {
        ret = i2c_reg_read_byte_dt(bus, AS7263_PHYS_STATUS, &status);
        if (ret) {
            return ret;
        }
        if (status & AS7263_STATUS_RX_VALID) {
            return 0;
        }
        k_usleep(AS7263_POLL_DELAY_US);
    } while (k_uptime_get() < deadline);

    return -ETIMEDOUT;
}

int as7263_vreg_read(const struct i2c_dt_spec *bus, uint8_t vreg, uint8_t *data)
{
    uint8_t stale, status;
    int ret;

    /* Clear any stale RX byte left over from a prior/incomplete transaction
     * before starting a new read (per AS726x datasheet Fig 8, mirrors
     * SparkFun's virtualReadRegister pre-check). Without this the sensor's
     * indirection state machine can desync and every subsequent read hangs. */
    ret = i2c_reg_read_byte_dt(bus, AS7263_PHYS_STATUS, &status);
    if (ret) {
        return ret;
    }
    if (status & AS7263_STATUS_RX_VALID) {
        ret = i2c_reg_read_byte_dt(bus, AS7263_PHYS_READ, &stale);
        if (ret) {
            return ret;
        }
    }

    ret = as7263_wait_tx_ready(bus);

    if (ret) {
        return ret;
    }
    ret = i2c_reg_write_byte_dt(bus, AS7263_PHYS_WRITE, vreg);
    if (ret) {
        return ret;
    }
    ret = as7263_wait_rx_valid(bus);
    if (ret) {
        return ret;
    }
    return i2c_reg_read_byte_dt(bus, AS7263_PHYS_READ, data);
}

int as7263_vreg_write(const struct i2c_dt_spec *bus, uint8_t vreg, uint8_t data)
{
    int ret = as7263_wait_tx_ready(bus);

    if (ret) {
        return ret;
    }
    /* MSB set = pending write, per datasheet Fig 19 */
    ret = i2c_reg_write_byte_dt(bus, AS7263_PHYS_WRITE, (uint8_t)(vreg | 0x80));
    if (ret) {
        return ret;
    }
    ret = as7263_wait_tx_ready(bus);
    if (ret) {
        return ret;
    }
    return i2c_reg_write_byte_dt(bus, AS7263_PHYS_WRITE, data);
}

int as7263_vreg_update(const struct i2c_dt_spec *bus, uint8_t vreg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    int ret = as7263_vreg_read(bus, vreg, &cur);

    if (ret) {
        return ret;
    }
    cur = (cur & ~mask) | (val & mask);
    return as7263_vreg_write(bus, vreg, cur);
}

/* Big-endian 16-bit: vreg_high then vreg_high+1 = low byte */
int as7263_vreg_read16(const struct i2c_dt_spec *bus, uint8_t vreg_high, uint16_t *val)
{
    uint8_t hi, lo;
    int ret = as7263_vreg_read(bus, vreg_high, &hi);

    if (ret) {
        return ret;
    }
    ret = as7263_vreg_read(bus, (uint8_t)(vreg_high + 1), &lo);
    if (ret) {
        return ret;
    }
    *val = ((uint16_t)hi << 8) | lo;
    return 0;
}

/* Big-endian IEEE-754 float across 4 consecutive virtual registers */
int as7263_vreg_read_float(const struct i2c_dt_spec *bus, uint8_t vreg_base, float *val)
{
    uint8_t b[4];
    uint32_t bits;
    int ret;

    for (int i = 0; i < 4; i++) {
        ret = as7263_vreg_read(bus, (uint8_t)(vreg_base + i), &b[i]);
        if (ret) {
            return ret;
        }
    }
    bits = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | b[3];
    memcpy(val, &bits, sizeof(bits));
    return 0;
}