/**
 * @file as7343_reg.c
 * @brief Raw register read/write for AS7343 over Zephyr i2c_dt_spec.
 *
 * Only this file touches i2c_dt_spec. Handles REG_BANK switching:
 * registers 0x58-0x66 need REG_BANK=1, registers 0x80+ need REG_BANK=0
 * (set via CFG0 bit 4). No sensor-API awareness here.
 */

#include <zephyr/drivers/i2c.h>
#include "as7343.h"

#define AS7343_BANK_SPLIT 0x80 /* addrs >= this use bank 0, below use bank 1 */

/* Tracks last-set bank per device to skip redundant writes. -1 = unknown. */
struct as7343_reg_ctx {
    int8_t current_bank; /* -1 unknown, 0 or 1 */
};

static struct as7343_reg_ctx ctx = { .current_bank = -1 };

static int as7343_set_bank(const struct i2c_dt_spec *bus, uint8_t bank)
{
    uint8_t cfg0;
    int ret;

    if (ctx.current_bank == (int8_t)bank) {
        return 0; /* already set */
    }

    ret = i2c_reg_read_byte_dt(bus, AS7343_REG_CFG0, &cfg0);
    if (ret) {
        return ret;
    }

    if (bank) {
        cfg0 |= BIT(4);
    } else {
        cfg0 &= ~BIT(4);
    }

    ret = i2c_reg_write_byte_dt(bus, AS7343_REG_CFG0, cfg0);
    if (ret) {
        return ret;
    }

    ctx.current_bank = (int8_t)bank;
    return 0;
}

static inline uint8_t as7343_bank_for(uint8_t reg)
{
    return (reg >= AS7343_BANK_SPLIT) ? 0 : 1;
}

int as7343_reg_read(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *data)
{
    int ret = as7343_set_bank(bus, as7343_bank_for(reg));

    if (ret) {
        return ret;
    }
    return i2c_reg_read_byte_dt(bus, reg, data);
}

int as7343_reg_write(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t data)
{
    int ret = as7343_set_bank(bus, as7343_bank_for(reg));

    if (ret) {
        return ret;
    }
    return i2c_reg_write_byte_dt(bus, reg, data);
}

int as7343_reg_read_burst(const struct i2c_dt_spec *bus, uint8_t reg,
               uint8_t *data, size_t len)
{
    int ret = as7343_set_bank(bus, as7343_bank_for(reg));

    if (ret) {
        return ret;
    }
    return i2c_burst_read_dt(bus, reg, data, len);
}

int as7343_reg_write_burst(const struct i2c_dt_spec *bus, uint8_t reg,
                const uint8_t *data, size_t len)
{
    int ret = as7343_set_bank(bus, as7343_bank_for(reg));

    if (ret) {
        return ret;
    }
    return i2c_burst_write_dt(bus, reg, data, len);
}

/* 16-bit registers latch LSB-then-MSB on write, LSB-then-MSB on read (datasheet sec 9). */
int as7343_reg_write16(const struct i2c_dt_spec *bus, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };

    return as7343_reg_write_burst(bus, reg, buf, sizeof(buf));
}

int as7343_reg_read16(const struct i2c_dt_spec *bus, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    int ret = as7343_reg_read_burst(bus, reg, buf, sizeof(buf));

    if (ret) {
        return ret;
    }
    *val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return 0;
}

/* Set a bit-bank of a register, preserving other bits. */
int as7343_reg_update(const struct i2c_dt_spec *bus, uint8_t reg,
               uint8_t mask, uint8_t val)
{
    uint8_t cur;
    int ret = as7343_reg_read(bus, reg, &cur);

    if (ret) {
        return ret;
    }
    cur = (cur & ~mask) | (val & mask);
    return as7343_reg_write(bus, reg, cur);
}