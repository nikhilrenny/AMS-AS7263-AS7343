/**
 * @file as7263.h
 * @brief Register map and data types for the AMS AS7263 NIR spectral sensor.
 *
 * Pure C, platform-agnostic. AS7263 has no direct-addressed register file —
 * all access goes through 3 physical I2C registers (STATUS/WRITE/READ) using
 * a polled virtual-register protocol (datasheet sec "I2C Slave Interface").
 * No SMUX, no bank switching, single photodiode bank.
 *
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AS7263_H_
#define ZEPHYR_DRIVERS_SENSOR_AS7263_H_

#include <stdint.h>
#include <stddef.h>

#define AS7263_I2C_ADDR      0x49
#define AS7263_NUM_CHANNELS  6

/* Physical I2C registers (the only ones ever addressed directly on the bus) */
#define AS7263_PHYS_STATUS   0x00
#define AS7263_PHYS_WRITE    0x01
#define AS7263_PHYS_READ     0x02

#define AS7263_STATUS_RX_VALID  BIT(0)
#define AS7263_STATUS_TX_VALID  BIT(1)

/* Virtual register addresses (accessed via the WRITE/READ indirection) */
#define AS7263_VREG_HW_TYPE      0x00 /* device type, expect ~0x20 per datasheet */
#define AS7263_VREG_HW_VERSION   0x01
#define AS7263_VREG_FW_SUB       0x02
#define AS7263_VREG_FW_MAJMIN    0x03
#define AS7263_VREG_CONTROL      0x04 /* RST[7] INT[6] GAIN[5:4] BANK[3:2] DATA_RDY[1] */
#define AS7263_VREG_INT_T        0x05 /* integration time = value * 2.8ms */
#define AS7263_VREG_DEVICE_TEMP  0x06
#define AS7263_VREG_LED_CONTROL  0x07

#define AS7263_VREG_R_HIGH  0x08
#define AS7263_VREG_R_LOW   0x09
#define AS7263_VREG_S_HIGH  0x0A
#define AS7263_VREG_S_LOW   0x0B
#define AS7263_VREG_T_HIGH  0x0C
#define AS7263_VREG_T_LOW   0x0D
#define AS7263_VREG_U_HIGH  0x0E
#define AS7263_VREG_U_LOW   0x0F
#define AS7263_VREG_V_HIGH  0x10
#define AS7263_VREG_V_LOW   0x11
#define AS7263_VREG_W_HIGH  0x12
#define AS7263_VREG_W_LOW   0x13

#define AS7263_VREG_R_CAL  0x14 /* 4-byte IEEE-754 float, big-endian */
#define AS7263_VREG_S_CAL  0x18
#define AS7263_VREG_T_CAL  0x1C
#define AS7263_VREG_U_CAL  0x20
#define AS7263_VREG_V_CAL  0x24
#define AS7263_VREG_W_CAL  0x28

/* Control_Setup bitfield */
typedef union {
    struct {
        uint8_t rsvd     : 1;
        uint8_t data_rdy : 1;
        uint8_t bank     : 2;
        uint8_t gain     : 2;
        uint8_t interrupt: 1;
        uint8_t rst      : 1;
    };
    uint8_t byte;
} as7263_reg_control_t;

typedef enum {
    AS7263_GAIN_1X = 0x00,
    AS7263_GAIN_3_7X,
    AS7263_GAIN_16X,
    AS7263_GAIN_64X,
} as7263_gain_t;

typedef enum {
    AS7263_BANK_MODE_0 = 0x00, /* continuous: S,T,U,V */
    AS7263_BANK_MODE_1,        /* continuous: R,T,U,W */
    AS7263_BANK_MODE_2,        /* continuous: all six */
    AS7263_BANK_MODE_3_ONESHOT,/* one-shot: all six, triggered by writing this */
} as7263_bank_t;

typedef enum {
    AS7263_CH_R_610NM = 0,
    AS7263_CH_S_680NM,
    AS7263_CH_T_730NM,
    AS7263_CH_U_760NM,
    AS7263_CH_V_810NM,
    AS7263_CH_W_860NM,
} as7263_channel_t;

struct as7263_dev {
    uint16_t data[AS7263_NUM_CHANNELS]; /* last-read raw channel values */
};

void as7263_dev_init(struct as7263_dev *dev);

/* Virtual-register I/O — implemented in as7263_reg.c, over i2c_dt_spec.
 * Every access is a polled write-then-read transaction (datasheet Fig 8/13);
 * there is no burst mode. Callers should expect these to block briefly. */
struct i2c_dt_spec;
int as7263_vreg_read(const struct i2c_dt_spec *bus, uint8_t vreg, uint8_t *data);
int as7263_vreg_write(const struct i2c_dt_spec *bus, uint8_t vreg, uint8_t data);
int as7263_vreg_read16(const struct i2c_dt_spec *bus, uint8_t vreg_high, uint16_t *val);
int as7263_vreg_read_float(const struct i2c_dt_spec *bus, uint8_t vreg_base, float *val);
int as7263_vreg_update(const struct i2c_dt_spec *bus, uint8_t vreg, uint8_t mask, uint8_t val);

#endif /* ZEPHYR_DRIVERS_SENSOR_AS7263_H_ */