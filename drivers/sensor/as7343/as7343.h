/**
 * @file as7343.h
 * @brief Register map and data types for the AMS AS7343 multispectral sensor.
 *
 * Pure C, platform-agnostic. No I2C/bus calls here — see as7343_reg.c for
 * register access over Zephyr's i2c_dt_spec, and as7343.c for the Zephyr
 * sensor_driver_api glue.
 *
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AS7343_H_
#define ZEPHYR_DRIVERS_SENSOR_AS7343_H_

#include <stdint.h>
#include <stddef.h>

/* ------------------------------------------------------------------------ */
/* I2C addressing                                                          */
/* ------------------------------------------------------------------------ */

#define AS7343_I2C_ADDR   0x39
#define AS7343_DEVICE_ID  0x81  /* expected value of ID register on boot */
#define AS7343_NUM_CHANNELS 18

/* ------------------------------------------------------------------------ */
/* Enum definitions                                                        */
/* ------------------------------------------------------------------------ */

typedef enum {
    AS7343_REG_BANK_0 = 0x00, /* default */
    AS7343_REG_BANK_1 = 0x01,
} as7343_reg_bank_t;

/*
 * Sensor channels. Datasheet names: FZ, FY, FXL, NIR, 2xVIS, FD, F1-F8.
 * With auto_smux = 18 channels, read-out order is:
 *   Cycle 1: FZ, FY, FXL, NIR, 2xVIS, FD
 *   Cycle 2: F2, F3, F4, F6, 2xVIS, FD
 *   Cycle 3: F1, F7, F8, F5, 2xVIS, FD
 */
typedef enum {
    AS7343_CH_BLUE_FZ_450NM = 0,
    AS7343_CH_GREEN_FY_555NM,
    AS7343_CH_ORANGE_FXL_600NM,
    AS7343_CH_NIR_855NM,
    AS7343_CH_VIS_1,
    AS7343_CH_FD_1,
    AS7343_CH_DARK_BLUE_F2_425NM,
    AS7343_CH_LIGHT_BLUE_F3_475NM,
    AS7343_CH_BLUE_F4_515NM,
    AS7343_CH_BROWN_F6_640NM,
    AS7343_CH_VIS_2,
    AS7343_CH_FD_2,
    AS7343_CH_PURPLE_F1_405NM,
    AS7343_CH_RED_F7_690NM,
    AS7343_CH_DARK_RED_F8_745NM,
    AS7343_CH_GREEN_F5_550NM,
    AS7343_CH_VIS_3,
    AS7343_CH_FD_3,
} as7343_channel_t;

typedef enum {
    AS7343_AGAIN_0_5 = 0x00,
    AS7343_AGAIN_1,
    AS7343_AGAIN_2,
    AS7343_AGAIN_4,
    AS7343_AGAIN_8,
    AS7343_AGAIN_16,
    AS7343_AGAIN_32,
    AS7343_AGAIN_64,
    AS7343_AGAIN_128,
    AS7343_AGAIN_256,
    AS7343_AGAIN_512,
    AS7343_AGAIN_1024,
    AS7343_AGAIN_2048,
} as7343_again_t;

typedef enum {
    AS7343_FD_GAIN_0_5 = 0x00,
    AS7343_FD_GAIN_1,
    AS7343_FD_GAIN_2,
    AS7343_FD_GAIN_4,
    AS7343_FD_GAIN_8,
    AS7343_FD_GAIN_16,
    AS7343_FD_GAIN_32,
    AS7343_FD_GAIN_64,
    AS7343_FD_GAIN_128,
    AS7343_FD_GAIN_256,
    AS7343_FD_GAIN_512,
    AS7343_FD_GAIN_1024,
    AS7343_FD_GAIN_2048,
} as7343_fd_gain_t;

typedef enum {
    AS7343_FIFO_THRESHOLD_LVL_1 = 0x00,
    AS7343_FIFO_THRESHOLD_LVL_4,
    AS7343_FIFO_THRESHOLD_LVL_8,
    AS7343_FIFO_THRESHOLD_LVL_16,
} as7343_fifo_threshold_t;

typedef enum {
    AS7343_SP_TH_CH_0 = 0x00,
    AS7343_SP_TH_CH_1,
    AS7343_SP_TH_CH_2,
    AS7343_SP_TH_CH_3,
    AS7343_SP_TH_CH_4,
    AS7343_SP_TH_CH_5,
} as7343_spectral_threshold_channel_t;

typedef enum {
    AS7343_AUTOSMUX_6_CHANNELS  = 0x00,
    AS7343_AUTOSMUX_12_CHANNELS = 0x02,
    AS7343_AUTOSMUX_18_CHANNELS = 0x03,
} as7343_auto_smux_channel_t;

typedef enum {
    AS7343_GPIO_MODE_OUTPUT = 0x00,
    AS7343_GPIO_MODE_INPUT  = 0x01,
} as7343_gpio_mode_t;

typedef enum {
    AS7343_GPIO_OUTPUT_LOW  = 0x00,
    AS7343_GPIO_OUTPUT_HIGH = 0x01,
} as7343_gpio_output_t;

/* ------------------------------------------------------------------------ */
/* Register addresses + bitfield layouts                                   */
/* ------------------------------------------------------------------------ */

#define AS7343_REG_AUXID  0x58
typedef union {
    struct { uint8_t auxid : 4; uint8_t reserved : 4; };
    uint8_t byte;
} as7343_reg_auxid_t;

#define AS7343_REG_REVID  0x59
typedef union {
    struct { uint8_t revid : 3; uint8_t reserved : 5; };
    uint8_t byte;
} as7343_reg_revid_t;

#define AS7343_REG_ID     0x5A  /* plain uint8_t register */

#define AS7343_REG_CFG12  0x66
typedef union {
    struct { uint8_t reserved : 5; uint8_t sp_th_ch : 3; };
    uint8_t byte;
} as7343_reg_cfg12_t;

#define AS7343_REG_ENABLE 0x80
typedef union {
    struct {
        uint8_t pon : 1;
        uint8_t sp_en : 1;
        uint8_t reserved : 1;
        uint8_t wen : 1;
        uint8_t smuxen : 1;
        uint8_t reserved_one : 1;
        uint8_t fden : 1;
        uint8_t reserved_two : 1;
    };
    uint8_t byte;
} as7343_reg_enable_t;

#define AS7343_REG_ATIME  0x81  /* plain uint8_t register */
#define AS7343_REG_WTIME  0x83  /* plain uint8_t register */

#define AS7343_REG_SP_TH_L 0x84
typedef union {
    struct { uint8_t lsb; uint8_t msb; };
    uint16_t word;
} as7343_reg_sp_th_l_t;

#define AS7343_REG_SP_TH_H 0x86
typedef union {
    struct { uint8_t lsb; uint8_t msb; };
    uint16_t word;
} as7343_reg_sp_th_h_t;

#define AS7343_REG_STATUS 0x93
typedef union {
    struct {
        uint8_t sint : 1;
        uint8_t reserved : 1;
        uint8_t fint : 1;
        uint8_t aint : 1;
        uint8_t reserved_one : 3;
        uint8_t asat : 1;
    };
    uint8_t byte;
} as7343_reg_status_t;

#define AS7343_REG_ASTATUS 0x94
typedef union {
    struct { uint8_t again_status : 4; uint8_t reserved : 3; uint8_t asat_status : 1; };
    uint8_t byte;
} as7343_reg_astatus_t;

/* Data0..Data17: 18 x 16-bit channel registers, 2 bytes apart */
#define AS7343_REG_DATA0  0x95

typedef union {
    struct { uint8_t data_l; uint8_t data_h; };
    uint16_t word;
} as7343_reg_data_t;

#define AS7343_REG_STATUS2 0x90
typedef union {
    struct {
        uint8_t fdsat_dig : 1;
        uint8_t fdsat_ana : 1;
        uint8_t reserved : 1;
        uint8_t asat_ana : 1;
        uint8_t asat_dig : 1;
        uint8_t reserved_one : 1;
        uint8_t avalid : 1;
        uint8_t reserved_two : 1;
    };
    uint8_t byte;
} as7343_reg_status2_t;

#define AS7343_REG_STATUS3 0x91
typedef union {
    struct {
        uint8_t reserved : 4;
        uint8_t int_sp_l : 1;
        uint8_t int_sp_h : 1;
        uint8_t reserved_one : 2;
    };
    uint8_t byte;
} as7343_reg_status3_t;

#define AS7343_REG_STATUS4 0xBC
typedef union {
    struct {
        uint8_t int_busy : 1;
        uint8_t sai_act : 1;
        uint8_t sp_trig : 1;
        uint8_t reserved : 1;
        uint8_t fd_trig : 1;
        uint8_t ov_temp : 1;
        uint8_t reserved_one : 1;
        uint8_t fifo_ov : 1;
    };
    uint8_t byte;
} as7343_reg_status4_t;

#define AS7343_REG_FD_STATUS 0xE3
typedef union {
    struct {
        uint8_t fd_100hz_det : 1;
        uint8_t fd_120hz_det : 1;
        uint8_t fd_100hz_valid : 1;
        uint8_t fd_120hz_valid : 1;
        uint8_t fd_saturation : 1;
        uint8_t fd_meas_valid : 1;
        uint8_t reserved : 2;
    };
    uint8_t byte;
} as7343_reg_fd_status_t;

/* NOTE: shares address 0x93 with AS7343_REG_STATUS — datasheet quirk,
 * inherited from AMS/SparkFun original, not introduced here. */
#define AS7343_REG_STATUS5 0x93
typedef union {
    struct {
        uint8_t reserved : 2;
        uint8_t sint_smux : 1;
        uint8_t sint_fd : 1;
        uint8_t reserved_one : 4;
    };
    uint8_t byte;
} as7343_reg_status5_t;

#define AS7343_REG_CFG0 0xBF
typedef union {
    struct {
        uint8_t reserved : 2;
        uint8_t wlong : 1;
        uint8_t reserved_one : 1;
        uint8_t reg_bank : 1;
        uint8_t low_power : 1;
        uint8_t reserved_two : 2;
    };
    uint8_t byte;
} as7343_reg_cfg0_t;

#define AS7343_REG_CFG1 0xC6
typedef union {
    struct { uint8_t again : 5; uint8_t reserved : 3; };
    uint8_t byte;
} as7343_reg_cfg1_t;

#define AS7343_REG_CFG3 0xC7
typedef union {
    struct { uint8_t reserved : 4; uint8_t sai : 1; uint8_t reserved2 : 3; };
    uint8_t byte;
} as7343_reg_cfg3_t;

#define AS7343_REG_CFG6 0xF5
typedef union {
    struct { uint8_t reserved : 3; uint8_t smux_cmd : 2; uint8_t reserved_one : 3; };
    uint8_t byte;
} as7343_reg_cfg6_t;

#define AS7343_REG_CFG8 0xC9
typedef union {
    struct { uint8_t reserved : 6; uint8_t fifo_th : 2; };
    uint8_t byte;
} as7343_reg_cfg8_t;

#define AS7343_REG_CFG9 0xCA
typedef union {
    struct {
        uint8_t reserved : 5;
        uint8_t sienc_smux : 1;
        uint8_t reserved_one : 1;
        uint8_t sienc_fd : 1;
        uint8_t reserved_two : 1;
    };
    uint8_t byte;
} as7343_reg_cfg9_t;

#define AS7343_REG_CFG10 0x65
typedef union {
    struct { uint8_t fd_pers : 3; uint8_t reserved : 5; };
    uint8_t byte;
} as7343_reg_cfg10_t;

#define AS7343_REG_PERS 0xCF
typedef union {
    struct { uint8_t apers : 4; uint8_t reserved : 4; };
    uint8_t byte;
} as7343_reg_pers_t;

#define AS7343_REG_GPIO 0x6B
typedef union {
    struct {
        uint8_t gpio_in : 1;
        uint8_t gpio_out : 1;
        uint8_t gpio_in_en : 1;
        uint8_t gpio_inv : 1;
        uint8_t reserved : 4;
    };
    uint8_t byte;
} as7343_reg_gpio_t;

#define AS7343_REG_ASTEP 0xD4
typedef union {
    struct { uint8_t astep_l; uint8_t astep_h; };
    uint16_t word;
} as7343_reg_astep_t;

#define AS7343_REG_CFG20 0xD6
typedef union {
    struct { uint8_t reserved : 5; uint8_t auto_smux : 2; uint8_t fd_fifo_8b : 1; };
    uint8_t byte;
} as7343_reg_cfg20_t;

#define AS7343_REG_LED 0xCD
typedef union {
    struct { uint8_t led_drive : 7; uint8_t led_act : 1; };
    uint8_t byte;
} as7343_reg_led_t;

#define AS7343_REG_AGC_GAIN_MAX 0xD7
typedef union {
    struct { uint8_t reserved : 4; uint8_t agc_fd_gain_max : 4; };
    uint8_t byte;
} as7343_reg_agc_gain_max_t;

#define AS7343_REG_AZ_CONFIG 0xDE   /* plain uint8_t register */
#define AS7343_REG_FD_TIME1  0xE0   /* plain uint8_t register */

#define AS7343_REG_FD_TIME2 0xE2
typedef union {
    struct { uint8_t fd_time_h : 3; uint8_t fd_gain : 5; }; /* TODO: confirm bit order vs datasheet */
    uint8_t byte;
} as7343_reg_fd_time2_t;

#define AS7343_REG_FD_TIME_CFG0 0xDF
typedef union {
    struct { uint8_t reserved : 7; uint8_t fifo_write_fd : 1; };
    uint8_t byte;
} as7343_reg_fd_cfg0_t;

#define AS7343_REG_INTENAB 0xF9
typedef union {
    struct {
        uint8_t sein : 1;
        uint8_t reserved : 1;
        uint8_t fien : 1;
        uint8_t sp_ien : 1;
        uint8_t reserved_one : 3;
        uint8_t asien : 1;
    };
    uint8_t byte;
} as7343_reg_intenab_t;

#define AS7343_REG_CONTROL 0xFA
typedef union {
    struct {
        uint8_t clear_sai_act : 1;
        uint8_t fifo_clr : 1;
        uint8_t sp_man_az : 1;
        uint8_t sw_reset : 1;
        uint8_t reserved : 4;
    };
    uint8_t byte;
} as7343_reg_control_t;

#define AS7343_REG_FIFO_MAP 0xFC
typedef union {
    struct {
        uint8_t fifo_write_astatus : 1;
        uint8_t fifo_write_ch0_data : 1;
        uint8_t fifo_write_ch1_data : 1;
        uint8_t fifo_write_ch2_data : 1;
        uint8_t fifo_write_ch3_data : 1;
        uint8_t fifo_write_ch4_data : 1;
        uint8_t fifo_write_ch5_data : 1;
        uint8_t reserved : 1;
    };
    uint8_t byte;
} as7343_reg_fifo_map_t;

#define AS7343_REG_FIFO_LVL 0xFD  /* plain uint8_t register */

#define AS7343_REG_FDATA 0xFE
typedef union {
    struct { uint8_t fdata_l; uint8_t fdata_h; };
    uint16_t word;
} as7343_reg_fdata_t;

/* Direct Chain Configuration (AN001033) — reaches SMUX chains 2 & 3,
 * unlike the RAM/SMUXEN method which only reaches chain 1. */
#define AS7343_REG_CHAINCMD   0xE4 /* target_chain[6:4], chain_length[3:0] */
#define AS7343_REG_CHAIN_SMUX 0xE7 /* nibble-pair pixel mapping shift register */

/* ------------------------------------------------------------------------ */
/* Device data (no bus handle here — that lives in the platform bus layer) */
/* ------------------------------------------------------------------------ */

struct as7343_dev {
    uint16_t data[AS7343_NUM_CHANNELS]; /* last-read channel values */
};

void as7343_dev_init(struct as7343_dev *dev);

/* Raw register I/O — implemented in as7343_reg.c, over i2c_dt_spec */
struct i2c_dt_spec;
int as7343_reg_read(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *data);
int as7343_reg_write(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t data);
int as7343_reg_read_burst(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *data, size_t len);
int as7343_reg_write_burst(const struct i2c_dt_spec *bus, uint8_t reg, const uint8_t *data, size_t len);
int as7343_reg_read16(const struct i2c_dt_spec *bus, uint8_t reg, uint16_t *val);
int as7343_reg_write16(const struct i2c_dt_spec *bus, uint8_t reg, uint16_t val);
int as7343_reg_update(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t mask, uint8_t val);

#endif /* ZEPHYR_DRIVERS_SENSOR_AS7343_H_ */