/**
 * @file as7343.c
 * @brief Zephyr sensor_driver_api implementation for the AS7343.
 *
 * Uses Direct Chain Configuration (datasheet sec 3.4 / AN001033) to read
 * all 18 channels across 3 SMUX chains, giving full remap control (not
 * limited to the RAM/SMUXEN default like SparkFun's library).
 *
 * Part of the LEAF multispectral sensing system.
 * Proprietary and confidential. Not for public distribution.
 */

#define DT_DRV_COMPAT ams_as7343

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "as7343.h"

LOG_MODULE_REGISTER(as7343, CONFIG_SENSOR_LOG_LEVEL);

/* Custom attributes beyond standard sensor_attribute enum */
enum as7343_attr {
    SENSOR_ATTR_AS7343_AGAIN = SENSOR_ATTR_PRIV_START,
    SENSOR_ATTR_AS7343_ATIME,
    SENSOR_ATTR_AS7343_ASTEP,
    SENSOR_ATTR_AS7343_WTIME,
    SENSOR_ATTR_AS7343_LED_ON,
    SENSOR_ATTR_AS7343_LED_DRIVE,
    SENSOR_ATTR_AS7343_GPIO_MODE,
    SENSOR_ATTR_AS7343_GPIO_OUT,
    SENSOR_ATTR_AS7343_FD_GAIN,
    SENSOR_ATTR_AS7343_SMUX_CHAIN, /* val1 = chain (1-3), ptr via val2 unused: see set_chain() */
    SENSOR_ATTR_AS7343_AUTOGAIN_MAX,     /* set search ceiling (0-12) */
    SENSOR_ATTR_AS7343_AUTOGAIN_TRIGGER, /* write-only: run AN001051 search */
};

struct as7343_config {
    struct i2c_dt_spec bus;
};

struct as7343_data {
    struct as7343_dev dev;
    uint8_t autogain_max; /* search ceiling, default AS7343_AGAIN_2048 */
};

/* AN001051 fig 1 thresholds: optimize between 50%-90% of max counts */
#define AS7343_AUTOGAIN_HI_NUM 9
#define AS7343_AUTOGAIN_LO_NUM 5

/* Default 18-channel Direct Chain map, per AN001033 sec 3.4 (nibble pairs,
 * 20 entries per chain: NA,D,EXT,NIR,FLICKER,RB,F3,Y,F8,F6,RT,LT,F5,F4,F1,Z,F7,F2,XL,LB). */
static const uint8_t chain1_default[20] = {
    0x00,0x00, 0x00,0x04, 0x06,0x05, 0x00,0x02, 0x00,0x00,
    0x00,0x05, 0x00,0x00, 0x00,0x01, 0x00,0x00, 0x03,0x00,
};
static const uint8_t chain2_default[20] = {
    0x00,0x00, 0x00,0x00, 0x06,0x05, 0x02,0x00, 0x00,0x04,
    0x00,0x05, 0x00,0x03, 0x00,0x00, 0x00,0x01, 0x00,0x00,
};
static const uint8_t chain3_default[20] = {
    0x00,0x00, 0x00,0x00, 0x06,0x05, 0x00,0x00, 0x03,0x00,
    0x00,0x05, 0x04,0x00, 0x01,0x00, 0x02,0x00, 0x00,0x00,
};

static int as7343_write_chain(const struct i2c_dt_spec *bus, uint8_t target_chain,
                   const uint8_t *nibbles /* 20 bytes, values 0-6 each */)
{
    /* target_chain: 4 = Chain1, 5 = Chain2, 6 = Chain3 (datasheet CHAINCMD bits [6:4]) */
    uint8_t cmd = (uint8_t)((target_chain << 4) | 0x06); /* chain_length=6 per AN001033 examples use 0x46/0x56/0x66 */
    int ret;

    for (int i = 0; i < 20; i += 2) {
        uint8_t byte = (uint8_t)((nibbles[i] << 4) | nibbles[i + 1]);

        ret = as7343_reg_write(bus, AS7343_REG_CHAIN_SMUX, byte);
        if (ret) {
            return ret;
        }
        ret = as7343_reg_write(bus, AS7343_REG_CHAINCMD, cmd);
        if (ret) {
            return ret;
        }
    }
    return 0;
}

static int as7343_configure_default_chains(const struct i2c_dt_spec *bus)
{
    int ret;

    ret = as7343_write_chain(bus, 0x4, chain1_default);
    if (ret) return ret;
    ret = as7343_write_chain(bus, 0x5, chain2_default);
    if (ret) return ret;
    return as7343_write_chain(bus, 0x6, chain3_default);
}

static int as7343_wait_avalid(const struct i2c_dt_spec *bus, k_timeout_t timeout)
{
    int64_t deadline = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);
    uint8_t status2;
    int ret;

    do {
        ret = as7343_reg_read(bus, AS7343_REG_STATUS2, &status2);
        if (ret) {
            return ret;
        }
        if (status2 & BIT(6)) { /* AVALID */
            return 0;
        }
        k_msleep(2);
    } while (k_uptime_get() < deadline);

    return -ETIMEDOUT;
}

/* Shared by sample_fetch and the autogain search below */
static int as7343_measure_raw(const struct i2c_dt_spec *bus, uint16_t *out)
{
    /* PON=1, SP_EN=1 */
    int ret = as7343_reg_write(bus, AS7343_REG_ENABLE, BIT(0) | BIT(1));

    if (ret) {
        return ret;
    }

    ret = as7343_wait_avalid(bus, K_MSEC(500));
    if (ret) {
        LOG_WRN("AS7343 sample timeout");
        return ret;
    }

    ret = as7343_reg_read_burst(bus, AS7343_REG_DATA0, (uint8_t *)out,
                     AS7343_NUM_CHANNELS * sizeof(uint16_t));
    if (ret) {
        return ret;
    }

    /* Disable spectral measurement between fetches to save power */
    return as7343_reg_write(bus, AS7343_REG_ENABLE, BIT(0));
}

static uint16_t as7343_max_channel(const uint16_t *data)
{
    uint16_t max = 0;

    for (int i = 0; i < AS7343_NUM_CHANNELS; i++) {
        if (data[i] > max) {
            max = data[i];
        }
    }
    return max;
}

/* NOTE: (ATIME+1)*(ASTEP+1) per AN001051 fig 1 — unverified against the
 * full AS7343 datasheet's integration-time section this session (only
 * register map + AN001051 were available). Confirm on bench bring-up. */
static int as7343_max_counts(const struct i2c_dt_spec *bus, uint32_t *out)
{
    uint8_t atime;
    uint16_t astep;
    int ret = as7343_reg_read(bus, AS7343_REG_ATIME, &atime);

    if (ret) {
        return ret;
    }
    ret = as7343_reg_read16(bus, AS7343_REG_ASTEP, &astep);
    if (ret) {
        return ret;
    }
    uint32_t counts = (uint32_t)(atime + 1) * (uint32_t)(astep + 1);

    *out = (counts > 65535) ? 65535 : counts;
    return 0;
}

/* AN001051 sec 2.1 (search between saturation/noise) + sec 2.2 (log2
 * refinement closer to the max limit). Write-only trigger — run once per
 * session or when lighting changes, not on every sample_fetch. */
static int as7343_autogain_run(const struct as7343_config *cfg, struct as7343_data *data)
{
    const struct i2c_dt_spec *bus = &cfg->bus;
    uint8_t max_gain = data->autogain_max;
    uint8_t current_gain = (uint8_t)((max_gain + 1) / 2);
    uint8_t saturation_gain = (uint8_t)(max_gain + 1); /* sentinel: unset */
    uint32_t max_counts;
    uint32_t raw;
    int ret;

    if (current_gain > max_gain) {
        current_gain = max_gain;
    }

    for (;;) {
        ret = as7343_reg_update(bus, AS7343_REG_CFG1, 0x1F, current_gain);
        if (ret) {
            return ret;
        }
        ret = as7343_measure_raw(bus, data->dev.data);
        if (ret) {
            return ret;
        }
        ret = as7343_max_counts(bus, &max_counts);
        if (ret) {
            return ret;
        }

        raw = as7343_max_channel(data->dev.data);
        uint32_t hi = max_counts * AS7343_AUTOGAIN_HI_NUM / 10;
        uint32_t lo = max_counts * AS7343_AUTOGAIN_LO_NUM / 10;

        if (raw > hi) { /* saturation: halve gain */
            if (current_gain == 0) {
                break;
            }
            if (current_gain < saturation_gain) {
                saturation_gain = current_gain;
            }
            current_gain >>= 1;
        } else if (raw < lo) { /* noise: raise gain toward max */
            if (current_gain == max_gain) {
                break;
            }
            uint8_t new_gain = (uint8_t)((max_gain + current_gain + 1) / 2);

            if (new_gain == current_gain) {
                new_gain++;
            }
            if (new_gain >= saturation_gain) {
                break;
            }
            current_gain = new_gain;
        } else {
            break; /* in range, done searching */
        }
    }

    ret = as7343_max_counts(bus, &max_counts);
    if (ret) {
        return ret;
    }
    raw = as7343_max_channel(data->dev.data);
    if (raw > max_counts * AS7343_AUTOGAIN_HI_NUM / 10) {
        LOG_ERR("AS7343 autogain: optimization not possible, still saturated");
        return -EIO;
    }

    /* sec 2.2: refine closer to the max limit via log2 scaling */
    if (raw > 0) {
        int diff_gain = 0;
        uint32_t hi = max_counts * AS7343_AUTOGAIN_HI_NUM / 10;

        while (((uint64_t)raw << (diff_gain + 1)) <= hi) {
            diff_gain++;
        }
        if ((int)current_gain + diff_gain > max_gain) {
            diff_gain = max_gain - current_gain;
        }
        if ((int)current_gain + diff_gain < 0) {
            diff_gain = -(int)current_gain;
        }

        current_gain = (uint8_t)((int)current_gain + diff_gain);
        ret = as7343_reg_update(bus, AS7343_REG_CFG1, 0x1F, current_gain);
        if (ret) {
            return ret;
        }
        ret = as7343_measure_raw(bus, data->dev.data); /* final measurement at settled gain */
        if (ret) {
            return ret;
        }
    }

    LOG_INF("AS7343 autogain settled: gain=%u", current_gain);
    return 0;
}

static int as7343_sample_fetch(const struct device *devp, enum sensor_channel chan)
{
    const struct as7343_config *cfg = devp->config;
    struct as7343_data *data = devp->data;

    ARG_UNUSED(chan);

    return as7343_measure_raw(&cfg->bus, data->dev.data);
}

static int as7343_channel_get(const struct device *devp, enum sensor_channel chan,
                   struct sensor_value *val)
{
    struct as7343_data *data = devp->data;
    int idx;

    if (chan < SENSOR_CHAN_PRIV_START) {
        return -ENOTSUP;
    }
    idx = chan - SENSOR_CHAN_PRIV_START;
    if (idx < 0 || idx >= AS7343_NUM_CHANNELS) {
        return -ENOTSUP;
    }

    val->val1 = data->dev.data[idx];
    val->val2 = 0;
    return 0;
}

static int as7343_attr_set(const struct device *devp, enum sensor_channel chan,
                enum sensor_attribute attr, const struct sensor_value *val)
{
    const struct as7343_config *cfg = devp->config;
    struct as7343_data *data = devp->data;

    ARG_UNUSED(chan);

    switch ((int)attr) {
    case SENSOR_ATTR_AS7343_AUTOGAIN_MAX:
        data->autogain_max = (uint8_t)CLAMP(val->val1, 0, 12);
        return 0;
    case SENSOR_ATTR_AS7343_AUTOGAIN_TRIGGER:
        return as7343_autogain_run(cfg, data);
    case SENSOR_ATTR_AS7343_AGAIN:
        return as7343_reg_update(&cfg->bus, AS7343_REG_CFG1, 0x1F, (uint8_t)val->val1);
    case SENSOR_ATTR_AS7343_ATIME:
        return as7343_reg_write(&cfg->bus, AS7343_REG_ATIME, (uint8_t)val->val1);
    case SENSOR_ATTR_AS7343_ASTEP:
        return as7343_reg_write16(&cfg->bus, AS7343_REG_ASTEP, (uint16_t)val->val1);
    case SENSOR_ATTR_AS7343_WTIME:
        return as7343_reg_write(&cfg->bus, AS7343_REG_WTIME, (uint8_t)val->val1);
#ifdef CONFIG_AS7343_LED
    case SENSOR_ATTR_AS7343_LED_ON:
        return as7343_reg_update(&cfg->bus, AS7343_REG_LED, 0x80, val->val1 ? 0x80 : 0);
    case SENSOR_ATTR_AS7343_LED_DRIVE:
        return as7343_reg_update(&cfg->bus, AS7343_REG_LED, 0x7F, (uint8_t)val->val1);
#endif
#ifdef CONFIG_AS7343_GPIO
    case SENSOR_ATTR_AS7343_GPIO_MODE:
        return as7343_reg_update(&cfg->bus, AS7343_REG_GPIO, BIT(2), val->val1 ? BIT(2) : 0);
    case SENSOR_ATTR_AS7343_GPIO_OUT:
        return as7343_reg_update(&cfg->bus, AS7343_REG_GPIO, BIT(1), val->val1 ? BIT(1) : 0);
#endif
#ifdef CONFIG_AS7343_FLICKER
    case SENSOR_ATTR_AS7343_FD_GAIN:
        return as7343_reg_update(&cfg->bus, AS7343_REG_FD_TIME2, 0xF8, (uint8_t)(val->val1 << 3));
#endif
    default:
        return -ENOTSUP;
    }
}

static int as7343_attr_get(const struct device *devp, enum sensor_channel chan,
                enum sensor_attribute attr, struct sensor_value *val)
{
    const struct as7343_config *cfg = devp->config;
    uint8_t raw;
    int ret;

    ARG_UNUSED(chan);

    switch ((int)attr) {
    case SENSOR_ATTR_AS7343_AGAIN:
        ret = as7343_reg_read(&cfg->bus, AS7343_REG_CFG1, &raw);
        val->val1 = raw & 0x1F;
        return ret;
    case SENSOR_ATTR_AS7343_ATIME:
        ret = as7343_reg_read(&cfg->bus, AS7343_REG_ATIME, &raw);
        val->val1 = raw;
        return ret;
    default:
        return -ENOTSUP;
    }
}

static int as7343_init(const struct device *devp)
{
    const struct as7343_config *cfg = devp->config;
    struct as7343_data *data = devp->data;
    uint8_t id;
    int ret;

    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("AS7343 I2C bus not ready");
        return -ENODEV;
    }

    as7343_dev_init(&data->dev);
    data->autogain_max = AS7343_AGAIN_2048; /* default search ceiling */

    /* PON=1 briefly to read ID (needs oscillator active) */
    ret = as7343_reg_write(&cfg->bus, AS7343_REG_ENABLE, BIT(0));
    if (ret) {
        return ret;
    }
    k_usleep(300); /* INT_BUSY window per datasheet sec 10.2.6 */

    ret = as7343_reg_read(&cfg->bus, AS7343_REG_ID, &id);
    if (ret) {
        return ret;
    }
    if (id != AS7343DeviceID) {
        LOG_ERR("AS7343 ID mismatch: got 0x%02x, expected 0x%02x", id, AS7343DeviceID);
        return -ENODEV;
    }

    ret = as7343_configure_default_chains(&cfg->bus);
    if (ret) {
        LOG_ERR("AS7343 SMUX chain config failed: %d", ret);
        return ret;
    }

    /* Power down until first sample_fetch */
    return as7343_reg_write(&cfg->bus, AS7343_REG_ENABLE, 0x00);
}

static DEVICE_API(sensor, as7343_driver_api) = {
    .sample_fetch = as7343_sample_fetch,
    .channel_get = as7343_channel_get,
    .attr_set = as7343_attr_set,
    .attr_get = as7343_attr_get,
};

#define AS7343_INIT(inst)                                                     \
    static struct as7343_data as7343_data_##inst;                             \
    static const struct as7343_config as7343_config_##inst = {                \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                    \
    };                                                                        \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, as7343_init, NULL,                     \
                      &as7343_data_##inst, &as7343_config_##inst, \
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
                      &as7343_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS7343_INIT)