/**
 * @file as7263.c
 * @brief Zephyr sensor_driver_api implementation for the AS7263.
 *
 * One-shot mode (BANK=3): writing BANK bits triggers exactly two internal
 * conversion cycles, then DATA_RDY sets. No SMUX, single bank, and every
 * register access is a polled transaction — sample_fetch is inherently
 * slower than AS7343's burst read (12 individual byte reads for raw data).
 *
 * Part of the LEAF multispectral sensing system.
 * Proprietary and confidential. Not for public distribution.
 */

#define DT_DRV_COMPAT ams_as7263

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "as7263.h"

LOG_MODULE_REGISTER(as7263, CONFIG_SENSOR_LOG_LEVEL);

enum as7263_attr {
    SENSOR_ATTR_AS7263_GAIN = SENSOR_ATTR_PRIV_START,
    SENSOR_ATTR_AS7263_INT_TIME,
#ifdef CONFIG_AS7263_LED
    SENSOR_ATTR_AS7263_LED_ON,
    SENSOR_ATTR_AS7263_LED_DRIVE,
#endif
};

struct as7263_config {
    struct i2c_dt_spec bus;
};

struct as7263_data {
    struct as7263_dev dev;
};

static const uint8_t as7263_raw_regs[AS7263_NUM_CHANNELS] = {
    AS7263_VREG_R_HIGH, AS7263_VREG_S_HIGH, AS7263_VREG_T_HIGH,
    AS7263_VREG_U_HIGH, AS7263_VREG_V_HIGH, AS7263_VREG_W_HIGH,
};

static int as7263_wait_data_rdy(const struct i2c_dt_spec *bus, k_timeout_t timeout)
{
    int64_t deadline = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);
    uint8_t ctrl;
    int ret;

    do {
        ret = as7263_vreg_read(bus, AS7263_VREG_CONTROL, &ctrl);
        if (ret) {
            return ret;
        }
        if (ctrl & BIT(1)) { /* DATA_RDY */
            return 0;
        }
        k_msleep(5);
    } while (k_uptime_get() < deadline);

    return -ETIMEDOUT;
}

static int as7263_sample_fetch(const struct device *devp, enum sensor_channel chan)
{
    const struct as7263_config *cfg = devp->config;
    struct as7263_data *data = devp->data;
    int ret;

    ARG_UNUSED(chan);

    /* Set BANK=3 (one-shot), preserving GAIN/other bits */
    ret = as7263_vreg_update(&cfg->bus, AS7263_VREG_CONTROL, 0x0C,
                  (uint8_t)(AS7263_BANK_MODE_3_ONESHOT << 2));
    if (ret) {
        return ret;
    }

    ret = as7263_wait_data_rdy(&cfg->bus, K_MSEC(2000));
    if (ret) {
        LOG_WRN("AS7263 sample timeout");
        return ret;
    }

    for (int i = 0; i < AS7263_NUM_CHANNELS; i++) {
        ret = as7263_vreg_read16(&cfg->bus, as7263_raw_regs[i], &data->dev.data[i]);
        if (ret) {
            return ret;
        }
    }

    return 0;
}

static int as7263_channel_get(const struct device *devp, enum sensor_channel chan,
                   struct sensor_value *val)
{
    struct as7263_data *data = devp->data;
    int idx;

    if (chan < SENSOR_CHAN_PRIV_START) {
        return -ENOTSUP;
    }
    idx = chan - SENSOR_CHAN_PRIV_START;
    if (idx < 0 || idx >= AS7263_NUM_CHANNELS) {
        return -ENOTSUP;
    }

    val->val1 = data->dev.data[idx];
    val->val2 = 0;
    return 0;
}

static int as7263_attr_set(const struct device *devp, enum sensor_channel chan,
                enum sensor_attribute attr, const struct sensor_value *val)
{
    const struct as7263_config *cfg = devp->config;

    ARG_UNUSED(chan);

    switch ((int)attr) {
    case SENSOR_ATTR_AS7263_GAIN:
        return as7263_vreg_update(&cfg->bus, AS7263_VREG_CONTROL, 0x30,
                       (uint8_t)(val->val1 << 4));
    case SENSOR_ATTR_AS7263_INT_TIME:
        return as7263_vreg_write(&cfg->bus, AS7263_VREG_INT_T, (uint8_t)val->val1);
#ifdef CONFIG_AS7263_LED
    case SENSOR_ATTR_AS7263_LED_ON:
        return as7263_vreg_update(&cfg->bus, AS7263_VREG_LED_CONTROL, BIT(3),
                       val->val1 ? BIT(3) : 0);
    case SENSOR_ATTR_AS7263_LED_DRIVE:
        return as7263_vreg_update(&cfg->bus, AS7263_VREG_LED_CONTROL, 0x30,
                       (uint8_t)(val->val1 << 4));
#endif
    default:
        return -ENOTSUP;
    }
}

static int as7263_attr_get(const struct device *devp, enum sensor_channel chan,
                enum sensor_attribute attr, struct sensor_value *val)
{
    const struct as7263_config *cfg = devp->config;
    uint8_t raw;
    int ret;

    ARG_UNUSED(chan);

    switch ((int)attr) {
    case SENSOR_ATTR_AS7263_GAIN:
        ret = as7263_vreg_read(&cfg->bus, AS7263_VREG_CONTROL, &raw);
        val->val1 = (raw >> 4) & 0x03;
        return ret;
    case SENSOR_ATTR_AS7263_INT_TIME:
        ret = as7263_vreg_read(&cfg->bus, AS7263_VREG_INT_T, &raw);
        val->val1 = raw;
        return ret;
#ifdef CONFIG_AS7263_LED
    case SENSOR_ATTR_AS7263_LED_DRIVE:
        ret = as7263_vreg_read(&cfg->bus, AS7263_VREG_LED_CONTROL, &raw);
        val->val1 = (raw >> 4) & 0x03;
        return ret;
#endif
    default:
        return -ENOTSUP;
    }
}

void as7263_dev_init(struct as7263_dev *dev)
{
    memset(dev, 0, sizeof(*dev));
}

static int as7263_init(const struct device *devp)
{
    const struct as7263_config *cfg = devp->config;
    struct as7263_data *data = devp->data;
    uint8_t hw_type;
    int ret;

    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("AS7263 I2C bus not ready");
        return -ENODEV;
    }

    as7263_dev_init(&data->dev);

    /* Soft reset (RST bit, control reg bit 7) before first read, matching
     * SparkFun's begin(). The chip doesn't power-cycle on MCU reflash, so
     * a half-completed transaction from a previous build can otherwise
     * leave the virtual-register interface permanently wedged. */
    ret = as7263_vreg_update(&cfg->bus, AS7263_VREG_CONTROL, BIT(7), BIT(7));
    if (ret) {
        LOG_ERR("AS7263 soft reset failed: %d", ret);
        return ret;
    }
    k_msleep(1000);

    ret = as7263_vreg_read(&cfg->bus, AS7263_VREG_HW_TYPE, &hw_type);
    if (ret) {
        LOG_ERR("AS7263 not responding: %d", ret);
        return ret;
    }
    /* Soft check only — exact value varies across the AS726x family;
     * confirm on bench bring-up and tighten if desired. */
    LOG_INF("AS7263 HW type byte: 0x%02x", hw_type);

    return 0;
}

static DEVICE_API(sensor, as7263_driver_api) = {
    .sample_fetch = as7263_sample_fetch,
    .channel_get = as7263_channel_get,
    .attr_set = as7263_attr_set,
    .attr_get = as7263_attr_get,
};

#define AS7263_INIT(inst)                                                     \
    static struct as7263_data as7263_data_##inst;                            \
    static const struct as7263_config as7263_config_##inst = {               \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                   \
    };                                                                       \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, as7263_init, NULL,                    \
                      &as7263_data_##inst, &as7263_config_##inst,\
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
                      &as7263_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS7263_INIT)