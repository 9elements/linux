// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * max6639.c - Support for Maxim MAX6639
 *
 * 2-Channel Temperature Monitor with Dual PWM Fan-Speed Controller
 *
 * Copyright (C) 2010, 2011 Roland Stigge <stigge@antcom.de>
 *
 * based on the initial MAX6639 support from semptian.net
 * by He Changqing <hechangqing@semptian.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x2c, 0x2e, 0x2f, I2C_CLIENT_END };

/* The MAX6639 registers, valid channel numbers: 0, 1 */
#define MAX6639_REG_TEMP(ch)			(0x00 + (ch))
#define MAX6639_REG_STATUS			0x02
#define MAX6639_REG_OUTPUT_MASK			0x03
#define MAX6639_REG_GCONFIG			0x04
#define MAX6639_REG_TEMP_EXT(ch)		(0x05 + (ch))
#define MAX6639_REG_ALERT_LIMIT(ch)		(0x08 + (ch))
#define MAX6639_REG_OT_LIMIT(ch)		(0x0A + (ch))
#define MAX6639_REG_THERM_LIMIT(ch)		(0x0C + (ch))
#define MAX6639_REG_FAN_CONFIG1(ch)		(0x10 + (ch) * 4)
#define MAX6639_REG_FAN_CONFIG2a(ch)		(0x11 + (ch) * 4)
#define MAX6639_REG_FAN_CONFIG2b(ch)		(0x12 + (ch) * 4)
#define MAX6639_REG_FAN_CONFIG3(ch)		(0x13 + (ch) * 4)
#define MAX6639_REG_FAN_CNT(ch)			(0x20 + (ch))
#define MAX6639_REG_TARGET_CNT(ch)		(0x22 + (ch))
#define MAX6639_REG_FAN_PPR(ch)			(0x24 + (ch))
#define MAX6639_REG_TARGTDUTY(ch)		(0x26 + (ch))
#define MAX6639_REG_FAN_START_TEMP(ch)		(0x28 + (ch))
#define MAX6639_REG_DEVID			0x3D
#define MAX6639_REG_MANUID			0x3E
#define MAX6639_REG_DEVREV			0x3F

/* Register bits */
#define MAX6639_GCONFIG_STANDBY			0x80
#define MAX6639_GCONFIG_POR			0x40
#define MAX6639_GCONFIG_DISABLE_TIMEOUT		0x20
#define MAX6639_GCONFIG_CH2_LOCAL		0x10
#define MAX6639_GCONFIG_PWM_FREQ_HI		0x08

#define MAX6639_FAN_CONFIG1_PWM			0x80
#define MAX6639_REG_FAN_CONFIG2a_PWM_POL	0x02
#define MAX6639_FAN_CONFIG3_FREQ_MASK		0x03
#define MAX6639_FAN_CONFIG3_DEFAULT		0x83
#define MAX6639_REG_TARGTDUTY_SLOT		120

#define MAX6639_FAN_CONFIG3_THERM_FULL_SPEED	0x40

#define MAX6639_FAN_PPR_MASK(ppr)		((ppr - 1) << 6)

#define MAX6639_NDEV				2

static const int rpm_ranges[] = { 2000, 4000, 8000, 16000 };

/* Supported PWM frequency */
static const unsigned int freq_table[] = { 20, 33, 50, 100, 5000, 8333, 12500,
					   25000 };

#define FAN_FROM_REG(val, rpm_range)	((val) == 0 || (val) == 255 ? \
				0 : (rpm_ranges[rpm_range] * 30) / (val))
#define TEMP_LIMIT_TO_REG(val)	clamp_val((val) / 1000, 0, 255)

/*
 * Client data (each client gets its own)
 */
struct max6639_data {
	struct regmap *regmap;
	struct device *hwmon_dev;
	struct mutex update_lock;
	bool valid;		/* true if following fields are valid */
	unsigned long last_updated;	/* In jiffies */

	/* Register values sampled regularly */
	u16 temp[MAX6639_NDEV];		/* Temperature, in 1/8 C, 0..255 C */
	bool temp_fault[MAX6639_NDEV];	/* Detected temperature diode failure */
	bool fan_enable[MAX6639_NDEV];
	u8 fan[MAX6639_NDEV];		/* Register value: TACH count for fans >=30 */
	u32 target_rpm[MAX6639_NDEV];
	u32 max_rpm[MAX6639_NDEV];
	u8 status;			/* Detected channel alarms and fan failures */

	/* Register values only written to */
	u8 pwm[MAX6639_NDEV];		/* Register value: Duty cycle 0..120 */
	u8 temp_therm[MAX6639_NDEV];	/* THERM Temperature, 0..255 C (->_max) */
	u8 temp_alert[MAX6639_NDEV];	/* ALERT Temperature, 0..255 C (->_crit) */
	u8 temp_ot[MAX6639_NDEV];	/* OT Temperature, 0..255 C (->_emergency) */

	/* Register values initialized only once */
	u8 ppr[MAX6639_NDEV];	/* Pulses per rotation 0..3 for 1..4 ppr */
	u8 rpm_range[MAX6639_NDEV]; /* Index in above rpm_ranges table */

	/* Optional regulator for FAN supply */
	struct regulator *reg;
	/* max6639 pwm chip */
	struct pwm_chip chip;
	struct pwm_device *pwmd[MAX6639_NDEV]; /* max6639 has two pwm device */
};

static struct max6639_data *max6639_update_device(struct device *dev)
{
	struct max6639_data *data = dev_get_drvdata(dev);
	struct max6639_data *ret = data;
	int i, err;
	int status_reg;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + 2 * HZ) || !data->valid) {
		int res;

		dev_dbg(dev, "Starting max6639 update\n");

		err = regmap_read(data->regmap, MAX6639_REG_STATUS, &status_reg);
		if (err < 0) {
			ret = ERR_PTR(err);
			goto abort;
		}

		data->status = status_reg;

		for (i = 0; i < MAX6639_NDEV; i++) {
			err = regmap_read(data->regmap, MAX6639_REG_FAN_CNT(i), &res);
			if (err < 0) {
				ret = ERR_PTR(err);
				goto abort;
			}
			data->fan[i] = res;

			err = regmap_read(data->regmap, MAX6639_REG_TEMP_EXT(i), &res);
			if (err < 0) {
				ret = ERR_PTR(err);
				goto abort;
			}
			data->temp[i] = res >> 5;
			data->temp_fault[i] = res & 0x01;

			err = regmap_read(data->regmap, MAX6639_REG_TEMP(i), &res);
			if (err < 0) {
				ret = ERR_PTR(err);
				goto abort;
			}
			data->temp[i] |= res << 3;
		}

		data->last_updated = jiffies;
		data->valid = true;
	}
abort:
	mutex_unlock(&data->update_lock);

	return ret;
}

static int max6639_temp_set_max(struct max6639_data *data, int channel, unsigned long val)
{
	int res;

	mutex_lock(&data->update_lock);
	data->temp_therm[channel] = TEMP_LIMIT_TO_REG(val);
	res = regmap_write(data->regmap, MAX6639_REG_THERM_LIMIT(channel),
			   data->temp_therm[channel]);
	mutex_unlock(&data->update_lock);
	return res;
}

static int max6639_temp_set_crit(struct max6639_data *data, int channel, unsigned long val)
{
	int res;

	mutex_lock(&data->update_lock);
	data->temp_alert[channel] = TEMP_LIMIT_TO_REG(val);
	res = regmap_write(data->regmap, MAX6639_REG_ALERT_LIMIT(channel),
			   data->temp_alert[channel]);
	mutex_unlock(&data->update_lock);
	return res;
}

static int max6639_temp_set_emergency(struct max6639_data *data, int channel, unsigned long val)
{
	int res;

	mutex_lock(&data->update_lock);
	data->temp_ot[channel] = TEMP_LIMIT_TO_REG(val);
	res = regmap_write(data->regmap, MAX6639_REG_OT_LIMIT(channel),
			   data->temp_ot[channel]);
	mutex_unlock(&data->update_lock);
	return res;
}

static int set_ppr(struct max6639_data *data, u8 channel, u8 ppr)
{
	return regmap_write(data->regmap, MAX6639_REG_FAN_PPR(channel), MAX6639_FAN_PPR_MASK(ppr));
}

static int max6639_read_fan(struct device *dev, u32 attr, int channel,
			    long *val)
{
	struct max6639_data *data = max6639_update_device(dev);

	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (attr) {
	case hwmon_fan_input:
		*val = FAN_FROM_REG(data->fan[channel], data->rpm_range[channel]);
		return 0;
	case hwmon_fan_fault:
		*val = !!(data->status & (2 >> channel));
		return 0;
	case hwmon_fan_pulses:
		*val = data->ppr[channel];
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int max6639_write_fan(struct device *dev, u32 attr, int channel,
			    long val)
{
	struct max6639_data *data = max6639_update_device(dev);
	int err;
	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (attr) {
	case hwmon_fan_pulses:
		if (val <= 0 || val > 5) {
			dev_err(dev, "invalid pulses-per-revolution %ld. Valid range id 1 - 4.",
				val);
			return -EINVAL;
		}
		/* Set Fan pulse per revolution */
		err = set_ppr(data, channel, val);
		if (err)
			dev_err(dev, "Failed to set pulses-per-revolution");
		else
			data->ppr[channel] = val;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max6639_fan_is_visible(const void *_data, u32 attr, int channel)
{
	struct max6639_data *data = (struct max6639_data *) _data;

	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (attr) {
	case hwmon_fan_input:
	case hwmon_fan_fault:
		if (data->fan_enable[channel])
			return 0444;
		else
			return 0;
	case hwmon_fan_pulses:
		if (data->fan_enable[channel])
			return 0644;
		else
			return 0;
	default:
		return 0;
	}
}

static int max6639_read_pwm(struct device *dev, u32 attr, int channel,
			    long *val)
{
	struct max6639_data *data = max6639_update_device(dev);
	struct pwm_state state;

	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (attr) {
	case hwmon_pwm_input:
		pwm_get_state(data->pwmd[channel], &state);
		*val = pwm_get_relative_duty_cycle(&state, 255);
		return 0;
	case hwmon_pwm_freq:
		pwm_get_state(data->pwmd[channel], &state);
		*val = DIV_ROUND_UP_ULL(NSEC_PER_SEC, state.period);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int max6639_write_pwm(struct device *dev, u32 attr, int channel,
			    long val)
{
	struct max6639_data *data = max6639_update_device(dev);
	struct pwm_state state;
	int err, duty_cycle;

	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (attr) {
	case hwmon_pwm_input:
		val = clamp_val(val, 0, 255);
		pwm_get_state(data->pwmd[channel], &state);
		pwm_set_relative_duty_cycle(&state, val, 255);
		err = pwm_apply_state(data->pwmd[channel], &state);
		return err;

	case hwmon_pwm_freq:
		val = clamp_val(val, 0, 25000);
		pwm_get_state(data->pwmd[channel], &state);
		duty_cycle = pwm_get_relative_duty_cycle(&state, 255);
		state.period = DIV_ROUND_UP_ULL(NSEC_PER_SEC, val);
		pwm_set_relative_duty_cycle(&state, duty_cycle, 255);
		err = pwm_apply_state(data->pwmd[channel], &state);
		return err;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max6639_pwm_is_visible(const void *_data, u32 attr, int channel)
{
	struct max6639_data *data = (struct max6639_data *) _data;

	if (IS_ERR(data))
		return PTR_ERR(data);

	if (IS_ERR(data->pwmd[channel]))
		return 0;

	switch (attr) {
	case hwmon_pwm_input:
	case hwmon_pwm_freq:
		if (data->fan_enable[channel])
			return 0644;
		else
			return 0;
	default:
		return 0;
	}
}

static int max6639_read_temp(struct device *dev, u32 attr, int channel,
			    long *val)
{
	struct max6639_data *data = max6639_update_device(dev);

	switch (attr) {
	case hwmon_temp_input:
		*val = data->temp[channel] * 125;
		return 0;
	case hwmon_temp_fault:
		*val = data->temp_fault[channel] * 125;
		return 0;
	case hwmon_temp_max:
		*val = data->temp_therm[channel] * 1000;
		return 0;
	case hwmon_temp_crit:
		*val = data->temp_alert[channel] * 1000;
		return 0;
	case hwmon_temp_emergency:
		*val = data->temp_ot[channel] * 1000;
		return 0;
	case hwmon_temp_max_alarm:
		*val = !!(data->status & (0x08 >> channel));
		return 0;
	case hwmon_temp_crit_alarm:
		*val = !!(data->status & (0x80 >> channel));
		return 0;
	case hwmon_temp_emergency_alarm:
		*val = !!(data->status & (0x20 >> channel));
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int max6639_write_temp(struct device *dev, u32 attr, int channel,
			    long val)
{
	struct max6639_data *data = max6639_update_device(dev);

	switch (attr) {
	case hwmon_temp_max:
		return max6639_temp_set_max(data, channel, val);
	case hwmon_temp_crit:
		return max6639_temp_set_crit(data, channel, val);
	case hwmon_temp_emergency:
		return max6639_temp_set_emergency(data, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t MAX6639_NDEV_is_visible(const void *_data, u32 attr, int channel)
{

	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_fault:
	case hwmon_temp_max_alarm:
	case hwmon_temp_crit_alarm:
	case hwmon_temp_emergency_alarm:
		return 0444;
	case hwmon_temp_max:
	case hwmon_temp_crit:
	case hwmon_temp_emergency:
		return 0644;
	default:
		return 0;
	}
}

static int max6639_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return max6639_read_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return max6639_read_pwm(dev, attr, channel, val);
	case hwmon_temp:
		return max6639_read_temp(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max6639_write(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_fan:
		return max6639_write_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return max6639_write_pwm(dev, attr, channel, val);
	case hwmon_temp:
		return max6639_write_temp(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max6639_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return max6639_fan_is_visible(data, attr, channel);
	case hwmon_pwm:
		return max6639_pwm_is_visible(data, attr, channel);
	case hwmon_temp:
		return MAX6639_NDEV_is_visible(data, attr, channel);
	default:
		return 0;
	}
}

static const struct hwmon_channel_info * const max6639_info[] = {
	HWMON_CHANNEL_INFO(fan,
			HWMON_F_INPUT | HWMON_F_FAULT | HWMON_F_PULSES,
			HWMON_F_INPUT | HWMON_F_FAULT | HWMON_F_PULSES),
	HWMON_CHANNEL_INFO(pwm,
			HWMON_PWM_INPUT | HWMON_PWM_FREQ,
			HWMON_PWM_INPUT | HWMON_PWM_FREQ),
	HWMON_CHANNEL_INFO(temp,
			HWMON_T_INPUT | HWMON_T_FAULT | HWMON_T_MAX | HWMON_T_MAX_ALARM |
			HWMON_T_CRIT | HWMON_T_CRIT_ALARM | HWMON_T_EMERGENCY |
			HWMON_T_EMERGENCY_ALARM,
			HWMON_T_INPUT | HWMON_T_FAULT | HWMON_T_MAX | HWMON_T_MAX_ALARM |
			HWMON_T_CRIT | HWMON_T_CRIT_ALARM | HWMON_T_EMERGENCY |
			HWMON_T_EMERGENCY_ALARM),
	NULL
};

static const struct hwmon_ops max6639_hwmon_ops = {
	.is_visible = max6639_is_visible,
	.read = max6639_read,
	.write = max6639_write,
};

static const struct hwmon_chip_info max6639_chip_info = {
	.ops = &max6639_hwmon_ops,
	.info = max6639_info,
};

static struct max6639_data *to_max6639_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct max6639_data, chip);
}

static int max6639_pwm_get_state(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_state *state)
{

	struct max6639_data *data = to_max6639_pwm(chip);
	int value, i = pwm->hwpwm, x, err;
	unsigned int freq;

	mutex_lock(&data->update_lock);

	err = regmap_read(data->regmap, MAX6639_REG_FAN_CONFIG1(i), &value);
	if (err < 0)
		goto abort;

	if (value & MAX6639_FAN_CONFIG1_PWM) {
		state->enabled = true;

		/* Determine frequency from respective registers */
		err = regmap_read(data->regmap, MAX6639_REG_FAN_CONFIG3(i), &value);
		if (err < 0)
			goto abort;
		x = value & MAX6639_FAN_CONFIG3_FREQ_MASK;

		err = regmap_read(data->regmap, MAX6639_REG_GCONFIG, &value);
		if (err < 0)
			goto abort;
		if (value & MAX6639_GCONFIG_PWM_FREQ_HI)
			x |= 0x4;
		x &= 0x7;
		freq = freq_table[x];

		state->period = DIV_ROUND_UP(NSEC_PER_SEC, freq);

		err = regmap_read(data->regmap, MAX6639_REG_TARGTDUTY(i), &value);
		if (err < 0)
			goto abort;
		/* max6639 supports 120 slots only */
		state->duty_cycle = mul_u64_u32_div(state->period, value, 120);

		err = regmap_read(data->regmap, MAX6639_REG_FAN_CONFIG2a(i), &value);
		if (err < 0)
			goto abort;
		value &= MAX6639_REG_FAN_CONFIG2a_PWM_POL;
		state->polarity = (value != 0);
	} else
		state->enabled = false;

abort:
	mutex_unlock(&data->update_lock);
	return value;
}

static int max6639_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct max6639_data *data = to_max6639_pwm(chip);
	int value, i = pwm->hwpwm, x, err;
	unsigned int freq;
	struct pwm_state cstate;

	cstate = pwm->state;

	mutex_lock(&data->update_lock);

	if (state->period != cstate.period) {
		/* Configure frequency */
		freq = DIV_ROUND_UP_ULL(NSEC_PER_SEC, state->period);

		/* Chip supports limited number of frequency */
		for (x = 0; x < sizeof(freq_table); x++)
			if (freq <= freq_table[x])
				break;

		err = regmap_read(data->regmap, MAX6639_REG_FAN_CONFIG3(i), &value);
		if (err < 0)
			goto abort;

		value &= ~MAX6639_FAN_CONFIG3_FREQ_MASK;
		value |= (x & MAX6639_FAN_CONFIG3_FREQ_MASK);
		err = regmap_write(data->regmap, MAX6639_REG_FAN_CONFIG3(i), value);
		if (err < 0)
			goto abort;

		err = regmap_read(data->regmap, MAX6639_REG_GCONFIG, &value);
		if (err < 0)
			goto abort;

		if (x >> 2)
			value &= ~MAX6639_GCONFIG_PWM_FREQ_HI;
		else
			value |= MAX6639_GCONFIG_PWM_FREQ_HI;
		err = regmap_write(data->regmap, MAX6639_REG_GCONFIG, value);
		if (err < 0)
			goto abort;
	}

	/* Configure dutycycle */
	if (state->duty_cycle != cstate.duty_cycle ||
	    state->period != cstate.period) {
		value = DIV_ROUND_DOWN_ULL(state->duty_cycle * MAX6639_REG_TARGTDUTY_SLOT,
					   state->period);
		err = regmap_write(data->regmap, MAX6639_REG_TARGTDUTY(i), value);
		if (err < 0)
			goto abort;
	}

	/* Configure polarity */
	if (state->polarity != cstate.polarity) {
		err = regmap_read(data->regmap, MAX6639_REG_FAN_CONFIG2a(i), &value);
		if (err < 0)
			goto abort;
		if (state->polarity == PWM_POLARITY_NORMAL)
			value |= MAX6639_REG_FAN_CONFIG2a_PWM_POL;
		else
			value &= ~MAX6639_REG_FAN_CONFIG2a_PWM_POL;
		err = regmap_write(data->regmap, MAX6639_REG_FAN_CONFIG2a(i), value);
		if (err < 0)
			goto abort;
	}

	if (state->enabled != cstate.enabled) {
		err = regmap_read(data->regmap, MAX6639_REG_FAN_CONFIG1(i), &value);
		if (err < 0)
			goto abort;
		if (state->enabled)
			value |= MAX6639_FAN_CONFIG1_PWM;
		else
			value &= ~MAX6639_FAN_CONFIG1_PWM;

		err = regmap_write(data->regmap, MAX6639_REG_FAN_CONFIG1(i), value);
		if (err < 0)
			goto abort;
	}
	value = 0;

abort:
	mutex_unlock(&data->update_lock);

	return err;
}

static const struct pwm_ops max6639_pwm_ops = {
	.apply = max6639_pwm_apply,
	.get_state = max6639_pwm_get_state,
};

/*
 *  returns respective index in rpm_ranges table
 *  3 by default on invalid range
 */
static int rpm_range_to_reg(int range)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rpm_ranges); i++) {
		if (range <= rpm_ranges[i])
			return i;
	}

	return 3; /* default: 16000 RPM */
}

static int max6639_init_client(struct i2c_client *client,
			       struct max6639_data *data)
{
	int i, err;
	struct pwm_state state;

	/* Reset chip to default values, see below for GCONFIG setup */
	err = regmap_write(data->regmap, MAX6639_REG_GCONFIG, MAX6639_GCONFIG_POR);
	if (err)
		goto exit;

	for (i = 0; i < MAX6639_NDEV; i++) {
		/* Set Fan pulse per revolution */
		err = set_ppr(data, i, data->ppr[i]);
		if (err)
			goto exit;

		/* Fans config PWM, RPM */
		err = regmap_write(data->regmap, MAX6639_REG_FAN_CONFIG1(i),
				   MAX6639_FAN_CONFIG1_PWM | data->rpm_range[i]);
		if (err)
			goto exit;

		/*
		 * /THERM full speed enable,
		 * PWM frequency 25kHz, see also GCONFIG below
		 */
		err = regmap_write(data->regmap, MAX6639_REG_FAN_CONFIG3(i),
				   MAX6639_FAN_CONFIG3_DEFAULT);
		if (err)
			goto exit;

		/* Max. temp. 80C/90C/100C */
		data->temp_therm[i] = 80;
		data->temp_alert[i] = 90;
		data->temp_ot[i] = 100;

		err = regmap_write(data->regmap, MAX6639_REG_THERM_LIMIT(i), data->temp_therm[i]);
		if (err)
			goto exit;
		err = regmap_write(data->regmap, MAX6639_REG_ALERT_LIMIT(i), data->temp_alert[i]);
		if (err)
			goto exit;
		err = regmap_write(data->regmap, MAX6639_REG_OT_LIMIT(i), data->temp_ot[i]);
		if (err)
			goto exit;

		/* Configure PWM controller */
		if (data->pwmd[i]) {
			pwm_get_state(data->pwmd[i], &state);
			state.period = DIV_ROUND_UP(NSEC_PER_SEC, 25000);
			state.polarity = PWM_POLARITY_NORMAL;
			pwm_set_relative_duty_cycle(&state, data->target_rpm[i],
						data->max_rpm[i]);
			err = pwm_apply_state(data->pwmd[i], &state);
			if (err)
				goto exit;
		}
	}

	err = i2c_smbus_write_byte_data(client, MAX6639_REG_OUTPUT_MASK,
					!data->fan_enable[0] << 1 | !data->fan_enable[1]);

	/* Start monitoring */
	err = regmap_write(data->regmap, MAX6639_REG_GCONFIG,
			   MAX6639_GCONFIG_DISABLE_TIMEOUT | MAX6639_GCONFIG_CH2_LOCAL |
			   MAX6639_GCONFIG_PWM_FREQ_HI);

exit:
	return err;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int max6639_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int dev_id, manu_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* Actual detection via device and manufacturer ID */
	dev_id = i2c_smbus_read_byte_data(client, MAX6639_REG_DEVID);
	manu_id = i2c_smbus_read_byte_data(client, MAX6639_REG_MANUID);
	if (dev_id != 0x58 || manu_id != 0x4D)
		return -ENODEV;

	strscpy(info->type, "max6639", I2C_NAME_SIZE);

	return 0;
}

static void max6639_regulator_disable(void *data)
{
	regulator_disable(data);
}

static bool max6639_regmap_is_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX6639_REG_TEMP(0):
	case MAX6639_REG_TEMP_EXT(0):
	case MAX6639_REG_TEMP(1):
	case MAX6639_REG_TEMP_EXT(1):
	case MAX6639_REG_STATUS:
	case MAX6639_REG_FAN_CNT(0):
	case MAX6639_REG_FAN_CNT(1):
		return true;
	default:
		return false;
	}
}

static const struct regmap_config max6639_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX6639_REG_DEVREV,
	.cache_type = REGCACHE_MAPLE,
	.volatile_reg = max6639_regmap_is_volatile,
};

static int max6639_probe_child_from_dt(struct i2c_client *client,
				      struct device_node *child,
				      struct max6639_data *data)

{
	struct device *dev = &client->dev;
	u32 i, maxrpm;
	int val, err;

	err = of_property_read_u32(child, "reg", &i);
	if (err) {
		dev_err(dev, "missing reg property of %pOFn\n", child);
		return err;
	}

	if (i >= 2) {
		dev_err(dev, "invalid reg %d of %pOFn\n", i, child);
		return -EINVAL;
	}

	data->fan_enable[i] = true;

	err = of_property_read_u32(child, "pulses-per-revolution", &val);
	if (err) {
		dev_err(dev, "missing pulses-per-revolution property of %pOFn",
			child);
		return err;
	}

	if (val <= 0 || val > 5) {
		dev_err(dev, "invalid pulses-per-revolution %d of %pOFn\n", val,
			child);
		return -EINVAL;
	}
	data->ppr[i] = val;

	err = of_property_read_u32(child, "max-rpm", &maxrpm);
	if (err) {
		dev_err(dev, "missing max-rpm property of %pOFn\n", child);
		return err;
	}

	data->rpm_range[i] = rpm_range_to_reg(maxrpm);
	data->max_rpm[i] = maxrpm;

	err = of_property_read_u32(child, "target-rpm", &val);
	/* Use provided target RPM else default to maxrpm */
	if (!err)
		data->target_rpm[i] = val;
	else
		data->target_rpm[i] = maxrpm;

	/* Get pwms property for PWM control */
	data->pwmd[i] = devm_fwnode_pwm_get(dev, &child->fwnode, NULL);

	if (!IS_ERR(data->pwmd[i]))
		return 0;

	if (PTR_ERR(data->pwmd[i]) == -EPROBE_DEFER)
		return PTR_ERR(data->pwmd[i]);

	dev_dbg(dev, "Using chip default PWM");
	data->pwmd[i] = pwm_request_from_chip(&data->chip, i, NULL);
	if (!IS_ERR(data->pwmd[i]))
		return 0;

	dev_dbg(dev, "Failed to configure pwm for fan %d", i);
	return PTR_ERR_OR_ZERO(data->pwmd[i]);
}

static int max6639_probe_from_dt(struct i2c_client *client,
				struct max6639_data *data)
{
	struct device *dev = &client->dev;
	const struct device_node *np = dev->of_node;
	struct device_node *child;
	int err;

	/* Compatible with non-DT platforms */
	if (!np)
		return 0;

	for_each_child_of_node(np, child) {
		if (strcmp(child->name, "fan"))
			continue;

		err = max6639_probe_child_from_dt(client, child, data);
		if (err) {
			of_node_put(child);
			return err;
		}
	}

	return 0;
}

static int max6639_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max6639_data *data;
	int err;

	data = devm_kzalloc(dev, sizeof(struct max6639_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = devm_regmap_init_i2c(client, &max6639_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev,
				     PTR_ERR(data->regmap),
				     "regmap initialization failed\n");

	/* Add PWM controller of max6639 */
	data->chip.dev = dev;
	data->chip.ops = &max6639_pwm_ops;
	data->chip.npwm = MAX6639_NDEV;
	err = devm_pwmchip_add(dev, &data->chip);
	if (err < 0)
		return dev_err_probe(dev, err, "failed to add PWM chip\n");

	data->reg = devm_regulator_get_optional(dev, "fan");
	if (IS_ERR(data->reg)) {
		if (PTR_ERR(data->reg) != -ENODEV)
			return PTR_ERR(data->reg);

		data->reg = NULL;
	} else {
		/* Spin up fans */
		err = regulator_enable(data->reg);
		if (err) {
			dev_err(dev, "Failed to enable fan supply: %d\n", err);
			return err;
		}
		err = devm_add_action_or_reset(dev, max6639_regulator_disable,
					       data->reg);
		if (err) {
			dev_err(dev, "Failed to register action: %d\n", err);
			return err;
		}
	}

	mutex_init(&data->update_lock);
	/* Probe from DT to get configuration */
	err = max6639_probe_from_dt(client, data);
	if (err)
		return dev_err_probe(dev, err, "max6639 DT probe failed\n");

	/* Initialize the max6639 chip */
	err = max6639_init_client(client, data);
	if (err < 0)
		return err;

	data->hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
						     data, &max6639_chip_info,
						     NULL);
	if (IS_ERR(data->hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(data->hwmon_dev),
				     "unable to register hwmon device\n");
	return 0;
}

static int max6639_suspend(struct device *dev)
{
	struct max6639_data *data = dev_get_drvdata(dev);
	int ret, err;

	err = regmap_read(data->regmap, MAX6639_REG_GCONFIG, &ret);

	if (err < 0)
		return err;

	if (data->reg)
		regulator_disable(data->reg);

	return regmap_write(data->regmap, MAX6639_REG_GCONFIG, ret | MAX6639_GCONFIG_STANDBY);
}

static int max6639_resume(struct device *dev)
{
	struct max6639_data *data = dev_get_drvdata(dev);
	int ret, err;

	if (data->reg) {
		ret = regulator_enable(data->reg);
		if (ret) {
			dev_err(dev, "Failed to enable fan supply: %d\n", ret);
			return ret;
		}
	}

	err = regmap_read(data->regmap, MAX6639_REG_GCONFIG, &ret);
	if (err < 0)
		return err;

	return regmap_write(data->regmap, MAX6639_REG_GCONFIG, ret & ~MAX6639_GCONFIG_STANDBY);
}

static const struct i2c_device_id max6639_id[] = {
	{"max6639", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, max6639_id);

static DEFINE_SIMPLE_DEV_PM_OPS(max6639_pm_ops, max6639_suspend, max6639_resume);

static const struct of_device_id max6639_of_match[] = {
	{ .compatible = "maxim,max6639", },
	{ },
};

static struct i2c_driver max6639_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		   .name = "max6639",
		   .pm = pm_sleep_ptr(&max6639_pm_ops),
		   .of_match_table = max6639_of_match,
		   },
	.probe = max6639_probe,
	.id_table = max6639_id,
	.detect = max6639_detect,
	.address_list = normal_i2c,
};

module_i2c_driver(max6639_driver);

MODULE_AUTHOR("Roland Stigge <stigge@antcom.de>");
MODULE_DESCRIPTION("max6639 driver");
MODULE_LICENSE("GPL");
