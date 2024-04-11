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
#include <linux/platform_data/max6639.h>
#include <linux/pwm.h>

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
#define MAX6639_FAN_CONFIG3_DEFAULT		0x83
#define MAX6639_FAN_CONFIG3_FREQ_MASK		0x03
#define MAX6639_REG_TARGTDUTY_SLOT		120

#define MAX6639_FAN_CONFIG3_THERM_FULL_SPEED	0x40

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
	struct i2c_client *client;
	struct mutex update_lock;
	bool valid;		/* true if following fields are valid */
	unsigned long last_updated;	/* In jiffies */

	/* Register values sampled regularly */
	u16 temp[2];		/* Temperature, in 1/8 C, 0..255 C */
	bool temp_fault[2];	/* Detected temperature diode failure */
	u8 fan[2];		/* Register value: TACH count for fans >=30 */
	u8 status;		/* Detected channel alarms and fan failures */

	/* Register values only written to */
	u8 pwm[2];		/* Register value: Duty cycle 0..120 */
	u8 temp_therm[2];	/* THERM Temperature, 0..255 C (->_max) */
	u8 temp_alert[2];	/* ALERT Temperature, 0..255 C (->_crit) */
	u8 temp_ot[2];		/* OT Temperature, 0..255 C (->_emergency) */

	/* Register values initialized only once */
	u8 ppr;			/* Pulses per rotation 0..3 for 1..4 ppr */
	u8 rpm_range;		/* Index in above rpm_ranges table */

	/* Optional regulator for FAN supply */
	struct regulator *reg;
	/* max6639 pwm chip */
	struct pwm_chip chip;
	struct pwm_device *pwmd[2]; /* max6639 has two pwm device */
};

static struct max6639_data *max6639_update_device(struct device *dev)
{
	struct max6639_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct max6639_data *ret = data;
	int i;
	int status_reg;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + 2 * HZ) || !data->valid) {
		int res;

		dev_dbg(&client->dev, "Starting max6639 update\n");

		status_reg = i2c_smbus_read_byte_data(client,
						      MAX6639_REG_STATUS);
		if (status_reg < 0) {
			ret = ERR_PTR(status_reg);
			goto abort;
		}

		data->status = status_reg;

		for (i = 0; i < 2; i++) {
			res = i2c_smbus_read_byte_data(client,
					MAX6639_REG_FAN_CNT(i));
			if (res < 0) {
				ret = ERR_PTR(res);
				goto abort;
			}
			data->fan[i] = res;

			res = i2c_smbus_read_byte_data(client,
					MAX6639_REG_TEMP_EXT(i));
			if (res < 0) {
				ret = ERR_PTR(res);
				goto abort;
			}
			data->temp[i] = res >> 5;
			data->temp_fault[i] = res & 0x01;

			res = i2c_smbus_read_byte_data(client,
					MAX6639_REG_TEMP(i));
			if (res < 0) {
				ret = ERR_PTR(res);
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

static ssize_t temp_input_show(struct device *dev,
			       struct device_attribute *dev_attr, char *buf)
{
	long temp;
	struct max6639_data *data = max6639_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	if (IS_ERR(data))
		return PTR_ERR(data);

	temp = data->temp[attr->index] * 125;
	return sprintf(buf, "%ld\n", temp);
}

static ssize_t temp_fault_show(struct device *dev,
			       struct device_attribute *dev_attr, char *buf)
{
	struct max6639_data *data = max6639_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	if (IS_ERR(data))
		return PTR_ERR(data);

	return sprintf(buf, "%d\n", data->temp_fault[attr->index]);
}

static ssize_t temp_max_show(struct device *dev,
			     struct device_attribute *dev_attr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", (data->temp_therm[attr->index] * 1000));
}

static ssize_t temp_max_store(struct device *dev,
			      struct device_attribute *dev_attr,
			      const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned long val;
	int res;

	res = kstrtoul(buf, 10, &val);
	if (res)
		return res;

	mutex_lock(&data->update_lock);
	data->temp_therm[attr->index] = TEMP_LIMIT_TO_REG(val);
	i2c_smbus_write_byte_data(client,
				  MAX6639_REG_THERM_LIMIT(attr->index),
				  data->temp_therm[attr->index]);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t temp_crit_show(struct device *dev,
			      struct device_attribute *dev_attr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", (data->temp_alert[attr->index] * 1000));
}

static ssize_t temp_crit_store(struct device *dev,
			       struct device_attribute *dev_attr,
			       const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned long val;
	int res;

	res = kstrtoul(buf, 10, &val);
	if (res)
		return res;

	mutex_lock(&data->update_lock);
	data->temp_alert[attr->index] = TEMP_LIMIT_TO_REG(val);
	i2c_smbus_write_byte_data(client,
				  MAX6639_REG_ALERT_LIMIT(attr->index),
				  data->temp_alert[attr->index]);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t temp_emergency_show(struct device *dev,
				   struct device_attribute *dev_attr,
				   char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", (data->temp_ot[attr->index] * 1000));
}

static ssize_t temp_emergency_store(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned long val;
	int res;

	res = kstrtoul(buf, 10, &val);
	if (res)
		return res;

	mutex_lock(&data->update_lock);
	data->temp_ot[attr->index] = TEMP_LIMIT_TO_REG(val);
	i2c_smbus_write_byte_data(client,
				  MAX6639_REG_OT_LIMIT(attr->index),
				  data->temp_ot[attr->index]);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t pwm_show(struct device *dev, struct device_attribute *dev_attr,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);
	struct pwm_state state;

	pwm_get_state(data->pwmd[attr->index], &state);

	return sprintf(buf, "%d\n", pwm_get_relative_duty_cycle(&state, 255));
}

static ssize_t pwm_store(struct device *dev,
			 struct device_attribute *dev_attr, const char *buf,
			 size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct max6639_data *data = dev_get_drvdata(dev);
	struct pwm_state state;
	unsigned long val;
	int res;

	res = kstrtoul(buf, 10, &val);
	if (res)
		return res;

	val = clamp_val(val, 0, 255);

	pwm_get_state(data->pwmd[attr->index], &state);
	pwm_set_relative_duty_cycle(&state, val, 255);
	pwm_apply_state(data->pwmd[attr->index], &state);

	return count;
}

static ssize_t fan_input_show(struct device *dev,
			      struct device_attribute *dev_attr, char *buf)
{
	struct max6639_data *data = max6639_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	if (IS_ERR(data))
		return PTR_ERR(data);

	return sprintf(buf, "%d\n", FAN_FROM_REG(data->fan[attr->index],
		       data->rpm_range));
}

static ssize_t alarm_show(struct device *dev,
			  struct device_attribute *dev_attr, char *buf)
{
	struct max6639_data *data = max6639_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	if (IS_ERR(data))
		return PTR_ERR(data);

	return sprintf(buf, "%d\n", !!(data->status & (1 << attr->index)));
}

static SENSOR_DEVICE_ATTR_RO(temp1_input, temp_input, 0);
static SENSOR_DEVICE_ATTR_RO(temp2_input, temp_input, 1);
static SENSOR_DEVICE_ATTR_RO(temp1_fault, temp_fault, 0);
static SENSOR_DEVICE_ATTR_RO(temp2_fault, temp_fault, 1);
static SENSOR_DEVICE_ATTR_RW(temp1_max, temp_max, 0);
static SENSOR_DEVICE_ATTR_RW(temp2_max, temp_max, 1);
static SENSOR_DEVICE_ATTR_RW(temp1_crit, temp_crit, 0);
static SENSOR_DEVICE_ATTR_RW(temp2_crit, temp_crit, 1);
static SENSOR_DEVICE_ATTR_RW(temp1_emergency, temp_emergency, 0);
static SENSOR_DEVICE_ATTR_RW(temp2_emergency, temp_emergency, 1);
static SENSOR_DEVICE_ATTR_RW(pwm1, pwm, 0);
static SENSOR_DEVICE_ATTR_RW(pwm2, pwm, 1);
static SENSOR_DEVICE_ATTR_RO(fan1_input, fan_input, 0);
static SENSOR_DEVICE_ATTR_RO(fan2_input, fan_input, 1);
static SENSOR_DEVICE_ATTR_RO(fan1_fault, alarm, 1);
static SENSOR_DEVICE_ATTR_RO(fan2_fault, alarm, 0);
static SENSOR_DEVICE_ATTR_RO(temp1_max_alarm, alarm, 3);
static SENSOR_DEVICE_ATTR_RO(temp2_max_alarm, alarm, 2);
static SENSOR_DEVICE_ATTR_RO(temp1_crit_alarm, alarm, 7);
static SENSOR_DEVICE_ATTR_RO(temp2_crit_alarm, alarm, 6);
static SENSOR_DEVICE_ATTR_RO(temp1_emergency_alarm, alarm, 5);
static SENSOR_DEVICE_ATTR_RO(temp2_emergency_alarm, alarm, 4);


static struct attribute *max6639_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp1_fault.dev_attr.attr,
	&sensor_dev_attr_temp2_fault.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp2_crit.dev_attr.attr,
	&sensor_dev_attr_temp1_emergency.dev_attr.attr,
	&sensor_dev_attr_temp2_emergency.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan1_fault.dev_attr.attr,
	&sensor_dev_attr_fan2_fault.dev_attr.attr,
	&sensor_dev_attr_temp1_max_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_max_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_emergency_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_emergency_alarm.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(max6639);

static struct max6639_data *to_max6639_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct max6639_data, chip);
}

static int max6639_pwm_get_state(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_state *state)
{

	struct max6639_data *data = to_max6639_pwm(chip);
	struct i2c_client *client = data->client;
	int value, i = pwm->hwpwm, x;
	unsigned int freq;

	mutex_lock(&data->update_lock);

	value = i2c_smbus_read_byte_data(client, MAX6639_REG_FAN_CONFIG1(i));
	if (value < 0)
		goto abort;

	if (value & MAX6639_FAN_CONFIG1_PWM) {
		state->enabled = true;

		/* Determine frequency from respective registers */
		value = i2c_smbus_read_byte_data(client,
						 MAX6639_REG_FAN_CONFIG3(i));
		if (value < 0)
			goto abort;
		x = value & MAX6639_FAN_CONFIG3_FREQ_MASK;

		value = i2c_smbus_read_byte_data(client, MAX6639_REG_GCONFIG);
		if (value < 0)
			goto abort;
		if (value & MAX6639_GCONFIG_PWM_FREQ_HI)
			x |= 0x4;
		x &= 0x7;
		freq = freq_table[x];

		state->period = DIV_ROUND_UP(NSEC_PER_SEC, freq);

		value = i2c_smbus_read_byte_data(client,
						 MAX6639_REG_TARGTDUTY(i));
		if (value < 0)
			goto abort;
		/* max6639 supports 120 slots only */
		state->duty_cycle = mul_u64_u32_div(state->period, value, 120);

		value = i2c_smbus_read_byte_data(client,
						 MAX6639_REG_FAN_CONFIG2a(i));
		if (value < 0)
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
	struct i2c_client *client = data->client;
	int value = 0, i = pwm->hwpwm, x;
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

		value = i2c_smbus_read_byte_data(client,
						 MAX6639_REG_FAN_CONFIG3(i));
		if (value < 0)
			goto abort;

		value &= ~MAX6639_FAN_CONFIG3_FREQ_MASK;
		value |= (x & MAX6639_FAN_CONFIG3_FREQ_MASK);
		value = i2c_smbus_write_byte_data(client,
						  MAX6639_REG_FAN_CONFIG3(i),
						  value);
		if (value < 0)
			goto abort;

		value = i2c_smbus_read_byte_data(client, MAX6639_REG_GCONFIG);
		if (value < 0)
			goto abort;

		if (x >> 2)
			value &= ~MAX6639_GCONFIG_PWM_FREQ_HI;
		else
			value |= MAX6639_GCONFIG_PWM_FREQ_HI;
		value = i2c_smbus_write_byte_data(client, MAX6639_REG_GCONFIG,
						  value);
		if (value < 0)
			goto abort;
	}

	/* Configure dutycycle */
	if (state->duty_cycle != cstate.duty_cycle ||
	    state->period != cstate.period) {
		value = DIV_ROUND_DOWN_ULL(
				state->duty_cycle * MAX6639_REG_TARGTDUTY_SLOT,
				state->period);
		value = i2c_smbus_write_byte_data(client,
						  MAX6639_REG_TARGTDUTY(i),
						  value);
		if (value < 0)
			goto abort;
	}

	/* Configure polarity */
	if (state->polarity != cstate.polarity) {
		value = i2c_smbus_read_byte_data(client,
						 MAX6639_REG_FAN_CONFIG2a(i));
		if (value < 0)
			goto abort;
		if (state->polarity == PWM_POLARITY_NORMAL)
			value |= MAX6639_REG_FAN_CONFIG2a_PWM_POL;
		else
			value &= ~MAX6639_REG_FAN_CONFIG2a_PWM_POL;
		value = i2c_smbus_write_byte_data(client,
						  MAX6639_REG_FAN_CONFIG2a(i),
						  value);
		if (value < 0)
			goto abort;
	}

	if (state->enabled != cstate.enabled) {
		value = i2c_smbus_read_byte_data(client, MAX6639_REG_FAN_CONFIG1(i));
		if (value < 0)
			goto abort;
		if (state->enabled)
			value |= MAX6639_FAN_CONFIG1_PWM;
		else
			value &= ~MAX6639_FAN_CONFIG1_PWM;

		value = i2c_smbus_write_byte_data(client, MAX6639_REG_FAN_CONFIG1(i),
						value);
		if (value < 0)
			goto abort;
	}
	value = 0;

abort:
	mutex_unlock(&data->update_lock);

	return value;
}

static const struct pwm_ops max6639_pwm_ops = {
	.apply = max6639_pwm_apply,
	.get_state = max6639_pwm_get_state,
};

/*
 *  returns respective index in rpm_ranges table
 *  1 by default on invalid range
 */
static int rpm_range_to_reg(int range)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rpm_ranges); i++) {
		if (rpm_ranges[i] == range)
			return i;
	}

	return 1; /* default: 4000 RPM */
}

static int max6639_init_client(struct i2c_client *client,
			       struct max6639_data *data)
{
	struct max6639_platform_data *max6639_info =
		dev_get_platdata(&client->dev);
	int i;
	int rpm_range = 1; /* default: 4000 RPM */
	struct pwm_state state;
	int err;

	/* Reset chip to default values, see below for GCONFIG setup */
	err = i2c_smbus_write_byte_data(client, MAX6639_REG_GCONFIG,
				  MAX6639_GCONFIG_POR);
	if (err)
		goto exit;

	/* Fans pulse per revolution is 2 by default */
	if (max6639_info && max6639_info->ppr > 0 &&
			max6639_info->ppr < 5)
		data->ppr = max6639_info->ppr;
	else
		data->ppr = 2;
	data->ppr -= 1;

	if (max6639_info)
		rpm_range = rpm_range_to_reg(max6639_info->rpm_range);
	data->rpm_range = rpm_range;

	for (i = 0; i < 2; i++) {

		/* Set Fan pulse per revolution */
		err = i2c_smbus_write_byte_data(client,
				MAX6639_REG_FAN_PPR(i),
				data->ppr << 6);
		if (err)
			goto exit;

		/* Fans config PWM, RPM */
		err = i2c_smbus_write_byte_data(client,
			MAX6639_REG_FAN_CONFIG1(i),
			MAX6639_FAN_CONFIG1_PWM | rpm_range);
		if (err)
			goto exit;

		/* Fans PWM polarity high by default */
		if (max6639_info && max6639_info->pwm_polarity == 0)
			err = i2c_smbus_write_byte_data(client,
				MAX6639_REG_FAN_CONFIG2a(i), 0x00);
		else
			err = i2c_smbus_write_byte_data(client,
				MAX6639_REG_FAN_CONFIG2a(i), 0x02);
		if (err)
			goto exit;

		/*
		 * /THERM full speed enable,
		 * PWM frequency 25kHz, see also GCONFIG below
		 */
		err = i2c_smbus_write_byte_data(client,
			MAX6639_REG_FAN_CONFIG3(i),
			MAX6639_FAN_CONFIG3_THERM_FULL_SPEED | 0x03);
		if (err)
			goto exit;

		/* Max. temp. 80C/90C/100C */
		data->temp_therm[i] = 80;
		data->temp_alert[i] = 90;
		data->temp_ot[i] = 100;
		err = i2c_smbus_write_byte_data(client,
				MAX6639_REG_THERM_LIMIT(i),
				data->temp_therm[i]);
		if (err)
			goto exit;
		err = i2c_smbus_write_byte_data(client,
				MAX6639_REG_ALERT_LIMIT(i),
				data->temp_alert[i]);
		if (err)
			goto exit;
		err = i2c_smbus_write_byte_data(client,
				MAX6639_REG_OT_LIMIT(i), data->temp_ot[i]);
		if (err)
			goto exit;

		dev_dbg(&client->dev, "Using chip default PWM");
		data->pwmd[i] = pwm_request_from_chip(&data->chip, i, NULL);
		if (IS_ERR(data->pwmd[i]))
			return PTR_ERR(data->pwmd[i]);
		pwm_get_state(data->pwmd[i], &state);
		state.period = DIV_ROUND_UP(NSEC_PER_SEC, 25000);
		state.polarity = PWM_POLARITY_NORMAL;
		pwm_set_relative_duty_cycle(&state, 0, 255);
		pwm_apply_state(data->pwmd[i], &state);
	}
	/* Start monitoring */
	err = i2c_smbus_write_byte_data(client, MAX6639_REG_GCONFIG,
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

static int max6639_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max6639_data *data;
	struct device *hwmon_dev;
	int err;

	data = devm_kzalloc(dev, sizeof(struct max6639_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	/* Add PWM controller of max6639 */
	data->chip.dev = dev;
	data->chip.ops = &max6639_pwm_ops;
	data->chip.npwm = 2;
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

	/* Initialize the max6639 chip */
	err = max6639_init_client(client, data);
	if (err < 0)
		return err;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data,
							   max6639_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static int max6639_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max6639_data *data = dev_get_drvdata(dev);
	int ret = i2c_smbus_read_byte_data(client, MAX6639_REG_GCONFIG);

	if (ret < 0)
		return ret;

	if (data->reg)
		regulator_disable(data->reg);

	return i2c_smbus_write_byte_data(client,
			MAX6639_REG_GCONFIG, ret | MAX6639_GCONFIG_STANDBY);
}

static int max6639_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max6639_data *data = dev_get_drvdata(dev);
	int ret;

	if (data->reg) {
		ret = regulator_enable(data->reg);
		if (ret) {
			dev_err(dev, "Failed to enable fan supply: %d\n", ret);
			return ret;
		}
	}

	ret = i2c_smbus_read_byte_data(client, MAX6639_REG_GCONFIG);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
			MAX6639_REG_GCONFIG, ret & ~MAX6639_GCONFIG_STANDBY);
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
