// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for regulators in MAX5970 and MAX5978 IC
 *
 * Copyright (c) 2021 9elements GmbH
 *
 * Author: Patrick Rudolph <patrick.rudolph@9elements.com>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/version.h>

enum max597x_chip_type {
	MAX597x_TYPE_MAX5970,
	MAX597x_TYPE_MAX5978,
};

struct max597x_data {
	int num_switches;
	int shunt_micro_ohms;
	const struct max597x_regulator *regulator;
	struct regulator *parent;
	unsigned int uV;
	unsigned int irng;
	unsigned int mon_rng;
	struct regmap *regmap;
};

struct max597x_iio {
	int shunt_micro_ohms[2];
	struct regmap *regmap;
	unsigned int irng[2];
	unsigned int mon_rng[2];
};

struct max597x_regulator {
	unsigned int channel;
	struct regulator_desc rdesc;
};

enum max597x_regulator_id {
	MAX597X_SW0,
	MAX597X_SW1,
};

#define MAX5970_NUM_SWITCHES 2
#define MAX5978_NUM_SWITCHES 1

#define MAX5970_REG_CURRENT_L(ch)		(0x01 + (ch) * 4)
#define MAX5970_REG_CURRENT_H(ch)		(0x00 + (ch) * 4)
#define MAX5970_REG_VOLTAGE_L(ch)		(0x03 + (ch) * 4)
#define MAX5970_REG_VOLTAGE_H(ch)		(0x02 + (ch) * 4)
#define MAX5970_REG_MON_RANGE			0x18
#define  MAX5970_MON(reg, ch)			(((reg) >> (ch * 2)) & 3)
#define MAX5970_REG_CHx_UV_WARN_H(ch)		(0x1A + (ch) * 10)
#define MAX5970_REG_CHx_UV_WARN_L(ch)		(0x1B + (ch) * 10)
#define MAX5970_REG_CHx_UV_CRIT_H(ch)		(0x1C + (ch) * 10)
#define MAX5970_REG_CHx_UV_CRIT_L(ch)		(0x1D + (ch) * 10)
#define MAX5970_REG_CHx_OV_WARN_H(ch)		(0x1E + (ch) * 10)
#define MAX5970_REG_CHx_OV_WARN_L(ch)		(0x1F + (ch) * 10)
#define MAX5970_REG_CHx_OV_CRIT_H(ch)		(0x20 + (ch) * 10)
#define MAX5970_REG_CHx_OV_CRIT_L(ch)		(0x21 + (ch) * 10)

#define  MAX5970_VAL2REG_H(x)		(((x) >> 2) & 0xFF)
#define  MAX5970_VAL2REG_L(x)		((x) & 0x3)

#define MAX5970_REG_DAC_FAST(ch)	(0x2E + (ch))

#define MAX5970_FAST2SLOW_RATIO		200

#define MAX5970_REG_STATUS0		0x31
#define  MAX5970_CB_IFAULTF(ch)		(1 << (ch))
#define  MAX5970_CB_IFAULTS(ch)		(1 << ((ch) + 4))

#define MAX5970_REG_STATUS1		0x32
#define  STATUS1_PROT(reg)		(((reg) >> 6) & 0x3)
#define  STATUS1_PROT_SHUTDOWN		0
#define  STATUS1_PROT_CLEAR_PG		1
#define  STATUS1_PROT_ALERT_ONLY	2

#define MAX5970_REG_STATUS2		0x33
#define  MAX5970_IRNG(reg, ch)		(((reg) >> (ch * 2)) & 3)

#define MAX5970_REG_STATUS3		0x34
#define  MAX5970_STATUS3_ALERT		BIT(4)
#define  MAX5970_STATUS3_PG(ch)		BIT(ch)

#define MAX5970_REG_FAULT0		0x35
#define  UV_STATUS_WARN(ch)		(1 << (ch))
#define  UV_STATUS_CRIT(ch)		(1 << ((ch) + 4))

#define MAX5970_REG_FAULT1		0x36
#define  OV_STATUS_WARN(ch)		(1 << (ch))
#define  OV_STATUS_CRIT(ch)		(1 << ((ch) + 4))

#define MAX5970_REG_FAULT2		0x37
#define  OC_STATUS_WARN(ch)		(1 << (ch))

#define MAX5970_REG_CHXEN		0x3b
#define  CHXEN(ch)			(3 << ((ch) * 2))

#define MAX_REGISTERS			0x49
#define ADC_MASK			0x3FF

static int max597x_uvp_ovp_check_mode(struct regulator_dev *rdev, int severity)
{
	int ret, reg;

	/* Status1 register contains the soft strap values sampled at POR */
	ret = regmap_read(rdev->regmap, MAX5970_REG_STATUS1, &reg);
	if (ret)
		return ret;

	/* Check soft straps match requested mode */
	if (severity == REGULATOR_SEVERITY_PROT) {
		if (STATUS1_PROT(reg) != STATUS1_PROT_SHUTDOWN)
			return -EOPNOTSUPP;

		return 0;
	}
	if (STATUS1_PROT(reg) == STATUS1_PROT_SHUTDOWN)
		return -EOPNOTSUPP;

	return 0;
}

static int max597x_set_vp(struct regulator_dev *rdev, int lim_uV, int severity,
			  bool enable, bool overvoltage)
{
	int off_h, off_l, reg, ret;
	struct max597x_data *data = rdev_get_drvdata(rdev);

	const struct max597x_regulator *regulator = data->regulator;

	if (overvoltage) {
		if (severity == REGULATOR_SEVERITY_WARN) {
			off_h = MAX5970_REG_CHx_OV_WARN_H(regulator->channel);
			off_l = MAX5970_REG_CHx_OV_WARN_L(regulator->channel);
		} else {
			off_h = MAX5970_REG_CHx_OV_CRIT_H(regulator->channel);
			off_l = MAX5970_REG_CHx_OV_CRIT_L(regulator->channel);
		}
	} else {
		if (severity == REGULATOR_SEVERITY_WARN) {
			off_h = MAX5970_REG_CHx_UV_WARN_H(regulator->channel);
			off_l = MAX5970_REG_CHx_UV_WARN_L(regulator->channel);
		} else {
			off_h = MAX5970_REG_CHx_UV_CRIT_H(regulator->channel);
			off_l = MAX5970_REG_CHx_UV_CRIT_L(regulator->channel);
		}
	}

	if (enable)
		/* reg = ADC_MASK * (lim_uV / 1000000) / (data->mon_rng / 1000000) */
		reg = ADC_MASK * lim_uV / data->mon_rng;
	else
		reg = 0;

	ret = regmap_write(rdev->regmap, off_h, MAX5970_VAL2REG_H(reg));
	if (ret)
		return ret;

	ret = regmap_write(rdev->regmap, off_l, MAX5970_VAL2REG_L(reg));
	if (ret)
		return ret;

	return 0;
}

static int max597x_set_uvp(struct regulator_dev *rdev, int lim_uV, int severity,
			   bool enable)
{
	int ret;

	/*
	 * MAX5970 has enable control as a special value in limit reg. Can't
	 * set limit but keep feature disabled or enable W/O given limit.
	 */
	if ((lim_uV && !enable) || (!lim_uV && enable))
		return -EINVAL;

	ret = max597x_uvp_ovp_check_mode(rdev, severity);
	if (ret)
		return ret;

	return max597x_set_vp(rdev, lim_uV, severity, enable, false);
}

static int max597x_set_ovp(struct regulator_dev *rdev, int lim_uV, int severity,
			   bool enable)
{
	int ret;

	/*
	 * MAX5970 has enable control as a special value in limit reg. Can't
	 * set limit but keep feature disabled or enable W/O given limit.
	 */
	if ((lim_uV && !enable) || (!lim_uV && enable))
		return -EINVAL;

	ret = max597x_uvp_ovp_check_mode(rdev, severity);
	if (ret)
		return ret;

	return max597x_set_vp(rdev, lim_uV, severity, enable, true);
}

static int max597x_set_ocp(struct regulator_dev *rdev, int lim_uA,
			   int severity, bool enable)
{
	int ret, val, reg;
	unsigned int vthst, vthfst;

	struct max597x_data *data = rdev_get_drvdata(rdev);
	const struct max597x_regulator *regulator = data->regulator;

	/*
	 * MAX5970 has enable control as a special value in limit reg. Can't
	 * set limit but keep feature disabled or enable W/O given limit.
	 */
	if ((lim_uA && !enable) || (!lim_uA && enable))
		return -EINVAL;
	if (severity != REGULATOR_SEVERITY_PROT)
		return -EINVAL;

	/* Calc Vtrip threshold in uV. */
	vthst = div_u64(mul_u32_u32(data->shunt_micro_ohms, lim_uA), 1000000);

	/* Add 120% margin */
	vthst = div_u64(mul_u32_u32(vthst, 120), 100);

	/* Calc fast Vtrip threshold in uV */
	vthfst = vthst * (MAX5970_FAST2SLOW_RATIO / 100);

	if (vthfst > data->irng) {
		dev_err(&rdev->dev, "Current limit out of range\n");
		return -EINVAL;
	}

	/* Program fast trip threshold */
	if (enable)
		val = div_u64(mul_u32_u32(0xFF, vthfst), data->irng);
	else
		val = 0xFF;

	reg = MAX5970_REG_DAC_FAST(regulator->channel);
	ret = regmap_write(rdev->regmap, reg, val);

	return ret;
}

static int max597x_voltage_op(struct regulator_dev *rdev)
{
	struct max597x_data *data = rdev_get_drvdata(rdev);

	return data->uV;	/* Output is not regulated */
}

static int max597x_get_status(struct regulator_dev *rdev)
{
	int val, ret;

	ret = regmap_read(rdev->regmap, MAX5970_REG_STATUS3, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	if (val & MAX5970_STATUS3_ALERT)
		return REGULATOR_STATUS_ERROR;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0)
		return ret;

	if (ret)
		return REGULATOR_STATUS_ON;

	return REGULATOR_STATUS_OFF;
}

static int max597x_enable(struct regulator_dev *rdev)
{
	struct max597x_data *data = rdev_get_drvdata(rdev);
	int ret;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0)
		return ret;

	/* Nothing to do */
	if (ret)
		return 0;

	ret = regulator_enable(data->parent);
	if (ret < 0)
		return ret;
	return regulator_enable_regmap(rdev);
}

static int max597x_disable(struct regulator_dev *rdev)
{
	struct max597x_data *data = rdev_get_drvdata(rdev);
	int ret;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0)
		return ret;

	/* Nothing to do */
	if (!ret)
		return 0;

	ret = regulator_disable_regmap(rdev);
	if (ret < 0)
		return ret;
	return regulator_disable(data->parent);
}

static const struct regulator_ops max597x_switch_ops = {
	.enable				= max597x_enable,
	.disable			= max597x_disable,
	.is_enabled			= regulator_is_enabled_regmap,
	.get_voltage			= max597x_voltage_op,
	.get_status			= max597x_get_status,
	.set_over_voltage_protection	= max597x_set_ovp,
	.set_under_voltage_protection	= max597x_set_uvp,
	.set_over_current_protection	= max597x_set_ocp,
};

#define MAX597X_SWITCH(_ID, _ereg, _chan) {                      \
	.rdesc = {                                               \
		.name            = #_ID,                         \
		.of_match        = of_match_ptr(#_ID),           \
		.ops             = &max597x_switch_ops,          \
		.regulators_node = of_match_ptr("regulators"),   \
		.type            = REGULATOR_VOLTAGE,            \
		.id              = MAX597X_##_ID,                \
		.owner           = THIS_MODULE,                  \
		.enable_reg      = _ereg,                        \
		.enable_mask     = CHXEN((_chan)),               \
	},                                                       \
	.channel = (_chan),                                      \
}

static const struct max597x_regulator regulators[] = {
	MAX597X_SWITCH(SW0, MAX5970_REG_CHXEN, 0),
	MAX597X_SWITCH(SW1, MAX5970_REG_CHXEN, 1),
};

static int max597x_regmap_read_clear(struct regmap *map, unsigned int reg,
				     unsigned int *val)
{
	int ret;

	ret = regmap_read(map, reg, val);
	if (ret)
		return ret;

	if (*val)
		return regmap_write(map, reg, *val);

	return 0;
}

static int max597x_irq_handler(int irq, struct regulator_irq_data *rid,
			       unsigned long *dev_mask)
{
	struct regulator_err_state *stat;
	struct max597x_data *d = (struct max597x_data *)rid->data;
	int val, ret, i;

	ret = max597x_regmap_read_clear(d->regmap, MAX5970_REG_FAULT0, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	*dev_mask = 0;
	for (i = 0; i < d->num_switches; i++) {
		stat = &rid->states[i];
		stat->notifs = 0;
		stat->errors = 0;
	}

	for (i = 0; i < d->num_switches; i++) {
		stat = &rid->states[i];

		if (val & UV_STATUS_CRIT(i)) {
			*dev_mask |= 1 << i;
			stat->notifs |= REGULATOR_EVENT_UNDER_VOLTAGE;
			stat->errors |= REGULATOR_ERROR_UNDER_VOLTAGE;
		} else if (val & UV_STATUS_WARN(i)) {
			*dev_mask |= 1 << i;
			stat->notifs |= REGULATOR_EVENT_UNDER_VOLTAGE_WARN;
			stat->errors |= REGULATOR_ERROR_UNDER_VOLTAGE_WARN;
		}
	}

	ret = max597x_regmap_read_clear(d->regmap, MAX5970_REG_FAULT1, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	for (i = 0; i < d->num_switches; i++) {
		stat = &rid->states[i];

		if (val & OV_STATUS_CRIT(i)) {
			*dev_mask |= 1 << i;
			stat->notifs |= REGULATOR_EVENT_REGULATION_OUT;
			stat->errors |= REGULATOR_ERROR_REGULATION_OUT;
		} else if (val & OV_STATUS_WARN(i)) {
			*dev_mask |= 1 << i;
			stat->notifs |= REGULATOR_EVENT_OVER_VOLTAGE_WARN;
			stat->errors |= REGULATOR_ERROR_OVER_VOLTAGE_WARN;
		}
	}

	ret = max597x_regmap_read_clear(d->regmap, MAX5970_REG_FAULT2, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	for (i = 0; i < d->num_switches; i++) {
		stat = &rid->states[i];

		if (val & OC_STATUS_WARN(i)) {
			*dev_mask |= 1 << i;
			stat->notifs |= REGULATOR_EVENT_OVER_CURRENT_WARN;
			stat->errors |= REGULATOR_ERROR_OVER_CURRENT_WARN;
		}
	}

	ret = regmap_read(d->regmap, MAX5970_REG_STATUS0, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	for (i = 0; i < d->num_switches; i++) {
		stat = &rid->states[i];

		if ((val & MAX5970_CB_IFAULTF(i)) || (val & MAX5970_CB_IFAULTS(i))) {
			*dev_mask |= 1 << i;
			stat->notifs |= REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_DISABLE;
			stat->errors |= REGULATOR_ERROR_OVER_CURRENT | REGULATOR_ERROR_FAIL;

			/* Clear the sub-IRQ status */
			regulator_disable_regmap(stat->rdev);
		}
	}
	return 0;
}

static const struct regmap_config max597x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX_REGISTERS,
};

#define MAX597X_ADC_CHANNEL(_idx, _type) {			\
	.type = IIO_ ## _type,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.address = MAX5970_REG_ ## _type ## _L(_idx),		\
}

static const struct iio_chan_spec max5978_adc_iio_channels[] = {
	MAX597X_ADC_CHANNEL(0, VOLTAGE),
	MAX597X_ADC_CHANNEL(0, CURRENT),
};

static const struct iio_chan_spec max5970_adc_iio_channels[] = {
	MAX597X_ADC_CHANNEL(0, VOLTAGE),
	MAX597X_ADC_CHANNEL(0, CURRENT),
	MAX597X_ADC_CHANNEL(1, VOLTAGE),
	MAX597X_ADC_CHANNEL(1, CURRENT),
};

static int max597x_iio_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long info)
{
	int ret;
	struct max597x_iio *data = iio_priv(iio_dev);
	unsigned int reg_l, reg_h;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(data->regmap, chan->address, &reg_l);
		if (ret < 0)
			return ret;
		ret = regmap_read(data->regmap, chan->address - 1, &reg_h);
		if (ret < 0)
			return ret;
		*val = (reg_h << 2) | (reg_l & 3);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:

		switch (chan->address) {
		case MAX5970_REG_CURRENT_L(0):
			fallthrough;
		case MAX5970_REG_CURRENT_L(1):
			*val = data->irng[chan->channel] * 1000; /* in A, convert to mA */
			*val2 = data->shunt_micro_ohms[chan->channel] * ADC_MASK;
			return IIO_VAL_FRACTIONAL;

		case MAX5970_REG_VOLTAGE_L(0):
			fallthrough;
		case MAX5970_REG_VOLTAGE_L(1):
			*val = data->mon_rng[chan->channel]; /* in uV, convert to mV */
			*val2 = ADC_MASK * 1000;
			return IIO_VAL_FRACTIONAL;
		}

		break;
	}
	return -EINVAL;
}

static const struct iio_info max597x_adc_iio_info = {
	.read_raw = &max597x_iio_read_raw,
};

static int max597x_parse_dt(struct device *dev, struct max597x_data *data)
{
	struct device_node *regulators;
	struct of_regulator_match match;
	u32 shunt_microohms;
	int ret;

	regulators = of_get_child_by_name(dev->of_node, "regulators");
	if (!regulators) {
		dev_err(dev, "regulator node not found\n");
		return -ENODEV;
	}

	match.name = data->regulator->rdesc.name;
	ret = of_regulator_match(dev, regulators, &match, 1);
	of_node_put(regulators);

	if (ret < 0) {
		dev_err(dev, "Regulator match failed, err %d\n", ret);
		return ret;
	}

	if (match.of_node) {
		ret = of_property_read_u32(match.of_node, "shunt-resistor-micro-ohms",
					   &shunt_microohms);
		if (ret < 0) {
			dev_err(dev, "property 'shunt-resistor-micro-ohms' not found, err %d\n",
				ret);
			return ret;
		}

		data->shunt_micro_ohms = shunt_microohms;
	}

	return 0;
}

static int max597x_adc_range(struct device *dev,
			     struct regmap *regmap,
			     const int ch,
			     struct max597x_data *data)
{
	unsigned int reg;
	int ret;

	/* Decode current ADC range */
	ret = regmap_read(regmap, MAX5970_REG_STATUS2, &reg);
	if (ret)
		return ret;
	switch (MAX5970_IRNG(reg, ch)) {
	case 0:
		data->irng = 100000; /* 100 mV */
		break;
	case 1:
		data->irng = 50000; /* 50 mV */
		break;
	case 2:
		data->irng = 25000; /* 25 mV */
		break;
	default:
		return -EINVAL;
	}
	dev_info(dev, "Current ADC%d full-scale %d mV\n", ch, data->irng / 1000);

	/* Decode current voltage monitor range */
	ret = regmap_read(regmap, MAX5970_REG_MON_RANGE, &reg);
	if (ret)
		return ret;

	data->mon_rng = 16000000 >> MAX5970_MON(reg, ch);
	dev_info(dev, "Voltage ADC%d limit %d mV\n", ch, data->mon_rng / 1000);

	return 0;
}

static int max597x_setup_irq(struct device *dev,
			     int irq,
			     struct regulator_dev *rdevs[MAX5970_NUM_SWITCHES],
			     int num_switches,
			     struct max597x_data *data)
{
	struct regulator_irq_desc max597x_notif = {
		.name = "max597x-irq",
		.map_event = max597x_irq_handler,
		.data = data,
	};
	int errs = REGULATOR_ERROR_UNDER_VOLTAGE |
		   REGULATOR_ERROR_UNDER_VOLTAGE_WARN |
		   REGULATOR_ERROR_OVER_VOLTAGE_WARN |
		   REGULATOR_ERROR_REGULATION_OUT |
		   REGULATOR_ERROR_OVER_CURRENT |
		   REGULATOR_ERROR_OVER_CURRENT_WARN |
		   REGULATOR_ERROR_FAIL;
	void *irq_helper;

	/* Register notifiers - can fail if IRQ is not given */
	irq_helper = devm_regulator_irq_helper(dev, &max597x_notif,
					       irq, 0, errs, NULL,
					       &rdevs[0], num_switches);
	if (IS_ERR(irq_helper)) {
		if (PTR_ERR(irq_helper) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_warn(dev, "IRQ disabled %pe\n", irq_helper);
	}

	return 0;
}

static int max597x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	unsigned int i;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regulator_dev *rdevs[MAX5970_NUM_SWITCHES];
	struct max597x_data *data;
	int num_switches = 0;
	struct regmap *regmap;
	struct iio_dev *indio_dev;
	struct max597x_iio *priv;
	int ret;
	enum max597x_chip_type chip = id->driver_data;

	switch (chip) {
	case MAX597x_TYPE_MAX5970:
		num_switches = 2;
		break;
	case MAX597x_TYPE_MAX5978:
		num_switches = 1;
		break;
	}

	regmap = devm_regmap_init_i2c(cl, &max597x_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&cl->dev, "No regmap\n");
		return -EINVAL;
	}

	/* registering iio */
	indio_dev = devm_iio_device_alloc(&cl->dev, sizeof(*priv));
	if (!indio_dev) {
		dev_err(&cl->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}
	indio_dev->name = dev_name(&cl->dev);
	indio_dev->info = &max597x_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	if (num_switches == 1) {
		indio_dev->channels = max5978_adc_iio_channels;
		indio_dev->num_channels = ARRAY_SIZE(max5978_adc_iio_channels);
	} else {
		indio_dev->channels = max5970_adc_iio_channels;
		indio_dev->num_channels = ARRAY_SIZE(max5970_adc_iio_channels);
	}

	priv = iio_priv(indio_dev);
	priv->regmap = regmap;

	for (i = 0; i < num_switches; i++) {
		data = devm_kzalloc(&cl->dev, sizeof(struct max597x_data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		data->num_switches = num_switches;
		data->regulator = &regulators[i];
		data->regmap = regmap;
		data->parent = devm_regulator_get(&cl->dev, i == 0 ? "vss1": "vss2");
		if (IS_ERR(data->parent))
			return PTR_ERR(data->parent);

		/* Store supply voltage to return only supported output voltage */
		ret = regulator_get_voltage(data->parent);
		if (ret < 0)
			return ret;

		data->uV = ret;

		ret = max597x_parse_dt(&cl->dev, data);
		if (ret < 0)
			return ret;

		ret = max597x_adc_range(&cl->dev, regmap, i, data);
		if (ret < 0)
			goto err_regulator_disable;

		/* Set shunt value for IIO backend */
		priv->shunt_micro_ohms[i] = data->shunt_micro_ohms;
		priv->irng[i] = data->irng;
		priv->mon_rng[i] = data->mon_rng;
		dev_info(&cl->dev, "Shunt%d ADC upper limit %d mA\n", i,
			 data->irng * 1000 / data->shunt_micro_ohms);

		config.dev = &cl->dev;
		config.driver_data = (void *)data;
		config.regmap = regmap;
		rdev = devm_regulator_register(&cl->dev,
					       &regulators[i].rdesc,
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&cl->dev, "failed to register regulator %s\n",
				regulators[i].rdesc.name);
			return PTR_ERR(rdev);
		}
		rdevs[i] = rdev;
	}

	if (cl->irq) {
		ret = max597x_setup_irq(&cl->dev, cl->irq, rdevs, num_switches, data);
		if (ret) {
			dev_err(&cl->dev, "IRQ setup failed");
			return ret;
		}
	}

	ret = devm_iio_device_register(&cl->dev, indio_dev);
	if (ret) {
		dev_err(&cl->dev, "could not register iio device");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id max597x_table[] = {
	{ .name = "max5970", MAX597x_TYPE_MAX5970 },
	{ .name = "max5978", MAX597x_TYPE_MAX5978 },
	{},
};
MODULE_DEVICE_TABLE(i2c, max597x_table);

static const struct of_device_id max597x_of_match[] = {
	{ .compatible = "maxim,max5970", .data = (void *)MAX597x_TYPE_MAX5970 },
	{ .compatible = "maxim,max5978", .data = (void *)MAX597x_TYPE_MAX5978 },
	{},
};
MODULE_DEVICE_TABLE(of, max597x_of_match);

static struct i2c_driver max597x_driver = {
	.id_table = max597x_table,
	.driver = {
		.name	= "max597x",
		.of_match_table	= of_match_ptr(max597x_of_match),
	},
	.probe	= max597x_probe,
};
module_i2c_driver(max597x_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("MAX5970_hot-swap controller driver");
MODULE_LICENSE("GPL v2");
