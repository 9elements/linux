// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for regulators in MAX5970 and MAX5978 IC
 *
 * Copyright (c) 2022 9elements GmbH
 *
 * Author: Patrick Rudolph <patrick.rudolph@9elements.com>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/version.h>

#include "max597x.h"

struct max597x_data {
	int num_switches;
	int shunt_micro_ohms;
	unsigned int irng;
	unsigned int mon_rng;
	struct regmap *regmap;
};

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
	int rdev_id = rdev_get_id(rdev);

	if (overvoltage) {
		if (severity == REGULATOR_SEVERITY_WARN) {
			off_h = MAX5970_REG_CHx_OV_WARN_H(rdev_id);
			off_l = MAX5970_REG_CHx_OV_WARN_L(rdev_id);
		} else {
			off_h = MAX5970_REG_CHx_OV_CRIT_H(rdev_id);
			off_l = MAX5970_REG_CHx_OV_CRIT_L(rdev_id);
		}
	} else {
		if (severity == REGULATOR_SEVERITY_WARN) {
			off_h = MAX5970_REG_CHx_UV_WARN_H(rdev_id);
			off_l = MAX5970_REG_CHx_UV_WARN_L(rdev_id);
		} else {
			off_h = MAX5970_REG_CHx_UV_CRIT_H(rdev_id);
			off_l = MAX5970_REG_CHx_UV_CRIT_L(rdev_id);
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
	int rdev_id = rdev_get_id(rdev);

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

	reg = MAX5970_REG_DAC_FAST(rdev_id);
	ret = regmap_write(rdev->regmap, reg, val);

	return ret;
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

static const struct regulator_ops max597x_switch_ops = {
	.enable				= regulator_enable_regmap,
	.disable			= regulator_disable_regmap,
	.is_enabled			= regulator_is_enabled_regmap,
	.get_status			= max597x_get_status,
	.set_over_voltage_protection	= max597x_set_ovp,
	.set_under_voltage_protection	= max597x_set_uvp,
	.set_over_current_protection	= max597x_set_ocp,
};

#define MAX597X_SWITCH(_ID, _ereg, _chan, _supply) {     \
	.name            = #_ID,                         \
	.of_match        = of_match_ptr(#_ID),           \
	.ops             = &max597x_switch_ops,          \
	.regulators_node = of_match_ptr("regulators"),   \
	.type            = REGULATOR_VOLTAGE,            \
	.id              = MAX597X_##_ID,                \
	.owner           = THIS_MODULE,                  \
	.supply_name     = _supply,                      \
	.enable_reg      = _ereg,                        \
	.enable_mask     = CHXEN((_chan)),               \
}

static const struct regulator_desc regulators[] = {
	MAX597X_SWITCH(SW0, MAX5970_REG_CHXEN, 0, "vss1"),
	MAX597X_SWITCH(SW1, MAX5970_REG_CHXEN, 1, "vss2"),
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

static int max597x_led_enable(struct device *dev, struct regmap *regmap)
{
	u32 led_enable;
	int ret;

	ret = of_property_read_u32(dev->of_node, "led-enable", &led_enable);
	if (!ret) {
		ret = regmap_update_bits(regmap, MAX5970_REG_LED_FLASH,
					 led_enable, 0);
		if (ret < 0) {
			dev_err(dev, "max597x: led_enable failed, err %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int max597x_parse_dt(struct device *dev, const struct regulator_desc *desc,
			    u32 *shunt_micro_ohms)
{
	struct device_node *node;
	struct of_regulator_match match;
	int ret;

	node = of_get_child_by_name(dev->of_node, "regulators");
	if (!node) {
		dev_err(dev, "regulator node not found\n");
		return -ENODEV;
	}

	match.name = desc->name;
	ret = of_regulator_match(dev, node, &match, 1);
	of_node_put(node);

	if (ret < 0) {
		dev_err(dev, "Regulator match failed, err %d\n", ret);
		return ret;
	}

	if (match.of_node) {
		ret = of_property_read_u32(match.of_node, "shunt-resistor-micro-ohms",
					   shunt_micro_ohms);
		if (ret < 0) {
			dev_err(dev, "property 'shunt-resistor-micro-ohms' not found, err %d\n",
				ret);
			return ret;
		}
	}

	return 0;
}

static int max597x_adc_range(struct regmap *regmap, const int ch,
			     u32 *irng, u32 *mon_rng)
{
	unsigned int reg;
	int ret;

	/* Decode current ADC range */
	ret = regmap_read(regmap, MAX5970_REG_STATUS2, &reg);
	if (ret)
		return ret;
	switch (MAX5970_IRNG(reg, ch)) {
	case 0:
		*irng = 100000; /* 100 mV */
		break;
	case 1:
		*irng = 50000; /* 50 mV */
		break;
	case 2:
		*irng = 25000; /* 25 mV */
		break;
	default:
		return -EINVAL;
	}

	/* Decode current voltage monitor range */
	ret = regmap_read(regmap, MAX5970_REG_MON_RANGE, &reg);
	if (ret)
		return ret;

	*mon_rng = MAX5970_MON_MAX_RANGE_UV >> MAX5970_MON(reg, ch);

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
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regulator_dev *rdevs[MAX5970_NUM_SWITCHES];
	struct max597x_data *data;
	int num_switches = 0;
	struct regmap *regmap;
	int ret, i;
	enum max597x_chip_type chip = id->driver_data;
	u32 irng[MAX5970_NUM_SWITCHES] = { }, mon_rng[MAX5970_NUM_SWITCHES] = { };
	u32 shunt_micro_ohms[MAX5970_NUM_SWITCHES] = { };

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

	for (i = 0; i < num_switches; i++) {
		data = devm_kzalloc(&cl->dev, sizeof(struct max597x_data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		data->num_switches = num_switches;
		data->regmap = regmap;

		ret = max597x_parse_dt(&cl->dev, &regulators[i],
				       &shunt_micro_ohms[i]);
		if (ret < 0)
			return ret;

		ret = max597x_adc_range(regmap, i, &irng[i], &mon_rng[i]);
		if (ret < 0)
			return ret;

		data->shunt_micro_ohms = shunt_micro_ohms[i];
		data->irng = irng[i];
		data->mon_rng = mon_rng[i];

		config.dev = &cl->dev;
		config.driver_data = (void *)data;
		config.regmap = regmap;
		rdev = devm_regulator_register(&cl->dev,
					       &regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&cl->dev, "failed to register regulator %s\n",
				regulators[i].name);
			return PTR_ERR(rdev);
		}
		rdevs[i] = rdev;
	}

	ret = max597x_led_enable(&cl->dev, regmap);
	if (ret)
		return ret;

	if (cl->irq) {
		ret = max597x_setup_irq(&cl->dev, cl->irq, rdevs, num_switches, data);
		if (ret) {
			dev_err(&cl->dev, "IRQ setup failed");
			return ret;
		}
	}

	return max597x_iio_configure(&cl->dev, chip, regmap, irng, mon_rng, shunt_micro_ohms);
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
