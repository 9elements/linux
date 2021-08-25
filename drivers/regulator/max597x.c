// SPDX-License-Identifier: GPL-2.0
//
// Device driver for regulators in MAX5970 and MAX5978 IC
//
// Copyright (c) 2021 9elements GmbH
//
// Authors:
// patrick Rudolph <patrick.rudolph@9elements.com>

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

enum max597x_chip_type {
	MAX597x_TYPE_MAX5970,
	MAX597x_TYPE_MAX5978,
};

struct max597x_data {
	struct regmap *regmap;
	int num_switches;
	int shunt_micro_ohms;
	const struct max597x_regulator *regulator;
};

struct max597x_regulator {
	unsigned int channel;
	struct regulator_desc rdesc;
};

enum max597x_regulator_id {
	MAX579X_SW0,
	MAX579X_SW1,
};

#define MAX5970_NUM_SWITCHES 2
#define MAX5978_NUM_SWITCHES 1

#define MAX5970_REG_VOLTAGE_L(ch)		(0x03 + (ch) * 4)
#define MAX5970_REG_VOLTAGE_H(ch)		(0x02 + (ch) * 4)
#define MAX5970_REG_CHx_UV_WARN_H(ch)		(0x1A + (ch) * 10)
#define MAX5970_REG_CHx_UV_WARN_L(ch)		(0x1B + (ch) * 10)
#define MAX5970_REG_CHx_UV_CRIT_H(ch)		(0x1C + (ch) * 10)
#define MAX5970_REG_CHx_UV_CRIT_L(ch)		(0x1D + (ch) * 10)
#define MAX5970_REG_CHx_OV_WARN_H(ch)		(0x1E + (ch) * 10)
#define MAX5970_REG_CHx_OV_WARN_L(ch)		(0x1F + (ch) * 10)
#define MAX5970_REG_CHx_OV_CRIT_H(ch)		(0x20 + (ch) * 10)
#define MAX5970_REG_CHx_OV_CRIT_L(ch)		(0x21 + (ch) * 10)

#define  MAX5970_REG_V_H_VAL(x)		(((x) >> 2) & 0xFF)
#define  MAX5970_REG_V_L_VAL(x)		((x) & 0x3)

#define MAX5970_REG_DAC_FAST(ch)	(0x2E + (ch))

#define MAX5970_FAST2SLOW_RATIO		200

#define MAX5970_REG_STATUS0		0x31
#define  MAX5970_CB_IFAULTF(ch)		(1 << (ch))
#define  MAX5970_CB_IFAULTS(ch)		(1 << ((ch)+4))

#define MAX5970_REG_STATUS2		0x33
#define  MAX5970_IRNG_MASK		3

#define MAX5970_REG_FAULT0		0x35
#define  UV_STATUS_WARN(ch)		(1 << (ch))
#define  UV_STATUS_CRIT(ch)		(1 << ((ch)+4))

#define MAX5970_REG_FAULT1		0x36
#define  OV_STATUS_WARN(ch)		(1 << (ch))
#define  OV_STATUS_CRIT(ch)		(1 << ((ch)+4))

#define MAX5970_REG_STATUS1		0x31
#define  STATUS1_PROT(reg)		(((reg) >> 6) & 0x3)
#define  STATUS1_PROT_SHUTDOWN		0
#define  STATUS1_PROT_CLEAR_PG		1
#define  STATUS1_PROT_ALERT_ONLY	2

#define VOLTAGE_ADJUST_CTRL_SHIFT	8
#define VOLTAGE_ADJUST_CTRL_MASK	0x3ff
#define VOLTAGE_ADJUST_CTRL_SCAL	(16000000 / VOLTAGE_ADJUST_CTRL_MASK)

static int max597x_voltage_op(struct regulator_dev *rdev)
{
	struct max597x_data *data = rdev_get_drvdata(rdev);
	const struct max597x_regulator *regulator = data->regulator;
	unsigned int val, val_h;
	int ret;

	ret = regmap_read(rdev->regmap, MAX5970_REG_VOLTAGE_L(regulator->channel), &val);
	if (ret)
		return ret;

	ret = regmap_read(rdev->regmap, MAX5970_REG_VOLTAGE_H(regulator->channel), &val_h);
	if (ret)
		return ret;

	val |= (val_h << VOLTAGE_ADJUST_CTRL_SHIFT);

	return (val & VOLTAGE_ADJUST_CTRL_MASK) * VOLTAGE_ADJUST_CTRL_SCAL;
}


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
		 	return -ENOTSUPP;
		
		return 0;
	} else {
 		if (STATUS1_PROT(reg) == STATUS1_PROT_SHUTDOWN)
		 	return -ENOTSUPP;
			
		return 0;
	}

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
		reg = lim_uV / 1000000 * 0x3FF / 16;
	else
		reg = 0;

	ret = regmap_write(rdev->regmap, off_h, MAX5970_REG_V_H_VAL(reg));
	if (ret)
		return ret;

	ret = regmap_write(rdev->regmap, off_l, MAX5970_REG_V_L_VAL(reg));
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
	int ret, val;
	int max_range;
	int vthst, vthfst;

	struct max597x_data *data = rdev_get_drvdata(rdev);
	const struct max597x_regulator *regulator = data->regulator;

	/*
	 * MAX5970 has enable control as a special value in limit reg. Can't
	 * set limit but keep feature disabled or enable W/O given limit.
	 */
	if ((lim_uA && !enable) || (!lim_uA && enable))
		return -EINVAL;

	ret = regmap_read(rdev->regmap, MAX5970_REG_STATUS2, &val);
	if (ret)
		return ret;

	switch (val & MAX5970_IRNG_MASK) {
	case 0:
		max_range = 100000; // 100 mV
		break;
	case 1:
		max_range = 50000; // 50 mV
		break;
	case 2:
		max_range = 25000; // 25 mV
		break;
	default:
		return -EINVAL;
	}

	/* Calc Vtrip threshold in uV. */
	vthst = data->shunt_micro_ohms * lim_uA / 1000000;

	/* Calc fast Vtrip threshold in uV */
	vthfst = vthst * (MAX5970_FAST2SLOW_RATIO / 100);

	if (vthfst > max_range) {
		dev_err(&rdev->dev, "Current limit out of range\n");
		return -EINVAL;
	}

	/* Program fast trip threshold */
	if (enable)
		val = 255 * vthfst / max_range;
	else
		val = 255;
	ret = regmap_write(rdev->regmap, val, MAX5970_REG_DAC_FAST(regulator->channel));

	return ret;
}

//FIXME: IRQ support

static const struct regulator_ops max597x_switch_ops = {
	.enable 			= regulator_enable_regmap,
	.disable			= regulator_disable_regmap,
	.is_enabled			= regulator_is_enabled_regmap,
	.get_voltage			= max597x_voltage_op,
	.set_over_voltage_protection	= max597x_set_ovp,
	.set_under_voltage_protection	= max597x_set_uvp,
	.set_over_current_protection	= max597x_set_ocp,
};

#define MAX597X_SWITCH(_ID, ereg, chan) {                        \
	.rdesc = {                                               \
		.name            = #_ID,                         \
		.of_match        = of_match_ptr(#_ID),           \
		.ops             = &max597x_switch_ops,          \
		.regulators_node = of_match_ptr("regulators"),   \
		.type            = REGULATOR_VOLTAGE,            \
		.id              = MAX579X_##_ID,                \
		.owner           = THIS_MODULE,                  \
		.enable_reg      = ereg,                         \
		.enable_mask     = 3 << (chan * 2),              \
	},                                                       \
	.channel = (chan),                                       \
}

static const struct max597x_regulator regulators[] = {
	MAX597X_SWITCH(SW0, 0x3b, 0),
	MAX597X_SWITCH(SW1, 0x3b, 1),
};

static int max597x_uvd_handler(int irq, struct regulator_irq_data *rid,
			      unsigned long *dev_mask)
{
	int val, ret, i;
	struct max597x_data *d = (struct max597x_data *)rid->data;

	ret = regmap_read(d->regmap, MAX5970_REG_FAULT0, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	rid->opaque = val;
	*dev_mask = 0;

	for (i = 0; i < d->num_switches; i++) {
		struct regulator_err_state *stat;

		if ((val & UV_STATUS_WARN(i)) || (val & UV_STATUS_CRIT(i)))
			*dev_mask |= 1 << i;
		
		stat  = &rid->states[i];
	
		if (val & UV_STATUS_CRIT(i)) {
			stat->notifs	= REGULATOR_EVENT_UNDER_VOLTAGE;
			stat->errors	= REGULATOR_ERROR_UNDER_VOLTAGE;
		} else if (val & UV_STATUS_WARN(i)) {
			stat->notifs	= REGULATOR_EVENT_UNDER_VOLTAGE_WARN;
			stat->errors	= REGULATOR_ERROR_UNDER_VOLTAGE_WARN;
		}
	}

	/* Clear the sub-IRQ status */
	return regmap_write(d->regmap, MAX5970_REG_FAULT0, val);
}

static int max597x_ovd_handler(int irq, struct regulator_irq_data *rid,
			      unsigned long *dev_mask)
{
	int val, ret, i;
	struct max597x_data *d = (struct max597x_data *)rid->data;

	ret = regmap_read(d->regmap, MAX5970_REG_FAULT1, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	rid->opaque = val;
	*dev_mask = 0;

	for (i = 0; i < d->num_switches; i++) {
		struct regulator_err_state *stat;

		if ((val & OV_STATUS_WARN(i)) || (val & OV_STATUS_CRIT(i)))
			*dev_mask |= 1 << i;

		stat  = &rid->states[i];
	
		if (val & OV_STATUS_CRIT(i)) {
			stat->notifs	= REGULATOR_EVENT_REGULATION_OUT;
			stat->errors	= REGULATOR_ERROR_REGULATION_OUT;
		} else if (val & OV_STATUS_WARN(i)) {
			stat->notifs	= REGULATOR_EVENT_OVER_VOLTAGE_WARN;
			stat->errors	= REGULATOR_ERROR_OVER_VOLTAGE_WARN;
		}
	}

	/* Clear the sub-IRQ status */
	return regmap_write(d->regmap, MAX5970_REG_FAULT1, val);
}

static int max597x_ovc_handler(int irq, struct regulator_irq_data *rid,
			       unsigned long *dev_mask)
{
	int val, ret, i;
	struct max597x_data *d = (struct max597x_data *)rid->data;

	ret = regmap_read(d->regmap, MAX5970_REG_STATUS0, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	rid->opaque = val;
	*dev_mask = 0;

	for (i = 0; i < d->num_switches; i++) {
		struct regulator_err_state *stat;

		stat  = &rid->states[i];

		if ((val & MAX5970_CB_IFAULTF(i)) || (val & MAX5970_CB_IFAULTS(i))) {
			*dev_mask |= 1 << i;
			stat->notifs = REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_DISABLE;
			stat->errors = REGULATOR_ERROR_OVER_CURRENT | REGULATOR_ERROR_FAIL;
		}
	}

	/* Clear the sub-IRQ status */
	return regmap_write(d->regmap, MAX5970_REG_STATUS0, 0);
}

static int max597x_probe(struct platform_device *pdev)
{
	unsigned int i;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regulator_dev *rdevs[MAX5970_NUM_SWITCHES];
	struct max597x_data *data;
	int uvd_errs = REGULATOR_ERROR_UNDER_VOLTAGE |
			REGULATOR_ERROR_UNDER_VOLTAGE_WARN;
	int ovd_errs = REGULATOR_ERROR_OVER_VOLTAGE_WARN |
			REGULATOR_ERROR_REGULATION_OUT;
	int ovc_errs = REGULATOR_ERROR_OVER_CURRENT |
			REGULATOR_ERROR_FAIL;
	void *ret;
	int irq;
	int num_switches;
	struct regmap *regmap;

	struct regulator_irq_desc max597x_notif_uvd = {
		.name = "max597x-uvd",
		.irq_off_ms = 1000,
		.map_event = max597x_uvd_handler,
	};
	struct regulator_irq_desc max597x_notif_ovd = {
		.name = "max597x-ovd",
		.irq_off_ms = 1000,
		.map_event = max597x_ovd_handler,
	};
	struct regulator_irq_desc max597x_notif_ovc = {
		.name = "max597x-ovc",
		.irq_off_ms = 1000,
		.map_event = max597x_ovc_handler,
	};
	enum max597x_chip_type chip = platform_get_device_id(pdev)->driver_data;
	switch (chip) {
	case MAX597x_TYPE_MAX5970:
		num_switches = 2;
		break;
	case MAX597x_TYPE_MAX5978:
		num_switches= 1;
		break;
	}

	if (pdev->dev.of_node) {
		if (device_property_read_u32(&pdev->dev, "shunt-resistor-micro-ohms",
		    &data->shunt_micro_ohms) < 0)
			dev_warn(&pdev->dev, "devicetree property shunt-resistor-micro-ohms not found\n");
	} else
		dev_warn(&pdev->dev, "No devicetree node associated\n");

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		dev_err(&pdev->dev, "No regmap\n");
		return -EINVAL;
	}
	
	for (i = 0; i < num_switches; i++) {
		data = devm_kzalloc(&pdev->dev, sizeof(struct max597x_data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		data->regmap = regmap;
		data->num_switches = num_switches;
		data->regulator = &regulators[i];
	
		max597x_notif_uvd.data = data;
		max597x_notif_ovd.data = data;
		max597x_notif_ovc.data = data;

		config.dev = pdev->dev.parent;
		config.driver_data = (void *)data;
		rdev = devm_regulator_register(&pdev->dev,
					&regulators[i].rdesc,
					&config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "failed to register regulator %s\n",
				regulators[i].rdesc.name);
			return PTR_ERR(rdev);
		}
		rdevs[i] = rdev;
	}

	irq = platform_get_irq_byname(pdev, "max597x-uvd");
	if (irq >= 0) {
		/* Register notifiers - can fail if IRQ is not given */
		ret = devm_regulator_irq_helper(&pdev->dev, &max597x_notif_uvd,
						irq, 0, uvd_errs, NULL,
						&rdevs[0],
						data->num_switches);
		if (IS_ERR(ret)) {
			if (PTR_ERR(ret) == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			dev_warn(&pdev->dev, "UVD disabled %pe\n", ret);
		}
	}

	irq = platform_get_irq_byname(pdev, "max597x-ovd");
	if (irq >= 0) {
		ret = devm_regulator_irq_helper(&pdev->dev, &max597x_notif_ovd,
						irq, 0, ovd_errs, NULL,
						&rdevs[0],
						data->num_switches);
		if (IS_ERR(ret)) {
			if (PTR_ERR(ret) == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			dev_warn(&pdev->dev, "OVD disabled %pe\n", ret);
		}
	}

	irq = platform_get_irq_byname(pdev, "max597x-ovc");
	if ((irq >= 0) && (data->shunt_micro_ohms > 0)) {

		ret = devm_regulator_irq_helper(&pdev->dev, &max597x_notif_ovc,
						irq, 0, ovc_errs, NULL,
						&rdevs[0],
						data->num_switches);
		if (IS_ERR(ret)) {
			if (PTR_ERR(ret) == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			dev_warn(&pdev->dev, "OVC disabled %pe\n", ret);
		}
	}
	
	return 0;
}

static const struct platform_device_id max597x_table[] = {
	{ .name = "max5970", MAX597x_TYPE_MAX5970 },
	{ .name = "max5978", MAX597x_TYPE_MAX5978 },
	{},
};
MODULE_DEVICE_TABLE(platform, max597x_table);

static struct platform_driver max597x_driver = {
	.id_table = max597x_table,
	.driver = {
		.name	= "max597x",
	},
	.probe	= max597x_probe,
};
module_platform_driver(max597x_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("Max5970_hot-swap controller driver");
MODULE_LICENSE("GPL v2");
