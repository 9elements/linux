// SPDX-License-Identifier: GPL-2.0
//
// Device driver for regulators in MAX5970 IC
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

struct max5970_regulator {
	unsigned int channel;
	struct regulator_desc rdesc;
};

enum max5970_regulator_id {
	MAX5790_SW0,
	MAX5790_SW1,
};

#define MAX5970_NUM_SWITCHES 2

#define MAX5970_REG_VOLTAGE_L(x)	(3 + 4 * (x))
#define MAX5970_REG_VOLTAGE_H(x)	(2 + 4 * (x))
#define MAX5970_REG_UV_WARN_H		0x1A
#define MAX5970_REG_UV_WARN_L		0x1B
#define MAX5970_REG_UV_CRIT_H		0x1C
#define MAX5970_REG_UV_CRIT_L		0x1D
#define MAX5970_REG_OV_WARN_H		0x1E
#define MAX5970_REG_OV_WARN_L		0x1F
#define MAX5970_REG_OV_CRIT_H		0x20
#define MAX5970_REG_OV_CRIT_L		0x21
#define  MAX5970_REG_V_H_VAL(x)		(((x) >> 2) & 0xFF)
#define  MAX5970_REG_V_L_VAL(x)		((x) & 0x3)

#define MAX5970_REG_UV_STATUS		0x35
#define MAX5970_REG_OV_STATUS		0x36
#define  UV_STATUS_WARN(x)		(((x) >> 0) & 0x3)
#define  UV_STATUS_CRIT(x)		(((x) >> 4) & 0x3)

#define MAX5970_REG_STATUS1		0x31
#define  STATUS1_PROT(x)		(((x) >> 6) & 0x3)
#define  STATUS1_PROT_SHUTDOWN		0
#define  STATUS1_PROT_CLEAR_PG		1
#define  STATUS1_PROT_ALERT_ONLY	2

#define VOLTAGE_ADJUST_CTRL_SHIFT	8
#define VOLTAGE_ADJUST_CTRL_MASK	0x3ff
#define VOLTAGE_ADJUST_CTRL_SCAL	(16000000 / VOLTAGE_ADJUST_CTRL_MASK)

static int max5970_voltage_op(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val, val_h;
	const struct max5970_regulator *regulator = rdev_get_drvdata(rdev);

	ret = regmap_read(rdev->regmap, MAX5970_REG_VOLTAGE_L(regulator->channel), &val);
	if (ret)
		return ret;

	ret = regmap_read(rdev->regmap, MAX5970_REG_VOLTAGE_H(regulator->channel), &val_h);
	if (ret)
		return ret;

	val |= (val_h << VOLTAGE_ADJUST_CTRL_SHIFT);

	return (val & VOLTAGE_ADJUST_CTRL_MASK) * VOLTAGE_ADJUST_CTRL_SCAL;
}


static int max5970_uvp_ovp_check_mode(struct regulator_dev *rdev, int severity)
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

static int max5970_set_vp(struct regulator_dev *rdev, int lim_uV, int severity,
			  bool enable, bool overvoltage)
{
	int off_h, off_l, reg, ret;

	if (overvoltage) {
		if (severity == REGULATOR_SEVERITY_WARN) {
			off_h = MAX5970_REG_OV_WARN_H;
			off_l = MAX5970_REG_OV_WARN_L;
		} else {
			off_h = MAX5970_REG_OV_CRIT_H;
			off_l = MAX5970_REG_OV_CRIT_L;
		}
	} else {
		if (severity == REGULATOR_SEVERITY_WARN) {
			off_h = MAX5970_REG_UV_WARN_H;
			off_l = MAX5970_REG_UV_WARN_L;
		} else {
			off_h = MAX5970_REG_UV_CRIT_H;
			off_l = MAX5970_REG_UV_CRIT_L;
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

static int max5970_set_uvp(struct regulator_dev *rdev, int lim_uV, int severity,
			  bool enable)
{
	int ret;

	/*
	 * MAX5970 has enable control as a special value in limit reg. Can't
	 * set limit but keep feature disabled or enable W/O given limit.
	 */
	if ((lim_uV && !enable) || (!lim_uV && enable))
		return -EINVAL;

	ret = max5970_uvp_ovp_check_mode(rdev, severity);
	if (ret)
		return ret;

	return max5970_set_vp(rdev, lim_uV, severity, enable, false);
}

static int max5970_set_ovp(struct regulator_dev *rdev, int lim_uV, int severity,
			  bool enable)
{
	int ret;

	/*
	 * MAX5970 has enable control as a special value in limit reg. Can't
	 * set limit but keep feature disabled or enable W/O given limit.
	 */
	if ((lim_uV && !enable) || (!lim_uV && enable))
		return -EINVAL;

	ret = max5970_uvp_ovp_check_mode(rdev, severity);
	if (ret)
		return ret;

	return max5970_set_vp(rdev, lim_uV, severity, enable, true);
}

//FIXME: IRQ support

static const struct regulator_ops max5970_switch_ops = {
	.enable 			= regulator_enable_regmap,
	.disable			= regulator_disable_regmap,
	.is_enabled			= regulator_is_enabled_regmap,
	.get_voltage			= max5970_voltage_op,
	.set_over_voltage_protection	= max5970_set_ovp,
	.set_under_voltage_protection	= max5970_set_uvp,
};

#define MAX5970_SWITCH(_ID, ereg, chan) {                        \
	.rdesc = {                                               \
		.name            = #_ID,                         \
		.of_match        = of_match_ptr(#_ID),           \
		.ops             = &max5970_switch_ops,          \
		.regulators_node = of_match_ptr("regulators"),   \
		.type            = REGULATOR_VOLTAGE,            \
		.id              = MAX5790_##_ID,                \
		.owner           = THIS_MODULE,                  \
		.enable_reg      = ereg,                         \
		.enable_mask     = 3 << (chan * 2),              \
	},                                                       \
	.channel = (chan),                                       \
}

static const struct max5970_regulator regulators[] = {
	MAX5970_SWITCH(SW0, 0x3b, 0),
	MAX5970_SWITCH(SW1, 0x3b, 1),
};

static int max5970_uvd_handler(int irq, struct regulator_irq_data *rid,
			      unsigned long *dev_mask)
{
	int val, ret, i;
	struct regmap *regmap = (struct regmap *)rid->data;

	ret = regmap_read(regmap, MAX5970_REG_UV_STATUS, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	rid->opaque = val;
	*dev_mask = 0;

	if (!UV_STATUS_WARN(val) && !UV_STATUS_CRIT(val))
		return 0;

	*dev_mask = UV_STATUS_WARN(val) | UV_STATUS_CRIT(val);

	for (i = 0; i < MAX5970_NUM_SWITCHES; i++) {
		struct regulator_err_state *stat;

		stat  = &rid->states[i];
	
		if (UV_STATUS_CRIT(val) & BIT(i)) {
			stat->notifs	= REGULATOR_EVENT_UNDER_VOLTAGE;
			stat->errors	= REGULATOR_ERROR_UNDER_VOLTAGE;
		} else if (UV_STATUS_WARN(val) & BIT(i)) {
			stat->notifs	= REGULATOR_EVENT_UNDER_VOLTAGE_WARN;
			stat->errors	= REGULATOR_ERROR_UNDER_VOLTAGE_WARN;
		}
	}

	/* Clear the sub-IRQ status */
	return regmap_write(regmap, MAX5970_REG_UV_STATUS, 0);
}

static int max5970_ovd_handler(int irq, struct regulator_irq_data *rid,
			      unsigned long *dev_mask)
{
	int val, ret, i;
	struct regmap *regmap = (struct regmap *)rid->data;

	ret = regmap_read(regmap, MAX5970_REG_OV_STATUS, &val);
	if (ret)
		return REGULATOR_FAILED_RETRY;

	rid->opaque = val;
	*dev_mask = 0;

	if (!UV_STATUS_WARN(val) && !UV_STATUS_CRIT(val))
		return 0;

	*dev_mask = UV_STATUS_WARN(val) | UV_STATUS_CRIT(val);

	for (i = 0; i < MAX5970_NUM_SWITCHES; i++) {
		struct regulator_err_state *stat;

		stat  = &rid->states[i];
	
		if (UV_STATUS_CRIT(val) & BIT(i)) {
			stat->notifs	= REGULATOR_EVENT_REGULATION_OUT;
			stat->errors	= REGULATOR_ERROR_REGULATION_OUT;
		} else if (UV_STATUS_WARN(val) & BIT(i)) {
			stat->notifs	= REGULATOR_EVENT_OVER_VOLTAGE_WARN;
			stat->errors	= REGULATOR_ERROR_OVER_VOLTAGE_WARN;
		}
	}

	/* Clear the sub-IRQ status */
	return regmap_write(regmap, MAX5970_REG_OV_STATUS, 0);
}


static int max5970_probe(struct platform_device *pdev)
{
	unsigned int i;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regulator_dev *rdevs[MAX5970_NUM_SWITCHES];
	struct regmap *regmap;
	int uvd_errs = REGULATOR_ERROR_UNDER_VOLTAGE |
			REGULATOR_ERROR_UNDER_VOLTAGE_WARN;
	int ovd_errs = REGULATOR_ERROR_OVER_VOLTAGE_WARN |
			REGULATOR_ERROR_REGULATION_OUT;
	void *ret;
	int irq;
	struct regulator_irq_desc max5970_notif_uvd = {
		.name = "max5970-uvd",
		.irq_off_ms = 1000,
		.map_event = max5970_uvd_handler,
	};
	struct regulator_irq_desc max5970_notif_ovd = {
		.name = "max5970-ovd",
		.irq_off_ms = 1000,
		.map_event = max5970_ovd_handler,
	};

	config.dev = pdev->dev.parent;
	
	for (i = 0; i < ARRAY_SIZE(regulators); i++) {
		config.driver_data = (void *) &regulators[i];

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

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		dev_err(&pdev->dev, "No regmap\n");
		return -EINVAL;
	}
	max5970_notif_uvd.data = regmap;
	max5970_notif_ovd.data = regmap;

	irq = platform_get_irq_byname(pdev, "max5970-uvd");

	/* Register notifiers - can fail if IRQ is not given */
	ret = devm_regulator_irq_helper(&pdev->dev, &max5970_notif_uvd,
					irq, 0, uvd_errs, NULL,
					&rdevs[0],
					MAX5970_NUM_SWITCHES);
	if (IS_ERR(ret)) {
		if (PTR_ERR(ret) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_warn(&pdev->dev, "UVD disabled %pe\n", ret);
	}

	irq = platform_get_irq_byname(pdev, "max5970-ovd");

	ret = devm_regulator_irq_helper(&pdev->dev, &max5970_notif_ovd,
					irq, 0, ovd_errs, NULL,
					&rdevs[0],
					MAX5970_NUM_SWITCHES);
	if (IS_ERR(ret)) {
		if (PTR_ERR(ret) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_warn(&pdev->dev, "OVD disabled %pe\n", ret);
	}
	
	return 0;
}

static const struct platform_device_id max5970_table[] = {
	{ .name = "max5970" },
	{},
};
MODULE_DEVICE_TABLE(platform, max5970_table);

static struct platform_driver max5970_driver = {
	.id_table = max5970_table,
	.driver = {
		.name	= "max5970",
	},
	.probe	= max5970_probe,
};
module_platform_driver(max5970_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("Max5970_hot-swap controller driver");
MODULE_LICENSE("GPL v2");
