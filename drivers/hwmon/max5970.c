// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for regulators in MAX5970 and MAX5978 IC
 *
 * Copyright (c) 2022 9elements GmbH
 *
 * Author: Patrick Rudolph <patrick.rudolph@9elements.com>
 */

#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/mfd/max5970.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct max5970_hwmon {
	int num_switches, irng[MAX5970_NUM_SWITCHES], mon_rng[MAX5970_NUM_SWITCHES];
	struct regmap *regmap;
};

static int max5970_read_adc(struct regmap *regmap, int reg, long *val)
{
	u8 reg_data[2];
	int ret;

	ret = regmap_bulk_read(regmap, reg, &reg_data[0], 2);
	if (ret < 0)
		return ret;

	*val = (reg_data[0] << 2) | (reg_data[1] & 3);

	return 0;
}

static int max5970_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct max5970_hwmon *ddata = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			/* Read the 10bit current ADC for the channel */
			ret = max5970_read_adc(ddata->regmap, MAX5970_REG_CURRENT_H(channel), val);
			/* Multiply with IMON range & then divid by 0x3ff */
			*val *= ddata->irng[channel];
			*val /= (1 << 10)-1;
			/* Convert the voltage meansurement across shunt resistor to current */
			//*val /= <shunt resistor value here>
			return ret;
		default:
			return -EOPNOTSUPP;
		}

	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			ret = max5970_read_adc(ddata->regmap, MAX5970_REG_VOLTAGE_H(channel), val);
			/* Multiply with MON range & then divid by 0x3ff */
			*val *= ddata->mon_rng[channel];
			*val /= ADC_MASK;

			return ret;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max5970_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	struct max5970_hwmon *ddata = (struct max5970_hwmon *)data;

	if (channel >= ddata->num_switches)
		return 0;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			return 0444;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_ops max5970_hwmon_ops = {
	.is_visible = max5970_is_visible,
	.read = max5970_read,
};

static const struct hwmon_channel_info *max5970_info[] = {
	HWMON_CHANNEL_INFO(in, HWMON_I_INPUT, HWMON_I_INPUT),
	HWMON_CHANNEL_INFO(curr, HWMON_C_INPUT, HWMON_C_INPUT),
	NULL
};

static const struct hwmon_chip_info max5970_chip_info = {
	.ops = &max5970_hwmon_ops,
	.info = max5970_info,
};

static int max5970_adc_range(struct regmap *regmap, const int ch,
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
		*irng = 100000;	/* 100 mV */
		break;
	case 1:
		*irng = 50000;	/* 50 mV */
		break;
	case 2:
		*irng = 25000;	/* 25 mV */
		break;
	default:
		return -EINVAL;
	}

	/* Decode current voltage monitor range */
	ret = regmap_read(regmap, MAX5970_REG_MON_RANGE, &reg);
	if (ret)
		return ret;

	*mon_rng = MAX5970_MON_MAX_RANGE_UV >> MAX5970_MON(reg, ch);
	*mon_rng /= 1000; /* uV to mV */

	return 0;
}

static int max5970_sensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *i2c = to_i2c_client(pdev->dev.parent);
	struct max5970_hwmon *ddata;
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct device *hwmon_dev;
	int err;

	if (!regmap)
		return -EPROBE_DEFER;

	ddata = devm_kzalloc(dev, sizeof(struct max5970_hwmon), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	if (of_device_is_compatible(i2c->dev.of_node, "maxim,max5978"))
		ddata->num_switches = TYPE_MAX5978;
	else if (of_device_is_compatible(i2c->dev.of_node, "maxim,max5970"))
		ddata->num_switches = TYPE_MAX5970;
	else
		return -ENODEV;

	ddata->regmap = regmap;

	for (int i = 0; i < ddata->num_switches; i++) {
		err = max5970_adc_range(regmap, i, &ddata->irng[i], &ddata->mon_rng[i]);
		if (err < 0)
			return err;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev,
							 "max5970_hwmon", ddata,
							 &max5970_chip_info, NULL);

	if (IS_ERR(hwmon_dev)) {
		err = PTR_ERR(hwmon_dev);
		dev_err_probe(dev, err, "Unable to register hwmon device\n");
		return err;
	}

	return 0;
}

static struct platform_driver max5970_sensor_driver = {
	.probe = max5970_sensor_probe,
	.driver = {
		.name = "max5970-hwmon",
	},
};
module_platform_driver(max5970_sensor_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("MAX5970_hot-swap controller driver");
MODULE_LICENSE("GPL v2");
