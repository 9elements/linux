// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Infineon bcm6123
 *
 * Copyright (c) 2021 9elements GmbH
 *
 * VOUT_MODE is not supported by the device. The driver fakes VOUT linear16
 * mode with exponent value -8 as direct mode with m=256/b=0/R=0.
 *
 * The device supports VOUT_PEAK, IOUT_PEAK, and TEMPERATURE_PEAK, however
 * this driver does not currently support them.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pmbus.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include "pmbus.h"

static struct pmbus_platform_data bcm6123_plat_data = {
	.flags = PMBUS_NO_CAPABILITY,
};

static struct pmbus_driver_info bcm6123_info = {
	.pages = 2,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_IN] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.m[PSC_VOLTAGE_IN] = 1,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = 1,
	.m[PSC_VOLTAGE_OUT] = 1,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = 1,
	.m[PSC_CURRENT_IN] = 1,
	.b[PSC_CURRENT_IN] = 0,
	.R[PSC_CURRENT_IN] = 3,
	.m[PSC_CURRENT_OUT] = 1,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = 2,
	.func[0] = 0, /* Summing page without voltage readings */
	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT
	    | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP
	    | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT
	    | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT
	    | PMBUS_HAVE_IIN | PMBUS_HAVE_POUT,
};

static int bcm6123_probe(struct i2c_client *client)
{
#if IS_ENABLED(CONFIG_SENSORS_BCM6123_REGULATOR)
	struct regulator_init_data *init_data;
	struct device *dev = &client->dev;
	struct device_node *np;
	struct of_regulator_match rmatch[1];
	struct regulator_desc *rdesc;
	int matched;
	const struct regulator_desc bcm6123_reg_desc = {
		.name = "vout0",
		.id = 1,
		.of_match = of_match_ptr("vout0"),
		.regulators_node = of_match_ptr("regulators"),
		.ops = &pmbus_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	};

	np = of_get_child_by_name(dev->of_node, "regulators");
	if (!np) {
		dev_err(dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	rmatch[0].name = bcm6123_reg_desc.of_match;
	matched = of_regulator_match(dev, np, rmatch, ARRAY_SIZE(rmatch));
	of_node_put(np);

	if (matched <= 0) {
		dev_err(dev, "could not find %s sub-node\n",
			bcm6123_reg_desc.of_match);
		return matched;
	}

	init_data = rmatch[0].init_data;
	if (!init_data->constraints.min_uV ||
	    init_data->constraints.min_uV != init_data->constraints.max_uV) {
		dev_err(dev, "only fixed voltage regulators are supported\n");
		return -EINVAL;
	}

	rdesc = devm_kzalloc(dev, sizeof(*rdesc), GFP_KERNEL);
	if (!rdesc)
		return -ENOMEM;

	memcpy(rdesc, &bcm6123_reg_desc, sizeof(bcm6123_reg_desc));

	rdesc->fixed_uV = init_data->constraints.min_uV;
	rdesc->n_voltages = 1;

	bcm6123_info.reg_desc = rdesc;
	bcm6123_info.num_regulators = 1;
#endif

	client->dev.platform_data = &bcm6123_plat_data;

	return pmbus_do_probe(client, &bcm6123_info);
}

static const struct i2c_device_id bcm6123_id[] = {
	{"bcm6123", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bcm6123_id);

#ifdef CONFIG_OF
static const struct of_device_id bcm6123_of_match[] = {
	{ .compatible = "vicor,bcm6123" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm6123_of_match);
#endif

/* This is the driver that will be inserted */
static struct i2c_driver bcm6123_driver = {
	.driver = {
		   .name = "bcm6123",
		   .of_match_table = of_match_ptr(bcm6123_of_match),
		   },
	.probe_new = bcm6123_probe,
	.id_table = bcm6123_id,
};

module_i2c_driver(bcm6123_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("PMBus driver for Vicor bcm6123");
MODULE_LICENSE("GPL");
