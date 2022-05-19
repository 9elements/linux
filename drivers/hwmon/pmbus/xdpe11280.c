// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for Infineon Multi-phase Digital VR Controllers
 *
 * Copyright (c) 2020 Mellanox Technologies. All rights reserved.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>

#include "pmbus.h"

#if IS_ENABLED(CONFIG_SENSORS_XDPE112_REGULATOR)
#define XDPE112_REGULATOR(_name, _id)				\
	[_id] = {						\
		.name = (_name # _id),				\
		.id = (_id),					\
		.of_match = of_match_ptr(_name # _id),		\
		.regulators_node = of_match_ptr("regulators"),	\
		.ops = &pmbus_regulator_ops,			\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
		.supply_name = "vdrv",				\
	}

static struct regulator_desc xdpe112_reg_desc[] = {
	XDPE112_REGULATOR("vout", 0),
	XDPE112_REGULATOR("vout", 1),
};
#endif /* CONFIG_SENSORS_XDPE112_REGULATOR */

static struct pmbus_driver_info xdpe112_info = {
	.pages = 2,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP |
		PMBUS_HAVE_POUT | PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP |
		PMBUS_HAVE_POUT | PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
#if IS_ENABLED(CONFIG_SENSORS_XDPE112_REGULATOR)
	.num_regulators = 2,
	.reg_desc = xdpe112_reg_desc,
#endif
};

static int xdpe112_probe(struct i2c_client *client)
{
	if (client->dev.of_node->full_name)
		strlcpy(client->name, client->dev.of_node->full_name, I2C_NAME_SIZE);

	return pmbus_do_probe(client, &xdpe112_info);
}

static const struct i2c_device_id xdpe112_id[] = {
	{"xdpe11280", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, xdpe112_id);

static const struct of_device_id __maybe_unused xdpe112_of_match[] = {
	{.compatible = "infineon,xdpe11280"},
	{}
};
MODULE_DEVICE_TABLE(of, xdpe112_of_match);

static struct i2c_driver xdpe112_driver = {
	.driver = {
		.name = "xdpe11280",
		.of_match_table = of_match_ptr(xdpe112_of_match),
	},
	.probe_new = xdpe112_probe,
	.id_table = xdpe112_id,
};

module_i2c_driver(xdpe112_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("PMBus driver for Infineon XDPE112 family");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
