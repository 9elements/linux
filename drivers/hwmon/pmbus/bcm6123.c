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
