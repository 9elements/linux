// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Vicor PLI1209BC Digital Supervisor
 *
 * Copyright (c) 2022 9elements GmbH
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pmbus.h>
#include "pmbus.h"

/*
 * The capability command is only supported at page 0. Probing the device while
 * the page register is set to 1 will falsely enable PEC support. Disable
 * capability probing accordingly, since the PLI1209BC does not have any
 * additional capabilities.
 */
static struct pmbus_platform_data pli1209bc_plat_data = {
	.flags = PMBUS_NO_CAPABILITY,
};

static struct pmbus_driver_info pli1209bc_info = {
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
	.func[0] = PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT
	    | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT
	    | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP
	    | PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT,
	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT
	    | PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT
	    | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT
	    | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP
	    | PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT
};

static int pli1209bc_probe(struct i2c_client *client)
{
	client->dev.platform_data = &pli1209bc_plat_data;
	return pmbus_do_probe(client, &pli1209bc_info);
}

static const struct i2c_device_id pli1209bc_id[] = {
	{"pli1209bc", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, pli1209bc_id);

#ifdef CONFIG_OF
static const struct of_device_id pli1209bc_of_match[] = {
	{ .compatible = "vicor,pli1209bc" },
	{ },
};
MODULE_DEVICE_TABLE(of, pli1209bc_of_match);
#endif

/* This is the driver that will be inserted */
static struct i2c_driver pli1209bc_driver = {
	.driver = {
		   .name = "pli1209bc",
		   .of_match_table = of_match_ptr(pli1209bc_of_match),
		   },
	.probe_new = pli1209bc_probe,
	.id_table = pli1209bc_id,
};

module_i2c_driver(pli1209bc_driver);

MODULE_AUTHOR("Marcello Sylvester Bauer <sylv@sylv.io>");
MODULE_DESCRIPTION("PMBus driver for Vicor PLI1209BC");
MODULE_LICENSE("GPL");
