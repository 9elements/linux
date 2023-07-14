// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Infineon TDA38640
 *
 * Copyright (c) 2022 9elements GmbH
 *
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include "pmbus.h"

#if IS_ENABLED(CONFIG_SENSORS_TDA38640_REGULATOR)
static const struct regulator_desc tda38640_reg_desc[] = {
	PMBUS_REGULATOR("vout", 0),
};
#endif /* CONFIG_SENSORS_TDA38640_REGULATOR */

static int tda38640_read_byte_data(struct i2c_client *client, int page, int reg)
{
	int ret, on_off_config;

	if (reg != PMBUS_OPERATION)
		return -ENODATA;

	ret = pmbus_read_byte_data(client, page, reg);
	if (ret < 0)
		return ret;

	ret &= ~PB_OPERATION_CONTROL_ON;

	on_off_config = pmbus_read_byte_data(client, page,
					     PMBUS_ON_OFF_CONFIG);
	if (on_off_config < 0)
		return on_off_config;

	if (on_off_config & PB_ON_OFF_CONFIG_EN_PIN_REQ)
		ret |= PB_OPERATION_CONTROL_ON;

	return ret;
}

static int tda38640_write_byte_data(struct i2c_client *client, int page,
				    int reg, u8 byte)
{
	int enable, ret;

	if (reg != PMBUS_OPERATION)
		return -ENODATA;

	enable = byte & PB_OPERATION_CONTROL_ON;

	byte &= ~PB_OPERATION_CONTROL_ON;
	ret = pmbus_write_byte_data(client, page, reg, byte);
	if (ret < 0)
		return ret;

	return pmbus_update_byte_data(client, page, PMBUS_ON_OFF_CONFIG,
				      PB_ON_OFF_CONFIG_EN_PIN_REQ,
				      enable ? PB_ON_OFF_CONFIG_EN_PIN_REQ : 0);
}

static struct pmbus_driver_info tda38640_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_POWER] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT
	    | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP
#if IS_ENABLED(CONFIG_SENSORS_TDA38640_REGULATOR)
           | PMBUS_HAVE_PGOOD
#endif
	    | PMBUS_HAVE_IIN
	    | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT
	    | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT
	    | PMBUS_HAVE_POUT | PMBUS_HAVE_PIN,
#if IS_ENABLED(CONFIG_SENSORS_TDA38640_REGULATOR)
	.num_regulators = 1,
	.reg_desc = tda38640_reg_desc,
#endif
};

static int tda38640_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev_of_node(dev);
	struct pmbus_driver_info *info;
	u32 en_pin_lvl;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	memcpy(info, &tda38640_info, sizeof(*info));

	if (!CONFIG_SENSORS_TDA38640_REGULATOR || !np ||
	    of_property_read_u32(np, "infineon,en-pin-fixed-level", &en_pin_lvl))
		return pmbus_do_probe(client, info);

	/*
	 * Apply ON_OFF_CONFIG workaround as enabling the regulator using the
	 * OPERATION register doesn't work in SVID mode.
	 */
	info->read_byte_data = tda38640_read_byte_data;
	info->write_byte_data = tda38640_write_byte_data;

	ret = i2c_smbus_read_byte_data(client, PMBUS_ON_OFF_CONFIG);
	if (ret < 0)
		return ret;

	ret &= ~(PB_ON_OFF_CONFIG_POWERUP_ANYTIME |
		PB_ON_OFF_CONFIG_OPERATION_REQ |
		PB_ON_OFF_CONFIG_POLARITY_HIGH);

	if (en_pin_lvl)
		ret |= PB_ON_OFF_CONFIG_POLARITY_HIGH;

	ret = i2c_smbus_write_byte_data(client, PMBUS_ON_OFF_CONFIG, ret);
	if (ret < 0)
		return ret;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id tda38640_id[] = {
	{"tda38640", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tda38640_id);

#ifdef CONFIG_OF
static const struct of_device_id tda38640_of_match[] = {
	{ .compatible = "infineon,tda38640"},
	{ },
};
MODULE_DEVICE_TABLE(of, tda38640_of_match);
#endif

/* This is the driver that will be inserted */
static struct i2c_driver tda38640_driver = {
	.driver = {
		   .name = "tda38640",
		   },
	.probe_new = tda38640_probe,
	.id_table = tda38640_id,
};

module_i2c_driver(tda38640_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("PMBus driver for Infineon TDA38640");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
