// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim MAX735x I2C mux driver
 *
 * Copyright (c) 2021 Patrick Rudolph <patrick.rudolph@9elements.com>
 *
 * This module supports the MAX735x series of I2C multiplexer/switch
 * chips made by Maxim.
 * This includes the:
 *	 MAX7356, MAX7357, MAX7358.
 *
 * These chips are all controlled via the I2C bus itself, and all have a
 * single 8-bit register. The upstream "parent" bus fans out eight
 * downstream busses or channels; which of these are selected is
 * determined by the chip type and register contents.
 *
 * None of the MAX7357 and MAX7358 extended functionality had been
 * implemented.
 *
 * Based on: i2c-mux-ltc4306.C
 *
 * Datasheet: http://cds.linear.com/docs/en/datasheet/4306.pdf
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c-mux.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define MAX735X_NCHANS 8
#define MAX735x_REG_SWITCH 0

enum max_type {
	max_7356,
	max_7357,
	max_7358,
};

struct max735x {
	struct regmap *regmap;
	struct gpio_chip gpiochip;
	const struct chip_desc *chip;
};

static const struct i2c_device_id max735x_id[] = {
	{ "max7356", max_7356 },
	{ "max7357", max_7357 },
	{ "max7358", max_7358 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max735x_id);

static const struct of_device_id max735x_of_match[] = {
	{ .compatible = "maxim,max7356" },
	{ .compatible = "maxim,max7357" },
	{ .compatible = "maxim,max7358" },
	{}
};
MODULE_DEVICE_TABLE(of, max735x_of_match);

static const struct regmap_config max735x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 1,
	.cache_type = REGCACHE_FLAT,
};

static int max735x_select_mux(struct i2c_mux_core *muxc, u32 chan)
{
	struct max735x *data = i2c_mux_priv(muxc);
	u8 bit = BIT(chan);

	if (chan >= MAX735X_NCHANS)
		return -EINVAL;

	return regmap_write(data->regmap, MAX735x_REG_SWITCH, bit);
}

static int max735x_deselect_mux(struct i2c_mux_core *muxc, u32 chan)
{
	struct max735x *data = i2c_mux_priv(muxc);

	return regmap_write(data->regmap, MAX735x_REG_SWITCH, 0);
}

static int max735x_probe(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_mux_core *muxc;
	struct max735x *data;
	struct gpio_desc *gpio;
	bool idle_disc;
	int num, ret;

	idle_disc = device_property_read_bool(&client->dev,
					      "i2c-mux-idle-disconnect");

	muxc = i2c_mux_alloc(adap, &client->dev,
			     MAX735X_NCHANS, sizeof(*data),
			     I2C_MUX_LOCKED, max735x_select_mux,
			     idle_disc ? max735x_deselect_mux : NULL);
	if (!muxc)
		return -ENOMEM;
	data = i2c_mux_priv(muxc);

	i2c_set_clientdata(client, muxc);

	data->regmap = devm_regmap_init_i2c(client, &max735x_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	/* Reset and enable the mux if an enable GPIO is specified. */
	gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	if (gpio) {
		udelay(1);
		gpiod_set_value(gpio, 1);
	}

	/*
	 * Write the mux register at addr to verify
	 * that the mux is in fact present. This also
	 * initializes the mux to disconnected state.
	 */
	if (regmap_write(data->regmap, MAX735x_REG_SWITCH, 0) < 0) {
		dev_warn(&client->dev, "probe failed\n");
		return -ENODEV;
	}

	/* Now create an adapter for each channel */
	for (num = 0; num < MAX735X_NCHANS; num++) {
		ret = i2c_mux_add_adapter(muxc, 0, num, 0);
		if (ret) {
			i2c_mux_del_adapters(muxc);
			return ret;
		}
	}

	dev_info(&client->dev,
		 "registered %d multiplexed busses for I2C switch %s\n",
		 num, client->name);

	return 0;
}

static int max735x_remove(struct i2c_client *client)
{
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);

	i2c_mux_del_adapters(muxc);

	return 0;
}

static struct i2c_driver max735x_driver = {
	.driver		= {
		.name	= "max735x",
		.of_match_table = of_match_ptr(max735x_of_match),
	},
	.probe_new	= max735x_probe,
	.remove		= max735x_remove,
	.id_table	= max735x_id,
};

module_i2c_driver(max735x_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("Maxim MAX735x I2C mux/switch driver");
MODULE_LICENSE("GPL v2");
