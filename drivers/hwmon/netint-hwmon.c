// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Netint Quadra T1,T1A,T2A pcie cards smbus temperature sensor driver
 *
 *   Author: Alexander Hansen <alexander.hansen@9elements.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

// Register Definitions
#define REG_TEMP 0x3
#define REG_VENDOR 0x9 //2 bytes
#define REG_LAST 63

static bool netint_regmap_is_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static umode_t netint_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr, int channel){

	if(type != hwmon_temp)
		return 0;

	switch(attr){
	case hwmon_temp_input:
		return 0444;
	default:
		return 0;
	}
}

static int netint_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel, long *temp)
{
	struct regmap *regmap = dev_get_drvdata(dev);
	int regval, err;

	switch (attr) {
	case hwmon_temp_input:
		err = regmap_read(regmap, REG_TEMP, &regval);
		break;
	case hwmon_temp_label:
	default:
		//dev_err(dev, "netint-hwmon: trying to read unsupported attribute\n");
		return -EOPNOTSUPP;
	}
	if (err < 0){
		//dev_err(dev, "netint-hwmon: unable to read attribute\n");
		return err;
	}
	*temp = regval * 1000; //convention to use millidegrees
	return 0;
}

static const struct hwmon_channel_info *netint_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops netint_hwmon_ops = {
	.is_visible = netint_is_visible,
	.read = netint_read,
};

static const struct hwmon_chip_info netint_chip_info = {
	.ops = &netint_hwmon_ops,
	.info = netint_info,
};

static const struct regmap_config netint_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_LAST,
	.volatile_reg = netint_regmap_is_volatile,
};

static int netint_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct regmap *regmap;
	int err = 0;

	if(client->addr < 0x46 || client->addr > 0x49){
		dev_err(dev, "netint-hwmon: invalid address 0x%04x for this driver\n", client->addr);
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(client, &netint_regmap_config);
	if (IS_ERR(regmap)){
		dev_err(dev, "netint-hwmon: failed to allocate register map\n");
		return PTR_ERR(regmap);
	}

	i2c_set_clientdata(client, regmap);

	int vendor1 = 0;
	int vendor2 = 0;
	err |= regmap_read(regmap, REG_VENDOR, &vendor1);
	err |= regmap_read(regmap, REG_VENDOR+1, &vendor2);
	uint16_t vendorid = (vendor1 << 8) | vendor2;

	if(vendorid != 0x1d82)
		dev_warn(dev, "netint-hwmon: vendor id did not match: 0x%04x\n", vendorid);


	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, regmap, &netint_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id netint_ids[] = {
	{ "netint", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, netint_ids);

static const struct of_device_id __maybe_unused netint_of_match[] = {
	{ .compatible = "netint,temp"},
	{},
};
MODULE_DEVICE_TABLE(of, netint_of_match);

static struct i2c_driver netint_hwmon_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "netint",
	},
	.probe		= netint_i2c_probe,
	.id_table	= netint_ids,
	.address_list	= I2C_ADDRS(0x46, 0x47, 0x48, 0x49),
};
module_i2c_driver(netint_hwmon_driver);

MODULE_AUTHOR("Alexander Hansen <alexander.hansen@9elements.com>");
MODULE_DESCRIPTION("Netint Quadra smbus hwmon driver");
MODULE_LICENSE("GPL");
