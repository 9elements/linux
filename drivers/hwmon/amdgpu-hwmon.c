// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AMD Radeon Pro V620 smbus temperature and power sensor hwmon driver
 *
 *   Author: Alexander Hansen <alexander.hansen@9elements.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

//Register Definitions
#define REG_DEVICE_ID 0xef05
#define REG_GPU_TDIODE_TEMP 0xef10
#define REG_GPU_DIE_HOTSPOT_TEMP 0xef13
#define REG_GPU_MEM_TEMP 0xef14
#define REG_GPU_RUNTIME_FREQ 0xef18
#define REG_CARD_ACTUAL_POWER 0xef28
#define REG_LAST 0xef2a

static int amdgpu_smbus_read(struct i2c_client *client, uint16_t reg, int width, int *value){

	int ret;
	uint8_t input[2] = {(uint8_t)((reg & 0xff00) >> 8), (uint8_t)(reg & 0xff)};
	uint8_t buf[2];

	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = input,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = width,
			.buf = buf,
		}
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if(ret < 0){
		return ret;
	}

	if(width == 2)
		*value = (buf[1] << 8) | buf[0];
	else
		*value = buf[0];

	return 0;
}

static umode_t amdgpu_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr, int channel)
{
	if(type != hwmon_temp && type != hwmon_power)
		return 0;

	switch(attr){
		case hwmon_temp_input:
		case hwmon_temp_label:
			if(channel <= 2) return 0444;
		case hwmon_power_input:
		case hwmon_power_label:
			if(channel == 0) return 0444;
		default: return 0;
	}
}

static int amdgpu_read_string(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel, const char **str)
{
	char* temp_names[] = {"tdiode_temp", "die_hotspot_temp", "mem_temp"};

	switch(attr){
		case hwmon_temp_label:
			*str = temp_names[channel];
			break;
		case hwmon_power_label:
			*str = "power_consumption";
			break;
		default:
			return -EOPNOTSUPP;
	}
	return 0;
}

static int amdgpu_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel, long *value)
{
	struct i2c_client *client = dev_get_drvdata(dev);
	int regval, err, reg;

	switch(attr){
	case hwmon_temp_input:
		switch(channel){
			case 0: reg = REG_GPU_TDIODE_TEMP; break;
			case 1: reg = REG_GPU_DIE_HOTSPOT_TEMP; break;
			case 2: reg = REG_GPU_MEM_TEMP; break;
			default:
				//dev_err(dev, "trying to read temp from unsupported channel\n");
				return -EOPNOTSUPP;
		}
		err = amdgpu_smbus_read(client, reg, 1, &regval);
		break;
	case hwmon_power_input:
		err = amdgpu_smbus_read(client, REG_CARD_ACTUAL_POWER, 2, &regval);
		break;
	default:
		//dev_err(dev, "trying to read unsupported attribute\n");
		return -EOPNOTSUPP;
	}

	if(err < 0){
		//dev_err(dev, "unable to read attribute\n");
		return err;
	}

	*value = regval * 1000;
	return 0;
}

static const struct hwmon_channel_info *amdgpu_info[] = {
	HWMON_CHANNEL_INFO(temp,
		HWMON_T_INPUT | HWMON_T_LABEL, /*tdiode temp*/
		HWMON_T_INPUT | HWMON_T_LABEL, /*die temp*/
		HWMON_T_INPUT | HWMON_T_LABEL  /*mem temp*/),
	HWMON_CHANNEL_INFO(power,
		HWMON_P_INPUT | HWMON_P_LABEL /*power consumption*/),
	NULL
};

static const struct hwmon_ops amdgpu_hwmon_ops = {
	.is_visible = amdgpu_is_visible,
	.read = amdgpu_read,
	.read_string = amdgpu_read_string,
};

static const struct hwmon_chip_info amdgpu_chip_info = {
	.ops = &amdgpu_hwmon_ops,
	.info = amdgpu_info,
};

static int amdgpu_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	int err = 0;

	if(client->addr != 0x56){
		dev_err(dev, "invalid address 0x%04x for this driver\n", client->addr);
		return -EINVAL;
	}

	i2c_set_clientdata(client, client);

	int device_id = 0;
	err |= amdgpu_smbus_read(client, REG_DEVICE_ID, 2, &device_id);

	if(err != 0){
		dev_err(dev, "could not read device id");
		return -EINVAL;
	}

	if(device_id != 0x73a1){
		dev_err(dev, "device id did not match, was: 0x%04x\n", device_id);
		return -EINVAL;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, client, &amdgpu_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id amdgpu_ids[] = {
	{"amdgpu_hwmon", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, amdgpu_ids);

static const struct of_device_id __maybe_unused amdgpu_of_match[] = {
	{ .compatible = "amd,amdgpu_hwmon"},
	{},
};
MODULE_DEVICE_TABLE(of, amdgpu_of_match);

static struct i2c_driver amdgpu_hwmon_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "amdgpu_hwmon",
	},
	.probe = amdgpu_i2c_probe,
	.id_table = amdgpu_ids,
	.address_list = I2C_ADDRS(0x56),
};
module_i2c_driver(amdgpu_hwmon_driver);

MODULE_AUTHOR("Alexander Hansen <alexander.hansen@9elements.com>");
MODULE_DESCRIPTION("AMD GPU smbus hwmon driver");
MODULE_LICENSE("GPL");
