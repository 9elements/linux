// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * sbrmi.c - hwmon driver for a SB-RMI mailbox
 *           compliant AMD SoC device.
 *
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/regmap.h>

#include "sbrmi-common.h"

#define SOCK_0_ADDR	0x3C
#define SOCK_1_ADDR	0x38

/* Do not allow setting negative power limit */
#define SBRMI_PWR_MIN	0

/* SBRMI REVISION REG */
#define SBRMI_REV	0x0
#define SBRMI_REV_BRTH (0x21)

#define DIMM_BASE_ID		(0x80)
#define MAX_DIMM_COUNT		(12)
#define DIMM_POWER_OFFSET	(17)
#define DIMM_TEMP_OFFSET	(21)
#define DIMM_TEMP_SCALE 	1/4
#define MAX_WAIT_TIME_SEC	(3)

/* SBRMI registers data out is 1 byte */
#define SBRMI_REG_DATA_SIZE		0x1
/* Default SBRMI register address is 1 byte */
#define SBRMI_REG_ADDR_SIZE_DEF		0x1
/* TURIN SBRMI register address is 2 byte */
#define SBRMI_REG_ADDR_SIZE_TWO_BYTE	0x2

/* Two xfers, one write and one read require to read the data */
#define I3C_I2C_MSG_XFER_SIZE		0x2

static int configure_regmap(struct apml_sbrmi_device *rmi_dev);

enum sbrmi_msg_id {
	SBRMI_READ_PKG_PWR_CONSUMPTION = 0x1,
	SBRMI_WRITE_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_MAX_PWR_LIMIT,
	SBRMI_READ_DIMM_POWER_CONSUMPTION = 0x47,
	SBRMI_READ_DIMM_THERMAL_SENSOR = 0x48,
};

static int sbrmi_get_max_pwr_limit(struct apml_sbrmi_device *rmi_dev)
{
	struct apml_message msg = { 0 };
	int ret;

	msg.cmd = SBRMI_READ_PKG_MAX_PWR_LIMIT;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;
	ret = rmi_mailbox_xfer(rmi_dev, &msg);
	if (ret < 0)
		return ret;
	rmi_dev->pwr_limit_max = msg.data_out.mb_out[RD_WR_DATA_INDEX];

	return ret;
}

static int sbrmi_read(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long *val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };
	int ret;

	if (type != hwmon_power && type != hwmon_temp)
		return -EINVAL;

	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in sbrmi_read\n", ret);
			return ret;
		}
	}

	mutex_lock(&rmi_dev->lock);
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;

	switch (attr) {
	case hwmon_power_input:
		if (channel > 0) {
			msg.cmd = SBRMI_READ_DIMM_POWER_CONSUMPTION;
			msg.data_in.mb_in[RD_WR_DATA_INDEX] = rmi_dev->dimm_id[channel-1];
		} else {
			msg.cmd = SBRMI_READ_PKG_PWR_CONSUMPTION;
		}
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_power_cap:
		msg.cmd = SBRMI_READ_PKG_PWR_LIMIT;
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_temp_input:
		msg.cmd = SBRMI_READ_DIMM_THERMAL_SENSOR;
		msg.data_in.mb_in[RD_WR_DATA_INDEX] = rmi_dev->dimm_id[channel];
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_power_cap_max:
		/* Cache maximum power limit */
		if (!rmi_dev->pwr_limit_max) {
			ret = sbrmi_get_max_pwr_limit(rmi_dev);
			if (ret < 0)
				goto out;
		}
		msg.data_out.mb_out[RD_WR_DATA_INDEX] = rmi_dev->pwr_limit_max;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}
	if (ret < 0)
		goto out;

	if (type == hwmon_power) {
		/* hwmon power attributes are in microWatt */
		if (channel > 0) {
			*val = (msg.data_out.mb_out[RD_WR_DATA_INDEX] >> DIMM_POWER_OFFSET) * 1000;
		} else {
			*val = (long)msg.data_out.mb_out[RD_WR_DATA_INDEX] * 1000;
		}
	} else if (type == hwmon_temp) {
		/* sbrmi temp is floating point, convert to deg C rational num */
		*val = (msg.data_out.mb_out[RD_WR_DATA_INDEX] >> DIMM_TEMP_OFFSET) * 1000 * DIMM_TEMP_SCALE;
	}
out:
	mutex_unlock(&rmi_dev->lock);
	return ret;
}

static int sbrmi_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };
	int ret;

	if (type != hwmon_power && attr != hwmon_power_cap)
		return -EINVAL;

	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in sbrmi_write\n", ret);
			return ret;
		}
	}

	/*
	 * hwmon power attributes are in microWatt
	 * mailbox read/write is in mWatt
	 */
	val /= 1000;

	val = clamp_val(val, SBRMI_PWR_MIN, rmi_dev->pwr_limit_max);

	msg.cmd = SBRMI_WRITE_PKG_PWR_LIMIT;
	msg.data_in.mb_in[RD_WR_DATA_INDEX] = val;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 0;

	mutex_lock(&rmi_dev->lock);
	ret = rmi_mailbox_xfer(rmi_dev, &msg);
	mutex_unlock(&rmi_dev->lock);
	return ret;
}

static umode_t sbrmi_is_visible(const void *data,
				enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	switch (type) {
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_cap_max:
			return 0444;
		case hwmon_power_cap:
			return 0644;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		}
		break;
	default:
		break;
	}
	return 0;
}

static struct hwmon_channel_info sbrmi_power_info = {
	hwmon_power,
	NULL
};

static struct hwmon_channel_info sbrmi_temp_info = {
	hwmon_temp,
	NULL
};

static const struct hwmon_channel_info *sbrmi_info[] = {
	&sbrmi_power_info,
	&sbrmi_temp_info,
	NULL
};

static const struct hwmon_ops sbrmi_hwmon_ops = {
	.is_visible = sbrmi_is_visible,
	.read = sbrmi_read,
	.write = sbrmi_write,
};

static const struct hwmon_chip_info sbrmi_chip_info = {
	.ops = &sbrmi_hwmon_ops,
	.info = sbrmi_info,
};

static long sbrmi_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int __user *arguser = (int  __user *)arg;
	struct apml_message msg = { 0 };
	struct apml_sbrmi_device *rmi_dev;
	bool read = false;
	int ret = -EFAULT;

	rmi_dev = fp->private_data;
	if (!rmi_dev)
		return ret;

	/*
	 * If device remove/unbind is called do not allow new transaction
	 * Tickets: https://ontrack-internal.amd.com/browse/PLAT-122500
	 * 	    https://ontrack-internal.amd.com/browse/DCSM-154
	 */
	if (atomic_read(&rmi_dev->no_new_trans))
		return -EBUSY;

	/* Copy the structure from user */
	if (copy_struct_from_user(&msg, sizeof(msg), arguser,
				  sizeof(struct apml_message)))
		return ret;

	/*
	 * Only one I2C/I3C transaction can happen at
	 * one time. Take lock across so no two protocol is
	 * invoked at same time, modifying the register value.
	 */
	mutex_lock(&rmi_dev->lock);

	/* Verify device unbind/remove is not invoked */
	if (atomic_read(&rmi_dev->no_new_trans)) {
		mutex_unlock(&rmi_dev->lock);
		return -EBUSY;
	}
	/* Is this a read/monitor/get request */
	if (msg.data_in.reg_in[RD_FLAG_INDEX])
		read = true;
	/*
	 * Set the in_progress variable to true, to wait for
	 * completion during unbind/remove of driver
	 */
	atomic_set(&rmi_dev->in_progress, 1);
	switch (msg.cmd) {
	case 0 ... 0x999:
		/* Mailbox protocol */
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case APML_CPUID:
		/* CPUID protocol */
		ret = rmi_cpuid_read(rmi_dev, &msg);
		break;
	case APML_MCA_MSR:
		/* MCAMSR protocol */
		ret = rmi_mca_msr_read(rmi_dev, &msg);
		break;
	case APML_REG:
		/* REG R/W */
		if (read) {
			ret = regmap_read(rmi_dev->regmap,
					  msg.data_in.mb_in[REG_OFF_INDEX],
					  &msg.data_out.mb_out[RD_WR_DATA_INDEX]);
		} else {
			ret = regmap_write(rmi_dev->regmap,
					    msg.data_in.reg_in[REG_OFF_INDEX],
					    msg.data_in.reg_in[REG_VAL_INDEX]);
		}
		break;
	default:
		break;
	}

	/* Send complete only if device is unbinded/remove */
	if (atomic_read(&rmi_dev->no_new_trans))
		complete(&rmi_dev->misc_fops_done);
	atomic_set(&rmi_dev->in_progress, 0);
	mutex_unlock(&rmi_dev->lock);

	/* Copy results back to user only for get/monitor commands and firmware failures */
	if ((read && !ret) || ret == -EPROTOTYPE) {
		if (copy_to_user(arguser, &msg, sizeof(struct apml_message)))
			ret = -EFAULT;
	}
	return ret;
}

static int sbrmi_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct apml_sbrmi_device *rmi_dev = container_of(mdev, struct apml_sbrmi_device,
							 sbrmi_misc_dev);
	int ret = 0;

	if (!rmi_dev)
		return -ENODEV;

	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in misc dev open\n", ret);
			return ret;
		}
	}
	filp->private_data = rmi_dev;
	return 0;
}

static int sbrmi_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static const struct file_operations sbrmi_fops = {
	.owner		= THIS_MODULE,
	.open		= sbrmi_open,
	.release	= sbrmi_release,
	.unlocked_ioctl	= sbrmi_ioctl,
	.compat_ioctl	= sbrmi_ioctl,
};

static int create_misc_rmi_device(struct apml_sbrmi_device *rmi_dev,
				  struct device *dev)
{
	int ret;

	rmi_dev->sbrmi_misc_dev.name		= devm_kasprintf(dev, GFP_KERNEL, "apml_rmi%d", rmi_dev->sock_num);
	rmi_dev->sbrmi_misc_dev.minor		= MISC_DYNAMIC_MINOR;
	rmi_dev->sbrmi_misc_dev.fops		= &sbrmi_fops;
	rmi_dev->sbrmi_misc_dev.parent		= dev;
	rmi_dev->sbrmi_misc_dev.nodename	= devm_kasprintf(dev, GFP_KERNEL, "sbrmi%d", rmi_dev->sock_num);
	rmi_dev->sbrmi_misc_dev.mode		= 0600;

	ret = misc_register(&rmi_dev->sbrmi_misc_dev);
	if (ret)
		return ret;

	dev_info(dev, "register %s device\n", rmi_dev->sbrmi_misc_dev.name);
	return ret;
}

static int sbrmi_i2c_identify_reg_addr_size(struct i2c_client *i2cdev, u32 *size, u32 *rev)
{
	struct i2c_msg xfer[I3C_I2C_MSG_XFER_SIZE];
	int reg = SBRMI_REV;
	int val_size = SBRMI_REG_DATA_SIZE;
	int ret;
	int probe = 3;

	do
	{
		// Attempt two byte addressing mode
		xfer[0].addr = i2cdev->addr;
		xfer[0].flags = 0;
		xfer[0].len = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
		xfer[0].buf = (void *)&reg;

		xfer[1].addr = i2cdev->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = val_size;
		xfer[1].buf = (void *)rev;

		ret = i2c_transfer(i2cdev->adapter, xfer, I3C_I2C_MSG_XFER_SIZE);

		if (ret >= 0)
		{
			pr_err("I2C SBRMI_REV command returned value: %d\n", *rev);
			if(*rev == SBRMI_REV_BRTH)
				*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
			else
				*size = SBRMI_REG_ADDR_SIZE_DEF;

			return 0;
		}
		else
		{
			probe--;
			continue;
		}
	} while (probe > 0);

	probe = 3;
	do
	{
		// Attempt one byte addressing mode
		xfer[0].addr = i2cdev->addr;
		xfer[0].flags = 0;
		xfer[0].len = SBRMI_REG_ADDR_SIZE_DEF;
		xfer[0].buf = (void *)&reg;

		xfer[1].addr = i2cdev->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = val_size;
		xfer[1].buf = (void *)rev;

		ret = i2c_transfer(i2cdev->adapter, xfer, I3C_I2C_MSG_XFER_SIZE);
		if (ret >= 0)
		{
			pr_err("I2C SBRMI_REV command returned value: %d\n", *rev);
			if(*rev == SBRMI_REV_BRTH)
				*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
			else
				*size = SBRMI_REG_ADDR_SIZE_DEF;

			return 0;
		}
		else
		{
			probe--;
			continue;
		}
	} while (probe > 0);

	pr_err("I2C SBRMI_REV error code value: %d\n", ret);
	return ret;
}

static int sbrmi_hwmon_add_chan_info(struct device *dev, int num)
{
	u32 *cfg;
	int i;

	cfg = devm_kcalloc(dev, num + 2, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	// power1: PKG_PWR
	// power2 ~ powerN+1: DIMM_PWR
	cfg[0] = HWMON_P_INPUT | HWMON_P_CAP | HWMON_P_CAP_MAX;
	for (i = 1; i <= num; i++) {
		cfg[i] = HWMON_P_INPUT;
	}
	sbrmi_power_info.config = cfg;

	cfg = devm_kcalloc(dev, num + 1, sizeof(*cfg), GFP_KERNEL);
	if (!cfg) {
		devm_kfree(dev, sbrmi_power_info.config);
		sbrmi_power_info.config = NULL;
		return -ENOMEM;
	}

	// temp1 ~ tempN: DIMM_TEMP
	for (i = 0; i < num; i++) {
		cfg[i] = HWMON_T_INPUT;
	}
	sbrmi_temp_info.config = cfg;

	return 0;
}

static int sbrmi_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;
	int ret;
	int i;
	u32 dimm_cnt;
	u32 *dimm_id;

	rmi_dev = devm_kzalloc(dev, sizeof(struct apml_sbrmi_device), GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;

	atomic_set(&rmi_dev->in_progress, 0);
	atomic_set(&rmi_dev->no_new_trans, 0);
	rmi_dev->client = client;

	ret = configure_regmap(rmi_dev);
	if (ret < 0) {
		pr_err("regmap configuration failed with return value:%d in Probe\n", ret);
	}
	mutex_init(&rmi_dev->lock);

	dev_set_drvdata(dev, (void *)rmi_dev);

	dimm_id = (u32 *)devm_kzalloc(dev, sizeof(u32) * MAX_DIMM_COUNT, GFP_KERNEL);
	if (!dimm_id)
		return -ENOMEM;

	dimm_cnt = 0;
	// Read DIMM Count
	ret = of_property_read_u32(dev->of_node, "dimm-count", &dimm_cnt);

	if ( (dimm_cnt == 0) || (dimm_cnt > MAX_DIMM_COUNT) )
	{
		dev_info(&client->dev, "SBRMI: Cannot read DIMM Count or exceeds max, default it to %d\n", MAX_DIMM_COUNT);
		dimm_cnt = MAX_DIMM_COUNT;
	}

	// Read DIMM ID
	ret = of_property_read_u32_array(dev->of_node, "dimm-ids", dimm_id, dimm_cnt);
	if (ret) {
		dev_info(&client->dev, "SBRMI: Cannot read DIMM IDs, set them to base address\n");

		for (i = 0; i < dimm_cnt; i++)
			dimm_id[i] = DIMM_BASE_ID + i;
	}
	rmi_dev->dimm_id = dimm_id;

	ret = sbrmi_hwmon_add_chan_info(dev, dimm_cnt);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 rmi_dev,
							 &sbrmi_chip_info,
							 NULL);

	if (!hwmon_dev)
		return PTR_ERR_OR_ZERO(hwmon_dev);

	if (client->addr == SOCK_0_ADDR)
		rmi_dev->sock_num = 0;
	if (client->addr == SOCK_1_ADDR)
		rmi_dev->sock_num = 1;

	init_completion(&rmi_dev->misc_fops_done);
	return create_misc_rmi_device(rmi_dev, dev);
}

static int sbrmi_i3c_identify_reg_addr_size(struct i3c_device *i3cdev, u32 *size, u32 *rev)
{
	struct i3c_priv_xfer xfers[I3C_I2C_MSG_XFER_SIZE];
	int reg = SBRMI_REV;
	int val_size = SBRMI_REG_DATA_SIZE;
	int ret;
	int probe = 3;

	do
	{
		// Attempt two byte addressing mode
		xfers[0].rnw = false;
		xfers[0].len = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
		xfers[0].data.out = &reg;

		xfers[1].rnw = true;
		xfers[1].len = val_size;
		xfers[1].data.in = rev;

		ret = i3c_device_do_priv_xfers(i3cdev, xfers, I3C_I2C_MSG_XFER_SIZE);

		if (ret >= 0)
		{
			pr_err("I3C SBRMI_REV command returned value: %d\n", *rev);
			if(*rev == SBRMI_REV_BRTH)
				*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
			else
				*size = SBRMI_REG_ADDR_SIZE_DEF;

			return 0;
		}
		else
		{
			probe--;
			continue;
		}
	} while (probe > 0);

	probe = 3;
	do
	{
		// Attempt one byte addressing mode
		xfers[0].rnw = false;
		xfers[0].len = SBRMI_REG_ADDR_SIZE_DEF;
		xfers[0].data.out = &reg;

		xfers[1].rnw = true;
		xfers[1].len = val_size;
		xfers[1].data.in = rev;

		ret = i3c_device_do_priv_xfers(i3cdev, xfers, I3C_I2C_MSG_XFER_SIZE);

		if (ret >= 0)
		{
			pr_err("I3C SBRMI_REV command returned value: %d\n", *rev);
			if(*rev == SBRMI_REV_BRTH)
				*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
			else
				*size = SBRMI_REG_ADDR_SIZE_DEF;

			return 0;
		}
		else
		{
			probe--;
			continue;
		}
	} while (probe > 0);

	pr_err("I3C SBRMI_REV error code value: %d\n", ret);
	return ret;
}

static int init_rmi_regmap(struct apml_sbrmi_device *rmi_dev, u32 size, u32 rev)
{
	struct regmap_config sbrmi_regmap_config = {
		.reg_bits = 8 * size,
		.val_bits = 8,
		.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	};
	struct regmap *regmap;

	if (rmi_dev->i3cdev) {
		regmap = devm_regmap_init_i3c(rmi_dev->i3cdev,
					      &sbrmi_regmap_config);
		if (IS_ERR(regmap)) {
			dev_err(&rmi_dev->i3cdev->dev,
				"Failed to register i3c regmap %d\n",
				(int)PTR_ERR(regmap));
			return PTR_ERR(regmap);
		}
	} else if (rmi_dev->client) {
		regmap = devm_regmap_init_i2c(rmi_dev->client,
					      &sbrmi_regmap_config);
		if (IS_ERR(regmap))
			return PTR_ERR(rmi_dev->regmap);
	} else {
		return -ENODEV;
	}

	rmi_dev->regmap = regmap;
	rmi_dev->rev = rev;
	return 0;
}

/*
 * configure_regmap call should happen in probe, currently for Turin
 * the I3C APML client controllers are not initialized and also
 * Efuse need to be done so move the config to first transaction
 * https://ontrack-internal.amd.com/browse/PLAT-126960
 */
static int configure_regmap(struct apml_sbrmi_device *rmi_dev)
{
	u32 size = 2;
	u32 rev = 0;
	int ret = 0;

	if (rmi_dev->i3cdev) {
		ret = sbrmi_i3c_identify_reg_addr_size(rmi_dev->i3cdev, &size, &rev);
		if (ret < 0) {
			pr_err("Reg size identification failed with return value:%d\n", ret);
			return ret;
		}
	} else if (rmi_dev->client) {
		ret = sbrmi_i2c_identify_reg_addr_size(rmi_dev->client, &size, &rev);
		if (ret < 0) {
			pr_err("Reg size identification failed with return value:%d\n", ret);
			return ret;
		}
	} else {
		return ret;
	}
	ret = init_rmi_regmap(rmi_dev, size, rev);
	return ret;
}

static int sbrmi_i3c_probe(struct i3c_device *i3cdev)
{
	struct device *dev = &i3cdev->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;
	int ret;
	int i;
	u32 dimm_cnt;
	u32 *dimm_id;

	rmi_dev = devm_kzalloc(dev, sizeof(struct apml_sbrmi_device), GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;

	atomic_set(&rmi_dev->in_progress, 0);
	atomic_set(&rmi_dev->no_new_trans, 0);
	rmi_dev->i3cdev = i3cdev;

	ret = configure_regmap(rmi_dev);
	if (ret < 0) {
		pr_err("regmap configuration failed with return value:%d in Probe\n", ret);
	}
	mutex_init(&rmi_dev->lock);

	dev_set_drvdata(dev, (void *)rmi_dev);

	dimm_id = (u32 *)devm_kzalloc(dev, sizeof(u32) * MAX_DIMM_COUNT, GFP_KERNEL);
	if (!dimm_id)
		return -ENOMEM;

	dimm_cnt = 0;
	// Read DIMM Count
	ret = of_property_read_u32(dev->of_node, "dimm-count", &dimm_cnt);

	if ( (dimm_cnt == 0) || (dimm_cnt > MAX_DIMM_COUNT) )
	{
		dev_info(&i3cdev->dev, "SBRMI: Cannot read DIMM Count or exceeds max, default it to %d\n", MAX_DIMM_COUNT);
		dimm_cnt = MAX_DIMM_COUNT;
	}

	// Read DIMM ID
	ret = of_property_read_u32_array(dev->of_node, "dimm-ids", dimm_id, dimm_cnt);
	if (ret) {
		dev_info(&i3cdev->dev, "SBRMI: Cannot read DIMM IDs, set them to base address\n");

		for (i = 0; i < dimm_cnt; i++)
			dimm_id[i] = DIMM_BASE_ID + i;
	}
	rmi_dev->dimm_id = dimm_id;

	ret = sbrmi_hwmon_add_chan_info(dev, dimm_cnt);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "sbrmi_i3c", rmi_dev,
							 &sbrmi_chip_info, NULL);

	if (!hwmon_dev)
		return PTR_ERR_OR_ZERO(hwmon_dev);

	if (i3cdev->desc->info.static_addr == SOCK_0_ADDR)
		rmi_dev->sock_num = 0;
	if (i3cdev->desc->info.static_addr == SOCK_1_ADDR)
		rmi_dev->sock_num = 1;

	init_completion(&rmi_dev->misc_fops_done);
	return create_misc_rmi_device(rmi_dev, dev);
}

static void sbrmi_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&client->dev);

	if (!rmi_dev)
		return;
	/*
	 * Set the no_new_trans so no new transaction can
	 * occur in sbrmi_ioctl
	 */
	atomic_set(&rmi_dev->no_new_trans, 1);
	/*
	 * If any transaction is in progress wait for the
	 * transaction to get complete
	 * Max wait is 3 sec for any pending transaction to
	 * complete, https://ontrack-internal.amd.com/browse/DCSM-84
	 */
	if (atomic_read(&rmi_dev->in_progress))
		wait_for_completion_timeout(&rmi_dev->misc_fops_done,
					    MAX_WAIT_TIME_SEC * HZ);
	misc_deregister(&rmi_dev->sbrmi_misc_dev);
	/* Assign fops and parent of misc dev to NULL */
	rmi_dev->sbrmi_misc_dev.fops = NULL;
	rmi_dev->sbrmi_misc_dev.parent = NULL;

	if (rmi_dev->dimm_id)
		devm_kfree(dev, rmi_dev->dimm_id);
	if (sbrmi_power_info.config)
		devm_kfree(dev, sbrmi_power_info.config);
	if (sbrmi_temp_info.config)
		devm_kfree(dev, sbrmi_temp_info.config);

	dev_info(&client->dev, "Removed sbrmi driver\n");
}

static void sbrmi_i3c_remove(struct i3c_device *i3cdev)
{
	struct device *dev = &i3cdev->dev;
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&i3cdev->dev);

	if (!rmi_dev)
		return;
	/*
	 * Set the no_new_trans so no new transaction can
	 * occur in sbrmi_ioctl
	 */
	atomic_set(&rmi_dev->no_new_trans, 1);
	/*
	 * If any transaction is in progress wait for the
	 * transaction to get complete
	 * Max wait is 3 sec for any pending transaction to
	 * complete, https://ontrack-internal.amd.com/browse/DCSM-84
	 */
	if (atomic_read(&rmi_dev->in_progress))
		wait_for_completion_timeout(&rmi_dev->misc_fops_done,
					MAX_WAIT_TIME_SEC * HZ);
	misc_deregister(&rmi_dev->sbrmi_misc_dev);
	/* Assign fops and parent of misc dev to NULL */
	rmi_dev->sbrmi_misc_dev.fops = NULL;
	rmi_dev->sbrmi_misc_dev.parent = NULL;

	if (rmi_dev->dimm_id)
		devm_kfree(dev, rmi_dev->dimm_id);
	if (sbrmi_power_info.config)
		devm_kfree(dev, sbrmi_power_info.config);
	if (sbrmi_temp_info.config)
		devm_kfree(dev, sbrmi_temp_info.config);

	dev_info(&i3cdev->dev, "Removed sbrmi_i3c driver\n");
}

static const struct i2c_device_id sbrmi_id[] = {
	{"sbrmi", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sbrmi_id);

static const struct of_device_id __maybe_unused sbrmi_of_match[] = {
	{
		.compatible = "amd,sbrmi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sbrmi_of_match);

static const struct i3c_device_id sbrmi_i3c_id[] = {
	I3C_DEVICE_EXTRA_INFO(0x112, 0x0, 0x2, NULL),
	{}
};
MODULE_DEVICE_TABLE(i3c, sbrmi_i3c_id);

static struct i2c_driver sbrmi_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "sbrmi",
		.of_match_table = of_match_ptr(sbrmi_of_match),
	},
	.probe = sbrmi_i2c_probe,
	.remove = sbrmi_i2c_remove,
	.id_table = sbrmi_id,
};

static struct i3c_driver sbrmi_i3c_driver = {
	.driver = {
		.name = "sbrmi_i3c",
	},
	.probe = sbrmi_i3c_probe,
	.remove = sbrmi_i3c_remove,
	.id_table = sbrmi_i3c_id,
};

module_i3c_i2c_driver(sbrmi_i3c_driver, &sbrmi_driver)

MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_AUTHOR("Naveenkrishna Chatradhi <naveenkrishna.chatradhi@amd.com>");
MODULE_DESCRIPTION("Hwmon driver for AMD SB-RMI emulated sensor");
MODULE_LICENSE("GPL");
