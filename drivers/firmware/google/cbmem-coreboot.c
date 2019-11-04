// SPDX-License-Identifier: GPL-2.0-only
/*
 * cbmem-coreboot.c
 *
 * Exports CBMEM as attributes in sysfs.
 *
 * Copyright 2012-2013 David Herrmann <dh.herrmann@gmail.com>
 * Copyright 2017 Google Inc.
 * Copyright 2019 9elements Agency GmbH
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/io.h>

#include "coreboot_table.h"

#define CB_TAG_CBMEM_ENTRY 0x31

struct cb_priv {
	struct lb_cbmem_entry entry;
};

static ssize_t id_show(struct device *dev,
		       struct device_attribute *attr, char *buffer)
{
	const struct cb_priv *priv = dev_get_drvdata(dev);

	return sprintf(buffer, "%#08x\n", priv->entry.id);
}

static ssize_t size_show(struct device *dev,
			 struct device_attribute *attr, char *buffer)
{
	const struct cb_priv *priv = dev_get_drvdata(dev);

	return sprintf(buffer, "%u\n", priv->entry.entry_size);
}

static ssize_t address_show(struct device *dev,
			    struct device_attribute *attr, char *buffer)
{
	const struct cb_priv *priv = dev_get_drvdata(dev);

	return sprintf(buffer, "%#016llx\n", priv->entry.address);
}

static DEVICE_ATTR_RO(id);
static DEVICE_ATTR_RO(size);
static DEVICE_ATTR_RO(address);

static struct attribute *cb_mem_attrs[] = {
	&dev_attr_address.attr,
	&dev_attr_id.attr,
	&dev_attr_size.attr,
	NULL
};

static ssize_t data_read(struct file *filp, struct kobject *kobj,
			 struct bin_attribute *bin_attr,
			 char *buffer, loff_t offset, size_t count)
{
	const struct device *dev = kobj_to_dev(kobj);
	const struct cb_priv *priv = dev_get_drvdata(dev);
	void *ptr;

	/* CBMEM is always RAM with unknown caching attributes. */
	ptr = memremap(priv->entry.address, priv->entry.entry_size,
		       MEMREMAP_WB | MEMREMAP_WT);
	if (!ptr)
		return -ENOMEM;

	count = memory_read_from_buffer(buffer, count, &offset, ptr,
					priv->entry.entry_size);
	memunmap(ptr);

	return count;
}

static BIN_ATTR_RO(data, 0);

static struct bin_attribute *cb_mem_bin_attrs[] = {
	&bin_attr_data,
	NULL
};

static const struct attribute_group cb_mem_attr_group = {
	.name = "cbmem_attributes",
	.attrs = cb_mem_attrs,
	.bin_attrs = cb_mem_bin_attrs,
};

static const struct attribute_group *attribute_groups[] = {
	&cb_mem_attr_group,
	NULL,
};

static int cbmem_probe(struct coreboot_device *cdev)
{
	struct device *dev = &cdev->dev;
	struct cb_priv *priv;

	priv = devm_kmalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	memcpy(&priv->entry, &cdev->cbmem_entry, sizeof(priv->entry));

	dev_set_drvdata(dev, priv);

	return 0;
}

static struct coreboot_driver cbmem_driver = {
	.probe = cbmem_probe,
	.drv = {
		.name = "cbmem",
		.dev_groups = attribute_groups,
	},
	.tag = CB_TAG_CBMEM_ENTRY,
};

module_coreboot_driver(cbmem_driver);

MODULE_AUTHOR("9elements Agency GmbH");
MODULE_LICENSE("GPL");
