// SPDX-License-Identifier: GPL-2.0-only
/*
 * bootmedia-coreboot.c
 *
 * Exports the active VBOOT partition name through boot media params.
 *
 * Copyright 2012-2013 David Herrmann <dh.herrmann@gmail.com>
 * Copyright 2017 Google Inc.
 * Copyright 2019 Patrick Rudolph <patrick.rudolph@9elements.com>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "coreboot_table.h"
#include "fmap-coreboot.h"

#define CB_TAG_BOOT_MEDIA_PARAMS 0x30

/* The current CBFS partition */
static char *name;

static ssize_t cbfs_active_partition_read(struct file *filp,
					  struct kobject *kobp,
					  struct bin_attribute *bin_attr,
					  char *buf,
					  loff_t pos, size_t count)
{
	if (!name)
		return -ENODEV;

	return memory_read_from_buffer(buf, count, &pos, name, strlen(name));
}

static int bmp_probe(struct coreboot_device *dev)
{
	struct lb_boot_media_params *b = &dev->bmp;
	const char *tmp;

	/* Sanity checks on the data we got */
	if ((b->cbfs_offset == ~0) ||
	    b->cbfs_size == 0 ||
	    ((b->cbfs_offset + b->cbfs_size) > b->boot_media_size)) {
		pr_warn("coreboot: Boot media params contains invalid data\n");
		return -ENODEV;
	}

	tmp = coreboot_fmap_region_to_name(b->cbfs_offset, b->cbfs_size);
	if (!tmp) {
		pr_warn("coreboot: Active CBFS region not found in FMAP\n");
		return -ENODEV;
	}

	name = devm_kmalloc(&dev->dev, strlen(tmp) + 2, GFP_KERNEL);
	if (!name) {
		kfree(tmp);
		return -ENODEV;
	}
	snprintf(name, strlen(tmp) + 2, "%s\n", tmp);

	kfree(tmp);

	return 0;
}

static int bmp_remove(struct coreboot_device *dev)
{
	struct platform_device *pdev = dev_get_drvdata(&dev->dev);

	platform_device_unregister(pdev);

	return 0;
}

static struct coreboot_driver bmp_driver = {
	.probe = bmp_probe,
	.remove = bmp_remove,
	.drv = {
		.name = "bootmediaparams",
	},
	.tag = CB_TAG_BOOT_MEDIA_PARAMS,
};

static struct bin_attribute cbfs_partition_bin_attr = {
	.attr = {.name = "cbfs_active_partition", .mode = 0444},
	.read = cbfs_active_partition_read,
};

static int __init coreboot_bmp_init(void)
{
	int err;

	err = sysfs_create_bin_file(firmware_kobj, &cbfs_partition_bin_attr);
	if (err)
		return err;

	return coreboot_driver_register(&bmp_driver);
}

static void coreboot_bmp_exit(void)
{
	coreboot_driver_unregister(&bmp_driver);
	sysfs_remove_bin_file(firmware_kobj, &cbfs_partition_bin_attr);
}

module_init(coreboot_bmp_init);
module_exit(coreboot_bmp_exit);

MODULE_AUTHOR("9elements Agency GmbH");
MODULE_LICENSE("GPL");
