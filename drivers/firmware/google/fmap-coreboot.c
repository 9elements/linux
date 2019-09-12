// SPDX-License-Identifier: GPL-2.0-only
/*
 * fmap-coreboot.c
 *
 * Exports the binary FMAP through coreboot table.
 *
 * Copyright 2012-2013 David Herrmann <dh.herrmann@gmail.com>
 * Copyright 2017 Google Inc.
 * Copyright 2019 9elements Agency GmbH <patrick.rudolph@9elements.com>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/io.h>

#include "coreboot_table.h"
#include "fmap_serialized.h"

#define CB_TAG_FMAP 0x37

static void *fmap;
static u32 fmap_size;

/*
 * Convert FMAP region to name.
 * The caller has to free the string.
 * Return NULL if no containing region was found.
 */
const char *coreboot_fmap_region_to_name(const u32 start, const u32 size)
{
	const char *name = NULL;
	struct fmap *iter;
	u32 size_old = ~0;
	int i;

	iter = fmap;
	/* Find smallest containing region */
	for (i = 0; i < iter->nareas && fmap; i++) {
		if (iter->areas[i].offset <= start &&
		    iter->areas[i].size >= size &&
		    iter->areas[i].size <= size_old) {
			size_old = iter->areas[i].size;
			name = iter->areas[i].name;
		}
	}

	if (name)
		return kstrdup(name, GFP_KERNEL);
	return NULL;
}
EXPORT_SYMBOL(coreboot_fmap_region_to_name);

static ssize_t fmap_read(struct file *filp, struct kobject *kobp,
			 struct bin_attribute *bin_attr, char *buf,
			 loff_t pos, size_t count)
{
	if (!fmap)
		return -ENODEV;

	return memory_read_from_buffer(buf, count, &pos, fmap, fmap_size);
}

static int fmap_probe(struct coreboot_device *dev)
{
	struct lb_cbmem_ref *cbmem_ref = &dev->cbmem_ref;
	struct fmap *header;

	if (!cbmem_ref)
		return -ENODEV;

	header = memremap(cbmem_ref->cbmem_addr, sizeof(*header), MEMREMAP_WB);
	if (!header) {
		pr_warn("coreboot: Failed to remap FMAP\n");
		return -ENOMEM;
	}

	/* Validate FMAP signature */
	if (memcmp(header->signature, FMAP_SIGNATURE,
		   sizeof(header->signature))) {
		pr_warn("coreboot: FMAP signature mismatch\n");
		memunmap(header);
		return -ENODEV;
	}

	/* Validate FMAP version */
	if (header->ver_major != FMAP_VER_MAJOR) {
		pr_warn("coreboot: FMAP version not supported\n");
		memunmap(header);
		return -ENODEV;
	}

	pr_info("coreboot: Got valid FMAP v%u.%u for 0x%x byte ROM\n",
		header->ver_major, header->ver_minor, header->size);

	fmap_size = sizeof(*header) + header->nareas * sizeof(struct fmap_area);
	memunmap(header);

	fmap = devm_memremap(&dev->dev, cbmem_ref->cbmem_addr, fmap_size,
			     MEMREMAP_WB);
	if (!fmap) {
		pr_warn("coreboot: Failed to remap FMAP\n");
		return -ENOMEM;
	}

	return 0;
}

static int fmap_remove(struct coreboot_device *dev)
{
	struct platform_device *pdev = dev_get_drvdata(&dev->dev);

	platform_device_unregister(pdev);

	return 0;
}

static struct coreboot_driver fmap_driver = {
	.probe = fmap_probe,
	.remove = fmap_remove,
	.drv = {
		.name = "fmap",
	},
	.tag = CB_TAG_FMAP,
};

static struct bin_attribute fmap_bin_attr = {
	.attr = {.name = "fmap", .mode = 0444},
	.read = fmap_read,
};

static int __init coreboot_fmap_init(void)
{
	int err;

	err = sysfs_create_bin_file(firmware_kobj, &fmap_bin_attr);
	if (err)
		return err;

	return coreboot_driver_register(&fmap_driver);
}

static void coreboot_fmap_exit(void)
{
	coreboot_driver_unregister(&fmap_driver);
	sysfs_remove_bin_file(firmware_kobj, &fmap_bin_attr);
}

module_init(coreboot_fmap_init);
module_exit(coreboot_fmap_exit);

MODULE_AUTHOR("9elements Agency GmbH");
MODULE_LICENSE("GPL");
