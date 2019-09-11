// SPDX-License-Identifier: GPL-2.0-only
/*
 * bootmedia-coreboot.c
 *
 * Exports the FMAP and active VBOOT slot through coreboot table.
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

#include "coreboot_table.h"
#include "fmap_serialized.h"

#define CB_TAG_BOOT_MEDIA_PARAMS 0x30

#define VBOOT_SLOT_AB "FW_MAIN"
#define VBOOT_SLOT_RECOVERY "COREBOOT"

/* Pointer to the memory maped FMAP */
static void *fmap_mmio;
static u32 fmap_size;

/* The current VBOOT slot */
const char *vboot_slot;

static ssize_t vboot_read(struct file *filp, struct kobject *kobp,
			       struct bin_attribute *bin_attr, char *buf,
			       loff_t pos, size_t count)
{
	if (!vboot_slot)
		return -ENODEV;
	if (pos < 0)
		return -EINVAL;
	if (pos > strlen(vboot_slot))
		return 0;

	if (count > strlen(vboot_slot) + 1 - pos)
		count = strlen(vboot_slot) + 1 - pos;

	/* Return NUL terminated string */
	memcpy(buf, vboot_slot + pos, count);

	return count;
}

static ssize_t fmap_read(struct file *filp, struct kobject *kobp,
			       struct bin_attribute *bin_attr, char *buf,
			       loff_t pos, size_t count)
{
	if (!fmap_mmio)
		return -ENODEV;
	if (pos < 0)
		return -EINVAL;
	if (pos >= fmap_size)
		return 0;

	if (count > fmap_size - pos)
		count = fmap_size - pos;

	/* Export the binary FMAP */
	memcpy_fromio(buf, fmap_mmio + pos, count);

	return count;
}

static int bmp_probe(struct coreboot_device *dev)
{
	struct lb_boot_media_params2 *bmp = &dev->bmp;
	void *mmio_addr = NULL;
	struct fmap fmap;
	int i;

	if (bmp->mmap_mmio_address == ~0)
		goto err_no_mmap;

	/* Verify that FMAP is memory mapped */
	if ((bmp->fmap_offset < bmp->mmap_offset))
		goto err_no_mmap;

	if (bmp->fmap_offset + sizeof(struct fmap) > (bmp->mmap_offset + bmp->mmap_size))
		goto err_no_mmap;

	mmio_addr = ioremap(bmp->mmap_mmio_address + bmp->fmap_offset, sizeof(fmap));

	/* Validate FMAP signature */
	memcpy_fromio(&fmap, mmio_addr, sizeof(struct fmap));
	if (memcmp(fmap.signature, FMAP_SIGNATURE, sizeof(fmap.signature))) {
		pr_warn("coreboot: FMAP signature missmatch\n");
		iounmap(mmio_addr);
		return -ENODEV;
	}

	fmap_size = sizeof(fmap) + fmap.nareas * sizeof(struct fmap_area);

	/* Make sure the whole FMAP is memory mapped */
	if (bmp->fmap_offset + fmap_size > bmp->mmap_offset + bmp->mmap_size)
		goto err_no_mmap;

	/* Backup the remap for sysfs access */
	fmap_mmio = mmio_addr;

	/* Try to find the active VBOOT slot in FMAP */
	for (i = 0; i < fmap.nareas; i++) {
		struct fmap_area area;
		memcpy_fromio(&area, mmio_addr + offsetof(struct fmap, areas[i]), sizeof(struct fmap_area));
		if (memcmp(VBOOT_SLOT_AB, area.name, strlen(VBOOT_SLOT_AB)) &&
		    memcmp(VBOOT_SLOT_RECOVERY, area.name, strlen(VBOOT_SLOT_RECOVERY)))
			continue;

		if (area.offset <= bmp->cbfs_offset &&
		    (area.offset + area.size) >= (bmp->cbfs_offset + bmp->cbfs_size)) {
			vboot_slot = kstrdup(area.name, GFP_KERNEL);
			break;
		}
	}

	return 0;

err_no_mmap:
	pr_warn("coreboot: FMAP isn't memory mapped\n");
	if (mmio_addr)
		iounmap(mmio_addr);

	return -ENODEV;
}

static int bmp_remove(struct coreboot_device *dev)
{
	struct platform_device *pdev = dev_get_drvdata(&dev->dev);

	if (fmap_mmio)
		iounmap(fmap_mmio);
	if (vboot_slot)
		kfree(vboot_slot);

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

static struct bin_attribute fmap_bin_attr = {
	.attr = {.name = "fmap", .mode = 0444},
	.read = fmap_read,
};

static struct bin_attribute vboot_bin_attr = {
	.attr = {.name = "vboot_active_slot", .mode = 0444},
	.read = vboot_read,
};

static int __init coreboot_bmp_init(void)
{
	int err;
	err = sysfs_create_bin_file(firmware_kobj, &fmap_bin_attr);
	if (err)
		return err;

	err = sysfs_create_bin_file(firmware_kobj, &vboot_bin_attr);
	if (err)
		return err;

	return coreboot_driver_register(&bmp_driver);
}

static void coreboot_bmp_exit(void)
{
	coreboot_driver_unregister(&bmp_driver);
	sysfs_remove_bin_file(firmware_kobj, &fmap_bin_attr);
	sysfs_remove_bin_file(firmware_kobj, &vboot_bin_attr);
}

module_init(coreboot_bmp_init);
module_exit(coreboot_bmp_exit);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_LICENSE("GPL");
