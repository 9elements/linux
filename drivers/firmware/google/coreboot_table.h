/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * coreboot_table.h
 *
 * Internal header for coreboot table access.
 *
 * Copyright 2014 Gerd Hoffmann <kraxel@redhat.com>
 * Copyright 2017 Google Inc.
 * Copyright 2017 Samuel Holland <samuel@sholland.org>
 */

#ifndef __COREBOOT_TABLE_H
#define __COREBOOT_TABLE_H

#include <linux/device.h>

/* Coreboot table header structure */
struct coreboot_table_header {
	char signature[4];
	u32 header_bytes;
	u32 header_checksum;
	u32 table_bytes;
	u32 table_checksum;
	u32 table_entries;
};

/* List of coreboot entry structures that is used */
/* Generic */
struct coreboot_table_entry {
	u32 tag;
	u32 size;
};

/* Points to a CBMEM entry */
struct lb_cbmem_ref {
	u32 tag;
	u32 size;

	u64 cbmem_addr;
};

/* Describes framebuffer setup by coreboot */
struct lb_framebuffer {
	u32 tag;
	u32 size;

	u64 physical_address;
	u32 x_resolution;
	u32 y_resolution;
	u32 bytes_per_line;
	u8  bits_per_pixel;
	u8  red_mask_pos;
	u8  red_mask_size;
	u8  green_mask_pos;
	u8  green_mask_size;
	u8  blue_mask_pos;
	u8  blue_mask_size;
	u8  reserved_mask_pos;
	u8  reserved_mask_size;
};

struct lb_boot_media_params2 {
	u32 tag;
	u32 size;

	/* keep compatible to lb_boot_media_params start */

	/* offsets are relative to start of boot media */
	u64 fmap_offset;
	u64 cbfs_offset;
	u64 cbfs_size;
	u64 boot_media_size;
	/* keep compatible to lb_boot_media_params end */

	/* offsets are relative to start of boot media */
	u64 cbfs_ro_offset;
	u64 mmap_offset;
	/* Size is in bytes */
	u64 cbfs_ro_size;
	u64 mmap_size;
	/* MMIO address of MMAPed boot media, ~0ULL if not MMAPed.*/
	u64 mmap_mmio_address;
};

/* A device, additionally with information from coreboot. */
struct coreboot_device {
	struct device dev;
	union {
		struct coreboot_table_entry entry;
		struct lb_cbmem_ref cbmem_ref;
		struct lb_framebuffer framebuffer;
		struct lb_boot_media_params2 bmp;
	};
};

/* A driver for handling devices described in coreboot tables. */
struct coreboot_driver {
	int (*probe)(struct coreboot_device *);
	int (*remove)(struct coreboot_device *);
	struct device_driver drv;
	u32 tag;
};

/* Register a driver that uses the data from a coreboot table. */
int coreboot_driver_register(struct coreboot_driver *driver);

/* Unregister a driver that uses the data from a coreboot table. */
void coreboot_driver_unregister(struct coreboot_driver *driver);

/* module_coreboot_driver() - Helper macro for drivers that don't do
 * anything special in module init/exit.  This eliminates a lot of
 * boilerplate.  Each module may only use this macro once, and
 * calling it replaces module_init() and module_exit()
 */
#define module_coreboot_driver(__coreboot_driver) \
	module_driver(__coreboot_driver, coreboot_driver_register, \
			coreboot_driver_unregister)

#endif /* __COREBOOT_TABLE_H */
