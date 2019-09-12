/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * bootmedia-coreboot.h
 *
 * Copyright 2019 9elements Agency GmbH <patrick.rudolph@9elements.com>
 */

#ifndef __FMAP_COREBOOT_H
#define __FMAP_COREBOOT_H

const char *coreboot_fmap_region_to_name(const u32 start, const u32 size);

#endif /* __FMAP_COREBOOT_H */
