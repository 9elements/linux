/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2010, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *    * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *    * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 */

#ifndef FLASHMAP_SERIALIZED_H__
#define FLASHMAP_SERIALIZED_H__

#define FMAP_SIGNATURE		"__FMAP__"
#define FMAP_VER_MAJOR		1	/* this header's FMAP major version */
#define FMAP_VER_MINOR		1	/* this header's FMAP minor version */
#define FMAP_STRLEN		32	/* maximum length for strings,
					 * including null-terminator
					 */

enum fmap_flags {
	FMAP_AREA_STATIC	= 1 << 0,
	FMAP_AREA_COMPRESSED	= 1 << 1,
	FMAP_AREA_RO		= 1 << 2,
	FMAP_AREA_PRESERVE	= 1 << 3,
};

/* Mapping of volatile and static regions in firmware binary */
struct fmap_area {
	u32 offset;                /* offset relative to base */
	u32 size;                  /* size in bytes */
	u8  name[FMAP_STRLEN];     /* descriptive name */
	u16 flags;                 /* flags for this area */
} __packed;

struct fmap {
	u8  signature[8];	/* "__FMAP__" (0x5F5F464D41505F5F) */
	u8  ver_major;		/* major version */
	u8  ver_minor;		/* minor version */
	u64 base;		/* address of the firmware binary */
	u32 size;		/* size of firmware binary in bytes */
	u8  name[FMAP_STRLEN];	/* name of this firmware binary */
	u16 nareas;		/* number of areas described by
				 * fmap_areas[] below
				 */
	struct fmap_area areas[];
} __packed;

#endif	/* FLASHMAP_SERIALIZED_H__ */
