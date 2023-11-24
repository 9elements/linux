/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_APML_SBRMI_H_
#define _AMD_APML_SBRMI_H_

#include <linux/miscdevice.h>
#include <uapi/linux/amd-apml.h>

/* Each client has this additional data */
/* in_progress: Variable is set if any transaction is in
 * progress in IOCTL
 * no_new_trans: Variable is set if rmmmod/unbind is called
 */
struct apml_sbrmi_device {
	struct miscdevice sbrmi_misc_dev;
	struct completion misc_fops_done;
	struct i3c_device *i3cdev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;
	u32 *dimm_id;
	u32 pwr_limit_max;
	atomic_t in_progress;
	atomic_t no_new_trans;
	u8 rev;
	u8 sock_num;
} __packed;

int rmi_mca_msr_read(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg);
int rmi_cpuid_read(struct apml_sbrmi_device *rmi_dev,
		   struct apml_message *msg);
int rmi_mailbox_xfer(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg);
#endif /*_AMD_APML_SBRMI_H_*/
