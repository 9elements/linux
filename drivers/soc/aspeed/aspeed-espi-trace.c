// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Aspeed AST2600 eSPI / PCIe / KCS boot trace module
 *
 * Polls SCU, eSPI, and LPC/KCS registers at high frequency and logs
 * every state change with nanosecond timestamps.  Designed to be loaded
 * on both a working and broken build so the two logs can be diff'd to
 * find timing-sensitive initialisation failures.
 *
 * Usage:
 *   modprobe aspeed-espi-trace            # start polling at 1ms
 *   modprobe aspeed-espi-trace poll_ms=5  # start polling at 5ms
 *   rmmod aspeed-espi-trace               # stop and dump final state
 *
 * Output goes to the kernel log (dmesg).  Use "dmesg -T" for wall-clock
 * timestamps or "dmesg -d" for delta timestamps between messages.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#define DRV_NAME "aspeed-espi-trace"

/* --- Hardware base addresses (AST2600) --- */
#define SCU_BASE	0x1E6E2000
#define SCU_SIZE	0x1000
#define ESPI_BASE	0x1E6EE000
#define ESPI_SIZE	0x200
#define LPC_BASE	0x1E789000
#define LPC_SIZE	0x200

/* --- SCU registers of interest --- */
#define SCU_IC0		0x560	/* PERST# / RCRST edge-detect + enable */
#define SCU_PCIE_CONF	0xC20	/* PCIe VGA/BMC device config */
#define SCU_PCIE_MSI	0xC24	/* PCIe MSI/INTx routing */

/* --- eSPI registers of interest --- */
#define ESPI_CTRL	0x000
#define ESPI_STS	0x004
#define ESPI_INT_STS	0x008
#define ESPI_INT_EN	0x00C
#define ESPI_CTRL2	0x080
#define ESPI_SYSEVT_IE	0x094
#define ESPI_SYSEVT	0x098
#define ESPI_GPIO_VAL	0x09C
#define ESPI_GEN_CAP	0x0A0
#define ESPI_CH0_CAP	0x0A4	/* Peripheral */
#define ESPI_CH1_CAP	0x0A8	/* Virtual Wire */
#define ESPI_CH2_CAP	0x0AC	/* OOB */
#define ESPI_CH3_CAP	0x0B0	/* Flash */
#define ESPI_SYSEVT1	0x104

/* --- LPC / KCS registers of interest --- */
#define LPC_HICR0	0x000	/* Host Interface Control 0 */
#define LPC_HICR2	0x008	/* Host Interface Control 2 (LPC/eSPI mode) */
#define LPC_HICR4	0x010	/* Host Interface Control 4 */
#define KCS_STR1	0x03C	/* KCS channel 1 status */
#define KCS_STR2	0x040	/* KCS channel 2 status */
#define KCS_STR3	0x044	/* KCS channel 3 status */
#define KCS_STR4	0x11C	/* KCS channel 4 status */

/* --- Snapshot structure --- */
struct reg_entry {
	u16  offset;		/* register offset from its base */
	u8   base_idx;		/* 0=SCU, 1=eSPI, 2=LPC */
	const char *name;
};

static const struct reg_entry watched_regs[] = {
	/* SCU */
	{ SCU_IC0,       0, "SCU_IC0" },
	{ SCU_PCIE_CONF, 0, "SCU_C20" },
	{ SCU_PCIE_MSI,  0, "SCU_C24" },
	/* eSPI */
	{ ESPI_CTRL,     1, "ESPI_CTRL" },
	{ ESPI_STS,      1, "ESPI_STS" },
	{ ESPI_INT_STS,  1, "ESPI_INT_STS" },
	{ ESPI_INT_EN,   1, "ESPI_INT_EN" },
	{ ESPI_CTRL2,    1, "ESPI_CTRL2" },
	{ ESPI_SYSEVT_IE,1, "ESPI_SYSEVT_IE" },
	{ ESPI_SYSEVT,   1, "ESPI_SYSEVT" },
	{ ESPI_GPIO_VAL, 1, "ESPI_GPIO_VAL" },
	{ ESPI_GEN_CAP,  1, "ESPI_GEN_CAP" },
	{ ESPI_CH0_CAP,  1, "ESPI_CH0_CAP" },
	{ ESPI_CH1_CAP,  1, "ESPI_CH1_CAP" },
	{ ESPI_CH2_CAP,  1, "ESPI_CH2_CAP" },
	{ ESPI_CH3_CAP,  1, "ESPI_CH3_CAP" },
	{ ESPI_SYSEVT1,  1, "ESPI_SYSEVT1" },
	/* LPC / KCS */
	{ LPC_HICR0,     2, "LPC_HICR0" },
	{ LPC_HICR2,     2, "LPC_HICR2" },
	{ LPC_HICR4,     2, "LPC_HICR4" },
	{ KCS_STR1,      2, "KCS_STR1" },
	{ KCS_STR2,      2, "KCS_STR2" },
	{ KCS_STR3,      2, "KCS_STR3" },
	{ KCS_STR4,      2, "KCS_STR4" },
};

#define NUM_REGS ARRAY_SIZE(watched_regs)

static void __iomem *bases[3];		/* SCU, eSPI, LPC */
static u32 prev[NUM_REGS];
static struct hrtimer poll_timer;
static unsigned int poll_ms = 1;
module_param(poll_ms, uint, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval in milliseconds (default 1)");

static void dump_all(const char *tag)
{
	int i;

	for (i = 0; i < NUM_REGS; i++) {
		const struct reg_entry *r = &watched_regs[i];
		u32 val = readl(bases[r->base_idx] + r->offset);

		pr_warn(DRV_NAME ": [%s] %-16s = 0x%08x\n", tag, r->name, val);
		prev[i] = val;
	}
}

/* --- Decode helpers for the most interesting registers --- */

static void decode_scu_ic0(u32 old, u32 new)
{
	u32 changed = old ^ new;

	if (changed & BIT(18))
		pr_warn(DRV_NAME ":   >> PCIE_PERST# LO->HI (deassert) %s\n",
			(new & BIT(18)) ? "DETECTED" : "cleared");
	if (changed & BIT(19))
		pr_warn(DRV_NAME ":   >> PCIE_PERST# HI->LO (assert) %s\n",
			(new & BIT(19)) ? "DETECTED" : "cleared");
	if (changed & BIT(20))
		pr_warn(DRV_NAME ":   >> PCIE_RCRST LO->HI %s\n",
			(new & BIT(20)) ? "DETECTED" : "cleared");
	if (changed & BIT(21))
		pr_warn(DRV_NAME ":   >> PCIE_RCRST HI->LO %s\n",
			(new & BIT(21)) ? "DETECTED" : "cleared");
}

static void decode_espi_ctrl(u32 old, u32 new)
{
	u32 changed = old ^ new;

	if (changed & BIT(1))
		pr_warn(DRV_NAME ":   >> PERIF_SW_RDY: %d -> %d\n",
			!!(old & BIT(1)), !!(new & BIT(1)));
	if (changed & BIT(3))
		pr_warn(DRV_NAME ":   >> VW_SW_RDY: %d -> %d\n",
			!!(old & BIT(3)), !!(new & BIT(3)));
	if (changed & BIT(4))
		pr_warn(DRV_NAME ":   >> OOB_SW_RDY: %d -> %d\n",
			!!(old & BIT(4)), !!(new & BIT(4)));
	if (changed & BIT(7))
		pr_warn(DRV_NAME ":   >> FLASH_SW_RDY: %d -> %d\n",
			!!(old & BIT(7)), !!(new & BIT(7)));
	if (changed & BIT(9))
		pr_warn(DRV_NAME ":   >> VW_GPIO_SW: %d -> %d\n",
			!!(old & BIT(9)), !!(new & BIT(9)));
	if (changed & GENMASK(11, 10))
		pr_warn(DRV_NAME ":   >> FLASH_EDAF_MODE: %d -> %d\n",
			(old >> 10) & 3, (new >> 10) & 3);
}

static void decode_espi_sysevt(u32 old, u32 new)
{
	u32 changed = old ^ new;

	if (changed & BIT(5))
		pr_warn(DRV_NAME ":   >> PLTRSTN: %d -> %d (%s)\n",
			!!(old & BIT(5)), !!(new & BIT(5)),
			(new & BIT(5)) ? "host running" : "host in reset");
	if (changed & BIT(20))
		pr_warn(DRV_NAME ":   >> SLV_BOOT_DONE: %d -> %d\n",
			!!(old & BIT(20)), !!(new & BIT(20)));
	if (changed & BIT(23))
		pr_warn(DRV_NAME ":   >> SLV_BOOT_STS: %d -> %d\n",
			!!(old & BIT(23)), !!(new & BIT(23)));
	if (changed & BIT(8))
		pr_warn(DRV_NAME ":   >> HOST_RST_WARN: %d -> %d\n",
			!!(old & BIT(8)), !!(new & BIT(8)));
	if (changed & BIT(27))
		pr_warn(DRV_NAME ":   >> HOST_RST_ACK: %d -> %d\n",
			!!(old & BIT(27)), !!(new & BIT(27)));
}

static void decode_espi_ctrl2(u32 old, u32 new)
{
	u32 changed = old ^ new;

	if (changed & BIT(8))
		pr_warn(DRV_NAME ":   >> AUTO_SLV_BOOT_STS: %d -> %d\n",
			!!(old & BIT(8)), !!(new & BIT(8)));
	if (changed & BIT(9))
		pr_warn(DRV_NAME ":   >> AUTO_SLV_BOOT_DONE: %d -> %d\n",
			!!(old & BIT(9)), !!(new & BIT(9)));
}

static void decode_espi_int_sts(u32 old, u32 new)
{
	u32 changed = old ^ new;

	if (changed & BIT(31))
		pr_warn(DRV_NAME ":   >> HW_RST_EVENT: %d -> %d\n",
			!!(old & BIT(31)), !!(new & BIT(31)));
	if (changed & BIT(8))
		pr_warn(DRV_NAME ":   >> VW_SYSEVT: %d -> %d\n",
			!!(old & BIT(8)), !!(new & BIT(8)));
	if (changed & BIT(9))
		pr_warn(DRV_NAME ":   >> VW_GPIO: %d -> %d\n",
			!!(old & BIT(9)), !!(new & BIT(9)));
}

static void decode_espi_sts(u32 old, u32 new)
{
	u32 changed = old ^ new;

	if (changed & BIT(15))
		pr_warn(DRV_NAME ":   >> ESPI_RSTN_PIN: %d -> %d (%s)\n",
			!!(old & BIT(15)), !!(new & BIT(15)),
			(new & BIT(15)) ? "deasserted" : "asserted");
	if (changed & BIT(0))
		pr_warn(DRV_NAME ":   >> ESPI_EN: %d -> %d\n",
			!!(old & BIT(0)), !!(new & BIT(0)));
}

static enum hrtimer_restart poll_callback(struct hrtimer *timer)
{
	int i;

	for (i = 0; i < NUM_REGS; i++) {
		const struct reg_entry *r = &watched_regs[i];
		u32 val = readl(bases[r->base_idx] + r->offset);

		if (val != prev[i]) {
			pr_warn(DRV_NAME ": %-16s 0x%08x -> 0x%08x\n",
				r->name, prev[i], val);

			/* Decode interesting fields */
			if (r->base_idx == 0 && r->offset == SCU_IC0)
				decode_scu_ic0(prev[i], val);
			else if (r->base_idx == 1 && r->offset == ESPI_CTRL)
				decode_espi_ctrl(prev[i], val);
			else if (r->base_idx == 1 && r->offset == ESPI_STS)
				decode_espi_sts(prev[i], val);
			else if (r->base_idx == 1 && r->offset == ESPI_INT_STS)
				decode_espi_int_sts(prev[i], val);
			else if (r->base_idx == 1 && r->offset == ESPI_CTRL2)
				decode_espi_ctrl2(prev[i], val);
			else if (r->base_idx == 1 && r->offset == ESPI_SYSEVT)
				decode_espi_sysevt(prev[i], val);

			prev[i] = val;
		}
	}

	hrtimer_forward_now(timer, ms_to_ktime(poll_ms));
	return HRTIMER_RESTART;
}

static int __init aspeed_espi_trace_init(void)
{
	bases[0] = ioremap(SCU_BASE, SCU_SIZE);
	bases[1] = ioremap(ESPI_BASE, ESPI_SIZE);
	bases[2] = ioremap(LPC_BASE, LPC_SIZE);

	if (!bases[0] || !bases[1] || !bases[2]) {
		pr_err(DRV_NAME ": failed to map registers\n");
		if (bases[0]) iounmap(bases[0]);
		if (bases[1]) iounmap(bases[1]);
		if (bases[2]) iounmap(bases[2]);
		return -ENOMEM;
	}

	pr_warn(DRV_NAME ": starting trace (poll every %u ms)\n", poll_ms);
	dump_all("INIT");

	hrtimer_init(&poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	poll_timer.function = poll_callback;
	hrtimer_start(&poll_timer, ms_to_ktime(poll_ms), HRTIMER_MODE_REL);

	return 0;
}

static void __exit aspeed_espi_trace_exit(void)
{
	hrtimer_cancel(&poll_timer);

	pr_warn(DRV_NAME ": stopping trace\n");
	dump_all("EXIT");

	iounmap(bases[0]);
	iounmap(bases[1]);
	iounmap(bases[2]);
}

module_init(aspeed_espi_trace_init);
module_exit(aspeed_espi_trace_exit);

MODULE_DESCRIPTION("AST2600 eSPI/PCIe/KCS boot trace for timing analysis");
MODULE_AUTHOR("9elements GmbH");
MODULE_LICENSE("GPL");
