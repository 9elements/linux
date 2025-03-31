// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for eSPI controller found on Aspeed SoCs.
 *
 * Copyright (C) 2025 9elements GmbH
 *
 * Author: Patrick Rudolph <patrick.rudolph@9elements.com>
 *
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <linux/mfd/aspeed-espi.h>

struct aspeed_espi_ctrl {
	struct device *dev;
	struct regmap_irq_chip_data *irq_data;
	struct regmap *regmap;
	struct clk *clk;
	int irq;
};

static const struct resource mafs_resources[] = {
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_FLASH_RX_CMPLT, "rx_cmplt"),
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_FLASH_TX_CMPLT, "tx_cmplt"),
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_FLASH_RX_ABT, "rx_abt"),
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_FLASH_TX_ABT, "tx_abt"),
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_FLASH_TX_ERR, "tx_err"),
};

static const struct resource vw_resources[] = {
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_VW_SYSEVT, "sysevt"),
	DEFINE_RES_IRQ_NAMED(ESPI_IRQ_VW_SYSEVT1, "sysevt1"),
};

static const struct mfd_cell cells[] = {
	{
		.name = "aspeed-espi-mafs",
		.num_resources = ARRAY_SIZE(mafs_resources),
		.resources = mafs_resources,
		.of_compatible = "aspeed,espi-mafs",
		.id = -1,
	},
	{
		.name = "aspeed-espi-vw-sysevt",
		.num_resources = ARRAY_SIZE(vw_resources),
		.resources = vw_resources,
		.of_compatible = "aspeed,espi-vw",
		.id = -1,
	},
};

static const struct regmap_irq aspeed_espi_irqs[] = {
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_PC_RX_CMPLT, 0, ESPI_INT_STS_PERIF_PC_RX_CMPLT),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_PC_TX_CMPLT, 0, ESPI_INT_STS_PERIF_PC_TX_CMPLT),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_NP_TX_CMPLT, 0, ESPI_INT_STS_PERIF_NP_TX_CMPLT),
	REGMAP_IRQ_REG(ESPI_IRQ_OOB_RX_CMPLT, 0, ESPI_INT_STS_OOB_RX_CMPLT),
	REGMAP_IRQ_REG(ESPI_IRQ_FLASH_RX_CMPLT, 0, ESPI_INT_STS_FLASH_RX_CMPLT),
	REGMAP_IRQ_REG(ESPI_IRQ_FLASH_TX_CMPLT, 0, ESPI_INT_STS_FLASH_TX_CMPLT),
	REGMAP_IRQ_REG(ESPI_IRQ_VW_SYSEVT, 0, ESPI_INT_STS_VW_SYSEVT),
	REGMAP_IRQ_REG(ESPI_IRQ_VW_GPIOEVT, 0, ESPI_INT_STS_VW_GPIOEVT),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_PC_TX_ERR, 0, ESPI_INT_STS_PERIF_PC_TX_ERR),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_NP_TX_ERR, 0, ESPI_INT_STS_PERIF_NP_TX_ERR),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_PC_RX_ABT, 0, ESPI_INT_STS_PERIF_PC_RX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_NP_RX_ABT, 0, ESPI_INT_STS_PERIF_NP_RX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_OOB_RX_ABT, 0, ESPI_INT_STS_OOB_RX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_FLASH_RX_ABT, 0, ESPI_INT_STS_FLASH_RX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_PC_TX_ABT, 0, ESPI_INT_STS_PERIF_PC_TX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_PERIF_NP_TX_ABT, 0, ESPI_INT_STS_PERIF_NP_TX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_OOB_TX_ABT, 0, ESPI_INT_STS_OOB_TX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_FLASH_TX_ABT, 0, ESPI_INT_STS_FLASH_TX_ABT),
	REGMAP_IRQ_REG(ESPI_IRQ_OOB_TX_ERR, 0, ESPI_INT_STS_OOB_TX_ERR),
	REGMAP_IRQ_REG(ESPI_IRQ_FLASH_TX_ERR, 0, ESPI_INT_STS_FLASH_TX_ERR),
	REGMAP_IRQ_REG(ESPI_IRQ_VW_SYSEVT1, 0, ESPI_INT_STS_VW_SYSEVT1),
	REGMAP_IRQ_REG(ESPI_IRQ_OOB_RX_TMOUT, 0, ESPI_INT_STS_OOB_RX_TMOUT),
	REGMAP_IRQ_REG(ESPI_IRQ_HW_RST_DEASSERT, 0, ESPI_INT_STS_HW_RST_DEASSERT),
};

static const struct regmap_irq_chip aspeed_espi_irq_chip = {
	.name			= "aspeed-espi",
	.status_base		= ESPI_INT_STS,
	.unmask_base		= ESPI_INT_EN,
	.ack_base		= ESPI_INT_STS,
	.num_regs		= 1,
	.irqs			= aspeed_espi_irqs,
	.num_irqs		= ARRAY_SIZE(aspeed_espi_irqs),
};

static inline bool aspeed_espi_ctrl_in_reset(struct regmap *regmap)
{
	u32 val;

	/* Cannot directly read the eSPI_RESET pin, thus or the channel enable
	 * bits. When all bits are clear the eSPI engine is in reset.
	 */
	regmap_read(regmap, ESPI_STS, &val);

	/* When at least one channel is enabled the eSPI engine is not in reset */
	return !(val & GENMASK(7, 4));
}

/* Reset IRQ happens on rising and falling edge. */
static irqreturn_t aspeed_espi_ctrl_reset_irq(int irq, void *data)
{
	struct aspeed_espi_ctrl *espi_ctrl = data;
	u32 val;
	regmap_read(espi_ctrl->regmap, ESPI_STS, &val);

	dev_err(espi_ctrl->dev, "IRQ %d ESPI_STS=%x\n", irq, val);

	mfd_remove_devices(espi_ctrl->dev);

	if (!aspeed_espi_ctrl_in_reset(espi_ctrl->regmap)) {
		dev_err(espi_ctrl->dev, "starting MTD devices...\n");

		WARN_ON(devm_mfd_add_devices(espi_ctrl->dev, PLATFORM_DEVID_NONE,
			cells, ARRAY_SIZE(cells), NULL, 0,
			regmap_irq_get_domain(espi_ctrl->irq_data)));
	}

	return IRQ_HANDLED;
}

static const struct regmap_config aspeed_espi_ctrl_regmap_config = {
	.fast_io = true,
	.max_register = 0x200,
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int aspeed_espi_ctrl_probe(struct platform_device *pdev)
{
	struct aspeed_espi_ctrl *espi_ctrl;
	struct device *dev = &pdev->dev;
	void __iomem *iomem;
	struct regmap *regmap;
	int ret, irq;

	espi_ctrl = devm_kzalloc(dev, sizeof(*espi_ctrl), GFP_KERNEL);
	if (!espi_ctrl)
		return -ENOMEM;

	dev_set_drvdata(dev, espi_ctrl);
	espi_ctrl->dev = dev;

	iomem = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(iomem))
		return PTR_ERR(iomem);

	regmap = devm_regmap_init_mmio(dev, iomem, &aspeed_espi_ctrl_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "cannot get remap: %ld\n", PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	espi_ctrl->regmap = regmap;

	espi_ctrl->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(espi_ctrl->clk)) {
		dev_err(dev, "cannot get clock: %ld\n", PTR_ERR(espi_ctrl->clk));
		return PTR_ERR(espi_ctrl->clk);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "cannot get IRQ: %d\n", irq);
		return irq;
	}

	/* clear the interrupt enable register */
	regmap_write(regmap, ESPI_INT_EN_CLR, 0xffffffff);
	regmap_write(regmap, ESPI_INT_STS, 0xffffffff);

	ret = devm_regmap_add_irq_chip(dev, regmap, irq, IRQF_ONESHOT, 0,
				       &aspeed_espi_irq_chip, &espi_ctrl->irq_data);
	if (ret) {
		dev_err(dev, "cannot enable IRQ chip: %d\n", ret);
		clk_disable_unprepare(espi_ctrl->clk);
		return ret;
	}
	espi_ctrl->irq = irq;

	/* Register reset IRQ notifier */
	irq = regmap_irq_get_virq(espi_ctrl->irq_data, ESPI_IRQ_HW_RST_DEASSERT);
	if (irq < 0) {
		dev_err(dev, "failed to get virtual IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, aspeed_espi_ctrl_reset_irq,
					0, pdev->name, espi_ctrl);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		return ret;
	}

	if (!aspeed_espi_ctrl_in_reset(espi_ctrl->regmap))
		WARN_ON(devm_mfd_add_devices(espi_ctrl->dev, PLATFORM_DEVID_NONE,
			cells, ARRAY_SIZE(cells), NULL, 0,
			regmap_irq_get_domain(espi_ctrl->irq_data)));

	/* When the Virtual Wire isn't exposed to userspace auto set TARGET_BOOT_DONE */
	if (!IS_ENABLED(CONFIG_GPIO_ASPEED_ESPI_VW)) {
		/*
		* eSPI specification:
		* TARGET_BOOT_DONE: Sent when EC or BMC has completed its boot process
		* as indication to eSPI controller to continue with the G3 to S0 exit.
		*/
		regmap_set_bits(espi_ctrl->regmap, ESPI_SYSEVT,
				ESPI_SYSEVT_TARGET_BOOT_STS | ESPI_SYSEVT_TARGET_BOOT_DONE);
	}

	return 0;
}

static void aspeed_espi_ctrl_remove(struct platform_device *pdev)
{
	struct aspeed_espi_ctrl *espi_ctrl = platform_get_drvdata(pdev);

	mfd_remove_devices(espi_ctrl->dev);

	if (!IS_ENABLED(CONFIG_GPIO_ASPEED_ESPI_VW))
		regmap_clear_bits(espi_ctrl->regmap, ESPI_SYSEVT, ESPI_SYSEVT_TARGET_BOOT_DONE);

	if (!IS_ERR_OR_NULL(espi_ctrl->clk))
		clk_unprepare(espi_ctrl->clk);

	if (espi_ctrl->irq_data)
		regmap_del_irq_chip(espi_ctrl->irq, espi_ctrl->irq_data);
}

static const struct of_device_id aspeed_espi_ctrl_of_match[] = {
	{ .compatible = "aspeed,ast2600-espi-ctrl" },
	{ .compatible = "aspeed,ast2500-espi-ctrl" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, aspeed_espi_ctrl_of_match);

static struct platform_driver aspeed_espi_mfd = {
	.probe	= aspeed_espi_ctrl_probe,
	.remove_new = aspeed_espi_ctrl_remove,
	.driver	= {
		.name		= "aspeed-espi-ctrl",
		.of_match_table	= aspeed_espi_ctrl_of_match,
	},
};

module_platform_driver(aspeed_espi_mfd);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("Aspeed eSPI Controller MFD driver");
MODULE_LICENSE("GPL v2");
