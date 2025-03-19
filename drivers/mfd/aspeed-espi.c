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

static const struct mfd_cell cells[] = {
	MFD_CELL_NAME("aspeed-espi-mafs"),
	MFD_CELL_NAME("aspeed-espi-virtualwire"),
};

static const struct regmap_irq aspeed_espi_irqs[] = {
	{ .reg_offset = 0, .mask = ESPI_INT_STS_PERIF_PC_RX_CMPLT, },
	{ .reg_offset = 0, .mask = ESPI_INT_STS_PERIF_PC_TX_CMPLT, },
	{ .reg_offset = 0, .mask = ESPI_INT_STS_PERIF_NP_TX_CMPLT, },
	{ .reg_offset = 0, .mask = ESPI_INT_STS_OOB_RX_CMPLT, },
	{ .reg_offset = 0, .mask = ESPI_INT_STS_OOB_TX_CMPLT, },
	{ .reg_offset = 0, .mask = ESPI_INT_STS_FLASH_RX_CMPLT, },
	{ .reg_offset = 0, .mask = ESPI_INT_STS_FLASH_TX_CMPLT, },

	{ .reg_offset = 1, .mask = ESPI_INT_EN_VW_SYSEVT >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_VW_GPIOEVT >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_PERIF_PC_TX_ERR >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_PERIF_NP_TX_ERR >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_PERIF_PC_RX_ABT >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_PERIF_NP_RX_ABT >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_OOB_RX_ABT >> 8, },
	{ .reg_offset = 1, .mask = ESPI_INT_STS_FLASH_RX_ABT >> 8, },

	{ .reg_offset = 2, .mask = ESPI_INT_STS_PERIF_NP_TX_ABT >> 16, },
	{ .reg_offset = 2, .mask = ESPI_INT_STS_OOB_TX_ABT >> 16, },
	{ .reg_offset = 2, .mask = ESPI_INT_STS_FLASH_TX_ABT >> 16, },
	{ .reg_offset = 2, .mask = ESPI_INT_STS_OOB_TX_ERR >> 16, },
	{ .reg_offset = 2, .mask = ESPI_INT_STS_FLASH_TX_ERR >> 16, },
	{ .reg_offset = 2, .mask = ESPI_INT_STS_VW_SYSEVT1 >> 16, },
	{ .reg_offset = 2, .mask = ESPI_INT_STS_OOB_RX_TMOUT >> 16, },

	{ .reg_offset = 3, .mask = ESPI_INT_STS_HW_RST_DEASSERT >> 24, },
};

static const struct regmap_irq_chip aspeed_espi_irq_chip = {
	.name			= "aspeed-espi",
	.status_base		= ESPI_INT_STS,
	.mask_base		= ESPI_INT_EN,
	.ack_base		= ESPI_INT_EN_CLR,
	.num_regs		= 4,
	.irqs			= aspeed_espi_irqs,
	.num_irqs		= ARRAY_SIZE(aspeed_espi_irqs),
};

static irqreturn_t aspeed_espi_ctrl_irq(int irq, void *data)
{
	struct aspeed_espi_ctrl *espi_ctrl = data;

	if (irq != ESPI_INT_STS_HW_RST_DEASSERT)
		return IRQ_NONE;

	mfd_remove_devices(espi_ctrl->dev);

	WARN_ON(devm_mfd_add_devices(espi_ctrl->dev, PLATFORM_DEVID_AUTO, cells, ARRAY_SIZE(cells),
		NULL, 0, regmap_irq_get_domain(espi_ctrl->irq_data)));

	return IRQ_HANDLED;
}

static int aspeed_espi_ctrl_probe(struct platform_device *pdev)
{
	struct aspeed_espi_ctrl *espi_ctrl;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	int rc, irq;
	u32 val;

	espi_ctrl = devm_kzalloc(dev, sizeof(*espi_ctrl), GFP_KERNEL);
	if (!espi_ctrl)
		return -ENOMEM;

	dev_set_drvdata(dev, espi_ctrl);

	espi_ctrl->dev = dev;

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap)) {
		dev_err(dev, "cannot get remap: %ld\n", PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "cannot get IRQ: %d\n", irq);
		return irq;
	}

	espi_ctrl->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(espi_ctrl->clk)) {
		dev_err(dev, "cannot get clock: %ld\n", PTR_ERR(espi_ctrl->clk));
		return PTR_ERR(espi_ctrl->clk);
	}

	rc = clk_prepare_enable(espi_ctrl->clk);
	if (rc) {
		dev_err(dev, "cannot enable clock: %d\n", rc);
		return rc;
	}

	// clear the interrupt enable register
	regmap_write(regmap, ESPI_INT_EN_CLR, 0xffffffff);
	regmap_write(regmap, ESPI_INT_STS, 0xffffffff);

	rc = devm_regmap_add_irq_chip(dev, regmap, irq, IRQF_ONESHOT, 0,
				      &aspeed_espi_irq_chip, &espi_ctrl->irq_data);
	if (rc) {
		dev_err(dev, "cannot enable IRQ chip: %d\n", rc);
		clk_disable_unprepare(espi_ctrl->clk);
		return rc;
	}

	/* Register reset notifier */
	irq = regmap_irq_get_virq(espi_ctrl->irq_data, ESPI_INT_EN_HW_RST_DEASSERT_SHIFT);
	if (irq < 0) {
		dev_err(dev, "failed to get virtual: %d\n", irq);
		return irq;
	}

	rc = devm_request_threaded_irq(dev, irq, NULL, aspeed_espi_ctrl_irq,
				       IRQF_ONESHOT, pdev->name, espi_ctrl);
	if (rc) {
		dev_err(dev, "failed to request interrupt=31: %d\n", rc);
		return rc;
	}

	/* When HW is already out of reset then call mfd_add_devices */
	regmap_read(regmap, ESPI_CTRL, &val);
	if (!(val & ESPI_CTRL_RESET_LVL)) {
		rc = devm_mfd_add_devices(espi_ctrl->dev, PLATFORM_DEVID_AUTO,
					  cells, ARRAY_SIZE(cells), NULL, 0,
					  regmap_irq_get_domain(espi_ctrl->irq_data));
		if (rc)
			return rc;
	}

	return 0;
}

static void aspeed_espi_ctrl_remove(struct platform_device *pdev)
{
	struct aspeed_espi_ctrl *espi_ctrl = platform_get_drvdata(pdev);

	mfd_remove_devices(espi_ctrl->dev);

	if (!IS_ERR_OR_NULL(espi_ctrl->clk))
		clk_unprepare(espi_ctrl->clk);

	if (espi_ctrl->irq_data)
		regmap_del_irq_chip(espi_ctrl->irq, espi_ctrl->irq_data);
}

static const struct of_device_id aspeed_espi_ctrl_of_match[] = {
	{ .compatible = "aspeed,ast2600-espi-ctrl", .data = (void *)ESPI_AST2500 },
	{ .compatible = "aspeed,ast2500-espi-ctrl", .data = (void *)ESPI_AST2600 },
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
