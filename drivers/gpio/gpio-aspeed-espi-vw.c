// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 ASPEED Technology Inc.
 */
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/gpio/driver.h>

#include <linux/mfd/aspeed-espi.h>

#define DRV_NAME  "aspeed-espi-vw-sysevt"

#define ASPEED_ESPI_SYSEVENTS_GPIOS 56

static const char *aspeed_espi_sysevt_gpio_names[ASPEED_ESPI_SYSEVENTS_GPIOS] = {
	"SLP_S3", "SLP_S4", "SLP_S5", "", "SLP_SUS", "PLTRST_N", "OOB_RST_WARN", "",
	"HOST_RST_WARN", "SMI_OUT", "NMI_OUT", "", "", "", "", "",
	"OOB_RST_ACK", "", "", "", "SLAVE_BOOT_DONE", "FATAL_ERR", "NON_FATAL_ERR", "SLAVE_BOOT_STATUS",
	"", "", "RCIN", "HOST_RST_ACK", "", "", "", "",

	"SUS_WARN", "SUS_PWRDN_ACK", "", "SLP_A", "SLP_LAN", "SLP_WIFI", "", "",
	"", "", "", "", "", "", "", "",
	"HOST_C10", "", "", "", "SUS_ACK", "", "", "",
};

/* Direction is fixed */
#define ASPEED_ESPI_SYSEVT_DIRECTION		0x00000777
#define ASPEED_ESPI_SYSEVT1_DIRECTION		0x0000103b

/**
 * struct aspeed_espi_sysevt - driver data
 * @regmap:         Device's regmap. Only direct access registers.
 */
struct aspeed_espi_vw_sysevt {
	struct gpio_chip gpio_chip;
	struct mutex lock;
	struct regmap *regmap;
	enum aspeed_espi_version version;
};

/**
 * aspeed_espi_sysevt_ready - Returns true when eSPI VW can be used
 * Since there's no interrupt when the eSPI HW is put into reset
 * status BITs must be checked if the MAFS channel is ready for use.
 *
 * @dev: MFD device to check
 *
 */
static bool aspeed_espi_sysevt_ready(struct device *dev)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = dev_get_drvdata(dev);
	u32 reg;

	/* Test if vw channel is ready */
	regmap_read(espi_gpio->regmap, ESPI_STS, &reg);

	if (!(reg & ESPI_CTRL_ENABLED)) {
		dev_info(dev, "eSPI controller not ready\n");
		return false;
	}
	if (!(reg & ESPI_CTRL_VW_CHAN_ENABLE)) {
		dev_info(dev, "eSPI vw channel not enabled\n");
		return false;
	}

	regmap_read(espi_gpio->regmap, ESPI_CTRL, &reg);

	if (!(reg & ESPI_CTRL_VW_RDY)) {
		dev_info(dev, "eSPI vw channel not ready\n");
		return false;
	}
	if (!(reg & ESPI_CTRL_RESET_LVL)) {
		dev_info(dev, "eSPI controller is in reset\n");
		return false;
	}

	return true;
}

static void aspeed_espi_vw_sysevt_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(gc);
	int reg = (d->hwirq < 32) ? ESPI_SYSEVT_INT_STS : ESPI_SYSEVT1_INT_STS;
	u32 bit = (d->hwirq < 32) ? BIT(d->hwirq) : BIT(d->hwirq - 32);

	regmap_set_bits(espi_gpio->regmap, reg, bit);
}

static void aspeed_espi_vw_sysevt_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(gc);
	int reg = (d->hwirq < 32) ? ESPI_SYSEVT_INT_EN : ESPI_SYSEVT1_INT_EN;
	u32 bit = (d->hwirq < 32) ? BIT(d->hwirq) : BIT(d->hwirq - 32);

	regmap_clear_bits(espi_gpio->regmap, reg, bit);
	gpiochip_disable_irq(gc, irqd_to_hwirq(d));
}

static void aspeed_espi_vw_sysevt_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(gc);
	int reg = (d->hwirq < 32) ? ESPI_SYSEVT_INT_EN : ESPI_SYSEVT1_INT_EN;
	u32 bit = (d->hwirq < 32) ? BIT(d->hwirq) : BIT(d->hwirq - 32);

	gpiochip_enable_irq(gc, irqd_to_hwirq(d));
	regmap_set_bits(espi_gpio->regmap, reg, bit);
}

static int aspeed_espi_vw_sysevt_irq_set_type(struct irq_data *d, u32 type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(gc);
	int reg = (d->hwirq < 32) ? ESPI_SYSEVT_INT_T0 : ESPI_SYSEVT1_INT_T0;
	u32 bit = (d->hwirq < 32) ? BIT(d->hwirq) : BIT(d->hwirq - 32);

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
		regmap_set_bits(espi_gpio->regmap, reg + 8, bit);
	else
		regmap_clear_bits(espi_gpio->regmap, reg + 8, bit);

	if (type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
		regmap_set_bits(espi_gpio->regmap, reg + 4, bit);
	else
		regmap_clear_bits(espi_gpio->regmap, reg + 4, bit);

	if (type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_EDGE_RISING))
		regmap_set_bits(espi_gpio->regmap, reg, bit);
	else
		regmap_clear_bits(espi_gpio->regmap, reg, bit);

	return 0;
}

static const struct irq_chip aspeed_espi_vw_sysevt_irq_chip = {
	.name = "aspeed-espi-sysevt-gpio",
	.irq_ack = aspeed_espi_vw_sysevt_irq_ack,
	.irq_mask = aspeed_espi_vw_sysevt_irq_mask,
	.irq_unmask = aspeed_espi_vw_sysevt_irq_unmask,
	.irq_set_type = aspeed_espi_vw_sysevt_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static irqreturn_t aspeed_espi_vw_sysevt_irq_sysevt(int irq, void *data)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = data;
	struct gpio_chip *gc = &espi_gpio->gpio_chip;
	unsigned long sysevt_sts;
	irq_hw_number_t hwirq;
	u32 int_sts;

	regmap_read(espi_gpio->regmap, ESPI_SYSEVT_INT_STS, &int_sts);

	/* On AST2600 those are automatically ACKed by HW */
	if (espi_gpio->version == ESPI_AST2500) {
		if (int_sts & ESPI_SYSEVT_INT_STS_HOST_RST_WARN)
			regmap_set_bits(espi_gpio->regmap, ESPI_SYSEVT,
					ESPI_SYSEVT_HOST_RST_ACK);

		if (int_sts & ESPI_SYSEVT_INT_STS_OOB_RST_WARN)
			regmap_set_bits(espi_gpio->regmap, ESPI_SYSEVT,
					ESPI_SYSEVT_OOB_RST_ACK);
	}

	regmap_write(espi_gpio->regmap, ESPI_SYSEVT_INT_STS, int_sts);

	sysevt_sts = int_sts;
	for_each_set_bit(hwirq, &sysevt_sts, 32) {
		int nested_irq = irq_find_mapping(gc->irq.domain, hwirq);

		if (unlikely(nested_irq <= 0))
			continue;

		handle_nested_irq(nested_irq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_vw_sysevt_irq_sysevt1(int irq, void *data)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = data;
	struct gpio_chip *gc = &espi_gpio->gpio_chip;
	unsigned long sysevt1_sts;
	irq_hw_number_t hwirq;
	u32 int_sts;

	regmap_read(espi_gpio->regmap, ESPI_SYSEVT1_INT_STS, &int_sts);

	/* On AST2600 those are automatically ACKed by HW */
	if (espi_gpio->version == ESPI_AST2500) {
		if (int_sts & ESPI_SYSEVT1_INT_STS_SUSPEND_WARN)
			regmap_set_bits(espi_gpio->regmap, ESPI_SYSEVT1,
					ESPI_SYSEVT1_SUSPEND_ACK);
	}

	regmap_write(espi_gpio->regmap, ESPI_SYSEVT1_INT_STS, int_sts);

	sysevt1_sts = int_sts;
	for_each_set_bit(hwirq, &sysevt1_sts, 32) {
		int nested_irq = irq_find_mapping(gc->irq.domain, hwirq + 32);

		if (unlikely(nested_irq <= 0))
			continue;

		handle_nested_irq(nested_irq);
	}

	return IRQ_HANDLED;
}

static int aspeed_espi_sysevt_gpio_get_dir(struct gpio_chip *chip, unsigned int offset)
{
	u32 dir = (offset < 32) ? ASPEED_ESPI_SYSEVT_DIRECTION : ASPEED_ESPI_SYSEVT1_DIRECTION;

	if (offset >= 32)
		offset -= 32;

	if (BIT(offset) & dir)
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int aspeed_espi_sysevt_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(chip);
	int reg = (offset < 32) ? ESPI_SYSEVT : ESPI_SYSEVT1;
	u32 val;
	int ret;

	if (!aspeed_espi_sysevt_ready(chip->parent))
		return -EIO;

	if (aspeed_espi_sysevt_gpio_get_dir(chip, offset) == GPIO_LINE_DIRECTION_OUT)
		return -EINVAL;

	if (offset >= 32)
		offset -= 32;

	ret = regmap_read(espi_gpio->regmap, reg, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset));
}

static void aspeed_espi_sysevt_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(chip);
	int reg = (offset < 32) ? ESPI_SYSEVT : ESPI_SYSEVT1;

	if (aspeed_espi_sysevt_gpio_get_dir(chip, offset) == GPIO_LINE_DIRECTION_IN)
		return;

	if (offset >= 32)
		offset -= 32;

	regmap_update_bits(espi_gpio->regmap, reg, BIT(offset), value ? BIT(offset) : 0);
}

static int aspeed_espi_sysevt_gpio_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct aspeed_espi_ctrl *ctrl = dev_get_drvdata(pdev->dev.parent);
	struct aspeed_espi_vw_sysevt *espi_gpio;
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *girq;
	struct gpio_chip *gc;
	int ret, irq;

	espi_gpio = devm_kzalloc(dev, sizeof(struct aspeed_espi_vw_sysevt), GFP_KERNEL);
	if (!espi_gpio)
		return -ENOMEM;

	dev_set_drvdata(dev, espi_gpio);
	espi_gpio->regmap = regmap;
	espi_gpio->version = ctrl->version;

	gc = &espi_gpio->gpio_chip;
	gc->base = -1;
	gc->can_sleep = 1;
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->get = aspeed_espi_sysevt_gpio_get;
	gc->set = aspeed_espi_sysevt_gpio_set;
	gc->get_direction = aspeed_espi_sysevt_gpio_get_dir;
	gc->ngpio = ASPEED_ESPI_SYSEVENTS_GPIOS;
	gc->names = aspeed_espi_sysevt_gpio_names;

	girq = &gc->irq;
	gpio_irq_chip_set_chip(girq, &aspeed_espi_vw_sysevt_irq_chip);
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;

	ret = devm_gpiochip_add_data(dev, gc, espi_gpio);
	if (ret) {
		dev_err(dev, "Failed to register gpio chip (%d)\n", ret);
		return ret;
	}

	/* Request virtual interrupts */
	int pirq = ESPI_INT_EN_VW_SYSEVT_SHIFT;
	irq = regmap_irq_get_virq(ctrl->irq_data, pirq);
	if (irq < 0) {
		dev_err(dev, "failed to get virtual interrupt=%d\n", pirq);
		ret = irq;
		return ret;
	}

	ret = devm_request_irq(dev, irq, aspeed_espi_vw_sysevt_irq_sysevt,
			       IRQF_ONESHOT, pdev->name, espi_gpio);
	if (ret) {
		dev_err(dev, "failed to request virtual interrupt=%d: %d\n", pirq, ret);
		return ret;
	}

	pirq = ESPI_INT_EN_VW_SYSEVT1_SHIFT;
	irq = regmap_irq_get_virq(ctrl->irq_data, pirq);
	if (irq < 0) {
		dev_err(dev, "failed to get virtual interrupt=%d\n", pirq);
		ret = irq;
		return ret;
	}

	ret = devm_request_irq(dev, irq, aspeed_espi_vw_sysevt_irq_sysevt1,
			       IRQF_ONESHOT, pdev->name, espi_gpio);
	if (ret) {
		dev_err(dev, "failed to request virtual interrupt=%d: %d\n", pirq, ret);
		return ret;
	}

	/* Set VW software ready */
	regmap_set_bits(espi_gpio->regmap, ESPI_CTRL, ESPI_CTRL_VW_SW_RDY);

	return 0;
}

static void aspeed_espi_sysevt_gpio_remove(struct platform_device *pdev)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
}

static struct platform_driver aspeed_espi_sysevt_gpio_driver = {
	.driver = {
		.name = DRV_NAME,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.remove_new = aspeed_espi_sysevt_gpio_remove,
	.probe = aspeed_espi_sysevt_gpio_probe,
};

module_platform_driver(aspeed_espi_sysevt_gpio_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("Aspeed SoC eSPI MAFS controller driver");
MODULE_LICENSE("GPL v2");
