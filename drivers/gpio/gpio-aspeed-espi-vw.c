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

#define ASPEED_ESPI_VW_TIMEOUT msecs_to_jiffies(100)
#define ASPEED_ESPI_SYSEVENTS_GPIOS 56

static const char *aspeed_espi_sysevt_gpio_names[ASPEED_ESPI_SYSEVENTS_GPIOS] = {
	"SLP_S3_N", "SLP_S4_N", "SLP_S5_N", "", "SLP_SUS", "PLTRST_N", "OOB_RST_WARN", "",
	"HOST_RST_WARN", "SMI_OUT", "NMI_OUT", "", "", "", "", "",
	"OOB_RST_ACK", "", "", "", "TARGET_BOOT_DONE", "FATAL_ERR", "NON_FATAL_ERR", "TARGET_BOOT_STATUS",
	"", "", "RCIN", "HOST_RST_ACK", "", "", "", "",

	"SUS_WARN", "SUS_PWRDN_ACK", "", "SLP_A", "SLP_LAN", "SLP_WIFI", "", "",
	"", "", "", "", "", "", "", "",
	"HOST_C10", "", "", "", "SUS_ACK", "", "", "",
};

enum aspeed_espi_version {
	ESPI_AST2500,
	ESPI_AST2600,
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
	bool auto_ack_warnings;
	bool auto_ack_target_boot_done;
};

/**
 * aspeed_espi_sysevt_ready - Returns true when eSPI VW can be used
 *
 * @dev: device to check
 * @timeout: timeout value in jiffies
 *
 */
static int aspeed_espi_sysevt_ready(struct device *dev, unsigned long timeout)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = dev_get_drvdata(dev);
	u32 cap_reg;

	timeout += jiffies;
	do {
		/* Test if VW channel is ready */
		regmap_read(espi_gpio->regmap, ESPI_CH1_CAP_N_CONF, &cap_reg);
		if ((cap_reg & (ESPI_CH1_CAP_N_READY | ESPI_CH1_CAP_N_ENABLED)) ==
		    (ESPI_CH1_CAP_N_READY | ESPI_CH1_CAP_N_ENABLED))
			return 0;

		msleep(1);
	} while (time_before(jiffies, timeout));

	dev_err(dev, "timed out waiting for HW ready\n");

	return -ETIMEDOUT;
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
	if (espi_gpio->auto_ack_warnings && espi_gpio->version == ESPI_AST2500) {
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
	if (espi_gpio->auto_ack_warnings && espi_gpio->version == ESPI_AST2500) {
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

	ret = aspeed_espi_sysevt_ready(chip->parent, ASPEED_ESPI_VW_TIMEOUT);
	if (ret)
		return ret;

	if (aspeed_espi_sysevt_gpio_get_dir(chip, offset) == GPIO_LINE_DIRECTION_OUT)
		return -EINVAL;

	if (offset >= 32)
		offset -= 32;

	ret = regmap_read(espi_gpio->regmap, reg, &val);
	if (ret)
		return ret;
	dev_err(chip->parent, "reg %x = %x\n", reg, val);

	return !!(val & BIT(offset));
}

static void aspeed_espi_sysevt_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(chip);
	int reg = (offset < 32) ? ESPI_SYSEVT : ESPI_SYSEVT1;
	int ret;

	ret = aspeed_espi_sysevt_ready(chip->parent, ASPEED_ESPI_VW_TIMEOUT);
	if (ret)
		return;

	if (aspeed_espi_sysevt_gpio_get_dir(chip, offset) == GPIO_LINE_DIRECTION_IN)
		return;

	if (offset >= 32)
		offset -= 32;

	regmap_update_bits(espi_gpio->regmap, reg, BIT(offset), value ? BIT(offset) : 0);
}

static int aspeed_espi_init_valid_mask(struct gpio_chip *chip,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(chip);

	if (espi_gpio->auto_ack_warnings) {
		clear_bit(6, valid_mask);
		clear_bit(8, valid_mask);
		clear_bit(16, valid_mask);
		clear_bit(27, valid_mask);
		clear_bit(32, valid_mask);
		clear_bit(52, valid_mask);
	}
	if (espi_gpio->auto_ack_target_boot_done) {
		clear_bit(20, valid_mask);
		clear_bit(23, valid_mask);
	}
	return 0;
}

static void aspeed_espi_irq_init_valid_mask(struct gpio_chip *chip,
					    unsigned long *valid_mask,
					    unsigned int ngpios)
{
	unsigned int i;

	/* input GPIOs are even bits */
	for (i = 0; i < ngpios; i++) {
		if (aspeed_espi_sysevt_gpio_get_dir(chip, i) != GPIO_LINE_DIRECTION_IN)
			clear_bit(i, valid_mask);
	}

	aspeed_espi_init_valid_mask(chip, valid_mask, ngpios);
}

static int aspeed_espi_irq_init_hw(struct gpio_chip *chip)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = gpiochip_get_data(chip);

	/* Mask interrupts. */
	regmap_write(espi_gpio->regmap, ESPI_SYSEVT_INT_EN, 0);
	regmap_write(espi_gpio->regmap, ESPI_SYSEVT1_INT_EN, 0);

	return 0;
}

static int aspeed_espi_sysevt_gpio_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
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

	if (of_device_is_compatible(pdev->dev.parent->of_node, "aspeed,ast2600-espi-ctrl"))
		espi_gpio->version = ESPI_AST2600;
	else
		espi_gpio->version = ESPI_AST2500;

	if (of_property_read_bool(dev->of_node, "aspeed,auto-ack-warn"))
		espi_gpio->auto_ack_warnings = true;

	if (of_property_read_bool(dev->of_node, "aspeed,auto-ack-target-boot"))
		espi_gpio->auto_ack_target_boot_done = true;

	ret = aspeed_espi_sysevt_ready(dev, ASPEED_ESPI_VW_TIMEOUT);
	if (ret)
		return ret;

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
	gc->init_valid_mask = aspeed_espi_init_valid_mask;

	girq = &gc->irq;
	gpio_irq_chip_set_chip(girq, &aspeed_espi_vw_sysevt_irq_chip);
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->init_valid_mask = aspeed_espi_irq_init_valid_mask;
	girq->init_hw = aspeed_espi_irq_init_hw;

	ret = devm_gpiochip_add_data(dev, gc, espi_gpio);
	if (ret) {
		dev_err(dev, "Failed to register gpio chip (%d)\n", ret);
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "sysevt");
	if (irq < 0) {
		dev_err(dev, "failed to get interrupt 0: %d\n", irq);
		ret = irq;
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, aspeed_espi_vw_sysevt_irq_sysevt,
					IRQF_ONESHOT, pdev->name, espi_gpio);
	if (ret) {
		dev_err(dev, "failed to request interrupt: %d\n", ret);
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "sysevt1");
	if (irq < 0) {
		dev_err(dev, "failed to get interrupt 1: %d\n", irq);
		ret = irq;
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, aspeed_espi_vw_sysevt_irq_sysevt1,
					IRQF_ONESHOT, pdev->name, espi_gpio);
	if (ret) {
		dev_err(dev, "failed to request interrupt: %d\n", ret);
		return ret;
	}

	/* Enable HW auto ACK of WARN signals if supported */
	if (espi_gpio->auto_ack_warnings) {
		if (espi_gpio->version == ESPI_AST2600)
			regmap_set_bits(espi_gpio->regmap, ESPI_CTRL2,
					ESPI_CTRL2_AUTO_ACK_HOST_RST_WARN |
					ESPI_CTRL2_AUTO_ACK_OOB_RST_WARN |
					ESPI_CTRL2_AUTO_ACK_SUS_WARN);
	}

	/* Set VW software ready */
	regmap_set_bits(espi_gpio->regmap, ESPI_CTRL, ESPI_CTRL_VW_SW_RDY);

	/* When auto ACK target boot done reserved GPIO lines */
	if (espi_gpio->auto_ack_target_boot_done) {
		regmap_set_bits(espi_gpio->regmap, ESPI_SYSEVT,
				ESPI_SYSEVT_TARGET_BOOT_STS |
				ESPI_SYSEVT_TARGET_BOOT_DONE);
	}

	return 0;
}

static void aspeed_espi_sysevt_gpio_remove(struct platform_device *pdev)
{
	struct aspeed_espi_vw_sysevt *espi_gpio = platform_get_drvdata(pdev);

	if (espi_gpio->auto_ack_target_boot_done)
		regmap_clear_bits(espi_gpio->regmap, ESPI_SYSEVT, ESPI_SYSEVT_TARGET_BOOT_DONE);

	regmap_clear_bits(espi_gpio->regmap, ESPI_CTRL, ESPI_CTRL_VW_SW_RDY);
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
