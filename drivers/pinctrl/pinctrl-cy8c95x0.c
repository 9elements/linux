// SPDX-License-Identifier: GPL-2.0-only
/*
 *  CY8C95X0 20/40/60 bit I2C GPIO port expander with interrupt support
 *
 *  Copyright (C) 2021 Patrick Rudolph <patrick.rudolph@9elements.com>
 *
 *  Derived from drivers/i2c/chips/pca953x.c
 */

#include <linux/bitmap.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/of.h>
#include <linux/pinctrl/pinmux.h>

#include <asm/unaligned.h>

/* Fast access registers */
#define CY8C95X0_INPUT		0x00
#define CY8C95X0_OUTPUT		0x08
#define CY8C95X0_INTSTATUS	0x10

#define CY8C95X0_INPUT_(x)	(CY8C95X0_INPUT + (x))
#define CY8C95X0_OUTPUT_(x)	(CY8C95X0_OUTPUT + (x))
#define CY8C95X0_INTSTATUS_(x)	(CY8C95X0_INTSTATUS + (x))

/* Port Select configures the port */
#define CY8C95X0_PORTSEL	0x18
/* port settings, write PORTSEL first */
#define CY8C95X0_INTMASK	0x19
#define CY8C95X0_PWMSEL		0x1A
#define CY8C95X0_INVERT		0x1B
#define CY8C95X0_DIRECTION	0x1C
/* Drive mode register change state on writing '1' */
#define CY8C95X0_DRV_PU		0x1D
#define CY8C95X0_DRV_PD		0x1E
#define CY8C95X0_DRV_ODH	0x1F
#define CY8C95X0_DRV_ODL	0x20
#define CY8C95X0_DRV_PP_FAST	0x21
#define CY8C95X0_DRV_PP_SLOW	0x22
#define CY8C95X0_DRV_HIZ	0x23
#define CY8C95X0_DEVID		0x2E
#define CY8C95X0_DEVID_(x)	(((x) >> 4) & 0xF)

#define CY8C95X0_PIN_TO_OFFSET(x) (((x) >= 20) ? ((x) + 4) : (x))
#define CY8C95X0_OFFSET_TO_PIN(x) (((x) >= 20) ? ((x) - 4) : (x))


struct cy8c95x0_platform_data {
	/* number of the first GPIO */
	unsigned	int gpio_base;

	/* initial polarity inversion setting */
	u32		invert;

	/* interrupt base */
	int		irq_base;

	const char	*const *names;
};

struct cy8c95x0_lookup {
	u8 port;
	u8 pin;
};

static const struct i2c_device_id cy8c95x0_id[] = {
	{ "cy8c9520", 20, },
	{ "cy8c9540", 40, },
	{ "cy8c9560", 60, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cy8c95x0_id);

#define OF_CY8C95X(__nrgpio) ((void *)(__nrgpio))

static const struct of_device_id cy8c95x0_dt_ids[] = {
	{ .compatible = "cypress,cy8c9520", .data = OF_CY8C95X(20), },
	{ .compatible = "cypress,cy8c9540", .data = OF_CY8C95X(40), },
	{ .compatible = "cypress,cy8c9560", .data = OF_CY8C95X(60), },
	{ }
};

MODULE_DEVICE_TABLE(of, cy8c95x0_dt_ids);

#define MAX_BANK 8
#define BANK_SZ 8
#define MAX_LINE	(MAX_BANK * BANK_SZ)

#define NBANK(chip) (cypress_get_nport(chip))
#define CY8C95X0_GPIO_MASK		GENMASK(7, 0)

struct cy8c95x0_chip {
	unsigned int gpio_start;
	/* protect serialized access to the registers behind the bank mux */
	struct mutex i2c_lock;
	struct regmap *regmap;
	/* protect serialized access to the interrupt controller bus */
	struct mutex irq_lock;
	DECLARE_BITMAP(irq_mask, MAX_LINE);
	DECLARE_BITMAP(irq_trig_raise, MAX_LINE);
	DECLARE_BITMAP(irq_trig_fall, MAX_LINE);
	DECLARE_BITMAP(irq_trig_low, MAX_LINE);
	DECLARE_BITMAP(irq_trig_high, MAX_LINE);
	DECLARE_BITMAP(push_pull, MAX_LINE);
	struct irq_chip irq_chip;
	atomic_t wakeup_path;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	unsigned long driver_data;
	struct regulator *regulator;

	struct device		*dev;

	struct pinctrl_dev	*pctldev;
	struct pinctrl_desc	pinctrl_desc;
	struct cy8c95x0_lookup cy8c95x0_lt[64];
	char name[32]; /* Chip Variant name */
	unsigned int tpin; /* Total number of pins */
};

static inline u8 cypress_get_nport(struct cy8c95x0_chip *chip)
{
	switch(chip->gpio_chip.ngpio)
	{
	case 0 ... 20:
		/* cy8c9520a */
		return 3;
	case 21 ... 40:
		/* cy8c9540a */
		return 6;
	case 41 ... 60:
		/* cy8c9560a */
		return 8;
	}
	return -1;
}

/* Per-port GPIO offset */
static const u8 cy8c9520a_port_offs[] = { /* Gport0, Gport1 & Gport2 */
	0,
	8,
	16,
	20,
};

static const u8 cy8c9540a_port_offs[] = { /* Gport0-5: 8, 8, 4, 8, 8, 4 */
	0,
	8,
	16,
	20,
	28,
	36,
	40,
};

static const u8 cy8c9560a_port_offs[] = { /* Gport0-7: 8, 8, 4, 8, 8, 8, 8, 8 */
	0,
	8,
	16,
	20,
	28,
	36,
	44,
	52,
	60,
};

static inline u8 cypress_get_port(struct cy8c95x0_chip *chip, unsigned int gpio)
{
	if (gpio >= 20)
		return (gpio + 4) / BANK_SZ;
	else
		return gpio / BANK_SZ;
}

static int cypress_get_pin(struct cy8c95x0_chip *chip, unsigned int gpio)
{
	if (gpio >= 20)
		gpio += 4;

	return gpio % BANK_SZ;
}

static bool cy8c95x0_readable_register(struct device *dev, unsigned int reg)
{
	return true;
}

static bool cy8c95x0_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CY8C95X0_INPUT_(0) ... CY8C95X0_INPUT_(7):
		return false;
	case CY8C95X0_DEVID:
		return false;
	}

	return true;
}

static bool cy8c95x0_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CY8C95X0_INPUT_(0) ... CY8C95X0_INPUT_(7):
	case CY8C95X0_INTSTATUS_(0) ... CY8C95X0_INTSTATUS_(7):
	case CY8C95X0_INTMASK:
	case CY8C95X0_INVERT:
	case CY8C95X0_DIRECTION:
	case CY8C95X0_DRV_PU:
	case CY8C95X0_DRV_PD:
	case CY8C95X0_DRV_ODH:
	case CY8C95X0_DRV_ODL:
	case CY8C95X0_DRV_PP_FAST:
	case CY8C95X0_DRV_PP_SLOW:
	case CY8C95X0_DRV_HIZ:

		return true;
	}

	return false;
}

static const struct regmap_config cy8c95x0_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = cy8c95x0_readable_register,
	.writeable_reg = cy8c95x0_writeable_register,
	.volatile_reg = cy8c95x0_volatile_register,

	.disable_locking = true,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x30,
};

static int cy8c95x0_write_regs(struct cy8c95x0_chip *chip, int reg, unsigned long *val)
{
	u8 value[MAX_BANK];
	int i, ret;

	for (i = 0; i < NBANK(chip); i++)
		value[i] = bitmap_get_value8(val, i * BANK_SZ);

	/* Custom function for muxed registers */
	if (reg == CY8C95X0_INTMASK || reg == CY8C95X0_INVERT ||
	    reg == CY8C95X0_DIRECTION) {
		for (i = 0; i < NBANK(chip); i++) {
			ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, i);
			if (ret < 0)
				goto out;

			ret = regmap_write(chip->regmap, reg, value[i]);
			if (ret < 0)
				goto out;
		}
	} else if (reg == CY8C95X0_INPUT || reg == CY8C95X0_OUTPUT ||
		   reg == CY8C95X0_INTSTATUS) {
		ret = regmap_bulk_write(chip->regmap, reg, value, NBANK(chip));
		if (ret < 0)
			goto out;
	} else {
		return -EINVAL;
	}

out:
	if (ret < 0)
		dev_err(&chip->client->dev, "failed writing register %d: err %d\n", reg, ret);

	return ret;
}

static int cy8c95x0_read_regs(struct cy8c95x0_chip *chip, int reg, unsigned long *val)
{
	u8 value[MAX_BANK];
	int read_val;
	int i;
	int ret = 0;

	/* Custom function for muxed registers */
	if (reg == CY8C95X0_INTMASK || reg == CY8C95X0_INVERT ||
	    reg == CY8C95X0_DIRECTION) {
		for (i = 0; i < NBANK(chip); i++) {
			ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, i);
			if (ret < 0)
				goto out;

			ret = regmap_read(chip->regmap, reg, &read_val);
			if (ret < 0)
				goto out;
			value[i] = read_val;
		}
	} else if (reg == CY8C95X0_INPUT || reg == CY8C95X0_OUTPUT ||
		   reg == CY8C95X0_INTSTATUS) {
		ret = regmap_bulk_read(chip->regmap, reg, value, NBANK(chip));
		if (ret < 0)
			goto out;
	} else {
		return -EINVAL;
	}

	for (i = 0; i < NBANK(chip); i++)
		bitmap_set_value8(val, value[i], i * BANK_SZ);
out:
	if (ret < 0)
		dev_err(&chip->client->dev, "failed reading register %d: err %d\n", reg, ret);

	return ret;
}

static int cy8c95x0_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	u8 port = chip->cy8c95x0_lt[off].port;
	u8 bit = chip->cy8c95x0_lt[off].pin;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* select bank */
	ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, port);
	if (ret)
		goto exit;

	/* Set direction to output if BIAS is enabled, else input */
	ret = regmap_write_bits(chip->regmap, CY8C95X0_DIRECTION, bit, bit);
	if (ret)
		goto exit;

	if (test_bit(off, chip->push_pull)) {
		/* 
		 * Disable driving the pin by forcing it to HighZ. Only setting the
		 * direction register isn't sufficient in Push-Pull mode.
		 */
		ret = regmap_write_bits(chip->regmap, CY8C95X0_DRV_HIZ, bit, bit);
		if (ret)
			goto exit;
		clear_bit(off, chip->push_pull);
	}
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}


static int cy8c95x0_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int off, int val)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	u8 port = chip->cy8c95x0_lt[off].port;
	u8 outreg = CY8C95X0_OUTPUT_(port);
	u8 bit = chip->cy8c95x0_lt[off].pin;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	ret = regmap_write_bits(chip->regmap, outreg, bit, val ? bit : 0);
	if (ret)
		goto exit;

	/* select port */
	ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, port);
	if (ret)
		goto exit;

	/* then direction */
	ret = regmap_write_bits(chip->regmap, CY8C95X0_DIRECTION, bit, 0);
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int cy8c95x0_gpio_get_value(struct gpio_chip *gc, unsigned int off)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	u8 inreg = CY8C95X0_INPUT_(cypress_get_pin(chip, off));
	u8 bit = chip->cy8c95x0_lt[off].pin;
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_read(chip->regmap, inreg, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/*
		 * NOTE:
		 * diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return !!(reg_val & bit);
}

static void cy8c95x0_gpio_set_value(struct gpio_chip *gc, unsigned int off,
				    int val)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	u8 outreg = CY8C95X0_OUTPUT_(cypress_get_pin(chip, off));
	u8 bit = chip->cy8c95x0_lt[off].pin;

	mutex_lock(&chip->i2c_lock);
	regmap_write_bits(chip->regmap, outreg, bit, val ? bit : 0);
	mutex_unlock(&chip->i2c_lock);
}

static int cy8c95x0_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	u8 port = chip->cy8c95x0_lt[off].port;
	u8 bit = chip->cy8c95x0_lt[off].pin;
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, port);
	if (ret < 0) {
		mutex_unlock(&chip->i2c_lock);
		return ret;
	}
	ret = regmap_read(chip->regmap, CY8C95X0_DIRECTION, &reg_val);
	if (ret < 0) {
		mutex_unlock(&chip->i2c_lock);

		return ret;
	}

	mutex_unlock(&chip->i2c_lock);


	if (reg_val & bit)
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int cy8c95x0_gpio_get_pincfg(struct cy8c95x0_chip *chip,
				    unsigned int off,
				    unsigned long *config)
{
	u8 port = chip->cy8c95x0_lt[off].port;
	enum pin_config_param param = pinconf_to_config_param(*config);
	struct device *dev = chip->dev;
	u8 bit = chip->cy8c95x0_lt[off].pin;
	unsigned int reg;
	u32 reg_val;
	u16 arg = 0;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* select port */
	ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, port);
	if (ret < 0) {
		mutex_unlock(&chip->i2c_lock);
		return ret;
	}

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		reg = CY8C95X0_DRV_PU;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		reg = CY8C95X0_DRV_PD;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		reg = CY8C95X0_DRV_HIZ;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		reg = CY8C95X0_DRV_ODL;
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		reg = CY8C95X0_DRV_ODH;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		reg = CY8C95X0_DRV_PP_FAST;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		reg = CY8C95X0_DIRECTION;
		break;
	case PIN_CONFIG_MODE_PWM:
		reg = CY8C95X0_PWMSEL;
		break;
	case PIN_CONFIG_OUTPUT:
		reg = CY8C95X0_OUTPUT_(port);
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		reg = CY8C95X0_DIRECTION;
		break;

	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
	case PIN_CONFIG_BIAS_BUS_HOLD:
	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
	case PIN_CONFIG_DRIVE_STRENGTH:
	case PIN_CONFIG_DRIVE_STRENGTH_UA:
	case PIN_CONFIG_INPUT_DEBOUNCE:
	case PIN_CONFIG_INPUT_SCHMITT:
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
	case PIN_CONFIG_MODE_LOW_POWER:
	case PIN_CONFIG_PERSIST_STATE:
	case PIN_CONFIG_POWER_SOURCE:
	case PIN_CONFIG_SKEW_DELAY:
	case PIN_CONFIG_SLEEP_HARDWARE_STATE:
	case PIN_CONFIG_SLEW_RATE:
	default:
		ret = -ENOTSUPP;
		goto exit;
	}
	/* Writing 1 to one of the drive mode registers will automatically
	 * clear conflicting set bits in the other drive mode registers.
	 */
	ret = regmap_read(chip->regmap, reg, &reg_val);
	if(reg_val & bit)
		arg = 1;

	*config = pinconf_to_config_packed(param, (u16)arg);
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int cy8c95x0_gpio_set_pincfg(struct cy8c95x0_chip *chip,
				    unsigned int off,
				    unsigned long config)
{
	u8 port = chip->cy8c95x0_lt[off].port;
	struct device *dev = chip->dev;
	u8 bit = chip->cy8c95x0_lt[off].pin;
	unsigned long param = pinconf_to_config_param(config);
	unsigned int reg, reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* select port */
	ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, port);
	if (ret < 0) {
		mutex_unlock(&chip->i2c_lock);
		return ret;
	}

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		clear_bit(off, chip->push_pull);
		reg = CY8C95X0_DRV_PU;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		clear_bit(off, chip->push_pull);
		reg = CY8C95X0_DRV_PD;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		clear_bit(off, chip->push_pull);
		reg = CY8C95X0_DRV_HIZ;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		clear_bit(off, chip->push_pull);
		reg = CY8C95X0_DRV_ODL;
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		clear_bit(off, chip->push_pull);
		reg = CY8C95X0_DRV_ODH;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		set_bit(off, chip->push_pull);
		reg = CY8C95X0_DRV_PP_FAST;
		break;
	case PIN_CONFIG_MODE_PWM:
		reg = CY8C95X0_PWMSEL;
		break;
	default:
		ret = -ENOTSUPP;
		goto exit;
	}
	/* Writing 1 to one of the drive mode registers will automatically
	 * clear conflicting set bits in the other drive mode registers.
	 */
	ret = regmap_write_bits(chip->regmap, reg, bit, bit);

exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int cy8c95x0_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				    unsigned long config)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	unsigned long arg = pinconf_to_config_argument(config);

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_INPUT_ENABLE:
		return cy8c95x0_gpio_direction_input(gc, offset);
	case PIN_CONFIG_OUTPUT:
		return cy8c95x0_gpio_direction_output(gc, offset, arg);
	case PIN_CONFIG_MODE_PWM:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		return cy8c95x0_gpio_set_pincfg(chip, offset, config);
	default:
		return -ENOTSUPP;
	}
}

static int cy8c95x0_gpio_get_multiple(struct gpio_chip *gc,
				      unsigned long *mask, unsigned long *bits)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	DECLARE_BITMAP(reg_val, MAX_LINE);
	DECLARE_BITMAP(tmp, MAX_LINE);
	DECLARE_BITMAP(shiftmask, MAX_LINE);
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = cy8c95x0_read_regs(chip, CY8C95X0_INPUT, reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret)
		return ret;
	
	bitmap_zero(shiftmask, MAX_LINE);
	bitmap_set(shiftmask, 0, 20);

	/* Fill the 4 bit gap of Gport2 */
	bitmap_shift_right(tmp, reg_val, 4, MAX_LINE);
	bitmap_replace(tmp, tmp, reg_val, shiftmask, MAX_LINE);

	bitmap_replace(bits, bits, tmp, mask, gc->ngpio);

	return 0;
}

static void cy8c95x0_gpio_set_multiple(struct gpio_chip *gc,
					unsigned long *mask, unsigned long *bits)
{
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	DECLARE_BITMAP(reg_val, MAX_LINE);
	int ret, level;

	mutex_lock(&chip->i2c_lock);
	ret = cy8c95x0_read_regs(chip, CY8C95X0_OUTPUT, reg_val);
	if (ret)
		goto exit;

	for_each_set_bit(level, bits, gc->ngpio)
		bitmap_set(reg_val, CY8C95X0_PIN_TO_OFFSET(level), 1);

	cy8c95x0_write_regs(chip, CY8C95X0_OUTPUT, reg_val);
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void cy8c95x0_setup_gpio(struct cy8c95x0_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = cy8c95x0_gpio_direction_input;
	gc->direction_output = cy8c95x0_gpio_direction_output;
	gc->get = cy8c95x0_gpio_get_value;
	gc->set = cy8c95x0_gpio_set_value;
	gc->get_direction = cy8c95x0_gpio_get_direction;
	gc->get_multiple = cy8c95x0_gpio_get_multiple;
	gc->set_multiple = cy8c95x0_gpio_set_multiple;
	gc->set_config = cy8c95x0_gpio_set_config;
	gc->can_sleep = false;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;

	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = NULL;

}

static void cy8c95x0_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = CY8C95X0_PIN_TO_OFFSET(irqd_to_hwirq(d));

	set_bit(hwirq, chip->irq_mask);
}

static void cy8c95x0_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = CY8C95X0_PIN_TO_OFFSET(irqd_to_hwirq(d));

	clear_bit(hwirq, chip->irq_mask);
}

static int cy8c95x0_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);

	if (on)
		atomic_inc(&chip->wakeup_path);
	else
		atomic_dec(&chip->wakeup_path);

	return irq_set_irq_wake(chip->client->irq, on);
}

static void cy8c95x0_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void cy8c95x0_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	DECLARE_BITMAP(irq_mask, MAX_LINE);
	DECLARE_BITMAP(reg_direction, MAX_LINE);
	int level;

	cy8c95x0_write_regs(chip, CY8C95X0_INTMASK, chip->irq_mask);

	/* Switch direction to input if needed */
	cy8c95x0_read_regs(chip, CY8C95X0_DIRECTION, reg_direction);

	bitmap_or(irq_mask, chip->irq_trig_fall, chip->irq_trig_raise, MAX_LINE);
	bitmap_complement(reg_direction, reg_direction, MAX_LINE);
	bitmap_and(irq_mask, irq_mask, reg_direction, MAX_LINE);

	/* Look for any newly setup interrupt */
	for_each_set_bit(level, irq_mask, MAX_LINE)
		cy8c95x0_gpio_direction_input(&chip->gpio_chip, level);

	mutex_unlock(&chip->irq_lock);
}

static int cy8c95x0_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = CY8C95X0_PIN_TO_OFFSET(irqd_to_hwirq(d));
	unsigned int trig_type;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
		trig_type = type;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		trig_type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		trig_type = IRQ_TYPE_EDGE_FALLING;
		break;
	default:
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	assign_bit(hwirq, chip->irq_trig_fall, trig_type & IRQ_TYPE_EDGE_FALLING);
	assign_bit(hwirq, chip->irq_trig_raise, trig_type & IRQ_TYPE_EDGE_RISING);
	assign_bit(hwirq, chip->irq_trig_low, type == IRQ_TYPE_LEVEL_LOW);
	assign_bit(hwirq, chip->irq_trig_high, type == IRQ_TYPE_LEVEL_HIGH);

	return 0;
}

static void cy8c95x0_irq_shutdown(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95x0_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = CY8C95X0_PIN_TO_OFFSET(irqd_to_hwirq(d));

	clear_bit(hwirq, chip->irq_trig_raise);
	clear_bit(hwirq, chip->irq_trig_fall);
	clear_bit(hwirq, chip->irq_trig_low);
	clear_bit(hwirq, chip->irq_trig_high);
}

static bool cy8c95x0_irq_pending(struct cy8c95x0_chip *chip, unsigned long *pending)
{
	struct gpio_chip *gc = &chip->gpio_chip;
	DECLARE_BITMAP(cur_stat, MAX_LINE);
	DECLARE_BITMAP(new_stat, MAX_LINE);
	DECLARE_BITMAP(trigger, MAX_LINE);
	int ret;

	/* Read the current interrupt status from the device */
	ret = cy8c95x0_read_regs(chip, CY8C95X0_INTSTATUS, trigger);
	if (ret)
		return false;

	/* Check latched inputs and clear interrupt status */
	ret = cy8c95x0_read_regs(chip, CY8C95X0_INPUT, cur_stat);
	if (ret)
		return false;

	/* Apply filter for rising/falling edge selection */
	bitmap_replace(new_stat, chip->irq_trig_fall, chip->irq_trig_raise, cur_stat, MAX_LINE);

	bitmap_and(pending, new_stat, trigger, MAX_LINE);

	return !bitmap_empty(pending, MAX_LINE);
}

static irqreturn_t cy8c95x0_irq_handler(int irq, void *devid)
{
	struct cy8c95x0_chip *chip = devid;
	struct gpio_chip *gc = &chip->gpio_chip;
	DECLARE_BITMAP(pending, MAX_LINE);
	int level, pin;
	bool ret;

	bitmap_zero(pending, MAX_LINE);

	mutex_lock(&chip->i2c_lock);
	ret = cy8c95x0_irq_pending(chip, pending);
	mutex_unlock(&chip->i2c_lock);

	if (!ret)
		return IRQ_RETVAL(0);

	ret = 0;

	for_each_set_bit(level, pending, MAX_LINE) {
		/* Account for 4bit gap in GPort2 */
		pin = CY8C95X0_OFFSET_TO_PIN(level);
		int nested_irq = irq_find_mapping(gc->irq.domain, pin);

		if (unlikely(nested_irq <= 0)) {
			dev_warn_ratelimited(gc->parent, "unmapped interrupt %d\n", pin);
			continue;
		}

		if (test_bit(level, chip->irq_trig_low))
			while (!cy8c95x0_gpio_get_value(gc, pin))
				handle_nested_irq(nested_irq);
		else if (test_bit(level, chip->irq_trig_high))
			while (cy8c95x0_gpio_get_value(gc, pin))
				handle_nested_irq(nested_irq);
		else
			handle_nested_irq(nested_irq);

		ret = 1;
	}

	return IRQ_RETVAL(ret);
}

static unsigned int cy8c95x0_pins[60];

static const char * const cy8c95x0_groups[] = {
	"gp00",
	"gp01",
	"gp02",
	"gp03",
	"gp04",
	"gp05",
	"gp06",
	"gp07",

	"gp10",
	"gp11",
	"gp12",
	"gp13",
	"gp14",
	"gp15",
	"gp16",
	"gp17",

	"gp20",
	"gp21",
	"gp22",
	"gp23",

	"gp30",
	"gp31",
	"gp32",
	"gp33",
	"gp34",
	"gp35",
	"gp36",
	"gp37",

	"gp40",
	"gp41",
	"gp42",
	"gp43",
	"gp44",
	"gp45",
	"gp46",
	"gp47",

	"gp50",
	"gp51",
	"gp52",
	"gp53",
	"gp54",
	"gp55",
	"gp56",
	"gp57",

	"gp60",
	"gp61",
	"gp62",
	"gp63",
	"gp64",
	"gp65",
	"gp66",
	"gp67",

	"gp70",
	"gp71",
	"gp72",
	"gp73",
	"gp74",
	"gp75",
	"gp76",
	"gp77",
};

static int cy8c95x0_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct cy8c95x0_chip *chip = pinctrl_dev_get_drvdata(pctldev);
	return chip->tpin;
}

static const char *cy8c95x0_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						unsigned int group)
{
	return cy8c95x0_groups[group];
}

static int cy8c95x0_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					unsigned int group,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	struct cy8c95x0_chip *chip = pinctrl_dev_get_drvdata(pctldev);
	u8 i;

	for (i = 0 ; i < chip->tpin; i++)
		cy8c95x0_pins[i] = i;
	*pins = cy8c95x0_pins;
	*num_pins = chip->tpin;
	return 0;
}

static const struct pinctrl_ops cy8c95x0_pinctrl_ops = {
	.get_groups_count = cy8c95x0_pinctrl_get_groups_count,
	.get_group_name = cy8c95x0_pinctrl_get_group_name,
	.get_group_pins = cy8c95x0_pinctrl_get_group_pins,
#ifdef CONFIG_OF
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
#endif
};

static int cy8c95x0_get_functions_count(struct pinctrl_dev *pctldev)
{
	return 2;
}

static const char *cy8c95x0_get_fname(struct pinctrl_dev *pctldev, unsigned selector)
{
	if (selector == 0)
		return "gpio";
	else
		return "pwm";

}

static int cy8c95x0_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
			  const char * const **groups,
			  unsigned * const num_groups)
{
	struct cy8c95x0_chip *chip = pinctrl_dev_get_drvdata(pctldev);

	*groups = cy8c95x0_groups;
	*num_groups = chip->tpin;
	return 0;
}

static int cy8c95x0_pinmux_cfg(struct cy8c95x0_chip *chip,
				   unsigned int val,
				   unsigned long off)
{
	u8 port = chip->cy8c95x0_lt[off].port;
	u8 bit = chip->cy8c95x0_lt[off].pin;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* select port */
	ret = regmap_write(chip->regmap, CY8C95X0_PORTSEL, port);
	if (ret < 0)
		goto exit;

	if (val)
		val = bit & 0xff;

	ret = regmap_write_bits(chip->regmap, CY8C95X0_PWMSEL, bit, val);
	if (ret < 0)
		goto exit;

	/* Set direction to output & set output to 1 so that PWM can work */
	ret = regmap_write_bits(chip->regmap, CY8C95X0_DIRECTION, bit, bit);
	if (ret < 0)
		goto exit;

	ret = regmap_write_bits(chip->regmap, CY8C95X0_OUTPUT_(port), bit, bit);

exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int cy8c95x0_set_mux(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{
	struct cy8c95x0_chip *chip = pinctrl_dev_get_drvdata(pctldev);

	return cy8c95x0_pinmux_cfg(chip, selector, group);
}

static struct pinmux_ops cy8c95x0_pmxops = {
	.get_functions_count = cy8c95x0_get_functions_count,
	.get_function_name = cy8c95x0_get_fname,
	.get_function_groups = cy8c95x0_get_groups,
	.set_mux = cy8c95x0_set_mux,
	.strict = true,
};

static int cy8c95x0_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *config)
{
	struct cy8c95x0_chip *chip = pinctrl_dev_get_drvdata(pctldev);
	int ret;

	ret = cy8c95x0_gpio_get_pincfg(chip, pin, config);
	return ret;
}

static int cy8c95x0_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	struct cy8c95x0_chip *chip = pinctrl_dev_get_drvdata(pctldev);
	int ret = 0;
	int i;


	for (i = 0; i < num_configs; i++) {
		ret = cy8c95x0_gpio_set_pincfg(chip, pin, configs[i]);
	}

	return ret;
}

static const struct pinconf_ops cy8c95x0_pinconf_ops = {
	.pin_config_get = cy8c95x0_pinconf_get,
	.pin_config_set = cy8c95x0_pinconf_set,
	.is_generic = true,
};

static const struct pinctrl_pin_desc cy8c9560_pins[] = {
	PINCTRL_PIN(0, "gp00"),
	PINCTRL_PIN(1, "gp01"),
	PINCTRL_PIN(2, "gp02"),
	PINCTRL_PIN(3, "gp03"),
	PINCTRL_PIN(4, "gp04"),
	PINCTRL_PIN(5, "gp05"),
	PINCTRL_PIN(6, "gp06"),
	PINCTRL_PIN(7, "gp07"),

	PINCTRL_PIN(8, "gp10"),
	PINCTRL_PIN(9, "gp11"),
	PINCTRL_PIN(10, "gp12"),
	PINCTRL_PIN(11, "gp13"),
	PINCTRL_PIN(12, "gp14"),
	PINCTRL_PIN(13, "gp15"),
	PINCTRL_PIN(14, "gp16"),
	PINCTRL_PIN(15, "gp17"),

	PINCTRL_PIN(16, "gp20"),
	PINCTRL_PIN(17, "gp21"),
	PINCTRL_PIN(18, "gp22"),
	PINCTRL_PIN(19, "gp23"),

	PINCTRL_PIN(20, "gp30"),
	PINCTRL_PIN(21, "gp31"),
	PINCTRL_PIN(22, "gp32"),
	PINCTRL_PIN(23, "gp33"),
	PINCTRL_PIN(24, "gp34"),
	PINCTRL_PIN(25, "gp35"),
	PINCTRL_PIN(26, "gp36"),
	PINCTRL_PIN(27, "gp37"),

	PINCTRL_PIN(28, "gp40"),
	PINCTRL_PIN(29, "gp41"),
	PINCTRL_PIN(30, "gp42"),
	PINCTRL_PIN(31, "gp43"),
	PINCTRL_PIN(32, "gp44"),
	PINCTRL_PIN(33, "gp45"),
	PINCTRL_PIN(34, "gp46"),
	PINCTRL_PIN(35, "gp47"),

	PINCTRL_PIN(36, "gp50"),
	PINCTRL_PIN(37, "gp51"),
	PINCTRL_PIN(38, "gp52"),
	PINCTRL_PIN(39, "gp53"),
	PINCTRL_PIN(40, "gp54"),
	PINCTRL_PIN(41, "gp55"),
	PINCTRL_PIN(42, "gp56"),
	PINCTRL_PIN(43, "gp57"),

	PINCTRL_PIN(44, "gp60"),
	PINCTRL_PIN(45, "gp61"),
	PINCTRL_PIN(46, "gp62"),
	PINCTRL_PIN(47, "gp63"),
	PINCTRL_PIN(48, "gp64"),
	PINCTRL_PIN(49, "gp65"),
	PINCTRL_PIN(50, "gp66"),
	PINCTRL_PIN(51, "gp67"),

	PINCTRL_PIN(52, "gp70"),
	PINCTRL_PIN(53, "gp71"),
	PINCTRL_PIN(54, "gp72"),
	PINCTRL_PIN(55, "gp73"),
	PINCTRL_PIN(56, "gp74"),
	PINCTRL_PIN(57, "gp75"),
	PINCTRL_PIN(58, "gp76"),
	PINCTRL_PIN(59, "gp77"),

};



static int cy8c95x0_irq_setup(struct cy8c95x0_chip *chip, int irq_base)
{
	struct i2c_client *client = chip->client;
	struct irq_chip *irq_chip = &chip->irq_chip;
	struct gpio_irq_chip *girq;
	DECLARE_BITMAP(pending_irqs, MAX_LINE);
	int ret;

	if (!client->irq) {
		dev_warn(&client->dev, "No interrupt support enabled\n");
		return 0;
	}

	if (irq_base == -1) {
		dev_warn(&client->dev, "Invalid IRQ base\n");
		return 0;
	}
	dev_info(&client->dev, "client->irq: %d, irq_base: %d\n", client->irq, irq_base);

	mutex_init(&chip->irq_lock);

	bitmap_zero(pending_irqs, MAX_LINE);

	/* Read IRQ status register to clear all pending interrupts */
	mutex_lock(&chip->i2c_lock);
	ret = cy8c95x0_irq_pending(chip, pending_irqs);
	mutex_unlock(&chip->i2c_lock);
	if (ret) {
		dev_err(&client->dev, "failed to clear irq status register\n");
		return ret;
	}

	/* Mask all interrupts */
	bitmap_fill(chip->irq_mask, MAX_LINE);

	irq_chip->name = devm_kasprintf(&chip->client->dev, GFP_KERNEL, "%s-irq", chip->name);
	irq_chip->irq_mask = cy8c95x0_irq_mask;
	irq_chip->irq_unmask = cy8c95x0_irq_unmask;
	irq_chip->irq_set_wake = cy8c95x0_irq_set_wake;
	irq_chip->irq_bus_lock = cy8c95x0_irq_bus_lock;
	irq_chip->irq_bus_sync_unlock = cy8c95x0_irq_bus_sync_unlock;
	irq_chip->irq_set_type = cy8c95x0_irq_set_type;
	irq_chip->irq_shutdown = cy8c95x0_irq_shutdown;

	girq = &chip->gpio_chip.irq;
	girq->chip = irq_chip;
	/* This will let us handle the parent IRQ in the driver */
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;
	girq->first = irq_base; /* FIXME: get rid of this */

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, cy8c95x0_irq_handler,
					IRQF_ONESHOT | IRQF_SHARED | IRQF_TRIGGER_HIGH,
					dev_name(&client->dev), chip);
	if (ret) {
		dev_err(&client->dev, "failed to request irq %d\n",
			client->irq);
		return ret;
	}
	dev_info(&client->dev, "Registered threaded IRQ\n");

	return 0;
}

static int device_cy8c95x0_init(struct cy8c95x0_chip *chip, u32 invert)
{
	DECLARE_BITMAP(val, MAX_LINE);
	int ret;
	unsigned int devid;
	int index, i;

	bitmap_zero(chip->push_pull, MAX_LINE);

	ret = regcache_sync_region(chip->regmap, CY8C95X0_OUTPUT,
				   CY8C95X0_OUTPUT + NBANK(chip));
	if (ret) {
		dev_err(chip->dev, "%s: Failed at line %d", __func__, __LINE__);
		goto out;
	}

	/* Set all pins to input. FIXME: Leave Power on defauls? */
	bitmap_fill(val, MAX_LINE);
	ret = cy8c95x0_write_regs(chip, CY8C95X0_DIRECTION, val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed at line %d", __func__, __LINE__);
		goto out;
	}

	/* set platform specific polarity inversion */
	if (invert)
		bitmap_fill(val, MAX_LINE);
	else
		bitmap_zero(val, MAX_LINE);

	ret = cy8c95x0_write_regs(chip, CY8C95X0_INVERT, val);

	/* Initialize pin lookup table */
	ret = regmap_read(chip->regmap, CY8C95X0_DEVID, &devid);
	if (ret) {
		dev_err(chip->dev, "%s: Failed at line %d", __func__, __LINE__);
		goto out;
	}

	switch (devid) {
	case 0x20:
		index = 0;
		break;
	case 0x40:
		index = 1;
		break;
	case 0x60:
		index = 2;
		break;
	default:
		index = -1;
	}
	if (index == -1) {
		dev_err(chip->dev, "%s: Failed at line %d", __func__, __LINE__);
		goto out;
	}

	strscpy(chip->name, cy8c95x0_id[index].name, I2C_NAME_SIZE);
	chip->tpin = cy8c95x0_id[index].driver_data;
	/* Update lookup table */
	for (i = 0 ; i < chip->tpin; i++) {
		chip->cy8c95x0_lt[i].port = cypress_get_port(chip, i);
		chip->cy8c95x0_lt[i].pin = BIT(cypress_get_pin(chip, i));
	}

	chip->gpio_chip.label = devm_kasprintf(&chip->client->dev, GFP_KERNEL, "%s@%s", \
							chip->name, dev_name(&chip->client->dev));
out:
	return ret;
}

static int cy8c95x0_detect(struct i2c_client *client,
			   struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int devid, chip_index;
	const char *name;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* Only probe A0 addresses. The chips supports any address at cost of GPIOs */
	for (chip_index = 0x20; chip_index <= 0x21; chip_index++) {
		devid = i2c_smbus_read_byte_data(client, CY8C95X0_DEVID);
		switch (CY8C95X0_DEVID_(devid)) {
		case 2:
			name = cy8c95x0_id[0].name;
			break;
		case 4:
			name = cy8c95x0_id[1].name;
			break;
		case 6:
			name = cy8c95x0_id[2].name;
			break;
		default:
			return -ENODEV;
		}

		dev_info(&client->dev, "Found a %s chip at 0x%02x.\n", name, client->addr);
		strscpy(info->type, name, I2C_NAME_SIZE);
	}

	return -ENODEV;
}

static int cy8c95x0_probe(struct i2c_client *client)
{
	struct cy8c95x0_platform_data *pdata;
	struct cy8c95x0_chip *chip;
	int irq_base = 0;
	int ret;
	u32 invert = 0;
	struct regulator *reg;
	u32 ngpios;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		irq_base = pdata->irq_base;
		chip->gpio_start = pdata->gpio_base;
		invert = pdata->invert;
	} else {
		chip->gpio_start = -1;
		irq_base = 0;
	}

	chip->client = client;
	chip->dev = &client->dev;

	reg = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(reg)) {
		if (PTR_ERR(reg) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	} else {
		ret = regulator_enable(reg);
		if (ret) {
			dev_err(&client->dev, "failed to enable regulator vdd: %d\n", ret);
			return ret;
		}
		chip->regulator = reg;
	}

	/* Set the device type */
	if (client->dev.of_node)
		chip->driver_data = (unsigned long)of_device_get_match_data(&client->dev);
	else
		chip->driver_data = i2c_match_id(cy8c95x0_id, client)->driver_data;

	if (!chip->driver_data) {
		ret = -ENODEV;
		goto err_exit;
	}

	i2c_set_clientdata(client, chip);

	ngpios = -1;
	if (client->dev.of_node)
		of_property_read_u32(client->dev.of_node, "ngpios", &ngpios);

	if (ngpios < 0 || ngpios > (chip->driver_data & CY8C95X0_GPIO_MASK))
		ngpios = chip->driver_data & CY8C95X0_GPIO_MASK;

	cy8c95x0_setup_gpio(chip, ngpios);

	chip->regmap = devm_regmap_init_i2c(client, &cy8c95x0_i2c_regmap);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		goto err_exit;
	}

	regcache_mark_dirty(chip->regmap);

	mutex_init(&chip->i2c_lock);
	/*
	 * In case we have an i2c-mux controlled by a GPIO provided by an
	 * expander using the same driver higher on the device tree, read the
	 * i2c adapter nesting depth and use the retrieved value as lockdep
	 * subclass for chip->i2c_lock.
	 *
	 * REVISIT: This solution is not complete. It protects us from lockdep
	 * false positives when the expander controlling the i2c-mux is on
	 * a different level on the device tree, but not when it's on the same
	 * level on a different branch (in which case the subclass number
	 * would be the same).
	 *
	 * TODO: Once a correct solution is developed, a similar fix should be
	 * applied to all other i2c-controlled GPIO expanders (and potentially
	 * regmap-i2c).
	 */
	lockdep_set_subclass(&chip->i2c_lock,
			     i2c_adapter_depth(client->adapter));

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */

	ret = device_cy8c95x0_init(chip, invert);
	if (ret)
		goto err_exit;

	ret = cy8c95x0_irq_setup(chip, irq_base);
	if (ret)
		goto err_exit;

	ret = devm_gpiochip_add_data(&client->dev, &chip->gpio_chip, chip);
	if (ret)
		goto err_exit;

	chip->pinctrl_desc.pctlops = &cy8c95x0_pinctrl_ops;
	chip->pinctrl_desc.confops = &cy8c95x0_pinconf_ops;
	chip->pinctrl_desc.pmxops = &cy8c95x0_pmxops;
	chip->pinctrl_desc.npins = chip->gpio_chip.ngpio;
	chip->pinctrl_desc.name = devm_kasprintf(&chip->client->dev, GFP_KERNEL, "pinctrl-%s", \
								chip->name);
	chip->pinctrl_desc.pins = cy8c9560_pins;
	if (chip->pinctrl_desc.npins <= 20)
		chip->pinctrl_desc.npins = 20;
	else if (chip->pinctrl_desc.npins <= 40)
		chip->pinctrl_desc.npins = 40;
	else
		chip->pinctrl_desc.npins = 60;

	chip->pinctrl_desc.owner = THIS_MODULE;
	chip->pctldev = devm_pinctrl_register(&client->dev, &chip->pinctrl_desc, chip);

	if (IS_ERR(chip->pctldev)) {
		dev_err_probe(&client->dev, PTR_ERR(chip->pctldev), "can't register controller\n");
	}

	return 0;

err_exit:
	if (!IS_ERR_OR_NULL(chip->regulator))
		regulator_disable(chip->regulator);
	return ret;
}

static int cy8c95x0_remove(struct i2c_client *client)
{
	struct cy8c95x0_chip *chip = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(chip->regulator))
		regulator_disable(chip->regulator);

	return 0;
}

static struct i2c_driver cy8c95x0_driver = {
	.driver = {
		.name	= "cy8c95x0-pinctrl",
		.of_match_table = cy8c95x0_dt_ids,
	},
	.probe_new	= cy8c95x0_probe,
	.remove		= cy8c95x0_remove,
	.id_table	= cy8c95x0_id,
	.detect		= cy8c95x0_detect,
};

module_i2c_driver(cy8c95x0_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_AUTHOR("Yash S <yash.s@9elements.com>");
MODULE_DESCRIPTION("PINCTRL driver for CY8C95X0");
MODULE_LICENSE("GPL");
