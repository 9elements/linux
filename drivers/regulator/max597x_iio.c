// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for regulators in MAX5970 and MAX5978 IC
 *
 * Copyright (c) 2022 9elements GmbH
 *
 * Author: Patrick Rudolph <patrick.rudolph@9elements.com>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

#include "max597x.h"

struct max597x_iio {
	struct regmap *regmap;
	int shunt_micro_ohms[MAX5970_NUM_SWITCHES];
	unsigned int irng[MAX5970_NUM_SWITCHES];
	unsigned int mon_rng[MAX5970_NUM_SWITCHES];
};

#define MAX597X_ADC_CHANNEL(_idx, _type) {			\
	.type = IIO_ ## _type,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.address = MAX5970_REG_ ## _type ## _L(_idx),		\
}

static const struct iio_chan_spec max5978_adc_iio_channels[] = {
	MAX597X_ADC_CHANNEL(0, VOLTAGE),
	MAX597X_ADC_CHANNEL(0, CURRENT),
};

static const struct iio_chan_spec max5970_adc_iio_channels[] = {
	MAX597X_ADC_CHANNEL(0, VOLTAGE),
	MAX597X_ADC_CHANNEL(0, CURRENT),
	MAX597X_ADC_CHANNEL(1, VOLTAGE),
	MAX597X_ADC_CHANNEL(1, CURRENT),
};

static int max597x_iio_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long info)
{
	int ret;
	struct max597x_iio *data = iio_priv(iio_dev);
	unsigned int reg_l, reg_h;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(data->regmap, chan->address, &reg_l);
		if (ret < 0)
			return ret;
		ret = regmap_read(data->regmap, chan->address - 1, &reg_h);
		if (ret < 0)
			return ret;
		*val = (reg_h << 2) | (reg_l & 3);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:

		switch (chan->address) {
		case MAX5970_REG_CURRENT_L(0):
			fallthrough;
		case MAX5970_REG_CURRENT_L(1):
			*val = data->irng[chan->channel] * 1000; /* in A, convert to mA */
			*val2 = data->shunt_micro_ohms[chan->channel] * ADC_MASK;
			return IIO_VAL_FRACTIONAL;

		case MAX5970_REG_VOLTAGE_L(0):
			fallthrough;
		case MAX5970_REG_VOLTAGE_L(1):
			*val = data->mon_rng[chan->channel]; /* in uV, convert to mV */
			*val2 = ADC_MASK * 1000;
			return IIO_VAL_FRACTIONAL;
		}

		break;
	}
	return -EINVAL;
}

static const struct iio_info max597x_adc_iio_info = {
	.read_raw = &max597x_iio_read_raw,
};

int max597x_iio_configure(struct device *dev,
			  enum max597x_chip_type chip,
			  struct regmap *regmap,
			  u32 irng[MAX5970_NUM_SWITCHES],
			  u32 mon_rng[MAX5970_NUM_SWITCHES],
			  u32 shunt_micro_ohms[MAX5970_NUM_SWITCHES])
{
	struct iio_dev *indio_dev;
	struct max597x_iio *priv;
	int ret, i;

	/* registering iio */
	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev) {
		dev_err(dev, "failed allocating iio device\n");
		return -ENOMEM;
	}
	indio_dev->name = dev_name(dev);
	indio_dev->info = &max597x_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	switch (chip) {
	case MAX597x_TYPE_MAX5970:
		indio_dev->channels = max5970_adc_iio_channels;
		indio_dev->num_channels = ARRAY_SIZE(max5970_adc_iio_channels);
		break;
	case MAX597x_TYPE_MAX5978:
		indio_dev->channels = max5978_adc_iio_channels;
		indio_dev->num_channels = ARRAY_SIZE(max5978_adc_iio_channels);
		break;
	}

	priv = iio_priv(indio_dev);
	priv->regmap = regmap;
	for (i = 0; i < indio_dev->num_channels; i++) {
		priv->irng[i] = irng[i];
		priv->mon_rng[i] = mon_rng[i];
		priv->shunt_micro_ohms[i] = shunt_micro_ohms[i];
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		dev_err(dev, "could not register iio device");

	return ret;
}
