// SPDX-License-Identifier: GPL-2.0
/*
 * Linear Technology LTC2308 SPI ADC driver
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#define LTC2308_DEVICE_ID	0x00
#define LTC2308_ADC_RESOLUTION	12
#define LTC2308_SLEEP_MODE	BIT(0)
#define LTC2308_ADDR_SEL_0	BIT(2)
#define LTC2308_ADDR_SEL_1	BIT(3)
#define LTC2308_SIGN		BIT(4)
#define LTC2308_SINGLE_ENDED_CH	BIT(5)

enum ltc2308_ch_polarity {
	LTC2308_UNIPOLAR_CH = BIT(1),
	LTC2308_BIPOLAR_CH = 0x00
};

struct ltc2308_dev {
	struct regulator		*vref;
	struct spi_device		*spi;
	struct mutex			lock;
	enum ltc2308_ch_polarity	polarity;
	bool				sleep_mode;
};

static int ltc2308_read(struct ltc2308_dev *adc, unsigned char cfg, int *val)
{
	unsigned char spi_tx_buffer[2];
	unsigned char spi_rx_buffer[2];
	struct spi_transfer xfer = {
		.tx_buf = spi_tx_buffer,
		.rx_buf = spi_rx_buffer,
		.rx_buf = val,
		.len = 2,
		.bits_per_word = LTC2308_ADC_RESOLUTION
	};

	spi_tx_buffer[0] = (cfg | adc->polarity) << 2;
	spi_sync_transfer(adc->spi, &xfer, 1);
	*val = (spi_rx_buffer[0] << 8 ) | spi_rx_buffer[1];

	return 0;
}

static int ltc2308_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ltc2308_dev *adc = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&adc->lock);
		ltc2308_read(adc, chan->address, val);
		mutex_unlock(&adc->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(adc->vref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = LTC2308_ADC_RESOLUTION;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}

}

static int lc2308_set_sleep_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int mode)
{
	struct ltc2308_dev *adc = iio_priv(indio_dev);

	adc->sleep_mode = mode;

	return 0;
}

static int lc2308_get_sleep_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct ltc2308_dev *adc = iio_priv(indio_dev);

	return adc->sleep_mode;
}

static void ltc2308_regulator_disable(void *data)
{
	struct ltc2308_dev *adc = data;

	regulator_disable(adc->vref);
}

static void ltc2308_parse_dt(struct ltc2308_dev *adc, struct device *dev)
{
	struct device_node *np = dev->of_node;
	bool mode;

	mode = of_property_read_bool(np, "adi,bipolar");
	if (mode)
		adc->polarity = mode ? LTC2308_BIPOLAR_CH : LTC2308_UNIPOLAR_CH;
}

static const struct iio_info ltc2308_info = {
	.read_raw = ltc2308_read_raw
};

static const char * const ltc2308_sleep_modes [] = { "DISABLED", "ENABLED" };

static const struct iio_enum ltc2308_sleep_mode_enum = {
	.items = ltc2308_sleep_modes,
	.num_items = ARRAY_SIZE(ltc2308_sleep_modes),
	.set = lc2308_set_sleep_mode,
	.get = lc2308_get_sleep_mode
};

static struct iio_chan_spec_ext_info ltc2308_ext_info[] = {
	IIO_ENUM("sleep_mode",
		 IIO_SHARED_BY_ALL,
		 &ltc2308_sleep_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("sleep_mode",
				  IIO_SHARED_BY_ALL,
				  &ltc2308_sleep_mode_enum),
	{ }
};

#define LTC2308_CHAN(_index, _name) {					\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _index,						\
	.address = ((_index / 2) | ((_index % 2) ? LTC2308_SIGN : 0) |	\
		   LTC2308_SINGLE_ENDED_CH),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = ltc2308_ext_info,					\
	.datasheet_name = _name,					\
}

#define LTC2308_CHAN_DIFF(_index, _name) {				\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _index + 8,						\
	.address = ((_index / 2) | ((_index % 2) ? LTC2308_SIGN : 0)),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = ltc2308_ext_info,					\
	.datasheet_name = _name,					\
}

static const struct iio_chan_spec ltc2308_channels[] = {
	LTC2308_CHAN(0, "CH0"),
	LTC2308_CHAN(1, "CH1"),
	LTC2308_CHAN(2, "CH2"),
	LTC2308_CHAN(3, "CH3"),
	LTC2308_CHAN(4, "CH4"),
	LTC2308_CHAN(5, "CH5"),
	LTC2308_CHAN(6, "CH6"),
	LTC2308_CHAN(7, "CH7")
	// LTC2308_CHAN_DIFF(0, "CH0-CH1"),
	// LTC2308_CHAN_DIFF(1, "CH2-CH3"),
	// LTC2308_CHAN_DIFF(2, "CH4-CH5"),
	// LTC2308_CHAN_DIFF(3, "CH6-CH7"),
	// LTC2308_CHAN_DIFF(4, "CH1-CH0"),
	// LTC2308_CHAN_DIFF(5, "CH3-CH2"),
	// LTC2308_CHAN_DIFF(6, "CH5-CH4"),
	// LTC2308_CHAN_DIFF(7, "CH7-CH6")
};

static int ltc2308_probe(struct spi_device *spi)
{
	struct ltc2308_dev *adc;
	struct iio_dev *iio_adc;
	int ret;

	iio_adc = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!iio_adc)
		return -ENOMEM;

	adc = iio_priv(iio_adc);

	adc->spi = spi;
	adc->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->vref))
		return PTR_ERR(adc->vref);
	ret = regulator_enable(adc->vref);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable vref regulator\n");
		return ret;
	}
	ret = devm_add_action_or_reset(&spi->dev, ltc2308_regulator_disable,
				       adc);
	if (ret)
		return ret;

	iio_adc->dev.parent = &spi->dev;
	iio_adc->name = spi_get_device_id(spi)->name;
	iio_adc->modes = INDIO_DIRECT_MODE;
	iio_adc->info = &ltc2308_info;
	iio_adc->channels = ltc2308_channels;
	iio_adc->num_channels = ARRAY_SIZE(ltc2308_channels);

	ltc2308_parse_dt(adc, &spi->dev);
	mutex_init(&adc->lock);
	spi_set_drvdata(spi, iio_adc);

	return devm_iio_device_register(&spi->dev, iio_adc);
}

static int ltc2308_remove(struct spi_device *spi)
{
	struct iio_dev *iio_adc = spi_get_drvdata(spi);
	struct ltc2308_dev *adc = iio_priv(iio_adc);

	iio_device_unregister(iio_adc);

	return regulator_disable(adc->vref);
}

static const struct spi_device_id ltc2308_id[] = {
	{ "ltc2308", LTC2308_DEVICE_ID }
}
MODULE_DEVICE_TABLE(spi, ltc2308_id);

static const struct of_device_id ltc2308_of_match[] = {
	{ .compatible = "adi,ltc2308" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2308_of_match);

static struct spi_driver ltc2308_driver = {
	.driver = {
		.name = "ltc2308",
		.of_match_table = ltc2308_of_match,
	},
	.probe = ltc2308_probe,
	.remove = ltc2308_remove,
	.id_table = ltc2308_id,
};
module_spi_driver(ltc2308_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Linear Technology LTC2308 ADC");
MODULE_LICENSE("GPL v2");
