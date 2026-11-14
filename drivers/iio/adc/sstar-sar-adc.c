// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q SAR ADC
 *
 * A conversion samples all channels; the result of each channel is held
 * in its own data register. The pin controller switches the input pads
 * between the analog and GPIO modes.
 *
 * Channel 6 measures the temperature sensor, which is powered from the PM
 * sleep block. Its reading drops by one step for every 1.37 degrees, the
 * reading at the calibration temperature is stored in the OTP.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bits.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* 16-bit registers on the RIU, 4-byte stride */
#define SAR_CTRL		0x00
#define  SAR_CTRL_START		BIT(14)
/* Value from the vendor driver, the bits are not documented */
#define  SAR_CTRL_INIT		0x0a20
#define SAR_TS_CTRL		0x40
#define  SAR_TS_CTRL_DUMMY	BIT(0)
#define SAR_TS_REF		0x64
/* Clear for the 2.0 V reference */
#define  SAR_TS_REF_SEL		BIT(6)
#define SAR_DATA(ch)		(0x100 + (ch) * 4)
#define  SAR_DATA_MASK		GENMASK(9, 0)

#define SAR_TS_CHANNEL		6

/* Temperature sensor power in the PM sleep block */
#define PMSLEEP_TS		0xbc
#define  PMSLEEP_TS_EN		BIT(2)

/* Millidegrees per step and calibration temperature */
#define SAR_TS_SLOPE		1370
#define SAR_TS_CAL_TEMP		27000
/* Calibration reading used by the vendor driver for untrimmed chips */
#define SAR_TS_CAL_DEFAULT	400

#define SAR_TIMEOUT_US		10000

struct sstar_sar {
	void __iomem *base;
	/* Serializes the conversions */
	struct mutex lock;
	u16 ts_cal;
};

static int sstar_sar_read(struct sstar_sar *sar, unsigned int ch, int *val)
{
	u16 ctrl;
	int ret;

	guard(mutex)(&sar->lock);

	ctrl = readw(sar->base + SAR_CTRL);
	writew(ctrl | SAR_CTRL_START, sar->base + SAR_CTRL);

	ret = readw_poll_timeout(sar->base + SAR_CTRL, ctrl,
				 !(ctrl & SAR_CTRL_START), 1, SAR_TIMEOUT_US);
	if (ret)
		return ret;

	*val = readw(sar->base + SAR_DATA(ch)) & SAR_DATA_MASK;

	return 0;
}

static int sstar_sar_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	struct sstar_sar *sar = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = sstar_sar_read(sar, chan->address, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		ret = sstar_sar_read(sar, chan->address, val);
		if (ret)
			return ret;

		*val = SAR_TS_SLOPE * (sar->ts_cal - *val) + SAR_TS_CAL_TEMP;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info sstar_sar_info = {
	.read_raw = sstar_sar_read_raw,
};

#define SSTAR_SAR_CHANNEL(_ch)					\
	{							\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.channel = _ch,					\
		.address = _ch,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	}

/* The temperature channel is last, it is only there with its calibration */
static const struct iio_chan_spec sstar_sar_channels[] = {
	SSTAR_SAR_CHANNEL(0),
	SSTAR_SAR_CHANNEL(1),
	SSTAR_SAR_CHANNEL(2),
	SSTAR_SAR_CHANNEL(3),
	{
		.type = IIO_TEMP,
		.address = SAR_TS_CHANNEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static void sstar_sar_ts_disable(void *data)
{
	regmap_clear_bits(data, PMSLEEP_TS, PMSLEEP_TS_EN);
}

/* Returns 0 without the temperature sensor description */
static int sstar_sar_ts_init(struct device *dev, struct sstar_sar *sar)
{
	struct regmap *pmsleep;
	int ret, val;
	u16 cal;

	if (!of_property_present(dev->of_node, "sstar,pmsleep"))
		return 0;

	pmsleep = syscon_regmap_lookup_by_phandle(dev->of_node,
						  "sstar,pmsleep");
	if (IS_ERR(pmsleep))
		return dev_err_probe(dev, PTR_ERR(pmsleep),
				     "failed to get the PM sleep block\n");

	ret = nvmem_cell_read_u16(dev, "ts-calib", &cal);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read the calibration\n");

	sar->ts_cal = cal ?: SAR_TS_CAL_DEFAULT;

	writew(readw(sar->base + SAR_TS_REF) & ~SAR_TS_REF_SEL,
	       sar->base + SAR_TS_REF);
	writew(readw(sar->base + SAR_TS_CTRL) & ~SAR_TS_CTRL_DUMMY,
	       sar->base + SAR_TS_CTRL);

	ret = regmap_set_bits(pmsleep, PMSLEEP_TS, PMSLEEP_TS_EN);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, sstar_sar_ts_disable, pmsleep);
	if (ret)
		return ret;

	/* The first conversion after power up is not valid */
	ret = sstar_sar_read(sar, SAR_TS_CHANNEL, &val);
	if (ret)
		return ret;

	return 1;
}

static int sstar_sar_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct sstar_sar *sar;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*sar));
	if (!indio_dev)
		return -ENOMEM;

	sar = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &sar->lock);
	if (ret)
		return ret;

	sar->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sar->base))
		return PTR_ERR(sar->base);

	writew(SAR_CTRL_INIT, sar->base + SAR_CTRL);

	ret = sstar_sar_ts_init(dev, sar);
	if (ret < 0)
		return ret;

	indio_dev->name = "sstar-sar-adc";
	indio_dev->info = &sstar_sar_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = sstar_sar_channels;
	indio_dev->num_channels = ARRAY_SIZE(sstar_sar_channels) - !ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id sstar_sar_of_match[] = {
	{ .compatible = "sstar,ssc338q-sar-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_sar_of_match);

static struct platform_driver sstar_sar_driver = {
	.driver = {
		.name = "sstar-sar-adc",
		.of_match_table = sstar_sar_of_match,
	},
	.probe = sstar_sar_probe,
};
module_platform_driver(sstar_sar_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q SAR ADC driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
