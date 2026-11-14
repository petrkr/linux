// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q random number generator
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* 16-bit registers on the RIU, 4-byte stride */
#define SSTAR_RNG_CTRL		0x00
#define  SSTAR_RNG_CTRL_EN	BIT(7)
#define SSTAR_RNG_DATA		0x08
#define SSTAR_RNG_STATUS	0x0c
#define  SSTAR_RNG_STATUS_READY	BIT(0)

#define SSTAR_RNG_TIMEOUT_US	1000

/*
 * The output is not conditioned: the bits are slightly biased and adjacent
 * bits are correlated. The byte distribution gives a min-entropy of about
 * 0.9 bits per bit, credit half of the data to account for the correlation.
 */
#define SSTAR_RNG_QUALITY	512

struct sstar_rng {
	struct hwrng rng;
	void __iomem *base;
};

static struct sstar_rng *to_sstar_rng(struct hwrng *rng)
{
	return container_of(rng, struct sstar_rng, rng);
}

static int sstar_rng_init(struct hwrng *rng)
{
	struct sstar_rng *priv = to_sstar_rng(rng);
	u16 ctrl;

	ctrl = readw(priv->base + SSTAR_RNG_CTRL);
	writew(ctrl | SSTAR_RNG_CTRL_EN, priv->base + SSTAR_RNG_CTRL);

	return 0;
}

static void sstar_rng_cleanup(struct hwrng *rng)
{
	struct sstar_rng *priv = to_sstar_rng(rng);
	u16 ctrl;

	ctrl = readw(priv->base + SSTAR_RNG_CTRL);
	writew(ctrl & ~SSTAR_RNG_CTRL_EN, priv->base + SSTAR_RNG_CTRL);
}

static int sstar_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct sstar_rng *priv = to_sstar_rng(rng);
	u16 *data = buf;
	size_t read = 0;
	u16 status;
	int ret;

	while (read + sizeof(*data) <= max) {
		if (wait) {
			ret = readw_poll_timeout(priv->base + SSTAR_RNG_STATUS,
						 status,
						 status & SSTAR_RNG_STATUS_READY,
						 1, SSTAR_RNG_TIMEOUT_US);
			if (ret)
				return read ? read : ret;
		} else if (!(readw(priv->base + SSTAR_RNG_STATUS) &
			     SSTAR_RNG_STATUS_READY)) {
			break;
		}

		*data++ = readw(priv->base + SSTAR_RNG_DATA);
		read += sizeof(*data);
	}

	return read;
}

static int sstar_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sstar_rng *priv;
	struct clk *clk;

	priv = devm_kzalloc_obj(dev, *priv);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	priv->rng.name = dev_name(dev);
	priv->rng.init = sstar_rng_init;
	priv->rng.cleanup = sstar_rng_cleanup;
	priv->rng.read = sstar_rng_read;
	priv->rng.quality = SSTAR_RNG_QUALITY;

	return devm_hwrng_register(dev, &priv->rng);
}

static const struct of_device_id sstar_rng_of_match[] = {
	{ .compatible = "sstar,ssc338q-rng" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_rng_of_match);

static struct platform_driver sstar_rng_driver = {
	.driver = {
		.name = "sstar-rng",
		.of_match_table = sstar_rng_of_match,
	},
	.probe = sstar_rng_probe,
};
module_platform_driver(sstar_rng_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q random number generator driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
