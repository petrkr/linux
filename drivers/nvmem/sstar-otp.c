// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q OTP
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>

/*
 * The fuse words are mirrored in 16-bit registers on the RIU, 4-byte
 * stride. Word n is at byte offset 2 * n of the NVMEM device.
 */
#define SSTAR_OTP_WORD_SIZE	2
#define SSTAR_OTP_REG_STRIDE	4

static int sstar_otp_read(void *context, unsigned int offset, void *val,
			  size_t bytes)
{
	void __iomem *base = context;
	u16 *data = val;
	unsigned int word = offset / SSTAR_OTP_WORD_SIZE;

	for (; bytes >= SSTAR_OTP_WORD_SIZE; bytes -= SSTAR_OTP_WORD_SIZE)
		*data++ = readw(base + word++ * SSTAR_OTP_REG_STRIDE);

	return 0;
}

static int sstar_otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {};
	struct nvmem_device *nvmem;
	struct resource *res;
	void __iomem *base;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	config.dev = dev;
	config.name = "sstar-otp";
	config.id = NVMEM_DEVID_NONE;
	config.owner = THIS_MODULE;
	config.read_only = true;
	config.root_only = true;
	config.word_size = SSTAR_OTP_WORD_SIZE;
	config.stride = SSTAR_OTP_WORD_SIZE;
	config.size = resource_size(res) / SSTAR_OTP_REG_STRIDE *
		      SSTAR_OTP_WORD_SIZE;
	config.reg_read = sstar_otp_read;
	config.priv = base;

	nvmem = devm_nvmem_register(dev, &config);

	return PTR_ERR_OR_ZERO(nvmem);
}

static const struct of_device_id sstar_otp_of_match[] = {
	{ .compatible = "sstar,ssc338q-otp" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_otp_of_match);

static struct platform_driver sstar_otp_driver = {
	.driver = {
		.name = "sstar-otp",
		.of_match_table = sstar_otp_of_match,
	},
	.probe = sstar_otp_probe,
};
module_platform_driver(sstar_otp_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q OTP driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
