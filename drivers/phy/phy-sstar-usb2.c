// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q USB 2.0 PHY
 *
 * The PHY is a UTMI transceiver with a battery charger detection block
 * on its data lines. Most of the bits are not documented.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

/* 16-bit registers on the RIU, 4-byte stride */
#define UTMI_PWR		0x00
#define  UTMI_PWR_ON		0x0001
#define  UTMI_PWR_TERM_OV	BIT(1)
#define  UTMI_PWR_DP_PDEN	BIT(6)
#define  UTMI_PWR_DM_PDEN	BIT(7)
#define  UTMI_PWR_TED_PD	BIT(10)
#define  UTMI_PWR_PREAMP_PD	BIT(11)
#define  UTMI_PWR_IREF_PD	BIT(14)
#define UTMI_CTRL1		0x04
#define  UTMI_CTRL1_FL_SEL_OV	BIT(10)
#define  UTMI_CTRL1_DISC_WIN	GENMASK(13, 11)
#define  UTMI_CTRL1_DISC_WIN_VAL	(BIT(11) | BIT(13))
#define UTMI_CTRL2		0x08
#define  UTMI_CTRL2_RST_CLK_GATE	BIT(5)
#define  UTMI_CTRL2_LS_DEGLITCH	BIT(6)
#define UTMI_CTRL3		0x0c
#define  UTMI_CTRL3_RESET	GENMASK(1, 0)
#define  UTMI_CTRL3_HS_CUR_MASK	GENMASK(6, 5)
#define  UTMI_CTRL3_HS_CUR_FORCE	BIT(6)
#define  UTMI_CTRL3_IMPROVED_CDR	BIT(9)
#define UTMI_CLK		0x10
#define  UTMI_CLK_INIT		0x0c2f
#define  UTMI_CLK_240_AS_120	BIT(3)
#define  UTMI_CLK_RX_FIX	(BIT(8) | BIT(15))
#define UTMI_CTRL5		0x14
#define  UTMI_CTRL5_TX_LATCH	BIT(15)
#define UTMI_CTRL10		0x28
#define  UTMI_CTRL10_CHIRP_SRC	BIT(13)
#define  UTMI_CTRL10_55_IF	BIT(14)
#define UTMI_DISC_LEVEL		0x54
#define  UTMI_DISC_LEVEL_MASK	GENMASK(7, 0)
#define  UTMI_DISC_LEVEL_VAL	0x62
#define  UTMI_DISC_LEVEL_CHIRP	0x80
#define UTMI_EYE0		0x58
#define  UTMI_EYE0_VAL		0x0210
#define  UTMI_EYE0_CHIRP	0x0010
#define UTMI_EYE1		0x5c
#define  UTMI_EYE1_VAL		0x8100
#define  UTMI_EYE1_CHIRP	0x0000
#define UTMI_CAL		0x78
#define  UTMI_CAL_START		BIT(0)
#define  UTMI_CAL_END		BIT(1)
#define  UTMI_CAL_VAL		GENMASK(15, 4)
#define UTMI_PREAMBLE		0x7c
#define  UTMI_PREAMBLE_EN	BIT(15)
#define UTMI_CHIRP		0x80
#define  UTMI_CHIRP_HW		BIT(4)
#define UTMI_TERM_OV		0xa4
#define  UTMI_TERM_OV_ALL	GENMASK(5, 0)

#define BC_PWR			0x00
#define  BC_PWR_DOWN		GENMASK(7, 0)
#define BC_CTRL			0x04
#define  BC_CTRL_HOST_EN	BIT(14)
#define BC_SW			0x18
#define  BC_SW_HOST_TRI		BIT(6)

#define UTMI_CAL_TIMEOUT_US	10000

struct sstar_usb2_phy {
	void __iomem *utmi;
	void __iomem *bc;
};

static void sstar_usb2_phy_rmw(void __iomem *reg, u16 clr, u16 set)
{
	writew((readw(reg) & ~clr) | set, reg);
}

static int sstar_usb2_phy_calibrate(struct phy *phy)
{
	struct sstar_usb2_phy *priv = phy_get_drvdata(phy);
	void __iomem *cal = priv->utmi + UTMI_CAL;
	u16 val;
	int ret;

	sstar_usb2_phy_rmw(cal, 0, UTMI_CAL_START);
	usleep_range(1000, 2000);
	sstar_usb2_phy_rmw(cal, UTMI_CAL_START, 0);

	ret = readw_poll_timeout(cal, val, val & UTMI_CAL_END, 10,
				 UTMI_CAL_TIMEOUT_US);
	if (ret)
		return ret;

	val &= UTMI_CAL_VAL;
	if (val == 0 || val == UTMI_CAL_VAL)
		dev_warn(&phy->dev, "calibration failed\n");

	return 0;
}

static int sstar_usb2_phy_power_on(struct phy *phy)
{
	struct sstar_usb2_phy *priv = phy_get_drvdata(phy);
	void __iomem *utmi = priv->utmi;
	int ret;

	writew(UTMI_PWR_ON, utmi + UTMI_PWR);
	writew(UTMI_CLK_INIT, utmi + UTMI_CLK);

	/* The squelch level must be set before the calibration */
	sstar_usb2_phy_rmw(utmi + UTMI_DISC_LEVEL, UTMI_DISC_LEVEL_MASK,
			   UTMI_DISC_LEVEL_VAL & 0xf);

	ret = sstar_usb2_phy_calibrate(phy);
	if (ret)
		return ret;

	sstar_usb2_phy_rmw(utmi + UTMI_CTRL3, UTMI_CTRL3_HS_CUR_MASK,
			   UTMI_CTRL3_HS_CUR_FORCE);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL1, UTMI_CTRL1_DISC_WIN,
			   UTMI_CTRL1_DISC_WIN_VAL);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL3, UTMI_CTRL3_IMPROVED_CDR, 0);
	sstar_usb2_phy_rmw(utmi + UTMI_CLK, 0,
			   UTMI_CLK_240_AS_120 | UTMI_CLK_RX_FIX);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL5, 0, UTMI_CTRL5_TX_LATCH);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL10, 0,
			   UTMI_CTRL10_CHIRP_SRC | UTMI_CTRL10_55_IF);
	sstar_usb2_phy_rmw(utmi + UTMI_CHIRP, UTMI_CHIRP_HW, 0);
	sstar_usb2_phy_rmw(utmi + UTMI_DISC_LEVEL, UTMI_DISC_LEVEL_MASK,
			   UTMI_DISC_LEVEL_VAL);
	sstar_usb2_phy_rmw(utmi + UTMI_EYE0, 0, UTMI_EYE0_VAL);
	sstar_usb2_phy_rmw(utmi + UTMI_EYE1, 0, UTMI_EYE1_VAL);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL2, 0,
			   UTMI_CTRL2_LS_DEGLITCH | UTMI_CTRL2_RST_CLK_GATE);
	sstar_usb2_phy_rmw(utmi + UTMI_PREAMBLE, 0, UTMI_PREAMBLE_EN);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL1, 0, UTMI_CTRL1_FL_SEL_OV);

	/* The charger detection switch disconnects the data lines */
	sstar_usb2_phy_rmw(priv->bc + BC_SW, BC_SW_HOST_TRI, 0);
	sstar_usb2_phy_rmw(priv->bc + BC_CTRL, BC_CTRL_HOST_EN, 0);
	sstar_usb2_phy_rmw(priv->bc + BC_PWR, BC_PWR_DOWN, 0);
	sstar_usb2_phy_rmw(utmi + UTMI_PWR, UTMI_PWR_IREF_PD, 0);

	return 0;
}

static int sstar_usb2_phy_power_off(struct phy *phy)
{
	struct sstar_usb2_phy *priv = phy_get_drvdata(phy);
	void __iomem *utmi = priv->utmi;

	/* Pull the data lines low and power down the transceiver */
	sstar_usb2_phy_rmw(utmi + UTMI_PWR, 0,
			   UTMI_PWR_TERM_OV | UTMI_PWR_DP_PDEN |
			   UTMI_PWR_DM_PDEN);
	sstar_usb2_phy_rmw(utmi + UTMI_TERM_OV, 0, UTMI_TERM_OV_ALL);
	sstar_usb2_phy_rmw(utmi + UTMI_PWR, 0,
			   UTMI_PWR_TED_PD | UTMI_PWR_PREAMP_PD |
			   UTMI_PWR_IREF_PD);
	sstar_usb2_phy_rmw(priv->bc + BC_PWR, 0, BC_PWR_DOWN);

	return 0;
}

/*
 * Called before a port reset. The high-speed handshake needs a lower
 * chirp detection level and a different eye setting than normal
 * operation.
 */
static int sstar_usb2_phy_prepare_reset(struct phy *phy)
{
	struct sstar_usb2_phy *priv = phy_get_drvdata(phy);
	void __iomem *utmi = priv->utmi;

	writew(UTMI_EYE0_CHIRP, utmi + UTMI_EYE0);
	writew(UTMI_EYE1_CHIRP, utmi + UTMI_EYE1);
	sstar_usb2_phy_rmw(utmi + UTMI_DISC_LEVEL, UTMI_DISC_LEVEL_MASK,
			   UTMI_DISC_LEVEL_CHIRP);

	return 0;
}

/* Called after a port reset, restores the normal operation */
static int sstar_usb2_phy_reset(struct phy *phy)
{
	struct sstar_usb2_phy *priv = phy_get_drvdata(phy);
	void __iomem *utmi = priv->utmi;

	writew(UTMI_EYE0_VAL, utmi + UTMI_EYE0);
	writew(UTMI_EYE1_VAL, utmi + UTMI_EYE1);
	sstar_usb2_phy_rmw(utmi + UTMI_DISC_LEVEL, UTMI_DISC_LEVEL_MASK,
			   UTMI_DISC_LEVEL_VAL);

	sstar_usb2_phy_rmw(utmi + UTMI_CTRL3, 0, UTMI_CTRL3_RESET);
	sstar_usb2_phy_rmw(utmi + UTMI_CTRL3, UTMI_CTRL3_RESET, 0);

	return 0;
}

static const struct phy_ops sstar_usb2_phy_ops = {
	.power_on = sstar_usb2_phy_power_on,
	.power_off = sstar_usb2_phy_power_off,
	.calibrate = sstar_usb2_phy_prepare_reset,
	.reset = sstar_usb2_phy_reset,
	.owner = THIS_MODULE,
};

static int sstar_usb2_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct sstar_usb2_phy *priv;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->utmi = devm_platform_ioremap_resource_byname(pdev, "utmi");
	if (IS_ERR(priv->utmi))
		return PTR_ERR(priv->utmi);

	priv->bc = devm_platform_ioremap_resource_byname(pdev, "bc");
	if (IS_ERR(priv->bc))
		return PTR_ERR(priv->bc);

	phy = devm_phy_create(dev, NULL, &sstar_usb2_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "failed to create PHY\n");

	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id sstar_usb2_phy_of_match[] = {
	{ .compatible = "sstar,ssc338q-usb2-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_usb2_phy_of_match);

static struct platform_driver sstar_usb2_phy_driver = {
	.driver = {
		.name = "sstar-usb2-phy",
		.of_match_table = sstar_usb2_phy_of_match,
	},
	.probe = sstar_usb2_phy_probe,
};
module_platform_driver(sstar_usb2_phy_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q USB 2.0 PHY driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
