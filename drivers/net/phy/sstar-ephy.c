// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q internal Ethernet PHY
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/linkmode.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>

/*
 * The PHY reports an ID of 0, so it is matched by the MAC it is built into.
 * Its analog front end is set up by the MAC driver.
 */
static int sstar_ephy_match_phy_device(struct phy_device *phydev,
				       const struct phy_driver *phydrv)
{
	struct device *parent = phydev->mdio.bus->parent;

	return phydev->phy_id == 0 && parent &&
	       of_device_is_compatible(parent->of_node, "sstar,ssc338q-emac");
}

static int sstar_ephy_get_features(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_abilities(phydev);
	if (ret)
		return ret;

	/* Half duplex does not work, every frame ends in a collision */
	linkmode_clear_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
			   phydev->supported);
	linkmode_clear_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT,
			   phydev->supported);

	return 0;
}

static int sstar_ephy_read_status(struct phy_device *phydev)
{
	int bmcr, lpa, ret;

	ret = genphy_read_status(phydev);
	if (ret)
		return ret;

	if (phydev->autoneg != AUTONEG_ENABLE || !phydev->link)
		return 0;

	lpa = phy_read(phydev, MII_LPA);
	if (lpa < 0)
		return lpa;

	if (lpa & (LPA_10HALF | LPA_10FULL | LPA_100HALF | LPA_100FULL))
		return 0;

	/*
	 * A link partner without autonegotiation is detected by parallel
	 * detection, which leaves the link partner abilities empty. The
	 * detected speed is reported in BMCR. The duplex cannot be detected,
	 * assume full duplex as half duplex does not work.
	 */
	bmcr = phy_read(phydev, MII_BMCR);
	if (bmcr < 0)
		return bmcr;

	phydev->speed = bmcr & BMCR_SPEED100 ? SPEED_100 : SPEED_10;
	phydev->duplex = DUPLEX_FULL;

	return 0;
}

static struct phy_driver sstar_ephy_driver[] = {
	{
		.name			= "SigmaStar SSC338Q internal PHY",
		.match_phy_device	= sstar_ephy_match_phy_device,
		.flags			= PHY_IS_INTERNAL,
		.get_features		= sstar_ephy_get_features,
		.read_status		= sstar_ephy_read_status,
		.suspend		= genphy_suspend,
		.resume			= genphy_resume,
	},
};
module_phy_driver(sstar_ephy_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q internal Ethernet PHY driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
