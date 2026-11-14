// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q I2C master (MIIC)
 *
 * The controller generates start and stop conditions on command and
 * transfers one byte at a time, flagging the end of each step in its
 * interrupt status register.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/*
 * 16-bit registers on the RIU with a 4-byte stride, some of them used as
 * two byte registers
 */
#define MIIC_REG(word)		((word) * 4)
#define MIIC_REG_HI(word)	((word) * 4 + 1)

#define MIIC_CFG		MIIC_REG(0x00)
#define  MIIC_CFG_RESET		BIT(0)
#define  MIIC_CFG_EN_CLKSTR	BIT(3)
#define  MIIC_CFG_EN_FILTER	BIT(5)
#define  MIIC_CFG_EN_PUSH1T	BIT(6)
#define MIIC_CMD_START		MIIC_REG(0x01)
#define MIIC_CMD_STOP		MIIC_REG_HI(0x01)
#define  MIIC_CMD_GO		BIT(0)
#define MIIC_WDATA		MIIC_REG(0x02)
#define MIIC_WDATA_GET		MIIC_REG_HI(0x02)
#define  MIIC_WDATA_GET_NACK	BIT(0)
#define MIIC_RDATA		MIIC_REG(0x03)
#define MIIC_RDATA_CFG		MIIC_REG_HI(0x03)
#define  MIIC_RDATA_CFG_TRIG	BIT(0)
#define  MIIC_RDATA_CFG_NACK	BIT(1)
#define MIIC_INT_CTL		MIIC_REG(0x04)
#define  MIIC_INT_FLAG		BIT(0)
#define MIIC_CUR_STATE		MIIC_REG(0x05)
#define  MIIC_CUR_STATE_MASK	GENMASK(4, 0)
#define MIIC_STP_CNT		MIIC_REG(0x08)
#define MIIC_CKH_CNT		MIIC_REG(0x09)
#define MIIC_CKL_CNT		MIIC_REG(0x0a)
#define MIIC_SDA_CNT		MIIC_REG(0x0b)
#define MIIC_STT_CNT		MIIC_REG(0x0c)
#define MIIC_LTH_CNT		MIIC_REG(0x0d)

/* Timing from the vendor driver, in input clock cycles */
#define MIIC_STT_STP_CYCLES	38
#define MIIC_SDA_LTH_CYCLES	5
/* Cycles of an SCL period spent outside the high and low counters */
#define MIIC_CLK_OVERHEAD	8

#define MIIC_TIMEOUT_US		10000

struct sstar_i2c {
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *base;
};

static int sstar_i2c_wait(struct sstar_i2c *i2c)
{
	u8 val;
	int ret;

	ret = readb_poll_timeout(i2c->base + MIIC_INT_CTL, val,
				 val & MIIC_INT_FLAG, 2, MIIC_TIMEOUT_US);
	writeb(MIIC_INT_FLAG, i2c->base + MIIC_INT_CTL);

	return ret;
}

static void sstar_i2c_reset(struct sstar_i2c *i2c)
{
	u8 cfg = MIIC_CFG_EN_CLKSTR | MIIC_CFG_EN_FILTER | MIIC_CFG_EN_PUSH1T;

	writeb(MIIC_INT_FLAG, i2c->base + MIIC_INT_CTL);
	writeb(cfg | MIIC_CFG_RESET, i2c->base + MIIC_CFG);
	writeb(cfg, i2c->base + MIIC_CFG);
}

static int sstar_i2c_start(struct sstar_i2c *i2c)
{
	writeb(MIIC_CMD_GO, i2c->base + MIIC_CMD_START);

	return sstar_i2c_wait(i2c);
}

/*
 * The state machine does not always return to 0 after a stop; the vendor
 * driver also accepts the interrupt flag that the stop raises.
 */
static bool sstar_i2c_stop_done(struct sstar_i2c *i2c)
{
	return !(readb(i2c->base + MIIC_CUR_STATE) & MIIC_CUR_STATE_MASK) ||
	       (readb(i2c->base + MIIC_INT_CTL) & MIIC_INT_FLAG);
}

static void sstar_i2c_stop(struct sstar_i2c *i2c)
{
	bool done;

	writeb(MIIC_CMD_GO, i2c->base + MIIC_CMD_STOP);
	read_poll_timeout(sstar_i2c_stop_done, done, done, 2, MIIC_TIMEOUT_US,
			  false, i2c);
	writeb(MIIC_INT_FLAG, i2c->base + MIIC_INT_CTL);
}

static int sstar_i2c_write_byte(struct sstar_i2c *i2c, u8 byte)
{
	int ret;

	writeb(byte, i2c->base + MIIC_WDATA);

	ret = sstar_i2c_wait(i2c);
	if (ret)
		return ret;

	if (readb(i2c->base + MIIC_WDATA_GET) & MIIC_WDATA_GET_NACK)
		return -ENXIO;

	return 0;
}

static int sstar_i2c_read_byte(struct sstar_i2c *i2c, u8 *byte, bool last)
{
	int ret;

	writeb(MIIC_RDATA_CFG_TRIG | (last ? MIIC_RDATA_CFG_NACK : 0),
	       i2c->base + MIIC_RDATA_CFG);

	ret = sstar_i2c_wait(i2c);
	if (ret)
		return ret;

	*byte = readb(i2c->base + MIIC_RDATA);

	return 0;
}

static int sstar_i2c_msg(struct sstar_i2c *i2c, struct i2c_msg *msg)
{
	bool read = msg->flags & I2C_M_RD;
	unsigned int i;
	int ret;

	ret = sstar_i2c_start(i2c);
	if (ret)
		return ret;

	ret = sstar_i2c_write_byte(i2c, i2c_8bit_addr_from_msg(msg));
	if (ret)
		return ret;

	for (i = 0; i < msg->len; i++) {
		if (read)
			ret = sstar_i2c_read_byte(i2c, &msg->buf[i],
						  i == msg->len - 1);
		else
			ret = sstar_i2c_write_byte(i2c, msg->buf[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int sstar_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			  int num)
{
	struct sstar_i2c *i2c = i2c_get_adapdata(adap);
	int i, ret = 0;

	for (i = 0; i < num; i++) {
		ret = sstar_i2c_msg(i2c, &msgs[i]);
		if (ret)
			break;
	}

	sstar_i2c_stop(i2c);

	if (ret) {
		if (ret != -ENXIO)
			dev_dbg(i2c->dev, "transfer failed: %d\n", ret);
		sstar_i2c_reset(i2c);
		return ret;
	}

	return num;
}

static u32 sstar_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sstar_i2c_algo = {
	.xfer		= sstar_i2c_xfer,
	.functionality	= sstar_i2c_func,
};

static int sstar_i2c_set_timing(struct sstar_i2c *i2c, unsigned long rate,
				u32 bus_freq)
{
	unsigned int cycles, high, low;

	cycles = DIV_ROUND_UP(rate, bus_freq);
	if (cycles <= MIIC_CLK_OVERHEAD + 4)
		return -EINVAL;

	cycles -= MIIC_CLK_OVERHEAD;
	high = cycles / 2 - 1;
	low = cycles - high;

	writew(high, i2c->base + MIIC_CKH_CNT);
	writew(low, i2c->base + MIIC_CKL_CNT);
	writew(MIIC_STT_STP_CYCLES, i2c->base + MIIC_STT_CNT);
	writew(MIIC_STT_STP_CYCLES, i2c->base + MIIC_STP_CNT);
	writew(MIIC_SDA_LTH_CYCLES, i2c->base + MIIC_SDA_CNT);
	writew(MIIC_SDA_LTH_CYCLES, i2c->base + MIIC_LTH_CNT);

	return 0;
}

static int sstar_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_timings timings;
	struct sstar_i2c *i2c;
	struct clk *clk;
	int ret;

	i2c = devm_kzalloc_obj(dev, *i2c);
	if (!i2c)
		return -ENOMEM;

	i2c->dev = dev;

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	i2c_parse_fw_timings(dev, &timings, true);

	sstar_i2c_reset(i2c);

	ret = sstar_i2c_set_timing(i2c, clk_get_rate(clk),
				   timings.bus_freq_hz);
	if (ret)
		return dev_err_probe(dev, ret, "unsupported bus frequency %u\n",
				     timings.bus_freq_hz);

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &sstar_i2c_algo;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;
	strscpy(i2c->adap.name, dev_name(dev), sizeof(i2c->adap.name));
	i2c_set_adapdata(&i2c->adap, i2c);

	return devm_i2c_add_adapter(dev, &i2c->adap);
}

static const struct of_device_id sstar_i2c_of_match[] = {
	{ .compatible = "sstar,ssc338q-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_i2c_of_match);

static struct platform_driver sstar_i2c_driver = {
	.driver = {
		.name = "sstar-i2c",
		.of_match_table = sstar_i2c_of_match,
	},
	.probe = sstar_i2c_probe,
};
module_platform_driver(sstar_i2c_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q I2C master driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
