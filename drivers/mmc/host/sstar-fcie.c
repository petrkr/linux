// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q FCIE SD host controller
 *
 * The controller sends a command from its command FIFO, receives the
 * response into the same FIFO and moves the data between the card and
 * memory with its own DMA engine, one contiguous buffer per request.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

/* 16-bit registers on the RIU, 4-byte stride */
#define FCIE_MIE_EVENT		0x00
#define FCIE_MIE_INT_EN		0x01
#define FCIE_MMA_PRI		0x02
#define FCIE_DMA_ADDR_L		0x03
#define FCIE_DMA_ADDR_H		0x04
#define FCIE_DMA_LEN_L		0x05
#define FCIE_DMA_LEN_H		0x06
#define FCIE_MIE_FUNC_CTL	0x07
#define FCIE_JOB_BLK_CNT	0x08
#define FCIE_BLK_SIZE		0x09
#define FCIE_CMD_RSP_SIZE	0x0a
#define FCIE_MODE		0x0b
#define FCIE_CTL		0x0c
#define FCIE_STS		0x0d
#define FCIE_BOOT_MOD		0x0e
#define FCIE_DDR_MOD		0x0f
#define FCIE_FIFO(byte)		(0x20 + (byte) / 2)
#define FCIE_BOOT		0x37
#define FCIE_RST		0x3f

#define EV_DATA_END		BIT(0)
#define EV_CMD_END		BIT(1)
#define EV_ERR_STS		BIT(2)
#define EV_BUSY_END		BIT(4)
#define EV_R2N_RDY		BIT(5)
#define EV_ALL			(EV_DATA_END | EV_CMD_END | EV_ERR_STS | \
				 EV_BUSY_END | EV_R2N_RDY)

#define MMA_PRI_RW		(BIT(0) | BIT(1))
#define FUNC_SDIO_MODE		BIT(2)

#define MODE_CLK_EN		BIT(0)
#define MODE_BUS_4		BIT(1)
#define MODE_DMA_RD_CLK_STOP	BIT(7)

#define CTL_RSPR2_EN		BIT(0)
#define CTL_RSP_EN		BIT(1)
#define CTL_CMD_EN		BIT(2)
#define CTL_DTRX_EN		BIT(3)
#define CTL_JOB_DIR		BIT(4)
#define CTL_JOB_START		BIT(6)

#define STS_RD_CERR		BIT(0)
#define STS_WR_CERR		BIT(1)
#define STS_WR_TOUT		BIT(2)
#define STS_CMD_NORSP		BIT(3)
#define STS_CMDRSP_CERR		BIT(4)
#define STS_RD_TOUT		BIT(5)
#define STS_DAT0		BIT(8)
#define STS_ERRS		GENMASK(5, 0)
#define STS_DATA_ERRS		(STS_RD_CERR | STS_WR_CERR | STS_WR_TOUT | \
				 STS_RD_TOUT)

#define CMD_SIZE		(5 << 8)
#define BOOT_MOD_BOOT		BIT(2)
#define BOOT_NAND_SRAM_IMI	GENMASK(2, 0)
#define RST_SOFT		BIT(0)
#define RST_STS			GENMASK(3, 1)

/* Pad timing used by the vendor driver */
#define DDR_MOD_TIMING		(BIT(10) | BIT(13) | BIT(14))

#define FCIE_RESET_TIMEOUT_US	100000
#define FCIE_CMD_TIMEOUT_MS	100
#define FCIE_DATA_TIMEOUT_MS	3000
#define FCIE_BUSY_TIMEOUT_US	1000000

#define FCIE_MAX_BLOCKS		1024

struct sstar_fcie {
	struct mmc_host *mmc;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct work_struct work;
	struct completion done;
	struct mmc_request *mrq;
	u16 mode;
	unsigned int nrc_us;
	u16 events;
};

static u16 fcie_readw(struct sstar_fcie *host, unsigned int reg)
{
	return readw(host->base + reg * 4);
}

static void fcie_writew(struct sstar_fcie *host, u16 val, unsigned int reg)
{
	writew(val, host->base + reg * 4);
}

static void fcie_rmw(struct sstar_fcie *host, unsigned int reg, u16 clear,
		     u16 set)
{
	fcie_writew(host, (fcie_readw(host, reg) & ~clear) | set, reg);
}

static u8 fcie_fifo_byte(struct sstar_fcie *host, unsigned int byte)
{
	return fcie_readw(host, FCIE_FIFO(byte)) >> (byte & 1 ? 8 : 0);
}

static u32 fcie_rsp_word(struct sstar_fcie *host, unsigned int first)
{
	return fcie_fifo_byte(host, first) << 24 |
	       fcie_fifo_byte(host, first + 1) << 16 |
	       fcie_fifo_byte(host, first + 2) << 8 |
	       fcie_fifo_byte(host, first + 3);
}

static void fcie_reset(struct sstar_fcie *host)
{
	u16 val;

	fcie_rmw(host, FCIE_CTL, CTL_JOB_START, 0);
	fcie_rmw(host, FCIE_RST, RST_SOFT, 0);
	readw_poll_timeout(host->base + FCIE_RST * 4, val,
			   (val & RST_STS) == RST_STS, 10,
			   FCIE_RESET_TIMEOUT_US);
	fcie_rmw(host, FCIE_RST, 0, RST_SOFT);
	readw_poll_timeout(host->base + FCIE_RST * 4, val, !(val & RST_STS),
			   10, FCIE_RESET_TIMEOUT_US);
}

static void fcie_clear_status(struct sstar_fcie *host)
{
	fcie_writew(host, STS_ERRS, FCIE_STS);
	fcie_writew(host, EV_ALL, FCIE_MIE_EVENT);
}

static irqreturn_t sstar_fcie_irq(int irq, void *dev_id)
{
	struct sstar_fcie *host = dev_id;
	u16 events;

	events = fcie_readw(host, FCIE_MIE_EVENT) &
		 fcie_readw(host, FCIE_MIE_INT_EN);
	if (!events)
		return IRQ_NONE;

	fcie_writew(host, 0, FCIE_MIE_INT_EN);
	host->events = events;
	complete(&host->done);

	return IRQ_HANDLED;
}

/* Start the job set up in the control register and wait for the events */
static int fcie_run(struct sstar_fcie *host, u16 ctl, u16 events,
		    unsigned int timeout_ms)
{
	reinit_completion(&host->done);
	fcie_writew(host, events | EV_ERR_STS, FCIE_MIE_INT_EN);
	fcie_rmw(host, FCIE_CTL, 0, ctl | CTL_JOB_START);

	if (!wait_for_completion_timeout(&host->done,
					 msecs_to_jiffies(timeout_ms))) {
		fcie_writew(host, 0, FCIE_MIE_INT_EN);
		return -ETIMEDOUT;
	}

	/* Wait for the remaining events of a combined command and data job */
	while ((fcie_readw(host, FCIE_MIE_EVENT) & events) != events) {
		if (fcie_readw(host, FCIE_STS) & STS_ERRS)
			break;
		reinit_completion(&host->done);
		fcie_writew(host, events | EV_ERR_STS, FCIE_MIE_INT_EN);
		if (!wait_for_completion_timeout(&host->done,
						 msecs_to_jiffies(timeout_ms))) {
			fcie_writew(host, 0, FCIE_MIE_INT_EN);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int fcie_cmd_status(struct sstar_fcie *host, struct mmc_command *cmd)
{
	u16 sts = fcie_readw(host, FCIE_STS);

	if (!(mmc_resp_type(cmd) & MMC_RSP_CRC))
		sts &= ~STS_CMDRSP_CERR;

	if (sts & STS_CMD_NORSP)
		return -ETIMEDOUT;
	if (sts & STS_CMDRSP_CERR)
		return -EILSEQ;

	return 0;
}

static int fcie_data_status(struct sstar_fcie *host)
{
	u16 sts = fcie_readw(host, FCIE_STS);

	if (sts & (STS_WR_TOUT | STS_RD_TOUT))
		return -ETIMEDOUT;
	if (sts & (STS_RD_CERR | STS_WR_CERR))
		return -EILSEQ;

	return 0;
}

static void fcie_setup_cmd(struct sstar_fcie *host, struct mmc_command *cmd,
			   struct mmc_data *data, dma_addr_t addr)
{
	u16 rsp_size = 0, ctl = 0;
	u32 arg = cmd->arg;

	if (mmc_resp_type(cmd) & MMC_RSP_136) {
		rsp_size = 0x10;
		ctl = CTL_RSPR2_EN | CTL_RSP_EN;
	} else if (mmc_resp_type(cmd) & MMC_RSP_PRESENT) {
		rsp_size = 5;
		ctl = CTL_RSP_EN;
	}

	fcie_writew(host, (arg >> 24) << 8 | 0x40 | cmd->opcode, FCIE_FIFO(0));
	fcie_writew(host, (arg & 0xff00) | ((arg >> 16) & 0xff), FCIE_FIFO(2));
	fcie_writew(host, arg & 0xff, FCIE_FIFO(4));

	if (data) {
		u32 len = data->blocks * data->blksz;

		fcie_writew(host, data->blksz, FCIE_BLK_SIZE);
		fcie_writew(host, data->blocks, FCIE_JOB_BLK_CNT);
		fcie_writew(host, len & 0xffff, FCIE_DMA_LEN_L);
		fcie_writew(host, len >> 16, FCIE_DMA_LEN_H);
		fcie_writew(host, lower_32_bits(addr) & 0xffff,
			    FCIE_DMA_ADDR_L);
		fcie_writew(host, lower_32_bits(addr) >> 16, FCIE_DMA_ADDR_H);
	}

	fcie_writew(host, CMD_SIZE | rsp_size, FCIE_CMD_RSP_SIZE);
	fcie_writew(host, FUNC_SDIO_MODE, FCIE_MIE_FUNC_CTL);
	fcie_writew(host, host->mode | (data ? MODE_DMA_RD_CLK_STOP : 0),
		    FCIE_MODE);
	fcie_writew(host, ctl | (data && data->flags & MMC_DATA_WRITE ?
				 CTL_JOB_DIR : 0), FCIE_CTL);
	fcie_writew(host, MMA_PRI_RW, FCIE_MMA_PRI);
	fcie_writew(host, DDR_MOD_TIMING, FCIE_DDR_MOD);
	fcie_rmw(host, FCIE_BOOT_MOD, BOOT_MOD_BOOT, 0);
	fcie_rmw(host, FCIE_BOOT, BOOT_NAND_SRAM_IMI, 0);

	/* Ncr: keep the clock running between commands */
	udelay(host->nrc_us);
	fcie_clear_status(host);
}

static void fcie_read_rsp(struct sstar_fcie *host, struct mmc_command *cmd)
{
	cmd->resp[0] = fcie_rsp_word(host, 1);
	if (mmc_resp_type(cmd) & MMC_RSP_136) {
		cmd->resp[1] = fcie_rsp_word(host, 5);
		cmd->resp[2] = fcie_rsp_word(host, 9);
		cmd->resp[3] = fcie_rsp_word(host, 13) & ~0xff;
	}
}

static int fcie_wait_busy(struct sstar_fcie *host)
{
	u16 sts;

	return readw_poll_timeout(host->base + FCIE_STS * 4, sts,
				  sts & STS_DAT0, 10, FCIE_BUSY_TIMEOUT_US);
}

static void fcie_send_cmd(struct sstar_fcie *host, struct mmc_command *cmd,
			  struct mmc_data *data, dma_addr_t addr)
{
	bool read = data && data->flags & MMC_DATA_READ;
	int ret;

	fcie_setup_cmd(host, cmd, data, addr);

	if (read)
		ret = fcie_run(host, CTL_CMD_EN | CTL_DTRX_EN,
			       EV_CMD_END | EV_DATA_END, FCIE_DATA_TIMEOUT_MS);
	else
		ret = fcie_run(host, CTL_CMD_EN, EV_CMD_END,
			       FCIE_CMD_TIMEOUT_MS);

	cmd->error = fcie_cmd_status(host, cmd);
	if (!cmd->error && ret)
		cmd->error = ret;
	if (cmd->error)
		goto err;

	fcie_read_rsp(host, cmd);

	if ((mmc_resp_type(cmd) & MMC_RSP_BUSY) && fcie_wait_busy(host)) {
		cmd->error = -ETIMEDOUT;
		goto err;
	}

	if (!data)
		return;

	if (!read) {
		fcie_clear_status(host);
		fcie_writew(host, CTL_JOB_DIR, FCIE_CTL);
		ret = fcie_run(host, CTL_DTRX_EN, EV_DATA_END,
			       FCIE_DATA_TIMEOUT_MS);
		data->error = fcie_data_status(host) ?: ret;
	} else {
		data->error = fcie_data_status(host);
	}

	if (!data->error)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		fcie_reset(host);

	return;

err:
	if (data)
		data->error = cmd->error;
	fcie_reset(host);
}

static void sstar_fcie_work(struct work_struct *work)
{
	struct sstar_fcie *host = container_of(work, struct sstar_fcie, work);
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data = mrq->data;
	dma_addr_t addr = 0;

	if (data) {
		enum dma_data_direction dir = mmc_get_dma_dir(data);

		if (dma_map_sg(host->dev, data->sg, data->sg_len, dir) != 1) {
			mrq->cmd->error = -ENOMEM;
			goto done;
		}
		addr = sg_dma_address(data->sg);
	}

	fcie_send_cmd(host, mrq->cmd, data, addr);

	if (data) {
		dma_unmap_sg(host->dev, data->sg, data->sg_len,
			     mmc_get_dma_dir(data));

		if (mrq->stop && (mrq->cmd->error || !mrq->sbc))
			fcie_send_cmd(host, mrq->stop, NULL, 0);
	}

done:
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

static void sstar_fcie_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sstar_fcie *host = mmc_priv(mmc);

	host->mrq = mrq;
	schedule_work(&host->work);
}

static unsigned int fcie_nrc_us(unsigned long rate)
{
	if (!rate)
		return 0;

	/* 8 clock cycles, rounded up */
	return DIV_ROUND_UP(8 * USEC_PER_SEC, rate);
}

static void sstar_fcie_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sstar_fcie *host = mmc_priv(mmc);
	unsigned long rate;
	int ret;

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		break;
	case MMC_POWER_UP:
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		break;
	default:
		break;
	}

	host->mode = ios->bus_width == MMC_BUS_WIDTH_4 ? MODE_BUS_4 : 0;

	if (ios->clock) {
		ret = clk_set_rate(host->clk, ios->clock);
		if (ret)
			dev_err(host->dev, "failed to set clock to %u Hz\n",
				ios->clock);
		rate = clk_get_rate(host->clk);
		host->nrc_us = fcie_nrc_us(rate);
		mmc->actual_clock = rate;
		host->mode |= MODE_CLK_EN;
	} else {
		mmc->actual_clock = 0;
	}

	fcie_writew(host, FUNC_SDIO_MODE, FCIE_MIE_FUNC_CTL);
	fcie_writew(host, host->mode, FCIE_MODE);
}

static const struct mmc_host_ops sstar_fcie_ops = {
	.request	= sstar_fcie_request,
	.set_ios	= sstar_fcie_set_ios,
	.get_cd		= mmc_gpio_get_cd,
	.get_ro		= mmc_gpio_get_ro,
};

static int sstar_fcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sstar_fcie *host;
	struct mmc_host *mmc;
	int irq, ret;

	mmc = devm_mmc_alloc_host(dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dev = dev;
	INIT_WORK(&host->work, sstar_fcie_work);
	init_completion(&host->done);

	host->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->base))
		return PTR_ERR(host->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	host->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(host->clk))
		return dev_err_probe(dev, PTR_ERR(host->clk),
				     "failed to get clock\n");

	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return ret;
	if (!mmc->ocr_avail)
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc->ops = &sstar_fcie_ops;
	mmc->f_min = clk_round_rate(host->clk, 400000);
	if (!mmc->f_max)
		mmc->f_max = clk_round_rate(host->clk, 50000000);
	mmc->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED |
		     MMC_CAP_MMC_HIGHSPEED;

	/* One contiguous DMA buffer per request */
	mmc->max_segs = 1;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = FCIE_MAX_BLOCKS;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	fcie_writew(host, 0, FCIE_MIE_INT_EN);
	fcie_writew(host, FUNC_SDIO_MODE, FCIE_MIE_FUNC_CTL);
	fcie_reset(host);
	fcie_clear_status(host);

	ret = devm_request_irq(dev, irq, sstar_fcie_irq, 0, dev_name(dev),
			       host);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, host);

	return mmc_add_host(mmc);
}

static void sstar_fcie_remove(struct platform_device *pdev)
{
	struct sstar_fcie *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	cancel_work_sync(&host->work);
}

static const struct of_device_id sstar_fcie_of_match[] = {
	{ .compatible = "sstar,ssc338q-fcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_fcie_of_match);

static struct platform_driver sstar_fcie_driver = {
	.driver = {
		.name = "sstar-fcie",
		.of_match_table = sstar_fcie_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = sstar_fcie_probe,
	.remove = sstar_fcie_remove,
};
module_platform_driver(sstar_fcie_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q FCIE SD host controller driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
