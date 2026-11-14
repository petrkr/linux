// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q BDMA controller
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/log2.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "dmaengine.h"
#include "virt-dma.h"

#define SSTAR_BDMA_NR_CHANS		4
#define SSTAR_BDMA_CHAN_STRIDE		0x80

/* 16-bit registers on the RIU, 4-byte stride, 32-bit values in two of them */
#define SSTAR_BDMA_CTRL			0x00
#define  SSTAR_BDMA_CTRL_TRIG		BIT(0)
#define  SSTAR_BDMA_CTRL_STOP		BIT(4)
#define SSTAR_BDMA_STATUS		0x04
#define  SSTAR_BDMA_STATUS_BUSY		BIT(1)
#define  SSTAR_BDMA_STATUS_INT		BIT(2)
#define  SSTAR_BDMA_STATUS_DONE		BIT(3)
#define  SSTAR_BDMA_STATUS_CLEAR	(SSTAR_BDMA_STATUS_BUSY | \
					 SSTAR_BDMA_STATUS_INT | \
					 SSTAR_BDMA_STATUS_DONE)
#define SSTAR_BDMA_CONFIG		0x08
#define  SSTAR_BDMA_CONFIG_SRC		GENMASK(3, 0)
#define  SSTAR_BDMA_CONFIG_SRC_DW	GENMASK(6, 4)
#define  SSTAR_BDMA_CONFIG_DST		GENMASK(11, 8)
#define  SSTAR_BDMA_CONFIG_DST_DW	GENMASK(14, 12)
#define SSTAR_BDMA_MISC			0x0c
#define  SSTAR_BDMA_MISC_INT_EN		BIT(1)
/* Memory behind each side of the transfer, 0 selects the MIU */
#define  SSTAR_BDMA_MISC_MEM_SEL	GENMASK(15, 12)
#define SSTAR_BDMA_SRC_ADDR		0x10
#define SSTAR_BDMA_DST_ADDR		0x18
#define SSTAR_BDMA_SIZE			0x20
#define SSTAR_BDMA_PATTERN		0x28
#define SSTAR_BDMA_LINE_EN		0x40
#define SSTAR_BDMA_SRC_WIDTH		0x48
#define SSTAR_BDMA_SRC_PITCH		0x50
#define SSTAR_BDMA_DST_WIDTH		0x58
#define SSTAR_BDMA_DST_PITCH		0x60

/*
 * Source and destination selections. Memory is reached through two ports,
 * the other selections are peripherals and come from the client.
 */
#define SSTAR_BDMA_SEL_MEM0		0
#define SSTAR_BDMA_SEL_MEM1		1
#define SSTAR_BDMA_SEL_FILL		4
#define SSTAR_BDMA_SEL_MAX		15

/* Data widths, log2 of the width in bytes */
#define SSTAR_BDMA_DW_4			2
#define SSTAR_BDMA_DW_16		4

#define SSTAR_BDMA_SLAVE_WIDTHS		(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
					 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
					 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | \
					 BIT(DMA_SLAVE_BUSWIDTH_8_BYTES) | \
					 BIT(DMA_SLAVE_BUSWIDTH_16_BYTES))

#define SSTAR_BDMA_MAX_LEN		U32_MAX
#define SSTAR_BDMA_STOP_TIMEOUT_US	1000

struct sstar_bdma_desc {
	struct virt_dma_desc vd;
	u16 config;
	dma_addr_t src;
	dma_addr_t dst;
	u32 len;
	u32 pattern;
	/* Line mode, used when width is not 0 */
	u32 src_width;
	u32 src_pitch;
	u32 dst_width;
	u32 dst_pitch;
};

struct sstar_bdma_chan {
	struct virt_dma_chan vc;
	void __iomem *base;
	/* Peripheral selection from the client specifier */
	u8 slave_sel;
	struct dma_slave_config slave_cfg;
	/* Descriptor being transferred, NULL when idle */
	struct sstar_bdma_desc *desc;
};

struct sstar_bdma {
	struct dma_device ddev;
	struct sstar_bdma_chan chans[SSTAR_BDMA_NR_CHANS];
};

static struct sstar_bdma_chan *to_sstar_bdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sstar_bdma_chan, vc.chan);
}

static struct sstar_bdma_desc *to_sstar_bdma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct sstar_bdma_desc, vd);
}

static void sstar_bdma_writew(struct sstar_bdma_chan *bc, unsigned int reg,
			      u16 val)
{
	writew(val, bc->base + reg);
}

static void sstar_bdma_writel(struct sstar_bdma_chan *bc, unsigned int reg,
			      u32 val)
{
	writew(lower_16_bits(val), bc->base + reg);
	writew(upper_16_bits(val), bc->base + reg + 4);
}

/* bc->vc.lock must be held */
static void sstar_bdma_start(struct sstar_bdma_chan *bc)
{
	struct virt_dma_desc *vd = vchan_next_desc(&bc->vc);
	struct sstar_bdma_desc *desc;

	if (!vd) {
		bc->desc = NULL;
		return;
	}

	list_del(&vd->node);
	desc = to_sstar_bdma_desc(vd);
	bc->desc = desc;

	sstar_bdma_writew(bc, SSTAR_BDMA_STATUS, SSTAR_BDMA_STATUS_CLEAR);
	sstar_bdma_writew(bc, SSTAR_BDMA_CONFIG, desc->config);
	sstar_bdma_writel(bc, SSTAR_BDMA_SRC_ADDR, desc->src);
	sstar_bdma_writel(bc, SSTAR_BDMA_DST_ADDR, desc->dst);
	sstar_bdma_writel(bc, SSTAR_BDMA_SIZE, desc->len);
	sstar_bdma_writel(bc, SSTAR_BDMA_PATTERN, desc->pattern);

	if (desc->src_width) {
		sstar_bdma_writel(bc, SSTAR_BDMA_SRC_WIDTH, desc->src_width);
		sstar_bdma_writel(bc, SSTAR_BDMA_SRC_PITCH, desc->src_pitch);
		sstar_bdma_writel(bc, SSTAR_BDMA_DST_WIDTH, desc->dst_width);
		sstar_bdma_writel(bc, SSTAR_BDMA_DST_PITCH, desc->dst_pitch);
		sstar_bdma_writew(bc, SSTAR_BDMA_LINE_EN, 1);
	} else {
		sstar_bdma_writew(bc, SSTAR_BDMA_LINE_EN, 0);
	}

	sstar_bdma_writew(bc, SSTAR_BDMA_MISC, SSTAR_BDMA_MISC_INT_EN);
	sstar_bdma_writew(bc, SSTAR_BDMA_CTRL, SSTAR_BDMA_CTRL_TRIG);
}

/* bc->vc.lock must be held */
static int sstar_bdma_stop(struct sstar_bdma_chan *bc)
{
	u16 status;
	int ret;

	sstar_bdma_writew(bc, SSTAR_BDMA_MISC, 0);
	sstar_bdma_writew(bc, SSTAR_BDMA_CTRL, SSTAR_BDMA_CTRL_STOP);
	ret = readw_poll_timeout_atomic(bc->base + SSTAR_BDMA_STATUS, status,
					!(status & SSTAR_BDMA_STATUS_BUSY),
					1, SSTAR_BDMA_STOP_TIMEOUT_US);
	sstar_bdma_writew(bc, SSTAR_BDMA_CTRL, 0);
	sstar_bdma_writew(bc, SSTAR_BDMA_STATUS, SSTAR_BDMA_STATUS_CLEAR);

	return ret;
}

static irqreturn_t sstar_bdma_irq(int irq, void *data)
{
	struct sstar_bdma_chan *bc = data;
	u16 status;

	spin_lock(&bc->vc.lock);

	status = readw(bc->base + SSTAR_BDMA_STATUS);
	if (!(status & SSTAR_BDMA_STATUS_INT)) {
		spin_unlock(&bc->vc.lock);
		return IRQ_NONE;
	}

	sstar_bdma_writew(bc, SSTAR_BDMA_STATUS, SSTAR_BDMA_STATUS_CLEAR);

	/* The interrupt of a stopped transfer finds no descriptor */
	if (bc->desc) {
		vchan_cookie_complete(&bc->desc->vd);
		sstar_bdma_start(bc);
	}

	spin_unlock(&bc->vc.lock);

	return IRQ_HANDLED;
}

static struct dma_async_tx_descriptor *
sstar_bdma_prep(struct dma_chan *chan, struct sstar_bdma_desc *desc,
		unsigned long flags)
{
	return vchan_tx_prep(&to_sstar_bdma_chan(chan)->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *
sstar_bdma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		       size_t len, unsigned long flags)
{
	struct sstar_bdma_desc *desc;

	if (!len || len > SSTAR_BDMA_MAX_LEN)
		return NULL;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->config = FIELD_PREP(SSTAR_BDMA_CONFIG_SRC, SSTAR_BDMA_SEL_MEM0) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_SRC_DW, SSTAR_BDMA_DW_16) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_DST, SSTAR_BDMA_SEL_MEM1) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_DST_DW, SSTAR_BDMA_DW_16);
	desc->src = src;
	desc->dst = dst;
	desc->len = len;

	return sstar_bdma_prep(chan, desc, flags);
}

static struct dma_async_tx_descriptor *
sstar_bdma_prep_memset(struct dma_chan *chan, dma_addr_t dst, int value,
		       size_t len, unsigned long flags)
{
	struct sstar_bdma_desc *desc;

	if (!len || len > SSTAR_BDMA_MAX_LEN)
		return NULL;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;

	/* The fill engine repeats a 32-bit pattern, replicate the byte */
	desc->config = FIELD_PREP(SSTAR_BDMA_CONFIG_SRC, SSTAR_BDMA_SEL_FILL) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_SRC_DW, SSTAR_BDMA_DW_4) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_DST, SSTAR_BDMA_SEL_MEM1) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_DST_DW, SSTAR_BDMA_DW_16);
	desc->dst = dst;
	desc->len = len;
	desc->pattern = (u8)value * 0x01010101U;

	return sstar_bdma_prep(chan, desc, flags);
}

/*
 * The line mode copies lines of a fixed width, the source and destination
 * have their own width and pitch. Accept a single chunk per frame copied
 * between two memory buffers.
 */
static struct dma_async_tx_descriptor *
sstar_bdma_prep_interleaved(struct dma_chan *chan,
			    struct dma_interleaved_template *xt,
			    unsigned long flags)
{
	struct sstar_bdma_desc *desc;
	size_t width, len;

	if (xt->dir != DMA_MEM_TO_MEM || xt->frame_size != 1 || !xt->numf ||
	    !xt->src_inc || !xt->dst_inc)
		return NULL;

	width = xt->sgl[0].size;
	if (!width || check_mul_overflow(width, xt->numf, &len) ||
	    len > SSTAR_BDMA_MAX_LEN)
		return NULL;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->config = FIELD_PREP(SSTAR_BDMA_CONFIG_SRC, SSTAR_BDMA_SEL_MEM0) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_SRC_DW, SSTAR_BDMA_DW_16) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_DST, SSTAR_BDMA_SEL_MEM1) |
		       FIELD_PREP(SSTAR_BDMA_CONFIG_DST_DW, SSTAR_BDMA_DW_16);
	desc->src = xt->src_start;
	desc->dst = xt->dst_start;
	desc->len = len;
	desc->src_width = width;
	desc->src_pitch = width + dmaengine_get_src_icg(xt, &xt->sgl[0]);
	desc->dst_width = width;
	desc->dst_pitch = width + dmaengine_get_dst_icg(xt, &xt->sgl[0]);

	return sstar_bdma_prep(chan, desc, flags);
}

static int sstar_bdma_width(enum dma_slave_buswidth width)
{
	if (!(SSTAR_BDMA_SLAVE_WIDTHS & BIT(width)))
		return -EINVAL;

	return ilog2(width);
}

/*
 * The controller moves one block per transfer, the device side starts at
 * the address from the slave configuration.
 */
static struct dma_async_tx_descriptor *
sstar_bdma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
			 unsigned int sg_len, enum dma_transfer_direction dir,
			 unsigned long flags, void *context)
{
	struct sstar_bdma_chan *bc = to_sstar_bdma_chan(chan);
	struct dma_slave_config *cfg = &bc->slave_cfg;
	struct sstar_bdma_desc *desc;
	int width;

	if (sg_len != 1 || !sg_dma_len(sgl))
		return NULL;

	if (dir == DMA_DEV_TO_MEM)
		width = sstar_bdma_width(cfg->src_addr_width);
	else if (dir == DMA_MEM_TO_DEV)
		width = sstar_bdma_width(cfg->dst_addr_width);
	else
		return NULL;

	if (width < 0)
		return NULL;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;

	if (dir == DMA_DEV_TO_MEM) {
		desc->config = FIELD_PREP(SSTAR_BDMA_CONFIG_SRC, bc->slave_sel) |
			       FIELD_PREP(SSTAR_BDMA_CONFIG_SRC_DW, width) |
			       FIELD_PREP(SSTAR_BDMA_CONFIG_DST, SSTAR_BDMA_SEL_MEM0) |
			       FIELD_PREP(SSTAR_BDMA_CONFIG_DST_DW, SSTAR_BDMA_DW_16);
		desc->src = cfg->src_addr;
		desc->dst = sg_dma_address(sgl);
	} else {
		desc->config = FIELD_PREP(SSTAR_BDMA_CONFIG_SRC, SSTAR_BDMA_SEL_MEM0) |
			       FIELD_PREP(SSTAR_BDMA_CONFIG_SRC_DW, SSTAR_BDMA_DW_16) |
			       FIELD_PREP(SSTAR_BDMA_CONFIG_DST, bc->slave_sel) |
			       FIELD_PREP(SSTAR_BDMA_CONFIG_DST_DW, width);
		desc->src = sg_dma_address(sgl);
		desc->dst = cfg->dst_addr;
	}
	desc->len = sg_dma_len(sgl);

	return sstar_bdma_prep(chan, desc, flags);
}

static int sstar_bdma_slave_config(struct dma_chan *chan,
				   struct dma_slave_config *cfg)
{
	struct sstar_bdma_chan *bc = to_sstar_bdma_chan(chan);

	bc->slave_cfg = *cfg;

	return 0;
}

static void sstar_bdma_issue_pending(struct dma_chan *chan)
{
	struct sstar_bdma_chan *bc = to_sstar_bdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&bc->vc.lock, flags);
	if (vchan_issue_pending(&bc->vc) && !bc->desc)
		sstar_bdma_start(bc);
	spin_unlock_irqrestore(&bc->vc.lock, flags);
}

static int sstar_bdma_terminate_all(struct dma_chan *chan)
{
	struct sstar_bdma_chan *bc = to_sstar_bdma_chan(chan);
	unsigned long flags;
	int ret = 0;
	LIST_HEAD(head);

	spin_lock_irqsave(&bc->vc.lock, flags);

	if (bc->desc) {
		vchan_terminate_vdesc(&bc->desc->vd);
		bc->desc = NULL;
		ret = sstar_bdma_stop(bc);
	}
	vchan_get_all_descriptors(&bc->vc, &head);

	spin_unlock_irqrestore(&bc->vc.lock, flags);

	vchan_dma_desc_free_list(&bc->vc, &head);

	return ret;
}

static void sstar_bdma_synchronize(struct dma_chan *chan)
{
	vchan_synchronize(&to_sstar_bdma_chan(chan)->vc);
}

static void sstar_bdma_free_chan_resources(struct dma_chan *chan)
{
	vchan_free_chan_resources(&to_sstar_bdma_chan(chan)->vc);
}

static void sstar_bdma_desc_free(struct virt_dma_desc *vd)
{
	kfree(to_sstar_bdma_desc(vd));
}

/* The specifier holds the channel number and the peripheral selection */
static struct dma_chan *sstar_bdma_of_xlate(struct of_phandle_args *spec,
					    struct of_dma *ofdma)
{
	struct sstar_bdma *bdma = ofdma->of_dma_data;
	struct sstar_bdma_chan *bc;
	struct dma_chan *chan;

	if (spec->args_count != 2 || spec->args[0] >= SSTAR_BDMA_NR_CHANS ||
	    spec->args[1] > SSTAR_BDMA_SEL_MAX)
		return NULL;

	bc = &bdma->chans[spec->args[0]];
	chan = dma_get_slave_channel(&bc->vc.chan);
	if (chan)
		bc->slave_sel = spec->args[1];

	return chan;
}

static void sstar_bdma_of_free(void *data)
{
	of_dma_controller_free(data);
}

static int sstar_bdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dma_device *ddev;
	struct sstar_bdma *bdma;
	void __iomem *base;
	struct clk *clk;
	unsigned int i;
	int ret;

	bdma = devm_kzalloc_obj(dev, *bdma);
	if (!bdma)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ddev = &bdma->ddev;
	ddev->dev = dev;
	INIT_LIST_HEAD(&ddev->channels);

	for (i = 0; i < SSTAR_BDMA_NR_CHANS; i++) {
		struct sstar_bdma_chan *bc = &bdma->chans[i];
		int irq;

		bc->base = base + i * SSTAR_BDMA_CHAN_STRIDE;
		bc->vc.desc_free = sstar_bdma_desc_free;
		vchan_init(&bc->vc, ddev);

		/* Stop what the bootloader may have left running */
		sstar_bdma_writew(bc, SSTAR_BDMA_MISC, 0);
		sstar_bdma_writew(bc, SSTAR_BDMA_STATUS,
				  SSTAR_BDMA_STATUS_CLEAR);

		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;

		ret = devm_request_irq(dev, irq, sstar_bdma_irq, 0,
				       dev_name(dev), bc);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request IRQ %d\n", irq);
	}

	dma_cap_set(DMA_MEMCPY, ddev->cap_mask);
	dma_cap_set(DMA_MEMSET, ddev->cap_mask);
	dma_cap_set(DMA_INTERLEAVE, ddev->cap_mask);
	dma_cap_set(DMA_SLAVE, ddev->cap_mask);
	ddev->directions = BIT(DMA_MEM_TO_MEM) | BIT(DMA_DEV_TO_MEM) |
			   BIT(DMA_MEM_TO_DEV);
	ddev->src_addr_widths = SSTAR_BDMA_SLAVE_WIDTHS;
	ddev->dst_addr_widths = SSTAR_BDMA_SLAVE_WIDTHS;
	ddev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	ddev->device_prep_dma_memcpy = sstar_bdma_prep_memcpy;
	ddev->device_prep_dma_memset = sstar_bdma_prep_memset;
	ddev->device_prep_interleaved_dma = sstar_bdma_prep_interleaved;
	ddev->device_prep_slave_sg = sstar_bdma_prep_slave_sg;
	ddev->device_config = sstar_bdma_slave_config;
	ddev->device_issue_pending = sstar_bdma_issue_pending;
	ddev->device_tx_status = dma_cookie_status;
	ddev->device_terminate_all = sstar_bdma_terminate_all;
	ddev->device_synchronize = sstar_bdma_synchronize;
	ddev->device_free_chan_resources = sstar_bdma_free_chan_resources;

	ret = dmaenginem_async_device_register(ddev);
	if (ret)
		return ret;

	ret = of_dma_controller_register(dev->of_node, sstar_bdma_of_xlate, bdma);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, sstar_bdma_of_free, dev->of_node);
}

static const struct of_device_id sstar_bdma_of_match[] = {
	{ .compatible = "sstar,ssc338q-bdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_bdma_of_match);

static struct platform_driver sstar_bdma_driver = {
	.driver = {
		.name = "sstar-bdma",
		.of_match_table = sstar_bdma_of_match,
	},
	.probe = sstar_bdma_probe,
};
module_platform_driver(sstar_bdma_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q BDMA controller driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
