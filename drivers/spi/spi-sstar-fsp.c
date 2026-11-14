// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q FSP SPI flash controller
 *
 * The FSP (flash serial peripheral) shifts out up to 10 bytes from its
 * write buffer and then shifts in up to 10 bytes into its read buffer, all
 * in one chip select cycle. Larger SPI NAND page cache transfers are split
 * into several such operations.
 *
 * The QSPI block next to it reads the flash on behalf of the BDMA engine:
 * it sends the read command set up in its registers with the address the
 * BDMA asks for. When a BDMA channel is available, page cache reads use
 * this path through a direct mapping.
 *
 * With a second BDMA channel, page cache loads are sent in one operation,
 * the FSP takes the data from the BDMA instead of its write buffer.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/* 16-bit registers on the RIU, 4-byte stride */
#define FSP_WDATA(byte)		(0x60 + (byte) / 2)
#define FSP_RDATA(byte)		(0x65 + (byte) / 2)
#define FSP_WSIZE		0x6a
#define FSP_RSIZE		0x6b
#define FSP_CTRL		0x6c
#define  FSP_CTRL_EN		GENMASK(2, 0)
#define FSP_TRIGGER		0x6d
#define  FSP_TRIGGER_GO		BIT(0)
#define FSP_DONE		0x6e
#define  FSP_DONE_FLAG		BIT(0)
#define FSP_CLEAR		0x6f
#define  FSP_CLEAR_DONE		BIT(0)
#define FSP_CTRL2		0x75
#define FSP_CTRL3		0x76
/*
 * The write buffer can take its data from the BDMA: the bytes from the
 * outside replace the buffer starting at the given index.
 */
#define FSP_OUTSIDE_SIZE	0x78
#define FSP_OUTSIDE		0x79
#define  FSP_OUTSIDE_EN		BIT(12)
#define  FSP_OUTSIDE_INDEX	GENMASK(3, 0)
#define FSP_OUTSIDE_MAX		0xfff

/* QSPI block following the FSP registers */
#define QSPI_OFFSET		0x200
#define QSPI_DUMMY		0x70
#define  QSPI_DUMMY_USER	BIT(4)
#define  QSPI_DUMMY_CYCLES	GENMASK(3, 0)
#define QSPI_MODE		0x72
#define QSPI_CTRL		0x7a
#define QSPI_FUNC		0x7d
#define  QSPI_FUNC_ADDR2	BIT(11)
#define  QSPI_FUNC_DUMMY	BIT(12)

/* QSPI read modes and the read command each of them sends */
static const struct {
	u8 opcode;
	u8 mode;
	u8 addr_buswidth;
	u8 data_buswidth;
} sstar_fsp_read_modes[] = {
	{ 0x03, 0x00, 1, 1 },
	{ 0x0b, 0x01, 1, 1 },
	{ 0x3b, 0x02, 1, 2 },
	{ 0xbb, 0x03, 2, 2 },
	{ 0x6b, 0x0a, 1, 4 },
	{ 0xeb, 0x0b, 4, 4 },
};

#define FSP_BUF_SIZE		10
#define FSP_TIMEOUT_US		100000

/* Bounce buffer for the BDMA, holds a 4 KiB page with its spare area */
#define FSP_DMA_BUF_SIZE	SZ_8K
#define FSP_DMA_TIMEOUT_MS	1000

/* SPI NAND page cache load opcodes */
#define SPINAND_PROGRAM_LOAD		0x02
#define SPINAND_PROGRAM_LOAD_RANDOM	0x84

struct sstar_fsp {
	void __iomem *base;
	struct device *dev;
	struct dma_chan *rx;
	struct dma_chan *tx;
	void *dma_buf;
	dma_addr_t dma_addr;
};

static void fsp_writew(struct sstar_fsp *fsp, u16 val, unsigned int reg)
{
	writew(val, fsp->base + reg * 4);
}

static u16 fsp_readw(struct sstar_fsp *fsp, unsigned int reg)
{
	return readw(fsp->base + reg * 4);
}

static void qspi_writew(struct sstar_fsp *fsp, u16 val, unsigned int reg)
{
	writew(val, fsp->base + QSPI_OFFSET + reg * 4);
}

static int sstar_fsp_read_mode(const struct spi_mem_op *op)
{
	unsigned int i;

	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1 ||
	    op->addr.nbytes != 2 || op->data.dir != SPI_MEM_DATA_IN)
		return -EINVAL;

	if (op->dummy.nbytes &&
	    op->dummy.nbytes * 8 / op->dummy.buswidth > QSPI_DUMMY_CYCLES + 1)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(sstar_fsp_read_modes); i++)
		if (sstar_fsp_read_modes[i].opcode == op->cmd.opcode &&
		    sstar_fsp_read_modes[i].addr_buswidth == op->addr.buswidth &&
		    sstar_fsp_read_modes[i].data_buswidth == op->data.buswidth)
			return sstar_fsp_read_modes[i].mode;

	return -EINVAL;
}

static void sstar_fsp_dma_done(void *data)
{
	complete(data);
}

static int sstar_fsp_wait_dma(struct sstar_fsp *fsp, struct dma_chan *chan,
			      struct completion *done)
{
	if (!wait_for_completion_timeout(done,
					 msecs_to_jiffies(FSP_DMA_TIMEOUT_MS))) {
		dmaengine_terminate_sync(chan);
		return -ETIMEDOUT;
	}

	return 0;
}

/* Read through the QSPI read path, len is at most FSP_DMA_BUF_SIZE */
static int sstar_fsp_qspi_read(struct sstar_fsp *fsp,
			       const struct spi_mem_op *op, u32 addr,
			       size_t len, void *buf)
{
	struct dma_slave_config cfg = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr = addr,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES,
	};
	struct dma_async_tx_descriptor *tx;
	DECLARE_COMPLETION_ONSTACK(done);
	unsigned int cycles = 0;
	u16 func = QSPI_FUNC_ADDR2;
	int ret;

	if (op->dummy.nbytes) {
		cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;
		func |= QSPI_FUNC_DUMMY;
	}

	qspi_writew(fsp, QSPI_DUMMY_USER |
			 FIELD_PREP(QSPI_DUMMY_CYCLES, cycles ? cycles - 1 : 0),
		    QSPI_DUMMY);
	qspi_writew(fsp, sstar_fsp_read_mode(op), QSPI_MODE);
	qspi_writew(fsp, func, QSPI_FUNC);

	ret = dmaengine_slave_config(fsp->rx, &cfg);
	if (ret)
		return ret;

	tx = dmaengine_prep_slave_single(fsp->rx, fsp->dma_addr, len,
					 DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!tx)
		return -EIO;

	tx->callback = sstar_fsp_dma_done;
	tx->callback_param = &done;
	dmaengine_submit(tx);
	dma_async_issue_pending(fsp->rx);

	ret = sstar_fsp_wait_dma(fsp, fsp->rx, &done);
	if (ret)
		return ret;

	memcpy(buf, fsp->dma_buf, len);

	return 0;
}

static bool sstar_fsp_use_qspi(struct sstar_fsp *fsp,
			       const struct spi_mem_op *op)
{
	return fsp->rx && sstar_fsp_read_mode(op) >= 0;
}

static int sstar_fsp_xfer(struct sstar_fsp *fsp, const u8 *tx,
			  unsigned int tx_len, u8 *rx, unsigned int rx_len)
{
	unsigned int i;
	u16 done;
	int ret;

	fsp_writew(fsp, FSP_CTRL_EN, FSP_CTRL);
	fsp_writew(fsp, 0, FSP_CTRL2);
	fsp_writew(fsp, 0, FSP_CTRL3);

	for (i = 0; i < tx_len; i += 2)
		fsp_writew(fsp, tx[i] | (i + 1 < tx_len ? tx[i + 1] << 8 : 0),
			   FSP_WDATA(i));

	fsp_writew(fsp, tx_len, FSP_WSIZE);
	fsp_writew(fsp, rx_len, FSP_RSIZE);
	fsp_writew(fsp, FSP_TRIGGER_GO, FSP_TRIGGER);

	ret = readw_poll_timeout(fsp->base + FSP_DONE * 4, done,
				 done & FSP_DONE_FLAG, 1, FSP_TIMEOUT_US);
	fsp_writew(fsp, FSP_CLEAR_DONE, FSP_CLEAR);
	if (ret)
		return ret;

	for (i = 0; i < rx_len; i++)
		rx[i] = fsp_readw(fsp, FSP_RDATA(i)) >> (i & 1 ? 8 : 0);

	return 0;
}

static unsigned int sstar_fsp_header(const struct spi_mem_op *op, u32 addr,
				     u16 opcode, u8 *buf)
{
	unsigned int i, len = 0;

	buf[len++] = opcode;
	for (i = op->addr.nbytes; i; i--)
		buf[len++] = addr >> ((i - 1) * 8);
	for (i = 0; i < op->dummy.nbytes; i++)
		buf[len++] = 0;

	return len;
}

static bool sstar_fsp_is_program_load(const struct spi_mem_op *op)
{
	return op->data.dir == SPI_MEM_DATA_OUT && op->addr.nbytes &&
	       (op->cmd.opcode == SPINAND_PROGRAM_LOAD ||
		op->cmd.opcode == SPINAND_PROGRAM_LOAD_RANDOM);
}

static unsigned int sstar_fsp_header_len(const struct spi_mem_op *op)
{
	return op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
}

static bool sstar_fsp_use_dma_load(struct sstar_fsp *fsp,
				   const struct spi_mem_op *op)
{
	return fsp->tx && sstar_fsp_is_program_load(op) &&
	       op->data.nbytes <= FSP_OUTSIDE_MAX &&
	       op->cmd.buswidth == 1 && op->addr.buswidth == 1 &&
	       op->data.buswidth == 1;
}

/* Load the page cache in one operation with the data from the BDMA */
static int sstar_fsp_dma_program_load(struct sstar_fsp *fsp,
				      const struct spi_mem_op *op)
{
	struct dma_slave_config cfg = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
	};
	unsigned int len = op->data.nbytes;
	struct dma_async_tx_descriptor *tx;
	DECLARE_COMPLETION_ONSTACK(done);
	u8 buf[FSP_BUF_SIZE];
	unsigned int hdr, i;
	u16 status;
	int ret;

	ret = dmaengine_slave_config(fsp->tx, &cfg);
	if (ret)
		return ret;

	memcpy(fsp->dma_buf, op->data.buf.out, len);

	tx = dmaengine_prep_slave_single(fsp->tx, fsp->dma_addr, len,
					 DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!tx)
		return -EIO;

	tx->callback = sstar_fsp_dma_done;
	tx->callback_param = &done;
	dmaengine_submit(tx);
	dma_async_issue_pending(fsp->tx);

	hdr = sstar_fsp_header(op, op->addr.val, op->cmd.opcode, buf);

	fsp_writew(fsp, FSP_CTRL_EN, FSP_CTRL);
	fsp_writew(fsp, 0, FSP_CTRL2);
	fsp_writew(fsp, 0, FSP_CTRL3);
	for (i = 0; i < hdr; i += 2)
		fsp_writew(fsp, buf[i] | (i + 1 < hdr ? buf[i + 1] << 8 : 0),
			   FSP_WDATA(i));
	fsp_writew(fsp, hdr, FSP_WSIZE);
	fsp_writew(fsp, 0, FSP_RSIZE);
	/* Size and index as programmed by the vendor driver */
	fsp_writew(fsp, len + 1, FSP_OUTSIDE_SIZE);
	fsp_writew(fsp, FSP_OUTSIDE_EN | FIELD_PREP(FSP_OUTSIDE_INDEX, hdr),
		   FSP_OUTSIDE);
	fsp_writew(fsp, FSP_TRIGGER_GO, FSP_TRIGGER);

	ret = sstar_fsp_wait_dma(fsp, fsp->tx, &done);
	if (!ret)
		ret = readw_poll_timeout(fsp->base + FSP_DONE * 4, status,
					 status & FSP_DONE_FLAG, 1,
					 FSP_TIMEOUT_US);

	fsp_writew(fsp, 0, FSP_OUTSIDE_SIZE);
	fsp_writew(fsp, 0, FSP_OUTSIDE);
	fsp_writew(fsp, FSP_CLEAR_DONE, FSP_CLEAR);

	return ret;
}

static bool sstar_fsp_supports_op(struct spi_mem *mem,
				  const struct spi_mem_op *op)
{
	struct sstar_fsp *fsp = spi_controller_get_devdata(mem->spi->controller);

	if (!spi_mem_default_supports_op(mem, op))
		return false;

	/* Only the QSPI read path drives more than one data line */
	if (op->addr.buswidth > 1 || op->dummy.buswidth > 1 ||
	    op->data.buswidth > 1)
		return sstar_fsp_use_qspi(fsp, op);

	if (op->cmd.nbytes != 1 || op->addr.nbytes > 3 ||
	    sstar_fsp_header_len(op) > FSP_BUF_SIZE)
		return false;

	/* Page cache loads are split into several operations */
	if (sstar_fsp_is_program_load(op))
		return sstar_fsp_header_len(op) < FSP_BUF_SIZE;

	if (op->data.dir == SPI_MEM_DATA_OUT)
		return sstar_fsp_header_len(op) + op->data.nbytes <=
		       FSP_BUF_SIZE;

	return true;
}

static int sstar_fsp_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	struct sstar_fsp *fsp = spi_controller_get_devdata(mem->spi->controller);

	if (op->data.dir == SPI_MEM_DATA_IN)
		op->data.nbytes = min_t(unsigned int, op->data.nbytes,
					sstar_fsp_use_qspi(fsp, op) ?
					FSP_DMA_BUF_SIZE : FSP_BUF_SIZE);

	return 0;
}

/*
 * Load the page cache in pieces: the first piece with the requested opcode,
 * which may reset the cache, the following ones with the random load that
 * keeps the bytes already loaded.
 */
static int sstar_fsp_program_load(struct sstar_fsp *fsp,
				  const struct spi_mem_op *op)
{
	const u8 *data = op->data.buf.out;
	u16 opcode = op->cmd.opcode;
	unsigned int off = 0;
	u8 buf[FSP_BUF_SIZE];

	do {
		unsigned int len = sstar_fsp_header(op, op->addr.val + off,
						    opcode, buf);
		unsigned int count = min_t(unsigned int, op->data.nbytes - off,
					   FSP_BUF_SIZE - len);
		int ret;

		memcpy(buf + len, data + off, count);
		ret = sstar_fsp_xfer(fsp, buf, len + count, NULL, 0);
		if (ret)
			return ret;

		off += count;
		opcode = SPINAND_PROGRAM_LOAD_RANDOM;
	} while (off < op->data.nbytes);

	return 0;
}

static int sstar_fsp_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct sstar_fsp *fsp = spi_controller_get_devdata(mem->spi->controller);
	u8 buf[FSP_BUF_SIZE];
	unsigned int len;

	if (sstar_fsp_use_dma_load(fsp, op))
		return sstar_fsp_dma_program_load(fsp, op);

	if (sstar_fsp_is_program_load(op))
		return sstar_fsp_program_load(fsp, op);

	if (sstar_fsp_use_qspi(fsp, op))
		return sstar_fsp_qspi_read(fsp, op, op->addr.val,
					   op->data.nbytes, op->data.buf.in);

	len = sstar_fsp_header(op, op->addr.val, op->cmd.opcode, buf);

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		memcpy(buf + len, op->data.buf.out, op->data.nbytes);
		len += op->data.nbytes;
	}

	if (op->data.dir == SPI_MEM_DATA_IN)
		return sstar_fsp_xfer(fsp, buf, len, op->data.buf.in,
				      op->data.nbytes);

	return sstar_fsp_xfer(fsp, buf, len, NULL, 0);
}

static int sstar_fsp_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
	struct sstar_fsp *fsp =
		spi_controller_get_devdata(desc->mem->spi->controller);

	struct spi_mem_dirmap_info *info = &desc->info;

	if (!fsp->rx || sstar_fsp_read_mode(&info->primary_op_tmpl) < 0)
		return -EOPNOTSUPP;

	if (info->secondary_op_tmpl.cmd.nbytes &&
	    sstar_fsp_read_mode(&info->secondary_op_tmpl) < 0)
		return -EOPNOTSUPP;

	return 0;
}

static ssize_t sstar_fsp_dirmap_read(struct spi_mem_dirmap_desc *desc,
				     u64 offs, size_t len, void *buf)
{
	struct sstar_fsp *fsp =
		spi_controller_get_devdata(desc->mem->spi->controller);
	int ret;

	len = min_t(size_t, len, FSP_DMA_BUF_SIZE);
	ret = sstar_fsp_qspi_read(fsp, desc->info.op_tmpl,
				  desc->info.offset + offs, len, buf);

	return ret ? ret : len;
}

static const struct spi_controller_mem_ops sstar_fsp_mem_ops = {
	.adjust_op_size	= sstar_fsp_adjust_op_size,
	.supports_op	= sstar_fsp_supports_op,
	.exec_op	= sstar_fsp_exec_op,
	.dirmap_create	= sstar_fsp_dirmap_create,
	.dirmap_read	= sstar_fsp_dirmap_read,
};

static void sstar_fsp_release_dma(void *data)
{
	dma_release_channel(data);
}

/* Returns NULL when the channel is not described */
static struct dma_chan *sstar_fsp_request_dma(struct device *dev,
					      const char *name)
{
	struct dma_chan *chan;
	int ret;

	chan = dma_request_chan(dev, name);
	if (IS_ERR(chan)) {
		if (PTR_ERR(chan) == -EPROBE_DEFER)
			return chan;
		return NULL;
	}

	ret = devm_add_action_or_reset(dev, sstar_fsp_release_dma, chan);

	return ret ? ERR_PTR(ret) : chan;
}

/* The FSP alone handles all operations without BDMA channels */
static int sstar_fsp_init_dma(struct sstar_fsp *fsp)
{
	struct device *dev = fsp->dev;
	struct dma_chan *rx, *tx;

	rx = sstar_fsp_request_dma(dev, "rx");
	if (IS_ERR(rx))
		return PTR_ERR(rx);

	tx = sstar_fsp_request_dma(dev, "tx");
	if (IS_ERR(tx))
		return PTR_ERR(tx);

	if (!rx && !tx)
		return 0;

	fsp->dma_buf = dmam_alloc_coherent(dev, FSP_DMA_BUF_SIZE,
					   &fsp->dma_addr, GFP_KERNEL);
	if (!fsp->dma_buf)
		return -ENOMEM;

	fsp->rx = rx;
	fsp->tx = tx;

	return 0;
}

static int sstar_fsp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct sstar_fsp *fsp;
	struct clk *clk;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*fsp));
	if (!ctlr)
		return -ENOMEM;

	fsp = spi_controller_get_devdata(ctlr);
	fsp->dev = dev;

	fsp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fsp->base))
		return PTR_ERR(fsp->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	/* Value from the vendor driver, the meaning is unknown */
	qspi_writew(fsp, 0, QSPI_CTRL);

	ret = sstar_fsp_init_dma(fsp);
	if (ret)
		return ret;

	ctlr->dev.of_node = dev->of_node;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 1;
	ctlr->mem_ops = &sstar_fsp_mem_ops;
	if (fsp->rx)
		ctlr->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD;

	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id sstar_fsp_of_match[] = {
	{ .compatible = "sstar,ssc338q-fsp" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_fsp_of_match);

static struct platform_driver sstar_fsp_driver = {
	.driver = {
		.name = "sstar-fsp",
		.of_match_table = sstar_fsp_of_match,
	},
	.probe = sstar_fsp_probe,
};
module_platform_driver(sstar_fsp_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q FSP SPI flash controller driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
