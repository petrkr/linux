// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q (Infinity6E) clock generator
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/sstar,ssc338q-clkgen.h>

/*
 * Each clock is controlled by a 16-bit register on the RIU (4-byte stride),
 * some registers hold two clocks, one per byte. A bit of the clock's field
 * (bit 0 unless stated otherwise) gates the clock when set and the bits
 * starting at bit 2 select the parent.
 */
#define SSC338Q_CLKGEN_MUX_SHIFT	2

struct ssc338q_clkgen_clk {
	unsigned int id;
	const char *name;
	unsigned int offset;
	/* Position of the clock's field in the register */
	unsigned int shift;
	/* Position of the gate bit relative to the field, 0 when not set */
	unsigned int gate_bit;
	unsigned int mux_width;
	/* Register values of the parents, when not all mux inputs are used */
	const u32 *mux_table;
	const struct clk_parent_data *parents;
	unsigned int num_parents;
	unsigned long flags;
};

/* Fixed divider in front of a mux input */
struct ssc338q_clkgen_factor {
	const char *name;
	const char *parent;
	unsigned int div;
};

/* Bit set once at probe */
struct ssc338q_clkgen_setup {
	unsigned int offset;
	unsigned int bit;
};

struct ssc338q_clkgen_data {
	const struct ssc338q_clkgen_factor *factors;
	unsigned int num_factors;
	const struct ssc338q_clkgen_setup *setup;
	unsigned int num_setup;
	const struct ssc338q_clkgen_clk *clks;
	unsigned int num_clks;
};

static const struct clk_parent_data ssc338q_uart_parents[] = {
	{ .fw_name = "mpll-div5" },
	{ .fw_name = "mpll-div6" },
	{ .fw_name = "xtal-div2" },
};

static const struct clk_parent_data ssc338q_emac_ahb_parents[] = {
	{ .fw_name = "mpll-div6" },
	{ .fw_name = "mpll-div7" },
	{ .fw_name = "mpll-div10" },
};

static const struct clk_parent_data ssc338q_aesdma_parents[] = {
	{ .fw_name = "mpll-div10" },
	{ .fw_name = "mpll-div5" },
};

static const struct ssc338q_clkgen_factor ssc338q_clkgen_factors[] = {
	{ .name = "mpll_div20", .parent = "mpll-div10", .div = 2 },
	{ .name = "xtal_div80", .parent = "xtal-div2", .div = 40 },
	{ .name = "mpll_div8", .parent = "mpll-div4", .div = 2 },
	{ .name = "mpll_div12", .parent = "mpll-div3", .div = 4 },
	{ .name = "mpll_div14", .parent = "mpll-div7", .div = 2 },
	{ .name = "mpll_div16", .parent = "mpll-div4", .div = 4 },
	{ .name = "mpll_div24", .parent = "mpll-div3", .div = 8 },
	{ .name = "mpll_div32", .parent = "mpll-div4", .div = 8 },
	{ .name = "mpll_div40", .parent = "mpll-div10", .div = 4 },
	{ .name = "mpll_div160", .parent = "mpll-div10", .div = 16 },
	{ .name = "utmi_192m_div4", .parent = "utmi-192m", .div = 4 },
	{ .name = "utmi_160m_div4", .parent = "utmi-160m", .div = 4 },
	{ .name = "utmi_160m_div5", .parent = "utmi-160m", .div = 5 },
	{ .name = "utmi_160m_div8", .parent = "utmi-160m", .div = 8 },
};

/* All the inputs of the SD clock mux are provided */
static const struct clk_parent_data ssc338q_sd_parents[] = {
	{ .name = "utmi_192m_div4", .index = -1 },
	{ .name = "mpll_div20", .index = -1 },
	{ .name = "utmi_160m_div4", .index = -1 },
	{ .name = "mpll_div24", .index = -1 },
	{ .name = "utmi_160m_div5", .index = -1 },
	{ .name = "utmi_160m_div8", .index = -1 },
	{ .fw_name = "xtal-div2" },
	{ .name = "xtal_div80", .index = -1 },
};

/*
 * The other input of the SPI flash clock mux comes from the encoder PLL,
 * which has no driver.
 */
static const struct clk_parent_data ssc338q_spi_flash_parents[] = {
	{ .fw_name = "xtal-div2" },
	{ .name = "mpll_div32", .index = -1 },
	{ .name = "mpll_div24", .index = -1 },
	{ .name = "mpll_div16", .index = -1 },
	{ .name = "mpll_div12", .index = -1 },
	{ .fw_name = "mpll-div10" },
	{ .name = "mpll_div8", .index = -1 },
};

static const u32 ssc338q_spi_flash_mux_table[] = { 0, 1, 2, 4, 5, 6, 7 };

/* The other inputs of the PWM clock mux are dividers of the crystal */
static const struct clk_parent_data ssc338q_pwm_parents[] = {
	{ .fw_name = "xtal-div2" },
	{ .fw_name = "mpll-div10" },
};

static const u32 ssc338q_pwm_mux_table[] = { 0, 5 };

/* The I2C timing counters are set up for the 12 MHz input */
static const struct clk_parent_data ssc338q_miic_parents[] = {
	{ .fw_name = "xtal-div2" },
};

static const u32 ssc338q_miic_mux_table[] = { 2 };

/*
 * The other inputs of the sensor clock mux come from the LPLL and the ARM
 * PLL, which have no driver.
 */
static const struct clk_parent_data ssc338q_sr_mclk_parents[] = {
	{ .name = "mpll_div32", .index = -1 },
	{ .name = "mpll_div12", .index = -1 },
	{ .name = "mpll_div14", .index = -1 },
	{ .name = "mpll_div16", .index = -1 },
	{ .name = "utmi_192m_div4", .index = -1 },
	{ .name = "mpll_div20", .index = -1 },
	{ .name = "mpll_div24", .index = -1 },
	{ .fw_name = "xtal" },
	{ .name = "mpll_div40", .index = -1 },
	{ .fw_name = "xtal-div2" },
	{ .name = "mpll_div160", .index = -1 },
};

/* The last two inputs of the MIPI CSI clock muxes are not connected */
static const struct clk_parent_data ssc338q_csi_parents[] = {
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "utmi-240m" },
	{ .fw_name = "mpll-div5" },
	{ .fw_name = "mpll-div6" },
	{ .fw_name = "mpll-div10" },
	{ .name = "utmi_160m_div4", .index = -1 },
};

/* The other input comes from the LPLL, which has no driver */
static const struct clk_parent_data ssc338q_mipi_tx_csi_parents[] = {
	{ .fw_name = "utmi-160m" },
	{ .fw_name = "mpll-div6" },
	{ .name = "mpll_div8", .index = -1 },
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "utmi-240m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "upll-320m" },
};

static const u32 ssc338q_mipi_tx_csi_mux_table[] = { 1, 2, 3, 4, 5, 6, 7 };

/* The other input comes from the encoder PLL, which has no driver */
static const struct clk_parent_data ssc338q_vhe_parents[] = {
	{ .fw_name = "utmi-480m" },
	{ .fw_name = "mpll-div2" },
	{ .fw_name = "upll-384m" },
	{ .fw_name = "upll-320m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "utmi-192m" },
	{ .fw_name = "mpll-div7" },
};

static const u32 ssc338q_vhe_mux_table[] = { 1, 2, 3, 4, 5, 6, 7 };

static const struct clk_parent_data ssc338q_mfe_parents[] = {
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "upll-384m" },
	{ .fw_name = "upll-320m" },
};

static const u32 ssc338q_mfe_mux_table[] = { 0, 1, 2 };

/* The last two inputs of the JPEG encoder clock mux are not connected */
static const struct clk_parent_data ssc338q_jpe_parents[] = {
	{ .fw_name = "utmi-480m" },
	{ .fw_name = "mpll-div2" },
	{ .fw_name = "upll-384m" },
	{ .fw_name = "upll-320m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "mpll-div4" },
};

/* All the inputs of the ISP clock mux are provided */
static const struct clk_parent_data ssc338q_isp_parents[] = {
	{ .fw_name = "upll-320m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "utmi-192m" },
	{ .fw_name = "mpll-div5" },
	{ .fw_name = "mpll-div7" },
	{ .name = "mpll_div12", .index = -1 },
	{ .fw_name = "upll-384m" },
};

/* All the inputs of the FCLK1 clock mux are provided */
static const struct clk_parent_data ssc338q_fclk1_parents[] = {
	{ .fw_name = "mpll-345m" },
	{ .fw_name = "mpll-div2" },
	{ .fw_name = "upll-384m" },
	{ .fw_name = "upll-320m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "utmi-240m" },
	{ .fw_name = "mpll-div5" },
	{ .fw_name = "mpll-div7" },
};

/*
 * The other input of the ODCLK clock mux comes from the LPLL, which has no
 * driver.
 */
static const struct clk_parent_data ssc338q_odclk_parents[] = {
	{ .fw_name = "mpll-div10" },
	{ .name = "mpll_div20", .index = -1 },
	{ .name = "mpll_div40", .index = -1 },
};

/* The last input of the IVE clock mux is not connected */
static const struct clk_parent_data ssc338q_ive_parents[] = {
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "mpll-div5" },
	{ .fw_name = "mpll-div7" },
	{ .fw_name = "mpll-div10" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "upll-320m" },
	{ .fw_name = "upll-384m" },
};

/* All the inputs of the MCU clock mux are provided */
static const struct clk_parent_data ssc338q_mcu_parents[] = {
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "mpll-div2" },
	{ .fw_name = "upll-384m" },
	{ .name = "mpll_div8", .index = -1 },
};

/* The last input of the LDC clock mux is not connected */
static const struct clk_parent_data ssc338q_ldc_parents[] = {
	{ .fw_name = "utmi-480m" },
	{ .fw_name = "mpll-div2" },
	{ .fw_name = "upll-384m" },
	{ .fw_name = "upll-320m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "mpll-div4" },
	{ .fw_name = "mpll-div10" },
};

/* All the inputs of the DIP clock mux are provided */
static const struct clk_parent_data ssc338q_dip_parents[] = {
	{ .fw_name = "utmi-480m" },
	{ .fw_name = "mpll-div2" },
	{ .fw_name = "upll-384m" },
	{ .fw_name = "upll-320m" },
	{ .fw_name = "mpll-div3" },
	{ .fw_name = "utmi-240m" },
	{ .fw_name = "mpll-div5" },
	{ .fw_name = "mpll-div7" },
};

/* The other inputs of the audio clock mux are not provided */
static const struct clk_parent_data ssc338q_bach_parents[] = {
	{ .fw_name = "upll-384m" },
};

static const u32 ssc338q_bach_mux_table[] = { 0 };

/*
 * Bit 5 of the SD and SPI flash clock registers switches between the 12 MHz
 * boot clock and the clock selected by the mux, use the mux.
 */
static const struct ssc338q_clkgen_setup ssc338q_clkgen_setup[] = {
	{ .offset = 0x0e0, .bit = 5 },
	{ .offset = 0x10c, .bit = 5 },
};

static const struct ssc338q_clkgen_clk ssc338q_clkgen_clks[] = {
	{
		.id = SSC338Q_CLKGEN_UART0,
		.name = "uart0",
		.offset = 0xc4,
		.mux_width = 2,
		.parents = ssc338q_uart_parents,
		.num_parents = ARRAY_SIZE(ssc338q_uart_parents),
	},
	{
		.id = SSC338Q_CLKGEN_UART1,
		.name = "uart1",
		.offset = 0xc4,
		.shift = 8,
		.mux_width = 2,
		.parents = ssc338q_uart_parents,
		.num_parents = ARRAY_SIZE(ssc338q_uart_parents),
	},
	{
		/*
		 * The synthesizer input is not provided. The register also
		 * holds the synthesizer clock and its enable and reset.
		 */
		.id = SSC338Q_CLKGEN_FUART,
		.name = "fuart",
		.offset = 0x0d0,
		.mux_width = 2,
		.parents = ssc338q_uart_parents,
		.num_parents = ARRAY_SIZE(ssc338q_uart_parents),
	},
	{
		.id = SSC338Q_CLKGEN_EMAC_AHB,
		.name = "emac_ahb",
		.offset = 0x108,
		.mux_width = 2,
		.parents = ssc338q_emac_ahb_parents,
		.num_parents = ARRAY_SIZE(ssc338q_emac_ahb_parents),
	},
	{
		.id = SSC338Q_CLKGEN_AESDMA,
		.name = "aesdma",
		.offset = 0x184,
		.mux_width = 2,
		.parents = ssc338q_aesdma_parents,
		.num_parents = ARRAY_SIZE(ssc338q_aesdma_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SD,
		.name = "sd",
		.offset = 0x10c,
		.mux_width = 3,
		.parents = ssc338q_sd_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sd_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SPI_FLASH,
		.name = "spi_flash",
		.offset = 0x0e0,
		.mux_width = 3,
		.mux_table = ssc338q_spi_flash_mux_table,
		.parents = ssc338q_spi_flash_parents,
		.num_parents = ARRAY_SIZE(ssc338q_spi_flash_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MIIC0,
		.name = "miic0",
		.offset = 0x0dc,
		.mux_width = 2,
		.mux_table = ssc338q_miic_mux_table,
		.parents = ssc338q_miic_parents,
		.num_parents = ARRAY_SIZE(ssc338q_miic_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MIIC1,
		.name = "miic1",
		.offset = 0x0dc,
		.shift = 8,
		.mux_width = 2,
		.mux_table = ssc338q_miic_mux_table,
		.parents = ssc338q_miic_parents,
		.num_parents = ARRAY_SIZE(ssc338q_miic_parents),
	},
	{
		.id = SSC338Q_CLKGEN_PWM,
		.name = "pwm",
		.offset = 0x0e0,
		.shift = 8,
		.mux_width = 3,
		.mux_table = ssc338q_pwm_mux_table,
		.parents = ssc338q_pwm_parents,
		.num_parents = ARRAY_SIZE(ssc338q_pwm_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SR00_MCLK,
		.name = "sr00_mclk",
		.offset = 0x188,
		.shift = 8,
		.mux_width = 4,
		.parents = ssc338q_sr_mclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sr_mclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SR01_MCLK,
		.name = "sr01_mclk",
		.offset = 0x194,
		.mux_width = 4,
		.parents = ssc338q_sr_mclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sr_mclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_SR1_MCLK,
		.name = "sr1_mclk",
		.offset = 0x194,
		.shift = 8,
		.mux_width = 4,
		.parents = ssc338q_sr_mclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_sr_mclk_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_LPTX_M00,
		.name = "csi_lptx_m00",
		.offset = 0x160,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_MAC_M00,
		.name = "csi_mac_m00",
		.offset = 0x160,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_NS_M00,
		.name = "ns_m00",
		.offset = 0x164,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_LPTX_M01,
		.name = "csi_lptx_m01",
		.offset = 0x164,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_MAC_M01,
		.name = "csi_mac_m01",
		.offset = 0x168,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_NS_M01,
		.name = "ns_m01",
		.offset = 0x168,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_LPTX_M1,
		.name = "csi_lptx_m1",
		.offset = 0x16c,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_CSI_MAC_M1,
		.name = "csi_mac_m1",
		.offset = 0x16c,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_NS_M1,
		.name = "ns_m1",
		.offset = 0x170,
		.mux_width = 3,
		.parents = ssc338q_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MIPI1_TX_CSI,
		.name = "mipi1_tx_csi",
		.offset = 0x170,
		.shift = 8,
		.mux_width = 3,
		.mux_table = ssc338q_mipi_tx_csi_mux_table,
		.parents = ssc338q_mipi_tx_csi_parents,
		.num_parents = ARRAY_SIZE(ssc338q_mipi_tx_csi_parents),
	},
	{
		.id = SSC338Q_CLKGEN_VHE,
		.name = "vhe",
		.offset = 0x1a0,
		.gate_bit = 7,
		.mux_width = 3,
		.mux_table = ssc338q_vhe_mux_table,
		.parents = ssc338q_vhe_parents,
		.num_parents = ARRAY_SIZE(ssc338q_vhe_parents),
	},
	{
		.id = SSC338Q_CLKGEN_MFE,
		.name = "mfe",
		.offset = 0x1a4,
		.mux_width = 2,
		.mux_table = ssc338q_mfe_mux_table,
		.parents = ssc338q_mfe_parents,
		.num_parents = ARRAY_SIZE(ssc338q_mfe_parents),
	},
	{
		.id = SSC338Q_CLKGEN_JPE,
		.name = "jpe",
		.offset = 0x1a8,
		.mux_width = 3,
		.parents = ssc338q_jpe_parents,
		.num_parents = ARRAY_SIZE(ssc338q_jpe_parents),
	},
	{
		.id = SSC338Q_CLKGEN_ISP,
		.name = "isp",
		.offset = 0x184,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_isp_parents,
		.num_parents = ARRAY_SIZE(ssc338q_isp_parents),
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = SSC338Q_CLKGEN_FCLK1,
		.name = "fclk1",
		.offset = 0x190,
		.mux_width = 3,
		.parents = ssc338q_fclk1_parents,
		.num_parents = ARRAY_SIZE(ssc338q_fclk1_parents),
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = SSC338Q_CLKGEN_ODCLK,
		.name = "odclk",
		.offset = 0x198,
		.mux_width = 2,
		.parents = ssc338q_odclk_parents,
		.num_parents = ARRAY_SIZE(ssc338q_odclk_parents),
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = SSC338Q_CLKGEN_IVE,
		.name = "ive",
		.offset = 0x1a8,
		.shift = 8,
		.mux_width = 3,
		.parents = ssc338q_ive_parents,
		.num_parents = ARRAY_SIZE(ssc338q_ive_parents),
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		/* It also clocks blocks without a driver */
		.id = SSC338Q_CLKGEN_MCU,
		.name = "mcu",
		.offset = 0x004,
		.mux_width = 2,
		.parents = ssc338q_mcu_parents,
		.num_parents = ARRAY_SIZE(ssc338q_mcu_parents),
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = SSC338Q_CLKGEN_LDC,
		.name = "ldc",
		.offset = 0x14c,
		.mux_width = 3,
		.parents = ssc338q_ldc_parents,
		.num_parents = ARRAY_SIZE(ssc338q_ldc_parents),
	},
	{
		.id = SSC338Q_CLKGEN_DIP,
		.name = "dip",
		.offset = 0x148,
		.mux_width = 3,
		.parents = ssc338q_dip_parents,
		.num_parents = ARRAY_SIZE(ssc338q_dip_parents),
	},
	{
		.id = SSC338Q_CLKGEN_BACH,
		.name = "bach",
		.offset = 0x1b4,
		.gate_bit = 7,
		.mux_width = 2,
		.mux_table = ssc338q_bach_mux_table,
		.parents = ssc338q_bach_parents,
		.num_parents = ARRAY_SIZE(ssc338q_bach_parents),
	},
	/*
	 * The BDMA clock comes from the MIU clock, which has no driver. Its
	 * parent mux is left at its reset value, which selects the MIU clock.
	 */
	{
		.id = SSC338Q_CLKGEN_BDMA,
		.name = "bdma",
		.offset = 0x180,
	},
};

static const struct ssc338q_clkgen_data ssc338q_clkgen_data = {
	.factors = ssc338q_clkgen_factors,
	.num_factors = ARRAY_SIZE(ssc338q_clkgen_factors),
	.setup = ssc338q_clkgen_setup,
	.num_setup = ARRAY_SIZE(ssc338q_clkgen_setup),
	.clks = ssc338q_clkgen_clks,
	.num_clks = ARRAY_SIZE(ssc338q_clkgen_clks),
};

/*
 * The EMAC RX and TX clocks come from the internal PHY or the RMII pads.
 * Their parent mux is left at its reset value, which selects the internal
 * PHY, so they are registered as gates without a parent.
 */
static const struct ssc338q_clkgen_clk ssc338q_scgp_clks[] = {
	{
		.id = SSC338Q_SCGP_EMAC_RX,
		.name = "emac_rx",
		.offset = 0x88,
	},
	{
		.id = SSC338Q_SCGP_EMAC_TX,
		.name = "emac_tx",
		.offset = 0x8c,
	},
};

/*
 * Bit 7 of this register switches the SD clock between the 12 MHz boot
 * clock and the clock generator output, use the clock generator.
 */
static const struct ssc338q_clkgen_setup ssc338q_scgp_setup[] = {
	{ .offset = 0x94, .bit = 7 },
};

static const struct ssc338q_clkgen_data ssc338q_scgp_data = {
	.setup = ssc338q_scgp_setup,
	.num_setup = ARRAY_SIZE(ssc338q_scgp_setup),
	.clks = ssc338q_scgp_clks,
	.num_clks = ARRAY_SIZE(ssc338q_scgp_clks),
};

static struct clk_hw *ssc338q_clkgen_register(struct device *dev,
					      void __iomem *base,
					      spinlock_t *lock,
					      const struct ssc338q_clkgen_clk *clk)
{
	struct clk_gate *gate;
	struct clk_mux *mux;

	if (!clk->num_parents)
		return devm_clk_hw_register_gate(dev, clk->name, NULL, 0,
						 base + clk->offset,
						 clk->shift + clk->gate_bit,
						 CLK_GATE_SET_TO_DISABLE, lock);

	mux = devm_kzalloc_obj(dev, *mux);
	gate = devm_kzalloc_obj(dev, *gate);
	if (!mux || !gate)
		return ERR_PTR(-ENOMEM);

	mux->reg = base + clk->offset;
	mux->shift = clk->shift + SSC338Q_CLKGEN_MUX_SHIFT;
	mux->mask = BIT(clk->mux_width) - 1;
	mux->table = clk->mux_table;
	mux->lock = lock;

	gate->reg = base + clk->offset;
	gate->bit_idx = clk->shift + clk->gate_bit;
	gate->flags = CLK_GATE_SET_TO_DISABLE;
	gate->lock = lock;

	return devm_clk_hw_register_composite_pdata(dev, clk->name,
						    clk->parents,
						    clk->num_parents,
						    &mux->hw, &clk_mux_ops,
						    NULL, NULL,
						    &gate->hw, &clk_gate_ops,
						    clk->flags);
}

static int ssc338q_clkgen_probe(struct platform_device *pdev)
{
	const struct ssc338q_clkgen_data *data;
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *clk_data;
	void __iomem *base;
	/* Serializes read-modify-write of the mux and gate fields */
	spinlock_t *lock;
	unsigned int i;

	data = device_get_match_data(dev);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	lock = devm_kzalloc_obj(dev, *lock);
	if (!lock)
		return -ENOMEM;
	spin_lock_init(lock);

	for (i = 0; i < data->num_setup; i++) {
		void __iomem *reg = base + data->setup[i].offset;

		writew(readw(reg) | BIT(data->setup[i].bit), reg);
	}

	for (i = 0; i < data->num_factors; i++) {
		const struct ssc338q_clkgen_factor *factor = &data->factors[i];
		struct clk_hw *hw;

		hw = devm_clk_hw_register_fixed_factor_fwname(dev, dev->of_node,
							      factor->name,
							      factor->parent,
							      0, 1, factor->div);
		if (IS_ERR(hw))
			return dev_err_probe(dev, PTR_ERR(hw),
					     "failed to register %s\n",
					     factor->name);
	}

	clk_data = devm_kzalloc_flex(dev, *clk_data, hws, data->num_clks);
	if (!clk_data)
		return -ENOMEM;
	clk_data->num = data->num_clks;

	for (i = 0; i < data->num_clks; i++) {
		const struct ssc338q_clkgen_clk *clk = &data->clks[i];
		struct clk_hw *hw;

		hw = ssc338q_clkgen_register(dev, base, lock, clk);
		if (IS_ERR(hw))
			return dev_err_probe(dev, PTR_ERR(hw),
					     "failed to register %s\n",
					     clk->name);

		clk_data->hws[clk->id] = hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   clk_data);
}

static const struct of_device_id ssc338q_clkgen_of_match[] = {
	{ .compatible = "sstar,ssc338q-clkgen", .data = &ssc338q_clkgen_data },
	{ .compatible = "sstar,ssc338q-scgp", .data = &ssc338q_scgp_data },
	{ }
};

static struct platform_driver ssc338q_clkgen_driver = {
	.driver = {
		.name = "sstar-ssc338q-clkgen",
		.of_match_table = ssc338q_clkgen_of_match,
	},
	.probe = ssc338q_clkgen_probe,
};
builtin_platform_driver(ssc338q_clkgen_driver);
