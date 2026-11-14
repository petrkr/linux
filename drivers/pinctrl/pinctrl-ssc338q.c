// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q pin controller and GPIO
 *
 * Every pad has its own 16-bit PADTOP register holding its GPIO state and
 * configuration. The function muxes live in shared PADTOP, CHIPTOP and PM
 * sleep registers.
 *
 * The PM pads have the same kind of register in the PM GPIO block. The
 * pads of the SAR ADC are switched between their analog input and GPIO
 * modes in the SAR block, which also holds their GPIO state.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include <dt-bindings/gpio/sstar,ssc338q-gpio.h>

/* Pad register, 16 bits on the RIU with a 4-byte stride */
#define PAD_IN			BIT(0)
#define PAD_OUT			BIT(1)
#define PAD_OEN			BIT(2)
#define PAD_PM_GPIO_MODE	BIT(3)
#define PAD_PULL		BIT(4)
#define PAD_DRV			BIT(7)

/* SAR block, one bit per pad in each byte */
#define SAR_AISEL		0x44
#define  SAR_AISEL_ANALOG(n)	BIT(n)
#define  SAR_AISEL_OEN(n)	BIT(8 + (n))
#define SAR_GPIO		0x48
#define  SAR_GPIO_OUT(n)	BIT(n)
#define  SAR_GPIO_IN(n)		BIT(8 + (n))

/* Pad descriptor: register bank and register or bit index */
#define PAD_BANK_PADTOP		0
#define PAD_BANK_PM		1
#define PAD_BANK_SAR		2
#define PAD_BANK		GENMASK(11, 8)
#define PAD_INDEX		GENMASK(7, 0)

/* PADTOP function muxes */
#define PAD_MODE_SD		0x67
#define  PAD_MODE_SD0		BIT(8)
#define  PAD_MODE_SD0_CDZ	BIT(9)
#define PAD_MODE_I2C		0x6f
#define  PAD_MODE_I2C0		GENMASK(2, 0)
#define  PAD_MODE_I2C0_PADS	1	/* the I2C0_SCL and I2C0_SDA pads */
#define  PAD_MODE_I2C1		GENMASK(5, 4)
#define  PAD_MODE_I2C1_SR0	1	/* the SR0_IO18 and SR0_IO19 pads */
#define PAD_MODE_SR0		0x69
#define  PAD_MODE_SR0_MIPI	GENMASK(2, 0)
#define  PAD_MODE_SR0_MIPI_1	1	/* the SR0_IO02 to SR0_IO11 pads */
#define  PAD_MODE_SR0_CTRL	BIT(4)	/* the SR0_IO15 to SR0_IO17 pads */
#define  PAD_MODE_SR0_PDN	GENMASK(9, 8)
#define  PAD_MODE_SR0_PDN_1	1	/* the SR0_IO00 pad */
#define PAD_MODE_UART		0x6d
#define  PAD_MODE_UART1		GENMASK(6, 4)
#define PAD_MODE_FUART		0x6e
#define  PAD_MODE_FUART_MASK	GENMASK(6, 4)
#define  PAD_MODE_FUART_RX_TX	5	/* FUART_RX and FUART_TX, no flow control */
#define PAD_MODE_SPI		0x68
#define  PAD_MODE_SPI0		GENMASK(2, 0)
#define  PAD_MODE_SPI0_SD0	3	/* SPI0 mode 3 uses the SD0 pads */

/* CHIPTOP */
#define CHIP_ALLPAD		0x140
#define  CHIP_ALLPAD_IN		BIT(15)

/* PM sleep */
#define PM_LED_MODE		0xa0
#define  PM_LED_MODE_MASK	GENMASK(5, 4)
#define  PM_LED_MODE_ETH	1
#define PM_SPI_IS_GPIO		0xd4
/* SPI pads used by the flash controller, SPI_CZ2 as GPIO */
#define  PM_SPI_IS_GPIO_FSP	0x08

#define SSC338Q_BANK_PIN(_name, _bank, _index)				\
	{								\
		.number = SSC338Q_GPIO_##_name,				\
		.name = #_name,						\
		.drv_data = (void *)((_bank) << 8 | (_index)),		\
	}

#define SSC338Q_PIN(_name, _reg)					\
	SSC338Q_BANK_PIN(_name, PAD_BANK_PADTOP, _reg)
#define SSC338Q_PM_PIN(_name, _reg)					\
	SSC338Q_BANK_PIN(_name, PAD_BANK_PM, _reg)
#define SSC338Q_SAR_PIN(_name, _bit)					\
	SSC338Q_BANK_PIN(_name, PAD_BANK_SAR, _bit)

static const struct pinctrl_pin_desc ssc338q_pins[] = {
	SSC338Q_PIN(GPIO0, 0x00),
	SSC338Q_PIN(GPIO1, 0x01),
	SSC338Q_PIN(GPIO2, 0x02),
	SSC338Q_PIN(GPIO3, 0x03),
	SSC338Q_PIN(GPIO4, 0x04),
	SSC338Q_PIN(GPIO5, 0x05),
	SSC338Q_PIN(GPIO6, 0x06),
	SSC338Q_PIN(GPIO7, 0x07),
	SSC338Q_PIN(GPIO8, 0x08),
	SSC338Q_PIN(GPIO9, 0x09),
	SSC338Q_PIN(GPIO10, 0x0a),
	SSC338Q_PIN(GPIO11, 0x0b),
	SSC338Q_PIN(GPIO12, 0x0c),
	SSC338Q_PIN(GPIO13, 0x0d),
	SSC338Q_PIN(GPIO14, 0x0e),
	SSC338Q_PIN(GPIO15, 0x0f),
	SSC338Q_PIN(SD0_GPIO0, 0x10),
	SSC338Q_PIN(SD0_CDZ, 0x11),
	SSC338Q_PIN(SD0_D1, 0x12),
	SSC338Q_PIN(SD0_D0, 0x13),
	SSC338Q_PIN(SD0_CLK, 0x14),
	SSC338Q_PIN(SD0_CMD, 0x15),
	SSC338Q_PIN(SD0_D3, 0x16),
	SSC338Q_PIN(SD0_D2, 0x17),
	SSC338Q_PIN(I2S0_MCLK, 0x18),
	SSC338Q_PIN(I2S0_BCK, 0x19),
	SSC338Q_PIN(I2S0_WCK, 0x1a),
	SSC338Q_PIN(I2S0_DI, 0x1b),
	SSC338Q_PIN(I2S0_DO, 0x1c),
	SSC338Q_PIN(I2C0_SCL, 0x1d),
	SSC338Q_PIN(I2C0_SDA, 0x1e),
	SSC338Q_PIN(ETH_LED0, 0x20),
	SSC338Q_PIN(ETH_LED1, 0x21),
	SSC338Q_PIN(FUART_RX, 0x22),
	SSC338Q_PIN(FUART_TX, 0x23),
	SSC338Q_PIN(FUART_CTS, 0x24),
	SSC338Q_PIN(FUART_RTS, 0x25),
	SSC338Q_PIN(SD1_CDZ, 0x26),
	SSC338Q_PIN(SD1_D1, 0x27),
	SSC338Q_PIN(SD1_D0, 0x28),
	SSC338Q_PIN(SD1_CLK, 0x29),
	SSC338Q_PIN(SD1_CMD, 0x2a),
	SSC338Q_PIN(SD1_D3, 0x2b),
	SSC338Q_PIN(SD1_D2, 0x2c),
	SSC338Q_PIN(SD1_GPIO0, 0x2d),
	SSC338Q_PIN(SD1_GPIO1, 0x2e),
	SSC338Q_PIN(SPI_CZ, 0x30),
	SSC338Q_PIN(SPI_CK, 0x31),
	SSC338Q_PIN(SPI_DI, 0x32),
	SSC338Q_PIN(SPI_DO, 0x33),
	SSC338Q_PIN(SPI_WPZ, 0x34),
	SSC338Q_PIN(SPI_HLD, 0x35),
	SSC338Q_PIN(SR0_IO00, 0x36),
	SSC338Q_PIN(SR0_IO01, 0x37),
	SSC338Q_PIN(SR0_IO02, 0x38),
	SSC338Q_PIN(SR0_IO03, 0x39),
	SSC338Q_PIN(SR0_IO04, 0x3a),
	SSC338Q_PIN(SR0_IO05, 0x3b),
	SSC338Q_PIN(SR0_IO06, 0x3c),
	SSC338Q_PIN(SR0_IO07, 0x3d),
	SSC338Q_PIN(SR0_IO08, 0x3e),
	SSC338Q_PIN(SR0_IO09, 0x3f),
	SSC338Q_PIN(SR0_IO10, 0x40),
	SSC338Q_PIN(SR0_IO11, 0x41),
	SSC338Q_PIN(SR0_IO12, 0x42),
	SSC338Q_PIN(SR0_IO13, 0x43),
	SSC338Q_PIN(SR0_IO14, 0x44),
	SSC338Q_PIN(SR0_IO15, 0x45),
	SSC338Q_PIN(SR0_IO16, 0x46),
	SSC338Q_PIN(SR0_IO17, 0x47),
	SSC338Q_PIN(SR0_IO18, 0x48),
	SSC338Q_PIN(SR0_IO19, 0x49),
	SSC338Q_PIN(SR1_IO00, 0x4a),
	SSC338Q_PIN(SR1_IO01, 0x4b),
	SSC338Q_PIN(SR1_IO02, 0x4c),
	SSC338Q_PIN(SR1_IO03, 0x4d),
	SSC338Q_PIN(SR1_IO04, 0x4e),
	SSC338Q_PIN(SR1_IO05, 0x4f),
	SSC338Q_PIN(SR1_IO06, 0x50),
	SSC338Q_PIN(SR1_IO07, 0x51),
	SSC338Q_PIN(SR1_IO08, 0x52),
	SSC338Q_PIN(SR1_IO09, 0x53),
	SSC338Q_PIN(SR1_IO10, 0x54),
	SSC338Q_PIN(SR1_IO11, 0x55),
	SSC338Q_PIN(SR1_IO12, 0x56),
	SSC338Q_PIN(SR1_IO13, 0x57),
	SSC338Q_PIN(SR1_IO14, 0x58),
	SSC338Q_PIN(SR1_IO15, 0x59),
	SSC338Q_PIN(SR1_IO16, 0x5a),
	SSC338Q_PIN(SR1_IO17, 0x5b),
	SSC338Q_PIN(SR1_IO18, 0x5c),
	SSC338Q_PIN(SR1_IO19, 0x5d),
	SSC338Q_PM_PIN(PM_GPIO0, 0x00),
	SSC338Q_PM_PIN(PM_GPIO1, 0x01),
	SSC338Q_PM_PIN(PM_GPIO2, 0x02),
	SSC338Q_PM_PIN(PM_GPIO3, 0x03),
	SSC338Q_PM_PIN(PM_GPIO4, 0x04),
	SSC338Q_PM_PIN(PM_GPIO5, 0x05),
	SSC338Q_PM_PIN(PM_GPIO6, 0x06),
	SSC338Q_PM_PIN(PM_GPIO7, 0x07),
	SSC338Q_PM_PIN(PM_GPIO8, 0x08),
	SSC338Q_PM_PIN(PM_GPIO9, 0x09),
	SSC338Q_PM_PIN(PM_GPIO10, 0x10),
	SSC338Q_SAR_PIN(SAR_GPIO0, 0),
	SSC338Q_SAR_PIN(SAR_GPIO1, 1),
	SSC338Q_SAR_PIN(SAR_GPIO2, 2),
	SSC338Q_SAR_PIN(SAR_GPIO3, 3),
};

static const unsigned int ssc338q_sd0_pins[] = {
	SSC338Q_GPIO_SD0_D1, SSC338Q_GPIO_SD0_D0, SSC338Q_GPIO_SD0_CLK,
	SSC338Q_GPIO_SD0_CMD, SSC338Q_GPIO_SD0_D3, SSC338Q_GPIO_SD0_D2,
};

static const unsigned int ssc338q_sd0_cdz_pins[] = {
	SSC338Q_GPIO_SD0_CDZ,
};

static const unsigned int ssc338q_i2c0_pins[] = {
	SSC338Q_GPIO_I2C0_SCL, SSC338Q_GPIO_I2C0_SDA,
};

static const unsigned int ssc338q_i2c1_pins[] = {
	SSC338Q_GPIO_SR0_IO18, SSC338Q_GPIO_SR0_IO19,
};

static const unsigned int ssc338q_sr0_mipi_pins[] = {
	SSC338Q_GPIO_SR0_IO02, SSC338Q_GPIO_SR0_IO03, SSC338Q_GPIO_SR0_IO04,
	SSC338Q_GPIO_SR0_IO05, SSC338Q_GPIO_SR0_IO06, SSC338Q_GPIO_SR0_IO07,
	SSC338Q_GPIO_SR0_IO08, SSC338Q_GPIO_SR0_IO09, SSC338Q_GPIO_SR0_IO10,
	SSC338Q_GPIO_SR0_IO11,
};

static const unsigned int ssc338q_sr0_ctrl_pins[] = {
	SSC338Q_GPIO_SR0_IO15, SSC338Q_GPIO_SR0_IO16, SSC338Q_GPIO_SR0_IO17,
};

static const unsigned int ssc338q_sr0_pdn_pins[] = {
	SSC338Q_GPIO_SR0_IO00,
};

/* UART1 pad pairs; the vendor looks the mode up by the first pad as TX */
static const unsigned int ssc338q_uart1_gpio0_pins[] = {
	SSC338Q_GPIO_GPIO0, SSC338Q_GPIO_GPIO1,
};

static const unsigned int ssc338q_uart1_i2s0_pins[] = {
	SSC338Q_GPIO_I2S0_DI, SSC338Q_GPIO_I2S0_DO,
};

static const unsigned int ssc338q_uart1_eth_led_pins[] = {
	SSC338Q_GPIO_ETH_LED0, SSC338Q_GPIO_ETH_LED1,
};

static const unsigned int ssc338q_uart1_gpio4_pins[] = {
	SSC338Q_GPIO_GPIO4, SSC338Q_GPIO_GPIO5,
};

static const unsigned int ssc338q_uart1_fuart_pins[] = {
	SSC338Q_GPIO_FUART_CTS, SSC338Q_GPIO_FUART_RTS,
};

static const unsigned int ssc338q_uart1_gpio8_pins[] = {
	SSC338Q_GPIO_GPIO8, SSC338Q_GPIO_GPIO9,
};

static const unsigned int ssc338q_fuart_pins[] = {
	SSC338Q_GPIO_FUART_RX, SSC338Q_GPIO_FUART_TX,
};

static const unsigned int ssc338q_sar0_pins[] = { SSC338Q_GPIO_SAR_GPIO0 };
static const unsigned int ssc338q_sar1_pins[] = { SSC338Q_GPIO_SAR_GPIO1 };
static const unsigned int ssc338q_sar2_pins[] = { SSC338Q_GPIO_SAR_GPIO2 };
static const unsigned int ssc338q_sar3_pins[] = { SSC338Q_GPIO_SAR_GPIO3 };

enum ssc338q_group {
	SSC338Q_GROUP_SD0,
	SSC338Q_GROUP_SD0_CDZ,
	SSC338Q_GROUP_PM_SPI,
	SSC338Q_GROUP_PM_LED,
	SSC338Q_GROUP_I2C0,
	SSC338Q_GROUP_SAR0,
	SSC338Q_GROUP_SAR1,
	SSC338Q_GROUP_SAR2,
	SSC338Q_GROUP_SAR3,
	SSC338Q_GROUP_I2C1,
	SSC338Q_GROUP_UART1_GPIO0,
	SSC338Q_GROUP_UART1_I2S0,
	SSC338Q_GROUP_UART1_ETH_LED,
	SSC338Q_GROUP_UART1_GPIO4,
	SSC338Q_GROUP_UART1_FUART,
	SSC338Q_GROUP_UART1_GPIO8,
	SSC338Q_GROUP_FUART,
	SSC338Q_GROUP_SR0_MIPI,
	SSC338Q_GROUP_SR0_CTRL,
	SSC338Q_GROUP_SR0_PDN,
};

/* The PM sleep groups have no PADTOP pads */
static const struct pingroup ssc338q_groups[] = {
	[SSC338Q_GROUP_SD0] = PINCTRL_PINGROUP("sd0", ssc338q_sd0_pins,
					       ARRAY_SIZE(ssc338q_sd0_pins)),
	[SSC338Q_GROUP_SD0_CDZ] = PINCTRL_PINGROUP("sd0_cdz", ssc338q_sd0_cdz_pins,
						   ARRAY_SIZE(ssc338q_sd0_cdz_pins)),
	[SSC338Q_GROUP_PM_SPI] = PINCTRL_PINGROUP("pm_spi", NULL, 0),
	[SSC338Q_GROUP_PM_LED] = PINCTRL_PINGROUP("pm_led", NULL, 0),
	[SSC338Q_GROUP_I2C0] = PINCTRL_PINGROUP("i2c0", ssc338q_i2c0_pins,
						ARRAY_SIZE(ssc338q_i2c0_pins)),
	[SSC338Q_GROUP_SAR0] = PINCTRL_PINGROUP("sar0", ssc338q_sar0_pins, 1),
	[SSC338Q_GROUP_SAR1] = PINCTRL_PINGROUP("sar1", ssc338q_sar1_pins, 1),
	[SSC338Q_GROUP_SAR2] = PINCTRL_PINGROUP("sar2", ssc338q_sar2_pins, 1),
	[SSC338Q_GROUP_SAR3] = PINCTRL_PINGROUP("sar3", ssc338q_sar3_pins, 1),
	[SSC338Q_GROUP_I2C1] = PINCTRL_PINGROUP("i2c1", ssc338q_i2c1_pins,
						ARRAY_SIZE(ssc338q_i2c1_pins)),
	[SSC338Q_GROUP_UART1_GPIO0] = PINCTRL_PINGROUP("uart1_gpio0",
						ssc338q_uart1_gpio0_pins,
						ARRAY_SIZE(ssc338q_uart1_gpio0_pins)),
	[SSC338Q_GROUP_UART1_I2S0] = PINCTRL_PINGROUP("uart1_i2s0",
						ssc338q_uart1_i2s0_pins,
						ARRAY_SIZE(ssc338q_uart1_i2s0_pins)),
	[SSC338Q_GROUP_UART1_ETH_LED] = PINCTRL_PINGROUP("uart1_eth_led",
						ssc338q_uart1_eth_led_pins,
						ARRAY_SIZE(ssc338q_uart1_eth_led_pins)),
	[SSC338Q_GROUP_UART1_GPIO4] = PINCTRL_PINGROUP("uart1_gpio4",
						ssc338q_uart1_gpio4_pins,
						ARRAY_SIZE(ssc338q_uart1_gpio4_pins)),
	[SSC338Q_GROUP_UART1_FUART] = PINCTRL_PINGROUP("uart1_fuart",
						ssc338q_uart1_fuart_pins,
						ARRAY_SIZE(ssc338q_uart1_fuart_pins)),
	[SSC338Q_GROUP_UART1_GPIO8] = PINCTRL_PINGROUP("uart1_gpio8",
						ssc338q_uart1_gpio8_pins,
						ARRAY_SIZE(ssc338q_uart1_gpio8_pins)),
	[SSC338Q_GROUP_FUART] = PINCTRL_PINGROUP("fuart", ssc338q_fuart_pins,
						 ARRAY_SIZE(ssc338q_fuart_pins)),
	[SSC338Q_GROUP_SR0_MIPI] = PINCTRL_PINGROUP("sr0_mipi",
						ssc338q_sr0_mipi_pins,
						ARRAY_SIZE(ssc338q_sr0_mipi_pins)),
	[SSC338Q_GROUP_SR0_CTRL] = PINCTRL_PINGROUP("sr0_ctrl",
						ssc338q_sr0_ctrl_pins,
						ARRAY_SIZE(ssc338q_sr0_ctrl_pins)),
	[SSC338Q_GROUP_SR0_PDN] = PINCTRL_PINGROUP("sr0_pdn",
						ssc338q_sr0_pdn_pins,
						ARRAY_SIZE(ssc338q_sr0_pdn_pins)),
};

enum ssc338q_function {
	SSC338Q_FUNC_GPIO,
	SSC338Q_FUNC_SD0,
	SSC338Q_FUNC_SD0_CDZ,
	SSC338Q_FUNC_PM_SPI,
	SSC338Q_FUNC_ETH_LED,
	SSC338Q_FUNC_I2C0,
	SSC338Q_FUNC_ADC,
	SSC338Q_FUNC_I2C1,
	SSC338Q_FUNC_UART1,
	SSC338Q_FUNC_FUART,
	SSC338Q_FUNC_SR0_MIPI,
	SSC338Q_FUNC_SR0_CTRL,
	SSC338Q_FUNC_SR0_PDN,
};

static const char * const ssc338q_gpio_groups[] = {
	"sd0", "sd0_cdz", "i2c0", "sar0", "sar1", "sar2", "sar3", "i2c1",
	"uart1_gpio0", "uart1_i2s0", "uart1_eth_led", "uart1_gpio4",
	"uart1_fuart", "uart1_gpio8", "fuart", "sr0_mipi", "sr0_ctrl",
	"sr0_pdn",
};
static const char * const ssc338q_sd0_groups[] = { "sd0" };
static const char * const ssc338q_sd0_cdz_groups[] = { "sd0_cdz" };
static const char * const ssc338q_pm_spi_groups[] = { "pm_spi" };
static const char * const ssc338q_eth_led_groups[] = { "pm_led" };
static const char * const ssc338q_i2c0_groups[] = { "i2c0" };
static const char * const ssc338q_i2c1_groups[] = { "i2c1" };
static const char * const ssc338q_uart1_groups[] = {
	"uart1_gpio0", "uart1_i2s0", "uart1_eth_led", "uart1_gpio4",
	"uart1_fuart", "uart1_gpio8",
};
static const char * const ssc338q_fuart_groups[] = { "fuart" };
static const char * const ssc338q_sr0_mipi_groups[] = { "sr0_mipi" };
static const char * const ssc338q_sr0_ctrl_groups[] = { "sr0_ctrl" };
static const char * const ssc338q_sr0_pdn_groups[] = { "sr0_pdn" };
static const char * const ssc338q_adc_groups[] = {
	"sar0", "sar1", "sar2", "sar3",
};

static const struct pinfunction ssc338q_functions[] = {
	[SSC338Q_FUNC_GPIO] = PINCTRL_PINFUNCTION("gpio", ssc338q_gpio_groups,
						  ARRAY_SIZE(ssc338q_gpio_groups)),
	[SSC338Q_FUNC_SD0] = PINCTRL_PINFUNCTION("sd0", ssc338q_sd0_groups,
						 ARRAY_SIZE(ssc338q_sd0_groups)),
	[SSC338Q_FUNC_SD0_CDZ] = PINCTRL_PINFUNCTION("sd0_cdz", ssc338q_sd0_cdz_groups,
						     ARRAY_SIZE(ssc338q_sd0_cdz_groups)),
	[SSC338Q_FUNC_PM_SPI] = PINCTRL_PINFUNCTION("pm_spi", ssc338q_pm_spi_groups,
						    ARRAY_SIZE(ssc338q_pm_spi_groups)),
	[SSC338Q_FUNC_ETH_LED] = PINCTRL_PINFUNCTION("eth_led", ssc338q_eth_led_groups,
						     ARRAY_SIZE(ssc338q_eth_led_groups)),
	[SSC338Q_FUNC_I2C0] = PINCTRL_PINFUNCTION("i2c0", ssc338q_i2c0_groups,
						  ARRAY_SIZE(ssc338q_i2c0_groups)),
	[SSC338Q_FUNC_ADC] = PINCTRL_PINFUNCTION("adc", ssc338q_adc_groups,
						 ARRAY_SIZE(ssc338q_adc_groups)),
	[SSC338Q_FUNC_I2C1] = PINCTRL_PINFUNCTION("i2c1", ssc338q_i2c1_groups,
						  ARRAY_SIZE(ssc338q_i2c1_groups)),
	[SSC338Q_FUNC_UART1] = PINCTRL_PINFUNCTION("uart1", ssc338q_uart1_groups,
						   ARRAY_SIZE(ssc338q_uart1_groups)),
	[SSC338Q_FUNC_FUART] = PINCTRL_PINFUNCTION("fuart", ssc338q_fuart_groups,
						   ARRAY_SIZE(ssc338q_fuart_groups)),
	[SSC338Q_FUNC_SR0_MIPI] = PINCTRL_PINFUNCTION("sr0_mipi",
						      ssc338q_sr0_mipi_groups,
						      ARRAY_SIZE(ssc338q_sr0_mipi_groups)),
	[SSC338Q_FUNC_SR0_CTRL] = PINCTRL_PINFUNCTION("sr0_ctrl",
						      ssc338q_sr0_ctrl_groups,
						      ARRAY_SIZE(ssc338q_sr0_ctrl_groups)),
	[SSC338Q_FUNC_SR0_PDN] = PINCTRL_PINFUNCTION("sr0_pdn",
						     ssc338q_sr0_pdn_groups,
						     ARRAY_SIZE(ssc338q_sr0_pdn_groups)),
};

struct ssc338q_pinctrl {
	struct device *dev;
	void __iomem *padtop;
	void __iomem *pm_gpio;
	struct regmap *chiptop;
	struct regmap *pmsleep;
	struct regmap *sar;
	struct pinctrl_dev *pctldev;
	struct gpio_chip gc;
	/* Serializes read-modify-write of the PADTOP registers */
	raw_spinlock_t lock;
};

static unsigned int ssc338q_pad_bank(unsigned int pin)
{
	return FIELD_GET(PAD_BANK, (uintptr_t)ssc338q_pins[pin].drv_data);
}

static unsigned int ssc338q_pad_index(unsigned int pin)
{
	return FIELD_GET(PAD_INDEX, (uintptr_t)ssc338q_pins[pin].drv_data);
}

/* The pad register of a PADTOP or PM pad */
static void __iomem *ssc338q_pad(struct ssc338q_pinctrl *pctl, unsigned int pin)
{
	void __iomem *base = ssc338q_pad_bank(pin) == PAD_BANK_PM ?
			     pctl->pm_gpio : pctl->padtop;

	return base + ssc338q_pad_index(pin) * 4;
}

static void ssc338q_rmw(struct ssc338q_pinctrl *pctl, void __iomem *reg,
			u16 clear, u16 set)
{
	unsigned long flags;
	u16 val;

	raw_spin_lock_irqsave(&pctl->lock, flags);
	val = readw(reg);
	writew((val & ~clear) | set, reg);
	raw_spin_unlock_irqrestore(&pctl->lock, flags);
}

static int ssc338q_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(ssc338q_groups);
}

static const char *ssc338q_get_group_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	return ssc338q_groups[selector].name;
}

static int ssc338q_get_group_pins(struct pinctrl_dev *pctldev,
				  unsigned int selector,
				  const unsigned int **pins,
				  unsigned int *npins)
{
	*pins = ssc338q_groups[selector].pins;
	*npins = ssc338q_groups[selector].npins;

	return 0;
}

static const struct pinctrl_ops ssc338q_pctl_ops = {
	.get_groups_count	= ssc338q_get_groups_count,
	.get_group_name		= ssc338q_get_group_name,
	.get_group_pins		= ssc338q_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinconf_generic_dt_free_map,
};

static int ssc338q_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(ssc338q_functions);
}

static const char *ssc338q_get_function_name(struct pinctrl_dev *pctldev,
					     unsigned int selector)
{
	return ssc338q_functions[selector].name;
}

static int ssc338q_get_function_groups(struct pinctrl_dev *pctldev,
				       unsigned int selector,
				       const char * const **groups,
				       unsigned int * const ngroups)
{
	*groups = ssc338q_functions[selector].groups;
	*ngroups = ssc338q_functions[selector].ngroups;

	return 0;
}

static int ssc338q_mux_sd0(struct ssc338q_pinctrl *pctl, bool enable)
{
	void __iomem *mode = pctl->padtop + PAD_MODE_SD * 4;
	void __iomem *spi = pctl->padtop + PAD_MODE_SPI * 4;
	unsigned int i;

	if (!enable) {
		ssc338q_rmw(pctl, mode, PAD_MODE_SD0, 0);
		return 0;
	}

	/* SPI0 mode 3 claims the SD0 pads */
	if (FIELD_GET(PAD_MODE_SPI0, readw(spi)) == PAD_MODE_SPI0_SD0)
		ssc338q_rmw(pctl, spi, PAD_MODE_SPI0, 0);

	ssc338q_rmw(pctl, mode, 0, PAD_MODE_SD0);

	for (i = 0; i < ARRAY_SIZE(ssc338q_sd0_pins); i++)
		ssc338q_rmw(pctl, ssc338q_pad(pctl, ssc338q_sd0_pins[i]),
			    PAD_DRV, 0);

	return 0;
}

static int ssc338q_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			   unsigned int group)
{
	struct ssc338q_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *mode = pctl->padtop + PAD_MODE_SD * 4;

	switch (group) {
	case SSC338Q_GROUP_SD0:
		if (func == SSC338Q_FUNC_SD0 || func == SSC338Q_FUNC_GPIO)
			return ssc338q_mux_sd0(pctl, func == SSC338Q_FUNC_SD0);
		break;
	case SSC338Q_GROUP_SD0_CDZ:
		if (func == SSC338Q_FUNC_SD0_CDZ) {
			ssc338q_rmw(pctl, mode, 0, PAD_MODE_SD0_CDZ);
			return 0;
		}
		if (func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, mode, PAD_MODE_SD0_CDZ, 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_I2C0:
		if (func == SSC338Q_FUNC_I2C0 || func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_I2C * 4,
				    PAD_MODE_I2C0,
				    func == SSC338Q_FUNC_I2C0 ?
				    FIELD_PREP(PAD_MODE_I2C0,
					       PAD_MODE_I2C0_PADS) : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_I2C1:
		if (func == SSC338Q_FUNC_I2C1 || func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_I2C * 4,
				    PAD_MODE_I2C1,
				    func == SSC338Q_FUNC_I2C1 ?
				    FIELD_PREP(PAD_MODE_I2C1,
					       PAD_MODE_I2C1_SR0) : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_UART1_GPIO0 ... SSC338Q_GROUP_UART1_GPIO8:
		if (func == SSC338Q_FUNC_UART1 || func == SSC338Q_FUNC_GPIO) {
			unsigned int n = group - SSC338Q_GROUP_UART1_GPIO0 + 1;

			/* The ETH_LED pads are also claimed by the PM LED mux */
			if (func == SSC338Q_FUNC_UART1 &&
			    group == SSC338Q_GROUP_UART1_ETH_LED)
				regmap_update_bits(pctl->pmsleep, PM_LED_MODE,
						   PM_LED_MODE_MASK, 0);

			/* Only the UART1 field, the UART0 mode shares the register */
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_UART * 4,
				    PAD_MODE_UART1,
				    func == SSC338Q_FUNC_UART1 ?
				    FIELD_PREP(PAD_MODE_UART1, n) : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_FUART:
		if (func == SSC338Q_FUNC_FUART || func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_FUART * 4,
				    PAD_MODE_FUART_MASK,
				    func == SSC338Q_FUNC_FUART ?
				    FIELD_PREP(PAD_MODE_FUART_MASK,
					       PAD_MODE_FUART_RX_TX) : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_SR0_MIPI:
		if (func == SSC338Q_FUNC_SR0_MIPI || func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_SR0 * 4,
				    PAD_MODE_SR0_MIPI,
				    func == SSC338Q_FUNC_SR0_MIPI ?
				    FIELD_PREP(PAD_MODE_SR0_MIPI,
					       PAD_MODE_SR0_MIPI_1) : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_SR0_CTRL:
		if (func == SSC338Q_FUNC_SR0_CTRL || func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_SR0 * 4,
				    PAD_MODE_SR0_CTRL,
				    func == SSC338Q_FUNC_SR0_CTRL ?
				    PAD_MODE_SR0_CTRL : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_SR0_PDN:
		if (func == SSC338Q_FUNC_SR0_PDN || func == SSC338Q_FUNC_GPIO) {
			ssc338q_rmw(pctl, pctl->padtop + PAD_MODE_SR0 * 4,
				    PAD_MODE_SR0_PDN,
				    func == SSC338Q_FUNC_SR0_PDN ?
				    FIELD_PREP(PAD_MODE_SR0_PDN,
					       PAD_MODE_SR0_PDN_1) : 0);
			return 0;
		}
		break;
	case SSC338Q_GROUP_SAR0 ... SSC338Q_GROUP_SAR3:
		if (func == SSC338Q_FUNC_ADC || func == SSC338Q_FUNC_GPIO) {
			unsigned int n = group - SSC338Q_GROUP_SAR0;

			return regmap_update_bits(pctl->sar, SAR_AISEL,
						  SAR_AISEL_ANALOG(n),
						  func == SSC338Q_FUNC_ADC ?
						  SAR_AISEL_ANALOG(n) : 0);
		}
		break;
	case SSC338Q_GROUP_PM_SPI:
		if (func == SSC338Q_FUNC_PM_SPI)
			return regmap_write(pctl->pmsleep, PM_SPI_IS_GPIO,
					    PM_SPI_IS_GPIO_FSP);
		break;
	case SSC338Q_GROUP_PM_LED:
		if (func == SSC338Q_FUNC_ETH_LED)
			return regmap_update_bits(pctl->pmsleep, PM_LED_MODE,
						  PM_LED_MODE_MASK,
						  FIELD_PREP(PM_LED_MODE_MASK,
							     PM_LED_MODE_ETH));
		break;
	}

	dev_err(pctl->dev, "group %s does not support function %s\n",
		ssc338q_groups[group].name, ssc338q_functions[func].name);

	return -EINVAL;
}

static const struct pinmux_ops ssc338q_pmx_ops = {
	.get_functions_count	= ssc338q_get_functions_count,
	.get_function_name	= ssc338q_get_function_name,
	.get_function_groups	= ssc338q_get_function_groups,
	.set_mux		= ssc338q_set_mux,
};

static int ssc338q_pin_config_get(struct pinctrl_dev *pctldev,
				  unsigned int pin, unsigned long *config)
{
	struct ssc338q_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg;
	u16 val;

	if (ssc338q_pad_bank(pin) == PAD_BANK_SAR)
		return -ENOTSUPP;

	val = readw(ssc338q_pad(pctl, pin));

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (val & PAD_PULL)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(val & PAD_PULL))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (!(val & PAD_OEN))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_LEVEL:
		if (val & PAD_OEN)
			return -EINVAL;
		arg = !!(val & PAD_OUT);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int ssc338q_pin_config_set(struct pinctrl_dev *pctldev,
				  unsigned int pin, unsigned long *configs,
				  unsigned int num_configs)
{
	struct ssc338q_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *reg;
	unsigned int i;

	if (ssc338q_pad_bank(pin) == PAD_BANK_SAR)
		return -ENOTSUPP;

	reg = ssc338q_pad(pctl, pin);

	for (i = 0; i < num_configs; i++) {
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (pinconf_to_config_param(configs[i])) {
		case PIN_CONFIG_BIAS_DISABLE:
			ssc338q_rmw(pctl, reg, PAD_PULL, 0);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			ssc338q_rmw(pctl, reg, 0, PAD_PULL);
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			ssc338q_rmw(pctl, reg, 0, PAD_OEN);
			break;
		case PIN_CONFIG_LEVEL:
			ssc338q_rmw(pctl, reg, PAD_OUT | PAD_OEN,
				    arg ? PAD_OUT : 0);
			break;
		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static int ssc338q_pin_config_group_set(struct pinctrl_dev *pctldev,
					unsigned int group,
					unsigned long *configs,
					unsigned int num_configs)
{
	const struct pingroup *grp = &ssc338q_groups[group];
	unsigned int i;
	int ret;

	for (i = 0; i < grp->npins; i++) {
		ret = ssc338q_pin_config_set(pctldev, grp->pins[i], configs,
					     num_configs);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops ssc338q_conf_ops = {
	.is_generic		= true,
	.pin_config_get		= ssc338q_pin_config_get,
	.pin_config_set		= ssc338q_pin_config_set,
	.pin_config_group_set	= ssc338q_pin_config_group_set,
};

static int ssc338q_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	struct ssc338q_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int n = ssc338q_pad_index(offset);

	switch (ssc338q_pad_bank(offset)) {
	case PAD_BANK_PM:
		ssc338q_rmw(pctl, ssc338q_pad(pctl, offset), 0,
			    PAD_PM_GPIO_MODE);
		break;
	case PAD_BANK_SAR:
		return regmap_update_bits(pctl->sar, SAR_AISEL,
					  SAR_AISEL_ANALOG(n), 0);
	}

	return 0;
}

static int ssc338q_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct ssc338q_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int n = ssc338q_pad_index(offset);
	unsigned int val;
	bool input;
	int ret;

	if (ssc338q_pad_bank(offset) == PAD_BANK_SAR) {
		ret = regmap_read(pctl->sar, SAR_AISEL, &val);
		if (ret)
			return ret;
		input = val & SAR_AISEL_OEN(n);
	} else {
		input = readw(ssc338q_pad(pctl, offset)) & PAD_OEN;
	}

	return input ? GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int ssc338q_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	struct ssc338q_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int n = ssc338q_pad_index(offset);

	if (ssc338q_pad_bank(offset) == PAD_BANK_SAR)
		return regmap_update_bits(pctl->sar, SAR_AISEL,
					  SAR_AISEL_OEN(n), SAR_AISEL_OEN(n));

	ssc338q_rmw(pctl, ssc338q_pad(pctl, offset), 0, PAD_OEN);

	return 0;
}

static int ssc338q_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int offset, int value)
{
	struct ssc338q_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int n = ssc338q_pad_index(offset);
	int ret;

	if (ssc338q_pad_bank(offset) == PAD_BANK_SAR) {
		ret = regmap_update_bits(pctl->sar, SAR_GPIO, SAR_GPIO_OUT(n),
					 value ? SAR_GPIO_OUT(n) : 0);
		if (ret)
			return ret;

		return regmap_update_bits(pctl->sar, SAR_AISEL,
					  SAR_AISEL_OEN(n), 0);
	}

	ssc338q_rmw(pctl, ssc338q_pad(pctl, offset), PAD_OUT | PAD_OEN,
		    value ? PAD_OUT : 0);

	return 0;
}

static int ssc338q_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct ssc338q_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int n = ssc338q_pad_index(offset);
	unsigned int val;
	int ret;

	if (ssc338q_pad_bank(offset) == PAD_BANK_SAR) {
		ret = regmap_read(pctl->sar, SAR_GPIO, &val);
		if (ret)
			return ret;

		return !!(val & SAR_GPIO_IN(n));
	}

	return !!(readw(ssc338q_pad(pctl, offset)) & PAD_IN);
}

static int ssc338q_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int value)
{
	struct ssc338q_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int n = ssc338q_pad_index(offset);

	if (ssc338q_pad_bank(offset) == PAD_BANK_SAR)
		return regmap_update_bits(pctl->sar, SAR_GPIO, SAR_GPIO_OUT(n),
					  value ? SAR_GPIO_OUT(n) : 0);

	ssc338q_rmw(pctl, ssc338q_pad(pctl, offset), PAD_OUT,
		    value ? PAD_OUT : 0);

	return 0;
}

static const struct pinctrl_desc ssc338q_pinctrl_desc = {
	.name		= "ssc338q-pinctrl",
	.pins		= ssc338q_pins,
	.npins		= ARRAY_SIZE(ssc338q_pins),
	.pctlops	= &ssc338q_pctl_ops,
	.pmxops		= &ssc338q_pmx_ops,
	.confops	= &ssc338q_conf_ops,
	.owner		= THIS_MODULE,
};

static int ssc338q_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssc338q_pinctrl *pctl;
	const char **names;
	unsigned int i;
	int ret;

	pctl = devm_kzalloc_obj(dev, *pctl);
	if (!pctl)
		return -ENOMEM;

	pctl->dev = dev;
	raw_spin_lock_init(&pctl->lock);

	pctl->padtop = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->padtop))
		return PTR_ERR(pctl->padtop);

	pctl->pm_gpio = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pctl->pm_gpio))
		return PTR_ERR(pctl->pm_gpio);

	pctl->chiptop = syscon_regmap_lookup_by_phandle(dev->of_node,
							"sstar,chiptop");
	if (IS_ERR(pctl->chiptop))
		return dev_err_probe(dev, PTR_ERR(pctl->chiptop),
				     "failed to get CHIPTOP\n");

	pctl->pmsleep = syscon_regmap_lookup_by_phandle(dev->of_node,
							"sstar,pmsleep");
	if (IS_ERR(pctl->pmsleep))
		return dev_err_probe(dev, PTR_ERR(pctl->pmsleep),
				     "failed to get PM sleep\n");

	pctl->sar = syscon_regmap_lookup_by_phandle(dev->of_node, "sstar,sar");
	if (IS_ERR(pctl->sar))
		return dev_err_probe(dev, PTR_ERR(pctl->sar),
				     "failed to get SAR\n");

	/* Release the pads from the reset-time "all pads input" mode */
	ret = regmap_update_bits(pctl->chiptop, CHIP_ALLPAD, CHIP_ALLPAD_IN, 0);
	if (ret)
		return ret;

	ret = devm_pinctrl_register_and_init(dev, &ssc338q_pinctrl_desc, pctl,
					     &pctl->pctldev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register pinctrl\n");

	ret = pinctrl_enable(pctl->pctldev);
	if (ret)
		return ret;

	names = devm_kzalloc_objs(dev, *names, ARRAY_SIZE(ssc338q_pins));
	if (!names)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(ssc338q_pins); i++)
		names[i] = ssc338q_pins[i].name;

	pctl->gc.label = dev_name(dev);
	pctl->gc.names = names;
	pctl->gc.parent = dev;
	pctl->gc.owner = THIS_MODULE;
	pctl->gc.base = -1;
	pctl->gc.ngpio = SSC338Q_GPIO_COUNT;
	pctl->gc.can_sleep = false;
	pctl->gc.request = ssc338q_gpio_request;
	pctl->gc.get_direction = ssc338q_gpio_get_direction;
	pctl->gc.direction_input = ssc338q_gpio_direction_input;
	pctl->gc.direction_output = ssc338q_gpio_direction_output;
	pctl->gc.get = ssc338q_gpio_get;
	pctl->gc.set = ssc338q_gpio_set;

	return devm_gpiochip_add_data(dev, &pctl->gc, pctl);
}

static const struct of_device_id ssc338q_pinctrl_of_match[] = {
	{ .compatible = "sstar,ssc338q-pinctrl" },
	{ }
};

static struct platform_driver ssc338q_pinctrl_driver = {
	.driver = {
		.name = "ssc338q-pinctrl",
		.of_match_table = ssc338q_pinctrl_of_match,
	},
	.probe = ssc338q_pinctrl_probe,
};
builtin_platform_driver(ssc338q_pinctrl_driver);
