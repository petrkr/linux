// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q PWM controller
 *
 * Each channel counts its input clock, divided by a per-channel divider,
 * up to an 18-bit period and drives its output high until an 18-bit duty
 * value. A channel runs while its bit in the shared reset register is
 * clear, a stopped channel drives its output low.
 *
 * A duty value of 0 does not give a constant low output, so a 0% duty
 * cycle stops the channel instead, like the vendor driver does.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e).
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/spinlock.h>

/* 16-bit registers on the RIU, 4-byte stride */
#define PWM_DUTY_L		0x08
#define PWM_DUTY_H		0x0c
#define PWM_PERIOD_L		0x10
#define PWM_PERIOD_H		0x14
#define PWM_DIV			0x18
#define PWM_CTRL		0x1c
#define  PWM_CTRL_VDBEN_SW	BIT(0)
#define  PWM_CTRL_DBEN		BIT(1)
#define  PWM_CTRL_POLARITY	BIT(4)
#define PWM_SW_RESET		0x1fc

#define PWM_NUM			10
#define PWM_CNT_MAX		BIT(18)
#define PWM_DIV_MAX		256

struct sstar_pwm {
	void __iomem *base;
	unsigned long rate;
	/* Serializes the read-modify-write of the shared reset register */
	spinlock_t lock;
};

static void __iomem *sstar_pwm_reg(struct sstar_pwm *pwm, unsigned int ch,
				   unsigned int reg)
{
	unsigned int offset = ch < 4 ? ch * 0x80 : 0x200 + (ch - 4) * 0x40;

	return pwm->base + offset + reg;
}

static void sstar_pwm_write_cnt(struct sstar_pwm *pwm, unsigned int ch,
				unsigned int reg_l, unsigned int reg_h, u32 val)
{
	writew(val & 0xffff, sstar_pwm_reg(pwm, ch, reg_l));
	writew((val >> 16) & 0x3, sstar_pwm_reg(pwm, ch, reg_h));
}

static u32 sstar_pwm_read_cnt(struct sstar_pwm *pwm, unsigned int ch,
			      unsigned int reg_l, unsigned int reg_h)
{
	return readw(sstar_pwm_reg(pwm, ch, reg_l)) |
	       (readw(sstar_pwm_reg(pwm, ch, reg_h)) & 0x3) << 16;
}

static void sstar_pwm_set_running(struct sstar_pwm *pwm, unsigned int ch,
				  bool run)
{
	unsigned long flags;
	u16 val;

	spin_lock_irqsave(&pwm->lock, flags);
	val = readw(pwm->base + PWM_SW_RESET);
	if (run)
		val &= ~BIT(ch);
	else
		val |= BIT(ch);
	writew(val, pwm->base + PWM_SW_RESET);
	spin_unlock_irqrestore(&pwm->lock, flags);
}

static int sstar_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwmdev,
			   const struct pwm_state *state)
{
	struct sstar_pwm *pwm = pwmchip_get_drvdata(chip);
	bool inversed = state->polarity == PWM_POLARITY_INVERSED;
	unsigned int ch = pwmdev->hwpwm;
	u64 cycles, duty;
	u32 div;
	u16 ctrl;

	if (!state->enabled) {
		sstar_pwm_set_running(pwm, ch, false);
		return 0;
	}

	cycles = mul_u64_u64_div_u64(state->period, pwm->rate, NSEC_PER_SEC);
	if (cycles < 2)
		return -EINVAL;

	div = DIV64_U64_ROUND_UP(cycles, PWM_CNT_MAX);
	if (div > PWM_DIV_MAX) {
		div = PWM_DIV_MAX;
		cycles = PWM_CNT_MAX * PWM_DIV_MAX;
	}

	duty = mul_u64_u64_div_u64(state->duty_cycle, pwm->rate,
				   NSEC_PER_SEC);
	cycles = div64_u64(cycles, div);
	duty = min(div64_u64(duty, div), cycles);

	/* A constant level: low is a stopped channel, high a full duty */
	if (!duty || duty == cycles) {
		if (!duty != inversed) {
			sstar_pwm_set_running(pwm, ch, false);
			return 0;
		}
		duty = cycles;
		inversed = false;
	}

	writew(div - 1, sstar_pwm_reg(pwm, ch, PWM_DIV));
	sstar_pwm_write_cnt(pwm, ch, PWM_PERIOD_L, PWM_PERIOD_H, cycles - 1);
	sstar_pwm_write_cnt(pwm, ch, PWM_DUTY_L, PWM_DUTY_H, duty - 1);

	ctrl = readw(sstar_pwm_reg(pwm, ch, PWM_CTRL));
	ctrl |= PWM_CTRL_VDBEN_SW | PWM_CTRL_DBEN;
	if (inversed)
		ctrl |= PWM_CTRL_POLARITY;
	else
		ctrl &= ~PWM_CTRL_POLARITY;
	writew(ctrl, sstar_pwm_reg(pwm, ch, PWM_CTRL));

	sstar_pwm_set_running(pwm, ch, true);

	return 0;
}

static int sstar_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwmdev,
			       struct pwm_state *state)
{
	struct sstar_pwm *pwm = pwmchip_get_drvdata(chip);
	unsigned int ch = pwmdev->hwpwm;
	u32 div, period, duty;

	div = readw(sstar_pwm_reg(pwm, ch, PWM_DIV)) + 1;
	period = sstar_pwm_read_cnt(pwm, ch, PWM_PERIOD_L, PWM_PERIOD_H) + 1;
	duty = sstar_pwm_read_cnt(pwm, ch, PWM_DUTY_L, PWM_DUTY_H) + 1;

	state->enabled = !(readw(pwm->base + PWM_SW_RESET) & BIT(ch));
	state->period = DIV64_U64_ROUND_UP((u64)period * div * NSEC_PER_SEC,
					   pwm->rate);
	state->duty_cycle = DIV64_U64_ROUND_UP((u64)duty * div * NSEC_PER_SEC,
					       pwm->rate);
	state->polarity = readw(sstar_pwm_reg(pwm, ch, PWM_CTRL)) &
			  PWM_CTRL_POLARITY ? PWM_POLARITY_INVERSED :
			  PWM_POLARITY_NORMAL;

	return 0;
}

static const struct pwm_ops sstar_pwm_ops = {
	.apply = sstar_pwm_apply,
	.get_state = sstar_pwm_get_state,
};

static int sstar_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct sstar_pwm *pwm;
	struct clk *clk;

	chip = devm_pwmchip_alloc(dev, PWM_NUM, sizeof(*pwm));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	pwm = pwmchip_get_drvdata(chip);
	spin_lock_init(&pwm->lock);

	pwm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pwm->base))
		return PTR_ERR(pwm->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	pwm->rate = clk_get_rate(clk);
	if (!pwm->rate)
		return dev_err_probe(dev, -EINVAL, "invalid clock rate\n");

	chip->ops = &sstar_pwm_ops;

	return devm_pwmchip_add(dev, chip);
}

static const struct of_device_id sstar_pwm_of_match[] = {
	{ .compatible = "sstar,ssc338q-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_pwm_of_match);

static struct platform_driver sstar_pwm_driver = {
	.driver = {
		.name = "sstar-pwm",
		.of_match_table = sstar_pwm_of_match,
	},
	.probe = sstar_pwm_probe,
};
module_platform_driver(sstar_pwm_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q PWM controller driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
