// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar SSC338Q audio (BACH) capture from the analog microphone input
 *
 * The BACH block holds the digital audio path and a DMA engine with a
 * writer that stores the samples into a ring buffer in memory. It counts
 * the stored data in 8-byte units (level), software tells it how much it
 * has consumed. The analog front end (ATOP) holds the microphone amplifier
 * and the ADC.
 *
 * Register information from the SigmaStar Linux 4.9 BSP maintained by OpenIPC
 * (https://github.com/OpenIPC/linux, branch sigmastar-infinity6e), from
 * the vendor audio module and from the state it leaves the hardware in.
 *
 * Copyright (c) 2026 Petr Kracik <petrkr@petrkr.net>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* 16-bit registers on the RIU, 4-byte stride, three banks of 0x80 */
#define BACH(bank, reg)		(((bank) * 0x100 + (reg)) * 2)

#define BACH_SR			BACH(0, 0x02)
#define  BACH_SR_WR		GENMASK(11, 8)
#define BACH_DMA_INT		BACH(0, 0x80)
#define  BACH_DMA_INT_WR_CLR	BIT(9)
#define  BACH_DMA_INT_WR_EN0	BIT(11)
#define  BACH_DMA_INT_WR_EN1	BIT(14)
#define BACH_WR_FLAGS		BACH(0, 0x90)
#define BACH_WR_CTRL		BACH(0, 0x92)
#define  BACH_WR_CTRL_EN	BIT(15)
#define  BACH_WR_CTRL_RESET	BIT(14)
#define  BACH_WR_CTRL_TRIG	BIT(13)
#define  BACH_WR_CTRL_LATCH	BIT(12)
#define  BACH_WR_CTRL_ADDR_LO	GENMASK(11, 0)
#define BACH_WR_ADDR_HI		BACH(0, 0x94)
#define  BACH_WR_ADDR_HI_MASK	GENMASK(14, 0)
#define BACH_WR_SIZE		BACH(0, 0x96)
#define BACH_WR_COUNT		BACH(0, 0x98)
#define BACH_WR_THRESHOLD	BACH(0, 0x9a)
#define BACH_WR_LEVEL		BACH(0, 0x9e)
#define BACH_WR_MONO		BACH(0, 0xee)
#define  BACH_WR_MONO_EN	BIT(14)

/* Memory addresses and sizes are in 8-byte units */
#define BACH_UNIT		8

/* Analog front end */
#define ATOP_PWR		0x0c
#define  ATOP_PWR_COMMON	0x002d
#define  ATOP_PWR_ADC		0x0180
#define  ATOP_PWR_REF		0x1800
#define ATOP_ADC_MUX		0x14
#define  ATOP_ADC_MUX_MIC	0x0077
#define ATOP_ADC_GAIN		0x18
#define  ATOP_ADC_GAIN_MASK	GENMASK(2, 0)
#define ATOP_GAIN		0x20
#define  ATOP_GAIN_ADC_BOOST	BIT(0)
#define  ATOP_GAIN_MIC_MASK	GENMASK(3, 2)
#define  ATOP_GAIN_MIC_LOW	BIT(12)

/*
 * Gain steps in rising order as the vendor driver programs them: the
 * microphone amplifier field with its low range bit, the ADC field with
 * its boost bit.
 */
#define MIC_GAIN_STEPS		6
#define ADC_GAIN_STEPS		10
static const u8 sstar_bach_adc_gain[ADC_GAIN_STEPS] = {
	1, 2, 0, 3, 4, 5, 6, 7, 6 | 0x80, 7 | 0x80,
};

#define BACH_BUFFER_BYTES	SZ_64K

struct sstar_bach {
	struct device *dev;
	void __iomem *base;
	void __iomem *atop;
	struct clk *clk;
	/* Protects the writer state against the interrupt */
	spinlock_t lock;
	struct snd_pcm_substream *substream;
	unsigned int period_units;
	unsigned int buffer_units;
	/* Position of the oldest unconsumed data, in units */
	unsigned int pos;
	unsigned int mic_gain;
	unsigned int adc_gain;
};

struct sstar_bach_reg {
	u16 reg;
	u16 val;
};

/*
 * Values the vendor driver programs for the capture path. The meaning of
 * most of them is not known.
 */
static const struct sstar_bach_reg sstar_bach_init[] = {
	{ BACH(0, 0x00), 0x89ff },
	{ BACH(0, 0x02), 0x3088 },
	{ BACH(0, 0x06), 0x19b4 },
	{ BACH(0, 0x08), 0xf000 },
	{ BACH(0, 0x0a), 0x8000 },
	{ BACH(0, 0x0c), 0xc09a },
	{ BACH(0, 0x0e), 0x555a },
	{ BACH(0, 0x12), 0x0209 },
	{ BACH(0, 0x16), 0x007d },
	{ BACH(0, 0x26), 0xd400 },
	{ BACH(0, 0x40), 0x0037 },
	{ BACH(0, 0x48), 0x0037 },
	{ BACH(0, 0x50), 0x0037 },
	{ BACH(0, 0x60), 0x0037 },
	{ BACH(0, 0x80), 0x0094 },
	{ BACH(1, 0x0e), 0x0002 },
	{ BACH(1, 0x1c), 0x0003 },
	{ BACH(1, 0x2c), 0x0003 },
	{ BACH(1, 0x3a), 0x4002 },
	{ BACH(1, 0x3c), 0x0104 },
	{ BACH(1, 0x70), 0x0037 },
	{ BACH(1, 0x78), 0x111c },
	/* Writer with two channels, the microphone ADC on the left */
	{ BACH(1, 0x96), 0x0d47 },
};

static const struct sstar_bach_reg sstar_bach_atop_init[] = {
	{ 0x00, 0x0a14 },
	{ 0x04, 0x0030 },
	{ 0x08, 0x0080 },
	{ ATOP_ADC_MUX, ATOP_ADC_MUX_MIC },
	{ ATOP_GAIN, 0x2000 },
};

static u16 bach_readw(struct sstar_bach *bach, unsigned int reg)
{
	return readw(bach->base + reg);
}

static void bach_writew(struct sstar_bach *bach, u16 val, unsigned int reg)
{
	writew(val, bach->base + reg);
}

static void bach_update(struct sstar_bach *bach, unsigned int reg, u16 mask,
			u16 val)
{
	bach_writew(bach, (bach_readw(bach, reg) & ~mask) | (val & mask), reg);
}

static unsigned int sstar_bach_level(struct sstar_bach *bach)
{
	unsigned int level;

	bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_LATCH, BACH_WR_CTRL_LATCH);
	level = bach_readw(bach, BACH_WR_LEVEL);
	bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_LATCH, 0);

	return level;
}

/* Hands units back to the writer, a toggle of the trigger bit applies it */
static void sstar_bach_consume(struct sstar_bach *bach, unsigned int units)
{
	bach_writew(bach, units, BACH_WR_COUNT);
	bach_writew(bach, bach_readw(bach, BACH_WR_CTRL) ^ BACH_WR_CTRL_TRIG,
		    BACH_WR_CTRL);
}

static void sstar_bach_clear_int(struct sstar_bach *bach)
{
	bach_update(bach, BACH_DMA_INT, BACH_DMA_INT_WR_CLR,
		    BACH_DMA_INT_WR_CLR);
	bach_update(bach, BACH_DMA_INT, BACH_DMA_INT_WR_CLR, 0);
}

static irqreturn_t sstar_bach_irq(int irq, void *data)
{
	struct sstar_bach *bach = data;
	struct snd_pcm_substream *substream;
	unsigned int periods = 0;

	spin_lock(&bach->lock);

	substream = bach->substream;
	if (substream) {
		while (sstar_bach_level(bach) >= bach->period_units) {
			sstar_bach_consume(bach, bach->period_units);
			bach->pos = (bach->pos + bach->period_units) %
				    bach->buffer_units;
			periods++;
		}
	}

	sstar_bach_clear_int(bach);

	spin_unlock(&bach->lock);

	if (periods)
		snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

static void atop_update(struct sstar_bach *bach, unsigned int reg, u16 mask,
			u16 val)
{
	writew((readw(bach->atop + reg) & ~mask) | (val & mask),
	       bach->atop + reg);
}

static void sstar_bach_set_mic_gain(struct sstar_bach *bach, unsigned int step)
{
	u16 val;

	if (step < 2)
		val = FIELD_PREP(ATOP_GAIN_MIC_MASK, step) | ATOP_GAIN_MIC_LOW;
	else
		val = FIELD_PREP(ATOP_GAIN_MIC_MASK, step - 2);

	atop_update(bach, ATOP_GAIN, ATOP_GAIN_MIC_MASK | ATOP_GAIN_MIC_LOW,
		    val);
	bach->mic_gain = step;
}

static void sstar_bach_set_adc_gain(struct sstar_bach *bach, unsigned int step)
{
	u8 val = sstar_bach_adc_gain[step];

	atop_update(bach, ATOP_ADC_GAIN, ATOP_ADC_GAIN_MASK, val);
	atop_update(bach, ATOP_GAIN, ATOP_GAIN_ADC_BOOST,
		    val & 0x80 ? ATOP_GAIN_ADC_BOOST : 0);
	bach->adc_gain = step;
}

static int sstar_bach_gain_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = kcontrol->private_value - 1;

	return 0;
}

static struct sstar_bach *sstar_bach_from_kcontrol(struct snd_kcontrol *kcontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);

	return snd_soc_card_get_drvdata(component->card);
}

static int sstar_bach_gain_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct sstar_bach *bach = sstar_bach_from_kcontrol(kcontrol);

	ucontrol->value.integer.value[0] =
		kcontrol->private_value == MIC_GAIN_STEPS ? bach->mic_gain :
							    bach->adc_gain;

	return 0;
}

static int sstar_bach_gain_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct sstar_bach *bach = sstar_bach_from_kcontrol(kcontrol);
	long step = ucontrol->value.integer.value[0];
	bool mic = kcontrol->private_value == MIC_GAIN_STEPS;

	if (step < 0 || step >= kcontrol->private_value)
		return -EINVAL;

	if (step == (mic ? bach->mic_gain : bach->adc_gain))
		return 0;

	if (mic)
		sstar_bach_set_mic_gain(bach, step);
	else
		sstar_bach_set_adc_gain(bach, step);

	return 1;
}

#define SSTAR_BACH_GAIN(_name, _steps)				\
	{							\
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,		\
		.name = _name,					\
		.info = sstar_bach_gain_info,			\
		.get = sstar_bach_gain_get,			\
		.put = sstar_bach_gain_put,			\
		.private_value = _steps,			\
	}

static const struct snd_kcontrol_new sstar_bach_controls[] = {
	SSTAR_BACH_GAIN("Mic Boost Volume", MIC_GAIN_STEPS),
	SSTAR_BACH_GAIN("ADC Capture Volume", ADC_GAIN_STEPS),
};

static const struct snd_pcm_hardware sstar_bach_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = BACH_BUFFER_BYTES,
	.period_bytes_min = 256,
	.period_bytes_max = BACH_BUFFER_BYTES / 2,
	.periods_min = 2,
	.periods_max = BACH_BUFFER_BYTES / 256,
};

static struct sstar_bach *sstar_bach_from(struct snd_soc_component *component)
{
	return snd_soc_card_get_drvdata(component->card);
}

static int sstar_bach_pcm_open(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	int ret;

	snd_soc_set_runtime_hwparams(substream, &sstar_bach_pcm_hw);

	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 BACH_UNIT);
	if (ret)
		return ret;

	return snd_pcm_hw_constraint_integer(substream->runtime,
					     SNDRV_PCM_HW_PARAM_PERIODS);
}

static int sstar_bach_pcm_hw_params(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct sstar_bach *bach = sstar_bach_from(component);
	u16 sr;

	switch (params_rate(params)) {
	case 8000:
		sr = 0x0;
		break;
	case 16000:
		sr = 0x5;
		break;
	case 32000:
		sr = 0xa;
		break;
	case 48000:
		sr = 0xf;
		break;
	default:
		return -EINVAL;
	}

	bach_update(bach, BACH_SR, BACH_SR_WR, FIELD_PREP(BACH_SR_WR, sr));

	return 0;
}

static int sstar_bach_pcm_prepare(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct sstar_bach *bach = sstar_bach_from(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int addr = runtime->dma_addr / BACH_UNIT;
	unsigned long flags;

	spin_lock_irqsave(&bach->lock, flags);

	bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_EN | BACH_WR_CTRL_TRIG, 0);
	bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_RESET, BACH_WR_CTRL_RESET);
	bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_RESET, 0);

	bach->period_units = snd_pcm_lib_period_bytes(substream) / BACH_UNIT;
	bach->buffer_units = snd_pcm_lib_buffer_bytes(substream) / BACH_UNIT;
	bach->pos = 0;

	bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_ADDR_LO,
		    FIELD_PREP(BACH_WR_CTRL_ADDR_LO, addr));
	bach_writew(bach, FIELD_PREP(BACH_WR_ADDR_HI_MASK, addr >> 12),
		    BACH_WR_ADDR_HI);
	bach_writew(bach, bach->buffer_units, BACH_WR_SIZE);
	bach_writew(bach, bach->period_units, BACH_WR_THRESHOLD);
	bach_update(bach, BACH_WR_MONO, BACH_WR_MONO_EN, 0);

	spin_unlock_irqrestore(&bach->lock, flags);

	return 0;
}

static int sstar_bach_pcm_trigger(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream, int cmd)
{
	struct sstar_bach *bach = sstar_bach_from(component);
	u16 int_en = BACH_DMA_INT_WR_EN0 | BACH_DMA_INT_WR_EN1;
	unsigned long flags;

	spin_lock_irqsave(&bach->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		bach->substream = substream;
		sstar_bach_clear_int(bach);
		bach_update(bach, BACH_DMA_INT, int_en, int_en);
		bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_EN,
			    BACH_WR_CTRL_EN);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		bach_update(bach, BACH_WR_CTRL, BACH_WR_CTRL_EN, 0);
		bach_update(bach, BACH_DMA_INT, int_en, 0);
		bach->substream = NULL;
		break;
	default:
		spin_unlock_irqrestore(&bach->lock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&bach->lock, flags);

	return 0;
}

static snd_pcm_uframes_t
sstar_bach_pcm_pointer(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream)
{
	struct sstar_bach *bach = sstar_bach_from(component);

	return bytes_to_frames(substream->runtime, bach->pos * BACH_UNIT);
}

static int sstar_bach_pcm_new(struct snd_soc_component *component,
			      struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       component->dev, BACH_BUFFER_BYTES,
				       BACH_BUFFER_BYTES);

	return 0;
}

static void sstar_bach_hw_init(struct sstar_bach *bach)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sstar_bach_init); i++)
		bach_writew(bach, sstar_bach_init[i].val, sstar_bach_init[i].reg);

	for (i = 0; i < ARRAY_SIZE(sstar_bach_atop_init); i++)
		writew(sstar_bach_atop_init[i].val,
		       bach->atop + sstar_bach_atop_init[i].reg);

	/* Gains used by the vendor firmware */
	sstar_bach_set_mic_gain(bach, MIC_GAIN_STEPS - 1);
	sstar_bach_set_adc_gain(bach, 3);

	/* Power up the reference, the common analog part and the ADC */
	writew(readw(bach->atop + ATOP_PWR) &
	       ~(ATOP_PWR_REF | ATOP_PWR_COMMON | ATOP_PWR_ADC),
	       bach->atop + ATOP_PWR);
	msleep(50);
}

static const struct snd_soc_component_driver sstar_bach_component = {
	.name		= "sstar-bach",
	.open		= sstar_bach_pcm_open,
	.hw_params	= sstar_bach_pcm_hw_params,
	.prepare	= sstar_bach_pcm_prepare,
	.trigger	= sstar_bach_pcm_trigger,
	.pointer	= sstar_bach_pcm_pointer,
	.pcm_new	= sstar_bach_pcm_new,
	.controls	= sstar_bach_controls,
	.num_controls	= ARRAY_SIZE(sstar_bach_controls),
};

static struct snd_soc_dai_driver sstar_bach_dai = {
	.name = "sstar-bach",
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				  SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
};

SND_SOC_DAILINK_DEFS(sstar_bach,
	DAILINK_COMP_ARRAY(COMP_CPU("sstar-bach")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static int sstar_bach_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	struct snd_soc_card *card;
	struct sstar_bach *bach;
	int irq, ret;

	bach = devm_kzalloc_obj(dev, *bach);
	card = devm_kzalloc_obj(dev, *card);
	link = devm_kzalloc_obj(dev, *link);
	if (!bach || !card || !link)
		return -ENOMEM;

	bach->dev = dev;
	spin_lock_init(&bach->lock);

	bach->base = devm_platform_ioremap_resource_byname(pdev, "bach");
	if (IS_ERR(bach->base))
		return PTR_ERR(bach->base);

	bach->atop = devm_platform_ioremap_resource_byname(pdev, "atop");
	if (IS_ERR(bach->atop))
		return PTR_ERR(bach->atop);

	bach->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(bach->clk))
		return dev_err_probe(dev, PTR_ERR(bach->clk),
				     "failed to get clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, sstar_bach_irq, 0, dev_name(dev),
			       bach);
	if (ret)
		return ret;

	sstar_bach_hw_init(bach);

	ret = devm_snd_soc_register_component(dev, &sstar_bach_component,
					      &sstar_bach_dai, 1);
	if (ret)
		return ret;

	link->name = "sstar-bach";
	link->stream_name = "Capture";
	link->cpus = sstar_bach_cpus;
	link->num_cpus = ARRAY_SIZE(sstar_bach_cpus);
	link->codecs = sstar_bach_codecs;
	link->num_codecs = ARRAY_SIZE(sstar_bach_codecs);
	link->platforms = sstar_bach_platforms;
	link->num_platforms = ARRAY_SIZE(sstar_bach_platforms);
	link->cpus[0].dai_name = "sstar-bach";
	link->cpus[0].of_node = dev->of_node;
	link->platforms[0].of_node = dev->of_node;
	link->capture_only = 1;

	card->dev = dev;
	card->owner = THIS_MODULE;
	card->name = "SSC338Q";
	card->dai_link = link;
	card->num_links = 1;
	snd_soc_card_set_drvdata(card, bach);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id sstar_bach_of_match[] = {
	{ .compatible = "sstar,ssc338q-bach" },
	{ }
};
MODULE_DEVICE_TABLE(of, sstar_bach_of_match);

static struct platform_driver sstar_bach_driver = {
	.driver = {
		.name = "sstar-bach",
		.of_match_table = sstar_bach_of_match,
	},
	.probe = sstar_bach_probe,
};
module_platform_driver(sstar_bach_driver);

MODULE_DESCRIPTION("SigmaStar SSC338Q audio capture driver");
MODULE_AUTHOR("Petr Kracik <petrkr@petrkr.net>");
MODULE_LICENSE("GPL");
