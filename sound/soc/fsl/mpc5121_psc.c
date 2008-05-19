/*
 * Freescale MPC5121 PSC ALSA SoC Digital Audio Interface (DAI) driver
 *
 * Copyright 2008 Freescale Semiconductor, Inc.
 * Author: John Rigby <jrigby@freescale.com>
 *
 * Based on
 *     fsl_ssi.c -- Author: Timur Tabi <timur@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/mpc52xx_psc.h>
#include <asm/mpc512x.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "mpc5121_psc_info.h"

/*
 * Three scenarios:
 * 	AC97: PSC is slave so claim support for all speeds and let codec
 * 	determine the rate.
 * 	ICS Slave: PSC is slave so identical above.
 * 	ICS Master: PSC is master which can do any speed
 * 	
 * 	So for all three modes 8000-48000 continuous is ok
 */
#define PSC_SAMPLE_RATES SNDRV_PCM_RATE_8000_48000

/*
 * AC97 sample width is upto 20 bits, the next unpacked size if 32.
 * I2S works for 8, 16 and 32
 */
#define PSC_AC97_FORMATS SNDRV_PCM_FMTBIT_S32_BE
#define PSC_I2S_FORMATS ( 	\
    SNDRV_PCM_FMTBIT_S32_BE |	\
    SNDRV_PCM_FMTBIT_S16_BE | 	\
    SNDRV_PCM_FMTBIT_S8 | 	\
    0)

struct {
	int tot;
	int al;
	int em;
	int ur;
	int or;
	int me;
	int update;
} pscstats;

/**
 * mpc5121_psc_isr: PSC interrupt handler
 *
 * All data transfer is done via dma.  This handler is just for errors.
 *
 * @irq: IRQ of the PSC device
 * @dev_id: pointer to the psc_private structure for this PSC device
 */
static irqreturn_t mpc5121_psc_isr(int irq, void *dev_id)
{
	struct mpc5121_psc_private *psc_private = dev_id;
	irqreturn_t ret = IRQ_NONE;
	struct mpc512x_psc_fifo *fifo;
	u32 isr;

	fifo = (struct mpc512x_psc_fifo *)
	    (psc_private->psc + sizeof(struct mpc52xx_psc));

	isr = in_be32(&fifo->rxisr);
	out_be32(&fifo->rxisr, isr);

	isr = in_be32(&fifo->txisr);
	out_be32(&fifo->txisr, isr);

	if (isr & MPC512x_PSC_FIFO_ALARM)
		pscstats.al++;
	if (isr & MPC512x_PSC_FIFO_URERR)
		pscstats.ur++;
	if (isr & MPC512x_PSC_FIFO_ORERR)
		pscstats.or++;
	if (isr & MPC512x_PSC_FIFO_MEMERROR)
		pscstats.me++;
	if (isr & MPC512x_PSC_FIFO_EMPTY)
		pscstats.em++;

	if (isr) {
		pscstats.tot++;
		pscstats.update++;
		ret = IRQ_HANDLED;
	}

#ifdef DEBUG
	if (pscstats.update > 10000) {
		pscstats.update = 0;
		printk(KERN_ERR "al %d ur %d or %d me %d em %d tot %d\n",
			pscstats.al,
			pscstats.ur,
			pscstats.or,
			pscstats.me,
			pscstats.em,
			pscstats.tot);
	}
#endif

	return ret;
}

int mpc5121_psc_clkinit(struct mpc5121_psc_private *psc_private, int on)
{
	/* when adding master this will need to do more */
	char clockname[256];
	struct clk *clk;

	sprintf(clockname, "psc%d_mclk", psc_private->cpu_dai.id);

	clk = psc_private->clk = clk_get(NULL, clockname);
	if (IS_ERR(psc_private->clk)) {
		printk(KERN_ERR "%s: can't probe clock"
			" source for PSC SOC.\n", __FUNCTION__);
		psc_private->clk = NULL;
		return PTR_ERR(clk);
	}

	if (on)
		clk_enable(clk);
	else
		clk_disable(clk);

	clk_put(clk);

	return 0;
}

void mpc5121_psc_fifo_init(struct mpc5121_psc_private *psc_private)
{
	struct mpc512x_psc_fifo *fifo;
	unsigned long size;

	fifo = (struct mpc512x_psc_fifo *)
	    (psc_private->psc + sizeof(struct mpc52xx_psc));

	out_be32(&fifo->rxcmd, MPC512x_PSC_FIFO_RESET_SLICE);
	out_be32(&fifo->txcmd, MPC512x_PSC_FIFO_RESET_SLICE);

	/*
	 * Make sure that dma granularity does not
	 * exceed the fifo size.
	 */
	size = in_be32(&fifo->rxsz) * 4;
	if (size < psc_private->rx_dma_gran)
		printk(KERN_WARNING "rx dma granularity exceeds fifo size\n");
	out_be32(&fifo->rxalarm, size - psc_private->rx_dma_gran);

	size = in_be32(&fifo->txsz) * 4;
	if (size < psc_private->tx_dma_gran)
		printk(KERN_WARNING "tx dma granularity exceeds fifo size\n");
	out_be32(&fifo->txalarm, size - psc_private->tx_dma_gran);
}

void mpc5121_psc_fifo_enable(struct mpc5121_psc_private *psc_private)
{
	struct mpc512x_psc_fifo *fifo;

	fifo = (struct mpc512x_psc_fifo *)
	    (psc_private->psc + sizeof(struct mpc52xx_psc));

	out_be32(&fifo->rxcmd,
		 MPC512x_PSC_FIFO_ENABLE_SLICE | MPC512x_PSC_FIFO_ENABLE_DMA);
	out_be32(&fifo->txcmd,
		 MPC512x_PSC_FIFO_ENABLE_SLICE | MPC512x_PSC_FIFO_ENABLE_DMA);
}

int mpc5121_psc_init(struct device *dev, struct snd_soc_cpu_dai *cpu_dai)
{
	struct mpc5121_psc_private *psc_private = cpu_dai->private_data;
	int err;

	/*
	 * If this is the first stream opened, then request the IRQ
	 * and initialize the PSC registers.
	 */
	if (!psc_private->playback && !psc_private->capture) {
		struct mpc52xx_psc __iomem *psc = psc_private->psc;

		err = mpc5121_psc_clkinit(psc_private, 1);
		if (err < 0) {
			dev_err(dev, "could not initialize psc clk\n");
			goto noclock;
		}

		/* disable */
		out_8(&psc->command,
		      MPC52xx_PSC_TX_DISABLE | MPC52xx_PSC_RX_DISABLE);

		mpc5121_psc_fifo_init(psc_private);

		/* reset everything */
		out_8(&psc->command, MPC52xx_PSC_SEL_MODE_REG_1);
		out_8(&psc->command, MPC52xx_PSC_RST_RX);
		out_8(&psc->command, MPC52xx_PSC_RST_TX);
		out_8(&psc->command, MPC52xx_PSC_RST_ERR_STAT);
		out_8(&psc->command, MPC52xx_PSC_RST_BRK_CHG_INT);
		out_8(&psc->command, MPC52xx_PSC_STOP_BRK);

		switch(psc_private->format) {
		case SND_SOC_DAIFMT_AC97:
			/*
			 * set up the psc for AC97 mode
			 */
			out_be32(&psc->sicr,
				0x03000000 | /* SIM = 0011   : AC97 mode */
				0x00010000 | /* EnAC97 = 1   : Normal mode */
				0x00000100 | /* Outputs always enabled */
				0);

			out_be32(&psc->ac97slots,
				0x300 << 16  | /* Enable tx timeslot 3,4 */
				0x300	     | /* Enable rx timeslot 3,4 */
				0);

			/*
			 * Reset external AC97 codec
			 * Some codecs go into test mode if the data or sync lines
			 * are high when the reset line goes high.
			 * Avoid that by forcing them to GPIOs and driving them
			 * low during reset.
			 */
			mpc5121_pscgpio_make_gpio(cpu_dai->id, 1); 
			mpc5121_pscgpio_pin_low(cpu_dai->id, 1); 
			mpc5121_pscgpio_make_gpio(cpu_dai->id, 2); 
			mpc5121_pscgpio_pin_low(cpu_dai->id, 2); 

			out_8(&psc->op1, 0x02); iosync();
			udelay(1);
			out_8(&psc->op0, 0x02); iosync();
			udelay(1);

			/*
			 * Reset complete, change lines back to PSC signals.
			 */
			mpc5121_pscgpio_make_psc(cpu_dai->id, 1); 
			mpc5121_pscgpio_make_psc(cpu_dai->id, 2); 
			break;
		case SND_SOC_DAIFMT_I2S:
			out_be32(&psc->sicr,
				0x20000000 | /* DTS = 1      : Delay 1 bit time (I2S) */
				0x0f000000 | /* SIM = 1111   : Codec 32 bit */
				0x00400000 | /* I2S = 1      : I2S */
				0x00200000 | /* CLKPOL = 1   : */
				0x00000100 | /* Outputs always enabled */
				0);
			break;
		}

		/* enable the fifos */
		mpc5121_psc_fifo_enable(psc_private);

		/* enable rx and tx now */
		out_8(&psc->command, MPC52xx_PSC_TX_ENABLE |
			MPC52xx_PSC_RX_ENABLE);
	}

	return 0;
noclock:
	return err;
}
EXPORT_SYMBOL_GPL(mpc5121_psc_init);

/**
 * mpc5121_psc_startup: create a new substream
 *
 * This is the first function called when a stream is opened.
 *
 * If this is the first stream open, then grab the IRQ and program most of
 * the PSC registers.
 */
static int mpc5121_psc_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mpc5121_psc_private *psc_private =
	    rtd->dai->cpu_dai->private_data;

	/*
	 * If this is the first stream opened, then request the IRQ
	 * and initialize the PSC registers.
	 */
	if (!psc_private->playback && !psc_private->capture) {
		int ret;
		ret = request_irq(psc_private->irq, mpc5121_psc_isr,
				  IRQF_SHARED,
				  psc_private->name, psc_private);
		if (ret < 0) {
			dev_err(substream->pcm->card->dev,
				"could not claim irq %u\n", psc_private->irq);
			return ret;
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		psc_private->playback++;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		psc_private->capture++;

	return 0;
}

/**
 * mpc5121_psc_prepare: prepare the PSC.
 */
static int mpc5121_psc_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mpc5121_psc_private *psc_private =
	    rtd->dai->cpu_dai->private_data;
	struct mpc52xx_psc __iomem *psc = psc_private->psc;
	int width;
	int sicr;
	int bclkdiv;
	int ccr;

	switch(psc_private->format) {
	case SND_SOC_DAIFMT_AC97:
		/* nothing to do, original init is good */
		break;
	case SND_SOC_DAIFMT_I2S:
		/* reset everything */
		out_8(&psc->command, MPC52xx_PSC_SEL_MODE_REG_1);
		out_8(&psc->command, MPC52xx_PSC_RST_RX);
		out_8(&psc->command, MPC52xx_PSC_RST_TX);
		out_8(&psc->command, MPC52xx_PSC_RST_ERR_STAT);
		out_8(&psc->command, MPC52xx_PSC_STOP_BRK);

		/* format */
		sicr = 0x20000000  | /* DTS = 1      : Delay 1 bit time (I2S) */
			(psc_private->clk_dir == SND_SOC_CLOCK_OUT ?
			0x00800000 : /* GenClk = 1   : master */
			0x00000000)| /* GenClk = 0   : slave */
			0x00400000 | /* I2S = 1      : I2S */
			0x00200000 | /* CLKPOL = 1   : */
			0x00000100 | /* Outputs always enabled */
			0;

		width = snd_pcm_format_width(runtime->format);
		switch (width) {
		case 8:
			sicr |= 0x01000000;
			break;
		case 16:
			sicr |= 0x02000000;
			break;
		case 32:
			sicr |= 0x0f000000;
			break;
		}
		out_be32(&psc->sicr, sicr);


		/* rate based on 64 bit clks per frame */
		bclkdiv = psc_private->clk_rate / (runtime->rate * 64) - 1;

		ccr = ((64-1) << 24)
		    | ((bclkdiv & 0xff) << 16)
		    | (((bclkdiv >> 8) & 0xff) << 8);
		out_be32(&psc->ccr, ccr);
		out_8(&psc->ctur, (64/2)-1);

		out_8(&psc->command, MPC52xx_PSC_TX_ENABLE | MPC52xx_PSC_RX_ENABLE);
		break;
	}

	return 0;
}

/**
 * mpc5121_psc_trigger: start and stop the DMA transfer.
 *
 * This function is called by ALSA to start, stop, pause, and resume the DMA
 * transfer of data.
 *
 * The DMA channel is in external master start and pause mode, which
 * means the PSC completely controls the flow of data.
 */
static int mpc5121_psc_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mpc5121_psc_private *psc_private =
					rtd->dai->cpu_dai->private_data;
	struct mpc512x_psc_fifo *fifo;

	fifo = (struct mpc512x_psc_fifo *)
	    (psc_private->psc + sizeof(struct mpc52xx_psc));
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			out_be32(&fifo->tximr,
				MPC512x_PSC_FIFO_MEMERROR |
				MPC512x_PSC_FIFO_ORERR |
				MPC512x_PSC_FIFO_URERR);
		else
			out_be32(&fifo->rximr,
				MPC512x_PSC_FIFO_MEMERROR |
				MPC512x_PSC_FIFO_ORERR |
				MPC512x_PSC_FIFO_URERR);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			out_be32(&fifo->tximr, 0);
		else
			out_be32(&fifo->rximr, 0);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * mpc5121_psc_shutdown: shutdown the PSC
 *
 * Shutdown the PSC if there are no other substreams open.
 */
static void mpc5121_psc_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mpc5121_psc_private *psc_private =
	    rtd->dai->cpu_dai->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		psc_private->playback--;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		psc_private->capture--;

	/*
	 * If this is the last active substream, disable the PSC and release
	 * the IRQ.
	 */
	if (!psc_private->playback && !psc_private->capture) {
		void __iomem *psc = psc_private->psc;

		(void)psc;
		printk(KERN_INFO "psc shutting down %p\n", psc);

		free_irq(psc_private->irq, psc_private);
	}
}

/**
 * mpc5121_psc_set_sysclk: set the clock frequency and direction
 *
 * This function is called by the machine driver to tell us what the clock
 * frequency and direction are.
 *
 * When runniing as a clock slave (SND_SOC_CLOCK_IN) then we don't care about
 * the rate.  
 *
 * When running as a clock master (SND_SOC_CLOCK_OUT) then we use the clock
 * rate obtained from the clock driver via clk_get_rate.
 *
 * @clk_id: reserved, should be zero
 * @freq: the frequency of the given clock ID, currently ignored
 * @dir: SND_SOC_CLOCK_IN (clock slave) or SND_SOC_CLOCK_OUT (clock master)
 */
static int mpc5121_psc_set_sysclk(struct snd_soc_cpu_dai *mpc5121_psc_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct mpc5121_psc_private *psc_private =
	    container_of(mpc5121_psc_dai, struct mpc5121_psc_private, cpu_dai);
	
	psc_private->clk_dir = dir;
	return 0;
}

/**
 * mpc5121_psc_set_fmt: set the serial format.
 *
 * This function is called by the machine driver to tell us what serial
 * format to use.
 *
 * Currently AC97 and I2S are supported
 *
 * @format: one of SND_SOC_DAIFMT_xxx
 */
static int mpc5121_psc_set_fmt(struct snd_soc_cpu_dai *mpc5121_psc_dai,
			       unsigned int format)
{
	struct mpc5121_psc_private *psc_private =
	    container_of(mpc5121_psc_dai, struct mpc5121_psc_private, cpu_dai);

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	    case SND_SOC_DAIFMT_AC97:
	    case SND_SOC_DAIFMT_I2S:
		break;
	    default:
		return -EINVAL;
	}
	psc_private->format = format;
	return 0;
}

/**
 * mpc5121_psc_dai_template: template CPU DAI for the PSC
 */
static struct snd_soc_cpu_dai mpc5121_psc_dai_template = {
	.playback = {
		     /* The PSC does not support monaural audio. */
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = PSC_SAMPLE_RATES,
		     .formats = PSC_AC97_FORMATS,
		     },
	.capture = {
		    .channels_min = 2,
		    .channels_max = 2,
		    .rates = PSC_SAMPLE_RATES,
		    .formats = PSC_AC97_FORMATS,
		    },
	.ops = {
		.startup = mpc5121_psc_startup,
		.prepare = mpc5121_psc_prepare,
		.shutdown = mpc5121_psc_shutdown,
		.trigger = mpc5121_psc_trigger,
		},
	.dai_ops = {
		    .set_sysclk = mpc5121_psc_set_sysclk,
		    .set_fmt = mpc5121_psc_set_fmt,
		    },
};

/**
 * mpc5121_psc_create_dai: create a snd_soc_cpu_dai structure
 *
 * This function is called by the machine driver to create a snd_soc_cpu_dai
 * structure.  The function creates an psc_private object, which contains
 * the snd_soc_cpu_dai.  It also creates the sysfs statistics device.
 */
struct snd_soc_cpu_dai *mpc5121_psc_create_dai(struct mpc5121_psc_info
					       *psc_info)
{
	struct snd_soc_cpu_dai *mpc5121_psc_dai;
	struct mpc5121_psc_private *psc_private;
	struct mpc5121_ads_data *ads_data = container_of(psc_info,
			struct mpc5121_ads_data, psc_info);

	psc_private = kzalloc(sizeof(struct mpc5121_psc_private), GFP_KERNEL);
	if (!psc_private) {
		dev_err(psc_info->dev, "could not allocate DAI object\n");
		return NULL;
	}
	memcpy(&psc_private->cpu_dai, &mpc5121_psc_dai_template,
	       sizeof(struct snd_soc_cpu_dai));

	if (ads_data->dai_format == SND_SOC_DAIFMT_I2S) {
		psc_private->cpu_dai.playback.formats = PSC_I2S_FORMATS;
		psc_private->cpu_dai.capture.formats = PSC_I2S_FORMATS;
	}

	mpc5121_psc_dai = &psc_private->cpu_dai;

	sprintf(psc_private->name, "psc%u", (u8) psc_info->id);
	psc_private->psc = psc_info->psc;
	psc_private->phys = psc_info->phys;
	psc_private->irq = psc_info->irq;
	psc_private->dev = psc_info->dev;
	psc_private->rx_dma_gran = psc_info->rx_dma_gran;
	psc_private->tx_dma_gran = psc_info->tx_dma_gran;

	psc_private->dev->driver_data = mpc5121_psc_dai;
	psc_private->format = ads_data->dai_format;

	mpc5121_psc_dai->private_data = psc_private;
	mpc5121_psc_dai->name = psc_private->name;
	mpc5121_psc_dai->id = psc_info->id;

	return mpc5121_psc_dai;
}
EXPORT_SYMBOL_GPL(mpc5121_psc_create_dai);

/**
 * mpc5121_psc_destroy_dai: destroy the snd_soc_cpu_dai object
 *
 * This function undoes the operations of mpc5121_psc_create_dai()
 */
void mpc5121_psc_destroy_dai(struct snd_soc_cpu_dai *mpc5121_psc_dai)
{
	struct mpc5121_psc_private *psc_private =
	    container_of(mpc5121_psc_dai, struct mpc5121_psc_private, cpu_dai);

	kfree(psc_private);
}
EXPORT_SYMBOL_GPL(mpc5121_psc_destroy_dai);

MODULE_AUTHOR("John Rigby <jrigby@freescale.com>");
MODULE_DESCRIPTION("Freescale psc ASoC Driver");
MODULE_LICENSE("GPL");
