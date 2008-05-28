/*
 * Freescale MPC5121 ALSA SoC PCM driver
 * Copyright 2007,2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: John Rigby <jrigby@freescale.com>
 *
 *
 *  Originally copied from sound/mpc5121/mpc5121-pcm.c
 *  Freescale AC97 device driver for CPU MPC5121
 *  Author: Hongjun Chen <hong-jun.chen@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/fsldma.h>
#include <asm/mpc52xx_psc.h>

#include "mpc5121_pcm.h"
#include "mpc5121_psc_info.h"

#define BPF             4	/* bytes per frame */
#define SLOTS           2
#define SAMPLESIZE      (BPF*SLOTS)

#define FPP             (4096 / 2)	/* frames per period */
#define PERIODS_MAX     8
#define PERIODSIZE      (SAMPLESIZE*FPP)
#define BUFSIZE         (PERIODSIZE*PERIODS_MAX)

struct mpc512x_dma_config pcm_config;

/*
 * Support all these formats, actual supported format will
 * depend on what the soc and codec driver support.
 */
#define MPC5121_PCM_FORMATS (SNDRV_PCM_FMTBIT_S8 	| \
			    SNDRV_PCM_FMTBIT_U8 	| \
			    SNDRV_PCM_FMTBIT_S16_LE     | \
			    SNDRV_PCM_FMTBIT_S16_BE     | \
			    SNDRV_PCM_FMTBIT_U16_LE     | \
			    SNDRV_PCM_FMTBIT_U16_BE     | \
			    SNDRV_PCM_FMTBIT_S24_LE     | \
			    SNDRV_PCM_FMTBIT_S24_BE     | \
			    SNDRV_PCM_FMTBIT_U24_LE     | \
			    SNDRV_PCM_FMTBIT_U24_BE     | \
			    SNDRV_PCM_FMTBIT_S32_LE     | \
			    SNDRV_PCM_FMTBIT_S32_BE     | \
			    SNDRV_PCM_FMTBIT_U32_LE     | \
			    SNDRV_PCM_FMTBIT_U32_BE)

static const struct snd_pcm_hardware mpc512x_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED,
	.formats = MPC5121_PCM_FORMATS,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.period_bytes_min = (FPP * SAMPLESIZE * 1),
	.period_bytes_max = (FPP * SAMPLESIZE * PERIODS_MAX),
	.periods_min = 1,
	.periods_max = PERIODS_MAX,
	.buffer_bytes_max = (FPP * SAMPLESIZE * PERIODS_MAX),
	.fifo_size = 0,
};

struct mpc512x_runtime_data {
	int dma_ch;
	dma_addr_t dev_addr;
	struct fsl_dma_requestbuf dma_desc_array[PERIODS_MAX];
	dma_addr_t dma_desc_array_phys;
	int current_period;
	int stopping;
};

static int mpc512x_pcm_close(struct snd_pcm_substream *substream);
static int period_len;
static int mpc512x_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mpc512x_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	struct fsl_dma_requestbuf *dma_desc;
	dma_addr_t dma_buff_phys;

	period_len = period;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	dma_desc = rtd->dma_desc_array;
	dma_buff_phys = runtime->dma_addr;

	do {
		if (period > totsize)
			period = totsize;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			dma_desc->src = dma_buff_phys;
			dma_desc->dest = rtd->dev_addr;
			dma_desc->minor_loop = pcm_config.tx_dma_gran /
			    sizeof(u32);
			dma_desc->soff = sizeof(u32);
			dma_desc->doff = 0;
		} else {
			dma_desc->src = rtd->dev_addr;
			dma_desc->dest = dma_buff_phys;
			dma_desc->minor_loop = pcm_config.rx_dma_gran /
			    sizeof(u32);
			dma_desc->soff = 0;
			dma_desc->doff = sizeof(u32);
		}
		dma_desc->len = period;
		dma_desc++;
		dma_buff_phys += period;
		memset(dma_desc, 0, sizeof(*dma_desc));
	} while ((totsize -= period) > 0);

	return 0;
}

static int mpc512x_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int mpc512x_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct mpc512x_runtime_data *rtd = substream->runtime->private_data;
	rtd->current_period = 0;
	return 0;
}

static int mpc512x_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mpc512x_runtime_data *rtd = substream->runtime->private_data;
	int ret = 0;
	int dmaerr;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
		rtd->stopping++;
		break;

	case SNDRV_PCM_TRIGGER_START:
		if (rtd->stopping) {
			rtd->stopping = 0;
		} else {
			dmaerr = fsl_dma_config(rtd->dma_ch,
				&rtd->dma_desc_array[rtd->current_period], 1);
			if (dmaerr) {
				printk(KERN_ERR "unexpected error in "
					"mpc512x_pcm_trigger %d\n", -dmaerr);
			} else {
				fsl_dma_enable(rtd->dma_ch);
			}
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	default:
		ret = -EINVAL;
	}
	return ret;
}

static void mpc512x_pcm_dma_callback(void *dev_id, int err_status)
{
	struct snd_pcm_substream *substream = dev_id;
	struct mpc512x_runtime_data *rtd = substream->runtime->private_data;
	int dmaerr;

	if (!err_status) {
		if (rtd->stopping) {
			rtd->stopping = 0;
			return;
		}

		rtd->current_period++;
		rtd->current_period %= substream->runtime->periods;

		dmaerr = fsl_dma_config(rtd->dma_ch,
				&rtd->dma_desc_array[rtd->current_period], 1);
		if (dmaerr)
			printk(KERN_ERR "unexpected error in "
				"mpc512x_pcm_dma_callback %d\n", -dmaerr);
		else
			fsl_dma_enable(rtd->dma_ch);

		snd_pcm_period_elapsed(substream);
	} else {
		printk(KERN_ERR
		       "%s: DMA error on channel %d (Error Status=%#x)\n",
		       __FUNCTION__, rtd->dma_ch, err_status);
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
	}
}

static snd_pcm_uframes_t mpc512x_pcm_pointer(struct snd_pcm_substream
					     *substream)
{
	struct mpc512x_runtime_data *rtd = substream->runtime->private_data;

	return rtd->current_period * substream->runtime->period_size;
}

static int mpc512x_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mpc512x_runtime_data *rtd;
	int dma_ch_nr;
	int ret;

	runtime->hw = mpc512x_pcm_hardware;

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	ret = -ENOMEM;

	rtd = kzalloc(sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		goto err1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->dev_addr = pcm_config.tx_dev_addr;
		dma_ch_nr = pcm_config.tx_dma_ch_nr;
	} else {
		rtd->dev_addr = pcm_config.rx_dev_addr;
		dma_ch_nr = pcm_config.rx_dma_ch_nr;
	}

	/* Allocate DMA channel for this substream */
	ret = fsl_dma_chan_request(dma_ch_nr);
	if (ret < 0) {
		printk(KERN_ERR
		       "Err: can't allocate DMA channel %d for PSC!\n",
		       dma_ch_nr);
		goto err2;
	}
	rtd->dma_ch = ret;
	fsl_dma_callback_set(rtd->dma_ch,
			     mpc512x_pcm_dma_callback, (void *)substream);

	runtime->private_data = rtd;

	return 0;

err2:
	kfree(rtd);
err1:
	return ret;
}

static int mpc512x_pcm_close(struct snd_pcm_substream *substream)
{
	struct mpc512x_runtime_data *rtd = substream->runtime->private_data;

	if (rtd && rtd->dma_ch >= 0)
		fsl_dma_free_chan(rtd->dma_ch);

	kfree(rtd);
	return 0;
}

static struct snd_pcm_ops mpc512x_pcm_ops = {
	.open = mpc512x_pcm_open,
	.close = mpc512x_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mpc512x_pcm_hw_params,
	.hw_free = mpc512x_pcm_hw_free,
	.prepare = mpc512x_pcm_prepare,
	.trigger = mpc512x_pcm_trigger,
	.pointer = mpc512x_pcm_pointer,
};

static int mpc512x_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mpc512x_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
				       &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void mpc512x_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf || !buf->area)
			continue;
		dma_free_coherent(pcm->card->dev, buf->bytes,
				  buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int mpc512x_pcm_new(struct snd_card *card, struct snd_soc_codec_dai *dai,
			   struct snd_pcm *pcm)
{
	int ret = 0;

	if (dai->playback.channels_min) {
		ret =
		    mpc512x_pcm_preallocate_dma_buffer(pcm,
			       SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}
	if (dai->capture.channels_min) {
		ret =
		    mpc512x_pcm_preallocate_dma_buffer(pcm,
			       SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

	ret = 0;

out:
	return ret;
}

#ifdef CONFIG_PM
unsigned short headphone;
static int mpc512x_pcm_suspend(struct platform_device *pdev,
			       struct snd_soc_cpu_dai *dai)
{
	struct mpc5121_psc_private *psc_private = dai->private_data;
	struct mpc52xx_psc *psc = psc_private->psc;
	struct mpc512x_psc_fifo *fifo;

	fifo = (struct mpc512x_psc_fifo *)
	    (psc_private->psc + sizeof(struct mpc52xx_psc));

#ifdef CONFIG_SND_SOC_MPC5121_ADS
	if (psc_private->format == SND_SOC_DAIFMT_AC97)
		headphone = mpc5121_ac97_read(NULL, 4);
#endif

	/* Disable AC97 controller */
	out_be32(&psc->sicr, 0);

	/* Disable FIFO rx/tx slices,  */
	out_be32(&fifo->rxcmd, MPC512x_PSC_FIFO_RESET_SLICE);
	out_be32(&fifo->txcmd, MPC512x_PSC_FIFO_RESET_SLICE);

	/* Disable clock */
	mpc5121_psc_clkinit(psc_private, 0);

	return 0;
}

static int mpc512x_pcm_resume(struct platform_device *pdev,
			      struct snd_soc_cpu_dai *dai)
{
	struct mpc5121_psc_private *psc_private = dai->private_data;

	mpc5121_psc_init(&pdev->dev, dai);
#ifdef CONFIG_SND_SOC_MPC5121_ADS
	if (psc_private->format == SND_SOC_DAIFMT_AC97)
		mpc5121_ac97_reset(NULL);
	mpc5121_ac97_write(NULL, 4, headphone);
#else
	(void)psc_private;
#endif

	return 0;
}
#else
#define mpc512x_pcm_suspend	NULL
#define mpc512x_pcm_resume	NULL
#endif

struct snd_soc_platform mpc512x_soc_platform = {
	.name = "MPC5121-audio",
	.pcm_ops = &mpc512x_pcm_ops,
	.pcm_new = mpc512x_pcm_new,
	.pcm_free = mpc512x_pcm_free_dma_buffers,
	.suspend = mpc512x_pcm_suspend,
	.resume = mpc512x_pcm_resume,
};
EXPORT_SYMBOL_GPL(mpc512x_soc_platform);

/*
 * pass config info about the PSC driver to the DMA driver
 */
int mpc512x_dma_configure(struct mpc512x_dma_config *config)
{
	static int initialized;

	if (initialized)
		return 0;

	pcm_config = *config;
	initialized = 1;

	return 1;
}
EXPORT_SYMBOL_GPL(mpc512x_dma_configure);

MODULE_AUTHOR("John Rigby <jrigby@freescale.com>");
MODULE_DESCRIPTION("Freescale MPC512x ASoC PCM module");
MODULE_LICENSE("GPL");
