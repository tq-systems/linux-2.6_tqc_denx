/*
 * Freescale MPC5121ADS SoC Fabric driver
 *
 * Copyright 2008 Freescale Semiconductor Inc.
 * Author: John Rigby jrigby@freescale.com
 *
 * Originally based on mpc8610_hpcd.c:
 *    Freescale MPC8610HPCD ALSA SoC Fabric driver
 *    Author: Timur Tabi <timur@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <sound/soc.h>

#include <asm/of_device.h>
#include <asm/of_platform.h>
#include <asm-powerpc/mpc52xx_psc.h>

#include "../codecs/ac97.h"
#include "mpc5121_psc_info.h"
#include "mpc5121_pcm.h"

struct mpc52xx_psc *psc_reg_priv;

/**
 * mpc5121_ads_startup: program the board with various hardware parameters
 *
 * This function takes board-specific information, like clock frequencies
 * and serial data formats, and passes that information to the codec and
 * transport drivers.
 */
static int mpc5121_ads_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct mpc5121_ads_data *machine_data = rtd->socdev->dev->platform_data;
	int ret = 0;

	/* Tell the CPU driver what the serial protocol is. */
	if (cpu_dai->dai_ops.set_fmt) {
		ret = cpu_dai->dai_ops.set_fmt(cpu_dai,
					       machine_data->dai_format);
		if (ret < 0) {
			dev_err(substream->pcm->card->dev,
				"could not set CPU driver audio format\n");
			return ret;
		}
	}

	/* Tell the codec driver what the serial protocol is. */
	if (codec_dai->dai_ops.set_fmt) {
		ret = codec_dai->dai_ops.set_fmt(codec_dai,
						 machine_data->dai_format);
		if (ret < 0) {
			dev_err(substream->pcm->card->dev,
				"could not set codec driver audio format\n");
			return ret;
		}
	}

	/*
	 * Tell the CPU driver what the clock frequency is, and whether it's a
	 * slave or master.
	 */
	if (cpu_dai->dai_ops.set_sysclk &&
		machine_data->dai_format != SND_SOC_DAIFMT_AC97) {
		ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, 0,
						  machine_data->clk_frequency,
						  machine_data->
						  cpu_clk_direction);
		if (ret < 0) {
			dev_err(substream->pcm->card->dev,
				"could not set CPU driver clock parameters\n");
			return ret;
		}
	}

	/*
	 * Tell the codec driver what the MCLK frequency is, and whether it's
	 * a slave or master.
	 */
	if (codec_dai->dai_ops.set_sysclk &&
		machine_data->dai_format != SND_SOC_DAIFMT_AC97) {
		ret = codec_dai->dai_ops.set_sysclk(codec_dai, 0,
						    machine_data->clk_frequency,
						    machine_data->
						    codec_clk_direction);
		if (ret < 0) {
			dev_err(substream->pcm->card->dev,
				"could not set codec driver clock params\n");
			return ret;
		}
	}

	return 0;
}

/**
 * mpc5121_ads_ops: ASoC fabric driver operations
 */
static struct snd_soc_ops mpc5121_ads_ops = {
	.startup = mpc5121_ads_startup,
};

/**
 * mpc5121_ads_machine: ASoC machine data
 */
static struct snd_soc_machine mpc5121_ads_machine = {
	.name = "MPC5121 ADS",
	.num_links = 1,
};

/**
 * mpc5121_ads_probe: OF probe function for the fabric driver
 *
 * This function gets called when fsl,mpc5121-psc-ac97 node
 * is found in the device tree.
 */
static int mpc5121_ads_probe(struct of_device *ofdev,
			     const struct of_device_id *match)
{
	struct device_node *np = ofdev->node;
	struct device_node *codec_np = NULL;
	const char *sprop;
	const u32 *iprop;
	struct resource res;
	struct platform_device *sound_device = NULL;
	struct mpc5121_ads_data *machine_data;
	struct mpc5121_psc_info *psc_info;
	struct mpc512x_dma_config dma_config;
	int ret = -ENODEV;

	machine_data = kzalloc(sizeof(struct mpc5121_ads_data), GFP_KERNEL);
	if (!machine_data)
		return -ENOMEM;

	psc_info = &machine_data->psc_info;
	psc_info->dev = &ofdev->dev;

	/*
	 * Get the device ID
	 */
	iprop = of_get_property(np, "cell-index", NULL);
	if (!iprop) {
		dev_err(&ofdev->dev, "cell-index property not found\n");
		ret = -EINVAL;
		goto error;
	}
	psc_info->id = *iprop;

	/* Get the serial format and clock direction. */
	sprop = of_get_property(np, "fsl,mode", NULL);
	if (!sprop) {
		dev_err(&ofdev->dev, "fsl,mode property not found\n");
		ret = -EINVAL;
		goto error;
	}

	if (strcasecmp(sprop, "i2s-slave") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_I2S;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_OUT;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "i2s-master") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_I2S;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_IN;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else if (strcasecmp(sprop, "lj-slave") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_LEFT_J;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_OUT;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "lj-master") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_LEFT_J;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_IN;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else if (strcasecmp(sprop, "rj-slave") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_RIGHT_J;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_OUT;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "rj-master") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_RIGHT_J;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_IN;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else if (strcasecmp(sprop, "ac97-slave") == 0) {
		machine_data->dai_format = SND_SOC_DAIFMT_AC97;
		machine_data->codec_clk_direction = SND_SOC_CLOCK_OUT;
		machine_data->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else {
		dev_err(&ofdev->dev,
			"unrecognized fsl,mode property \"%s\"\n", sprop);
		ret = -EINVAL;
		goto error;
	}

	if (!machine_data->clk_frequency &&
		machine_data->dai_format != SND_SOC_DAIFMT_AC97) {
		dev_err(&ofdev->dev, "unknown clock frequency\n");
		ret = -EINVAL;
		goto error;
	}

	/* Read the PSC information from the device tree */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&ofdev->dev, "could not obtain PSC address\n");
		goto error;
	}

	if (!res.start) {
		dev_err(&ofdev->dev, "invalid PSC address\n");
		goto error;
	}

	psc_info->phys = res.start;
	psc_info->psc = ioremap(psc_info->phys, res.end - res.start + 1);

	psc_reg_priv = psc_info->psc;
	if (!psc_info->psc) {
		dev_err(&ofdev->dev, "could not map PSC address %x\n",
			psc_info->phys);
		ret = -EINVAL;
		goto error;
	}

	/* Get the IRQ of the PSC */
	psc_info->irq = irq_of_parse_and_map(np, 0);
	if (!psc_info->irq) {
		dev_err(&ofdev->dev, "could not get PSC IRQ\n");
		ret = -EINVAL;
		goto error;
	}

#define PSC_DMA_GRAN 64
	/*
	 * FIXME put this in the device tree
	 *
	 * DMA channels are dedicated:
	 * 	rx = psc number
	 * 	tx = psc number + 12.
	 *
	 * DMA granularity is PSC_DMA_GRAN bytes
	 *
	 * PSC FIFO data register are at
	 * 	rx offset is 0xfc
	 * 	tx offset is 0xbc
	 *
	 *
	 */
	dma_config.rx_dma_ch_nr = psc_info->id;
	dma_config.tx_dma_ch_nr = psc_info->id + 12;
	psc_info->rx_dma_gran = PSC_DMA_GRAN;
	psc_info->tx_dma_gran = PSC_DMA_GRAN;
	dma_config.rx_dma_gran = PSC_DMA_GRAN;
	dma_config.tx_dma_gran = PSC_DMA_GRAN;
	dma_config.rx_dev_addr = psc_info->phys + 0xfc;
	dma_config.tx_dev_addr = psc_info->phys + 0xbc;
	if (!mpc512x_dma_configure(&dma_config)) {
		dev_err(&ofdev->dev, "could not configure DMA device\n");
		ret = -EBUSY;
		goto error;
	}

	/*
	 * Initialize our DAI data structure.
	 */
	machine_data->dai.name = "AC97";
	machine_data->dai.stream_name = "AC97";
	/*
	 * Because what bytes the psc driver
	 * supports, force the ac97 driver to only
	 * support 32 bit big endian data
	 */
	ac97_dai.playback.formats = SNDRV_PCM_FMTBIT_S32_BE;
	ac97_dai.capture.formats = SNDRV_PCM_FMTBIT_S32_BE;
	/*
	 * Add 16KHz to ac97 rates
	 */
	ac97_dai.playback.rates |= SNDRV_PCM_RATE_16000;
	ac97_dai.capture.rates |= SNDRV_PCM_RATE_16000;
	machine_data->dai.codec_dai = &ac97_dai;

	machine_data->dai.cpu_dai = mpc5121_psc_create_dai(psc_info);
	machine_data->dai.ops = &mpc5121_ads_ops;

	mpc5121_ads_machine.dai_link = &machine_data->dai;

	/* Allocate a new audio platform device structure */
	sound_device = platform_device_alloc("soc-audio", -1);
	if (!sound_device) {
		dev_err(&ofdev->dev, "platform device allocation failed\n");
		ret = -ENOMEM;
		goto error;
	}

	machine_data->sound_devdata.machine = &mpc5121_ads_machine;
	machine_data->sound_devdata.codec_dev = &soc_codec_dev_ac97;
	machine_data->sound_devdata.platform = &mpc512x_soc_platform;
	sound_device->dev.platform_data = machine_data;

	/* Set the platform device and ASoC device to point to each other */
	platform_set_drvdata(sound_device, &machine_data->sound_devdata);
	machine_data->sound_devdata.dev = &sound_device->dev;

	/* Initialize PSC controller and codec */
	mpc5121_psc_init(&ofdev->dev, machine_data->dai.cpu_dai);
	ret = platform_device_add(sound_device);
	if (ret) {
		dev_err(&ofdev->dev, "platform device add failed\n");
		goto error;
	}

	dev_set_drvdata(&ofdev->dev, sound_device);
	if (machine_data->dai_format == SND_SOC_DAIFMT_AC97)
		mpc5121_ac97_reset(NULL);
	return 0;

error:
	of_node_put(codec_np);

	if (sound_device)
		platform_device_unregister(sound_device);

	if (machine_data->dai.cpu_dai)
		mpc5121_psc_destroy_dai(machine_data->dai.cpu_dai);

	if (psc_info->psc)
		iounmap(psc_info->psc);

	if (psc_info->irq)
		irq_dispose_mapping(psc_info->irq);

	kfree(machine_data);

	return ret;
}

/**
 * mpc5121_ads_remove: remove the OF device
 *
 * This function is called when the OF device is removed.
 */
static int mpc5121_ads_remove(struct of_device *ofdev)
{
	struct platform_device *sound_device = dev_get_drvdata(&ofdev->dev);
	struct mpc5121_ads_data *machine_data =
					sound_device->dev.platform_data;

	platform_device_unregister(sound_device);

	if (machine_data->dai.cpu_dai)
		mpc5121_psc_destroy_dai(machine_data->dai.cpu_dai);

	if (machine_data->psc_info.psc)
		iounmap(machine_data->psc_info.psc);

	kfree(machine_data);
	sound_device->dev.platform_data = NULL;

	dev_set_drvdata(&ofdev->dev, NULL);

	return 0;
}

static struct of_device_id mpc5121_ads_match[] = {
	{
	 .compatible = "fsl,mpc5121-psc-ac97",
	},
	{}
};

MODULE_DEVICE_TABLE(of, mpc5121_ads_match);

static struct of_platform_driver mpc5121_ads_of_driver = {
	.owner = THIS_MODULE,
	.name = "mpc5121_ads",
	.match_table = mpc5121_ads_match,
	.probe = mpc5121_ads_probe,
	.remove = mpc5121_ads_remove,
};

/**
 * mpc5121_ads_init: fabric driver initialization.
 *
 * This function is called when this module is loaded.
 */
static int __init mpc5121_ads_init(void)
{
	int ret;

	printk(KERN_INFO "Freescale MPC5121 ADS ALSA SoC fabric driver\n");

	ret = of_register_platform_driver(&mpc5121_ads_of_driver);

	if (ret)
		printk(KERN_ERR
		       "mpc5121-ads: failed to register platform driver\n");

	return ret;
}

/**
 * mpc5121_ads_exit: fabric driver exit
 *
 * This function is called when this driver is unloaded.
 */
static void __exit mpc5121_ads_exit(void)
{
	of_unregister_platform_driver(&mpc5121_ads_of_driver);
}

module_init(mpc5121_ads_init);
module_exit(mpc5121_ads_exit);

MODULE_AUTHOR("John Rigby <jrigby@freescale.com>");
MODULE_DESCRIPTION("Freescale MPC5121 ADS ALSA SoC fabric driver");
MODULE_LICENSE("GPL");
