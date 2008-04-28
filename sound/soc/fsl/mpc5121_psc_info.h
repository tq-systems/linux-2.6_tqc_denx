/*
 * mpc5121_psc_info.h - ALSC PSC interface for Freescale MPC5121ADS SoC
 *
 * Copyright 2008 Freescale Semiconductor Inc.
 * Author: John Rigby jrigby@freescale.com
 *
 * Based on fsl_ssi.h - Author Timur Tabi <timur@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */

#ifndef __MPC5121_PSC_INFO_H__
#define __MPC5121_PSC_INFO_H__

extern struct mpc52xx_psc *psc_reg_priv;

struct mpc5121_psc_info {
	unsigned int id;
	void __iomem *psc;
	dma_addr_t phys;
	unsigned int irq;
	struct device *dev;
	int rx_dma_gran;
	int tx_dma_gran;
};

/**
 * mpc5121_psc_private: per-PSC private data
 *
 * @name: short name for this device ("PSC0", "PSC1", etc)
 * @psc: pointer to the PSC's registers
 * @phys: physical address of the PSC registers
 * @irq: IRQ of this PSC
 * @dev: struct device pointer
 * @playback: the number of playback streams opened
 * @capture: the number of capture streams opened
 * @cpu_dai: the CPU DAI for this device
 * @format: the format of link
 */
struct mpc5121_psc_private {
	char name[8];
	void __iomem *psc;
	dma_addr_t phys;
	unsigned int irq;
	struct device *dev;
	unsigned int playback;
	unsigned int capture;
	struct snd_soc_cpu_dai cpu_dai;
	struct device_attribute dev_attr;
	struct clk *clk;
	int rx_dma_gran;
	int tx_dma_gran;
	int format;
};

/**
 * mpc5121_ads_data: fabric-specific ASoC device data
 *
 * This structure contains data for a single sound platform device on an
 * MPC5121e ADS.  Some of the data is taken from the device tree.
 */
struct mpc5121_ads_data {
	struct snd_soc_device sound_devdata;
	struct snd_soc_dai_link dai;
	struct snd_soc_machine machine;
	unsigned int dai_format;
	unsigned int codec_clk_direction;
	unsigned int cpu_clk_direction;
	unsigned int clk_frequency;
	struct mpc5121_psc_info psc_info;
};

struct snd_soc_cpu_dai *mpc5121_psc_create_dai(struct mpc5121_psc_info *);
void mpc5121_psc_destroy_dai(struct snd_soc_cpu_dai *);
int mpc5121_psc_init(struct device *dev, struct snd_soc_cpu_dai *cpu_dai);
int mpc5121_psc_clkinit(struct mpc5121_psc_private *psc_private, int on);
void mpc5121_psc_fifo_init(struct mpc5121_psc_private *psc_private);

void mpc5121_ac97_reset(struct snd_ac97 *ac97);
void mpc5121_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
			unsigned short val);
unsigned short mpc5121_ac97_read(struct snd_ac97 *ac97, unsigned short reg);
#endif				/* __MPC5121_PSC_INFO_H__  */
