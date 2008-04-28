/*
 * Freescale MPC5121 ALSA SoC PCM driver
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: John Rigby <jrigby@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __MPC5121_PCM_H__
#define __MPC5121_PCM_H__
struct mpc512x_dma_config {
	int rx_dma_ch_nr;
	int tx_dma_ch_nr;
	dma_addr_t rx_dev_addr;
	dma_addr_t tx_dev_addr;
	int rx_dma_gran;
	int tx_dma_gran;
};


extern struct snd_soc_platform mpc512x_soc_platform;

/*
 * pass config info about the PSC driver to the DMA driver
 */
extern int mpc512x_dma_configure(struct mpc512x_dma_config *config);
#endif /* __MPC5121_PCM_H__ */
