/*
 * Hongjun Chen <hong-jun.chen@freescale.com>
 * Copyright (C) Freescale Semicondutor, Inc. 2007, 2008. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */
#ifndef _FSLDMA_H_
#define _FSLDMA_H_

#include <linux/dmaengine.h>
#include "fsldma_reg.h"
#include <linux/init.h>
#include <linux/dmapool.h>
#include <linux/cache.h>
#include <linux/pci_ids.h>
#include <linux/dmaengine.h>

/* This setting can be changed by user */
#define MAX_TCD_NUM_PER_CH	200

#define FSL_LOW_COMPLETION_MASK	0xffffffc0
#define DMA_NO_CHAN		-1;

extern struct list_head dma_device_list;
extern struct list_head dma_client_list;

/* This defines the prototype of callback funtion registered by the drivers */
typedef void (*fsl_dma_callback_t) (void *arg, int error_status);

/*! This defines the list of device ID's for DMA */
enum fsl_dma_device {
	FSL_DMA_MDDRC = 32,
	FSL_DMA_MBX = 31,
	FSL_DMA_SDHC = 30,
	FSL_DMA_NFC = 29,
	FSL_DMA_PATA_TX = 28,
	FSL_DMA_PATA_RX = 27,
	FSL_DMA_LPC = 26,
	FSL_DMA_SPDIF_RX = 25,
	FSL_DMA_SPDIF_TX = 24,
};

struct ch_pri {
	u8 g_pri;	/* The priority of group which this channel locates */
	u8 ch_pri;	/* The priority of this channel */
	u8 preempt;	/* Preemptable by other higher priority channel */
};

struct fsl_dma_mtcd_buf {
	void *addr_v;
	TCD *addr_va;		/* Aligned in 32 bytes */
	dma_addr_t addr_p;
	dma_addr_t addr_pa;	/* Aligned in 32 bytes */
	size_t size;
};

/**
 * struct fsl_device - internal representation of a FSL DMA device
 * @reg: register space
 * @tcd: transfer control descriptor space
 * @common: embedded struct dma_device
 * @ch_stat: the channel usage status
 * @arbit_mode: the arbitration mode of group and channel
 * @chpri: the channel priority, it is a tip for hack.
 */
struct fsl_device {
	fsl_dma_reg *reg;
	TCD *tcd;

	/* TCDs in SDRAM memory */
	struct fsl_dma_mtcd_buf mtcd;

	struct dma_device common;

	u32 irq;
	spinlock_t ch_lock;	/* protect channel usage status */
	u8 ch_stat[FSL_DMA_CH_NUM];	/* the channel usage status */

	u8 arbit_mode;		/* the arbitration mode of group and channel,
				   init once */
};

/**
 * struct fsl_dma_chan - internal representation of a DMA channel
 * @device:
 * @sw_in_use:
 * @completion:
 * @completion_low:
 * @completion_high:
 * @completed_cookie: last cookie seen completed on cleanup
 * @cookie: value of last cookie given to client
 * @last_completion:
 * @xfercap:
 * @priority:  the
 * @desc_lock:
 * @free_desc:
 * @used_desc:
 * @resource:
 * @device_node:
 */

struct fsl_dma_chan {
	fsl_dma_callback_t cb_fn;	/* The callback function */
	void *cb_args;		/* The argument of callback function */

	int num_buf;
	int pending;

	struct fsl_device *device;
	struct dma_chan common;

	int ch_index;
	struct ch_pri pri;

	u32 status;
	struct semaphore sem_lock;
};

/* Arbitration mode of group and channel */
#define FSL_DMA_GROUP_FIX 	0x01
#define	FSL_DMA_CH_FIX   	0x02

/* This structure contains the information about a dma transfer */
struct fsl_dma_requestbuf {
	dma_addr_t src;		/* source address */
	dma_addr_t dest;	/* destination address */
	size_t soff;		/* Source address signed offset */
	size_t doff;		/* Destination address signed offset */
	size_t minor_loop;	/* Number of bytes for every minor loop */
	size_t len;		/* the length of this transfer : bytes */
};

int fsl_dma_cfg_arbit_mode(int arbit_mode);

/*!
 * Before this function is called by the driver at open time, function
 * fsl_dma_cfg_arbit_mode() should be called to configure the arbitration mode
 * for dma engine. The DMA driver would do any initialization steps that is
 * required to get the channel ready for data transfer.
 *
 * @param channel_id  a pre-defined id. The peripheral driver would specify
 *                     the id associated with its peripheral. This would be
 *                     used by the DMA driver to identify the peripheral
 *                     requesting DMA and do the necessary setup on the
 *                     channel associated with the particular peripheral.
 *                     The DMA driver could use static or dynamic DMA channel
 *                     allocation.
 * @return returns a negative number on error if request for a DMA channel
 *                     did not succeed, returns the channel number to be
 *                     used on success.
 */
int fsl_dma_chan_request(int channel_id);

/*!
 * This function would just configure the scatterlist specified by the
 * user into dma channel. This is a slight variation of fsl_dma_config(),
 * it is provided for the convenience of drivers that have a scatterlist
 * passed into them. It is the calling driver's responsibility to have the
 * correct physical address filled in the "dma_address" field of the
 * scatterlist.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @param sg           a scatterlist of buffers. The caller must guarantee
 *                     the dma_buf is available until the transfer is
 *                     completed.
 * @param num_buf      number of buffers in the array
 * @param num_of_bytes total number of bytes to transfer. If set to 0, this
 *                     would imply to use the length field of the scatterlist
 *                     for each DMA transfer. Else it would calculate the size
 *                     for each DMA transfer.
 * @return This function returns a negative number on error if buffer could not
 *                     be added with DMA for transfer.
 *                     On Success, it returns 0
 */
int fsl_dma_sg_config(int channel_num, struct scatterlist *sg,
		      int num_buf, int num_of_bytes);

/*!
 * This function is generally called by the driver at close time. The DMA
 * driver would do any cleanup associated with this channel.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @return returns a negative number on error or 0 on success
 */
void fsl_dma_free_chan(int channel_num);

/*!
 * This function would just configure the buffers specified by the user into
 * dma channel. The caller must call fsl_dma_enable to start this transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @param dma_buf      an array of physical addresses to the user defined
 *                     buffers. The caller must guarantee the dma_buf is
 *                     available until the transfer is completed.
 * @param num_buf      number of buffers in the array
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could
 *                     not be added with DMA for transfer.
 *                     On Success, it returns 0
 */
int fsl_dma_config(int channel_num,
		   struct fsl_dma_requestbuf *dma_buf, int num_buf);

/*!
 * This function is provided if the driver would like to set/change its
 * callback function.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @param callback     a callback function to provide notification on transfer
 *                     completion, user could specify NULL if he does not wish
 *                     to be notified
 * @param arg          an argument that gets passed in to the callback
 *                     function, used by the user to do any driver specific
 *                     operations.
 * @return this function returns a negative number on error if the callback
 *                     could not be set for the channel or 0 on success
 */
int fsl_dma_callback_set(int channel_num,
			 fsl_dma_callback_t callback, void *arg);

/*!
 * This starts DMA transfer. Or it restarts DMA on a stopped channel
 * previously stopped with fsl_dma_disable().
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @return returns a negative number on error or 0 on success
 */
void fsl_dma_enable(int channel_num);

/*!
 * This stops the DMA channel and any ongoing transfers. Subsequent use of
 * fsl_dma_enable() will restart the channel and restart the transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @return returns a negative number on error or 0 on success
 */
void fsl_dma_disable(int channel_num);

/*!
 * This indicates what status the channel is.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver.
 * @return return a negative number on error or zero/positive number on success
 */
int fsl_dma_status(int channel_num);
#endif				/* _FSLDMA_H_ */
