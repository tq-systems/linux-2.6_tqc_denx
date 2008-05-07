/*
 * Copyright 2007,2008 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * PATA driver for mpc512x
 *
 * Based on:
 *     linux/drivers/ide/arm/mxc_ide.c
 *
 * Based on Simtec BAST IDE driver:
 *     Copyright (c) 2003-2004 Simtec Electronics
 *        Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*!
 * @file mcp512x_ide.c
 *
 * @brief ATA driver
 *
 * @ingroup ATA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/ide.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <asm/of_device.h>
#include <asm/of_platform.h>
#include <asm/fsldma.h>
#include "mpc512x.h"

#define DRV_NAME	"mpc512x_ide"

static void fsl_ata_dma_callback(void *arg, int error);

/* List of registered interfaces */
static ide_hwif_t *ifs[1];

/*
 * This structure contains the timing parameters for
 * ATA bus timing in the 5 PIO modes.  The timings
 * are in nanoseconds, and are converted to clock
 * cycles before being stored in the ATA controller
 * timing registers.
 */
static struct {
	short t0, t1, t2_8, t2_16, t2i, t4, t9, tA;
} pio_specs[] = {
	[0] = {
	.t0 = 600, .t1 = 70, .t2_8 = 290, .t2_16 = 165, .t2i = 0,
	.t4 = 30, .t9 = 20, .tA = 50
	},
	[1] = {
	.t0 = 383, .t1 = 50, .t2_8 = 290, .t2_16 = 125, .t2i = 0,
	.t4 = 20, .t9 = 15, .tA = 50
	},
	[2] = {
	.t0 = 240, .t1 = 30, .t2_8 = 290, .t2_16 = 100, .t2i = 0,
	.t4 = 15, .t9 = 10, .tA = 50
	},
	[3] = {
	.t0 = 180, .t1 = 30, .t2_8 = 80, .t2_16 = 80, .t2i = 0,
	.t4 = 10, .t9 = 10, .tA = 50
	},
	[4] = {
	.t0 = 120, .t1 = 25, .t2_8 = 70, .t2_16 = 70, .t2i = 0,
	.t4 = 10, .t9 = 10, .tA = 50
	}
};

#define NR_PIO_SPECS (sizeof pio_specs / sizeof pio_specs[0])

/*
 * This structure contains the timing parameters for
 * ATA bus timing in the 3 MDMA modes.  The timings
 * are in nanoseconds, and are converted to clock
 * cycles before being stored in the ATA controller
 * timing registers.
 */
static struct {
	short t0M, tD, tH, tJ, tKW, tM, tN, tJNH;
} mdma_specs[] = {
	[0] = {
	.t0M = 480, .tD = 215, .tH = 20, .tJ = 20, .tKW = 215,
	.tM = 50, .tN = 15, .tJNH = 20
	},
	[1] = {
	.t0M = 150, .tD = 80, .tH = 15, .tJ = 5, .tKW = 50,
	.tM = 30, .tN = 10, .tJNH = 15
	},
	[2] = {
	.t0M = 120, .tD = 70, .tH = 10, .tJ = 5, .tKW = 25,
	.tM = 25, .tN = 10, .tJNH = 10
	}
};

#define NR_MDMA_SPECS (sizeof mdma_specs / sizeof mdma_specs[0])

/*
 * This structure contains the timing parameters for
 * ATA bus timing in the 6 UDMA modes.  The timings
 * are in nanoseconds, and are converted to clock
 * cycles before being stored in the ATA controller
 * timing registers.
 */
static struct {
	short t2CYC, tCYC, tDS, tDH, tDVS, tDVH, tCVS, tCVH, tFS_min, tLI_max,
	    tMLI, tAZ, tZAH, tENV_min, tSR, tRFS, tRP, tACK, tSS, tDZFS;
} udma_specs[] = {
	[0] = {
	.t2CYC = 235, .tCYC = 114, .tDS = 15, .tDH = 5, .tDVS = 70,
	.tDVH = 6, .tCVS = 70, .tCVH = 6, .tFS_min = 0, .tLI_max = 100,
	.tMLI = 20, .tAZ = 10, .tZAH = 20, .tENV_min = 20, .tSR = 50,
	.tRFS = 75, .tRP = 160, .tACK = 20, .tSS = 50, .tDZFS = 80
	},
	[1] = {
	.t2CYC = 156, .tCYC = 75, .tDS = 10, .tDH = 5, .tDVS = 48,
	.tDVH = 6, .tCVS = 48, .tCVH = 6, .tFS_min = 0, .tLI_max = 100,
	.tMLI = 20, .tAZ = 10, .tZAH = 20, .tENV_min = 20, .tSR = 30,
	.tRFS = 70, .tRP = 125, .tACK = 20, .tSS = 50, .tDZFS = 63
	},
	[2] = {
	.t2CYC = 117, .tCYC = 55, .tDS = 7, .tDH = 5, .tDVS = 34,
	.tDVH = 6, .tCVS = 34, .tCVH = 6, .tFS_min = 0, .tLI_max = 100,
	.tMLI = 20, .tAZ = 10, .tZAH = 20, .tENV_min = 20, .tSR = 20,
	.tRFS = 60, .tRP = 100, .tACK = 20, .tSS = 50, .tDZFS = 47
	},
	[3] = {
	.t2CYC = 86, .tCYC = 39, .tDS = 7, .tDH = 5, .tDVS = 20,
	.tDVH = 6, .tCVS = 20, .tCVH = 6, .tFS_min = 0, .tLI_max = 100,
	.tMLI = 20, .tAZ = 10, .tZAH = 20, .tENV_min = 20, .tSR = 20,
	.tRFS = 60, .tRP = 100, .tACK = 20, .tSS = 50, .tDZFS = 35
	},
	[4] = {
	.t2CYC = 57, .tCYC = 25, .tDS = 5, .tDH = 5, .tDVS = 7,
	.tDVH = 6, .tCVS = 7, .tCVH = 6, .tFS_min = 0, .tLI_max = 100,
	.tMLI = 20, .tAZ = 10, .tZAH = 20, .tENV_min = 20, .tSR = 50,
	.tRFS = 60, .tRP = 100, .tACK = 20, .tSS = 50, .tDZFS = 25
	},
	[5] = {
	.t2CYC = 38, .tCYC = 17, .tDS = 4, .tDH = 5, .tDVS = 5,
	.tDVH = 6, .tCVS = 10, .tCVH = 10, .tFS_min = 0, .tLI_max = 75,
	.tMLI = 20, .tAZ = 10, .tZAH = 20, .tENV_min = 20, .tSR = 20,
	.tRFS = 50, .tRP = 85, .tACK = 20, .tSS = 50, .tDZFS = 40
	}
};

#define NR_UDMA_SPECS (sizeof udma_specs / sizeof udma_specs[0])

/*!
 * Calculate values for the ATA bus timing registers and store
 * them into the hardware.
 *
 * @param       mode        Selects PIO, MDMA or UDMA modes
 *
 * @param       speed       Specifies the sub-mode number
 *
 * @return      EINVAL      speed out of range, or illegal mode
 */
static int set_ata_bus_timing(struct fsl_ata_priv *priv, int speed,
			      enum ata_mode mode)
{
	/* get the bus clock cycle time, in ns */
	int T = 1 * 1000 * 1000 * 1000 / clk_get_rate(priv->ata_clk);
	union fsl_ata_time_cfg cfg0, cfg1, cfg2, cfg3, cfg4, cfg5;
	/* every mode gets the same t_off and t_on */

	GET_TIME_CFG(&cfg0, FSL_ATA_TIME_OFF);
	cfg0.bytes.field1 = 3;
	cfg0.bytes.field2 = 3;
	SET_TIME_CFG(&cfg0, 3, FSL_ATA_TIME_OFF);

	switch (mode) {
	case PIO:
		if (speed < 0 || speed >= NR_PIO_SPECS)
			return -EINVAL;

		cfg0.bytes.field3 = (pio_specs[speed].t1 + T) / T;
		cfg0.bytes.field4 = (pio_specs[speed].t2_8 + T) / T;

		cfg1.bytes.field1 = (pio_specs[speed].t2_8 + T) / T;
		cfg1.bytes.field2 = (pio_specs[speed].tA + T) / T + 2;
		cfg1.bytes.field3 = 1;
		cfg1.bytes.field4 = (pio_specs[speed].t4 + T) / T;

		GET_TIME_CFG(&cfg2, FSL_ATA_TIME_9);
		cfg2.bytes.field1 = (pio_specs[speed].t9 + T) / T;

		SET_TIME_CFG(&cfg0, 0x0C, FSL_ATA_TIME_OFF);
		SET_TIME_CFG(&cfg1, 0x0F, FSL_ATA_TIME_2r);
		SET_TIME_CFG(&cfg2, 0x01, FSL_ATA_TIME_9);
		break;
	case MDMA:
		if (speed < 0 || speed >= NR_MDMA_SPECS)
			return -EINVAL;

		GET_TIME_CFG(&cfg2, FSL_ATA_TIME_9);
		GET_TIME_CFG(&cfg3, FSL_ATA_TIME_K);

		cfg2.bytes.field2 = (mdma_specs[speed].tM + T) / T;
		cfg2.bytes.field3 = (mdma_specs[speed].tJNH + T) / T;
		cfg2.bytes.field4 = (mdma_specs[speed].tD + T) / T;

		cfg3.bytes.field1 = (mdma_specs[speed].tKW + T) / T;

		SET_TIME_CFG(&cfg2, 0x0E, FSL_ATA_TIME_9);
		SET_TIME_CFG(&cfg3, 0x01, FSL_ATA_TIME_K);
		break;
	case UDMA:
		if (speed < 0 || speed >= NR_UDMA_SPECS)
			return -EINVAL;

		GET_TIME_CFG(&cfg3, FSL_ATA_TIME_K);

		cfg3.bytes.field2 = (udma_specs[speed].tACK + T) / T;
		cfg3.bytes.field3 = (udma_specs[speed].tENV_min + T) / T;
		cfg3.bytes.field4 = (udma_specs[speed].tRP + T) / T + 2;

		cfg4.bytes.field1 = (udma_specs[speed].tZAH + T) / T;
		cfg4.bytes.field2 = (udma_specs[speed].tMLI + T) / T;
		cfg4.bytes.field3 = (udma_specs[speed].tDVH + T) / T + 1;
		cfg4.bytes.field4 = (udma_specs[speed].tDZFS + T) / T;

		cfg5.bytes.field1 = (udma_specs[speed].tDVS + T) / T;
		cfg5.bytes.field2 = (udma_specs[speed].tCVH + T) / T;
		cfg5.bytes.field3 = (udma_specs[speed].tSS + T) / T;
		cfg5.bytes.field4 = (udma_specs[speed].tCYC + T) / T;

		SET_TIME_CFG(&cfg3, 0x0E, FSL_ATA_TIME_K);
		SET_TIME_CFG(&cfg4, 0x0F, FSL_ATA_TIME_ZAH);
		SET_TIME_CFG(&cfg5, 0x0F, FSL_ATA_TIME_DVS);
		break;
	default:
		;
		return -EINVAL;
	}
	return 0;
}

/*!
 * Placeholder for code to make any hardware tweaks
 * necessary to select a drive.  Currently we are
 * not aware of any.
 */
static void fsl_ata_selectproc(ide_drive_t *drive)
{
	return;
}

/*!
 * Called to set the PIO mode.
 *
 * @param   drive       Specifies the drive
 * @param   pio    Specifies the PIO mode number desired
 */
static void fsl_ata_set_pio_mode(ide_drive_t *drive, u8 pio)
{
	struct fsl_ata_priv *priv =
	    (struct fsl_ata_priv *)HWIF(drive)->hwif_data;

	set_ata_bus_timing(priv, pio, PIO);
}

/*!
 * Hardware-specific interrupt service routine for the ATA driver,
 * called mainly just to dismiss the interrupt at the hardware, and
 * to indicate whether there actually was an interrupt pending.
 *
 * The generic IDE related interrupt handling is done by the IDE layer.
 */
static int fsl_ata_ack_intr(struct hwif_s *hw)
{
	struct fsl_ata_priv *priv = hw->hwif_data;
	unsigned char status = ATA_RAW_READ(FSL_ATA_INTR_PENDING);
	unsigned char enable = ATA_RAW_READ(FSL_ATA_INTR_ENABLE);

	/*
	 * The only interrupts we can actually dismiss are the FIFO conditions.
	 * INTRQ comes from the bus, and must be dismissed according to IDE
	 * protocol, which will be done by the IDE layer, even when DMA
	 * is invovled (DMA can see it, but can't dismiss it).
	 */
	ATA_RAW_WRITE(status, FSL_ATA_INTR_CLEAR);

	if (status & enable & ~FSL_ATA_INTR_ATA_INTRQ2) {
		printk(KERN_ERR DRV_NAME "unexpected interrupt, "
		       "status=0x%02X\n", status);
	}

	return status ? 1 : 0;
}

/*!
 * Decodes the specified transfer mode and sets both timing and ultra modes
 *
 * @param       drive       Specifies the drive
 *
 * @param       speed   Specifies the desired transfer mode
 *
 * @return      EINVAL      Illegal mode specified
 */
static void fsl_ata_set_speed(ide_drive_t *drive, const u8 speed)
{
	struct fsl_ata_priv *priv =
	    (struct fsl_ata_priv *)HWIF(drive)->hwif_data;
	switch (speed) {
	case XFER_UDMA_7:
	case XFER_UDMA_6:
	case XFER_UDMA_5:
	case XFER_UDMA_4:
	case XFER_UDMA_3:
	case XFER_UDMA_2:
	case XFER_UDMA_1:
	case XFER_UDMA_0:
		priv->ultra = 1;
		set_ata_bus_timing(priv, speed - XFER_UDMA_0, UDMA);
		return;
		break;
	case XFER_MW_DMA_2:
	case XFER_MW_DMA_1:
	case XFER_MW_DMA_0:
		priv->ultra = 0;
		set_ata_bus_timing(priv, speed - XFER_MW_DMA_0, MDMA);
		return;
		break;
	}
}

static void __fsl_ata_resetproc(struct fsl_ata_priv *priv)
{
	printk(KERN_INFO "%s: resetting ATA controller\n", __func__);

	if (priv->dma_read_chan >= 0) {
		fsl_dma_free_chan(priv->dma_read_chan);
		priv->dma_read_chan = fsl_dma_chan_request(MPC512X_DMA_ATA_RX);
		if (priv->dma_read_chan < 0) {
			printk(KERN_ERR DRV_NAME ": "
			       "%s: could not reallocate RX DMA channel\n",
			       DRV_NAME);
		}
	}

	if (priv->dma_write_chan >= 0) {
		fsl_dma_free_chan(priv->dma_write_chan);
		priv->dma_write_chan = fsl_dma_chan_request(MPC512X_DMA_ATA_TX);
		if (priv->dma_write_chan < 0) {
			printk(KERN_ERR DRV_NAME ": "
			       "%s: could not reallocate TX DMA channel\n",
			       DRV_NAME);
		}
	}
	ATA_RAW_WRITE(0x00, FSL_ATA_CONTROL);
	udelay(100);
	ATA_RAW_WRITE(FSL_ATA_CTRL_ATA_RST_B, FSL_ATA_CONTROL);
	udelay(100);
}

/*!
 * Called by the IDE layer when something goes wrong
 *
 * @param       drive       Specifies the drive
 *
 */
static void fsl_ata_resetproc(ide_drive_t *drive)
{
	struct fsl_ata_priv *priv =
	    (struct fsl_ata_priv *)HWIF(drive)->hwif_data;
	__fsl_ata_resetproc(priv);
}

/*!
 * The DMA is done, and the drive is done.  We'll check the BD array for
 * errors, and unmap the scatter-gather list.
 *
 * @param       drive       The drive we're servicing
 *
 * @return      0 means all is well, others means DMA signalled an error ,
 */
static int fsl_ata_dma_end(ide_drive_t *drive)
{
	struct fsl_ata_priv *priv =
	    (struct fsl_ata_priv *)HWIF(drive)->hwif_data;
	int dma_stat = priv->dma_stat;
	ide_hwif_t *hwif = HWIF(drive);

	drive->waiting_for_dma = 0;

	/*
	 * We'll unmap the sg table regardless of status.
	 */
	dma_unmap_sg(priv->dev, hwif->sg_table, hwif->sg_nents,
		     hwif->sg_dma_direction);

	return dma_stat;
}

/*!
 * The end-of-DMA interrupt handler
 *
 * @param       drive       Specifies the drive
 *
 * @return      ide_stopped or ide_started
 */
static ide_startstop_t fsl_ata_dma_intr(ide_drive_t *drive)
{
	u8 stat, dma_stat;
	struct request *rq = HWGROUP(drive)->rq;
	ide_hwif_t *hwif = HWIF(drive);
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;
	u8 fifo_fill;

	if (!rq)
		return ide_stopped;

	fifo_fill = ATA_RAW_READ(FSL_ATA_FIFO_FILL);
	BUG_ON(fifo_fill);

	dma_stat = hwif->ide_dma_end(drive);
	stat = hwif->INB(IDE_STATUS_REG);	/* get drive status */
	if (OK_STAT(stat, DRIVE_READY, drive->bad_wstat | DRQ_STAT)) {
		if (dma_stat == FSL_DMA_DONE) {
			ide_end_request(drive, 1, rq->nr_sectors);
			return ide_stopped;
		}
		printk(KERN_ERR
		       "%s: fsl_ata_dma_intr: bad DMA status (0x%x)\n",
		       drive->name, dma_stat);
	}

	return ide_error(drive, "fsl_ata_dma_intr", stat);
}

static void fsl_ata_enable_irq(ide_hwif_t *hwif)
{
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;
	ATA_RAW_WRITE(FSL_ATA_INTR_ATA_INTRQ2, FSL_ATA_INTR_ENABLE);
}

static void fsl_ata_disable_irq(ide_hwif_t *hwif)
{
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;
	ATA_RAW_WRITE(0, FSL_ATA_INTR_ENABLE);
}

/*!
 * Masks drive interrupts temporarily at the hardware level
 *
 * @param       drive       Specifies the drive
 *
 * @param       mask        1 = disable interrupts, 0 = re-enable
 *
 */
static void fsl_ata_maskproc(ide_drive_t *drive, int mask)
{
	struct fsl_ata_priv *priv =
	    (struct fsl_ata_priv *)(HWIF(drive)->hwif_data);
	BUG_ON(!priv);

	if (mask) {
		ATA_RAW_WRITE(0, FSL_ATA_INTR_ENABLE);
	} else {
		ATA_RAW_WRITE(FSL_ATA_INTR_ATA_INTRQ2, FSL_ATA_INTR_ENABLE);
	}
}

/*!
 * DMA completion callback routine.  This gets called after the DMA request
 * has completed or aborted.All we need to do here is return ownership of
 * the drive's INTRQ signal back to the CPU, which will immediately raise
 * the interrupt to the IDE layer.
 *
 * @param       arg         The drive we're servicing
 * @param	error	    The error number of DMA transfer.
 */
static void fsl_ata_dma_callback(void *arg, int error)
{
	ide_hwif_t *hwif = HWIF((ide_drive_t *) arg);
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)(hwif->hwif_data);
	unsigned long fifo_fill;
	int cnt = 0;

	/*
	 * clean the fifo if the fill register is non-zero.
	 * If the fill register is non-zero, it is incorrect state.
	 */
	fifo_fill = ATA_RAW_READ(FSL_ATA_FIFO_FILL);
	cnt = 0;
	while (fifo_fill) {
		printk(KERN_ERR DRV_NAME ": "
			"waiting for fifo to drain %08lx\n", fifo_fill);
		fifo_fill = ATA_RAW_READ(FSL_ATA_FIFO_FILL);
		if (cnt++ > 100)
			break;
	}
	cnt = 0;
	while (fifo_fill) {
		unsigned long d;
		d = ATA_RAW_READ(FSL_ATA_FIFO_DATA_32);
		printk(KERN_ERR DRV_NAME ": "
			"draining fifo d/ff/c %08lx/%lx/%d\n",
			d, fifo_fill, cnt);
		fifo_fill = ATA_RAW_READ(FSL_ATA_FIFO_FILL);
		cnt++;
	}

	priv->dma_stat = error;
	/*
	 * Redirect ata_intrq back to us instead of the DMA.
	 */
	fsl_ata_enable_irq(hwif);
}

static int fsl_ata_dma_sg_config(struct fsl_ata_priv *priv, int chan,
				 struct scatterlist *sg, int sg_nents,
				 int dma_mode)
{
	struct fsl_dma_requestbuf *dma = priv->dma_reqbufs;
	int count = 0;

	/* fill the descriptors */
	while (sg_nents && sg_dma_len(sg)) {
		if (dma_mode == FSL_DMA_MODE_READ) {
			dma->src = priv->dma_addr;
			dma->soff = 0;
			dma->dest = (dma_addr_t) sg_dma_address(sg);
			dma->doff = 4;
		} else {
			dma->src = (dma_addr_t) sg_dma_address(sg);
			dma->soff = 4;
			dma->dest = priv->dma_addr;
			dma->doff = 0;
		}
		dma->len = sg_dma_len(sg);
		dma->minor_loop = FSL_ATA_DMA_WATERMARK / 4;

		dma++;
		sg++;
		sg_nents--;
		count++;
	}
	if (count) {
		if (fsl_dma_config(chan, priv->dma_reqbufs, count) < 0)
			return 0;
	}

	return count;
}

/*!
 * DMA set up.  This is called once per DMA request to the drive. It delivers
 * the scatter-gather list into the DMA channel and prepares both the ATA
 * controller and the DMA engine for the DMA
 * transfer.
 *
 * @param       drive     The drive we're servicing
 *
 * @return      0 on success, non-zero otherwise
 */
static int fsl_ata_dma_setup(ide_drive_t *drive)
{
	struct request *rq = HWGROUP(drive)->rq;
	ide_hwif_t *hwif = HWIF(drive);
	struct scatterlist *sg = hwif->sg_table;
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;
	int dma_ultra = priv->ultra ? FSL_ATA_CTRL_DMA_ULTRA : 0;
	int dma_mode = 0;
	int chan;
	u8 ata_control;
	u8 fifo_fill;

	BUG_ON(!rq);
	BUG_ON(!priv);
	BUG_ON(drive->waiting_for_dma);

	/*Initialize the dma state */
	priv->dma_stat = FSL_DMA_TRANSFER_ERROR;

	/*
	 * Prepare the ATA controller for the DMA
	 */
	if (rq_data_dir(rq)) {
		chan = priv->dma_write_chan;
		ata_control = FSL_ATA_CTRL_FIFO_RST_B |
		    FSL_ATA_CTRL_ATA_RST_B |
		    FSL_ATA_CTRL_FIFO_TX_EN |
		    FSL_ATA_CTRL_DMA_PENDING |
		    dma_ultra | FSL_ATA_CTRL_DMA_WRITE;

		dma_mode = FSL_DMA_MODE_WRITE;
	} else {
		chan = priv->dma_read_chan;
		ata_control = FSL_ATA_CTRL_FIFO_RST_B |
		    FSL_ATA_CTRL_ATA_RST_B |
		    FSL_ATA_CTRL_FIFO_RCV_EN |
		    FSL_ATA_CTRL_DMA_PENDING | dma_ultra;

		dma_mode = FSL_DMA_MODE_READ;
	}

	/*
	 * Set up the DMA interrupt callback
	 */
	fsl_dma_callback_set(chan, fsl_ata_dma_callback, (void *)drive);

	/*
	 * If the ATA FIFO isn't empty, we shouldn't even be here
	 */
	fifo_fill = ATA_RAW_READ(FSL_ATA_FIFO_FILL);
	BUG_ON(fifo_fill);	/* TODO: need better recovery here */

	ide_map_sg(drive, rq);

	hwif->sg_dma_direction = rq_data_dir(rq) ? DMA_TO_DEVICE :
	    DMA_FROM_DEVICE;

	hwif->sg_nents = dma_map_sg(priv->dev, sg, hwif->sg_nents,
				    hwif->sg_dma_direction);
	BUG_ON(!hwif->sg_nents);
	BUG_ON(hwif->sg_nents > MAX_TCD_NUM_PER_CH);

	if (fsl_ata_dma_sg_config(priv, chan, sg, hwif->sg_nents, dma_mode) ==
	    0)
		goto use_pio_instead;

	ATA_RAW_WRITE(ata_control, FSL_ATA_CONTROL);
	ATA_RAW_WRITE(FSL_ATA_DMA_WATERMARK / 2, FSL_ATA_FIFO_ALARM);

	/*
	 * Route ata_intrq to the DMA engine, and not to us.
	 */
	fsl_ata_disable_irq(hwif);

	/*
	 * The DMA and ATA controller are ready to go.
	 * fsl_ata_dma_start() will start the DMA transfer,
	 * and fsl_ata_dma_exec_cmd() will tickle the drive, which
	 * actually initiates the DMA transfer on the ATA bus.
	 * The ATA controller is DMA slave for both read and write.
	 */
	BUG_ON(drive->waiting_for_dma);
	drive->waiting_for_dma = 1;
	return 0;

use_pio_instead:
	pci_unmap_sg(hwif->pci_dev,
		     hwif->sg_table, hwif->sg_nents, hwif->sg_dma_direction);

	return 1;		/* revert to PIO for this request */
}

/*!
 * DMA timeout notification.  This gets called when the IDE layer above
 * us times out waiting for a request.
 *
 * @param       drive       The drive we're servicing
 *
 * @return      0 to attempt recovery, otherwise, an additional tick count
 *              to keep waiting
 */
static int fsl_ata_dma_timer_expiry(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;

	printk(KERN_ERR DRV_NAME ": "
		"%s %s: fifo_fill=%d\n", __FUNCTION__, drive->name,
	       readb(FSL_ATA_FIFO_FILL));

	fsl_ata_resetproc(drive);

	if (drive->waiting_for_dma)
		HWIF(drive)->ide_dma_end(drive);

	return 0;
}

/*!
 * Called by the IDE layer to start a DMA request on the specified drive.
 * The request has already been prepared by \b fsl_ata_dma_setup().  All
 * we need to do is pass the command through while specifying our timeout
 * handler.
 *
 * @param       drive       The drive we're servicing
 *
 * @param       cmd         The IDE command for the drive
 *
 */
static void fsl_ata_dma_exec_cmd(ide_drive_t *drive, u8 cmd)
{
	ide_execute_command(drive, cmd, fsl_ata_dma_intr, 2 * WAIT_CMD,
			    fsl_ata_dma_timer_expiry);
}

/*!
 * Called by the IDE layer to start the DMA controller.  The request has
 * already been prepared by \b fsl_ata_dma_setup().  All we do here
 * is tickle the DMA channel.
 *
 * @param       drive       The drive we're servicing
 *
 */
static void fsl_ata_dma_start(ide_drive_t *drive)
{
	struct request *rq = HWGROUP(drive)->rq;
	ide_hwif_t *hwif = HWIF(drive);
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;
	int chan = rq_data_dir(rq) ? priv->dma_write_chan : priv->dma_read_chan;

	BUG_ON(chan < 0);

	/*
	 * Tickle the DMA channel.  This starts the channel, but it is likely
	 * that DMA will yield and go idle before the DMA request arrives from
	 * the drive.  Nonetheless, at least the context will be hot.
	 */
	fsl_dma_enable(chan);
}

/*!
 * There is a race between the DMA interrupt and the timeout interrupt.  This
 * gets called during the IDE layer's timeout interrupt to see if the DMA
 * interrupt has also occured or is pending.
 *
 * @param       drive       The drive we're servicing
 *
 * @return      1 means there is a DMA interrupt pending, 0 otherwise
 */
static int fsl_ata_dma_test_irq(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;
	unsigned char status = ATA_RAW_READ(FSL_ATA_INTR_PENDING);

	/*
	 * We need to test the interrupt status without dismissing any.
	 */

	return status & (FSL_ATA_INTR_ATA_INTRQ1
			 | FSL_ATA_INTR_ATA_INTRQ2) ? 1 : 0;
}

/*!
 * Called once per controller to set up DMA
 *
 * @param       hwif       Specifies the IDE controller
 *
 */
static void fsl_ata_dma_init(ide_hwif_t *hwif)
{
	struct fsl_ata_priv *priv = (struct fsl_ata_priv *)hwif->hwif_data;

	hwif->dmatable_cpu = NULL;
	hwif->dmatable_dma = 0;
	hwif->set_dma_mode = fsl_ata_set_speed;
	hwif->resetproc = fsl_ata_resetproc;

	/*
	 * Allocate and setup the DMA channels
	 */
	priv->dma_read_chan = fsl_dma_chan_request(MPC512X_DMA_ATA_RX);
	if (priv->dma_read_chan < 0) {
		printk(KERN_ERR DRV_NAME "%s: couldn't get RX DMA channel\n",
		       hwif->name);
		goto err_out;
	}

	priv->dma_write_chan = fsl_dma_chan_request(MPC512X_DMA_ATA_TX);
	if (priv->dma_write_chan < 0) {
		printk(KERN_ERR DRV_NAME "%s: couldn't get TX DMA channel\n",
		       hwif->name);
		goto err_out;
	}

	set_ata_bus_timing(priv, 0, UDMA);

	/*
	 * All ready now
	 */
	hwif->ultra_mask = 0x7f;
	hwif->mwdma_mask = 0x07;
	hwif->swdma_mask = 0x07;

	hwif->cbl = ATA_CBL_PATA80;

	hwif->drives[0].unmask = 1;
	hwif->drives[1].unmask = 1;
	hwif->drives[0].autotune = IDE_TUNE_AUTO;
	hwif->drives[1].autotune = IDE_TUNE_AUTO;

	hwif->dma_off_quietly = &ide_dma_off_quietly;
	hwif->ide_dma_on = &__ide_dma_on;
	hwif->dma_setup = &fsl_ata_dma_setup;
	hwif->dma_exec_cmd = &fsl_ata_dma_exec_cmd;
	hwif->dma_start = &fsl_ata_dma_start;
	hwif->ide_dma_end = &fsl_ata_dma_end;
	hwif->ide_dma_test_irq = &fsl_ata_dma_test_irq;
	hwif->dma_host_off = &ide_dma_host_off;
	hwif->dma_host_on = &ide_dma_host_on;
	hwif->dma_timeout = &ide_dma_timeout;
	hwif->dma_lost_irq = &ide_dma_lost_irq;

	return;

err_out:
	if (priv->dma_read_chan >= 0)
		fsl_dma_free_chan(priv->dma_read_chan);
	if (priv->dma_write_chan >= 0)
		fsl_dma_free_chan(priv->dma_write_chan);
	kfree(priv);
	return;
}

/*!
 * MPC512X-specific IDE registration helper routine.  Among other things,
 * we tell the IDE layer where our standard IDE drive registers are,
 * through the \b io_ports array.  The IDE layer sends commands to and
 * reads status directly from attached IDE drives through these drive
 * registers.
 *
 * @param   base   Base address of memory mapped IDE standard drive registers
 *
 * @param   aux    Address of the auxilliary ATA control register
 *
 * @param   irq    IRQ number for our hardware
 *
 * @param   hwifp  Pointer to hwif structure to be updated by the IDE layer
 *
 * @param   priv   Pointer to private structure
 *
 * @return  ENODEV if no drives are present, 0 otherwise
 */
static int __init
fsl_ata_register(unsigned long base, unsigned long aux, int irq,
		 ide_hwif_t **hwifp, struct fsl_ata_priv *priv)
{
	int i = 0;
	hw_regs_t hw;
	ide_hwif_t *hwif;
	const int regsize = 4;
	u8 idx[4] = { 0xff, 0xff, 0xff, 0xff };

	i = 0;
	while (i < MAX_HWIFS && (ide_hwifs[i].io_ports[IDE_DATA_OFFSET] != 0))
		++i;
	if (i >= MAX_HWIFS) {
		printk(KERN_ERR DRV_NAME": "
			"no empty slot in ide_hwifs\n");
		return -ENODEV;
	}
	hwif = &ide_hwifs[i];

	memset(&hw, 0, sizeof(hw));
	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
		hw.io_ports[i] = (unsigned long)base;
		base += regsize;
	}
	hw.io_ports[IDE_CONTROL_OFFSET] = aux;
	hw.irq = irq;

	*hwifp = hwif;
	idx[0] = ide_register_hw(&hw, NULL, 1, hwifp);
	if (idx[0] == 0xff) {
		printk(KERN_ERR DRV_NAME"fsl_ata: IDE I/F register failed\n");
		return -1;
	}

	hwif->selectproc = &fsl_ata_selectproc;
	hwif->pio_mask = ATA_PIO4;
	hwif->set_pio_mode = &fsl_ata_set_pio_mode;
	hwif->ack_intr = &fsl_ata_ack_intr;
	hwif->maskproc = &fsl_ata_maskproc;
	hwif->hwif_data = (void *)priv;
	fsl_ata_enable_irq(hwif);

	fsl_ata_dma_init(hwif);

	ide_device_add(idx);

	return 0;
}

/*!
 * Driver initialization routine.  Prepares the hardware for ATA activity.
 */
static int __init mpc512x_ata_probe(struct of_device *op,
				    const struct of_device_id *match)
{
	int index = 0;
	struct fsl_ata_priv *priv;
	struct resource res_mem;
	int rv;

	/*
	 * Allocate a private structure
	 */
	priv = devm_kzalloc(&op->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		printk(KERN_ERR DRV_NAME ": "
		       "Error while allocating private structure\n");
		return ENOMEM;
	}

	priv->dev = NULL;
	priv->dma_read_chan = -1;
	priv->dma_write_chan = -1;

	/* Get IRQ and register */
	rv = of_address_to_resource(op->node, 0, &res_mem);
	if (rv) {
		printk(KERN_ERR DRV_NAME ": "
		       "Error while parsing device node resource\n");
		return rv;
	}

	priv->ata_irq = irq_of_parse_and_map(op->node, 0);
	if (priv->ata_irq == NO_IRQ) {
		printk(KERN_ERR DRV_NAME ": "
			"Error while mapping the irq\n");
		return -EINVAL;
	}

	/* Request mem region */
	if (!devm_request_mem_region(&op->dev, res_mem.start,
				     1 + res_mem.end - res_mem.start,
				     DRV_NAME)) {
		printk(KERN_ERR DRV_NAME ": "
		       "Error while requesting mem region\n");
		rv = -EBUSY;
		goto err;
	}

	/* mapin regs */
	priv->ata_regs = devm_ioremap(&op->dev, res_mem.start,
				      1 + res_mem.end - res_mem.start);
	if (!priv->ata_regs) {
		printk(KERN_ERR DRV_NAME ": "
		       "Error while mapping register set\n");
		rv = -ENOMEM;
		goto err;
	}
	priv->dma_addr = (dma_addr_t) res_mem.start + 0x18;

	priv->ata_clk = clk_get(&op->dev, "pata_clk");
	clk_enable(priv->ata_clk);

	/* Deassert the reset bit to enable the interface */
	ATA_RAW_WRITE(FSL_ATA_CTRL_ATA_RST_B, FSL_ATA_CONTROL);

	/* Set initial timing and mode */
	set_ata_bus_timing(priv, 0, PIO);

	/* Reset the interface */
	__fsl_ata_resetproc(priv);

	/*
	 * Enable hardware interrupts.
	 * INTRQ2 goes to us, so we enable it here, but we'll need to ignore
	 * it when DMA is doing the transfer.
	 */
	ATA_RAW_WRITE(FSL_ATA_INTR_ATA_INTRQ2, FSL_ATA_INTR_ENABLE);

	/*
	 * Now register
	 */
	index =
	    fsl_ata_register((unsigned long)FSL_ATA_IO,
			     (unsigned long)FSL_ATA_CTL, priv->ata_irq, &ifs[0],
			     priv);
	if (index == -1) {
		printk(KERN_ERR DRV_NAME ": "
		       "Unable to register the MPC512X IDE driver\n");
		ATA_RAW_WRITE(0, FSL_ATA_INTR_ENABLE);
		if (priv->dma_read_chan >= 0)
			fsl_dma_free_chan(priv->dma_read_chan);
		if (priv->dma_write_chan >= 0)
			fsl_dma_free_chan(priv->dma_write_chan);
		kfree(priv);
		return ENODEV;
	}
#ifdef ATA_USE_IORDY
	/* turn on IORDY protocol */

	udelay(25);
	ATA_RAW_WRITE(FSL_ATA_CTRL_ATA_RST_B | FSL_ATA_CTRL_IORDY_EN,
		      FSL_ATA_CONTROL);
#endif

	return 0;

	/* error exit */
err:
	irq_dispose_mapping(priv->ata_irq);
	return rv;
}

/*!
 * Driver exit routine.  Clean up.
 */
static int __exit mpc512x_ata_remove(struct of_device *op)
{
	ide_hwif_t *hwif = ifs[0];
	struct fsl_ata_priv *priv;

	BUG_ON(!hwif);
	priv = (struct fsl_ata_priv *)hwif->hwif_data;
	BUG_ON(!priv);

	/*
	 * Unregister the interface at the IDE layer.  This should shut
	 * down any drives and pending I/O.
	 */
	ide_unregister(hwif->index);

	/*
	 * Disable hardware interrupts.
	 */
	ATA_RAW_WRITE(0, FSL_ATA_INTR_ENABLE);

	/*
	 * Turn off the clock
	 */
	clk_disable(priv->ata_clk);
	clk_put(priv->ata_clk);

	/*
	 * Cleanup the irq_mapping
	 */
	irq_dispose_mapping(priv->ata_irq);

	if (priv->dma_read_chan >= 0)
		fsl_dma_free_chan(priv->dma_read_chan);
	if (priv->dma_write_chan >= 0)
		fsl_dma_free_chan(priv->dma_write_chan);
	/*
	 * Free the private structure.
	 */
	kfree(priv);
	hwif->hwif_data = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int mpc512x_ata_suspend(struct of_device *op, pm_message_t state)
{
	return 0;		/* FIXME */
}

static int mpc512x_ata_resume(struct of_device *op)
{
	return 0;		/* FIXME */
}

#else
#define mpc512x_ata_suspend NULL
#define mpc512x_ata_resume NULL
#endif

static struct of_device_id mpc512x_ata_of_match[] = {
	{
	 .compatible = "fsl,mpc5121-pata",
	 },
	{},
};

static struct of_platform_driver mpc512x_ata_of_platform_driver = {
	.owner = THIS_MODULE,
	.name = DRV_NAME,
	.match_table = mpc512x_ata_of_match,
	.probe = mpc512x_ata_probe,
	.remove = mpc512x_ata_remove,
	.suspend = mpc512x_ata_suspend,
	.resume = mpc512x_ata_resume,
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init fsl_ata_init(void)
{
	printk(KERN_INFO
	       "MPC512X: IDE driver, (c) 2004-2007 Freescale Semiconductor\n");
	return of_register_platform_driver(&mpc512x_ata_of_platform_driver);
}

static void __exit fsl_ata_exit(void)
{
	of_unregister_platform_driver(&mpc512x_ata_of_platform_driver);
}

module_init(fsl_ata_init);
module_exit(fsl_ata_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION
    ("Freescale MPC512X IDE "
     "(based on Simtec BAST IDE driver by Ben Dooks <ben@simtec.co.uk>)");
