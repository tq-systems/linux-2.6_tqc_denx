/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _FSL_ATA_H_
#define _FSL_ATA_H_

/*!
 * @defgroup ATA ATA/IDE Driver
 */

/*!
 * @file mpc512x_ide.h
 *
 * @brief MPC512X ATA/IDE hardware register and bit definitions.
 *
 * @ingroup ATA
 */

#define ATA_BASE_ADDR (priv->ata_regs)
#define IO_ADDRESS(addr) (addr)

#define FSL_ATA_IO   IO_ADDRESS((ATA_BASE_ADDR + 0xA0))
#define FSL_ATA_CTL  IO_ADDRESS((ATA_BASE_ADDR + 0xD8))

/*
 * Interface control registers
 */

#define FSL_ATA_FIFO_DATA_32    IO_ADDRESS((ATA_BASE_ADDR + 0x18))
#define FSL_ATA_FIFO_DATA_16    IO_ADDRESS((ATA_BASE_ADDR + 0x1C))
#define FSL_ATA_FIFO_FILL       IO_ADDRESS((ATA_BASE_ADDR + 0x20))
#define FSL_ATA_CONTROL		IO_ADDRESS((ATA_BASE_ADDR + 0x24))
#define FSL_ATA_INTR_PENDING    IO_ADDRESS((ATA_BASE_ADDR + 0x28))
#define FSL_ATA_INTR_ENABLE     IO_ADDRESS((ATA_BASE_ADDR + 0x2C))
#define FSL_ATA_INTR_CLEAR      IO_ADDRESS((ATA_BASE_ADDR + 0x30))
#define FSL_ATA_FIFO_ALARM      IO_ADDRESS((ATA_BASE_ADDR + 0x34))

/*
 * Control register bit definitions
 */

#define FSL_ATA_CTRL_FIFO_RST_B      0x80
#define FSL_ATA_CTRL_ATA_RST_B       0x40
#define FSL_ATA_CTRL_FIFO_TX_EN      0x20
#define FSL_ATA_CTRL_FIFO_RCV_EN     0x10
#define FSL_ATA_CTRL_DMA_PENDING     0x08
#define FSL_ATA_CTRL_DMA_ULTRA       0x04
#define FSL_ATA_CTRL_DMA_WRITE       0x02
#define FSL_ATA_CTRL_IORDY_EN        0x01

/*
 * Interrupt registers bit definitions
 */

#define FSL_ATA_INTR_ATA_INTRQ1      0x80
#define FSL_ATA_INTR_FIFO_UNDERFLOW  0x40
#define FSL_ATA_INTR_FIFO_OVERFLOW   0x20
#define FSL_ATA_INTR_CTRL_IDLE       0x10
#define FSL_ATA_INTR_ATA_INTRQ2      0x08

/*
 * timing registers
 */

#define FSL_ATA_TIME_OFF        IO_ADDRESS((ATA_BASE_ADDR + 0x00))
#define FSL_ATA_TIME_ON         IO_ADDRESS((ATA_BASE_ADDR + 0x01))
#define FSL_ATA_TIME_1          IO_ADDRESS((ATA_BASE_ADDR + 0x02))
#define FSL_ATA_TIME_2w         IO_ADDRESS((ATA_BASE_ADDR + 0x03))

#define FSL_ATA_TIME_2r         IO_ADDRESS((ATA_BASE_ADDR + 0x04))
#define FSL_ATA_TIME_AX         IO_ADDRESS((ATA_BASE_ADDR + 0x05))
#define FSL_ATA_TIME_PIO_RDX    IO_ADDRESS((ATA_BASE_ADDR + 0x06))
#define FSL_ATA_TIME_4          IO_ADDRESS((ATA_BASE_ADDR + 0x07))

#define FSL_ATA_TIME_9          IO_ADDRESS((ATA_BASE_ADDR + 0x08))
#define FSL_ATA_TIME_M          IO_ADDRESS((ATA_BASE_ADDR + 0x09))
#define FSL_ATA_TIME_JN         IO_ADDRESS((ATA_BASE_ADDR + 0x0A))
#define FSL_ATA_TIME_D          IO_ADDRESS((ATA_BASE_ADDR + 0x0B))

#define FSL_ATA_TIME_K          IO_ADDRESS((ATA_BASE_ADDR + 0x0C))
#define FSL_ATA_TIME_ACK        IO_ADDRESS((ATA_BASE_ADDR + 0x0D))
#define FSL_ATA_TIME_ENV        IO_ADDRESS((ATA_BASE_ADDR + 0x0E))
#define FSL_ATA_TIME_RPX        IO_ADDRESS((ATA_BASE_ADDR + 0x0F))

#define FSL_ATA_TIME_ZAH        IO_ADDRESS((ATA_BASE_ADDR + 0x10))
#define FSL_ATA_TIME_MLIX       IO_ADDRESS((ATA_BASE_ADDR + 0x11))
#define FSL_ATA_TIME_DVH        IO_ADDRESS((ATA_BASE_ADDR + 0x12))
#define FSL_ATA_TIME_DZFS       IO_ADDRESS((ATA_BASE_ADDR + 0x13))

#define FSL_ATA_TIME_DVS        IO_ADDRESS((ATA_BASE_ADDR + 0x14))
#define FSL_ATA_TIME_CVH        IO_ADDRESS((ATA_BASE_ADDR + 0x15))
#define FSL_ATA_TIME_SS         IO_ADDRESS((ATA_BASE_ADDR + 0x16))
#define FSL_ATA_TIME_CYC        IO_ADDRESS((ATA_BASE_ADDR + 0x17))

/*
 * other facts
 */
#define FSL_ATA_DMA_WATERMARK		32	/* XXX fixme */

#define MPC512X_DMA_ATA_RX		27	/* XXX fixme */
#define MPC512X_DMA_ATA_TX		28	/* XXX fixme */

#define FSL_DMA_DONE			0	/* XXX fixme */
#define FSL_DMA_REQUEST_TIMEOUT		1	/* XXX fixme */
#define FSL_DMA_TRANSFER_ERROR		2	/* XXX fixme */

#define FSL_DMA_MODE_READ		0	/* XXX fixme */
#define FSL_DMA_MODE_WRITE		1	/* XXX fixme */

#define FSL_ATA_DMA_BD_SIZE_MAX 0xFC00	/* max size of scatterlist segment */

/*! Private data for the drive structure. */
struct fsl_ata_priv {
	struct device *dev;	/*!< The device */
	int dma_read_chan;	/*!< DMA channel sdma api gave us for reads */
	int dma_write_chan;	/*!< DMA channel sdma api gave us for writes */
	int ultra;		/*!< Remember when we're in ultra mode */
	int dma_stat;		/*!< the state of DMA request */
	u8 enable;		/*!< Current hardware interrupt mask */
	void *ata_regs;		/*!< Base of ata registers */
	dma_addr_t dma_addr;	/* physical address for dma rd/wr */

	int ata_irq;		/*!< ATA irq number */
	struct fsl_dma_requestbuf
	 dma_reqbufs[MAX_TCD_NUM_PER_CH];
	struct clk *ata_clk;	/*!< rate/pm clk */
};

/*! ATA transfer mode for set_ata_bus_timing() */
enum ata_mode {
	PIO,			/*!< Specifies PIO mode */
	MDMA,			/*!< Specifies MDMA mode */
	UDMA			/*!< Specifies UDMA mode */
};

#define	INTRQ_MCU 0		/* Enable ATA_INTRQ on the CPU */
#define INTRQ_DMA 1		/* Enable ATA_INTRQ on the DMA engine */

/*!
 *  This structure defines the bits in the ATA TIME_CONFIGx
 */
union fsl_ata_time_cfg {
	unsigned long config;
	struct {
		unsigned char field4;
		unsigned char field3;
		unsigned char field2;
		unsigned char field1;
	} bytes;
};

/*!defines the macro for accessing the register */
#define ATA_RAW_WRITE(v, addr)	writel(v, addr)
#define ATA_RAW_READ(addr)	readl(addr)
/*! Get the configuration of TIME_CONFIG0 */
#define GET_TIME_CFG(t, base) ((t)->config = ATA_RAW_READ(base))

/*! Set the configuration of TIME_CONFIG0.
 * And mask is ignored. base is the start address of this configuration.
 */
#define SET_TIME_CFG(t, mask, base) (ATA_RAW_WRITE((t)->config, base))
#endif /* !_FSL_ATA_H_ */
