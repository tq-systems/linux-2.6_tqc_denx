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
#ifndef _FSL_REGISTERS_H_
#define _FSL_REGISTERS_H_

/* Macro definitions */
#define FSL_DMA_CH_NUM		64	/* Total channel number */
#define FSL_DMA_DESC_NUM_PER_CH	64	/* Descriptors per channel */
#define FSL_DMA_CH_NUM_IN_GROUP 16	/* Channel number in one group */
#define FSL_DMA_GROUP_NUM 	(FSL_DMA_CH_NUM / FSL_DMA_CH_NUM_IN_GROUP)
					/* Channel number in one group */

#define FSL_DMA_TCD_OFFSET	0x1000	/* TCD(transfer control descriptor)
					   area offset from IMMR
					 */
#define FSL_DMA_IRQ		65	/* DMA engine interrupt number */

/* Arbitration mode of group and channel */
#define FSL_DMA_GROUP_FIX	0x01	/* Fixed group arbitration, 0 for
					   round robin mode
					 */
#define FSL_DMA_CH_FIX		0x02	/* Fixed channel arbitration, 0 for
					   round robin mode
					 */

#define FSL_DMA_PRI_IN_USE	1	/* Hack: the global structure of
					   priority is in use
					 */
#define FSL_DMA_PRI_NOT_USE	0	/* Hack: the global structure of
					   priority is free
					 */

/* Register offset macro definitions */
#define FSL_DMA_DMACR_ERGA_RR	(1<<3)	/* Enable Round Robin Group
					   Arbitration */
#define FSL_DMA_DMACR_ERCA_RR	(1<<2)	/* Enable Round Robin Channel
					   Arbitration */

#define FSL_DMA_GPR3PRI(pri)	(pri<<14)	/* Channel Group 3 Priority */
#define FSL_DMA_GPR2PRI(pri)	(pri<<12)	/* Channel Group 2 Priority */
#define FSL_DMA_GPR1PRI(pri)	(pri<<10)	/* Channel Group 1 Priority */
#define FSL_DMA_GPR0PRI(pri)	(pri<<8)	/* Channel Group 0 Priority */

#define FSL_DMA_CH_PREEMPT	(1<<7)	/* Enable channel preemption */

#define FSL_DMA_DMAES_VLD	(1<<31)	/* At least one DMAERR bit is set */
#define FSL_DMA_DMAES_GPE	(1<<15)	/* Group priority error */
#define FSL_DMA_DMAES_CPE	(1<<14)	/* Channel priority error */
#define FSL_DMA_DMAES_ERRCHN(err) \
    ((err >> 8) & 0x3f)			/* Error channel number */
#define FSL_DMA_DMAES_SAE	(1<<7)	/* Source address error */
#define FSL_DMA_DMAES_SOE	(1<<6)	/* Source offset configuration */
#define FSL_DMA_DMAES_DAE	(1<<5)	/* Destination address error */
#define FSL_DMA_DMAES_DOE	(1<<4)	/* Destination offset error */
#define FSL_DMA_DMAES_NCE	(1<<3)	/* Nbytes/citer config error */
#define FSL_DMA_DMAES_SGE	(1<<2)	/* Scatter/gather config error */
#define FSL_DMA_DMAES_SBE	(1<<1)	/* Source bus error */
#define FSL_DMA_DMAES_DBE	(1<<0)	/* Destination bus error */

/* MPC5121 DMA engine registers */
typedef struct _fsl_dma_reg {
	/* 0x00 */
	u32 dmacr;		/* DMA control register */
	u32 dmaes;		/* DMA error status */
	/* 0x08 */
	u32 dmaerqh;		/* DMA enable request high(channels 63~32) */
	u32 dmaerql;		/* DMA enable request low(channels 31~0) */
	u32 dmaeeih;		/* DMA enable error interrupt high(ch63~32) */
	u32 dmaeeil;		/* DMA enable error interrupt low(ch31~0) */
	/* 0x18 */
	u8 dmaserq;		/* DMA set enable request */
	u8 dmacerq;		/* DMA clear enable request */
	u8 dmaseei;		/* DMA set enable error interrupt */
	u8 dmaceei;		/* DMA clear enable error interrupt */
	/* 0x1c */
	u8 dmacint;		/* DMA clear interrupt request */
	u8 dmacerr;		/* DMA clear error */
	u8 dmassrt;		/* DMA set start bit */
	u8 dmacdne;		/* DMA clear DONE status bit */
	/* 0x20 */
	u32 dmainth;		/* DMA interrupt request high(ch63~32) */
	u32 dmaintl;		/* DMA interrupt request low(ch31~0) */
	u32 dmaerrh;		/* DMA error high(ch63~32) */
	u32 dmaerrl;		/* DMA error low(ch31~0) */
	/* 0x30 */
	u32 dmahrsh;		/* DMA hw request status high(ch63~32) */
	u32 dmahrsl;		/* DMA hardware request status low(ch31~0) */
	u32 dmaihsa;		/* DMA interrupt high select AXE(ch63~32) */
	u32 dmailsa;		/* DMA interrupt low select AXE(ch31~0) */
	/* 0x40 ~ 0xff */
	u32 reserve0[48];	/* Reserved */
	/* 0x100 */
	u8 dchpri[FSL_DMA_CH_NUM];
	/* DMA channels(0~63) priority */
} __attribute__ ((packed)) fsl_dma_reg;

typedef struct _tcd {
	/* 0x00 */
	u32 saddr;		/* Source address */

	u32 smod:5;		/* Source address modulo */
	u32 ssize:3;		/* Source data transfer size */
	u32 dmod:5;		/* Destination address modulo */
	u32 dsize:3;		/* Destination data transfer size */
	u32 soff:16;		/* Signed source address offset */

	/* 0x08 */
	u32 nbytes;		/* Inner "minor" byte count */
	u32 slast;		/* Last source address adjustment */
	u32 daddr;		/* Destination address */

	/* 0x14 */
	u32 citer_elink:1;	/* Enable channel-to-channel linking on
				 * minor loop complete
				 */
	u32 citer_linkch:6;	/* Link channel for minor loop complete */
	u32 citer:9;		/* Current "major" iteration count */
	u32 doff:16;		/* Signed destination address offset */

	/* 0x18 */
	u32 dlast_sga;		/* Last Destination address adjustment/scatter
				 * gather address
				 */

	/* 0x1c */
	u32 biter_elink:1;	/* Enable channel-to-channel linking on major
				 * loop complete
				 */
	u32 biter_linkch:6;
	u32 biter:9;		/* Beginning "major" iteration count */
	u32 bwc:2;		/* Bandwidth control */
	u32 major_linkch:6;	/* Link channel number */
	u32 done:1;		/* Channel done */
	u32 active:1;		/* Channel active */
	u32 major_elink:1;	/* Enable channel-to-channel linking on major
				 * loop complete
				 */
	u32 e_sg:1;		/* Enable scatter/gather processing */
	u32 d_req:1;		/* Disable request */
	u32 int_half:1;		/* Enable an interrupt when major counter is
				 * half complete
				 */
	u32 int_maj:1;		/* Enable an interrupt when major iteration
				 * count completes
				 */
	u32 start:1;		/* Channel start */
} __attribute__ ((packed)) TCD;

typedef struct _fsl_dma_tcd {
	/* 0x1000 */
	TCD tcd[FSL_DMA_CH_NUM];
} fsl_dma_tcd;
#endif				/* _FSL_REGISTERS_H_ */
