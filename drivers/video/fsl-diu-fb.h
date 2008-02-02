/*
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  Freescale DIU Frame Buffer device driver
 *
 *  Author: Hongjun Chen <hong-jun.chen@freescale.com>
 *	    Paul Widmer <paul.widmer@freescale.com>
 *	    Srikanth Srinivasan <srikanth.srinivasan@freescale.com>
 *  Copyright (C) Freescale Semicondutor, Inc. 2007. All rights reserved.
 *
 *   Based on imxfb.c Copyright (C) 2004 S.Hauer, Pengutronix
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __FSL_DIU_FB_H__
#define __FSL_DIU_FB_H__

#include <linux/types.h>

struct mfb_alpha {
	int enable;
	int alpha;
};

struct mfb_chroma_key {
	int enable;
	u8 red_max;
	u8 green_max;
	u8 blue_max;
	u8 red_min;
	u8 green_min;
	u8 blue_min;
};

/* Minimum value that the pixel clock can be set to in pico seconds */
#define MIN_PIX_CLK 10000

#define MFB_SET_ALPHA		0x80014d00
#define MFB_GET_ALPHA		0x40014d00
#define MFB_SET_CHROMA_KEY	_IOW('M', 1, struct mfb_chroma_key)
#define MFB_WAIT_FOR_VSYNC	_IOW('F', 0x20, u_int32_t)
#define MFB_SET_BRIGHTNESS	_IOW('M', 3, __u8)

#define FBIOGET_GWINFO		0x46E0
#define FBIOPUT_GWINFO		0x46E1

#ifdef __KERNEL__
#include <linux/spinlock.h>

/*
 * These parameters give default parameters
 * for video output
 * 1024x768,
 * FIXME - change timing to proper amounts
 * hsync 31.5kHz, vsync 60Hz
 */
static struct fb_videomode __devinitdata fsl_diu_default_mode = {
	.refresh = 62,
	.xres = 1280,
	.yres = 768,
	.pixclock = 15151,
	.left_margin = 20,
	.right_margin = 20,
	.upper_margin = 5,
	.lower_margin = 5,
	.hsync_len = 30,
	.vsync_len = 12,
	.sync = FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED
};

/*
 * These are the fields of area descriptor(in DDR memory) for every plane
 */
struct diu_ad {
	/* Word 0(32-bit) in DDR memory */
	__le32 pix_fmt;		/* hard coding pixel format */

	/* Word 1(32-bit) in DDR memory */
	__le32 addr;

	/* Word 2(32-bit) in DDR memory */
	__le32 src_size_g_alpha;

	/* Word 3(32-bit) in DDR memory */
	__le32 aoi_size;

	/* Word 4(32-bit) in DDR memory */
	__le32 offset_xyi;

	/* Word 5(32-bit) in DDR memory */
	__le32 offset_xyd;

	/* Word 6(32-bit) in DDR memory */
	__le32 ckmax_r:8;
	__le32 ckmax_g:8;
	__le32 ckmax_b:8;
	__le32 res9:8;

	/* Word 7(32-bit) in DDR memory */
	__le32 res10:8;
	__le32 ckmin_r:8;
	__le32 ckmin_g:8;
	__le32 ckmin_b:8;

	/* Word 8(32-bit) in DDR memory */
	__le32 next_ad;

	/* Word 9(32-bit) in DDR memory, just for 64-bit aligned */
	__le32 paddr;
} __attribute__ ((packed));

/* DIU register map */
struct diu {
	__be32 desc[3];
	__be32 gamma;
	__be32 pallete;
	__be32 cursor;
	__be32 curs_pos;
	__be32 diu_mode;
	__be32 bgnd;
	__be32 bgnd_wb;
	__be32 disp_size;
	__be32 wb_size;
	__be32 wb_mem_addr;
	__be32 hsyn_para;
	__be32 vsyn_para;
	__be32 syn_pol;
	__be32 thresholds;
	__be32 int_status;
	__be32 int_mask;
	__be32 colorbar[8];
	__be32 filling;
	__be32 plut;
} __attribute__ ((packed));

struct diu_hw {
	struct diu __iomem *diu_reg;
	spinlock_t reg_lock;	/* Proctect hardware register space */

	u32 mode;		/* DIU operation mode */
};

struct diu_addr {
	u8 __iomem *vaddr;	/* Virtual address */
	dma_addr_t paddr;	/* Physical address */
	u32 offset;
};

struct diu_pool {
	struct diu_addr ad;
	struct diu_addr gamma;
	struct diu_addr pallete;
	struct diu_addr cursor;
};

#define FSL_DIU_BASE_OFFSET	0x2C000	/* Offset of Display Interface
					 * Unit(DIU)
					 */
#define INT_LCDC		64	/* DIU interrupt number */

#define FSL_PANEL_NUM	3	/* Number of panels */

/* Minimum X and Y resolutions */
#define MIN_XRES	64
#define MIN_YRES	64

/* HW cursor parameters */
#define MAX_CURS		32

/* Modes of operation of DIU */
#define MFB_MODE0	1	/* DIU off */
#define MFB_MODE1	2	/* All three planes output to display */
#define MFB_MODE2	3	/* Plane 1 to display, planes 2+3 written
				 * back to memory
				 */
#define MFB_MODE3	4	/* All three planes written back to memory */
#define MFB_MODE4	5	/* Color bar generation */

/* INT_STATUS/INT_MASK field descriptions */
#define INT_VSYNC	0x01	/* Vertical synchronize interrupt  */
#define INT_VSYNC_WB	0x02	/* Vertical synchronize interrupt for write
				 * back operation
				 */
#define INT_UNDRUN	0x04	/* Under run exception interrupt */
#define INT_PARERR	0x08	/* Display parameters error interrupt */
#define INT_LS_BF_VS	0x10	/* Lines before vsync. interrupt */

/* Panels'operation modes */
#define MFB_TYPE_OUTPUT	0	/* Panel output to display */
#define MFB_TYPE_OFF	1	/* Panel off */
#define MFB_TYPE_WB	2	/* Panel written back to memory */
#define MFB_TYPE_TEST	3	/* Panel generate color bar */

#ifdef CONFIG_MPC5121_ADS
/* Functional pin muxing */
#define MPC5121_IO_FUNC1        (0 << 7)
#define MPC5121_IO_FUNC2        (1 << 7)
#define MPC5121_IO_FUNC3        (2 << 7)
#define MPC5121_IO_FUNC4        (3 << 7)
#define MPC5121_IO_ST           (1 << 2)
#define MPC5121_IO_DS_1		(0)
#define MPC5121_IO_DS_2		(1)
#define MPC5121_IO_DS_3		(2)
#define MPC5121_IO_DS_4		(3)

/* DIU IO pin muxing registers' offset address */
#define MPC5121_IO_OFFSET       0x0a000	/* Offset from IMMRBAR */
#define MPC5121_IO_DIU_START    0x288
#define MPC5121_IO_DIU_END      0x2fc
#endif				/* CONFIG_MPC5121_ADS */

/* Workaround priority settings for DIU underrun */
#define CFG_MDDRCGRP_PM_CFG1	0x00077777
#define CFG_MDDRCGRP_PM_CFG2	0x00000000
#define CFG_MDDRCGRP_HIPRIO_CFG	0x00000001
#define CFG_MDDRCGRP_LUT0_MU    0xFFEEDDCC
#define CFG_MDDRCGRP_LUT0_ML	0xBBAAAAAA
#define CFG_MDDRCGRP_LUT1_MU    0x66666666
#define CFG_MDDRCGRP_LUT1_ML	0x55555555
#define CFG_MDDRCGRP_LUT2_MU    0x44444444
#define CFG_MDDRCGRP_LUT2_ML	0x44444444
#define CFG_MDDRCGRP_LUT3_MU    0x55555555
#define CFG_MDDRCGRP_LUT3_ML	0x55555558
#define CFG_MDDRCGRP_LUT4_MU    0x11111111
#define CFG_MDDRCGRP_LUT4_ML	0x11111122
#define CFG_MDDRCGRP_LUT0_AU    0xaaaaaaaa
#define CFG_MDDRCGRP_LUT0_AL	0xaaaaaaaa
#define CFG_MDDRCGRP_LUT1_AU    0x66666666
#define CFG_MDDRCGRP_LUT1_AL	0x66666666
#define CFG_MDDRCGRP_LUT2_AU    0x11111111
#define CFG_MDDRCGRP_LUT2_AL	0x11111111
#define CFG_MDDRCGRP_LUT3_AU    0x11111111
#define CFG_MDDRCGRP_LUT3_AL	0x11111111
#define CFG_MDDRCGRP_LUT4_AU    0x11111111
#define CFG_MDDRCGRP_LUT4_AL	0x11111111

#define COHERENCY_SIZE		(32 * 1024)

#endif				/* __KERNEL__ */
#endif				/* __FSL_DIU_FB_H__ */
