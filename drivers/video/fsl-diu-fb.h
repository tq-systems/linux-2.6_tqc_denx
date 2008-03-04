/*
 * Copyright 2007,2008 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  Freescale DIU Frame Buffer device driver
 *
 *  Authors: Hongjun Chen <hong-jun.chen@freescale.com>
 *           Paul Widmer <paul.widmer@freescale.com>
 *           Srikanth Srinivasan <srikanth.srinivasan@freescale.com>
 *           York Sun <yorksun@freescale.com>
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

/* FIXME: This should be changed to dev_dbg this will be done as soon as
 * we can obtain dev through the dts setup
 */
#ifdef DEBUG
#define DPRINTK(fmt, args...) printk("%s: " fmt,__FUNCTION__,## args)
#else
#define DPRINTK(fmt, args...)
#endif

/* Arbitrary threshold to determine the allocation method
 * See mpc8610fb_set_par(), map_video_memory(), and unmap_video_memory()
 */
#define MEM_ALLOC_THRESHOLD (1024*768*4+32)
/* Minimum value that the pixel clock can be set to in pico seconds
 * This is determined by platform clock/3 where the minimum platform
 * clock is 533MHz. This gives 5629 pico seconds.
 */
#define MIN_PIX_CLK 5629
#define MAX_PIX_CLK 96096

#include <linux/types.h>

struct mfb_alpha {
	int enable;
	int alpha;
};

struct mfb_chroma_key {
	int enable;
	__u8  red_max;
	__u8  green_max;
	__u8  blue_max;
	__u8  red_min;
	__u8  green_min;
	__u8  blue_min;
};

struct aoi_display_offset {
	int x_aoi_d;
	int y_aoi_d;
};

#define MFB_SET_CHROMA_KEY	_IOW('M', 1, struct mfb_chroma_key)
#define MFB_WAIT_FOR_VSYNC	_IOW('F', 0x20, u_int32_t)
#define MFB_SET_BRIGHTNESS	_IOW('M', 3, __u8)

#define MFB_SET_ALPHA		0x80014d00
#define MFB_GET_ALPHA		0x40014d00
#define MFB_SET_AOID		0x80084d04
#define MFB_GET_AOID		0x40084d04

#define FBIOGET_GWINFO		0x46E0
#define FBIOPUT_GWINFO		0x46E1

#ifdef __KERNEL__
#include <linux/spinlock.h>

/*
 * These parameters give default parameters
 * for video output 1024x768,
 * FIXME - change timing to proper amounts
 * hsync 31.5kHz, vsync 60Hz
 */
static struct fb_videomode __devinitdata fsl_diu_default_mode = {
	.refresh	= 60,
	.xres		= 1024,
	.yres		= 768,
	.pixclock	= 15385,
	.left_margin	= 160,
	.right_margin	= 24,
	.upper_margin	= 29,
	.lower_margin	= 3,
	.hsync_len	= 136,
	.vsync_len	= 6,
	.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode		= FB_VMODE_NONINTERLACED
};

static struct fb_videomode __devinitdata fsl_diu_mode_db[] = {
	{
		.name		= "1024x768-60",
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 768,
		.pixclock	= 15385,
		.left_margin	= 160,
		.right_margin	= 24,
		.upper_margin	= 29,
		.lower_margin	= 3,
		.hsync_len	= 136,
		.vsync_len	= 6,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "1024x768-70",
		.refresh	= 70,
		.xres		= 1024,
		.yres		= 768,
		.pixclock	= 16886,
		.left_margin	= 3,
		.right_margin	= 3,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 40,
		.vsync_len	= 18,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "1024x768-75",
		.refresh	= 75,
		.xres		= 1024,
		.yres		= 768,
		.pixclock	= 15009,
		.left_margin	= 3,
		.right_margin	= 3,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 80,
		.vsync_len	= 32,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "1280x1024-60",
		.refresh	= 60,
		.xres		= 1280,
		.yres		= 1024,
		.pixclock	= 9375,
		.left_margin	= 38,
		.right_margin	= 128,
		.upper_margin	= 2,
		.lower_margin	= 7,
		.hsync_len	= 216,
		.vsync_len	= 37,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "1280x1024-70",
		.refresh	= 70,
		.xres		= 1280,
		.yres		= 1024,
		.pixclock	= 9380,
		.left_margin	= 6,
		.right_margin	= 6,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 60,
		.vsync_len	= 94,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "1280x1024-75",
		.refresh	= 75,
		.xres		= 1280,
		.yres		= 1024,
		.pixclock	= 9380,
		.left_margin	= 6,
		.right_margin	= 6,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 60,
		.vsync_len	= 15,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "320x240",		/* for AOI only */
		.refresh	= 60,
		.xres		= 320,
		.yres		= 240,
		.pixclock	= 15385,
		.left_margin	= 0,
		.right_margin	= 0,
		.upper_margin	= 0,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
	{
		.name		= "1280x480-60",
		.refresh	= 60,
		.xres		= 1280,
		.yres		= 480,
		.pixclock	= 18939,
		.left_margin	= 353,
		.right_margin	= 47,
		.upper_margin	= 39,
		.lower_margin	= 4,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED
	},
};
/*
 * These are the fields of area descriptor(in DDR memory) for every plane
 */
struct diu_ad {
	/* Word 0(32-bit) in DDR memory */
/* 	__u16 comp; */
/* 	__u16 pixel_s:2; */
/* 	__u16 pallete:1; */
/* 	__u16 red_c:2; */
/* 	__u16 green_c:2; */
/* 	__u16 blue_c:2; */
/* 	__u16 alpha_c:3; */
/* 	__u16 byte_f:1; */
/* 	__u16 res0:3; */

	__u32 pix_fmt; /* hard coding pixel format */

	/* Word 1(32-bit) in DDR memory */
	__u32 addr;

	/* Word 2(32-bit) in DDR memory */
/* 	__u32 delta_xs:11; */
/* 	__u32 res1:1; */
/* 	__u32 delta_ys:11; */
/* 	__u32 res2:1; */
/* 	__u32 g_alpha:8; */
	__u32 src_size_g_alpha;

	/* Word 3(32-bit) in DDR memory */
/* 	__u32 delta_xi:11; */
/* 	__u32 res3:5; */
/* 	__u32 delta_yi:11; */
/* 	__u32 res4:3; */
/* 	__u32 flip:2; */
	__u32 aoi_size;

	/* Word 4(32-bit) in DDR memory */
	/*__u32 offset_xi:11;
	__u32 res5:5;
	__u32 offset_yi:11;
	__u32 res6:5;
	*/
	__u32 offset_xyi;

	/* Word 5(32-bit) in DDR memory */
	/*__u32 offset_xd:11;
	__u32 res7:5;
	__u32 offset_yd:11;
	__u32 res8:5; */
	__u32 offset_xyd;


	/* Word 6(32-bit) in DDR memory */
	__u32 ckmax_r:8;
	__u32 ckmax_g:8;
	__u32 ckmax_b:8;
	__u32 res9:8;

	/* Word 7(32-bit) in DDR memory */
	__u32 ckmin_r:8;
	__u32 ckmin_g:8;
	__u32 ckmin_b:8;
	__u32 res10:8;
/* 	__u32 res10:8; */

	/* Word 8(32-bit) in DDR memory */
	__u32 next_ad;

	/* Word 9(32-bit) in DDR memory, just for 64-bit aligned */
	__u32 paddr;
}__attribute__ ((packed));

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
	struct diu *diu_reg;
	spinlock_t reg_lock;

	__u32 mode;		/* DIU operation mode */
};

struct diu_addr {
	__u8 __iomem *  vaddr;	/* Virtual address */
	dma_addr_t paddr;	/* Physical address */
	__u32 	   offset;
};

struct diu_pool {
	struct diu_addr ad;
	struct diu_addr gamma;
	struct diu_addr pallete;
	struct diu_addr cursor;
};

#define FSL_DIU_BASE_OFFSET	0x2C000	/* Offset of Display Interface Unit(DIU) */
#define INT_LCDC		64	/* DIU interrupt number */

#define FSL_AOI_NUM	6	/* 5 AOIs (1 for plane 0, 2 for plane 1&2 each) and one dummy AOI */

/* Minimum X and Y resolutions */
#define MIN_XRES	64
#define MIN_YRES	64

/* HW cursor parameters */
#define MAX_CURS		32

/* Modes of operation of DIU */
#define MFB_MODE0	0	/* DIU off */
#define MFB_MODE1	1	/* All three planes output to display */
#define MFB_MODE2	2	/* Plane 1 to display, planes 2+3 written back to memory */
#define MFB_MODE3	3	/* All three planes written back to memory */
#define MFB_MODE4	4	/* Color bar generation */

/* INT_STATUS/INT_MASK field descriptions */
#define INT_VSYNC	0x01	/* Vertical synchronize interrupt  */
#define INT_VSYNC_WB	0x02	/* Vertical synchronize interrupt for write back operation */
#define INT_UNDRUN	0x04	/* Under run exception interrupt */
#define INT_PARERR	0x08	/* Display parameters error interrupt */
#define INT_LS_BF_VS	0x10	/* Lines before vsync. interrupt */

/* Panels'operation modes */
#define MFB_TYPE_OUTPUT	0	/* Panel output to display */
#define MFB_TYPE_OFF	1	/* Panel off */
#define MFB_TYPE_WB	2	/* Panel written back to memory */
#define MFB_TYPE_TEST	3	/* Panel generate color bar */

void *fsl_diu_alloc(unsigned long size, dma_addr_t *phys);
void fsl_diu_free(void *p, unsigned long size);

#endif /* __KERNEL__ */
#endif /* __FSL_DIU_FB_H__ */
