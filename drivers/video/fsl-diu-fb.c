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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <asm/of_platform.h>
#include <asm/mpc512x.h>
#include "fsl-diu-fb.h"

#undef DEBUG

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_INFO "%s: " fmt, \
					__FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#endif

static char *fb_mode;
static int fb_enabled;
static unsigned long default_bpp = 16;
static ATOMIC_NOTIFIER_HEAD(fsl_diu_notifier_list);
static struct clk *lcdc_clk;
static int irq;
static u32 * pcoherency;

struct mfb_info {
	int index;
	int type;
	char *id;
	int registered;
	int blank;
	unsigned long *pseudo_palette;
	int palette_size;
	unsigned char g_alpha;
	struct diu_ad *ad;
	int cursor_reset;
};

static int fsl_diu_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info);
static int fsl_diu_set_par(struct fb_info *info);
static int fsl_diu_setcolreg(unsigned regno, unsigned red, unsigned green,
			     unsigned blue, unsigned transp,
			     struct fb_info *info);
static int fsl_diu_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info);
static int fsl_diu_blank(int blank_mode, struct fb_info *info);
static int fsl_diu_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg);
static void disable_lcdc(struct fb_info *info);

#ifndef MODULE
static int __init fsl_diu_setup(char *);
#endif

static int map_video_memory(struct fb_info *info);
static void disable_panel(struct fb_info *info);

#ifdef CONFIG_PM
static int fsl_diu_of_suspend(struct of_device *op, pm_message_t state);
static int fsl_diu_of_resume(struct of_device *op);
#else
#define fsl_diu_suspend	0
#define fsl_diu_resume	0
#endif

static int fsl_diu_of_probe(struct of_device *op,
			    const struct of_device_id *match);

extern void *fsl_diu_alloc(unsigned long size, dma_addr_t *phys);

struct mfb_info mfbi_p0 = {
	.index = 0,
	.type = MFB_TYPE_OUTPUT,
	.id = "DISP0 Panel0",
	.g_alpha = 0,
	.registered = 0,
};

struct mfb_info mfbi_p1 = {
	.index = 1,
	.type = MFB_TYPE_OUTPUT,
	.id = "DISP0 Panel1",
	.g_alpha = 0xff,
	.registered = 0,
};

struct mfb_info mfbi_p2 = {
	.index = 2,
	.type = MFB_TYPE_OUTPUT,
	.id = "DISP0 Panel2",
	.g_alpha = 0xff,
	.registered = 0,
};

static struct diu_hw dr = {
	.mode = MFB_MODE1,
	.reg_lock = SPIN_LOCK_UNLOCKED,
};

struct diu_pool pool;

static struct fb_info fsl_diu_info[] = {
	{.par = &mfbi_p0},
/* FIXME it can't allocate too much for frame buffer */
#if !defined(CONFIG_MPC5121_ADS)
	{.par = &mfbi_p1},
	{.par = &mfbi_p2},
#endif
};

static struct fb_ops fsl_diu_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = fsl_diu_check_var,
	.fb_set_par = fsl_diu_set_par,
	.fb_setcolreg = fsl_diu_setcolreg,
	.fb_blank = fsl_diu_blank,
	.fb_pan_display = fsl_diu_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = fsl_diu_ioctl,
};

static struct of_device_id fsl_diu_of_match[] = {
	{.type = "display", .compatible = "fsl-diu",},
	{},
};

MODULE_DEVICE_TABLE(of, fsl_diu_of_match);

static struct of_platform_driver fsl_diu_of_driver = {
	.owner = THIS_MODULE,
	.name = "fsl-diu",
	.match_table = fsl_diu_of_match,
	.probe = fsl_diu_of_probe,
#ifdef CONFIG_PM
	.suspend = fsl_diu_of_suspend,
	.resume = fsl_diu_of_resume,
#endif
	.driver = {
		   .name = "fsl-diu",
		   },
};

/*
 * Checks to see if the hardware supports the state requested by var passed
 * in. This function does not alter the hardware state! If the var passed in
 * is slightly off by what the hardware can support then we alter the var
 * PASSED in to what we can do. If the hardware doesn't support mode change
 * a -EINVAL will be returned by the upper layers.
 */
static int fsl_diu_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	unsigned long htotal, vtotal;
	DPRINTK("Entered: fsl_diu_check_var\n");

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if (var->xoffset < 0)
		var->xoffset = 0;

	if (var->yoffset < 0)
		var->yoffset = 0;

	if (var->xoffset + info->var.xres > info->var.xres_virtual)
		var->xoffset = info->var.xres_virtual - info->var.xres;

	if (var->yoffset + info->var.yres > info->var.yres_virtual)
		var->yoffset = info->var.yres_virtual - info->var.yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16)) {
		var->bits_per_pixel = default_bpp;
	}

	switch (var->bits_per_pixel) {
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;

		break;
	}

	if (var->pixclock < MIN_PIX_CLK) {
		htotal = var->xres + var->right_margin + var->hsync_len +
		    var->left_margin;
		vtotal = var->yres + var->lower_margin + var->vsync_len +
		    var->upper_margin;
		var->pixclock = (vtotal * htotal * 6UL) / 100UL;
		var->pixclock = KHZ2PICOS(var->pixclock);
		DPRINTK("pixclock set for 60Hz refresh = %u ps\n",
			var->pixclock);
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	/* Copy nonstd field to/from sync for fbset usage */
	var->sync |= var->nonstd;
	var->nonstd |= var->sync;

	return 0;
}

static void set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;

	DPRINTK("Entered: set_fix\n");
	strncpy(fix->id, mfbi->id, strlen(mfbi->id));
	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;
}

static void unmap_video_memory(struct fb_info *info)
{
	DPRINTK("Entered: unmap_video_memory\n");
	dma_free_coherent(0, info->fix.smem_len, info->screen_base,
			  (dma_addr_t) info->fix.smem_start);

	info->screen_base = 0;
	info->fix.smem_start = 0;
	info->fix.smem_len = 0;
}

static u32 get_busfreq(void)
{
	struct device_node *node;
	u32 fs_busfreq = 0;
	unsigned int size;
	const unsigned int *prop;

	node = of_find_node_by_type(NULL, "cpu");
	if (node) {
		prop = of_get_property(node, "bus-frequency", &size);
		if (prop)
			fs_busfreq = *prop;
		of_node_put(node);
	}

	return fs_busfreq;
}


static void update_lcdc(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu_ad *ad = mfbi->ad;
	struct device_node *np;
	struct resource r;
	struct diu *hw;
	int i, j, index = mfbi->index;
	char __iomem *cursor_base, *gamma_table_base;

	/* variables for pixel clock calcs */
	unsigned long bestval, bestfreq, speed_ccb;
	unsigned long minpixclock, maxpixclock, pixclock, pixval;
	long err;
	u32 temp, adjust_value = 0;
	u32 *clkdvdr;

	DPRINTK("Entered: update_lcdc\n");

	spin_lock_init(&dr.reg_lock);
	hw = dr.diu_reg;

	if (mfbi->type == MFB_TYPE_OFF) {
		disable_panel(info);
		return;
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,mpc5121-immr");
	of_address_to_resource(np, 0, &r);
	of_node_put(np);
	DPRINTK("r.start: 0x%08x\n", r.start);

#if defined(CONFIG_MPC5121_ADS)
	clkdvdr = (u32 *) ioremap(r.start + 0xf0c, sizeof(u32));
	speed_ccb = get_busfreq() * 4;
#elif defined(CONFIG_MPC8610_HPCD)
	clkdvdr = (u32 *) ioremap(r.start + 0xe0800, sizeof(u32));
	speed_ccb = get_busfreq();
	adjust_value = 1;
#endif
	gamma_table_base = pool.gamma.vaddr;
	cursor_base = pool.cursor.vaddr;
	disable_lcdc(info);

	/* Prep for DIU init  - gamma table */
	for (i = 0; i <= 2; i++)
		for (j = 0; j <= 255; j++)
			*gamma_table_base++ = j;

	DPRINTK("update-lcdc: HW - %p\n Disabling DIU\n", hw);
	out_be32(&(hw->diu_mode), 0);

	/* Program DIU registers */
	out_be32(&(hw->desc[index]), ad->paddr);
	DPRINTK("DIU ad base address: 0x%08x\n", ad->paddr);
	out_be32(&(hw->desc[1]), 0);
	out_be32(&(hw->desc[2]), 0);
	out_be32(&(hw->gamma), pool.gamma.paddr);
	out_be32(&(hw->cursor), pool.cursor.paddr);
	out_be32(&(hw->bgnd), 0x007F7F7F);	/* BGND */
	out_be32(&(hw->bgnd_wb), 0);		/* BGND_WB */
	out_be32(&(hw->disp_size), (var->yres << 16 | var->xres));
	/* DISP SIZE */
	DPRINTK("DIU xres: %d\n", var->xres);
	DPRINTK("DIU yres: %d\n", var->yres);
	out_be32(&(hw->wb_size), 0);	/* WB SIZE */
	out_be32(&(hw->wb_mem_addr), 0);	/* WB MEM ADDR */

	/* Horizontal and vertical configuration register */
	temp = var->left_margin << 22 |	/* BP_H */
	    var->hsync_len << 11 |	/* PW_H */
	    var->right_margin;		/* FP_H */
	out_be32(&(hw->hsyn_para), temp);

	temp = var->upper_margin << 22 |	/* BP_V */
	    var->vsync_len << 11 |		/* PW_V  */
	    var->lower_margin;			/* FP_V  */
	out_be32(&(hw->vsyn_para), temp);

	DPRINTK("DIU right_margin - %d\n", var->right_margin);
	DPRINTK("DIU left_margin - %d\n", var->left_margin);
	DPRINTK("DIU hsync_len - %d\n", var->hsync_len);
	DPRINTK("DIU upper_margin - %d\n", var->upper_margin);
	DPRINTK("DIU lower_margin - %d\n", var->lower_margin);
	DPRINTK("DIU vsync_len - %d\n", var->vsync_len);
	DPRINTK("DIU HSYNC - 0x%08ux\n", hw->hsyn_para);
	DPRINTK("DIU VSYNC - 0x%08ux\n", hw->vsyn_para);
	DPRINTK("DIU HSYNC - 0x%08u\n", hw->hsyn_para);
	DPRINTK("DIU VSYNC - 0x%08u\n", hw->vsyn_para);

	/* Set up pixel clock */
	/* Calculate the pixel clock with the smallest error */
	/* calculate the following in steps to avoid overflow */
	DPRINTK("DIU pixclock in ps - %d\n", var->pixclock);
	temp = 1;
	temp *= 1000000000;
	temp /= var->pixclock;
	temp *= 1000;
	pixclock = temp;
	DPRINTK("DIU pixclock freq - %lu\n", pixclock);

	temp *= 5;
	temp /= 100;		/* pixclock * 0.05 */
	DPRINTK("deviation = %d\n", temp);
	minpixclock = pixclock - temp;
	maxpixclock = pixclock + temp;
	DPRINTK("DIU minpixclock - %lu\n", minpixclock);
	DPRINTK("DIU maxpixclock - %lu\n", maxpixclock);
	pixval = speed_ccb / pixclock;
	DPRINTK("DIU pixval = %lu\n", pixval);

	err = 100000000;
	bestval = pixval;
	DPRINTK("DIU bestval = %lu\n", bestval);
	bestfreq = 0;
	for (i = -1; i <= 1; i++) {
		temp = speed_ccb / ((pixval + i) + adjust_value);
		DPRINTK("DIU test pixval i= %d, pixval=%lu, temp freq. = %d\n",
			i, pixval, temp);
		if ((temp < minpixclock) || (temp > maxpixclock))
			DPRINTK("DIU exceeds monitor range (%lu to %lu)\n",
				minpixclock, maxpixclock);
		else if (abs(temp - pixclock) < err) {
			DPRINTK("Entered the else if block %d\n", i);
			err = abs(temp - pixclock);
			bestval = pixval + i;
			bestfreq = temp;
		}
	}

	DPRINTK("DIU chose = %lu\n", bestval);
	DPRINTK("DIU error = %lu\n NomPixClk ", err);
	DPRINTK("DIU: Best Freq = %lu\n", bestfreq);

	out_be32(&(hw->syn_pol), 0);	/* SYNC SIGNALS POLARITY */
	out_be32(&(hw->thresholds), 0x00037800);	/* The Thresholds */
	out_be32(&(hw->int_status), 0);	/* INTERRUPT STATUS */
	out_be32(&(hw->int_mask), ~(INT_UNDRUN | INT_VSYNC));	/* INT MASK */
	out_be32(&(hw->plut), 0x001F5F66);

#if defined(CONFIG_MPC8610_HPCD)
	/* Modify PXCLK in GUTS CLKDVDR */
	temp = in_be32(clkdvdr) & 0xA000FFFF;
	out_be32(clkdvdr, temp | (((bestval) & 0x1F) << 16));
#else				/* For MPC5121ADS */
	temp = in_be32(clkdvdr) & 0xffffff00;
	out_be32(clkdvdr, temp | bestval);
#endif
	iounmap(clkdvdr);
	/* Enable the DIU */
	out_be32(&(hw->diu_mode), 0x01);
}

/*
 * Using the fb_var_screeninfo in fb_info we set the resolution of this
 * particular framebuffer. This function alters the fb_fix_screeninfo stored
 * in fb_info. It doesn't not alter var in fb_info since we are using that
 * data. This means we depend on the data in var inside fb_info to be
 * supported by the hardware. fsl_diu_check_var is always called before
 * fsl_diu_set_par to ensure this.
 */
static int fsl_diu_set_par(struct fb_info *info)
{
	unsigned long len;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct diu_ad *ad = mfbi->ad;
	struct diu *hw;
	int index = mfbi->index;

	DPRINTK("Entered: fsl_diu_set_par\n");
	hw = dr.diu_reg;

	set_fix(info);
	mfbi->cursor_reset = 1;

	len = info->var.yres_virtual * info->fix.line_length;
	if (len > info->fix.smem_len) {
		if (info->fix.smem_start)
			unmap_video_memory(info);
		/* Memory allocation for framebuffer */
		if (map_video_memory(info)) {
			printk("Unable to allocate fb memory\n");
			return -ENOMEM;
		}
	}

	out_be32(&(hw->desc[index]), 0);

	if (var->bits_per_pixel == 32)
		ad->pix_fmt = 0x8888cb01;
	else if (var->bits_per_pixel == 24)
		ad->pix_fmt = 0x88083218;
	else if (var->bits_per_pixel == 16)
		ad->pix_fmt = 0x65052908;

	ad->addr = cpu_to_le32(info->fix.smem_start);
	ad->src_size_g_alpha =
	    cpu_to_le32((var->yres << 12) | var->xres) | mfbi->g_alpha;
	ad->aoi_size =
	    cpu_to_le32((var->yres_virtual << 16) | var->xres_virtual);
	/* Disable chroma keying function */
	ad->ckmax_r = 0;
	ad->ckmax_g = 0;
	ad->ckmax_b = 0;

	ad->ckmin_r = 255;
	ad->ckmin_g = 255;
	ad->ckmin_b = 255;

	/* This is the last area descriptor */
	ad->next_ad = 0;
	if (mfbi->index == 0)
		update_lcdc(info);

	return 0;
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

/*
 * Set a single color register. The values supplied have a 16 bit magnitude
 * which needs to be scaled in this function for the hardware. Things to take
 * into consideration are how many color registers, if any, are supported with
 * the current color visual. With truecolor mode no color palettes are
 * supported. Here a psuedo palette is created which we store the value in
 * pseudo_palette in struct fb_info. For pseudocolor mode we have a limited
 * color palette.
 */
static int fsl_diu_setcolreg(unsigned regno, unsigned red, unsigned green,
			     unsigned blue, unsigned transp,
			     struct fb_info *info)
{
	int ret = 1;
	DPRINTK("Entered: fsl_diu_setcoloreg\n");

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 v;

#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)
			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);
#undef CNVT_TOHW
			v = (red << info->var.red.offset) |
			    (green << info->var.green.offset) |
			    (blue << info->var.blue.offset) |
			    (transp << info->var.transp.offset);

			pal[regno] = v;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

/*
 * Pan (or wrap, depending on the `vmode' field) the display using the
 * 'xoffset' and 'yoffset' fields of the 'var' structure. If the values
 * don't fit, return -EINVAL.
 */
static int fsl_diu_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	DPRINTK("Entered: fsl_diu_pan_display\n");
	if ((info->var.xoffset == var->xoffset) &&
	    (info->var.yoffset == var->yoffset)) {
		return 0;	/* No change, do nothing */
	}

	if (var->xoffset < 0 || var->yoffset < 0
	    || var->xoffset + info->var.xres > info->var.xres_virtual
	    || var->yoffset + info->var.yres > info->var.yres_virtual)
		return -EINVAL;

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP) {
		info->var.vmode |= FB_VMODE_YWRAP;
	} else {
		info->var.vmode &= ~FB_VMODE_YWRAP;
	}
	return 0;
}

static void enable_panel(struct fb_info *info)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu *hw = dr.diu_reg;
	struct diu_ad *ad = mfbi->ad;

	DPRINTK("Entered: enable_panel\n");
	if (mfbi->type != MFB_TYPE_OFF)
		out_be32(&(hw->desc[mfbi->index]), ad->paddr);
}

static void disable_panel(struct fb_info *info)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: disable_panel\n");
	out_be32(&(hw->desc[mfbi->index]), 0);
}

static void enable_lcdc(struct fb_info *info)
{
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: enable_lcdc\n");
	if (!fb_enabled) {
#if defined(CONFIG_MPC5121_ADS)
		clk_enable(lcdc_clk);
#endif
		out_be32(&(hw->diu_mode), dr.mode);
		fb_enabled++;
	}
}

static void disable_lcdc(struct fb_info *info)
{
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: disable_lcdc\n");
	if (fb_enabled) {
		out_be32(&(hw->diu_mode), 0);
		fb_enabled = 0;
	}
}

/*
 * Blank the screen if blank_mode != 0, else unblank. Return 0 if blanking
 * succeeded, != 0 if un-/blanking failed.
 * blank_mode == 2: suspend vsync
 * blank_mode == 3: suspend hsync
 * blank_mode == 4: powerdown
 */
static int fsl_diu_blank(int blank_mode, struct fb_info *info)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;

	DPRINTK("Entered: fsl_diu_blank\n");
	mfbi->blank = blank_mode;

	switch (blank_mode) {
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		/* disable_panel(info); */
		break;
	case FB_BLANK_POWERDOWN:
		/* disable_lcdc(info); */
		break;
	case FB_BLANK_UNBLANK:
		/* FIXME */
		/* enable_panel(info); */
		/* enable_lcdc(info); */
		break;
	}

	return 0;
}

static int fsl_diu_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct diu_ad *ad = mfbi->ad;
	struct mfb_chroma_key ck;
	unsigned char global_alpha;

	DPRINTK("Entered: fsl_diu_ioctl\n");
	switch (cmd) {
	case MFB_GET_ALPHA:
		if (!arg)
			return -EINVAL;
		global_alpha = mfbi->g_alpha;
		if (copy_to_user
		    ((void *)arg, (void *)&global_alpha, sizeof(global_alpha)))
			return -EFAULT;
		DPRINTK("get global alpha of index %d\n", mfbi->index);
		break;
	case MFB_SET_ALPHA:
		if (!arg)
			return -EINVAL;

		/* set panel information */
		if (copy_from_user
		    ((void *)&global_alpha, (void *)arg, sizeof(global_alpha)))
			return -EFAULT;
		ad->src_size_g_alpha =
		    (ad->src_size_g_alpha & (~0xff)) | (global_alpha & 0xff);
		mfbi->g_alpha = global_alpha;
		DPRINTK("set global alpha for index %d\n", mfbi->index);
		break;
	case MFB_SET_CHROMA_KEY:
		if (mfbi->type != MFB_TYPE_OFF)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* set panel winformation */
		if (copy_from_user((void *)&ck, (void *)arg, sizeof(ck)))
			return -EFAULT;

		if (ck.enable &&
		    (ck.red_max < ck.red_min ||
		     ck.green_max < ck.green_min || ck.blue_max < ck.blue_min))
			return -EINVAL;

		if (!ck.enable) {
			ad->ckmax_r = 0;
			ad->ckmax_g = 0;
			ad->ckmax_b = 0;
			ad->ckmin_r = 255;
			ad->ckmin_g = 255;
			ad->ckmin_b = 255;
		} else {
			ad->ckmax_r = ck.red_max;
			ad->ckmax_g = ck.green_max;
			ad->ckmax_b = ck.blue_max;
			ad->ckmin_r = ck.red_min;
			ad->ckmin_g = ck.green_min;
			ad->ckmin_b = ck.blue_min;
		}

		enable_panel(info);
		break;
	case FBIOGET_GWINFO:
		if (mfbi->type == MFB_TYPE_OFF)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* get graphic window information */
		if (copy_to_user((void *)arg, (void *)ad, sizeof(*ad)))
			return -EFAULT;
		break;
	case FBIOPUT_GWINFO:
		if (mfbi->type == MFB_TYPE_OFF)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* set graphic window information */
		if (copy_from_user((void *)ad, (void *)arg, sizeof(*ad)))
			return -EFAULT;

		enable_panel(info);
		break;

	case FBIOGET_VSCREENINFO:
		printk(KERN_INFO "%s, FBIOGET_VSCREENINFO: 0x%08x\n",
		       __func__, FBIOGET_VSCREENINFO);
		if (mfbi->type == MFB_TYPE_OFF)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		if (copy_to_user((void *)arg, (void *)var, sizeof(*var)))
			return -EFAULT;
		break;
	case FBIOGET_HWCINFO:
		printk(KERN_INFO "%s, FBIOGET_HWCINFO:0x%08x\n", __func__,
		       FBIOGET_HWCINFO);
		break;
	case FBIOPUT_MODEINFO:
		printk(KERN_INFO "%s, FBIOPUT_MODEINFO:0x%08x\n", __func__,
		       FBIOPUT_MODEINFO);
		break;
	case FBIOGET_DISPINFO:
		printk(KERN_INFO "%s, FBIOGET_DISPINFO:0x%08x\n", __func__,
		       FBIOGET_DISPINFO);
		break;

	default:
		printk(KERN_ERR "Unknown ioctl command (0x%08X)\n", cmd);
		return 0;
	}

	return 0;
}

static int init_fbinfo(struct fb_info *info, struct device *pdev)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;

	DPRINTK("Entered: init_fbinfo\n");
	info->device = NULL;
	info->var.activate = FB_ACTIVATE_NOW;
	info->fbops = &fsl_diu_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = mfbi->pseudo_palette;

	/* Allocate colormap */
	fb_alloc_cmap(&info->cmap, 16, 0);
	return 0;
}

static int install_fb(struct fb_info *info, struct device *pdev)
{
	int rc;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;

	DPRINTK("Entered: install_fb\n");
	if (init_fbinfo(info, pdev))
		return -EINVAL;

	fb_mode = "1024x768-32@60";
	/* fb_mode = NULL; */
	/* fb_mode = "800x600-32@60"; */
	DPRINTK("3mode used = %s\n", fb_mode);
	rc = fb_find_mode(&info->var, info, fb_mode, NULL,
			  0, &fsl_diu_default_mode, 32);

	switch (rc) {
	case 1:
		DPRINTK("using mode specified in @mode\n");
		break;
	case 2:
		DPRINTK("using mode specified in @mode with"
			" ignored refresh rate\n");
		break;
	case 3:
		DPRINTK("using mode default mode\n");
		break;
	case 4:
		DPRINTK("using mode from list\n");
		break;
	default:
		DPRINTK("rc = %d\n", rc);
		DPRINTK("failed to find mode\n");
		return -EBUSY;
	}

	DPRINTK("xres_virtual %d\n", info->var.xres_virtual);
	DPRINTK("bits_per_pixel %d\n", info->var.bits_per_pixel);

	DPRINTK("info->var.yres_virtual0 = %d\n", info->var.yres_virtual);
	DPRINTK("info->fix.line_length0  = %d\n", info->fix.line_length);

	if (mfbi->type == MFB_TYPE_OFF)
		mfbi->blank = FB_BLANK_NORMAL;
	else
		mfbi->blank = FB_BLANK_UNBLANK;

	if (fsl_diu_check_var(&info->var, info)) {
		printk(KERN_ERR "fb_check_var failed");
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	if (fsl_diu_set_par(info)) {
		printk(KERN_ERR "fb_set_par failed");
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR "register_framebuffer failed");
		unmap_video_memory(info);
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	mfbi->registered = 1;
	printk(KERN_INFO "fb%d: %s fb device registered successfully.\n",
	       info->node, info->fix.id);

	return 0;
}

static void __exit uninstall_fb(struct fb_info *info)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;

	DPRINTK("Entered: uninstall_fb\n");
	if (!mfbi->registered)
		return;

	unregister_framebuffer(info);
	unmap_video_memory(info);
	if (&info->cmap)
		fb_dealloc_cmap(&info->cmap);

	mfbi->registered = 0;
}

static int map_video_memory(struct fb_info *info)
{
	DPRINTK("Entered: map_video_memory\n");
	info->fix.smem_len = info->fix.line_length * info->var.yres_virtual;
	info->screen_base = fsl_diu_alloc(info->fix.smem_len,
					 (dma_addr_t *) &info->fix.smem_start);
	if (info->screen_base == 0) {
		printk(KERN_ERR "Unable to allocate fb memory\n");
		return -EBUSY;
	}

	info->screen_size = info->fix.smem_len;

	/* Clear the screen */
	memset((char *)info->screen_base, 0, info->fix.smem_len);

	return 0;
}

static irqreturn_t fsl_diu_isr(int irq, void *dev_id)
{
	struct diu *hw = dr.diu_reg;
	unsigned int status = in_be32(&(hw->int_status));
	u32 *pcohn = pcoherency;
	int i;

	DPRINTK("Entered: fsl_diu_isr, status: %x\n", status);
	if (status) {
		/* This is the workaround for underrun */
		if (status & INT_UNDRUN) {
			out_be32(&(hw->diu_mode), 0);
			DPRINTK("Err: DIU occurs underrun!\n");
			udelay(1);
			out_be32(&(hw->diu_mode), 1);
		} else if ((status & INT_VSYNC) && pcohn) {
			for (i = 0; i < COHERENCY_SIZE / 4; i ++)
				*pcohn++ = i;
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/* FIXME: need to properly request IRQ will be used in future */
static void request_irq_local(void)
{
	unsigned long status;
	unsigned long flags;
	struct diu *hw;
	struct device_node *np;

	DPRINTK("Entered: request_irq_local\n");
	hw = dr.diu_reg;

	/* Read to clear the status */
	status = in_be32(&(hw->int_status));

	np = of_find_node_by_type(NULL, "display");
	if (!np) {
		pr_info
		   ("Err: didn't find node 'lcd-controller' in device tree!\n");
		return;
	}

	irq = irq_of_parse_and_map(np, 0);
	of_node_put(np);

	if (request_irq(irq, fsl_diu_isr, 0, "LCDC", 0))
		pr_info("Request LCDC IRQ failed.\n");
	else {
		spin_lock_irqsave(&fsl_diu_notifier_list.lock, flags);

		/* Enable interrupt in case client has registered */
		if (fsl_diu_notifier_list.head != NULL) {
			unsigned long status;
			unsigned long ints = INT_VSYNC | INT_PARERR;

			if (dr.mode == MFB_MODE2 || dr.mode == MFB_MODE3)
				ints |= INT_VSYNC_WB;

			/* Read to clear the status */
			status = in_be32(&(hw->int_status));

			/* Enable EOF, parameter error, and optional
			 * write back interrupt
			 */
			out_be32(&(hw->int_mask), ints);
		}

		spin_unlock_irqrestore(&fsl_diu_notifier_list.lock, flags);
	}
}

static void free_irq_local(void)
{
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: free_irq_local\n");
	/* Disable all LCDC interrupt */
	out_be32(&(hw->int_mask), 0x0);

	free_irq(irq, 0);
}

int fsl_diu_register_client(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;
	struct diu *hw = dr.diu_reg;
	DPRINTK("Entered: fsl_diu_register_client\n");

	ret = atomic_notifier_chain_register(&fsl_diu_notifier_list, nb);

	spin_lock_irqsave(&fsl_diu_notifier_list.lock, flags);

	/* Enable interrupt in case client has registered */
	if (fsl_diu_notifier_list.head != NULL) {
		unsigned long status;
		unsigned long ints = INT_VSYNC | INT_PARERR;

		if (dr.mode == MFB_MODE2 || dr.mode == MFB_MODE3)
			ints |= INT_VSYNC_WB;

		/* Read to clear the status */
		status = in_be32(&(hw->int_status));

		/* Enable EOF, parameter error, and optional
		 * write back interrupt
		 */
		out_be32(&(hw->int_mask), ints);
	}

	spin_unlock_irqrestore(&fsl_diu_notifier_list.lock, flags);

	return ret;
}
EXPORT_SYMBOL(fsl_diu_register_client);

int fsl_diu_unregister_client(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: fsl_diu_unregister_client\n");
	ret = atomic_notifier_chain_unregister(&fsl_diu_notifier_list, nb);

	spin_lock_irqsave(&fsl_diu_notifier_list.lock, flags);

	/* Mask interrupt in case no client registered */
	if (fsl_diu_notifier_list.head == NULL)
		out_be32(&(hw->int_mask), 0);

	spin_unlock_irqrestore(&fsl_diu_notifier_list.lock, flags);

	return ret;
}
EXPORT_SYMBOL(fsl_diu_unregister_client);

#ifdef CONFIG_PM
/*
 * Power management hooks. Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int fsl_diu_of_suspend(struct of_device *op, pm_message_t state)
{
	DPRINTK("Entered: fsl_diu_suspend\n");
	disable_lcdc(&fsl_diu_info[0]);

	return 0;
}

static int fsl_diu_of_resume(struct of_device *op)
{
	DPRINTK("Entered: fsl_diu_resume\n");
	enable_lcdc(&fsl_diu_info[0]);

	return 0;
}
#endif				/* CONFIG_PM */

/* Align to 64-bit(8-byte), 32-byte, etc. */
static int allocate_buf(struct diu_addr *buf, u32 size, u32 bytes_align)
{
	u32 offset, ssize;
	u32 mask;

	DPRINTK("Entered: allocate_buf\n");
	ssize = size + bytes_align;
	buf->vaddr = dma_alloc_coherent(0, ssize, &(buf->paddr),
					GFP_DMA | GFP_KERNEL);
	if (!buf->vaddr)
		return -ENOMEM;

	memset(buf->vaddr, 0, ssize);

	mask = bytes_align - 1;
	offset = (u32) buf->paddr & mask;
	if (offset) {
		buf->offset = bytes_align - offset;
		buf->paddr = (u32) buf->paddr + buf->offset;
	} else
		buf->offset = 0;
	return 0;
}

static void free_buf(struct diu_addr *buf, u32 size, u32 bytes_align)
{
	DPRINTK("Entered: free_buf\n");
	dma_free_coherent(0, size + bytes_align,
			  buf->vaddr, (buf->paddr - buf->offset));
	return;
}

static int fsl_diu_of_probe(struct of_device *op,
			    const struct of_device_id *match)
{
	struct mfb_info *mfbi;
	struct device_node *np;
	struct resource r;
	u32 *preg, base, end, size;
	int ret, i;
	u32 coherency;
#ifdef CONFIG_MPC5121_ADS
	u32 *clk_io, *clk_delay;
	struct ddr512x *mddrc;
#endif

	DPRINTK("Entered: fsl_diu_probe\n");

#ifdef CONFIG_MPC5121_ADS
	np = of_find_compatible_node(NULL, NULL, "fsl,mpc5121-immr");
	of_address_to_resource(np, 0, &r);
	of_node_put(np);

	lcdc_clk = clk_get(&op->dev, "diu_clk");
	if(!lcdc_clk) {
		printk("Err: without find diu_clk!\n");
		return -1;
	}
	clk_enable(lcdc_clk);

	/* Configure the delay of pixel clock */
	clk_delay = ioremap(r.start + 0xf54, sizeof(u32));
	*clk_delay |= 0x01000000;

	/* Configure DIU clock pin */
	clk_io = ioremap(r.start + 0xa284, sizeof(u32));
	*clk_io &= ~0x1ff;
	*clk_io |= MPC5121_IO_FUNC3 | MPC5121_IO_DS_4;

	/* Initialize IO pins (pin mux) for DIU function */
	base = r.start + MPC5121_IO_OFFSET;
	size = MPC5121_IO_DIU_END - MPC5121_IO_DIU_START + 4;
	end = base + MPC5121_IO_DIU_END;
	preg = (u32 *) ioremap(base + MPC5121_IO_DIU_START, size);
	while (size > 0) {
		*preg |= (MPC5121_IO_FUNC3 | MPC5121_IO_DS_4);
		preg++;
		size -= 4;
	}

	/* Workaround for underrun problem */
	mddrc = (struct ddr512x *) ioremap(r.start + MPC512x_DDR_BASE,
					sizeof(struct ddr512x));

	/* Initialize DDR Priority Manager */
	mddrc->prioman_config1 = CFG_MDDRCGRP_PM_CFG1;
	mddrc->prioman_config2 = CFG_MDDRCGRP_PM_CFG2;
	mddrc->hiprio_config = CFG_MDDRCGRP_HIPRIO_CFG;
	mddrc->lut_table0_main_upper = CFG_MDDRCGRP_LUT0_MU;
	mddrc->lut_table0_main_lower = CFG_MDDRCGRP_LUT0_ML;
	mddrc->lut_table1_main_upper = CFG_MDDRCGRP_LUT1_MU;
	mddrc->lut_table1_main_lower = CFG_MDDRCGRP_LUT1_ML;
	mddrc->lut_table2_main_upper = CFG_MDDRCGRP_LUT2_MU;
	mddrc->lut_table2_main_lower = CFG_MDDRCGRP_LUT2_ML;
	mddrc->lut_table3_main_upper = CFG_MDDRCGRP_LUT3_MU;
	mddrc->lut_table3_main_lower = CFG_MDDRCGRP_LUT3_ML;
	mddrc->lut_table4_main_upper = CFG_MDDRCGRP_LUT4_MU;
	mddrc->lut_table4_main_lower = CFG_MDDRCGRP_LUT4_ML;
	mddrc->lut_table0_alternate_upper = CFG_MDDRCGRP_LUT0_AU;
	mddrc->lut_table0_alternate_lower = CFG_MDDRCGRP_LUT0_AL;
	mddrc->lut_table1_alternate_upper = CFG_MDDRCGRP_LUT1_AU;
	mddrc->lut_table1_alternate_lower = CFG_MDDRCGRP_LUT1_AL;
	mddrc->lut_table2_alternate_upper = CFG_MDDRCGRP_LUT2_AU;
	mddrc->lut_table2_alternate_lower = CFG_MDDRCGRP_LUT2_AL;
	mddrc->lut_table3_alternate_upper = CFG_MDDRCGRP_LUT3_AU;
	mddrc->lut_table3_alternate_lower = CFG_MDDRCGRP_LUT3_AL;
	mddrc->lut_table4_alternate_upper = CFG_MDDRCGRP_LUT4_AU;
	mddrc->lut_table4_alternate_lower = CFG_MDDRCGRP_LUT4_AL;

	iounmap(clk_delay);
	iounmap(clk_io);
	iounmap(preg);
	iounmap(mddrc);
#endif

	/* Area descriptor memory pool aligns to 64-bit boundary */
	allocate_buf(&pool.ad, sizeof(struct diu_ad) * FSL_PANEL_NUM, 8);

	/* Get memory for Gamma Table  - 32-byte aligned memory */
	allocate_buf(&pool.gamma, 768, 32);

	/* For high performance, cursor bitmap buffer aligns to
	 * 32-byte boundary
	 */
	allocate_buf(&pool.cursor, MAX_CURS * MAX_CURS * 2, 32);

	/* Allocate memory buffer for pallete */
	allocate_buf(&pool.pallete, 256 * sizeof(u64), 32);

	/* Install ISR to handle underrun interrupt */
	request_irq_local();
	pcoherency = fsl_diu_alloc(COHERENCY_SIZE, &coherency);

	for (i = 0; i < sizeof(fsl_diu_info) / sizeof(struct fb_info); i++) {
		mfbi = (struct mfb_info *)fsl_diu_info[i].par;
		mfbi->ad = (struct diu_ad *)((u32) pool.ad.vaddr
					     + pool.ad.offset) + i;
		mfbi->ad->paddr = pool.ad.paddr + i * sizeof(struct diu_ad);

		/* Set up pallete */
		mfbi->pseudo_palette = (unsigned long *)(pool.pallete.vaddr +
							 pool.pallete.offset);
		ret = install_fb(&fsl_diu_info[i], &op->dev);
		if (ret) {
			printk(KERN_ERR "Failed to register"
			       " framebuffer %d\n", i);
			return ret;
		}
	}

	return 0;
}

int __init fsl_diu_init(void)
{
	struct device_node *np;
	struct resource r;
	int error;

	DPRINTK("Entered: fsl_diu_init\n");
	np = of_find_compatible_node(NULL, NULL, "fsl-diu");
	if (!np) {
		printk(KERN_ERR "Err: can't find device node 'fsl-diu'\n");
		return -ENODEV;
	}

	of_address_to_resource(np, 0, &r);
	of_node_put(np);

	DPRINTK("%s, r.start: 0x%08x\n", __func__, r.start);

	dr.diu_reg = (struct diu __iomem *)ioremap(r.start, sizeof(struct diu));
	spin_lock_init(&dr.reg_lock);

	/*
	 * For kernel boot options (in 'video=xxxfb:<options>' format)
	 */
#ifndef MODULE
	{
		char *option;

		if (fb_get_options("fsl-diu", &option))
			return -ENODEV;
		fsl_diu_setup(option);
	}
#endif
	error = of_register_platform_driver(&fsl_diu_of_driver);
	if (error)
		printk(KERN_ERR "Err: can't register FB device driver!\n");
	else
		printk(KERN_INFO "FSL-DIU-FB: registed FB device driver!\n");

	return error;
}

void __exit fsl_diu_exit(void)
{
	int i;

	DPRINTK("Entered: fsl_diu_exit\n");
	/* free_irq_local(); */
	for (i = sizeof(fsl_diu_info) / sizeof(struct fb_info); i > 0; i--)
		uninstall_fb(&fsl_diu_info[i - 1]);

	of_unregister_platform_driver(&fsl_diu_of_driver);
	iounmap(dr.diu_reg);

	free_buf(&pool.ad, sizeof(struct diu_ad) * FSL_PANEL_NUM, 8);
	free_buf(&pool.cursor, MAX_CURS * MAX_CURS * 2, 32);
	free_irq_local();
	return;
}

#ifndef MODULE
static int __init fsl_diu_setup(char *options)
{
	char *opt;

	DPRINTK("Entered: fsl_diu_setup\n");
	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;

		if (!strncmp(opt, "bpp=", 4))
			default_bpp = simple_strtoul(opt + 4, NULL, 0);
		else
			fb_mode = opt;
	}

	return 0;
}
#endif

module_init(fsl_diu_init);
module_exit(fsl_diu_exit);

MODULE_AUTHOR("Hongjun Chen <Hong-jun.chen@freescale>");
MODULE_DESCRIPTION("FSL DIU framebuffer driver");
MODULE_LICENSE("GPL");
