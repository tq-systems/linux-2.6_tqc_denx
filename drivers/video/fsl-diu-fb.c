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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <asm/uaccess.h>

#include <asm/of_platform.h>

#include <sysdev/fsl_soc.h>
#undef DEBUG
#include "fsl-diu-fb.h"

static int irq;
static char *fb_mode = "1024x768-32@60";
static int fb_enabled = 0;
static unsigned long default_bpp = 32;
static ATOMIC_NOTIFIER_HEAD(fsl_diu_notifier_list);
static int monitor_port = 0;
static struct diu_ad * dummy_ad;
void * dummy_aoi_virt;

#if defined(CONFIG_NOT_COHERENT_CACHE)
unsigned int * coherence_data;
dma_addr_t * coherence_data_phy; 
#endif

struct mfb_info {
	int index;
	int type;
	char *id;
	int registered;
	int blank;
	unsigned long pseudo_palette[16];
	struct diu_ad *ad;
	int cursor_reset;
	unsigned char g_alpha;
	unsigned int count;
	int x_aoi_d;		/* aoi display x offset to physical screen */
	int y_aoi_d;		/* aoi display y offset to physical screen */
};

/* FIXME: Change static declarations and rearrange functions */
static int fsl_diu_open(struct fb_info *info, int user);
static int fsl_diu_release(struct fb_info *info, int user);
static int fsl_diu_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int fsl_diu_set_par(struct fb_info *info);
static int fsl_diu_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info);
static int fsl_diu_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info);
static int fsl_diu_blank(int blank_mode, struct fb_info *info);
static int fsl_diu_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg);
static int fsl_diu_cursor(struct fb_info *info, struct fb_cursor *cursor);

int __init fsl_diu_init(void);
void __exit fsl_diu_exit(void);
#ifndef MODULE
static int __init fsl_diu_setup(char *);
#endif

static int init_fbinfo(struct fb_info *info,
			struct platform_device *pdev);
static int install_fb(struct fb_info *info,
			struct platform_device *pdev);
static void __exit uninstall_fb(struct fb_info *info);
static int map_video_memory(struct fb_info *info);
static void unmap_video_memory(struct fb_info *info);
static void set_fix(struct fb_info *info);
static void enable_lcdc(struct fb_info *info);
static void disable_lcdc(struct fb_info *info);
static void update_lcdc(struct fb_info *info);
static void request_irq_local(void);
static void free_irq_local(void);
static int fsl_diu_enable_panel(struct fb_info *info);
static int fsl_diu_disable_panel(struct fb_info *info);
static int fsl_diu_probe(struct platform_device *pdev);

#ifdef CONFIG_PM
static int fsl_diu_suspend(struct platform_device *pdev, pm_message_t state);
static int fsl_diu_resume(struct platform_device *pdev);
#else
#define fsl_diu_suspend	0
#define fsl_diu_resume	0
#endif

extern unsigned int platform_get_pixel_format(unsigned int bits_per_pixel, int monitor_port);
extern void platform_set_gamma_table(int monitor_port, char *gamma_table_base);
extern void platform_set_pixel_clock(unsigned int pixclock);
extern ssize_t platform_show_monitor_port(int monitor_port, char * buf);
extern int platform_set_monitor_port(int val);
extern int platform_set_sysfs_monitor_port(int val);

struct mfb_info mfbi_p0 = {
	.index = 0,
	.type = MFB_TYPE_OUTPUT,
	.id = "Panel0",
	.registered = 0,
	.count = 0,
	.x_aoi_d = 0,
	.y_aoi_d = 0,
};

struct mfb_info mfbi_p1_0 = {
	.index = 1,
	.type = MFB_TYPE_OUTPUT,
	.id = "Panel1 AOI0",
	.registered = 0,
	.g_alpha = 0xff,
	.count = 0,
	.x_aoi_d = 0,
	.y_aoi_d = 0,
};

struct mfb_info mfbi_p1_1 = {
	.index = 2,
	.type = MFB_TYPE_OUTPUT,
	.id = "Panel1 AOI1",
	.registered = 0,
	.g_alpha = 0xff,
	.count = 0,
	.x_aoi_d = 0,
	.y_aoi_d = 480,
};

struct mfb_info mfbi_p2_0 = {
	.index = 3,
	.type = MFB_TYPE_OUTPUT,
	.id = "Panel2 AOI0",
	.registered = 0,
	.g_alpha = 0xff,
	.count = 0,
	.x_aoi_d = 640,
	.y_aoi_d = 0,
};

struct mfb_info mfbi_p2_1 = {
	.index = 4,
	.type = MFB_TYPE_OUTPUT,
	.id = "Panel2 AOI1",
	.registered = 0,
	.g_alpha = 0xff,
	.count = 0,
	.x_aoi_d = 640,
	.y_aoi_d = 480,
};

static struct diu_hw dr = {
	.mode = MFB_MODE1,
	.reg_lock = SPIN_LOCK_UNLOCKED,
};

struct diu_pool pool;

static struct fb_info fsl_diu_info[] = {
	{.par = &mfbi_p0},
	{.par = &mfbi_p1_0},
	{.par = &mfbi_p1_1},
	{.par = &mfbi_p2_0},
	{.par = &mfbi_p2_1},
};

static struct platform_driver fsl_diu_driver = {
	.driver = {
		   .name = "fsl_diu",
		   .owner = THIS_MODULE,
		   .bus = &platform_bus_type,
		   },
	.probe = fsl_diu_probe,
	.suspend = fsl_diu_suspend,
	.resume = fsl_diu_resume,
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
	.fb_open = fsl_diu_open,
	.fb_release = fsl_diu_release,
};

static void write_reg(u32 *reg, u32 value)
{
	unsigned long flag;

	spin_lock_irqsave(&dr.reg_lock, flag);
	*reg = value;
	spin_unlock_irqrestore(&dr.reg_lock, flag);
}

static u32 read_reg(u32 *reg)
{
	u32 value;
	unsigned long flag;

	spin_lock_irqsave(&dr.reg_lock, flag);
	value = *reg;
	spin_unlock_irqrestore(&dr.reg_lock, flag);
	return value;
}

/* turn on fb if count == 1
 */
static int fsl_diu_open(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	int res = 0;

	mfbi->count++;
	if (mfbi->count == 1) {
		DPRINTK("open plane index %d\n",mfbi->index);
		fsl_diu_check_var(&info->var,info);
		fsl_diu_set_par(info);
		res = fsl_diu_enable_panel(info);
		if (res < 0)
			mfbi->count--;
	}

	return res;
}

/* turn off fb if count == 0
 */
static int fsl_diu_release(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	int res = 0;

	mfbi->count--;
	if (mfbi->count == 0) {
		DPRINTK("release plane index %d\n",mfbi->index);
		res = fsl_diu_disable_panel(info);
		if (res < 0)
			mfbi->count++;
	}

	return res;
}

/*
 * Checks to see if the hardware supports the state requested by var passed
 * in. This function does not alter the hardware state! If the var passed in
 * is slightly off by what the hardware can support then we alter the var
 * PASSED in to what we can do. If the hardware doesn't support mode change
 * a -EINVAL will be returned by the upper layers.
 */
static int fsl_diu_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned long htotal, vtotal;
	struct mfb_info *pmfbi, *cmfbi, *mfbi = (struct mfb_info *)info->par;
	int index = mfbi->index;

	DPRINTK("Entered: fsl_diu_check_var\n");
	DPRINTK("check_var xres: %d\n",var->xres);
	DPRINTK("check_var yres: %d\n",var->yres);

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
		var->red.offset = 0;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 16;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
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
	/* If the pixclock is below the minimum spec'd value then set to
	 * refresh rate for 60Hz since this is supported by most monitors.
	 * Refer to Documentation/fb/ for calculations.
	 */
	if ((var->pixclock < MIN_PIX_CLK) || (var->pixclock > MAX_PIX_CLK)) {
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

	/* check AOI position */
	switch (index) {
	case 0:
		if (mfbi->x_aoi_d != 0)
			mfbi->x_aoi_d = 0;
		if (mfbi->y_aoi_d != 0)
			mfbi->y_aoi_d = 0;
		break;
	case 1:			/* AOI 0 */
	case 3:
		cmfbi = (struct mfb_info *)fsl_diu_info[index+1].par;
		if ((mfbi->x_aoi_d + var->xres) > fsl_diu_info[0].var.xres)
			mfbi->x_aoi_d = fsl_diu_info[0].var.xres - var->xres;
		if (mfbi->x_aoi_d < 0)
			mfbi->x_aoi_d = 0;
		if ((var->xres + mfbi->x_aoi_d) > fsl_diu_info[0].var.xres)
			var->xres = fsl_diu_info[0].var.xres - mfbi->x_aoi_d;

		if (cmfbi->count > 0) {		/* AOI1 is open */
			if ((mfbi->y_aoi_d + var->yres) > cmfbi->y_aoi_d)
				mfbi->y_aoi_d = cmfbi->y_aoi_d - var->yres;
			if (mfbi->y_aoi_d < 0)
				mfbi->y_aoi_d = 0;
			if ((var->yres + mfbi->y_aoi_d) > cmfbi->y_aoi_d)
				var->yres = cmfbi->y_aoi_d - mfbi->y_aoi_d;
		}
		else {				/* AOI1 is close */
			if ((mfbi->y_aoi_d + var->yres) > fsl_diu_info[0].var.yres)
				mfbi->y_aoi_d = fsl_diu_info[0].var.yres - var->yres;
			if (mfbi->y_aoi_d < 0)
				mfbi->y_aoi_d = 0;
			if ((var->yres + mfbi->y_aoi_d) > fsl_diu_info[0].var.yres)
				var->yres = fsl_diu_info[0].var.yres - mfbi->y_aoi_d;
		}
		break;
	case 2:			/* AOI 1 */
	case 4:
		pmfbi = (struct mfb_info *)fsl_diu_info[index-1].par;
		if ((mfbi->x_aoi_d + var->xres) > fsl_diu_info[0].var.xres)
			mfbi->x_aoi_d = fsl_diu_info[0].var.xres - var->xres;
		if (mfbi->x_aoi_d < 0)
			mfbi->x_aoi_d = 0;
		if ((var->xres + mfbi->x_aoi_d) > fsl_diu_info[0].var.xres)
			var->xres = fsl_diu_info[0].var.xres - mfbi->x_aoi_d;

		if (pmfbi->count > 0) {		/* AOI0 is open */
			if ((mfbi->y_aoi_d + var->yres) > fsl_diu_info[0].var.yres)
				mfbi->y_aoi_d = fsl_diu_info[0].var.yres - var->yres;
			if (mfbi->y_aoi_d < (pmfbi->y_aoi_d+fsl_diu_info[index-1].var.yres))
				mfbi->y_aoi_d = pmfbi->y_aoi_d+fsl_diu_info[index-1].var.yres;
			if ((var->yres + mfbi->y_aoi_d) > fsl_diu_info[0].var.yres)
				var->yres = fsl_diu_info[0].var.yres - mfbi->y_aoi_d;
		}
		else {				/* AOI0 is close */
			if ((mfbi->y_aoi_d + var->yres) > fsl_diu_info[0].var.yres)
				mfbi->y_aoi_d = fsl_diu_info[0].var.yres - var->yres;
			if (mfbi->y_aoi_d < 0)
				mfbi->y_aoi_d = 0;
			if ((var->yres + mfbi->y_aoi_d) > fsl_diu_info[0].var.yres)
				var->yres = fsl_diu_info[0].var.yres - mfbi->y_aoi_d;
		}
		break;
	}
	return 0;
}

static u8 comp_index(u32 *col_offset, u32 offset, u8 num)
{
	int i, index;

	for(i = 0, index = 0; i < num; i++)
		if(offset > col_offset[i]) index++;

	return index;
}

static void color_comp_locate(struct diu_ad *ad, struct fb_var_screeninfo *var)
{
	u32 col_offset[4], index, num;
	u32 rgb_c, comp;
	DPRINTK("Entered: color_comp_locate\n");

	col_offset[0] = var->red.offset;
	col_offset[1] = var->green.offset;
	col_offset[2] = var->blue.offset;
	col_offset[3] = var->transp.offset;

	/* No alpha component exists */
	if(!var->transp.offset && !var->transp.length)
		num = 3;
	else
		num = 4;

	index = comp_index(&col_offset[0], var->red.offset, num);
	rgb_c = (index << 19);
	comp = var->red.length << (index * 4);

	index = comp_index(&col_offset[0], var->green.offset, num);
	rgb_c |= (index << 21);
	comp |= var->green.length << (index * 4);

	index = comp_index(&col_offset[0], var->blue.offset, num);
	rgb_c |= (index << 23);
	comp |= var->blue.length << (index * 4);

	if(num == 3)
		index = 3;
	else {
		index = comp_index(&col_offset[0], var->transp.offset, num);
		rgb_c |= (index << 25);
		comp |= var->transp.length << (index * 4);
	}
}

static void hide_cursor(struct fb_info *info)
{
	struct diu *pdiu = dr.diu_reg;
	pdiu->cursor = 0;
}

static void show_cursor(struct fb_info *info)
{
	struct diu *pdiu = dr.diu_reg;
	pdiu->cursor = pool.cursor.paddr + pool.cursor.offset;
}

/*
 * Copies a cursor image from user space to the proper place in driver
 * memory so that the hardware can display the cursor image *
 */
static void load_cursor_image(struct fb_info *info, u8 *data8,
				     u16 bg, u16 fg, u32 w, u32 h)
{
	u16 *dst = (u16 *)(pool.cursor.vaddr + pool.cursor.offset);
	int i, j;
	u32 b;
	u16 tmp;
	u32 *data = (u32 *)data8;

	w = (w + 1) & ~1;

	for (i = 0; i < 32; i++) {
		for (j = 0; j < 32; j++) {
			b = *data++;
			tmp = (b & (1 << 31)) ? b : bg;
			*dst = cpu_to_be16(tmp);
			dst ++;
		}
	}
}

/*
 * A cursor function that supports displaying a cursor image via hardware.
 * Within the kernel, copy and invert rops are supported.  If exported
 * to user space, only the copy rop will be supported.
 */
static int fsl_diu_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu *pdiu = dr.diu_reg;
	u8 data[MAX_CURS * MAX_CURS/8];
	int i, set = cursor->set;
	u16 fg, bg;

	if (cursor->image.width > MAX_CURS || cursor->image.height > MAX_CURS)
		return -EINVAL;

	hide_cursor(info);

	if (mfbi->cursor_reset) {
		set = FB_CUR_SETALL;
		mfbi->cursor_reset = 0;
	}

	if (set & FB_CUR_SETSIZE)
		memset_io(pool.cursor.vaddr + pool.cursor.offset,
				0, MAX_CURS * MAX_CURS * 2);

	if (set & FB_CUR_SETPOS) {
		u32 xx, yy, temp;

		yy = cursor->image.dy - info->var.yoffset;
		xx = cursor->image.dx - info->var.xoffset;
		temp = xx & 0xFFFF;
		temp |= yy << 16;

		write_reg(&(pdiu->curs_pos), temp);
	}

	if (set & (FB_CUR_SETSHAPE | FB_CUR_SETCMAP | FB_CUR_SETIMAGE)) {
		u32 bg_idx = cursor->image.bg_color;
		u32 fg_idx = cursor->image.fg_color;
		u32 s_pitch = (cursor->image.width+7) >> 3;
		u32 d_pitch = MAX_CURS/8;
		u8 *dat = (u8 *) cursor->image.data;
		u8 *msk = (u8 *) cursor->mask;
		u8 *src;

		src = kmalloc(s_pitch * cursor->image.height, GFP_ATOMIC);

		switch (cursor->rop) {
			case ROP_XOR:
				for (i = 0; i < s_pitch * cursor->image.height; i++)
					src[i] = dat[i] ^ msk[i];
				break;
			case ROP_COPY:
			default:
				for (i = 0; i < s_pitch * cursor->image.height; i++)
					src[i] = dat[i] & msk[i];
				break;
		}

		fb_pad_aligned_buffer(data, d_pitch, src, s_pitch,
						cursor->image.height);
		bg = ((info->cmap.red[bg_idx] & 0xe0) << 10) |
			((info->cmap.green[bg_idx] & 0xe0) << 5) |
			((info->cmap.blue[bg_idx] & 0xe0) >> 5) |
			1 << 15;

		fg = ((info->cmap.red[fg_idx] & 0xe0) << 10) |
			((info->cmap.green[fg_idx] & 0xe0) << 5) |
			((info->cmap.blue[fg_idx] & 0xe0) >> 5) |
			1 << 15;

		load_cursor_image(info, data, bg, fg,
					 cursor->image.width,
					 cursor->image.height);
		kfree(src);
	}

	if (cursor->enable)
		show_cursor(info);

	return 0;
}
/*
 * Using the fb_var_screeninfo in fb_info we set the resolution of this
 * particular framebuffer. This function alters the fb_fix_screeninfo stored
 * in fb_info. It does not alter var in fb_info since we are using that
 * data. This means we depend on the data in var inside fb_info to be
 * supported by the hardware. fsl_diu_check_var is always called before
 * fsl_diu_set_par to ensure this.
 */
static int fsl_diu_set_par(struct fb_info *info)
{
	unsigned long len;
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu_ad *ad = mfbi->ad;
	struct diu *hw;

	DPRINTK("Entered: fsl_diu_set_par\n");
	hw = dr.diu_reg;

	set_fix(info);
	mfbi->cursor_reset = 1;

	len = info->var.yres_virtual * info->fix.line_length;
	/* Alloc & dealloc each time resolution/bpp change */
	if (len != info->fix.smem_len ) {
		if (info->fix.smem_start)
			unmap_video_memory(info);
		DPRINTK("SET PAR: smem_len = %d\n", info->fix.smem_len);

		/* Memory allocation for framebuffer */
		if (map_video_memory(info)) {
			printk("Unable to allocate fb memory 1\n");
			return -ENOMEM;
		}
	}

	ad->pix_fmt = platform_get_pixel_format(var->bits_per_pixel,monitor_port);
	ad->addr    = cpu_to_le32(info->fix.smem_start);
	ad->src_size_g_alpha
			= cpu_to_le32((var->yres << 12) | var->xres) | mfbi->g_alpha;
	/* fix me. AOI should not be greater than display size */
	ad->aoi_size 	= cpu_to_le32(( var->yres << 16) |  var->xres);
	ad->offset_xyi = 0;
	ad->offset_xyd = cpu_to_le32(( mfbi->y_aoi_d << 16) |  mfbi->x_aoi_d);

	/* Disable chroma keying function */
	ad->ckmax_r = 0;
	ad->ckmax_g = 0;
	ad->ckmax_b = 0;

	ad->ckmin_r = 255;
	ad->ckmin_g = 255;
	ad->ckmin_b = 255;

	if (mfbi->index == 0)
		update_lcdc(info);
	return 0;
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
			   unsigned blue, unsigned transp, struct fb_info *info)
{
	int ret = 1;

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

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
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

static int fsl_diu_enable_panel(struct fb_info *info)
{
	struct mfb_info *pmfbi, *cmfbi, *mfbi = (struct mfb_info *)info->par;
	struct diu *hw = dr.diu_reg;
	struct diu_ad *ad = mfbi->ad;
	int res = 0;

	DPRINTK("Entered: enable_panel index %d\n",mfbi->index);
	if(mfbi->type != MFB_TYPE_OFF) {
		switch (mfbi->index) {
		case 0:				/* plane 0 */
			if (hw->desc[0] != ad->paddr)
				write_reg(&(hw->desc[0]), ad->paddr);
			break;
		case 1:				/* plane 1 AOI 0 */
			cmfbi = (struct mfb_info *) fsl_diu_info[2].par;
			if (hw->desc[1] != ad->paddr) {	/* AOI0 was closed */
				if (cmfbi->count > 0)	/* AOI1 is open */
					ad->next_ad = cpu_to_le32(cmfbi->ad->paddr);
				else
					ad->next_ad = 0;
				write_reg(&(hw->desc[1]), ad->paddr);
			}
			break;
		case 3:				/* plane 2 AOI 0 */
			cmfbi = (struct mfb_info *) fsl_diu_info[4].par;
			if (hw->desc[2] != ad->paddr) {	/* AOI0 was closed */
				if (cmfbi->count > 0)	/* AOI1 is open */
					ad->next_ad = cpu_to_le32(cmfbi->ad->paddr);
				else
					ad->next_ad = 0;
				write_reg(&(hw->desc[2]), ad->paddr);
			}
			break;
		case 2:				/* plane 1 AOI 1 */
			pmfbi = (struct mfb_info *)fsl_diu_info[1].par;
			ad->next_ad = 0;
			if (hw->desc[1] == dummy_ad->paddr)	/* AOI0 was closed */
				write_reg(&(hw->desc[1]), ad->paddr);
			else					/* AOI0 was open */
				pmfbi->ad->next_ad = cpu_to_le32(ad->paddr);
			break;
		case 4:				/* plane 2 AOI 1 */
			pmfbi = (struct mfb_info *)fsl_diu_info[3].par;
			ad->next_ad = 0;
			if (hw->desc[2] == dummy_ad->paddr)	/* AOI0 was closed */
				write_reg(&(hw->desc[2]), ad->paddr);
			else					/* AOI0 was open */
				pmfbi->ad->next_ad = cpu_to_le32(ad->paddr);
			break;
		default:
			res = -EINVAL;
			break;
		}
	}
	else
		res = -EINVAL;
	return res;
}

static int fsl_diu_disable_panel(struct fb_info *info)
{
	struct mfb_info *pmfbi, *cmfbi, *mfbi = (struct mfb_info *)info->par;
	struct diu *hw = dr.diu_reg;
	struct diu_ad *ad = mfbi->ad;
	int res = 0;

	DPRINTK("Entered: disable_panel\n");
	switch (mfbi->index) {
	case 0:					/* plane 0 */
		if (hw->desc[0] != dummy_ad->paddr)
			write_reg(&(hw->desc[0]), dummy_ad->paddr);
		break;
	case 1:					/* plane 1 AOI 0 */
		cmfbi = (struct mfb_info *) fsl_diu_info[2].par;
		if (cmfbi->count > 0)	/* AOI1 is open */
			write_reg(&(hw->desc[1]),cmfbi->ad->paddr);	/* move AOI1 to the first */
		else			/* AOI1 was closed */
			write_reg(&(hw->desc[1]),dummy_ad->paddr);	/* close AOI 0 */
		break;
	case 3:					/* plane 2 AOI 0 */
		cmfbi = (struct mfb_info *) fsl_diu_info[4].par;
		if (cmfbi->count > 0)	/* AOI1 is open */
			write_reg(&(hw->desc[2]),cmfbi->ad->paddr);	/* move AOI1 to the first */
		else			/* AOI1 was closed */
			write_reg(&(hw->desc[2]),dummy_ad->paddr);	/* close AOI 0 */
		break;
	case 2:					/* plane 1 AOI 1 */
		pmfbi = (struct mfb_info *)fsl_diu_info[1].par;
		if (hw->desc[1] != ad->paddr) {	/* AOI1 is not the first in the chain */
			if (pmfbi->count > 0)	/* AOI0 is open, must be the first */
				pmfbi->ad->next_ad = 0;
		}
		else				/* AOI1 is the first in the chain */
			write_reg(&(hw->desc[1]),dummy_ad->paddr);	/* close AOI 1 */
		break;
	case 4:					/* plane 2 AOI 1 */
		pmfbi = (struct mfb_info *)fsl_diu_info[3].par;
		if (hw->desc[2] != ad->paddr) {	/* AOI1 is not the first in the chain */
			if (pmfbi->count > 0)	/* AOI0 is open, must be the first */
				pmfbi->ad->next_ad = 0;
		}
		else				/* AOI1 is the first in the chain */
			write_reg(&(hw->desc[2]),dummy_ad->paddr);	/* close AOI 1 */
		break;
	default:
		res = -EINVAL;
		break;
	}

	return res;
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
	/* FIXME: fixes to enable_panel and enable lcdc needed */
	case FB_BLANK_NORMAL:
	/*	fsl_diu_disable_panel(info);*/
		break;
	case FB_BLANK_POWERDOWN:
	/*	disable_lcdc(info);	*/
		break;
	case FB_BLANK_UNBLANK:
	/*	fsl_diu_enable_panel(info);*/
		break;
	}

	return 0;
}

static int fsl_diu_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu_ad *ad = mfbi->ad;
	struct mfb_chroma_key ck;
	unsigned char global_alpha;
	struct aoi_display_offset aoi_d;

	DPRINTK("Entered: fsl_diu_ioctl\n");
	switch (cmd) {
	case MFB_SET_AOID:
		if (!arg)
			return -EINVAL;
		if (copy_from_user((void *)&aoi_d, (void *)arg, sizeof(aoi_d)))
			return -EFAULT;
		mfbi->x_aoi_d = aoi_d.x_aoi_d;
		mfbi->y_aoi_d = aoi_d.y_aoi_d;
		DPRINTK("set AOI display offset of index %d to (%d,%d)\n",mfbi->index,aoi_d.x_aoi_d,aoi_d.y_aoi_d);
		fsl_diu_check_var(&info->var,info);
		fsl_diu_set_par(info);
		break;
	case MFB_GET_AOID:
		if (!arg)
			return -EINVAL;
		aoi_d.x_aoi_d = mfbi->x_aoi_d;
		aoi_d.y_aoi_d = mfbi->y_aoi_d;
		if (copy_to_user((void *)arg, (void *)&aoi_d, sizeof(aoi_d)))
			return -EFAULT;
		DPRINTK("get AOI display offset of index %d (%d,%d)\n",mfbi->index,aoi_d.x_aoi_d,aoi_d.y_aoi_d);
		break;
	case MFB_GET_ALPHA:
		if (!arg)
			return -EINVAL;
		global_alpha = mfbi->g_alpha;
		if (copy_to_user((void *)arg, (void *)&global_alpha, sizeof(global_alpha)))
			return -EFAULT;
		DPRINTK("get global alpha of index %d\n",mfbi->index);
		break;
	case MFB_SET_ALPHA:
		if (!arg)
			return -EINVAL;

		/* set panel information */
		if (copy_from_user((void *)&global_alpha, (void *)arg, sizeof(global_alpha)))
			return -EFAULT;
		ad->src_size_g_alpha = (ad->src_size_g_alpha & (~0xff)) | (global_alpha & 0xff);
		mfbi->g_alpha = global_alpha;
		DPRINTK("set global alpha for index %d\n",mfbi->index);
		break;
	case MFB_SET_CHROMA_KEY:
		if (!arg)
			return -EINVAL;

		/* set panel winformation */
		if (copy_from_user((void *)&ck, (void *)arg, sizeof(ck)))
			return -EFAULT;

		if(ck.enable &&
		   (ck.red_max < ck.red_min ||
		    ck.green_max < ck.green_min ||
		    ck.blue_max < ck.blue_min))
			return -EINVAL;

		if(!ck.enable) {
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
		DPRINTK("set chroma key\n");
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
	case FBIOGET_HWCINFO:
		printk("%s, FBIOGET_HWCINFO:0x%08x\n", __func__, FBIOGET_HWCINFO);
		break;
	case FBIOPUT_MODEINFO:
		printk("%s, FBIOPUT_MODEINFO:0x%08x\n", __func__, FBIOPUT_MODEINFO);
		break;
	case FBIOGET_DISPINFO:
		printk("%s, FBIOGET_DISPINFO:0x%08x\n", __func__, FBIOGET_DISPINFO);
		break;

	default:
		printk("Unknown ioctl command (0x%08X)\n", cmd);
		return 0;
	}

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

static int init_fbinfo(struct fb_info *info,
			       struct platform_device *pdev)
{
	struct mfb_info *mfbi = (struct mfb_info *)info->par;

	DPRINTK("Entered: init_fbinfo\n");
	info->device = NULL;
	info->var.activate = FB_ACTIVATE_NOW;
	info->fbops = &fsl_diu_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = &mfbi->pseudo_palette;

	/* Allocate colormap */
	fb_alloc_cmap(&info->cmap, 16, 0);
	return 0;
}

static int install_fb(struct fb_info *info,
			      struct platform_device *pdev)
{
	int rc;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	const char * aoi_mode, * init_aoi_mode ="320x240";

	DPRINTK("Entered: install_fb\n");
	if (init_fbinfo(info, pdev))
		return -EINVAL;

	if (mfbi->index == 0)	/* plane 0 */
		aoi_mode = fb_mode;
	else
		aoi_mode = init_aoi_mode;
	DPRINTK("mode used = %s\n",aoi_mode);
	rc = fb_find_mode(&info->var, info, aoi_mode, fsl_diu_mode_db, ARRAY_SIZE(fsl_diu_mode_db), &fsl_diu_default_mode, default_bpp);

	switch (rc) {
	case 1:
		DPRINTK("using mode specified in @mode\n");
		break;
	case 2:
		DPRINTK("using mode specified in @mode with ignored refresh rate\n");
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
		break;
	}

	DPRINTK("xres_virtual %d\n", info->var.xres_virtual);
	DPRINTK("bits_per_pixel %d\n", info->var.bits_per_pixel);

	DPRINTK("info->var.yres_virtual = %d\n", info->var.yres_virtual);
	DPRINTK("info->fix.line_length = %d\n", info->fix.line_length);

	if (mfbi->type == MFB_TYPE_OFF)
		mfbi->blank = FB_BLANK_NORMAL;
	else
		mfbi->blank = FB_BLANK_UNBLANK;

	if (fsl_diu_check_var(&info->var,info)) {
		printk("fb_check_var failed");
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	if (fsl_diu_set_par(info)) {
		printk("fb_set_par failed");
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	if (register_framebuffer(info) < 0) {
		printk("register_framebuffer failed");
		unmap_video_memory(info);
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	mfbi->registered = 1;
	printk("fb%d: %s fb device registered successfully.\n",
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
	DPRINTK("info->var.xres_virtual = %d\n", info->var.xres_virtual);
	DPRINTK("info->var.yres_virtual = %d\n", info->var.yres_virtual);
	DPRINTK("info->fix.line_length  = %d\n", info->fix.line_length);

	info->fix.smem_len = info->fix.line_length * info->var.yres_virtual;
	DPRINTK("MAP_VIDEO_MEMORY: smem_len = %d\n", info->fix.smem_len);
	info->screen_base = fsl_diu_alloc(info->fix.smem_len,&info->fix.smem_start);
	if (info->screen_base == 0) {
		printk("Unable to allocate fb memory\n");
		return -EBUSY;
	}

	info->screen_size = info->fix.smem_len;

	DPRINTK("Allocated fb @ paddr=0x%08lx, size=%d.\n", info->fix.smem_start,
		info->fix.smem_len);
	DPRINTK("screen base 0x%08lx\n",(long unsigned int) info->screen_base);

	return 0;
}

static void unmap_video_memory(struct fb_info *info)
{
	DPRINTK("Entered: unmap_video_memory\n");
	fsl_diu_free(info->screen_base,info->fix.smem_len);
	info->screen_base = 0;
	info->fix.smem_start = 0;
	info->fix.smem_len = 0;
}

static void enable_lcdc(struct fb_info *info)
{
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: enable_lcdc\n");
	if (!fb_enabled) {
		write_reg(&(hw->diu_mode), dr.mode);
		fb_enabled++;
	}
}

static void disable_lcdc(struct fb_info *info)
{
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: disable_lcdc\n");
	if (fb_enabled) {
		write_reg(&(hw->diu_mode), 0);
		fb_enabled = 0;
	}
}

static void update_lcdc(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = (struct mfb_info *)info->par;
	struct diu *hw;
	int i, j;
	char __iomem * cursor_base, *gamma_table_base;

	u32 temp;

	DPRINTK("Entered: update_lcdc\n");

	spin_lock_init(&dr.reg_lock);
	hw = dr.diu_reg;

	if (mfbi->type == MFB_TYPE_OFF) {
		fsl_diu_disable_panel(info);
		return;
	}

	platform_set_monitor_port(monitor_port);
	
	gamma_table_base = pool.gamma.vaddr;
	cursor_base = pool.cursor.vaddr;
	/* Prep for DIU init  - gamma table, cursor table */

	for (i=0;i<=2;i++)
	   for(j=0;j<=255;j++)
	      *gamma_table_base++ = j;

	platform_set_gamma_table(monitor_port, pool.gamma.vaddr);

	DPRINTK("update-lcdc: HW - %p\n Disabling DIU\n", hw);
	disable_lcdc(info);

	/* Program DIU registers */

	write_reg(&(hw->gamma), pool.gamma.paddr);
	write_reg(&(hw->cursor), pool.cursor.paddr);

	write_reg(&(hw->bgnd), 0x007F7F7F); 	/* BGND */
	write_reg(&(hw->bgnd_wb), 0); 		/* BGND_WB */
	write_reg(&(hw->disp_size), (var->yres << 16 | var->xres));
						/* DISP SIZE */
	DPRINTK("DIU xres: %d\n", var->xres);
	DPRINTK("DIU yres: %d\n", var->yres);

	write_reg(&(hw->wb_size), 0); /* WB SIZE */
	write_reg(&(hw->wb_mem_addr), 0); /* WB MEM ADDR */

	/* Horizontal and vertical configuration register */
	temp = var->left_margin << 22 | /* BP_H */
	       var->hsync_len << 11 |   /* PW_H */
	       var->right_margin;       /* FP_H */

	write_reg(&(hw->hsyn_para), temp);

	temp = var->upper_margin << 22 | /* BP_V */
	       var->vsync_len << 11 |    /* PW_V  */
	       var->lower_margin;        /* FP_V  */

	write_reg(&(hw->vsyn_para), temp);

	DPRINTK("DIU right_margin - %d\n",var->right_margin);
	DPRINTK("DIU left_margin - %d\n",var->left_margin);
	DPRINTK("DIU hsync_len - %d\n",var->hsync_len);
	DPRINTK("DIU upper_margin - %d\n",var->upper_margin);
	DPRINTK("DIU lower_margin - %d\n",var->lower_margin);
	DPRINTK("DIU vsync_len - %d\n",var->vsync_len);
	DPRINTK("DIU HSYNC - 0x%08x\n",hw->hsyn_para);
	DPRINTK("DIU VSYNC - 0x%08x\n",hw->vsyn_para);

	platform_set_pixel_clock(var->pixclock);

	write_reg(&(hw->syn_pol), 0); /* SYNC SIGNALS POLARITY */
	write_reg(&(hw->thresholds), 0x00037800); /* The Thresholds */
	write_reg(&(hw->int_status), 0); /* INTERRUPT STATUS */
        write_reg(&(hw->int_mask), (0x1F&(~(INT_VSYNC | INT_UNDRUN)))); /* INT MASK */
	write_reg(&(hw->plut),0x01F5F666);

	/* Enable the DIU */
	enable_lcdc(info);
}

static irqreturn_t fsl_diu_isr(int irq, void *dev_id)
{
	struct diu *hw = dr.diu_reg;
	unsigned int status = read_reg(&(hw->int_status));

	if (status) {
                /* This is the workaround for underrun */
                if (status & INT_UNDRUN) {
                        out_be32(&(hw->diu_mode), 0);
                        DPRINTK("Err: DIU occurs underrun!\n");
                        udelay(1);
                        out_be32(&(hw->diu_mode), 1);
                }
#if defined(CONFIG_NOT_COHERENT_CACHE)
		else if (status & INT_VSYNC) {
			int i;
			unsigned int *ptr;
			ptr  = coherence_data;
			for (i=0;i<1024*8;i++)
				*ptr++ = 0;
		}
#endif
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
	status = read_reg(&(hw->int_status));

	np = of_find_node_by_type(NULL, "display");
	if(!np) {
		pr_info("Err: didn't find node 'lcd-controller' in device tree!\n");
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
			unsigned long ints = INT_PARERR;

#if !defined(CONFIG_NOT_COHERENT_CACHE)
			ints |= INT_VSYNC;
#endif

			if(dr.mode == MFB_MODE2 || dr.mode == MFB_MODE3)
				ints |= INT_VSYNC_WB;

			/* Read to clear the status */
			status = read_reg(&(hw->int_status));

			/* Enable EOF, parameter error, and optional
			 * write back interrupt
			 */
			write_reg(&(hw->int_mask), ints);
		}

		spin_unlock_irqrestore(&fsl_diu_notifier_list.lock, flags);
	}
}

static void free_irq_local(void)
{
	struct diu *hw = dr.diu_reg;

	DPRINTK("Entered: free_irq_local\n");
	/* Disable all LCDC interrupt */
	write_reg(&(hw->int_mask), 0x1f);

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
		unsigned long ints = INT_PARERR;

#if !defined(CONFIG_NOT_COHERENT_CACHE)
			ints |= INT_VSYNC;
#endif

		if(dr.mode == MFB_MODE2 || dr.mode == MFB_MODE3)
			ints |= INT_VSYNC_WB;

		/* Read to clear the status */
		status = read_reg(&(hw->int_status));

		/* Enable EOF, parameter error, and optional
		 * write back interrupt
		 */
		write_reg(&(hw->int_mask), ints);
	}

	spin_unlock_irqrestore(&fsl_diu_notifier_list.lock, flags);

	return ret;
}

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
		write_reg(&(hw->int_mask), 0x1f);

	spin_unlock_irqrestore(&fsl_diu_notifier_list.lock, flags);

	return ret;
}

#ifdef CONFIG_PM
/*
 * Power management hooks. Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int fsl_diu_suspend(struct platform_device *pdev, pm_message_t state)
{
	DPRINTK("Entered: fsl_diu_suspend\n");
	disable_lcdc(&fsl_diu_info[0]);

	return 0;
}

static int fsl_diu_resume(struct platform_device *pdev)
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
	buf->vaddr = dma_alloc_coherent(0, ssize,
					(dma_addr_t *) &(buf->paddr),
					GFP_DMA | GFP_KERNEL);
	if(!buf->vaddr)
		return -ENOMEM;

	memset(buf->vaddr, 0, ssize);

	mask = bytes_align - 1;
	offset = (u32)buf->paddr & mask;
	if(offset) {
		buf->offset = bytes_align - offset;
		buf->paddr = (u32)buf->paddr + offset;
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

static ssize_t store_monitor(struct device *device, struct device_attribute *attr,
			const char *buf, size_t count)
{
	char ** last = NULL;
	int val,old_monitor_port;

	val = simple_strtoul(buf, last, 0);

	old_monitor_port = monitor_port;

	monitor_port = platform_set_sysfs_monitor_port(val);

	if (old_monitor_port != monitor_port) {	/* All AOIs need adjust pixel format */
		fsl_diu_set_par (&fsl_diu_info[0]);
		fsl_diu_set_par (&fsl_diu_info[1]);
		fsl_diu_set_par (&fsl_diu_info[2]);
		fsl_diu_set_par (&fsl_diu_info[3]);
		fsl_diu_set_par (&fsl_diu_info[4]);
	}
	return count;
}

static ssize_t show_monitor(struct device *device, struct device_attribute *attr,
			char *buf)
{
	return platform_show_monitor_port(monitor_port, buf);
}

static struct device_attribute device_attrs[] = {
	__ATTR(monitor, S_IRUGO|S_IWUSR, show_monitor, store_monitor),
};

static int fsl_diu_probe(struct platform_device *pdev)
{
	struct mfb_info *mfbi;
	unsigned int dummy_ad_addr;
	int ret, i, error = 0;

	DPRINTK("Entered: fsl_diu_probe\n");

	/* Area descriptor memory pool aligns to 64-bit boundary */
	allocate_buf(&pool.ad, sizeof(struct diu_ad) * FSL_AOI_NUM, 8);

	/* Get memory for Gamma Table  - 32-byte aligned memory */
	allocate_buf(&pool.gamma, 768, 32);

	/* For high performance, cursor bitmap buffer aligns to 32-byte boundary */
	allocate_buf(&pool.cursor, MAX_CURS * MAX_CURS * 2, 32);

	i = sizeof(fsl_diu_info) / sizeof(struct fb_info);
	dummy_ad = (struct diu_ad *)((u32)pool.ad.vaddr + pool.ad.offset) + i;
	dummy_ad->paddr = pool.ad.paddr + i * sizeof(struct diu_ad);
	dummy_aoi_virt = fsl_diu_alloc(64, &dummy_ad_addr);
	dummy_ad->addr = cpu_to_le32(dummy_ad_addr);
	dummy_ad->pix_fmt = 0x88882317;
	dummy_ad->src_size_g_alpha = cpu_to_le32((4 << 12) | 4);	/* alpha = 0 */
	dummy_ad->aoi_size = cpu_to_le32(( 4 << 16) |  2);
	dummy_ad->offset_xyi = 0;
	dummy_ad->offset_xyd = 0;
	dummy_ad->next_ad = 0;
	memset(dummy_aoi_virt, 0x00, 64);
	write_reg(&(dr.diu_reg->desc[0]), dummy_ad->paddr);
	write_reg(&(dr.diu_reg->desc[1]), dummy_ad->paddr);
	write_reg(&(dr.diu_reg->desc[2]), dummy_ad->paddr);

	for (i = 0; i < sizeof(fsl_diu_info) / sizeof(struct fb_info); i++) {
		fsl_diu_info[i].fix.smem_start = 0;
		mfbi = (struct mfb_info *)fsl_diu_info[i].par;
		mfbi->ad = (struct diu_ad *)((u32)pool.ad.vaddr
					+ pool.ad.offset) + i;
		mfbi->ad->paddr = pool.ad.paddr + i * sizeof(struct diu_ad);
		if ((ret = install_fb(&fsl_diu_info[i], pdev))) {
			printk("Failed to register framebuffer %d\n", i);
			return ret;
		}
	}
	for (i = 0; i < ARRAY_SIZE(device_attrs); i++) {
		error = device_create_file(fsl_diu_info[0].dev, &device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(fsl_diu_info[0].dev, &device_attrs[i]);
	}

#if defined(CONFIG_NOT_COHERENT_CACHE)
	coherence_data = fsl_diu_alloc(32*1024, &coherence_data_phy);
#endif

	request_irq_local();
	return 0;
}

static struct platform_device fsl_diu_device = {
	.name   = "fsl_diu",
};

int __init fsl_diu_init(void)
{
	struct device_node *np;
	struct resource r;
	int error;
	DPRINTK("Entered: fsl_diu_init\n");
	np = of_find_compatible_node(NULL, NULL, "fsl-diu");
	if(!np) {
		printk("Err: can't find device node 'fsl-diu'\n");
		return -ENODEV;
	}

	of_address_to_resource(np, 0, &r);
	of_node_put(np);

	DPRINTK("%s, r.start: 0x%08x\n", __func__, r.start);

	dr.diu_reg = (struct diu *)ioremap(r.start, sizeof(struct diu));

	write_reg(&(dr.diu_reg->diu_mode), 0);				/* disable DIU anyway */

	spin_lock_init(&dr.reg_lock);


	/*
	 * For kernel boot options (in 'video=xxxfb:<options>' format)
	 */
#ifndef MODULE
	{
		char *option;

		if (fb_get_options("fslfb", &option))
			return -ENODEV;
		fsl_diu_setup(option);
	}
#endif
	error = platform_driver_register(&fsl_diu_driver);

	if (!error) {
		error = platform_device_register(&fsl_diu_device);
		if (error) {
			printk("Err: can't register FB device driver!\n");
			platform_driver_unregister(&fsl_diu_driver);
		}
		printk("FSL_DIU_FB: registed FB device driver!\n");
	}

	return error;
}

void __exit fsl_diu_exit(void)
{
	int i;

	DPRINTK("Entered: fsl_diu_exit\n");
	free_irq_local();
	for (i = sizeof(fsl_diu_info) / sizeof(struct fb_info); i > 0; i--)
		uninstall_fb(&fsl_diu_info[i - 1]);

	platform_driver_unregister(&fsl_diu_driver);
	iounmap(dr.diu_reg);

	free_buf(&pool.ad, sizeof(struct diu_ad) * FSL_AOI_NUM, 8);
	free_buf(&pool.cursor, MAX_CURS * MAX_CURS * 2, 32);
	fsl_diu_free(dummy_aoi_virt,64);
	return;
}

#ifndef MODULE
static int __init fsl_diu_setup(char *options)
{
	char *opt;
	int val;

	DPRINTK("Entered: fsl_diu_setup\n");
	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "monitor=", 8)) {
			val = simple_strtoul (opt + 8, NULL, 0);
			if ((val == 0 ) || (val == 1) || (val == 2))
				monitor_port = val;
		}
		else if (!strncmp(opt, "bpp=", 4))
			default_bpp = simple_strtoul(opt + 4, NULL, 0);
		else
			fb_mode = opt;
	}

	return 0;
}
#endif

module_init(fsl_diu_init);
module_exit(fsl_diu_exit);

EXPORT_SYMBOL(fsl_diu_register_client);
EXPORT_SYMBOL(fsl_diu_unregister_client);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("FSL DIU framebuffer driver");
MODULE_LICENSE("GPL");
