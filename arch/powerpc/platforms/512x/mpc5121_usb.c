/*
 * Copyright (C) 2007,2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Duck, <duck@freescale.com>, Tue Oct 2 2007
 *
 * Description:
 * MPC5121 USB platform-specific routines
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fsl_devices.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/stddef.h>

#define USBGENCTRL		0x200	/* NOTE: big endian */
#define GC_WU_INT_CLR		(1 << 5)	/* Wakeup int clear */
#define GC_ULPI_SEL		(1 << 4)	/* ULPI i/f select (usb0 only)*/
#define GC_PPP			(1 << 3)	/* Port Power Polarity */
#define GC_PFP			(1 << 2)	/* Power Fault Polarity */
#define GC_WU_ULPI_EN		(1 << 1)	/* Wakeup on ULPI event */
#define GC_WU_IE		(1 << 1)	/* Wakeup interrupt enable */

#define ISIPHYCTRL		0x204	/* NOTE: big endian */
#define PHYCTRL_PHYE		(1 << 4)	/* On-chip UTMI PHY enable */
#define PHYCTRL_BSENH		(1 << 3)	/* Bit Stuff Enable High */
#define PHYCTRL_BSEN		(1 << 2)	/* Bit Stuff Enable */
#define PHYCTRL_LSFE		(1 << 1)	/* Line State Filter Enable */
#define PHYCTRL_PXE		(1 << 0)	/* PHY oscillator enable */


int usb_platform_mph_init(struct platform_device *pdev)
{
	return -ENODEV;		/* no MPH port on 5121 */
}
EXPORT_SYMBOL_GPL(usb_platform_mph_init);


void usb_platform_mph_uninit(struct fsl_usb2_platform_data *pdata)
{
}
EXPORT_SYMBOL_GPL(usb_platform_mph_uninit);

static struct clk *dr_clk;
static int dr_used;

int usb_platform_dr_init(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("%s:  pdata %p\n\n", __FUNCTION__, pdata);

	/* enable the clock if we haven't already */
	if (!dr_used) {
		dr_clk = clk_get(&pdev->dev, "usb0_clk");
		if (IS_ERR(dr_clk)) {
			dev_err(&pdev->dev, "usb: clk_get failed\n");
			return -ENODEV;
		}
		clk_enable(dr_clk);
	}
	dr_used++;

	pdata->big_endian_desc = 1;
	pdata->big_endian_mmio = 0;
	pdata->le_setup_buf = 1;
	pdata->es = 1;

	if (pdata->phy_mode == FSL_USB2_PHY_UTMI_WIDE) {
		void __iomem *base = pdata->regs;

		out_be32(base + ISIPHYCTRL, PHYCTRL_PHYE | PHYCTRL_PXE);
		out_be32(base + USBGENCTRL, GC_PPP | GC_PFP);
	}
	pr_debug("%s: success\n", __FUNCTION__);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_platform_dr_init);

void usb_platform_dr_uninit(struct fsl_usb2_platform_data *pdata)
{
	pr_debug("%s\n", __FUNCTION__);

	pdata->regs = NULL;
	pdata->r_start = pdata->r_len = 0;

	dr_used--;
	if (!dr_used) {
		clk_disable(dr_clk);
		clk_put(dr_clk);
		dr_clk = NULL;
	}
}
EXPORT_SYMBOL_GPL(usb_platform_dr_uninit);

