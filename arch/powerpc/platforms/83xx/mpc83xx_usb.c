/*
 * Copyright (C) 2007,2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Duck, <duck@freescale.com>, Tue Oct 2 2007
 *
 * Description:
 * MPC83xx USB platform-specific routines
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

int usb_platform_dr_init(struct platform_device *pdev)
{
	int rc;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("%s:  pdata %p\n\n", __FUNCTION__, pdata);

	pdata->have_sysif_regs = 1;
	pr_debug("%s: success\n", __FUNCTION__);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_platform_dr_init);


void usb_platform_dr_uninit(struct fsl_usb2_platform_data *pdata)
{
	pr_debug("%s\n", __FUNCTION__);

}
EXPORT_SYMBOL_GPL(usb_platform_dr_uninit);

int usb_platform_mph_init(struct platform_device *pdev)
{
	int rc;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("%s\n", __FUNCTION__);

	pdata->have_sysif_regs = 1;

	pr_debug("%s: success\n", __FUNCTION__);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_platform_mph_init);


void usb_platform_mph_uninit(struct fsl_usb2_platform_data *pdata)
{
	pr_debug("%s\n", __FUNCTION__);
}
EXPORT_SYMBOL_GPL(usb_platform_mph_uninit);
