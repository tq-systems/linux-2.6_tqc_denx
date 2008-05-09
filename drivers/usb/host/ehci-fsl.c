/*
 * (C) Copyright David Brownell 2000-2002
 * Copyright (c) 2005 MontaVista Software
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Ported to 834x by Randy Vinson <rvinson@mvista.com> using code provided
 * by Hunter Wu.
 */

#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include "ehci-fsl.h"

extern struct resource *otg_get_resources(void);

#define EHCI_PROC_PTC

#ifdef EHCI_PROC_PTC		/* /proc PORTSC:PTC support */
#include <asm/uaccess.h>
#define EFPSL 3			/* ehci fsl proc string length */

static int ehci_fsl_proc_read(char *page, char **start, off_t off, int count,
			      int *eof, void *data)
{
	return 0;
}

static int ehci_fsl_proc_write(struct file *file, const char __user *buffer,
			       unsigned long count, void *data)
{
	int ptc;
	u32 portsc;
	struct ehci_hcd *ehci = (struct ehci_hcd *) data;
	char str[EFPSL] = {0};

	if (count > EFPSL-1)
		return -EINVAL;

	if (copy_from_user(str, buffer, count))
		return -EFAULT;

	str[count] = '\0';

	ptc = simple_strtoul(str, NULL, 0);

	portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);
	portsc &= ~(0xf << 16);
	portsc |= (ptc << 16);
	printk(KERN_INFO "PTC %x  portsc %08x\n", ptc, portsc);

	ehci_writel(ehci, portsc, &ehci->regs->port_status[0]);

	return count;
}

static int ehci_testmode_init(struct ehci_hcd *ehci)
{
	struct proc_dir_entry *entry;

	entry = create_proc_read_entry("driver/ehci-ptc", 0644, NULL,
				       ehci_fsl_proc_read, ehci);
	if (!entry)
		return -ENODEV;

	entry->write_proc = ehci_fsl_proc_write;
	return 0;
}
#else
static int ehci_testmode_init(struct ehci_hcd *ehci)
{
	return 0;
}
#endif	/* /proc PORTSC:PTC support */


/* PCI-based HCs are common, but plenty of non-PCI HCs are used too */

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_hcd_fsl_probe - initialize FSL-based HCDs
 * @drvier: Driver to be used for this HCD
 * @pdev: USB Host Controller being probed
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
int usb_hcd_fsl_probe(const struct hc_driver *driver,
		      struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	struct usb_hcd *hcd;
	struct resource *res;
	int irq;
	int retval;
	unsigned int temp;

	pr_debug("initializing FSL-SOC USB Controller\n");

	/* Need platform data for setup */
	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", pdev->dev.bus_id);
		return -ENODEV;
	}

	/*
	 * This is a host mode driver, verify that we're supposed to be
	 * in host mode.
	 */
	if (!((pdata->operating_mode == FSL_USB2_DR_HOST) ||
	      (pdata->operating_mode == FSL_USB2_MPH_HOST) ||
	      (pdata->operating_mode == FSL_USB2_DR_OTG))) {
		dev_err(&pdev->dev,
			"Non Host Mode configured for %s. Wrong driver linked.\n",
			pdev->dev.bus_id);
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, pdev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

#if defined(CONFIG_USB_OTG)
	if (pdata->operating_mode == FSL_USB2_DR_OTG) {
		res = otg_get_resources();
		if (!res) {
			dev_err(&pdev->dev,
				"Found HC with no IRQ. Check %s setup!\n",
				pdev->dev.bus_id);
			return -ENODEV;
		}
		irq = res[1].start;
		hcd->rsrc_start = res[0].start;
		hcd->rsrc_len = res[0].end - res[0].start + 1;
	} else
#endif
	{
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!res) {
			dev_err(&pdev->dev,
				"Found HC with no IRQ. Check %s setup!\n",
				pdev->dev.bus_id);
			return -ENODEV;
		}
		irq = res->start;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		hcd->rsrc_start = res->start;
		hcd->rsrc_len = res->end - res->start + 1;

		if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
					driver->description)) {
			dev_dbg(&pdev->dev, "controller already in use\n");
			retval = -EBUSY;
			goto err2;
		}
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto err3;
	}
	pdata->regs = hcd->regs;

	/*
	 * do platform specific init: check the clock, grab/config pins, etc.
	 */
	if (pdata->platform_init && pdata->platform_init(pdev)) {
		retval = -ENODEV;
		goto err3;
	}

	if (pdata->have_sysif_regs) {
		/* Enable USB controller */
		temp = in_be32(hcd->regs + FSL_SOC_USB_CTRL);
		out_be32(hcd->regs + FSL_SOC_USB_CTRL, temp | 0x4);
	}

	/* Set to Host mode */
	temp = in_le32(hcd->regs + FSL_SOC_USB_USBMODE);
	temp |= USBMODE_CM_HOST | (pdata->es ? USBMODE_ES : 0);
	out_le32(hcd->regs + FSL_SOC_USB_USBMODE, temp);

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		goto err4;

#ifdef CONFIG_USB_OTG
	if (pdata->operating_mode == FSL_USB2_DR_OTG) {
		struct ehci_hcd *ehci = hcd_to_ehci(hcd);

		dbg("pdev=0x%p  hcd=0x%p  ehci=0x%p\n", pdev, hcd, ehci);

		ehci->transceiver = otg_get_transceiver();
		dbg("ehci->transceiver=0x%p\n", ehci->transceiver);

		if (ehci->transceiver) {
			retval = otg_set_host(ehci->transceiver,
					      &ehci_to_hcd(ehci)->self);
			if (retval) {
				if (ehci->transceiver)
					put_device(ehci->transceiver->dev);
				goto err4;
			}
		} else {
			printk(KERN_ERR "can't find transceiver\n");
			retval = -ENODEV;
			goto err4;
		}
	}
#endif

	ehci_testmode_init(hcd_to_ehci(hcd));
	return retval;

      err4:
	iounmap(hcd->regs);
      err3:
	if (pdata->operating_mode != FSL_USB2_DR_OTG)
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
      err2:
	usb_put_hcd(hcd);
      err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", pdev->dev.bus_id, retval);
	if (pdata->platform_uninit)
		pdata->platform_uninit(pdata);
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_fsl_remove - shutdown processing for FSL-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_fsl_probe().
 *
 */
void usb_hcd_fsl_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	if (ehci->transceiver) {
		(void)otg_set_host(ehci->transceiver, 0);
		put_device(ehci->transceiver->dev);
	} else {
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	}

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	/*
	 * do platform specific un-initialization:
	 * release iomux pins, etc.
	 */
	if (pdata->platform_uninit)
		pdata->platform_uninit(pdata);

	iounmap(hcd->regs);
}

static void mpc83xx_setup_phy(struct ehci_hcd *ehci,
			      enum fsl_usb2_phy_modes phy_mode,
			      unsigned int port_offset)
{
	u32 portsc = 0;
	switch (phy_mode) {
	case FSL_USB2_PHY_ULPI:
		portsc |= PORT_PTS_ULPI;
		break;
	case FSL_USB2_PHY_SERIAL:
		portsc |= PORT_PTS_SERIAL;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		portsc |= PORT_PTS_PTW;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		portsc |= PORT_PTS_UTMI;
		break;
	case FSL_USB2_PHY_NONE:
		break;
	}
	ehci_writel(ehci, portsc, &ehci->regs->port_status[port_offset]);
}

static void mpc83xx_usb_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fsl_usb2_platform_data *pdata;
	void __iomem *non_ehci = hcd->regs;
	u32 tmp;

	pdata = hcd->self.controller->platform_data;

	/* Enable PHY interface in the control reg. */
	if (pdata->have_sysif_regs) {
		out_be32(non_ehci + FSL_SOC_USB_CTRL, 0x00000004);
		out_be32(non_ehci + FSL_SOC_USB_SNOOP1, 0x0000001b);
	}

#if defined(CONFIG_PPC32) && !defined(CONFIG_NOT_COHERENT_CACHE)
	/*
	 * Turn on cache snooping hardware, since some PowerPC platforms
	 * wholly rely on hardware to deal with cache coherent
	 */

	/* Setup Snooping for all the 4GB space */
	/* SNOOP1 starts from 0x0, size 2G */
	out_be32(non_ehci + FSL_SOC_USB_SNOOP1, 0x0 | SNOOP_SIZE_2GB);
	/* SNOOP2 starts from 0x80000000, size 2G */
	out_be32(non_ehci + FSL_SOC_USB_SNOOP2, 0x80000000 | SNOOP_SIZE_2GB);
#endif

	if ((pdata->operating_mode == FSL_USB2_DR_HOST) ||
			(pdata->operating_mode == FSL_USB2_DR_OTG))
		mpc83xx_setup_phy(ehci, pdata->phy_mode, 0);

	if (pdata->operating_mode == FSL_USB2_MPH_HOST) {
		unsigned int chip, rev, svr;

		svr = mfspr(SPRN_SVR);
		chip = svr >> 16;
		rev = (svr >> 4) & 0xf;

		/* Deal with USB Erratum #14 on MPC834x Rev 1.0 & 1.1 chips */
		if ((rev == 1) && (chip >= 0x8050) && (chip <= 0x8055))
			ehci->has_fsl_port_bug = 1;

		if (pdata->port_enables & FSL_USB2_PORT0_ENABLED)
			mpc83xx_setup_phy(ehci, pdata->phy_mode, 0);
		if (pdata->port_enables & FSL_USB2_PORT1_ENABLED)
			mpc83xx_setup_phy(ehci, pdata->phy_mode, 1);
	}

	/* put controller in host mode. */
	tmp = USBMODE_CM_HOST | (pdata->es ? USBMODE_ES : 0);
	ehci_writel(ehci, tmp, non_ehci + FSL_SOC_USB_USBMODE);

	if (pdata->have_sysif_regs) {
		out_be32(non_ehci + FSL_SOC_USB_PRICTRL, 0x0000000c);
		out_be32(non_ehci + FSL_SOC_USB_AGECNTTHRSH, 0x00000040);
		out_be32(non_ehci + FSL_SOC_USB_SICTRL, 0x00000001);
	}
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_fsl_reinit(struct ehci_hcd *ehci)
{
	mpc83xx_usb_setup(ehci_to_hcd(ehci));
	ehci_port_power(ehci, 0);
	return 0;
}

/* called during probe() after chip reset completes */
static int ehci_fsl_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;
	struct fsl_usb2_platform_data *pdata;
	pdata = hcd->self.controller->platform_data;

	ehci->big_endian_desc = pdata->big_endian_desc;
	ehci->big_endian_mmio = pdata->big_endian_mmio;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
	    HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci->is_tdi_rh_tt = 1;

	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	retval = ehci_fsl_reinit(ehci);
	return retval;
}

static const struct hc_driver ehci_fsl_hc_driver = {
	.description = hcd_name,
	.product_desc = "Freescale On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_fsl_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.start_port_reset = ehci_start_port_reset,
};

static int ehci_fsl_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	return usb_hcd_fsl_probe(&ehci_fsl_hc_driver, pdev);
}

static int ehci_fsl_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_fsl_remove(hcd, pdev);
	return 0;
}


#ifdef CONFIG_PM
/*
 * Holding pen for all the EHCI registers except port_status,
 * which is a zero element array and hence takes no space.
 * The port_status register is saved in usb_ehci_portsc.
 */
volatile static struct ehci_regs usb_ehci_regs;
static u32 usb_ehci_portsc;

/* suspend/resume, section 4.3 */

/* These routines rely on the bus (pci, platform, etc)
 * to handle powerdown and wakeup, and currently also on
 * transceivers that don't need any software attention to set up
 * the right sort of wakeup.
 *
 * They're also used for turning on/off the port when doing OTG.
 */
static int ehci_fsl_drv_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 tmp;

	/* DDD printk("%s()\n", __func__); */
	printk(KERN_INFO "USB Host suspended\n");

	hcd->state = HC_STATE_SUSPENDED;
	pdev->dev.power.power_state = PMSG_SUSPEND;

	if (hcd->driver->suspend)
		return hcd->driver->suspend(hcd, message);

	/* ignore non-host interrupts */
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	/* stop the controller */
	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp &= ~CMD_RUN;
	ehci_writel(ehci, tmp, &ehci->regs->command);

	/* save EHCI registers */
	usb_ehci_regs.command = ehci_readl(ehci, &ehci->regs->command);
	usb_ehci_regs.command &= ~CMD_RUN;
	usb_ehci_regs.status  = ehci_readl(ehci, &ehci->regs->status);
	usb_ehci_regs.intr_enable  = ehci_readl(ehci, &ehci->regs->intr_enable);
	usb_ehci_regs.frame_index  = ehci_readl(ehci, &ehci->regs->frame_index);
	usb_ehci_regs.segment  = ehci_readl(ehci, &ehci->regs->segment);
	usb_ehci_regs.frame_list  = ehci_readl(ehci, &ehci->regs->frame_list);
	usb_ehci_regs.async_next  = ehci_readl(ehci, &ehci->regs->async_next);
	usb_ehci_regs.configured_flag  = ehci_readl(ehci, &ehci->regs->configured_flag);
	usb_ehci_portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);

	/* clear the W1C bits */
	usb_ehci_portsc &= cpu_to_hc32(ehci, ~PORT_RWC_BITS);

	/* clear PP to cut power to the port */
	tmp = ehci_readl(ehci, &ehci->regs->port_status[0]);
	tmp &= ~PORT_POWER;
	ehci_writel(ehci, tmp, &ehci->regs->port_status[0]);

	return 0;
}

static int ehci_fsl_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 tmp;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	printk(KERN_INFO "USB Host resumed\n");

	/* set host mode */
	tmp = USBMODE_CM_HOST | (pdata->es ? USBMODE_ES : 0);
	ehci_writel(ehci, tmp, hcd->regs + FSL_SOC_USB_USBMODE);

	/* restore EHCI registers */
	ehci_writel(ehci, usb_ehci_regs.command, &ehci->regs->command);
	ehci_writel(ehci, usb_ehci_regs.status, &ehci->regs->status);
	ehci_writel(ehci, usb_ehci_regs.intr_enable, &ehci->regs->intr_enable);
	ehci_writel(ehci, usb_ehci_regs.frame_index, &ehci->regs->frame_index);
	ehci_writel(ehci, usb_ehci_regs.segment, &ehci->regs->segment);
	ehci_writel(ehci, usb_ehci_regs.frame_list, &ehci->regs->frame_list);
	ehci_writel(ehci, usb_ehci_regs.async_next, &ehci->regs->async_next);
	ehci_writel(ehci, usb_ehci_regs.configured_flag, &ehci->regs->configured_flag);
	ehci_writel(ehci, usb_ehci_portsc, &ehci->regs->port_status[0]);

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	hcd->state = HC_STATE_RUNNING;
	pdev->dev.power.power_state = PMSG_ON;

	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp |= CMD_RUN;
	ehci_writel(ehci, tmp, &ehci->regs->command);

	usb_hcd_resume_root_hub(hcd);

	return 0;
}
#endif				/* CONFIG_USB_OTG */

MODULE_ALIAS("fsl-ehci");

static struct platform_driver ehci_fsl_driver = {
	.probe = ehci_fsl_drv_probe,
	.remove = ehci_fsl_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend = ehci_fsl_drv_suspend,
	.resume = ehci_fsl_drv_resume,
#endif
	.driver = {
		   .name = "fsl-ehci",
		   },
};
