/*
 * Copyright (C) 2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Description:
 * This file contains power management code for MPC5121eADS
 *
 * This file is part of the Linux kernel
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/of_platform.h>
#include <asm/time.h>
#include <asm/mpc512x.h>
#include <asm/ipic.h>
#include <asm/reg.h>
#include <sysdev/fsl_soc.h>

#include "mpc512x_pm.h"

struct mpc512x_pm mpc512x_pm_data;

static u32 mpc512x_targeted_state = MPC512x_PM_NONE;

/*
 * Name       : mpc512x_pm_setup
 * Desc       : This function is called to setup and map the IO region.
 *
 * Parameters : void
 * Return     : void
 */
static int mpc512x_pm_setup(void)
{
	memset(&mpc512x_pm_data, 0, sizeof(mpc512x_pm_data));

	mpc512x_pm_data.mbar = ioremap(get_immrbase(), MPC512x_IMMRBAR_MEM_MAPPED);
       	if (!mpc512x_pm_data.mbar) {
               	printk(KERN_ERR "Error mapping MBAR registers\n");
                return -1;
       	}
	return 0;
}

/*
 * Name       : mpc512x_pm_release
 * Desc       : This function is called to unmap/release the allocated resources.
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_pm_release(void)
{
	unsigned long flags;

	if(!mpc512x_pm_data.mbar) return;

	local_irq_save(flags);
	iounmap(mpc512x_pm_data.mbar);
	memset(&mpc512x_pm_data, 0, sizeof(mpc512x_pm_data));
	local_irq_restore(flags);
}

/*
 * Name       : mpc512x_sleep
 * Desc       : This function is called to enter the Sleep mode by setting the
 *		[SLEEP] bit in HID0 and [POW] bit in MSR.
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_sleep(void)
{
	u32 hid0, msr;

	/* Enable SLEEP mode and disable the rest */
	hid0 = mfspr(SPRN_HID0);
	mtspr(SPRN_HID0, (hid0 & ~(HID0_DOZE | HID0_NAP
			 | HID0_DPM)) | HID0_SLEEP);
	asm volatile("isync" : : : "memory");
	asm volatile("sync" : : : "memory");

	msr = mfmsr();
	mtmsr(msr | MSR_EE);
	asm volatile("isync" : : : "memory");
	asm volatile("sync" : : : "memory");

	/* Enter Sleep mode*/
	msr = mfmsr();
	mtmsr(msr | MSR_POW);
	asm volatile("isync" : : : "memory");
	asm volatile("sync" : : : "memory");

	msr = mfmsr();
	mtmsr(msr & ~MSR_EE);
	asm volatile("isync" : : : "memory");
	asm volatile("sync" : : : "memory");

	/* Disable sleep modes */
	hid0 = mfspr(SPRN_HID0);
	mtspr(SPRN_HID0, (hid0 & ~(HID0_DOZE | HID0_NAP | HID0_SLEEP)));
	asm volatile("isync" : : : "memory");
	asm volatile("sync" : : : "memory");
}

/*
 * Name       : mpc512x_pmc_clrevent
 * Desc       : This function needs to be called by the interrupt handlers of
 * 		the wakeup sources. This is needed since a PMC interrupt is
 *		not guaranteed on MPC5121 v1.0, while an interrupt from the
 * 		wakeup source (GPIO / CAN) is.
 *
 * Parameters : void
 * Return     : void
 */
void mpc512x_pmc_clrevent(void)
{
	struct mpc512x_pmc *pmc;

	if(mpc512x_pm_data.mbar){
		pmc = (struct mpc512x_pmc *)((u32)mpc512x_pm_data.mbar +
					MPC512x_IMMRBAR_PMC_OFFSET);
		if(in_be32(&pmc->pmc_er)& 0x1){
			out_be32(&pmc->pmc_er, 0x1);
		}
	}
}

EXPORT_SYMBOL_GPL(mpc512x_pmc_clrevent);

/*
 * Name       : mpc512x_set_gpio_wakeup
 * Desc       : This function would initialise the gpio with the given detection mode
 * 		and enable the interrupt.
 *
 * Parameters :
 * Return     : int
 */
int mpc512x_set_gpio_wakeup(unsigned int gpio_num, unsigned int detect_mode)
{
	volatile u32 reg;
	u32 __iomem *gpio;

	gpio = ioremap((u32)get_immrbase() + MPC512x_IMMRBAR_GPIO_OFFSET,
			 MPC512x_GPIO_MEM_MAP);
	if(!gpio){
		printk("GPIO memory could not be mapped. \n");
		return -1;
	}
	gpio_num = 31 - gpio_num;
	reg = in_be32(&gpio[MPC512x_GPIO_IMR >> 2]);
	reg |= 1 << gpio_num;
	out_be32(&gpio[MPC512x_GPIO_IMR >> 2], reg);

	reg = in_be32(&gpio[MPC512x_GPIO_ICR2 >> 2]);
	reg &= ~(0x3 << (gpio_num * 2));
	reg |= ((detect_mode & 3)<< (gpio_num * 2));
	out_be32(&gpio[MPC512x_GPIO_ICR2 >> 2], reg);

	iounmap(gpio);
	return 0;
}

EXPORT_SYMBOL_GPL(mpc512x_set_gpio_wakeup);

/*
 * Name       : mpc512x_pm_valid
 * Desc       : Checks whether the PM state is valid
 *
 * Parameters : void
 * Return     : 1 - Valid , 0 - Invalid
 */
static int mpc512x_pm_valid(suspend_state_t state)
{
	switch(state){
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;
		default:
			return 0;
	}
}

/*
 * Name       : mpc512x_pm_settarget
 * Desc       : Set the state to which the system is to enter.
 *
 * Parameters : void
 * Return     : 0 - Success
 */
static int mpc512x_pm_settarget(suspend_state_t state)
{
	switch(state){
		case PM_SUSPEND_STANDBY:
			mpc512x_targeted_state = MPC512x_PM_STANDBY;
			break;
		case PM_SUSPEND_MEM:
			mpc512x_targeted_state = MPC512x_PM_SUSP_MEM;
			break;
		default:
			mpc512x_targeted_state = MPC512x_PM_NONE;
	}
	return 0;
}

/*
 * Name       : mpc512x_set_ipic_regs
 * Desc       : Save the IPIC Mask registers and enable the wakeup interrupts
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_set_ipic_regs(void)
{
	u32 *ipic = (u32 *)((u32)mpc512x_pm_data.mbar +
					MPC512x_IMMRBAR_IPIC_OFFSET);

	/* Save the current IPIC mask register values. */
	mpc512x_pm_data.ipic_simsr_h = in_be32(&ipic[IPIC_SIMSR_H >> 2]);
	mpc512x_pm_data.ipic_simsr_l = in_be32(&ipic[IPIC_SIMSR_L >> 2]);

	/* Disable all the interrupts except the wakeup sources */
	out_be32(&ipic[IPIC_SIMSR_H >> 2], MPC512x_IPIC_MSRH_MSCAN1
					 | MPC512x_IPIC_MSRH_MSCAN2);
	out_be32(&ipic[IPIC_SIMSR_L >> 2], MPC512x_IPIC_MSRL_RTCSEC |
					 MPC512x_IPIC_MSRL_GPIO);
}

/*
 * Name       : mpc512x_restore_ipic_regs
 * Desc       : Restore the IPIC Mask registers to original values.
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_restore_ipic_regs(void)
{
	u32 *ipic = (u32 *)((u32)mpc512x_pm_data.mbar +
					MPC512x_IMMRBAR_IPIC_OFFSET);

	/* Restore the IPIC masks to saved values */
	out_be32(&ipic[IPIC_SIMSR_L >> 2], mpc512x_pm_data.ipic_simsr_l);
	out_be32(&ipic[IPIC_SIMSR_H >> 2], mpc512x_pm_data.ipic_simsr_h);
}
/*
 * Name       : mpc512x_set_rtc_wakeup
 * Desc       : This Function would set up the Wake-Up source configurations
 * in the RTC registers. The RTC interrupts would be generated for GPIO[28-31]
 * and CAN 1 & 2 receive Interrupts.
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_set_rtc_wakeup(void)
{
	u32 rtc_reg;
	u32 *rtc;

	if(!mpc512x_pm_data.mbar) return;

	rtc = (u32 *)((u32)mpc512x_pm_data.mbar + MPC512x_IMMRBAR_RTC_OFFSET);

	rtc_reg = in_be32(&rtc[MPC512x_RTC_KEEPALIVE >> 2]);
	mpc512x_pm_data.rtc_keepalive = rtc_reg;

	/* Set the Active LVL values for the Wake-up Sources[1-5] */
	rtc_reg |= MPC512x_RTCKAR_WKUP_SRCLVL;
	out_be32(&rtc[MPC512x_RTC_KEEPALIVE >> 2], rtc_reg);

	/* Enable the Wake-Up sources and disable Hibernate mode. */
	rtc_reg |= (MPC512x_RTCKAR_WKUP_SRCEN | MPC512x_RTCKAR_DIS_HIBMODE);
	out_be32(&rtc[MPC512x_RTC_KEEPALIVE >> 2], rtc_reg);

	/* Set the Target Time Register to a Future Value */
	mpc512x_pm_data.rtc_targettime = in_be32(&rtc[MPC512x_RTC_TTR >> 2]);
	out_be32(&rtc[MPC512x_RTC_TTR >> 2], MPC512x_RTCTTR_MAXTIMEOUT);

}

static void mpc512x_restore_rtc_regs(void)
{
	u32 *rtc;

	if(!mpc512x_pm_data.mbar) return;

	rtc = (u32 *)((u32)mpc512x_pm_data.mbar + MPC512x_IMMRBAR_RTC_OFFSET);

	/* Restore the RTC Registers */
	out_be32(&rtc[MPC512x_RTC_KEEPALIVE >> 2],
				 mpc512x_pm_data.rtc_keepalive);
	out_be32(&rtc[MPC512x_RTC_TTR >> 2], mpc512x_pm_data.rtc_targettime);
}

/*
 * Name       : mpc512x_set_ddr_selfrefresh
 * Desc       : Set the DDRC SELFREFRESH registers, to enter and exit
 *		DDR Self Refresh mode on entering Deep Sleep mode.
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_set_ddr_selfrefresh(void)
{
	struct ddr512x *ddrc;

	if(!mpc512x_pm_data.mbar) return;

	ddrc =(struct ddr512x *)((u32)mpc512x_pm_data.mbar +
					MPC512x_IMMRBAR_DDRC_OFFSET);

	mpc512x_pm_data.ddrc_sysconfig = in_be32(&ddrc->ddr_sys_config);

	/* Write the register contents with SELF-REFRESH EN bit set.*/
	out_be32(&ddrc->ddr_sys_config,	mpc512x_pm_data.ddrc_sysconfig
					 | MPC512x_DDRC_SELFREFEN);

	/* Set the Self Refresh Entry Commands */
	out_be16(&ddrc->self_refresh_cmd_0, MPC512x_DDRC_SELF_REF_CMD0);
	out_be16(&ddrc->self_refresh_cmd_1, MPC512x_DDRC_SELF_REF_CMD1);
	out_be16(&ddrc->self_refresh_cmd_2, MPC512x_DDRC_SELF_REF_CMD2);
	out_be16(&ddrc->self_refresh_cmd_3, MPC512x_DDRC_SELF_REF_CMD3);

	/* Set the Self Refresh Exit Commands */
	out_be16(&ddrc->self_refresh_cmd_4, MPC512x_DDRC_SELF_REF_CMD4);
	out_be16(&ddrc->self_refresh_cmd_5, MPC512x_DDRC_SELF_REF_CMD5);
	out_be16(&ddrc->self_refresh_cmd_6, MPC512x_DDRC_SELF_REF_CMD6);
	out_be16(&ddrc->self_refresh_cmd_7, MPC512x_DDRC_SELF_REF_CMD7);
}

static void mpc512x_restore_ddr_regs(void)
{
	struct ddr512x *ddrc;

	if(!mpc512x_pm_data.mbar) return;

	ddrc =(struct ddr512x *)((u32)mpc512x_pm_data.mbar +
					MPC512x_IMMRBAR_DDRC_OFFSET);
	out_be32(&ddrc->ddr_sys_config, mpc512x_pm_data.ddrc_sysconfig);
}

/*
 * Name       : mpc512x_pm_prepare
 * Desc       : This function would map the IO regions. Also sets the DDRC and
 * 		RTC regs for Deep Sleep Mode.
 *
 * Parameters : void
 * Return     : int
 *		ENOSYS
 *
 */
static int mpc512x_pm_prepare(void)
{
	mpc512x_pm_setup();

	switch(mpc512x_targeted_state) {
		case MPC512x_PM_STANDBY:
			break;
		case MPC512x_PM_SUSP_MEM:
			mpc512x_set_ddr_selfrefresh();

			/*
			 *  Enable the wakeup sources and set RTC Target Time
			 *  to future
			 */
			mpc512x_set_rtc_wakeup();
			break;
   	}
	return 0;
}

/*
 * Name       : mpc512x_enter_deepsleep
 * Desc       : This function puts the MPC5121e system to Deep-Sleep State. The
 *		Core is first put to sleep. After this the H/W sequencers take
 *		the system to Deep-Sleep. Before entering the Deep-
 *		Sleep state the Wake-Up sources are set for GPIO[28-31] and CAN
 *		1 & 2 receiver interrupts.
 *
 * Parameters : void
 * Return     : int
 *
 */
static int mpc512x_enter_deepsleep(void)
{
        struct mpc512x_pmc *pmc;

	/* Enable the GPIO and CAN Interrupts */
	mpc512x_set_ipic_regs();

	/* Don't let DEC expire any time soon */
	mtspr(SPRN_DEC, MPC512x_DEC_MAXTIMEOUT);

	pmc = (struct mpc512x_pmc *)((u32)mpc512x_pm_data.mbar +
					MPC512x_IMMRBAR_PMC_OFFSET);
	/* Set the DSM, DDROFF & COREOFF bits in PMC CR register.*/
	out_be32(&pmc->pmc_cr, MPC512x_PMCCR_DSMEN | MPC512x_PMCCR_DDROFF
				 | MPC512x_PMCCR_COREOFF);
	out_be32(&pmc->pmc_mr, MPC512x_PMCMR_PMCIE);

	/* Put core to SLEEP so that MPC512x enters Deep-Sleep.*/
	mpc512x_sleep();

	/* We are out of Deep Sleep.. Lets restart jiffies */
	wakeup_decrementer();

	/* Reset the PMC CR register. */
	out_be32(&pmc->pmc_cr, 0x0);
	out_be32(&pmc->pmc_mr, 0x0);

	/* Restore the IPIC regs to their original values */
	mpc512x_restore_ipic_regs();
	return 0;
}

/*
 * Name       : mpc512x_pm_enter
 * Desc       : This function is exported to the Power Management Core. This
 *	`	function is called with the state which the system should enter.
 *
 * Parameters : state 	- PM_SUSPEND_STANDBY
			- PM_SUSPEND_MEM
 * Return     : int
 * 		-1 : FAILED
 *		0  : SUCCESS
 */
static int mpc512x_pm_enter(suspend_state_t state)
{
	if(!mpc512x_pm_data.mbar)
	{
		printk(KERN_ERR "Failed to enter PM mode as IO not mapped.\n");
		return -1;
	}

	switch(mpc512x_targeted_state){

		case MPC512x_PM_STANDBY:
			mpc512x_sleep();
			break;
		case MPC512x_PM_SUSP_MEM:
			mpc512x_enter_deepsleep();
		default:
			break;
	}
	return 0;
}

/*
 * Name       : mpc512x_pm_finish
 * Desc       : This routine is called by the kernel on exit from
 * 		power down modes. Restores the DDRC and RTC regs
 * 		suspend to memory. Also releases allocated resources.
 *
 * Parameters : void
 * Return     : void
 */
static void mpc512x_pm_finish(void)
{
	switch(mpc512x_targeted_state){

		case MPC512x_PM_STANDBY:
			break;
		case MPC512x_PM_SUSP_MEM:

			/* Restore the DDR and RTC registers on wake-up.*/
			mpc512x_restore_ddr_regs();
			mpc512x_restore_rtc_regs();
			break;
	}
	mpc512x_targeted_state = MPC512x_PM_NONE;

	mpc512x_pm_release();
}

static struct platform_suspend_ops mpc512x_pm_ops = {
	.valid		= mpc512x_pm_valid,
	.set_target	= mpc512x_pm_settarget,
	.prepare	= mpc512x_pm_prepare,
	.enter		= mpc512x_pm_enter,
	.finish		= mpc512x_pm_finish,
};

/*
 * Name       : mpc512x_pm_init
 * Desc       : This function registers the platform_suspend_ops
 * 		structure with the kernel.
 *
 * Parameters : void
 * Return     : int
 */
int __init mpc512x_pm_init(void)
{
	suspend_set_ops(&mpc512x_pm_ops);
	return 0;
}

