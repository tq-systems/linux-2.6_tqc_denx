/*
 * Copyright (C) 2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Description:
 * This file power management code for MPC5121eADS
 *
 * This file is part of the Linux kernel
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MPC512x_PM_H__
#define __MPC512x_PM_H__

/* Peripheral address offsets from IMMRBAR */
#define MPC512x_IMMRBAR_RTC_OFFSET		0xA00
#define MPC512x_IMMRBAR_IPIC_OFFSET		0xC00
#define MPC512x_IMMRBAR_PMC_OFFSET		0x1000
#define MPC512x_IMMRBAR_GPIO_OFFSET		0x1100
#define MPC512x_IMMRBAR_DDRC_OFFSET		0x9000

/* Memory mapped by Power management module */
#define MPC512x_IMMRBAR_MEM_MAPPED		0x10000
#define MPC512x_GPIO_MEM_MAP			0x100

/* Register offsets for RTC */
#define MPC512x_RTC_TTR				0x20
#define MPC512x_RTC_ATR				0x24
#define MPC512x_RTC_KEEPALIVE			0x28

/* Register offsets for GPIO */
#define MPC512x_GPIO_IMR			0x10
#define MPC512x_GPIO_ICR2			0x18

/* Bit positions in IPIC memory region */
#define MPC512x_IPIC_MSRH_MSCAN1		(1 << 4)
#define MPC512x_IPIC_MSRH_MSCAN2		(1 << 3)

#define MPC512x_IPIC_MSRL_GPIO			(1 << 17)
#define MPC512x_IPIC_MSRL_RTCSEC		(1 << 16)

/* Bit positions in PMC memory region */
#define MPC512x_PMCCR_DSMEN			(1 << 2)
#define MPC512x_PMCCR_DDROFF			(1 << 1)
#define MPC512x_PMCCR_COREOFF			(1 << 0)
#define MPC512x_PMCMR_PMCIE			(1 << 0)

/* Bit positions in DDRC memory region */
#define MPC512x_DDRC_SELFREFEN			(1 << 18)

/* DDRC commands to set the DRAM in and out of Self Refresh */
/* These commands have worked on the MPC5121ADS board */
#define MPC512x_DDRC_SELF_REF_CMD0		0x3C00
#define MPC512x_DDRC_SELF_REF_CMD1		0x4420
#define MPC512x_DDRC_SELF_REF_CMD2		0x4210
#define MPC512x_DDRC_SELF_REF_CMD3		0x1410

#define MPC512x_DDRC_SELF_REF_CMD4		0x1C00
#define MPC512x_DDRC_SELF_REF_CMD5		0x3C08
#define MPC512x_DDRC_SELF_REF_CMD6		0x4200
#define MPC512x_DDRC_SELF_REF_CMD7		0x3800

/* RTC Keep alive register values*/
#define MPC512x_RTCKAR_WKUP_SRCLVL		0x001F0000
#define MPC512x_RTCKAR_WKUP_SRCEN		0x1F000000
#define MPC512x_RTCKAR_DIS_HIBMODE		0x00000080

/* RTC Target Time register Timeout*/
#define MPC512x_RTCTTR_MAXTIMEOUT		0xFFFFFFFF

/* Decrementer timeout */
#define MPC512x_DEC_MAXTIMEOUT			0x7FFFFFFF

/* Power Management states */
#define MPC512x_PM_NONE				0
#define MPC512x_PM_STANDBY			1
#define MPC512x_PM_SUSP_MEM			2

/* PMC registers*/
struct mpc512x_pmc {
	u32 pmc_cr;		/* Configuration register - 0x00 */
	u32 pmc_er;		/* Event register	  - 0x04 */
	u32 pmc_mr;		/* Mask register	  - 0x08 */
	u32 pmc_sr;		/* Shadow register	  - 0x0C */
};

/* Data structure used by the Power management module */
struct mpc512x_pm{

	/* Pointer to IMMRBAR (ioremaped) */
	void __iomem *mbar;

	/* Registers saved/restored by PM module */
	u32 ipic_simsr_l;
	u32 ipic_simsr_h;
	u32 rtc_keepalive;
	u32 rtc_targettime;
	u32 ddrc_sysconfig;
};

#endif /* __MPC512x_PM_H__ */
