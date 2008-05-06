/*
 * Copyright (C) 2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Description:
 * This file contains lowlevel PM test code
 *
 * This file is part of the Linux kernel
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/time.h>
#include <asm/reg.h>
#include <sysdev/fsl_soc.h>

#define DEBUG

u8 *xmscan = NULL;
u32 *xgpio = NULL;
u32 once = 0;
u32 irq;

extern int mpc512x_set_gpio_wakeup(unsigned int gpio_num,
				   unsigned int detect_mode);
extern void mpc512x_pmc_clrevent(void);

static irqreturn_t mpc51xx_gpio_handler(int irq, void *dev_id)
{
	mpc512x_pmc_clrevent();

	if (xgpio) {
		out_be32((u32 *) ((u32) xgpio + 0x0C), 0xFF);
#ifdef DEBUG
		printk(" :)\n");
#endif
	}
	return IRQ_HANDLED;
}
static irqreturn_t mpc51xx_can0_handler(int irq, void *dev_id)
{
	u8 *mscan = xmscan;
	if (mscan)
		out_8(mscan + 8, in_8(mscan + 8));

	mpc512x_pmc_clrevent();
#ifdef DEBUG
	printk("c0 \n");
#endif
	return IRQ_HANDLED;
}

static irqreturn_t mpc51xx_can1_handler(int irq, void *dev_id)
{
	u8 *mscan = xmscan + 0x80;
	if (mscan)
		out_8(mscan + 8, in_8(mscan + 8));

	mpc512x_pmc_clrevent();
#ifdef DEBUG
	printk("c1 \n");
#endif
	return IRQ_HANDLED;
}

void mpc512x_can_setup(u8 * addr)
{
	u32 reg = 0;

	u8 *mscan = addr;

	/* Enable the CAN Module */
	reg = in_8(mscan + 0x01);
	reg |= 0x80;		//Assert CANE
	reg &= ~0x10;		//Deassert LISTEN
	out_8(mscan + 1, reg);

	reg = in_8(mscan);
	reg |= 0x2;		//Sleep Req
	out_8(mscan, reg);
	mdelay(20);
	reg = in_8(mscan);
	reg |= 0x1;		//Init req
	out_8(mscan, reg);

#define MSCAN_CANIDMR0_OFFSET           0x28	/* Identifier Mask Registers */
#define MSCAN_CANIDMR1_OFFSET           0x29	/* Identifier Mask Registers */
#define MSCAN_CANIDMR2_OFFSET           0x2C	/* Identifier Mask Registers */
#define MSCAN_CANIDMR3_OFFSET           0x2D	/* Identifier Mask Registers */
#define MSCAN_CANIDMR4_OFFSET           0x38	/* Identifier Mask Registers */
#define MSCAN_CANIDMR5_OFFSET           0x39	/* Identifier Mask Registers */
#define MSCAN_CANIDMR6_OFFSET           0x3C	/* Identifier Mask Registers */
#define MSCAN_CANIDMR7_OFFSET           0x3D	/* Identifier Mask Registers */
	mdelay(20);
	out_8(mscan + MSCAN_CANIDMR0_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR1_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR2_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR3_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR4_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR5_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR6_OFFSET, 0xFF);
	out_8(mscan + MSCAN_CANIDMR7_OFFSET, 0xFF);

	/* Come out of Init */
	reg = in_8(mscan);
	reg &= ~0x1;
	reg |= 0x4;		//Set WUPE
	out_8(mscan, reg);

	mdelay(20);
	/* Enabling the Interrupts for MSCAN 0 */
	reg = in_8(mscan + 9);
	reg |= 0xFF;
	out_8(mscan + 9, reg);
}

void mpc512x_pm_test_setup(void)
{
	struct device_node *ofn;
	u32 reg;
	u32 *clock = NULL;
	const u32 *cell_index;

	if (once == 0) {
		/* Enable the BDLC/MSCAN Periperal Clock */
		clock = ioremap((u32) get_immrbase() + 0xF00, 0x100);
		reg = in_be32((u32 *) ((u32) clock + 0x08));
		reg |= 0x02000000;
		out_be32((u32 *) ((u32) clock + 0x08), reg);
		iounmap(clock);

		/*Setup the irq handlers with dummy event ids */
		ofn = of_find_compatible_node(NULL, NULL, "fsl,mpc5121-gpio");
		irq = irq_of_parse_and_map(ofn, 0);
		of_node_put(ofn);
		reg =
		    request_irq(irq, mpc51xx_gpio_handler, IRQF_PERCPU,
				"mpx512x_test_gpio", (void *)0);
#ifdef DEBUG
		printk("gpio irq - %d ret = %d\n", irq, reg);
#endif

		for_each_compatible_node(ofn, NULL, "fsl,mpc5121-mscan") {
			cell_index = of_get_property(ofn, "cell-index", NULL);
			if (cell_index && *cell_index == 0) {
				irq = irq_of_parse_and_map(ofn, 0);
				reg = request_irq(irq, mpc51xx_can0_handler, IRQF_PERCPU,
					"mpx512x_test_can0", (void *)1);
#ifdef DEBUG
				printk("can0 irq - %d ret = %d\n", irq, reg);
#endif
			}
			if (cell_index && *cell_index == 1) {
				irq = irq_of_parse_and_map(ofn, 0);
				reg = request_irq(irq, mpc51xx_can1_handler, IRQF_PERCPU,
					"mpx512x_test_can1", (void *)2);
#ifdef DEBUG
				printk("can1 irq - %d ret = %d\n", irq, reg);
#endif
			}
		}

		xgpio = ioremap((u32) get_immrbase() + 0x1100, 0x100);
		xmscan = ioremap((u32) get_immrbase() + 0x1300, 0x100);

		/* Setup the CAN modules for testing wakeup */
		mpc512x_can_setup(xmscan);
		mpc512x_can_setup(xmscan + 0x80);

		out_be32((u32 *) ((u32) xgpio + 0xC), 0xFFFFFFFF);
		mpc512x_set_gpio_wakeup(28, 3);
		mpc512x_set_gpio_wakeup(29, 3);
		mpc512x_set_gpio_wakeup(30, 3);
		mpc512x_set_gpio_wakeup(31, 3);
		once = 1;
	}
}
