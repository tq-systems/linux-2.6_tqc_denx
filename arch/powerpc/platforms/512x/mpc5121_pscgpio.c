/*
 * Copyright (C) 2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: John Rigby, <jrigby@freescale.com>, April 2008
 *
 * Description:
 * MPC5121 psc gpio helper routines
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/of_platform.h>

#include <asm/io.h>

/* 
 * Helper routines for pscN_M pins.
 * Allow for making the psc pins into GPIO outputs and
 * driving them high or low.
 * These are for use by the various psc drivers (Audio, SPI, whatever)
 * so they don't have to be polluted with IOCTL and GPIO knowledge.
 *
 * TODO:  Make a full fledged GPIO driver. 
 */

/*
 * psc/pin to gpio number map
 *   psc pin gpio
 * ==========================
 *   0    0   8  (psc % 5)*4 + 8 + pin
 *   0    1   9
 *   0    2   10
 *   0    3   11
 *   0    4   0  psc % 8
 *   1    0   12
 *   1    1   13
 *   1    2   14
 *   1    3   15
 *   1    4   1
 *   2    0   16
 *   ....
 *   pin 4 cycles through GPIOs 0-7
 *   other pins cycle through GPIOs 8-27
 *   ....
 *   10   2   10
 *   10   3   11
 *   10   4   2
 *   11   0   12
 *   11   1   13
 *   11   2   14
 *   11   3   15
 *   11   4   3
 */

#define PSC_TO_GPIO(psc, pin) ( \
    pin == 4 ?			\
	(psc % 8)		\
    :				\
	(psc % 5) * 4 + 8 + pin	\
) 

#define PSC_IOCTL_SIZE (5 * sizeof(long))

#define PSC_TO_IOCTL_OFFSET(psc, pin) ( \
	0x20C				\
       	+ psc * PSC_IOCTL_SIZE		\
	+ pin * sizeof(long)		\
)

static void __iomem *gpioctl;
#define GPIODIR 0
#define GPIODAT 8
static void  __iomem *ioctl;

void mpc5121_pscgpio_make_gpio(int psc, int pin)
{
	out_be32(ioctl + PSC_TO_IOCTL_OFFSET(psc, pin), 0x00000183);
	setbits32(gpioctl+GPIODIR, 0x80000000 >> PSC_TO_GPIO(psc, pin));
}
EXPORT_SYMBOL(mpc5121_pscgpio_make_gpio);

void mpc5121_pscgpio_make_psc(int psc, int pin)
{
	out_be32(ioctl + PSC_TO_IOCTL_OFFSET(psc, pin), pin == 0 ? 0x00000007 : 0x00000003);
}
EXPORT_SYMBOL(mpc5121_pscgpio_make_psc);

void mpc5121_pscgpio_pin_high(int psc, int pin)
{
	setbits32(gpioctl+GPIODAT, 0x80000000 >> PSC_TO_GPIO(psc, pin));
}
EXPORT_SYMBOL(mpc5121_pscgpio_pin_high);

void mpc5121_pscgpio_pin_low(int psc, int pin)
{
	clrbits32(gpioctl+GPIODAT, 0x80000000 >> PSC_TO_GPIO(psc, pin));
}
EXPORT_SYMBOL(mpc5121_pscgpio_pin_low);

static int __init mpc5121_pscgpio_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,mpc5121-ioctl");
	if (np) {
		ioctl = of_iomap(np, 0);
		of_node_put(np);
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,mpc5121-gpio");
	if (np) {
		gpioctl = of_iomap(np, 0);
		of_node_put(np);
	}

	/* don't unmap these, they will be used later */
	/*
	 * iounmap(ioctl);
	 * iounmap(gpioctl);
	 */

	printk(KERN_INFO "mapped ioctl to %p and gpioctl to %p\n",
		(void *)ioctl, (void *)gpioctl);

	return 0;
}

arch_initcall(mpc5121_pscgpio_init);
