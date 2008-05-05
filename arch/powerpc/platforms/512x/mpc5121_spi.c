/*
 * Copyright (C) 2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: John Rigby, <jrigby@freescale.com>, May 2008
 *
 * Description:
 * MPC5121 spi setup 
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/spi/spi.h>
#include <linux/fsl_devices.h>

#include <asm/io.h>

#include "mpc512x.h"

static u32 get_int_prop(struct device_node *np, const char *name, u32 def)
{
	const u32 *prop;
	int len;

	prop = of_get_property(np, name, &len);
	if (prop && len == 4)
		return *prop;
	return def;
}

#define GET_INT_PROP(pd, np, propname)	\
	(pd->propname = get_int_prop(np, #propname, pd->propname))


#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
#define CFG_ADS7846
#include <linux/spi/ads7846.h>
#endif

#ifdef CFG_ADS7846

static struct ads7846_platform_data mpc5121eads_ads7846_platform_data __initdata = {
	.model			= 7843,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 500,
	.y_plate_ohms		= 500,
	.get_pendown_state	= mpc5121ads_get_pendown_state,
};

static void __init *ads7846_get_pdata(struct device_node *np)
{
	struct ads7846_platform_data *pd = &mpc5121eads_ads7846_platform_data;
	GET_INT_PROP(pd, np, model);
	GET_INT_PROP(pd, np, vref_delay_usecs);
	GET_INT_PROP(pd, np, x_plate_ohms);
	GET_INT_PROP(pd, np, y_plate_ohms);

	return pd;
}
#endif

struct spi_driver_device {
	char *of_device;
	char *modalias;
	int needirq;
	void *(*get_platform_data)(struct device_node *);
};

static struct spi_driver_device spi_devices[] __initdata = {
#ifdef CONFIG_SPI_SPIDEV
	{
		.of_device = "linux,spidev",
		.modalias = "spidev",
	},
#endif
#ifdef CFG_ADS7846
	{
		.of_device = "ti,ads7846",
		.modalias = "ads7846",
		.needirq = 1,
		.get_platform_data = ads7846_get_pdata,
	},
#endif
#ifdef CONFIG_SND_SOC_AD1939
	{
		.of_device = "ad,ad1938",
		.modalias = "AD1939",
	},
	{
		.of_device = "ad,ad1939",
		.modalias ="AD1939",
	},
#endif
};

static int __init find_spi_driver(struct device_node *node,
				     struct spi_board_info *info)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(spi_devices); i++) {
		if (!of_device_is_compatible(node, spi_devices[i].of_device))
			continue;
		if (spi_devices[i].needirq && info->irq == NO_IRQ) {
			printk(KERN_WARNING "mpc5121_spi.c %s needs valid irq\n",
			       spi_devices[i].modalias);	
			return -EINVAL;
		}
		if (strlcpy(info->modalias, spi_devices[i].modalias,
			    KOBJ_NAME_LEN) >= KOBJ_NAME_LEN)
			return -ENOMEM;
		if (spi_devices[i].get_platform_data)
			info->platform_data = spi_devices[i].get_platform_data(node);
		return 0;
	}
	return -ENODEV;
}

static int of_device_add_data(struct of_device *of_dev, const void *data, size_t size)
{
	void *d;

	d = kmalloc(size, GFP_KERNEL);
	if (d) {
		memcpy(d, data, size);
		of_dev->dev.platform_data = d;
	}
	return d ? 0 : -ENOMEM;
}

static void __init register_spi_bus(struct device_node *spi_node, int bus_num)
{
	struct device_node *node = NULL;
	struct of_device *of_dev;
	struct fsl_spi_platform_data pdata = {
		.bus_num = bus_num,
		.max_chipselect = 255,
	};

	of_dev = of_find_device_by_node(spi_node);
	if (of_dev) {
		of_device_add_data(of_dev, &pdata, sizeof(pdata));
	}

	while ((node = of_get_next_child(spi_node, node))) {
		struct spi_board_info *bp, info = {};

		bp = &info;
			
		bp->bus_num = bus_num;
		GET_INT_PROP(bp, node, chip_select);
		GET_INT_PROP(bp, node, max_speed_hz);
		info.irq = irq_of_parse_and_map(node, 0);

		if (find_spi_driver(node, &info) < 0)
			continue;

		spi_register_board_info(&info, 1);
	}
}

static int __init mpc5121_spi_init(void)
{
	struct device_node *np;
	int bus_num;

	for_each_compatible_node(np, NULL, "fsl,mpc5121-psc-spi") {
		bus_num = get_int_prop(np, "cell-index", -1);
		if (bus_num < 0 || bus_num > 11) {
			printk(KERN_WARNING "mpc5121_spi.c no cell-index spi node, skipping\n");
			continue;
		}
		register_spi_bus(np, bus_num);
		of_node_put(np);
	}

	return 0;
}

arch_initcall(mpc5121_spi_init);
