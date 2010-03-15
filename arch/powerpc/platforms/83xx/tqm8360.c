/*
 * Copyright (c) 2008 TQ-Systems GmbH
 *
 * Based on MPC8360E MDS and arch/ppc tqm834x ports
 *
 * Description:
 * TQM8360 board specific routines.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/initrd.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ipic.h>
#include <asm/irq.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include <sysdev/simple_gpio.h>
#include <asm/qe.h>
#include <asm/qe_ic.h>

#include "mpc83xx.h"

#undef DEBUG
#ifdef DEBUG
#define DBG(fmt...) udbg_printf(fmt)
#else
#define DBG(fmt...)
#endif

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init tqm8360_setup_arch(void)
{
	struct device_node *np;

	if (ppc_md.progress)
		ppc_md.progress("tqm8360_setup_arch()", 0);

#ifdef CONFIG_PCI
	for_each_compatible_node(np, "pci", "fsl,mpc8349-pci")
		mpc83xx_add_bridge(np);
#endif

#ifdef CONFIG_QUICC_ENGINE
	qe_reset();

	if ((np = of_find_node_by_name(NULL, "par_io")) != NULL) {
		par_io_init(np);
		of_node_put(np);

		for (np = NULL; (np = of_find_node_by_name(np, "ucc")) != NULL;)
			par_io_of_config(np);
	}

	if ((np = of_find_compatible_node(NULL, "network", "ucc_geth"))
			!= NULL){
		uint svid;

		/* handle mpc8360ea rev.2.1 erratum 2: RGMII Timing */
		svid = mfspr(SPRN_SVR);
		if (svid == 0x80480021) {
			void __iomem *immap;

			immap = ioremap(get_immrbase() + 0x14a8, 8);

			/*
			 * IMMR + 0x14A8[4:5] = 11 (clk delay for UCC 2)
			 * IMMR + 0x14A8[18:19] = 11 (clk delay for UCC 1)
			 */
			setbits32(immap, 0x0c003000);

			/*
			 * IMMR + 0x14AC[20:27] = 10101010
			 * (data delay for both UCC's)
			 */
			clrsetbits_be32(immap + 4, 0xff0, 0xaa0);

			iounmap(immap);
		}

		of_node_put(np);
	}
#endif				/* CONFIG_QUICC_ENGINE */
}

static struct of_device_id mpc836x_ids[] = {
	{ .type = "soc", },
	{ .compatible = "soc", },
	{ .compatible = "simple-bus", },
	{ .type = "qe", },
	{ .compatible = "fsl,qe", },
	{},
};

static int __init mpc836x_declare_of_platform_devices(void)
{
	/* Publish the QE devices */
	of_platform_bus_probe(NULL, mpc836x_ids, NULL);

	return 0;
}
machine_device_initcall(tqm8360, mpc836x_declare_of_platform_devices);

static void __init tqm8360_init_IRQ(void)
{
	struct device_node *np;

	np = of_find_node_by_type(NULL, "ipic");
	if (!np)
		return;

	ipic_init(np, 0);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();
	of_node_put(np);

#ifdef CONFIG_QUICC_ENGINE
	np = of_find_compatible_node(NULL, NULL, "fsl,qe-ic");
	if (!np) {
		np = of_find_node_by_type(NULL, "qeic");
		if (!np)
			return;
	}
	qe_ic_init(np, 0, qe_ic_cascade_low_ipic, qe_ic_cascade_high_ipic);
	of_node_put(np);
#endif				/* CONFIG_QUICC_ENGINE */
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init tqm8360_probe(void)
{
        unsigned long root = of_get_flat_dt_root();

        return of_flat_dt_is_compatible(root, "TQM8360");
}

define_machine(tqm8360) {
	.name		= "TQM8360",
	.probe		= tqm8360_probe,
	.setup_arch	= tqm8360_setup_arch,
	.init_IRQ	= tqm8360_init_IRQ,
	.get_irq	= ipic_get_irq,
	.restart	= mpc83xx_restart,
	.time_init	= mpc83xx_time_init,
	.calibrate_decr	= generic_calibrate_decr,
	.progress	= udbg_progress,
};
