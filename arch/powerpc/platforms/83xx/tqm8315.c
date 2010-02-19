/*
 * arch/powerpc/platforms/83xx/tqm8315.c
 *
 * Description: TQM8315 board specific routines.
 *
 * Author: Oliver Weber <o.weber@gateware.de>
 * Copyright (C) TQ Components 2009.
 *
 * This file is based on mpc831x_rdb.c,
 * Author: Lo Wlison <r43300@freescale.com>
 * Copyright (C) Freescale Semiconductor, Inc. 2006.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/pci.h>
#include <linux/of_platform.h>

#include <linux/time.h>
#include <asm/ipic.h>
#include <asm/udbg.h>
#include <sysdev/fsl_pci.h>
#include <sysdev/fsl_soc.h>

#include "mpc83xx.h"

#define MPC8315_SATA_PHYCTRL_REG_OFFSET	0x15C
#define PHYCTRLCFG_REFCLK_MASK		0x00000070
#define PHYCTRLCFG_REFCLK_50MHZ		0x00000050
#define PHYCTRLCFG_REFCLK_75MHZ		0x00000000
#define PHYCTRLCFG_REFCLK_100MHZ	0x00000060
#define PHYCTRLCFG_REFCLK_125MHZ	0x00000070
#define PHYCTRLCFG_REFCLK_150MHZ	0x00000020


#ifdef CONFIG_SATA_FSL
void init_mpc8315_sata_phy(void)
{
	u32 val32;
	void __iomem *immap;

	immap = ioremap(get_immrbase() + 0x18000, 0x1000);
	if (immap == NULL)
		return;

	/* Configure PHY for 125 MHz reference clock */
	val32 = ioread32(immap + MPC8315_SATA_PHYCTRL_REG_OFFSET);
	val32 &= ~PHYCTRLCFG_REFCLK_MASK;
	val32 |= PHYCTRLCFG_REFCLK_125MHZ;
	iowrite32(val32, immap + MPC8315_SATA_PHYCTRL_REG_OFFSET);

	iounmap(immap);
}
#endif

/*
 * Setup the architecture
 */
static void __init tqm8315_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
#endif

	if (ppc_md.progress)
		ppc_md.progress("tqm8315_setup_arch()", 0);

#ifdef CONFIG_PCI
	for_each_compatible_node(np, "pci", "fsl,mpc8349-pci")
		mpc83xx_add_bridge(np);
	for_each_compatible_node(np, "pci", "fsl,mpc8314-pcie")
		mpc83xx_add_bridge(np);
#endif
	mpc831x_usb_cfg();

#ifdef CONFIG_SATA_FSL
	init_mpc8315_sata_phy();
#endif
}

static void __init tqm8315_init_IRQ(void)
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
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init tqm8315_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	return of_flat_dt_is_compatible(root, "TQM8315")
		|| of_flat_dt_is_compatible(root, "fsl,tqm8315");
}

static struct of_device_id __initdata of_bus_ids[] = {
	{.compatible = "simple-bus"},
	{.compatible = "gianfar"},
	{},
};

static int __init declare_of_platform_devices(void)
{
	of_platform_bus_probe(NULL, of_bus_ids, NULL);
	return 0;
}
machine_device_initcall(tqm8315, declare_of_platform_devices);

define_machine(tqm8315) {
	.name			= "TQM8315",
	.probe			= tqm8315_probe,
	.setup_arch		= tqm8315_setup_arch,
	.init_IRQ		= tqm8315_init_IRQ,
	.get_irq		= ipic_get_irq,
	.restart		= mpc83xx_restart,
	.time_init		= mpc83xx_time_init,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
#endif
};
