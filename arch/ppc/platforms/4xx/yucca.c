/*
 * Yucca board specific routines
 *
 * Roland Dreier <rolandd@cisco.com> (based on luan.c by Matt Porter)
 *
 * Copyright 2004-2005 MontaVista Software Inc.
 * Copyright (c) 2005 Cisco Systems.  All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
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
#include <linux/types.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/initrd.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ocp.h>
#include <asm/pci-bridge.h>
#include <asm/time.h>
#include <asm/todc.h>
#include <asm/bootinfo.h>
#include <asm/ppc4xx_pic.h>
#include <asm/ppcboot.h>
#include <asm/tlbflush.h>

#include <syslib/gen550.h>
#include <syslib/ibm44x_common.h>
#include <syslib/ibm440gx_common.h>
#include <syslib/ibm440sp_common.h>
#include <syslib/ppc4xx_pcie.h>

extern bd_t __res;

static struct ibm44x_clocks clocks __initdata;

unsigned char ppc4xx_uic_ext_irq_cfg[] __initdata = {
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ15: EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ14: EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ13: EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ12: PCI-X slot */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ11: EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ10: EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ9:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ8:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ7:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ6:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ5:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ4:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ3:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ2:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ1:  EXT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* IRQ0:  EXT */
};

static void __init
yucca_calibrate_decr(void)
{
	unsigned int freq;

	if (mfspr(SPRN_CCR1) & CCR1_TCS)
		freq = YUCCA_TMR_CLK;
	else
		freq = clocks.cpu;

	ibm44x_calibrate_decr(freq);
}

static int
yucca_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: AMCC\n");
	seq_printf(m, "machine\t\t: PPC440SPe EVB (Yucca)\n");
	ibm440gx_show_cpuinfo(m);

	return 0;
}

static void __init yucca_set_emacdata(void)
{
	struct ocp_def *def;
	struct ocp_func_emac_data *emacdata;

	/* Set phy_map, phy_mode, and mac_addr for the EMAC */
	def = ocp_get_one_device(OCP_VENDOR_IBM, OCP_FUNC_EMAC, 0);
	emacdata = def->additions;
	emacdata->phy_map = 0x00000001;	/* Skip 0x00 */
	emacdata->phy_mode = PHY_MODE_GMII;
	memcpy(emacdata->mac_addr, __res.bi_enetaddr, 6);
}

enum yucca_hoses {
	HOSE_PCIX,
	HOSE_PCIE0,
	HOSE_PCIE1,
	HOSE_PCIE2,
	HOSE_MAX
};

static enum yucca_hoses hose_type[4];

#define is_pcix_hose(_hs_) ((_hs_) == HOSE_PCIX)
#define is_pcie_hose(_hs_) (((_hs_) >= HOSE_PCIE0) && ((_hs_) <= HOSE_PCIE2))
#define pcie_hose_num(_hs_) ((_hs_) - HOSE_PCIE0)

#define PCIX_READW(offset) \
	(readw((void *)((u32)pcix_reg_base+offset)))

#define PCIX_WRITEW(value, offset) \
	(writew(value, (void *)((u32)pcix_reg_base+offset)))

#define PCIX_WRITEL(value, offset) \
	(writel(value, (void *)((u32)pcix_reg_base+offset)))

static void __init
ppc440spe_setup_pcix(struct pci_controller *hose)
{
	void *pcix_reg_base;

	pcix_reg_base = ioremap64(PCIX0_REG_BASE, PCIX_REG_SIZE);

	/* Disable all windows */
	PCIX_WRITEL(0, PCIX0_POM0SA);
	PCIX_WRITEL(0, PCIX0_POM1SA);
	PCIX_WRITEL(0, PCIX0_POM2SA);
	PCIX_WRITEL(0, PCIX0_PIM0SA);
	PCIX_WRITEL(0, PCIX0_PIM0SAH);
	PCIX_WRITEL(0, PCIX0_PIM1SA);
	PCIX_WRITEL(0, PCIX0_PIM2SA);
	PCIX_WRITEL(0, PCIX0_PIM2SAH);

	/*
	 * Setup 512MB PLB->PCI outbound mem window
	 * (a_n000_0000->0_n000_0000)
	 * */
	PCIX_WRITEL(0x0000000d, PCIX0_POM0LAH);
	PCIX_WRITEL(hose->mem_space.start, PCIX0_POM0LAL);
	PCIX_WRITEL(0x00000000, PCIX0_POM0PCIAH);
	PCIX_WRITEL(hose->mem_space.start, PCIX0_POM0PCIAL);
	PCIX_WRITEL(~(hose->mem_space.end - hose->mem_space.start) | 1 ,
			PCIX0_POM0SA);

	/* Setup 1GB PCI->PLB inbound memory window at 0, enable MSIs */
	PCIX_WRITEL(0x00000000, PCIX0_PIM0LAH);
	PCIX_WRITEL(0x00000000, PCIX0_PIM0LAL);
	PCIX_WRITEL(0xc0000007, PCIX0_PIM0SA);
	PCIX_WRITEL(0xffffffff, PCIX0_PIM0SAH);

	/* Enable PCIX0 I/O, Mem, and Busmaster cycles */
	PCIX_WRITEW(PCIX_READW(PCIX0_COMMAND) | PCI_COMMAND_MEMORY |
			PCI_COMMAND_MASTER, PCIX0_COMMAND);

	iounmap(pcix_reg_base);
	eieio();
}

static inline int
yucca_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);

	if (hose_type[hose->index] == HOSE_PCIX) {
		static char pci_irq_table[][4] =
		/*
		 *	PCI IDSEL/INTPIN->INTLINE
		 *	  A   B   C   D
		 */
		{
			{ 49, -1, -1, -1 },	/* IDSEL 1 - PCIX0 Slot 0 */
		};
		const long min_idsel = 1, max_idsel = 1, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else if (hose_type[hose->index] == HOSE_PCIE0) {
		static char pci_irq_table[][4] =
		/*
		 *	PCI IDSEL/INTPIN->INTLINE
		 *	  A   B   C   D
		 */
		{
			{ 96, 97, 98, 99 },
		};
		const long min_idsel = 1, max_idsel = 1, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else if (hose_type[hose->index] == HOSE_PCIE1) {
		static char pci_irq_table[][4] =
		/*
		 *	PCI IDSEL/INTPIN->INTLINE
		 *	  A   B   C   D
		 */
		{
			{ 100, 101, 102, 103 },
		};
		const long min_idsel = 1, max_idsel = 1, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else if (hose_type[hose->index] == HOSE_PCIE2) {
		static char pci_irq_table[][4] =
		/*
		 *	PCI IDSEL/INTPIN->INTLINE
		 *	  A   B   C   D
		 */
		{
			{ 104, 105, 106, 107 },
		};
		const long min_idsel = 1, max_idsel = 1, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
	return -1;
}

static int __init yucca_pcie_card_present(int port)
{
   void __iomem *pcie_fpga_base;
   u16 reg;

   pcie_fpga_base = ioremap64(YUCCA_FPGA_REG_BASE, YUCCA_FPGA_REG_SIZE);
   reg = in_be16(pcie_fpga_base + FPGA_REG1C);
   iounmap(pcie_fpga_base);

   switch(port) {
   case 0: return !(reg & FPGA_REG1C_PE0_PRSNT);
   case 1: return !(reg & FPGA_REG1C_PE1_PRSNT);
   case 2: return !(reg & FPGA_REG1C_PE2_PRSNT);
   default: return 0;
   }
}

/*
 * For the given slot, set rootpoint mode, send power to the slot,
 * turn on the green LED and turn off the yellow LED, enable the clock
 * and turn off reset.
 */
static void __init yucca_setup_pcie_fpga_root_or_endpoint(int port)
{
	void __iomem *pcie_reg_fpga_base;
	u16 power, clock, green_led, yellow_led, reset_off, rootpoint, endpoint;

	pcie_reg_fpga_base = ioremap64(YUCCA_FPGA_REG_BASE, YUCCA_FPGA_REG_SIZE);

	switch(port) {
	case 0:
		rootpoint   = FPGA_REG1C_PE0_ROOTPOINT;
		endpoint    = 0;
		power 	    = FPGA_REG1A_PE0_PWRON;
		green_led   = FPGA_REG1A_PE0_GLED;
		clock 	    = FPGA_REG1A_PE0_REFCLK_ENABLE;
		yellow_led  = FPGA_REG1A_PE0_YLED;
		reset_off   = FPGA_REG1C_PE0_PERST;
		break;
	case 1:
		rootpoint   = 0;
		endpoint    = FPGA_REG1C_PE1_ENDPOINT;
		power 	    = FPGA_REG1A_PE1_PWRON;
		green_led   = FPGA_REG1A_PE1_GLED;
		clock 	    = FPGA_REG1A_PE1_REFCLK_ENABLE;
		yellow_led  = FPGA_REG1A_PE1_YLED;
		reset_off   = FPGA_REG1C_PE1_PERST;
		break;
	case 2:
		rootpoint   = 0;
		endpoint    = FPGA_REG1C_PE2_ENDPOINT;
		power 	    = FPGA_REG1A_PE2_PWRON;
		green_led   = FPGA_REG1A_PE2_GLED;
		clock 	    = FPGA_REG1A_PE2_REFCLK_ENABLE;
		yellow_led  = FPGA_REG1A_PE2_YLED;
		reset_off   = FPGA_REG1C_PE2_PERST;
		break;

	default:
		return;
	}

	out_be16(pcie_reg_fpga_base + FPGA_REG1A,
		 ~(power | clock | green_led) &
		 (yellow_led | in_be16(pcie_reg_fpga_base + FPGA_REG1A)));
#ifdef CONFIG_PCIE_ENDPOINT
	/* reset or unreset can only be done from root */
	out_be16(pcie_reg_fpga_base + FPGA_REG1C,
		 ~(rootpoint | reset_off) &
		 (endpoint | in_be16(pcie_reg_fpga_base + FPGA_REG1C)));
#else
	out_be16(pcie_reg_fpga_base + FPGA_REG1C,
		 ~(endpoint | reset_off) &
		 (rootpoint | in_be16(pcie_reg_fpga_base + FPGA_REG1C)));

	/*
	 * Leave device in reset for a while after powering on the
	 * slot to give it a chance to initialize.
	 */
	mdelay(250);

	out_be16(pcie_reg_fpga_base + FPGA_REG1C,
		 reset_off | in_be16(pcie_reg_fpga_base + FPGA_REG1C));
#endif
	iounmap(pcie_reg_fpga_base);
}

static void __init
yucca_setup_hoses(void)
{
	struct pci_controller *hose;
	char name[20];
	enum yucca_hoses hs;
	int bus_no = 0;

	for (hs = HOSE_PCIX; hs < HOSE_MAX; ++hs) {
		if (is_pcie_hose(hs)) {
			if (!yucca_pcie_card_present(pcie_hose_num(hs)))
				continue;

			pr_debug("PCIE%d: card present\n", pcie_hose_num(hs));

			yucca_setup_pcie_fpga_root_or_endpoint(pcie_hose_num(hs));
			if (ppc4xx_init_pcie_root_or_endport(pcie_hose_num(hs))) {
				printk(KERN_ERR "PCIE%d: initialization "
				       "failed\n", pcie_hose_num(hs));
				continue;
			}
		}

		hose = pcibios_alloc_controller();
		if (!hose)
			return;

		sprintf(name, "PCI%s%d host bridge",
			is_pcix_hose(hs) ? "X" : "E",
			is_pcie_hose(hs) ? pcie_hose_num(hs) : 0);

		if (is_pcix_hose(hs)) {
			hose->mem_space.start = YUCCA_PCIX_LOWER_MEM;
			hose->mem_space.end = hose->mem_space.start +
				YUCCA_PCIX_MEM_SIZE - 1;
		} else {
			hose->mem_space.start = YUCCA_PCIE_LOWER_MEM +
				pcie_hose_num(hs) * YUCCA_PCIE_MEM_SIZE;
			hose->mem_space.end = hose->mem_space.start +
				YUCCA_PCIE_MEM_SIZE - 1;
		}

		pci_init_resource(&hose->mem_resources[0],
				  hose->mem_space.start,
				  hose->mem_space.end,
				  IORESOURCE_MEM,
				  name);

		if (is_pcix_hose(hs))
			isa_io_base = (unsigned long) ioremap64(PCIX0_IO_BASE,
								PCIX_IO_SIZE);

		hose->io_base_virt = (void *)isa_io_base;
		hose->io_space.start = YUCCA_PCIX_LOWER_IO +
			((unsigned int)hs * YUCCA_PCI_HOST_SIZE_IO);
		hose->io_space.end = hose->io_space.start +
			YUCCA_PCI_HOST_SIZE_IO - 1;

		pci_init_resource(&hose->io_resource,
				  hose->io_space.start,
				  hose->io_space.end,
				  IORESOURCE_IO,
				  name);

		hose->first_busno = bus_no;
		hose->last_busno = 0xFF;
		hose_type[hose->index] = hs;

		if (is_pcix_hose(hs)) {
			ppc440spe_setup_pcix(hose);
			setup_indirect_pci(hose, PCIX0_CFGA, PCIX0_CFGD);
			hose->set_cfg_type = 1;
		} else {
			if (ppc4xx_setup_pcie(hose, pcie_hose_num(hs)) != 0) {
				printk(KERN_WARNING
				       "PCIE setup failed for hose no %d\n",
				       pcie_hose_num(hs));
				continue;
			}
		}

		/*
		 * Some cards like LSI8408E need delay before enumeration.
		 * At this point calibrate_delay hasn't been called yet so
		 * the mdelay value does not reflect exact millisecs value.
		 */
		mdelay(10000);

		hose->last_busno = pciauto_bus_scan(hose, hose->first_busno);
		bus_no = hose->last_busno + 1;
		pr_debug("%s: resources allocated\n", name);
	}

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = yucca_map_irq;
}

TODC_ALLOC();

static void __init
yucca_early_serial_map(void)
{
	struct uart_port port;

	/* Setup ioremapped serial port access */
	memset(&port, 0, sizeof(port));
	port.membase = ioremap64(PPC440SPE_UART0_ADDR, 8);
	port.irq = UART0_INT;
	port.uartclk = clocks.uart0;
	port.regshift = 0;
	port.iotype = UPIO_MEM;
	port.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	port.line = 0;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 0 failed\n");
	}

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	/* Configure debug serial access */
	gen550_init(0, &port);

	/* Purge TLB entry added in head_44x.S for early serial access */
	_tlbie(UART0_IO_BASE);
#endif

	port.membase = ioremap64(PPC440SPE_UART1_ADDR, 8);
	port.irq = UART1_INT;
	port.uartclk = clocks.uart1;
	port.line = 1;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 1 failed\n");
	}

	port.membase = ioremap64(PPC440SPE_UART2_ADDR, 8);
	port.irq = UART2_INT;
	port.uartclk = BASE_BAUD;
	port.line = 2;

	if (early_serial_setup(&port) != 0) {
		printk("Early serial init of port 2 failed\n");
	}
}

static void __init
yucca_setup_arch(void)
{
	yucca_set_emacdata();

#if !defined(CONFIG_BDI_SWITCH)
	/*
	 * The Abatron BDI JTAG debugger does not tolerate others
	 * mucking with the debug registers.
	 */
	mtspr(SPRN_DBCR0, (DBCR0_TDE | DBCR0_IDM));
#endif

	/*
	 * Determine various clocks.
	 * To be completely correct we should get SysClk
	 * from FPGA, because it can be changed by on-board switches
	 * --ebs
	 */
	/* 440GX and 440SPe clocking is the same - rd */
	ibm440gx_get_clocks(&clocks, 33333333, 6 * 1843200);
	ocp_sys_info.opb_bus_freq = clocks.opb;

	/* init to some ~sane value until calibrate_delay() runs */
	loops_per_jiffy = 50000000/HZ;

	/* Setup PCIXn host bridges */
	yucca_setup_hoses();

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#ifdef CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_HDA1;
#endif

	yucca_early_serial_map();

	/* Identify the system */
	printk("Yucca port (Roland Dreier <rolandd@cisco.com>)\n");
}

void __init platform_init(unsigned long r3, unsigned long r4,
		unsigned long r5, unsigned long r6, unsigned long r7)
{
	ibm44x_platform_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = yucca_setup_arch;
	ppc_md.show_cpuinfo = yucca_show_cpuinfo;
	ppc_md.find_end_of_memory = ibm440sp_find_end_of_memory;
	ppc_md.get_irq = NULL;		/* Set in ppc4xx_pic_init() */

	ppc_md.calibrate_decr = yucca_calibrate_decr;
#ifdef CONFIG_KGDB
	ppc_md.early_serial_map = yucca_early_serial_map;
#endif
}
