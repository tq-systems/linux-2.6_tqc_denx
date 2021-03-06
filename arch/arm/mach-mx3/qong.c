/*
 *  Copyright (C) 2009 Ilya Yanok, Emcraft Systems Ltd, <yanok@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <mach/common.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <mach/board-qong.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx3.h>
#include <mach/ipu.h>
#include <mach/mx3fb.h>
#include <linux/spi/spi.h>
#include <linux/mfd/mc13783.h>

#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)
#include <mach/spi.h>
#endif
#include "devices.h"

/* FPGA defines */
#define QONG_FPGA_VERSION(major, minor, rev)	\
	(((major & 0xF) << 12) | ((minor & 0xF) << 8) | (rev & 0xFF))

#define QONG_FPGA_BASEADDR 		CS1_BASE_ADDR
#define QONG_FPGA_PERIPH_SIZE 		(1 << 24)

#define QONG_FPGA_CTRL_BASEADDR		QONG_FPGA_BASEADDR
#define QONG_FPGA_CTRL_SIZE 		0x10
/* FPGA control registers */
#define QONG_FPGA_CTRL_VERSION		0x00

#define QONG_DNET_ID		1
#define QONG_DNET_BASEADDR	\
	(QONG_FPGA_BASEADDR + QONG_DNET_ID * QONG_FPGA_PERIPH_SIZE)
#define QONG_DNET_SIZE 		0x00001000

#define QONG_FPGA_IRQ		IOMUX_TO_IRQ(MX31_PIN_DTR_DCE1)

/*
 * This file contains the board-specific initialization routines.
 */

static struct imxuart_platform_data uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static int uart_pins[] = {
	MX31_PIN_CTS1__CTS1,
	MX31_PIN_RTS1__RTS1,
	MX31_PIN_TXD1__TXD1,
	MX31_PIN_RXD1__RXD1
};

static inline void mxc_init_imx_uart(void)
{
	mxc_iomux_setup_multiple_pins(uart_pins, ARRAY_SIZE(uart_pins),
			"uart-0");
	mxc_register_device(&mxc_uart_device0, &uart_pdata);
}

static struct resource dnet_resources[] = {
	{
		.name	= "dnet-memory",
		.start	= QONG_DNET_BASEADDR,
		.end	= QONG_DNET_BASEADDR + QONG_DNET_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= QONG_FPGA_IRQ,
		.end	= QONG_FPGA_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device dnet_device = {
	.name			= "dnet",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(dnet_resources),
	.resource		= dnet_resources,
};

static int __init qong_init_dnet(void)
{
	int ret;

	ret = platform_device_register(&dnet_device);
	return ret;
}

/* MTD NOR flash */

static struct physmap_flash_data qong_flash_data = {
	.width = 2,
};

static struct resource qong_flash_resource = {
	.start = CS0_BASE_ADDR,
	.end = CS0_BASE_ADDR + QONG_NOR_SIZE - 1,
	.flags = IORESOURCE_MEM,
};

static struct platform_device qong_nor_mtd_device = {
	.name = "physmap-flash",
	.id = 0,
	.dev = {
		.platform_data = &qong_flash_data,
		},
	.resource = &qong_flash_resource,
	.num_resources = 1,
};

static void qong_init_nor_mtd(void)
{
	(void)platform_device_register(&qong_nor_mtd_device);
}

/*
 * Hardware specific access to control-lines
 */
static void qong_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *nand_chip = mtd->priv;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		writeb(cmd, nand_chip->IO_ADDR_W + (1 << 24));
	else
		writeb(cmd, nand_chip->IO_ADDR_W + (1 << 23));
}

/*
 * Read the Device Ready pin.
 */
static int qong_nand_device_ready(struct mtd_info *mtd)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX31_PIN_NFRB));
}

static void qong_nand_select_chip(struct mtd_info *mtd, int chip)
{
	if (chip >= 0)
		gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_NFCE_B), 0);
	else
		gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_NFCE_B), 1);
}

static struct platform_nand_data qong_nand_data = {
	.chip = {
		.chip_delay		= 20,
		.options		= 0,
	},
	.ctrl = {
		.cmd_ctrl 		= qong_nand_cmd_ctrl,
		.dev_ready		= qong_nand_device_ready,
		.select_chip		= qong_nand_select_chip,
	}
};

static struct resource qong_nand_resource = {
	.start  	= CS3_BASE_ADDR,
	.end    	= CS3_BASE_ADDR + SZ_32M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device qong_nand_device = {
	.name		= "gen_nand",
	.id		= -1,
	.dev		= {
		.platform_data = &qong_nand_data,
	},
	.num_resources	= 1,
	.resource	= &qong_nand_resource,
};

static void __init qong_init_nand_mtd(void)
{
	/* init CS */
	__raw_writel(0x00004f00, CSCR_U(3));
	__raw_writel(0x20013b31, CSCR_L(3));
	__raw_writel(0x00020800, CSCR_A(3));
	mxc_iomux_set_gpr(MUX_SDCTL_CSD1_SEL, true);

	/* enable pin */
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_NFCE_B, IOMUX_CONFIG_GPIO));
	if (!gpio_request(IOMUX_TO_GPIO(MX31_PIN_NFCE_B), "nand_enable"))
		gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_NFCE_B), 0);

	/* ready/busy pin */
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_NFRB, IOMUX_CONFIG_GPIO));
	if (!gpio_request(IOMUX_TO_GPIO(MX31_PIN_NFRB), "nand_rdy"))
		gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_NFRB));

	/* write protect pin */
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_NFWP_B, IOMUX_CONFIG_GPIO));
	if (!gpio_request(IOMUX_TO_GPIO(MX31_PIN_NFWP_B), "nand_wp"))
		gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_NFWP_B), 1);

	platform_device_register(&qong_nand_device);
}

static void __init qong_init_fpga(void)
{
	void __iomem *regs;
	u32 fpga_ver;

	regs = ioremap(QONG_FPGA_CTRL_BASEADDR, QONG_FPGA_CTRL_SIZE);
	if (!regs) {
		printk(KERN_ERR "%s: failed to map registers, aborting.\n",
				__func__);
		return;
	}

	fpga_ver = readl(regs + QONG_FPGA_CTRL_VERSION);
	iounmap(regs);
	printk(KERN_INFO "Qong FPGA version %d.%d.%d\n",
			(fpga_ver & 0xF000) >> 12,
			(fpga_ver & 0x0F00) >> 8, fpga_ver & 0x00FF);
	if (fpga_ver < QONG_FPGA_VERSION(0, 8, 7)) {
		printk(KERN_ERR "qong: Unexpected FPGA version, FPGA-based "
				"devices won't be registered!\n");
		return;
	}

	/* register FPGA-based devices */
	qong_init_nand_mtd();
	qong_init_dnet();
}

static struct ipu_platform_data mx3_ipu_data = {
	.irq_base = MXC_IPU_IRQ_START,
};

static const struct fb_videomode fb_modedb[] = {
	{
		/* 320x240 @ 60 Hz Vbest */
		.name		= "Vbest-VGG322403",
		.refresh	= 60,
		.xres		= 320,
		.yres		= 240,
		.pixclock	= 156000,
		.left_margin	= 20,
		.right_margin	= 38,
		.upper_margin	= 7,
		.lower_margin	= 26,
		.hsync_len	= 30,
		.vsync_len	= 3,
		.sync		= FB_SYNC_OE_ACT_HIGH,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	}, {
		/* 640x480 @ 60 Hz Casio COM57H5M10XRC */
		.name		= "COM57H5M10XRC",
		.refresh	= 60,
		.xres		= 640,
		.yres		= 480,
		.pixclock	= 40000,
		.left_margin	= 120,
		.right_margin	= 10,
		.upper_margin	= 35,
		.lower_margin	= 7,
		.hsync_len	= 30,
		.vsync_len	= 3,
		.sync		= FB_SYNC_OE_ACT_HIGH,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	},
};

static struct mx3fb_platform_data mx3fb_pdata = {
	.dma_dev	= &mx3_ipu.dev,
	.name		= "Vbest-VGG322403",
	.mode		= fb_modedb,
	.num_modes	= ARRAY_SIZE(fb_modedb),
};

static void __init qong_init_lcd(void)
{
	/* Init Display Interface */
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD0, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD1, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD2, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD3, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD4, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD5, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD6, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD7, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD8, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD9, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD10, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD11, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD12, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD13, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD14, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD15, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD16, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_LD17, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_VSYNC3, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_HSYNC, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_FPSHIFT, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_DRDY0, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_D3_REV, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_CONTRAST, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_D3_SPL, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_D3_CLS, IOMUX_CONFIG_FUNC));

	mxc_register_device(&mx3_ipu, &mx3_ipu_data);
	mxc_register_device(&mx3_fb, &mx3fb_pdata);
}

#if defined(CONFIG_MFD_MC13783) || defined(CONFIG_MFD_MC13783_MODULE)
static struct mc13783_platform_data mc13783_pdata __initdata = {
	.flags = MC13783_USE_TOUCHSCREEN,
};

static struct spi_board_info qong_spi_devs[] __initdata = {
	{
		.modalias	= "mc13783",
		.max_speed_hz	= 1000000,
		.bus_num	= 1,
		.chip_select	= 0,
		.irq		= IOMUX_TO_IRQ(MX31_PIN_GPIO1_3),
		.platform_data	= &mc13783_pdata,
	},
};

static int qong_spi1_cs[] = { MXC_SPI_CS(0) };

struct spi_imx_master qong_spi1_master = {
	.chipselect = qong_spi1_cs,
	.num_chipselect = ARRAY_SIZE(qong_spi1_cs),
};

static void __init qong_init_spi(void)
{
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_CSPI2_SPI_RDY, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_CSPI2_SCLK, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_CSPI2_MISO, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_CSPI2_MOSI, IOMUX_CONFIG_FUNC));
	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_CSPI2_SS0, IOMUX_CONFIG_FUNC));

	mxc_iomux_mode(IOMUX_MODE(MX31_PIN_GPIO1_3, IOMUX_CONFIG_GPIO));

	if (!gpio_request(IOMUX_TO_GPIO(MX31_PIN_GPIO1_3), "spi1_irq"))
		gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_GPIO1_3));

	spi_register_board_info(qong_spi_devs, ARRAY_SIZE(qong_spi_devs));
	mxc_register_device(&mxc_spi_device1, &qong_spi1_master);
}
#endif

/*
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_init_imx_uart();
	qong_init_nor_mtd();
	qong_init_fpga();
	qong_init_lcd();
#if defined(CONFIG_MFD_MC13783) || defined(CONFIG_MFD_MC13783_MODULE)
	qong_init_spi();
#endif
}

static void __init qong_timer_init(void)
{
	mx31_clocks_init(26000000);
}

static struct sys_timer qong_timer = {
	.init	= qong_timer_init,
};

/*
 * The following uses standard kernel macros defined in arch.h in order to
 * initialize __mach_desc_QONG data structure.
 */

MACHINE_START(QONG, "Dave/DENX QongEVB-LITE")
	/* Maintainer: DENX Software Engineering GmbH */
	.phys_io        = AIPS1_BASE_ADDR,
	.io_pg_offst    = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = mx31_map_io,
	.init_irq       = mx31_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &qong_timer,
MACHINE_END
