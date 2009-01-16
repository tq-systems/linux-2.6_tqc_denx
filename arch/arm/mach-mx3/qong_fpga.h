/*
 * Dave/DENX Qong FPGA address space definitions
 *
 * Copyright (C) 2008 Dave S.r.l. <www.dave.eu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _QONG_FPGA_H
#define _QONG_FPGA_H

#include <mach/hardware.h>
#include <mach/irqs.h>

#define QONG_FPGA_VERSION(major, minor, rev)	\
	(((major & 0xF) << 12) | ((minor & 0xF) << 8) | (rev & 0xFF))

#define QONG_FPGA_BASEADDR 		CS1_BASE_ADDR
#define QONG_PERIPH_SIZE 		(1 << 24)

#define QONG_FPGA_CTRL_BASEADDR		QONG_FPGA_BASEADDR
#define QONG_FPGA_CTRL_SIZE 		0x10

#define QONG_DNET_ID		1
#define QONG_DNET_BASEADDR	\
	(QONG_FPGA_BASEADDR + QONG_DNET_ID * QONG_PERIPH_SIZE)
#define QONG_DNET_SIZE 		0x00001000

#define QONG_FPGA_IRQ		IOMUX_TO_IRQ(MX31_PIN_DTR_DCE1)
#define QONG_FPGA_IRQ_GPIO_POR	(2-1)
#define QONG_FPGA_IRQ_GPIO_NUM	(8)

/* FPGA control registers */
#define QONG_FPGA_CTRL_VERSION	 		0x00

#endif /* _QONG_FPGA_H */
