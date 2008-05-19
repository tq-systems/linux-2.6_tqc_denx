/*
 * Copyright (C) 2007 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MPC512X_H__
#define __MPC512X_H__
extern int mpc512x_add_bridge(struct device_node *dev);
void __init mpc5121_ads_cpld_map(void);
void __init mpc5121_ads_cpld_pic_init(void);

extern int mpc5121ads_get_pendown_state(void);
extern void mpc5121_ads_cpld_uart_foff(int);

#endif				/* __MPC512X_H__ */
