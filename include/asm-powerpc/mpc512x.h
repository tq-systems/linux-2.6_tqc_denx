/*
 * Copyright (C) 2007,2008 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: John Rigby, <jrigby@freescale.com>, Friday Apr 13 2007
 *
 * Description:
 * MPC5121 Prototypes and definitions
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_POWERPC_MPC512x_H__
#define __ASM_POWERPC_MPC512x_H__

/*
 * DDR Memory Controller Memory Map
 */
struct ddr512x {
	u32 ddr_sys_config;	/* System Configuration Register */
	u32 ddr_time_config0;	/* Timing Configuration Register */
	u32 ddr_time_config1;	/* Timing Configuration Register */
	u32 ddr_time_config2;	/* Timing Configuration Register */
	u32 ddr_command;	/* Command Register */
	u32 ddr_compact_command;	/* Compact Command Register */
	u32 self_refresh_cmd_0;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_1;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_2;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_3;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_4;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_5;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_6;	/* Enter/Exit Self Refresh Registers */
	u32 self_refresh_cmd_7;	/* Enter/Exit Self Refresh Registers */
	u32 DQS_config_offset_count;	/* DQS Config Offset Count */
	u32 DQS_config_offset_time;	/* DQS Config Offset Time */
	u32 DQS_delay_status;	/* DQS Delay Status */
	u32 res0[0xF];
	u32 prioman_config1;	/* Priority Manager Configuration */
	u32 prioman_config2;	/* Priority Manager Configuration */
	u32 hiprio_config;	/* High Priority Configuration */
	u32 lut_table0_main_upper;	/* LUT0 Main Upper */
	u32 lut_table1_main_upper;	/* LUT1 Main Upper */
	u32 lut_table2_main_upper;	/* LUT2 Main Upper */
	u32 lut_table3_main_upper;	/* LUT3 Main Upper */
	u32 lut_table4_main_upper;	/* LUT4 Main Upper */
	u32 lut_table0_main_lower;	/* LUT0 Main Lower */
	u32 lut_table1_main_lower;	/* LUT1 Main Lower */
	u32 lut_table2_main_lower;	/* LUT2 Main Lower */
	u32 lut_table3_main_lower;	/* LUT3 Main Lower */
	u32 lut_table4_main_lower;	/* LUT4 Main Lower */
	u32 lut_table0_alternate_upper;	/* LUT0 Alternate Upper */
	u32 lut_table1_alternate_upper; /* LUT1 Alternate Upper */
	u32 lut_table2_alternate_upper; /* LUT2 Alternate Upper */
	u32 lut_table3_alternate_upper; /* LUT3 Alternate Upper */
	u32 lut_table4_alternate_upper; /* LUT4 Alternate Upper */
	u32 lut_table0_alternate_lower; /* LUT0 Alternate Lower */
	u32 lut_table1_alternate_lower; /* LUT1 Alternate Lower */
	u32 lut_table2_alternate_lower; /* LUT2 Alternate Lower */
	u32 lut_table3_alternate_lower; /* LUT3 Alternate Lower */
	u32 lut_table4_alternate_lower; /* LUT4 Alternate Lower */
	u32 performance_monitor_config;
	u32 event_time_counter;
	u32 event_time_preset;
	u32 performance_monitor1_address_low;
	u32 performance_monitor2_address_low;
	u32 performance_monitor1_address_hi;
	u32 performance_monitor2_address_hi;
	u32 res1[2];
	u32 performance_monitor1_read_counter;
	u32 performance_monitor2_read_counter;
	u32 performance_monitor1_write_counter;
	u32 performance_monitor2_write_counter;
	u32 granted_ack_counter0;
	u32 granted_ack_counter1;
	u32 granted_ack_counter2;
	u32 granted_ack_counter3;
	u32 granted_ack_counter4;
	u32 cumulative_wait_counter0;
	u32 cumulative_wait_counter1;
	u32 cumulative_wait_counter2;
	u32 cumulative_wait_counter3;
	u32 cumulative_wait_counter4;
	u32 summed_priority_counter0;
	u32 summed_priority_counter1;
	u32 summed_priority_counter2;
	u32 summed_priority_counter3;
	u32 summed_priority_counter4;
	u32 res2[0x3AD];
};

#define MPC512x_DDR_BASE	0x9000	/* Offset of DRAM controller */

struct clk;

extern unsigned long mpc512x_find_ips_freq(struct device_node *node);
extern struct clk *clk_get(struct device *dev, const char *id);
extern int clk_enable(struct clk *clk);


/*
 * helper routines for switching psc pins to gpios and back
 * and driving them high or low
 */
extern void mpc5121_pscgpio_make_gpio(int psc, int pin);
extern void mpc5121_pscgpio_pin_high(int psc, int pin);
extern void mpc5121_pscgpio_pin_low(int psc, int pin);
extern void mpc5121_pscgpio_make_psc(int psc, int pin);

#endif /* __ASM_POWERPC_MPC512x_H__ */

