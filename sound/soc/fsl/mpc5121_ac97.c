/*
 * Copyright 2007,2008 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  Freescale AC97 SoC device driver for CPU MPC5121
 *
 *  Author: Hongjun Chen <hong-jun.chen@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/delay.h>
#include <linux/mutex.h>
#include <sound/ac97_codec.h>
#include <sound/soc.h>
#include <asm/mpc52xx_psc.h>
#include "mpc5121_psc_info.h"

/* AC97CMD register bit definitions */
#define	RD_FLG  		(1 << 31)
#define	CTRL_REG_INDEX(reg) 	(reg << 24)
#define	CMD_DATA(data)	 	(data << 8)
#define RD_CMD(reg, data) 	(RD_FLG | CTRL_REG_INDEX(reg) | CMD_DATA(data))
#define	WR_CMD(reg, data) 	(CTRL_REG_INDEX(reg) | CMD_DATA(data))

/* SR(status register) bit definitions */
enum {
	UNEX_RX_SLOT = (1 << 0),
	DATA_VALID = (1 << 1),
	DATA_OVR = (1 << 2),
	CMD_SEND = (1 << 3),
	ERR_PSC = (1 << 6),
	UNDER_RUN_ERR = (1 << 11),
	OVER_RUN_ERR = (1 << 12),
};

static DEFINE_MUTEX(car_mutex);

unsigned short mpc5121_ac97_read(struct snd_ac97 *ac97, unsigned short reg)
{
	unsigned short val = 0;
	struct mpc52xx_psc *psc_reg;
	u32 temp, r, v;
	int timeout = 5000;
	int retries = 10;

	/* For mysterious cause, this delay time is needed to right
	 * access of codec's register
	 */
	udelay(40);
	mutex_lock(&car_mutex);

	psc_reg = psc_reg_priv;

	/* Make sure status data register is empty */
	while (in_be16(&psc_reg->sr_csr.status) & DATA_VALID)
		temp = in_be32(&psc_reg->ac97data);

	/* Write READ register command in slot0 and 1 */
	out_be32(&psc_reg->ac97cmd, RD_CMD(reg, val));

	/* Wait for the transmission to complete */
	do {
		temp = in_be16(&psc_reg->sr_csr.status);
	} while ((temp & CMD_SEND) && timeout--);

	if (timeout <= 0) {
		printk(KERN_ERR "Err: timeout on slot 1 TX busy\n");
		temp = ~0;
		goto out;
	}

	/*
	 * Give the AC'97 codec more than enough time
	 * to respond. (42us = ~2 frames at 48kHz.)
	 */
	udelay(42);

	/* Wait for data */
	timeout = 80000;
	do {
		cond_resched();
		temp = in_be16(&psc_reg->sr_csr.status);
	} while (!(temp & DATA_VALID) && timeout--);

	if (timeout <= 0 && !(temp & DATA_VALID)) {
		printk(KERN_ERR "Err: timeout on RX valid\n");
		temp = ~0;
		goto out;
	}

	do {
		temp = in_be32(&psc_reg->ac97data);
		r = (temp >> 24) & 0x7f;
		v = (temp >> 8) & 0xffff;
		if (r == reg) {
			temp = v;
			break;
		} else if (--retries) {
			printk(KERN_ERR "ac97 read back fail.  retry\n");
			printk(KERN_ERR "%s, ac97data = 0x%08x\n",
				__func__, temp);
			continue;
		} else {
			printk(KERN_ERR "wrong ac97 register read back "
				"(%x != %x)\n",
			       r, reg);
			temp = ~0;
		}
	} while (retries);

out:
	mutex_unlock(&car_mutex);
	return temp;
}

void mpc5121_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
			unsigned short val)
{
	struct mpc52xx_psc *psc_reg;
	int timeout = 5000;
	u32 temp;

	/* For mysterious cause, this delay time is needed to right
	 * access of codec's register
	 */
	udelay(40);
	mutex_lock(&car_mutex);

	psc_reg = psc_reg_priv;

	/* Write READ register command in slot0 and 1 */
	psc_reg->ac97cmd = WR_CMD(reg, val);

	/*
	 * Wait for the transmission of both slots to complete.
	 */
	do {
		temp = psc_reg->sr_csr.status;
	} while ((temp & CMD_SEND) && timeout--);

	if (!timeout)
		printk(KERN_ERR "timeout waiting for write to complete\n");

	mutex_unlock(&car_mutex);
}

void mpc5121_ac97_reset(struct snd_ac97 *ac97)
{
	/* Initialize necessary registers of codec */
	mpc5121_ac97_write(ac97, 0, 0x0000);

	/* master channels: No attenuation */
	mpc5121_ac97_write(ac97, 2, 0x1f1f);
	mpc5121_ac97_write(ac97, 0x18, 0x0000);
	mpc5121_ac97_write(ac97, 0x2a, 0x01); /* unlock VAR */
	mpc5121_ac97_write(ac97, 0x2c, 0xbb80); /* 48KH */

	mpc5121_ac97_write(ac97, 0x1c, 0x0); /*R/L 22.5dB gain*/
	mpc5121_ac97_write(ac97, 0x1e, 0x0); /*R/L 22.5dB gain2*/
	mpc5121_ac97_write(ac97, 0x1a, 0x0); /* Capture MIC */
}
EXPORT_SYMBOL_GPL(mpc5121_ac97_reset);

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read = mpc5121_ac97_read,
	.write = mpc5121_ac97_write,
};
EXPORT_SYMBOL_GPL(soc_ac97_ops);
