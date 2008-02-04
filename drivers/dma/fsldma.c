/*
 * Hongjun Chen <hong-jun.chen@freescale.com>
 * Copyright (C) Freescale Semicondutor, Inc. 2007, 2008. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

/*
 * This driver supports an MPC5121 DMA engine, which does asynchronous
 * copy operations.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <asm/fsldma.h>
#include <asm/fsldma_reg.h>
#include <asm/semaphore.h>

#undef DEBUG

#ifdef DEBUG
#define DPRINTK(fmt, args...) \
    printk(KERN_DEBUG "%s: " fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#endif

static struct fsl_device *g_device;
static struct fsl_dma_chan *g_fchan;

/* internal functions */
static int __devinit fsl_init(void);
static void __devexit fsl_remove(void);
static void fsl_dma_memcpy_cleanup(struct fsl_dma_chan *fsl_chan);

static int alloc_dma_channels(struct fsl_device *device)
{
	struct fsl_dma_chan *fsl_chan;

	device->common.chancnt = FSL_DMA_CH_NUM;
	fsl_chan = kzalloc(sizeof(*fsl_chan) * device->common.chancnt,
			   GFP_KERNEL);
	if (!fsl_chan) {
		printk("Err: there is not enough memory"
		       " for channel structures!");
		return -ENOMEM;
	}
	g_fchan = fsl_chan;

	return device->common.chancnt;
}

/* Called for DMA engine initialization */
int fsl_dma_cfg_arbit_mode(int mode)
{
	fsl_dma_reg *reg;
	int i;
	u32 temp;

	if (g_device) {
		spin_lock_bh(&g_device->ch_lock);
		for (i = 0; i < FSL_DMA_CH_NUM; i++) {
			if (g_device->ch_stat[i]) {
				spin_unlock_bh(&g_device->ch_lock);
				return -EBUSY;
			}
		}
		spin_unlock_bh(&g_device->ch_lock);

		reg = g_device->reg;

		g_device->arbit_mode = mode;

		if (mode & FSL_DMA_GROUP_FIX) {	/* Group fixed arbitration */
			out_be32(&reg->dmacr,
				 FSL_DMA_GPR3PRI(3) | FSL_DMA_GPR2PRI(2)
				 | FSL_DMA_GPR1PRI(1) | FSL_DMA_GPR0PRI(0));
		} else {	/* Group round robin arbitration */
			out_be32(&reg->dmacr, FSL_DMA_DMACR_ERGA_RR);
		}

		/* Channel round robin arbitration */
		if (!(mode & FSL_DMA_CH_FIX)) {
			temp = in_be32(&reg->dmacr);
			out_be32(&reg->dmacr, temp | FSL_DMA_DMACR_ERCA_RR);
		}

		return 0;
	} else
		return -EFAULT;
}

static void free_chan(int channel_num)
{
	spin_lock_bh(&g_device->ch_lock);
	g_device->ch_stat[channel_num] = 0;
	spin_unlock_bh(&g_device->ch_lock);
}

int fsl_dma_status(int channel_num)
{
	fsl_dma_reg *reg;
	u8 ch;
	u32 status;

	if (!g_device) {
		printk("Err: DMA driver has not been initialized yet!\n");
		return -1;
	}

	reg = g_device->reg;

	status = in_be32(&reg->dmaes);
	if (!(status & FSL_DMA_DMAES_VLD))
		return 0;

	ch = FSL_DMA_DMAES_ERRCHN(status);
	printk(KERN_ERR "FSL channel config error: ch%d: ", ch);

	if (status & FSL_DMA_DMAES_GPE)
		printk(KERN_INFO "GPE\n");
	if (status & FSL_DMA_DMAES_CPE)
		printk(KERN_INFO "CPE\n");
	if (status & FSL_DMA_DMAES_SAE)
		printk(KERN_INFO "SAE\n");
	if (status & FSL_DMA_DMAES_SOE)
		printk(KERN_INFO "SOE\n");
	if (status & FSL_DMA_DMAES_DAE)
		printk(KERN_INFO "DAE\n");
	if (status & FSL_DMA_DMAES_DOE)
		printk(KERN_INFO "DOE\n");
	if (status & FSL_DMA_DMAES_NCE)
		printk(KERN_INFO "NCE\n");
	if (status & FSL_DMA_DMAES_SGE)
		printk(KERN_INFO "SGE\n");
	if (status & FSL_DMA_DMAES_SBE)
		printk(KERN_INFO "SBE\n");
	if (status & FSL_DMA_DMAES_DBE)
		printk(KERN_INFO "DBE\n");

	if (ch == channel_num)
		out_8(&reg->dmacerr, channel_num);

	return (ch + 1);
}

/* This function is generally called by the driver at open time.
 * The DMA driver would do any initialization steps that is required
 * to get the channel ready for data transfer.
 *
 * @param channel_id   a pre-defined id. The peripheral driver would specify
 *                     the id associated with its peripheral. This would be
 *                     used by the DMA driver to identify the peripheral
 *                     requesting DMA and do the necessary setup on the
 *                     channel associated with the particular peripheral.
 *                     The DMA driver could use static or dynamic DMA channel
 *                     allocation.
 * @return returns a negative number on error if request for a DMA channel
 *                     did not succeed, returns the channel number to be used
 *                     on success.
 */
int fsl_dma_chan_request(int channel_id)
{
	struct fsl_device *device = g_device;
	struct fsl_dma_chan *fsl_chan;

	if (!device) {
		fsl_init();
		if (!g_device)
			return -EINVAL;
		device = g_device;
	}

	if (channel_id < 0 || channel_id >= FSL_DMA_CH_NUM)
		return -EINVAL;

	fsl_chan = g_fchan + channel_id;
	spin_lock_bh(&device->ch_lock);
	if (device->ch_stat[channel_id] == 0) {
		device->ch_stat[channel_id] = 1;
		spin_unlock_bh(&device->ch_lock);
		sema_init(&fsl_chan->sem_lock, 1);
		return channel_id;
	}
	spin_unlock_bh(&device->ch_lock);

	return -EBUSY;
}

void fsl_dma_free_chan(int channel_num)
{
	struct fsl_dma_chan *fsl_chan = g_fchan + channel_num;
	fsl_dma_reg *reg = g_device->reg;

	if (channel_num < 0 || channel_num >= FSL_DMA_CH_NUM)
		return;

	DPRINTK("free channel: %d\n", channel_num);

	free_chan(channel_num);

	fsl_dma_memcpy_cleanup(fsl_chan);

	/* Clear enable request */
	out_8(&reg->dmacerq, channel_num);

	/* Clear interrupt, error interrupt and error etc. */
	out_8(&reg->dmaceei, channel_num);
	out_8(&reg->dmacerr, channel_num);
	out_8(&reg->dmacint, channel_num);
}

void fsl_dma_enable(int channel_num)
{
	fsl_dma_reg *reg = g_device->reg;
	u32 *ltcd, *mtcd;
	int j;

	if (channel_num < 0 || channel_num >= FSL_DMA_CH_NUM)
		return;

	/* Get the address of TCD in local CPU memory space */
	ltcd = (u32 *) (g_device->tcd + channel_num);
	mtcd = (u32 *) (g_device->mtcd.addr_va
			+ channel_num * MAX_TCD_NUM_PER_CH);

	/* Clear the local TCD area */
	memset(ltcd, 0, sizeof(TCD));

	for (j = 0; j < (sizeof(TCD) / 4); j++) {
		out_be32(ltcd, *mtcd);
		ltcd++;
		mtcd++;
	}

	/* Enable request */
	out_8(&reg->dmaserq, channel_num);
	return;
}

void fsl_dma_disable(int channel_num)
{
	fsl_dma_reg *reg = g_device->reg;
	if (channel_num < 0 || channel_num >= FSL_DMA_CH_NUM)
		return;

	/* Clear enable request */
	out_8(&reg->dmacerq, channel_num);

	return;
}

static int off_to_size(size_t off)
{
	int i, temp;

	/* For PSC FIFO DMA operation
	 * Destination/source address is fixed as the address
	 * of FIFO data register, so no address offset is needed
	 * for this operation, bus size is default 32bit.
	 */
	if (!off)
		return 2;

	temp = off;
	for (i = 0; ; i++) {
		temp = (int)(temp / 2);
		if (!temp)
			return i;
	}
}

/**
 * fsl_dma_config - function that initiates a FSL DMA transaction buffers
 * @channel_num	FSL DMA channel number
 * @dma_buf 	an array of physical addresses to the user defined
 *              buffers. The caller must guarantee the dma_buf is
 *              available until the transfer is completed.
 * @num_buf     number of buffers in the array
 * @mode        specifies whether this is READ or WRITE operation
 * @return 	This function returns a negative number on error if buffer
 *              could not be added with DMA for transfer. On Success, it
 *              returns 0
 */
int fsl_dma_config(int channel_num, struct fsl_dma_requestbuf *dma_buf,
		   int num_buf)
{
	struct fsl_dma_chan *fsl_chan = g_fchan + channel_num;
	LIST_HEAD(new_chain);
	fsl_dma_reg *reg = g_device->reg;
	int ch = channel_num, i, nbytes, iter;
	TCD *tcd;
	dma_addr_t tcd_dma;

	if (channel_num < 0 || channel_num >= FSL_DMA_CH_NUM
	    || num_buf <= 0 || num_buf > MAX_TCD_NUM_PER_CH)
		return -EINVAL;

	if (fsl_dma_status(channel_num) != DMA_SUCCESS && fsl_chan->num_buf)
		return -EBUSY;

	if (down_trylock(&fsl_chan->sem_lock))
		return -EBUSY;

	tcd = (TCD *) (g_device->tcd + channel_num);

	/* Use so much descriptor for this transmission */
	fsl_chan->num_buf = num_buf;
	fsl_chan->ch_index = channel_num;

	/* Enable error interrupt */
	out_8(&reg->dmaseei, ch);

	tcd = g_device->mtcd.addr_va + ch * MAX_TCD_NUM_PER_CH;
	tcd_dma = g_device->mtcd.addr_pa +
	    ch * MAX_TCD_NUM_PER_CH * sizeof(TCD);

	memset(tcd, 0, sizeof(TCD));

	for (i = 0; i < num_buf; i++) {
		tcd->saddr = dma_buf[i].src;
		tcd->daddr = dma_buf[i].dest;
		tcd->soff = dma_buf[i].soff;
		tcd->doff = dma_buf[i].doff;
		tcd->ssize = off_to_size(dma_buf[i].soff);
		tcd->dsize = off_to_size(dma_buf[i].doff);
		DPRINTK("doff: %d, soff: %d, dsize: %d, ssize: %d\n",
			tcd->doff, tcd->soff, tcd->dsize, tcd->ssize);

		if (dma_buf[i].soff >= dma_buf[i].doff)
			nbytes = dma_buf[i].minor_loop * dma_buf[i].soff;
		else
			nbytes = dma_buf[i].minor_loop * dma_buf[i].doff;

		tcd->nbytes = nbytes;
		tcd->slast = 0;
		tcd->dlast_sga = 0;

		iter = dma_buf[i].len / nbytes;

		if (tcd->soff && tcd->doff) {
			tcd->citer_elink = 1;
			tcd->biter_elink = 1;
			tcd->citer_linkch = channel_num;
			if (iter > 0x1ff)
				panic("iterations won't fit in field");
			tcd->citer = iter;
			tcd->biter = iter;
			tcd->start = 1;
		} else {
			/* citer_linkch contains the high bits of iter */
			tcd->citer_linkch = iter >> 9;
			tcd->biter_linkch = iter >> 9;
			tcd->citer = iter & 0x1ff;
			tcd->biter = iter & 0x1ff;
			if (i == (num_buf - 1))
				tcd->d_req = 1;
		}

		tcd->e_sg = 0;

		/* For chain mode */
		if (i != (num_buf - 1)) {
			tcd->dlast_sga = tcd_dma + (i + 1) * sizeof(TCD);
			tcd->e_sg = 1;
		} else {
			tcd->int_maj = 1;
		}

		DPRINTK("TCD dlast_sga: 0x%08x\n", tcd->dlast_sga);
		tcd++;
		memset(tcd, 0, sizeof(TCD));
	}
	return 0;
}

/* This function is called only when the channel occurs error
 * or has completed transaction.
 */
static void fsl_dma_memcpy_cleanup(struct fsl_dma_chan *chan)
{
	int ch;

	chan->status = g_device->reg->dmaes;

	ch = fsl_dma_status(chan->ch_index);
	if (ch)
		ch = ch - 1;

	if (ch == chan->ch_index) {
		DPRINTK("Channel %d halted, chanerr = %x\n", ch, chan->status);
	}
}

static void fsl_dma_isr_bh(u32 intst, u32 err, u32 high)
{
	struct fsl_device *device = g_device;
	struct fsl_dma_chan *fsl_chan;
	fsl_dma_reg *reg;
	u32 i, ch_base = 0, ch;
	u32 error;

	reg = device->reg;
	if (high)
		ch_base = 32;

	for (i = 0; i < 32; i++) {
		error = 0;
		if (intst & (1 << i)) {
			if (err & (1 << i))
				error = in_be32(&reg->dmaes);
			ch = i + ch_base;
			fsl_chan = g_fchan + ch;
			DPRINTK("channel %d occurs an interrupt\n", ch);

			/* Clear interrupt request */
			out_8(&reg->dmacint, ch);

			/* Clear enable error interrupt */
			out_8(&reg->dmaceei, ch);

			up(&fsl_chan->sem_lock);
			if (fsl_chan->cb_fn)
				fsl_chan->cb_fn(fsl_chan->cb_args, error);
		}
	}
	return;
}

static irqreturn_t fsl_do_interrupt(int irq, void *data)
{
	struct fsl_device *device = g_device;
	fsl_dma_reg *reg;
	u32 inth, intl, eeih, eeil;
	u32 errh, errl;

	DPRINTK("receive one interrupt!\n");
	if (!device)
	    return NO_IRQ;
	reg = device->reg;
	inth = in_be32(&reg->dmainth);
	intl = in_be32(&reg->dmaintl);
	eeih = in_be32(&reg->dmaeeih);
	eeil = in_be32(&reg->dmaeeil);

	errh = in_be32(&reg->dmaerrh);
	errl = in_be32(&reg->dmaerrl);

	inth &= eeih;
	intl &= eeil;

	DPRINTK("es:0x%08x\n", in_be32(&reg->dmaes));

	/* Clear all interrupt request */
	out_8(&reg->dmacint, 0x7f);

	if (!(inth & eeih) && !(intl & eeil))
		return IRQ_NONE;

	fsl_dma_isr_bh(inth, errh, 1);
	fsl_dma_isr_bh(intl, errl, 0);

	return IRQ_HANDLED;
}

int fsl_dma_callback_set(int channel_num, fsl_dma_callback_t callback,
			 void *arg)
{
	struct fsl_dma_chan *fsl_chan = g_fchan + channel_num;

	if (channel_num < 0 || channel_num >= FSL_DMA_CH_NUM)
		return -EINVAL;

	if (fsl_dma_status(channel_num) == DMA_IN_PROGRESS)
		return -EBUSY;

	fsl_chan->cb_fn = callback;
	fsl_chan->cb_args = arg;

	return 0;
}

/*
 * Perform a FSL transaction to verify the HW works.
 */
#define FSL_TEST_SIZE 0x200

static int fsl_self_test(struct fsl_device *device)
{
	struct fsl_dma_requestbuf buf[3];
	u8 *src0, *dest0;
	u8 *src1, *dest1;
	u8 *src2, *dest2;
	dma_addr_t sphyaddr0, dphyaddr0;
	dma_addr_t sphyaddr1, dphyaddr1;
	dma_addr_t sphyaddr2, dphyaddr2;
	int ch;
	dma_cookie_t cookie;
	int i, err = 0;

	src0 = dma_alloc_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE,
				  &sphyaddr0, GFP_KERNEL);
	src1 = dma_alloc_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE,
				  &sphyaddr1, GFP_KERNEL);
	src2 = dma_alloc_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE,
				  &sphyaddr2, GFP_KERNEL);
	if (!src0 || !src1 || !src2)
		return -ENOMEM;

	dest0 = dma_alloc_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE,
				   &dphyaddr0, GFP_KERNEL);
	dest1 = dma_alloc_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE,
				   &dphyaddr1, GFP_KERNEL);
	dest2 = dma_alloc_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE,
				   &dphyaddr2, GFP_KERNEL);
	if (!dest0 || !dest1 || !dest2) {
		dma_free_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE, src0,
				  sphyaddr0);
		dma_free_coherent(NULL, sizeof(u8) * FSL_TEST_SIZE, src1,
				  sphyaddr1);
		return -ENOMEM;
	}

	memset(dest0, 0, FSL_TEST_SIZE);
	memset(dest1, 0, FSL_TEST_SIZE);
	memset(dest2, 0, FSL_TEST_SIZE);

	/* Fill in src buffer */
	for (i = 0; i < FSL_TEST_SIZE; i++) {
		src0[i] = (u8) i;
		src1[i] = (u8) i;
		src2[i] = (u8) i;
	}

	/* Set arbitration mode chj */
	fsl_dma_cfg_arbit_mode(FSL_DMA_GROUP_FIX | FSL_DMA_CH_FIX);

	/* Search for one available dma channel */
	ch = fsl_dma_chan_request(32);
	if (ch < 0) {
		err = -ENODEV;
		goto out;
	}

	/* Both source and destination port size: 32 bits */
	buf[0].src = sphyaddr0;
	buf[0].dest = dphyaddr0;
	buf[0].soff = 4;
	buf[0].doff = 1;
	buf[0].len = FSL_TEST_SIZE;
	buf[0].minor_loop = 8;	/* Minor bytes: 32 */

	buf[1].src = sphyaddr1;
	buf[1].dest = dphyaddr1;
	buf[1].soff = 4;
	buf[1].doff = 1;
	buf[1].len = FSL_TEST_SIZE;
	buf[1].minor_loop = 8;	/* Minor bytes: 32 */

	buf[2].src = sphyaddr2;
	buf[2].dest = dphyaddr2;
	buf[2].soff = 4;
	buf[2].doff = 1;
	buf[2].len = FSL_TEST_SIZE;
	buf[2].minor_loop = 8;	/* Minor bytes: 32 */

	cookie = fsl_dma_config(ch, &buf[0], 3);
	fsl_dma_enable(ch);
	msleep(100);

	if (fsl_dma_status(ch) != DMA_SUCCESS) {
		printk(KERN_ERR
		       "fsldma: Self-test copy timed out, disabling\n");
		err = -ENODEV;
		goto free_resources;
	}
	if (memcmp(src0, dest0, FSL_TEST_SIZE) ||
	    memcmp(src1, dest1, FSL_TEST_SIZE) ||
	    memcmp(src2, dest2, FSL_TEST_SIZE)) {
		printk(KERN_ERR
		       "fsldma: Self-test copy failed compare, disabling\n");
		err = -ENODEV;
		fsl_dma_status(ch);
		goto free_resources;
	} else {
		printk("fsldma: Self-test copy successfully\n");
	}

free_resources:
	fsl_dma_free_chan(ch);
out:
	dma_free_coherent(NULL, FSL_TEST_SIZE, src0, sphyaddr0);
	dma_free_coherent(NULL, FSL_TEST_SIZE, dest0, dphyaddr0);
	dma_free_coherent(NULL, FSL_TEST_SIZE, src1, sphyaddr1);
	dma_free_coherent(NULL, FSL_TEST_SIZE, dest1, dphyaddr1);
	return err;
}

static int __devinit fsl_init(void)
{
	int err;
	unsigned long base_addr;
	fsl_dma_reg *reg;
	TCD *tcd;
	struct fsl_device *device;
	struct device_node *np;
	struct resource r;
	u32 mask, offset;

	if (g_device)
		return 0;

	/* Map the virtual IRQ number from device tree */
	np = of_find_compatible_node(NULL, NULL, "mpc512x-dma2");
	if (!np) {
		printk(KERN_ERR "Err: no 'mpc512x-dma2' in device tree!\n");
		return -EINVAL;
	}

	/* DMA register space */
	of_address_to_resource(np, 0, &r);
	of_node_put(np);
	DPRINTK("DMA engine register address: 0x%08x\n", r.start);
	reg = (fsl_dma_reg *) ioremap(r.start, sizeof(fsl_dma_reg));
	if (!reg) {
		err = -ENOMEM;
		goto err_regioremap;
	}

	/* DMA transfer control desciptor area */
	base_addr = r.start + FSL_DMA_TCD_OFFSET;
	DPRINTK("Local TCD start address: 0x%08x\n", (u32) base_addr);
	tcd = (TCD *) ioremap(base_addr, sizeof(TCD) * FSL_DMA_CH_NUM);
	if (!tcd) {
		err = -ENOMEM;
		goto err_tcdioremap;
	}

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		err = -ENOMEM;
		goto err_kzalloc;
	}

	device->irq = irq_of_parse_and_map(np, 0);
	of_node_put(np);

	err = request_irq(device->irq, &fsl_do_interrupt,
			  IRQF_SHARED, "mpc5121dma", device);
	if (err)
		goto err_irq;

	spin_lock_init(&device->ch_lock);
	device->reg = reg;
	device->tcd = tcd;
	g_device = device;

	/* Allocate buffer for TCDs in memory */
	device->mtcd.size = (MAX_TCD_NUM_PER_CH * FSL_DMA_CH_NUM + 1)
	    * sizeof(TCD);
	device->mtcd.addr_v =
	    (TCD *) dma_alloc_coherent(NULL, device->mtcd.size,
				       &device->mtcd.addr_p, GFP_KERNEL);
	mask = sizeof(TCD) - 1;
	offset = device->mtcd.addr_p & mask;
	device->mtcd.addr_va = device->mtcd.addr_v;
	device->mtcd.addr_pa = device->mtcd.addr_p;
	if (offset) {
		offset = sizeof(TCD) - offset;
		device->mtcd.addr_pa = device->mtcd.addr_p + offset;
		device->mtcd.addr_va = (TCD *) ((u32) device->mtcd.addr_v
						+ offset);
		DPRINTK("Note: TCD buffer address is re-aligned:"
			" offset: 0x%x\n", offset);
	}
	DPRINTK("tcd buf phy addr: 0x%08x\n", device->mtcd.addr_pa);

	alloc_dma_channels(device);
	printk(KERN_INFO "Freescale(R) MPC5121 DMA Engine found, %d channels\n",
	       FSL_DMA_CH_NUM);

	err = fsl_self_test(device);
	if (err)
		goto err_self_test;

	return 0;

err_irq:
err_kzalloc:
err_self_test:
	kfree(device);
	dma_free_coherent(NULL, device->mtcd.size,
			  device->mtcd.addr_v, device->mtcd.addr_p);
err_tcdioremap:
	iounmap(tcd);
err_regioremap:
	iounmap(reg);
	return err;
}

static void __devexit fsl_remove(void)
{
	struct fsl_device *device;

	if (!g_device)
		return;
	device = g_device;

	free_irq(device->irq, device);
	iounmap(device->reg);
	iounmap(device->tcd);
	dma_free_coherent(NULL, device->mtcd.size,
			  device->mtcd.addr_v, device->mtcd.addr_p);
	kfree(device);
}

static int __init fsl_init_module(void)
{
	return fsl_init();
}

module_init(fsl_init_module);

static void __exit fsl_exit_module(void)
{
	fsl_remove();
}

module_exit(fsl_exit_module);

/* MODULE API */
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor Inc.");
