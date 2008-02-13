/*
 * Copyright 2007,2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * based on mxc_nd.c, Copyright 2004-2007 Freescaale Semiconductor, Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/delay.h>

#include <asm/of_device.h>
#include <asm/of_platform.h>

#include "fslnand.h"

#define DRV_NAME "fslnfc"
#define DRV_VERSION "0.1"

/*
 * Starting with silicon rev 2.0 the the NFC_BIG bit is forced to zero
 * in the controller.  FORCELITTLE is set to run little endian on both
 * old and new silicon.
 */
#define FORCELITTLE

/*
 * Define delays in microsec for NAND device operations
 */
#define TROP_US_DELAY   2000
/*
 * Macros to get byte and bit positions of ECC
 */
#define COLPOS(x) ((x) >> 3)
#define BITPOS(x) ((x) & 0x7)

/* Define single bit Error positions in Main & Spare area */
#define MAIN_SINGLEBIT_ERROR 0x4
#define SPARE_SINGLEBIT_ERROR 0x1

static struct nfc_priv {
	int irq;
	struct device *dev;
	unsigned long phys_regs;
	void __iomem *regs;
	void __iomem *nandcsreg;
	int chips;
	struct clk *nfc_clk;
	struct mtd_info mtd;
	struct nand_chip nand;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *parts;
	int nr_parts;
#endif
	int suspended;
} *priv;


static struct of_device_id nfc_match[] = {
	{
		.compatible = "fsl,mpc5121-nfc",
	},
	{},
};

struct nand_info {
	bool bSpareOnly;
	bool bStatusRequest;
	u16 colAddr;
};

static struct nand_info g_nandfc_info;

#ifdef CONFIG_MTD_NAND_FSL_SWECC
static int hardware_ecc = 0;
#else
static int hardware_ecc = 1;
#endif

static int Ecc_disabled;

/*
 * OOB placement block for use with hardware ecc generation
 */
static struct nand_ecclayout nand_hw_eccoob_8 = {
	.eccbytes = 5,
	.eccpos = {6, 7, 8, 9, 10},
	.oobfree = {{0, 5}, {11, 5}}
};

static struct nand_ecclayout nand_hw_eccoob_16 = {
	.eccbytes = 5,
	.eccpos = {6, 7, 8, 9, 10},
	.oobfree = {{0, 6}, {12, 4}}
};

static int is_2kpage = 0;

static void nfc_cleanup(void);

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static wait_queue_head_t irq_waitq;

static irqreturn_t nfc_irq(int irq, void *dev_id)
{
	NFC_CONFIG1 |= NFC_INT_MSK;	/* disable interrupt */
	wake_up(&irq_waitq);
	return IRQ_HANDLED;
}

/*!
 * This function polls the NANDFC to wait for the basic operation to complete by
 * checking the INT bit of config2 register.
 *
 * @param       maxRetries     number of retry attempts (separated by 1 us)
 * @param       param           parameter for debug
 * @param       useirq         True if IRQ should be used rather than polling
 */
static void wait_op_done(int maxRetries, u16 param, bool useirq)
{
	if (useirq) {
		if ((NFC_CONFIG2 & NFC_INT) == 0) {
			NFC_CONFIG1 &= ~NFC_INT_MSK;	/* Enable interrupt */
			iosync();
			wait_event(irq_waitq, NFC_CONFIG2 & NFC_INT);
			NFC_CONFIG2 &= ~NFC_INT;
			iosync();
		}
	} else {
		while (maxRetries-- > 0) {
			if (NFC_CONFIG2 & NFC_INT) {
				NFC_CONFIG2 &= ~NFC_INT;
				iosync();
				break;
			}
			udelay(1);
		}
		if (maxRetries <= 0)
			DEBUG(MTD_DEBUG_LEVEL0, "%s(0x%02x): INT not set\n",
			      __FUNCTION__, param);
	}
}

/*!
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param       cmd     command for NAND Flash
 * @param       useirq  True if IRQ should be used rather than polling
 */
static void send_cmd(u16 cmd, bool useirq)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_cmd(0x%x, %d)\n", cmd, useirq);

	NFC_FLASH_CMD = (u16)cmd;
	iosync();
	NFC_CONFIG2 = NFC_CMD;
	iosync();

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, cmd, useirq);
}

/*!
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param       addr    address to be written to NFC.
 * @param       islast  True if this is the last address cycle for command
 */
static void send_addr(u16 addr, bool islast)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_addr(0x%02x %d)\n", addr, islast);

	NFC_FLASH_ADDR = addr;
	iosync();
	NFC_CONFIG2 = NFC_ADDR;
	iosync();

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, addr, islast);
}

/*!
 * This function requests the NANDFC to initate the transfer
 * of data currently in the NANDFC RAM buffer to the NAND device.
 *
 * @param	buf_id	      Specify Internal RAM Buffer number (0-3)
 * @param       bSpareOnly    set true if only the spare area is transferred
 */
static void send_prog_page(u8 buf_id, bool bSpareOnly)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_prog_page (buf=%d, spare=%d)\n", buf_id, bSpareOnly);

	/* NANDFC buffer 0 is used for page read/write */

	NFC_BUF_ADDR = buf_id;
	iosync();

	/* Configure spare or page+spare access */
	if (!is_2kpage) {
		if (bSpareOnly) {
			NFC_CONFIG1 |= NFC_SP_EN;
			iosync();
		} else {
			NFC_CONFIG1 &= ~(NFC_SP_EN);
			iosync();
		}
	}
	NFC_CONFIG2 = NFC_INPUT;
	iosync();

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, bSpareOnly, true);
}

/*!
 * This function will correct the single bit ECC error
 *
 * @param  buf_id	Specify Internal RAM Buffer number (0-3)
 * @param  eccpos 	Ecc byte and bit position
 * @param  bSpareOnly  	set to true if only spare area needs correction
 */

static void fsl_nand_correct_error(u8 buf_id, u16 eccpos, bool bSpareOnly)
{
	u16 col;
	u8 pos;
	volatile u8 *p;

	col = COLPOS(eccpos);	/* Get byte position */
	pos = BITPOS(eccpos);	/* Get bit position */

	DEBUG(MTD_DEBUG_LEVEL3,
	      "fsl_nand_correct_error (col=%d pos=%d)\n", col, pos);

	/* Set the pointer for main / spare area */
	if (!bSpareOnly)
		p = (volatile u8 *)MAIN_AREA0 + col + (512 * buf_id);
	else
		p = (volatile u8 *)SPARE_AREA0 + col + (16 * buf_id);
	/* Fix the data */
	*p ^= (1 << pos);
}

/*!
 * This function will maintains state of single bit Error
 * in Main & spare  area
 *
 * @param buf_id	Specify Internal RAM Buffer number (0-3)
 * @param spare  	set to true if only spare area needs correction
 */
static void fsl_nand_correct_ecc(u8 buf_id, bool spare)
{
	u16 value, ecc_status;
	/* Read the ECC result */
	ecc_status = NFC_ECC_STATUS_RESULT;
	DEBUG(MTD_DEBUG_LEVEL3,
	      "fsl_nand_correct_ecc (Ecc status=%x)\n", ecc_status);

	if (((ecc_status & 0xC) == MAIN_SINGLEBIT_ERROR)
	    || ((ecc_status & 0x3) == SPARE_SINGLEBIT_ERROR)) {
		if (Ecc_disabled) {
			if ((ecc_status & 0xC) == MAIN_SINGLEBIT_ERROR) {
				value = NFC_RSLTMAIN_AREA;
				/* Correct single bit error in Mainarea
				   NFC will not correct the error in
				   current page */
				fsl_nand_correct_error(buf_id, value, false);
			}
			if ((ecc_status & 0x3) == SPARE_SINGLEBIT_ERROR) {
				value = NFC_RSLTSPARE_AREA;
				/* Correct single bit error in Mainarea
				   NFC will not correct the error in
				   current page */
				fsl_nand_correct_error(buf_id, value, true);
			}

		} else {
			/* Disable ECC  */
			NFC_CONFIG1 &= ~(NFC_ECC_EN);
			iosync();
			Ecc_disabled = 1;
		}
	} else if (ecc_status == 0) {
		if (Ecc_disabled) {
			/* Enable ECC */
			NFC_CONFIG1 |= NFC_ECC_EN;
			iosync();
			Ecc_disabled = 0;
		}
	} else {
		/* 2-bit Error  Do nothing */
	}

}

/*!
 * This function requests the NANDFC to initated the transfer
 * of data from the NAND device into in the NANDFC ram buffer.
 *
 * @param  	buf_id		Specify Internal RAM Buffer number (0-3)
 * @param       bSpareOnly    	set true if only the spare area is transferred
 */
static void send_read_page(u8 buf_id, bool bSpareOnly)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_read_page (%d, %d)\n", buf_id, bSpareOnly);

	/* NANDFC buffer 0 is used for page read/write */
	NFC_BUF_ADDR = buf_id;
	iosync();

	/* Configure spare or page+spare access */
	if (!is_2kpage) {
		if (bSpareOnly) {
			NFC_CONFIG1 |= NFC_SP_EN;
			iosync();
		} else {
			NFC_CONFIG1 &= ~(NFC_SP_EN);
			iosync();
		}
	}

	NFC_CONFIG2 = NFC_OUTPUT;
	iosync();

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, bSpareOnly, true);

	/* If there are single bit errors in
	   two consecutive page reads then
	   the error is not  corrected by the
	   NFC for the second page.
	   Correct single bit error in driver */

	fsl_nand_correct_ecc(buf_id, bSpareOnly);
}

/*!
 * This function requests the NANDFC to perform a read of the
 * NAND device ID.
 */
static void send_read_id(void)
{
	struct nand_chip *this = &priv->nand;

	/* NANDFC buffer 0 is used for device ID output */

	NFC_BUF_ADDR = 0x0;
	iosync();

	/* Read ID into main buffer */
	NFC_CONFIG1 &= (~(NFC_SP_EN));
	iosync();
	NFC_CONFIG2 = NFC_ID;
	iosync();

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, 0, true);

	if (this->options & NAND_BUSWIDTH_16) {
		volatile u16 *mainBuf = MAIN_AREA0;

		/*
		 * Pack the every-other-byte result for 16-bit ID reads
		 * into every-byte as the generic code expects and various
		 * chips implement.
		 */
		/* this may only work for little endian machines */
		mainBuf[0] = (mainBuf[0] & 0xff) | ((mainBuf[1] & 0xff) << 8);
		mainBuf[1] = (mainBuf[2] & 0xff) | ((mainBuf[3] & 0xff) << 8);
		mainBuf[2] = (mainBuf[4] & 0xff) | ((mainBuf[5] & 0xff) << 8);
	}
}

/*!
 * This function requests the NANDFC to perform a read of the
 * NAND device status and returns the current status.
 *
 * @return  device status
 */
static u16 get_dev_status(void)
{
	volatile u8 *mainBuf = (volatile u8 *)MAIN_AREA1;
	u32 store;
	u16 ret;
	/* Issue status request to NAND device */

	/* store the main area1 first word, later do recovery */
	store = *((u32 *)mainBuf);
	/*
	 * NANDFC buffer 1 is used for device status to prevent
	 * corruption of read/write buffer on status requests.
	 */
	NFC_BUF_ADDR = 1;
	iosync();

	/* Read status into main buffer */
	NFC_CONFIG1 &= (~(NFC_SP_EN));
	iosync();
	NFC_CONFIG2 = NFC_STATUS;
	iosync();

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, 0, true);

	/* Status is placed in first word of main buffer */
	/* get status, then recover area 1 data */
	if (NFC_CONFIG1 & NFC_BIG) {
		ret = mainBuf[0];
	} else {
		ret = mainBuf[3];
	}

	*((u32 *)mainBuf) = store;

	return ret;
}

/*!
 * This functions is used by upper layer to checks if device is ready
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return  0 if device is busy else 1
 */
static int fsl_nand_dev_ready(struct mtd_info *mtd)
{
	/*
	 * NFC handles R/B internally.Therefore,this function
	 * always returns status as ready.
	 */
	return 1;
}

static void fsl_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	/*
	 * If HW ECC is enabled, we turn it on during init.  There is
	 * no need to enable again here.
	 */
}

static int fsl_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	/*
	 * 1-Bit errors are automatically corrected in HW.  No need for
	 * additional correction.  2-Bit errors cannot be corrected by
	 * HW ECC, so we need to return failure
	 */
	u16 ecc_status = NFC_ECC_STATUS_RESULT;

	if (((ecc_status & 0x3) == 2) || ((ecc_status >> 2) == 2)) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "FSL_NAND: HWECC uncorrectable 2-bit ECC error\n");
		return -1;
	}

	return 0;
}

static int fsl_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				  u_char *ecc_code)
{
	/*
	 * Just return success.  HW ECC does not read/write the NFC spare
	 * buffer.  Only the FLASH spare area contains the calcuated ECC.
	 */
	return 0;
}

/*!
 * This function reads byte from the NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static u_char fsl_nand_read_byte(struct mtd_info *mtd)
{
	u_char retVal = 0;
	u16 col;
	volatile u16 *mainBuf = MAIN_AREA0;
	volatile u16 *spareBuf = SPARE_AREA0;

	/* Check for status request */
	if (g_nandfc_info.bStatusRequest) {
		return (get_dev_status() & 0xFF);
	}

	col = g_nandfc_info.colAddr;
	/*
	 * swap the lower three bits when running
	 * little endian so the id will come out
	 * as expected
	 */
	if (!(NFC_CONFIG1 & NFC_BIG))
	    col ^= 0x3;

	if (g_nandfc_info.bSpareOnly) {
		retVal = ((volatile u8 *)spareBuf)[col];
	} else {
		retVal = ((volatile u8 *)mainBuf)[col];
	}

	/* Update saved column address */
	g_nandfc_info.colAddr++;

	return retVal;
}

/*!
  * This function reads word from the NAND Flash
  *
  * @param       mtd     MTD structure for the NAND Flash
  *
  * @return    data read from the NAND Flash
  */
static u16 fsl_nand_read_word(struct mtd_info *mtd)
{
	u16 col;
	u16 rdWord, retVal;
	volatile u16 *p;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "fsl_nand_read_word(col = %d)\n", g_nandfc_info.colAddr);

	col = g_nandfc_info.colAddr;
	/* Adjust saved column address */
	if (col < mtd->writesize && g_nandfc_info.bSpareOnly)
		col += mtd->writesize;

	if (col < mtd->writesize)
		p = (MAIN_AREA0) + (col >> 1);
	else
		p = (SPARE_AREA0) + ((col - mtd->writesize) >> 1);

	/* It appears read_word should be native endian, so the problem is just alignment */
	if (col & 0x1) {
		rdWord = *p;
		retVal = (rdWord & 0xff) << 8;
		rdWord = *(p + 1);
		retVal |= (rdWord >> 8) & 0xff;
	}
	else {
		retVal = *p;
	}

	/* Update saved column address */
	g_nandfc_info.colAddr = col + 2;

	return retVal;
}

/*!
 * This function writes data of length len to buffer buf. The data to be
 * written on NAND Flash is first copied to RAMbuffer. After the Data Input
 * Operation by the NFC, the data is written to NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be written to NAND Flash
 * @param       len     number of bytes to be written
 */
static void fsl_nand_write_buf(struct mtd_info *mtd,
			       const u_char *buf, int len)
{
	int n;
	int col;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "fsl_nand_write_buf(col = %d, len = %d)\n", g_nandfc_info.colAddr,
	      len);

	col = g_nandfc_info.colAddr;

	/* Adjust saved column address */
	if (col < mtd->writesize && g_nandfc_info.bSpareOnly)
		col += mtd->writesize;

	n = mtd->writesize + mtd->oobsize - col;
	n = min(len, n);

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s:%d: col = %d, n = %d\n", __FUNCTION__, __LINE__, col, n);
	memcpy((u8 *)(MAIN_AREA0) + col, buf, n);
	col += n;

	/* Update saved column address */
	g_nandfc_info.colAddr = col;
}

/*!
 * This function id is used to read the data buffer from the NAND Flash. To
 * read the data from NAND Flash first the data output cycle is initiated by
 * the NFC, which copies the data to RAMbuffer. This data of length len is
 * then copied to buffer buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be read from NAND Flash
 * @param       len     number of bytes to be read
 */

static void fsl_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{

	int n;
	int col;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "fsl_nand_read_buf(col = %d, len = %d)\n", g_nandfc_info.colAddr,
	      len);

	col = g_nandfc_info.colAddr;
	/* Adjust saved column address */
	if (col < mtd->writesize && g_nandfc_info.bSpareOnly)
		col += mtd->writesize;

	n = mtd->writesize + mtd->oobsize - col;
	n = min(len, n);

	memcpy(buf, (u8 *)(MAIN_AREA0) + col, n);
	col += n;

	/* Update saved column address */
	g_nandfc_info.colAddr = col;

}

/*!
 * This function is used by the upper layer to verify the data in NAND Flash
 * with the data in the buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be verified
 * @param       len     length of the data to be verified
 *
 * @return      -EFAULT if error else 0
 *
 */
static int
fsl_nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	return -EFAULT;
}

static void nfc_chipselect_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,mpc5121ads-cpld");
	priv->nandcsreg = of_iomap(np, 0) + 9;
	of_node_put(np);
}

static void nfc_cs_enable(int chip)
{
	u8 v;

	v = in_8(priv->nandcsreg);
	v |= 0xf;
	v &= ~(1<<chip);

	out_8(priv->nandcsreg, v);
}

/*!
 * This function is used by upper layer for select and deselect of the NAND
 * chip
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       chip    val indicating select or deselect
 */
static void nfc_hw_init(void);
static void fsl_nand_select_chip(struct mtd_info *mtd, int chip)
{
	BUG_ON(priv->suspended);
	if (chip >= priv->chips) {
		printk(KERN_ERR DRV_NAME ": "
			"ERROR: Illegal chip select (chip = %d)\n", chip);
		return;
	}

	if (chip < 0) {
		NFC_CONFIG1 &= (~(NFC_CE));
		iosync();
		return;
	}

	NFC_CONFIG1 |= NFC_CE;
#ifdef FORCELITTLE
	NFC_CONFIG1 &= ~(NFC_BIG);
#endif
	iosync();

	/* Turn on appropriate chip */
	nfc_cs_enable(chip);
}

/*!
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param       mtd             MTD structure for the NAND Flash
 * @param       command         command for NAND Flash
 * @param       column          column offset for the page read
 * @param       page_addr       page to be read from NAND Flash
 */
static void fsl_nand_command(struct mtd_info *mtd, unsigned command,
			     int column, int page_addr)
{
	bool useirq = true;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "fsl_nand_command (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
	      command, column, page_addr);

	/*
	 * Reset command state information
	 */
	g_nandfc_info.bStatusRequest = false;

	/*
	 * Command pre-processing step
	 */
	switch (command) {

	case NAND_CMD_STATUS:
		g_nandfc_info.colAddr = 0;
		g_nandfc_info.bStatusRequest = true;
		break;

	case NAND_CMD_READ0:
		g_nandfc_info.colAddr = column;
		g_nandfc_info.bSpareOnly = false;
		useirq = false;
		break;

	case NAND_CMD_READOOB:
		g_nandfc_info.colAddr = column;
		g_nandfc_info.bSpareOnly = true;
		useirq = false;
		if (is_2kpage)
			command = NAND_CMD_READ0;	/* only READ0 is valid */
		break;

	case NAND_CMD_SEQIN:
		if (column >= mtd->writesize) {
			if (is_2kpage) {
				/**
				  * FIXME: before send SEQIN command for write OOB,
				  * We must read one page out.
				  * For K9F1GXX has no READ1 command to set current HW
				  * pointer to spare area, we must write the whole page including OOB together.
				  */
				/* call itself to read a page */
				fsl_nand_command(mtd, NAND_CMD_READ0, 0,
						 page_addr);
			}
			g_nandfc_info.colAddr = column - mtd->writesize;
			g_nandfc_info.bSpareOnly = true;
			/* Set program pointer to spare region */
			if (!is_2kpage)
				send_cmd(NAND_CMD_READOOB, false);
		} else {
			g_nandfc_info.bSpareOnly = false;
			g_nandfc_info.colAddr = column;
			/* Set program pointer to page start */
			if (!is_2kpage)
				send_cmd(NAND_CMD_READ0, false);
		}
		useirq = false;
		break;

	case NAND_CMD_PAGEPROG:
		if (Ecc_disabled) {
			/* Enable Ecc for page writes */
			NFC_CONFIG1 |= NFC_ECC_EN;
			iosync();
		}

		send_prog_page(0, g_nandfc_info.bSpareOnly);

		if (is_2kpage) {
			/* data in 4 areas datas */
			send_prog_page(1, g_nandfc_info.bSpareOnly);
			send_prog_page(2, g_nandfc_info.bSpareOnly);
			send_prog_page(3, g_nandfc_info.bSpareOnly);
		}

		break;

	case NAND_CMD_ERASE1:
		useirq = false;
		break;
	}

	/*
	 * Write out the command to the device.
	 */
	send_cmd(command, useirq);

	/*
	 * Write out column address, if necessary
	 */
	if (column != -1) {
		/*
		 * FSL_NAND can only perform full page+spare or
		 * spare-only read/write.  When the upper layers
		 * layers perform a read/write buf operation,
		 * we will used the saved column adress to index into
		 * the full page.
		 */
		send_addr(0, page_addr == -1);
		if (is_2kpage)
			send_addr(0, false);	/* another col addr cycle for 2k page */
	}

	/*
	 * Write out page address, if necessary
	 */
	if (page_addr != -1) {
		send_addr((page_addr & 0xff), false);	/* paddr_0 - p_addr_7 */

		if (is_2kpage) {
			send_addr((page_addr >> 8) & 0xff, false);
			send_addr((page_addr >> 16) & 0xff, true);
		} else {
			/* One more address cycle for higher density devices */
			if (mtd->size >= 0x4000000) {
				send_addr((page_addr >> 8) & 0xff, false);	/* paddr_8 - paddr_15 */
				send_addr((page_addr >> 16) & 0xff, true);
			} else
				send_addr((page_addr >> 8) & 0xff, true);	/* paddr_8 - paddr_15 */
		}
	}

	/*
	 * Command post-processing step
	 */
	switch (command) {

	case NAND_CMD_RESET:
		break;

	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (is_2kpage) {
			/* send read confirm command */
			send_cmd(NAND_CMD_READSTART, true);
			/* read for each AREA */
			send_read_page(0, g_nandfc_info.bSpareOnly);
			send_read_page(1, g_nandfc_info.bSpareOnly);
			send_read_page(2, g_nandfc_info.bSpareOnly);
			send_read_page(3, g_nandfc_info.bSpareOnly);
		} else {
			send_read_page(0, g_nandfc_info.bSpareOnly);
		}
		break;

	case NAND_CMD_READID:
		send_read_id();
		break;

	case NAND_CMD_PAGEPROG:
		if (Ecc_disabled) {
			/* Disble Ecc after page writes */
			NFC_CONFIG1 &= ~(NFC_ECC_EN);
			iosync();
		}
		break;

	case NAND_CMD_STATUS:
		break;

	case NAND_CMD_ERASE2:
		break;
	}
}

/* Define some generic bad / good block scan pattern which are used
 * while scanning a device for factory marked good / bad blocks.
 */
static uint8_t scan_ff_pattern[] = { 0xff, 0xff };

static struct nand_bbt_descr smallpage_memorybased = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs = 5,
	.len = 1,
	.pattern = scan_ff_pattern
};

static struct nand_bbt_descr largepage_memorybased = {
	.options = 0,
	.offs = 0,
	.len = 2,
	.pattern = scan_ff_pattern
};

static int fsl_nand_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;

	/* Config before scanning */
	/* Do not rely on NFMS_BIT, set/clear NFMS bit based on mtd->writesize */
	if (mtd->writesize == 2048)
		is_2kpage = 1;
	else
		is_2kpage = 0;

	this->bbt_td = NULL;
	this->bbt_md = NULL;

	if (!this->badblock_pattern) {
		if (mtd->writesize == 2048)
			this->badblock_pattern = &smallpage_memorybased;
		else
			this->badblock_pattern = (mtd->writesize > 512) ?
			    &largepage_memorybased : &smallpage_memorybased;
	}
	/* Build bad block table */
	return nand_scan_bbt(mtd, this->badblock_pattern);
}

#ifdef CONFIG_MTD_PARTITIONS
static int parse_flash_partitions(struct device_node *dp)
{
	int i;
	const  char *name;
	struct device_node *pp;
	int nr_parts = 0;

	for (pp = dp->child; pp; pp = pp->sibling)
		nr_parts++;

	if (!nr_parts)
		return 0;

	priv->parts = devm_kzalloc(priv->dev,
			nr_parts * sizeof(struct mtd_partition),
			GFP_KERNEL);
	if (!priv->parts)
		return -ENOMEM;
	for (pp = dp->child, i =0; pp; pp = pp->sibling, i++) {
		const u32 *reg;
		int len;

		reg = of_get_property(pp, "reg", &len);
		if (!reg || (len != 2*sizeof(u32))) {
			printk(KERN_ERR DRV_NAME ": "
				"Invalid 'reg' on %s\n", dp->full_name);
			kfree(priv->parts);
			priv->parts = NULL;
			return -EINVAL;
		}
		priv->parts[i].offset = reg[0];
		priv->parts[i].size = reg[1];

		name = of_get_property(pp, "label", &len);
		if (!name)
			name = of_get_property(pp, "name", &len);
		priv->parts[i].name = (char *)name;

		if (of_get_property(pp, "read-only", &len))
			priv->parts[i].mask_flags = MTD_WRITEABLE;
	}
	return nr_parts;
}
#endif

static void nfc_hw_init(void)
{
	clk_enable(priv->nfc_clk);

	if (hardware_ecc)
		NFC_CONFIG1 |= NFC_ECC_EN;
	NFC_CONFIG1 |= NFC_INT_MSK;
#ifdef FORCELITTLE
	NFC_CONFIG1 &= ~(NFC_BIG);
#endif
	iosync();

	/* Reset NAND */
	priv->nand.cmdfunc(&priv->mtd, NAND_CMD_RESET, -1, -1);

	/* preset operation */
	/* Unlock the internal RAM Buffer */
	NFC_CONFIG = 0x2;
	iosync();

	/* Blocks to be unlocked */
	NFC_UNLOCKSTART_BLKADDR = 0x0;
	iosync();
	NFC_UNLOCKEND_BLKADDR = 0xffff;
	iosync();

	/* Unlock Block Command for given address range */
	NFC_WRPROT = 0x4;
	iosync();
}

static int nfc_probe(struct of_device *op,
		const struct of_device_id *match)
{
	struct device_node *dn = op->node;
	struct device *dev = &op->dev;
	int rv = 0;
	struct resource res;
	size_t size_regs;
	struct nand_chip *this;
	struct mtd_info *mtd;
	int const *width;
	const u32 *chips;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ERR DRV_NAME ": "
				"Error allocating priv structure\n");
		return -ENOMEM;
	}

	memset((char *)&g_nandfc_info, 0, sizeof(g_nandfc_info));

	this = &priv->nand;
	mtd = &priv->mtd;
	mtd->priv = this;
	mtd->name = "NAND";
	mtd->owner = THIS_MODULE;

	/* 50 us command delay time */
	this->chip_delay = 5;

	this->dev_ready = fsl_nand_dev_ready;
	this->cmdfunc = fsl_nand_command;
	this->select_chip = fsl_nand_select_chip;
	this->read_byte = fsl_nand_read_byte;
	this->read_word = fsl_nand_read_word;
	this->write_buf = fsl_nand_write_buf;
	this->read_buf = fsl_nand_read_buf;
	this->verify_buf = fsl_nand_verify_buf;
	this->scan_bbt = fsl_nand_scan_bbt;

	priv->irq = NO_IRQ;
	priv->dev = dev;

	rv = of_address_to_resource(dn, 0, &res);
	if (rv) {
		printk(KERN_ERR DRV_NAME ": "
		       "Error parsing memory resource\n");
		return rv;
	}

	priv->irq = irq_of_parse_and_map(dn, 0);
	if (priv->irq == NO_IRQ) {
		printk(KERN_ERR DRV_NAME ": "
				"Error mapping irq\n");
		return -EINVAL;
	}
	priv->phys_regs = res.start;
	size_regs = res.end - res.start + 1;

	if (!devm_request_mem_region(dev, priv->phys_regs,
				size_regs, DRV_NAME)) {
		printk(KERN_ERR DRV_NAME ": "
				"Error requesting register memory region\n");
		rv = -EBUSY;
		goto err;
	}

	priv->regs = devm_ioremap(dev, priv->phys_regs, size_regs);
	if (!priv->regs) {
		printk(KERN_ERR DRV_NAME ": "
				"Error mapping registers\n");
		rv = -ENOMEM;
		goto err;
	}

	nfc_chipselect_init();

	chips = of_get_property(dn, "chips", NULL);
	if (chips) {
		priv->chips = *chips;
	} else {
		printk(KERN_INFO DRV_NAME ": "
				"no chips property defaulting to 1\n");
		priv->chips = 1;
	}

	priv->nfc_clk = clk_get(dev, "nfc_clk");

	init_waitqueue_head(&irq_waitq);

	if (devm_request_irq(dev, priv->irq, &nfc_irq, 0, DRV_NAME, priv)) {
		printk(KERN_ERR DRV_NAME ": "
				"Error requesting irq\n");
		rv = -EINVAL;
		goto err;
	}

	if (hardware_ecc) {
		this->ecc.calculate = fsl_nand_calculate_ecc;
		this->ecc.hwctl = fsl_nand_enable_hwecc;
		this->ecc.correct = fsl_nand_correct_data;
		this->ecc.mode = NAND_ECC_HW;
		this->ecc.size = 512;
		this->ecc.bytes = 3;
		this->ecc.layout = &nand_hw_eccoob_8;
		iosync();
	} else {
		this->ecc.mode = NAND_ECC_SOFT;
	}


	nfc_hw_init();

	/* NAND bus width determines access funtions used by upper layer */
	width = of_get_property(dn, "bank_width", NULL);
	if (width && *width == 2) {
		this->options |= NAND_BUSWIDTH_16;
		this->ecc.layout = &nand_hw_eccoob_16;
	} else {
		this->options |= 0;
	}

	is_2kpage = 0;

	/* Scan to find existence of the device */
	if (nand_scan(mtd, priv->chips)) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "FSL_NAND: Unable to find any NAND device.\n");
		rv = -ENXIO;
		goto err;
	}

	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
	priv->nr_parts =
	    parse_mtd_partitions(mtd, part_probes, &priv->parts, 0);
	if (priv->nr_parts > 0)
		add_mtd_partitions(mtd, priv->parts, priv->nr_parts);
	else if ((priv->nr_parts = parse_flash_partitions(dn)) > 0) {
		dev_info(dev, "Using OF partition info\n");
		add_mtd_partitions(mtd, priv->parts, priv->nr_parts);
	}
	else
#endif
	{
		pr_info("Registering %s as whole device\n", mtd->name);
		add_mtd_device(mtd);
	}
	dev_set_drvdata(dev, mtd);

	return rv;

err:
	nfc_cleanup();
	return rv;
}

static void nfc_cleanup(void)
{
	if (!priv) {
		return;
	}
	if (priv->irq != NO_IRQ) {
		devm_free_irq(priv->dev, priv->irq, priv);
		priv->irq = NO_IRQ;
	}
	if (priv->nfc_clk) {
		clk_disable(priv->nfc_clk);
		clk_put(priv->nfc_clk);
		priv->nfc_clk = NULL;
	}
	if (priv->nandcsreg) {
		iounmap(priv->nandcsreg);
	}
}

static int nfc_remove(struct of_device *op)
{
	nfc_cleanup();
	return 0;
}

#ifdef CONFIG_PM
static int nfc_suspend(struct of_device *op, pm_message_t state)
{
	priv->suspended = 1;
	clk_disable(priv->nfc_clk);
	return 0;
}

static int nfc_resume(struct of_device *op)
{
	priv->suspended = 0;
	nfc_hw_init();
	return 0;
}
#else
#define nfc_suspend NULL
#define nfc_resume NULL
#endif

static struct of_platform_driver nfc_driver = {
	.owner		= THIS_MODULE,
	.name		= DRV_NAME,
	.match_table	= nfc_match,
	.probe		= nfc_probe,
	.remove		= nfc_remove,
	.suspend	= nfc_suspend,
	.resume		= nfc_resume,
	.driver		= {
		.name	= DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init nfc_init(void)
{
	return of_register_platform_driver(&nfc_driver);
}

static void __exit nfc_exit(void)
{
	of_unregister_platform_driver(&nfc_driver);
}

module_init(nfc_init);
module_exit(nfc_exit);

MODULE_AUTHOR("Ross Wille <wille@freescale.com>");
MODULE_DESCRIPTION("Freescale NAND Flash Controller driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
