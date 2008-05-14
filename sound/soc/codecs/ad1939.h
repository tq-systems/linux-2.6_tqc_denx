/*
 * AD1935/AD1936/AD1937/AD1938/AD1939 I2S ASoC Codec driver
 *
 * Copyright (c) 2007 MSC Vertriebsges.m.b.H,
 *	Manuel Lauss <mlau@msc-ge.com> <mano@roarinelk.homelinux.net>
 *
 * licensed under the GPLv2
 *
 */

#ifndef _AD1939_H_
#define _AD1939_H_

#define AD1939_PLLCTL0	0x00
#define AD1939_PLLCTL1	0x01
#define AD1939_DACCTL0	0x02
#define AD1939_DACCTL1	0x03
#define AD1939_DACCTL2	0x04
#define AD1939_DACMUTE	0x05
#define AD1939_VOL1L	0x06
#define AD1939_VOL1R	0x07
#define AD1939_VOL2L	0x08
#define AD1939_VOL2R	0x09
#define AD1939_VOL3L	0x0A
#define AD1939_VOL3R	0x0B
#define AD1939_VOL4L	0x0C
#define AD1939_VOL4R	0x0D
#define AD1939_ADCCTL0	0x0E
#define AD1939_ADCCTL1	0x0F
#define AD1939_ADCCTL2	0x10

#define AD1939_REGCOUNT	0x11

/*
 * AD1939 setup data
 */

/* TDM modes. Have a look at the manual to understand what these do. */
#define AD1939_TDM_MODE_TDM		1
#define AD1939_TDM_MODE_AUX		2
#define AD1939_TDM_MODE_DUALLINE	3

/* Master PLL clock source, select one */
#define AD1939_PLL_SRC_MCLK		0	/* external clock */
#define AD1939_PLL_SRC_DACLRCK		1	/* get from DAC LRCLK */
#define AD1939_PLL_SRC_ADCLRCK		2	/* get from ADC LRCLK */

/* clock sources for ADC, DAC. Refer to the manual for more information
 * (for 192000kHz modes, internal PLL _MUST_ be used). Select one for ADC
 * and DAC.
 */
#define AD1939_CLKSRC_DAC_PLL		0	/* DAC clocked by int. PLL */
#define AD1939_CLKSRC_DAC_MCLK		(1<<0)	/* DAC clocked by ext. MCK */
#define AD1939_CLKSRC_ADC_PLL		0	/* ADC clocked by int. PLL */
#define AD1939_CLKSRC_ADC_MCLK		(1<<1)	/* ADC clocked by ext. MCK */
#define AD1939_CLKSRC_ENABLE_ONCHIP_VREF	(1<<2)

/* I2S Bitclock sources for DAC and ADC I2S interfaces.
 * OR it to ad1939_setup_data.dac_adc_clksrc. Select one for ADC and DAC.
 */
#define AD1939_BCLKSRC_DAC_EXT		0	/* DAC I2SCLK from DBCLK pin */
#define AD1939_BCLKSRC_DAC_PLL		(1<<6)	/* DAC I2SCLK from int. PLL */
#define AD1939_BCLKSRC_ADC_EXT		0	/* DAC I2SCLK from DBCLK pin */
#define AD1939_BCLKSRC_ADC_PLL		(1<<7)	/* DAC I2SCLK from int. PLL */

/* MCLK_XO pin configuration */
#define AD1939_MCLKXO_MCLKXI		0	/* mirror MCLK_XI pin */
#define AD1939_MCLKXO_256FS		1
#define AD1939_MCLKXO_512FS		2
#define AD1939_MCLKXO_OFF		3	/* disable MCLK_XO output */

/* driver specific flags */
/* specify these flags if the LRCK and/or BCK pins of the ADC and DAC
 * parts are wired together on the PCB; to prevent both units from driving
 * the pin and resulting bad signals.  You then have to specify WHICH
 * unit gets to be the Master (clock generator) and which is slave.
 * NOTE: this is only used if the CODEC is configured as either BCK or
 * LRCK master; if the codec is BCK/LRCK slave (BCK and LRCK are driven
 * by external components) these settings are ignored!
 */
#define AD1939_DRV_ADCDAC_COMMON_BCK	(1<<0)
#define AD1939_DRV_ADCDAC_COMMON_LRCK	(1<<1)

/* define which unit gets to drive the BCK/LRCK pins if the codec is
 * required to be either BCK or LRCK master
 */
#define AD1939_DRV_DAC_LRCK_MASTER	0
#define AD1939_DRV_ADC_LRCK_MASTER	(1<<2)
#define AD1939_DRV_DAC_BCK_MASTER	0
#define AD1939_DRV_ADC_BCK_MASTER	(1<<3)

/* use TDM mode even for stereo (2-channel) signals */
#define AD1939_DRV_TDM_STEREO		(1<<4)

struct ad1939_setup_data {
	/* device address, WITHOUT the R/W bit! (default 0x04) */
	unsigned char dev_address;	/* I2C or SPI device address */
	unsigned char tdm_mode;		/* one of AD1939_TDM_MODE_* */
	unsigned char pll_src;		/* one of AD1939_PLL_SRC_* */
	unsigned char dac_adc_clksrc;	/* AD1939_{B,}CLKSRC_* or'ed together */
	unsigned char mclk_xo;		/* one of AD1939_MCLKXO_* */
	unsigned char drvflags;		/* driver flags */
	unsigned char mixpairs;		/* mixer chan pairs (L-R) to advertise */
	void(*powerfunc)(int);		/* codec power supply function */
};

extern struct snd_soc_codec_device	soc_codec_dev_ad1939;
extern struct snd_soc_codec_dai		ad1939_dai;

#endif
