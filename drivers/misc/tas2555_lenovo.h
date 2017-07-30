/*
 * ALSA SoC Texas Instruments TAS2555 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2015 Texas Instruments, Inc.
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef _TAS2555_H
#define _TAS2555_H

/* Page Control Register */
#define TAS2555_PAGECTL_REG			0

/* Book Control Register (available in page0 of each book) */
#define TAS2555_BOOKCTL_PAGE			0
#define TAS2555_BOOKCTL_REG			127

#define TAS2555_REG(book, page, reg)		(((book * 256 * 128) + \
						 (page * 128)) + reg)

#define TAS2555_BOOK_ID(reg)			(reg / (256 * 128))
#define TAS2555_PAGE_ID(reg)			((reg % (256 * 128)) / 128)
#define TAS2555_BOOK_REG(reg)			(reg % (256 * 128))
#define TAS2555_PAGE_REG(reg)			((reg % (256 * 128)) % 128)

/* Book0, Page0 registers */
#define TAS2555_SW_RESET_REG			TAS2555_REG(0, 0, 1)
#define TAS2555_REV_PGID_REG			TAS2555_REG(0, 0, 3)
#define TAS2555_POWER_CTRL1_REG			TAS2555_REG(0, 0, 4)
#define TAS2555_POWER_CTRL2_REG			TAS2555_REG(0, 0, 5)
#define TAS2555_SPK_CTRL_REG			TAS2555_REG(0, 0, 6)
#define TAS2555_MUTE_REG			TAS2555_REG(0, 0, 7)
#define TAS2555_CHANNEL_CTRL_REG		TAS2555_REG(0, 0, 8)
#define TAS2555_ADC_INPUT_SEL_REG		TAS2555_REG(0, 0, 9)
#define TAS2555_NONAME10_REG			TAS2555_REG(0, 0, 10)
#define TAS2555_NONAME11_REG			TAS2555_REG(0, 0, 11)
#define TAS2555_NONAME12_REG			TAS2555_REG(0, 0, 12)
#define TAS2555_NONAME13_REG			TAS2555_REG(0, 0, 13)
#define TAS2555_NONAME14_REG			TAS2555_REG(0, 0, 14)
#define TAS2555_NONAME15_REG			TAS2555_REG(0, 0, 15)
#define TAS2555_NONAME16_REG			TAS2555_REG(0, 0, 16)
#define TAS2555_NONAME17_REG			TAS2555_REG(0, 0, 17)
#define TAS2555_NONAME18_REG			TAS2555_REG(0, 0, 18)
#define TAS2555_SAR_SAMPLING_TIME_REG		TAS2555_REG(0, 0, 19)
#define TAS2555_SAR_ADC1_REG			TAS2555_REG(0, 0, 20)
#define TAS2555_SAR_ADC2_REG			TAS2555_REG(0, 0, 21)
#define TAS2555_CRC_CHECKSUM_REG		TAS2555_REG(0, 0, 32)
#define TAS2555_CRC_RESET_REG			TAS2555_REG(0, 0, 33)
#define TAS2555_DSP_MODE_SELECT_REG		TAS2555_REG(0, 0, 34)
#define TAS2555_NONAME42_REG			TAS2555_REG(0, 0, 42)
#define TAS2555_CLK_ERR_CTRL			TAS2555_REG(0, 0, 44)
#define TAS2555_POWER_UP_FLAG_REG		TAS2555_REG(0, 0, 100)
#define TAS2555_FLAGS_1				TAS2555_REG(0, 0, 104)
#define TAS2555_FLAGS_2				TAS2555_REG(0, 0, 108)
/* Book0, Page1 registers */
#define TAS2555_ASI1_DAC_FORMAT_REG		TAS2555_REG(0, 1, 1)
#define TAS2555_ASI1_ADC_FORMAT_REG		TAS2555_REG(0, 1, 2)
#define TAS2555_ASI1_OFFSET1_REG		TAS2555_REG(0, 1, 3)
#define TAS2555_ASI1_ADC_PATH_REG		TAS2555_REG(0, 1, 7)
#define TAS2555_ASI1_DAC_BCLK_REG		TAS2555_REG(0, 1, 8)
#define TAS2555_ASI1_DAC_WCLK_REG		TAS2555_REG(0, 1, 9)
#define TAS2555_ASI1_ADC_BCLK_REG		TAS2555_REG(0, 1, 10)
#define TAS2555_ASI1_ADC_WCLK_REG		TAS2555_REG(0, 1, 11)
#define TAS2555_ASI1_DIN_DOUT_MUX_REG		TAS2555_REG(0, 1, 12)
#define TAS2555_ASI1_BDIV_CLK_SEL_REG		TAS2555_REG(0, 1, 13)
#define TAS2555_ASI1_BDIV_CLK_RATIO_REG		TAS2555_REG(0, 1, 14)
#define TAS2555_ASI1_WDIV_CLK_RATIO_REG		TAS2555_REG(0, 1, 15)
#define TAS2555_ASI1_DAC_CLKOUT_REG		TAS2555_REG(0, 1, 16)
#define TAS2555_ASI1_ADC_CLKOUT_REG		TAS2555_REG(0, 1, 17)

#define TAS2555_ASI2_DAC_FORMAT_REG		TAS2555_REG(0, 1, 21)
#define TAS2555_ASI2_ADC_FORMAT_REG		TAS2555_REG(0, 1, 22)
#define TAS2555_ASI2_OFFSET1_REG		TAS2555_REG(0, 1, 23)
#define TAS2555_ASI2_ADC_PATH_REG		TAS2555_REG(0, 1, 27)
#define TAS2555_ASI2_DAC_BCLK_REG		TAS2555_REG(0, 1, 28)
#define TAS2555_ASI2_DAC_WCLK_REG		TAS2555_REG(0, 1, 29)
#define TAS2555_ASI2_ADC_BCLK_REG		TAS2555_REG(0, 1, 30)
#define TAS2555_ASI2_ADC_WCLK_REG		TAS2555_REG(0, 1, 31)
#define TAS2555_ASI2_DIN_DOUT_MUX_REG		TAS2555_REG(0, 1, 32)
#define TAS2555_ASI2_BDIV_CLK_SEL_REG		TAS2555_REG(0, 1, 33)
#define TAS2555_ASI2_BDIV_CLK_RATIO_REG		TAS2555_REG(0, 1, 34)
#define TAS2555_ASI2_WDIV_CLK_RATIO_REG		TAS2555_REG(0, 1, 35)
#define TAS2555_ASI2_DAC_CLKOUT_REG		TAS2555_REG(0, 1, 36)
#define TAS2555_ASI2_ADC_CLKOUT_REG		TAS2555_REG(0, 1, 37)

#define TAS2555_GPIO1_PIN_REG			TAS2555_REG(0, 1, 61)
#define TAS2555_GPIO2_PIN_REG			TAS2555_REG(0, 1, 62)
#define TAS2555_GPIO3_PIN_REG			TAS2555_REG(0, 1, 63)
#define TAS2555_GPIO4_PIN_REG			TAS2555_REG(0, 1, 64)
#define TAS2555_GPIO5_PIN_REG			TAS2555_REG(0, 1, 65)
#define TAS2555_GPIO6_PIN_REG			TAS2555_REG(0, 1, 66)
#define TAS2555_GPIO7_PIN_REG			TAS2555_REG(0, 1, 67)
#define TAS2555_GPIO8_PIN_REG			TAS2555_REG(0, 1, 68)
#define TAS2555_GPIO9_PIN_REG			TAS2555_REG(0, 1, 69)
#define TAS2555_GPIO10_PIN_REG			TAS2555_REG(0, 1, 70)

#define TAS2555_GPI_PIN_REG			TAS2555_REG(0, 1, 77)
#define TAS2555_GPIO_HIZ_CTRL1_REG		TAS2555_REG(0, 1, 79)
#define TAS2555_GPIO_HIZ_CTRL2_REG		TAS2555_REG(0, 1, 80)
#define TAS2555_GPIO_HIZ_CTRL3_REG		TAS2555_REG(0, 1, 81)
#define TAS2555_GPIO_HIZ_CTRL4_REG		TAS2555_REG(0, 1, 82)
#define TAS2555_GPIO_HIZ_CTRL5_REG		TAS2555_REG(0, 1, 83)

#define TAS2555_BIT_BANG_CTRL_REG		TAS2555_REG(0, 1, 87)
#define TAS2555_BIT_BANG_OUT1_REG		TAS2555_REG(0, 1, 88)
#define TAS2555_BIT_BANG_OUT2_REG		TAS2555_REG(0, 1, 89)
#define TAS2555_BIT_BANG_IN1_REG		TAS2555_REG(0, 1, 90)
#define TAS2555_BIT_BANG_IN2_REG		TAS2555_REG(0, 1, 91)
#define TAS2555_BIT_BANG_IN3_REG		TAS2555_REG(0, 1, 92)

#define TAS2555_PDM_IN_CLK_REG			TAS2555_REG(0, 1, 94)
#define TAS2555_PDM_IN_PIN_REG			TAS2555_REG(0, 1, 95)

#define TAS2555_ASIM_IFACE1_REG			TAS2555_REG(0, 1, 98)
#define TAS2555_ASIM_FORMAT_REG			TAS2555_REG(0, 1, 99)
#define TAS2555_ASIM_IFACE3_REG			TAS2555_REG(0, 1, 100)
#define TAS2555_ASIM_IFACE4_REG			TAS2555_REG(0, 1, 101)
#define TAS2555_ASIM_IFACE5_REG			TAS2555_REG(0, 1, 102)
#define TAS2555_ASIM_IFACE6_REG			TAS2555_REG(0, 1, 103)
#define TAS2555_ASIM_IFACE7_REG			TAS2555_REG(0, 1, 104)
#define TAS2555_ASIM_IFACE8_REG			TAS2555_REG(0, 1, 105)
#define TAS2555_ASIM_IFACE9_REG			TAS2555_REG(0, 1, 106)

#define TAS2555_MAIN_CLKIN_REG			TAS2555_REG(0, 1, 115)
#define TAS2555_PLL_CLKIN_REG			TAS2555_REG(0, 1, 116)
#define TAS2555_CLKOUT_MUX_REG			TAS2555_REG(0, 1, 117)
#define TAS2555_CLKOUT_CDIV_REG			TAS2555_REG(0, 1, 118)

#define TAS2555_HACK_GP01_REG			TAS2555_REG(0, 1, 122)

#define TAS2555_HACK01_REG			TAS2555_REG(0, 2, 10)

#define TAS2555_TEST_MODE_REG			TAS2555_REG(0, 253, 13)
#define TAS2555_CRYPTIC_REG			TAS2555_REG(0, 253, 71)

//#define TAS2555__REG      TAS2555_REG(0, 1, )
//#define TAS2555__REG      TAS2555_REG(1, 0, )
#define TAS2555_DAC_INTERPOL_REG		TAS2555_REG(100, 0, 1)
#define TAS2555_SOFT_MUTE_REG			TAS2555_REG(100, 0, 7)
#define TAS2555_PLL_P_VAL_REG			TAS2555_REG(100, 0, 27)
#define TAS2555_PLL_J_VAL_REG			TAS2555_REG(100, 0, 28)
#define TAS2555_PLL_D_VAL_MSB_REG		TAS2555_REG(100, 0, 29)
#define TAS2555_PLL_D_VAL_LSB_REG		TAS2555_REG(100, 0, 30)
#define TAS2555_CLK_MISC_REG			TAS2555_REG(100, 0, 31)
#define TAS2555_PLL_N_VAL_REG			TAS2555_REG(100, 0, 32)
#define TAS2555_DAC_MADC_VAL_REG		TAS2555_REG(100, 0, 33)
#define TAS2555_ISENSE_DIV_REG			TAS2555_REG(100, 0, 42)
#define TAS2555_RAMP_CLK_DIV_MSB_REG		TAS2555_REG(100, 0, 43)
#define TAS2555_RAMP_CLK_DIV_LSB_REG		TAS2555_REG(100, 0, 44)
/* Bits */
/* B0P0R4 - TAS2555_POWER_CTRL1_REG */
#define TAS2555_SW_SHUTDOWN			(0x1 << 0)
#define TAS2555_MADC_POWER_UP			(0x1 << 3)
#define TAS2555_MDAC_POWER_UP			(0x1 << 4)
#define TAS2555_NDIV_POWER_UP			(0x1 << 5)
#define TAS2555_PLL_POWER_UP			(0x1 << 6)
#define TAS2555_DSP_POWER_UP			(0x1 << 7)

/* B0P0R5 - TAS2555_POWER_CTRL2_REG */
#define TAS2555_VSENSE_ENABLE			(0x1 << 0)
#define TAS2555_ISENSE_ENABLE			(0x1 << 1)
#define TAS2555_BOOST_ENABLE			(0x1 << 5)
#define TAS2555_CLASSD_ENABLE			(0x1 << 7)

/* B0P0R6 - TAS2555_SPK_CTRL_REG */
#define TAS2555_DAC_GAIN_MASK			(0xf << 3)

/* B0P0R7 - TAS2555_MUTE_REG */
#define TAS2555_CLASSD_MUTE			(0x1 << 0)
#define TAS2555_ISENSE_MUTE			(0x1 << 1)

/* B0P253R13 - TAS2555_TEST_MODE_REG */
#define TAS2555_TEST_MODE_ENABLE		(13)
#define TAS2555_TEST_MODE_MASK			(0xf << 0)

/* B0P253R71 - TAS2555_CRYPTIC_REG */
#define TAS2555_OSC_TRIM_CAP(x)			((x & 0x3f) << 0)
#define TAS2555_DISABLE_ENCRYPTION		(0x1 << 6)
#define TAS2555_SL_COMP				(0x1 << 7)

/* B0P1R115/6 - TAS2555_MAIN/PLL_CLKIN_REG */
#define TAS2555_XXX_CLKIN_GPIO1			(0)
#define TAS2555_XXX_CLKIN_GPIO2			(1)
#define TAS2555_XXX_CLKIN_GPIO3			(2)
#define TAS2555_XXX_CLKIN_GPIO4			(3)
#define TAS2555_XXX_CLKIN_GPIO5			(4)
#define TAS2555_XXX_CLKIN_GPIO6			(5)
#define TAS2555_XXX_CLKIN_GPIO7			(6)
#define TAS2555_XXX_CLKIN_GPIO8			(7)
#define TAS2555_XXX_CLKIN_GPIO9			(8)
#define TAS2555_XXX_CLKIN_GPIO10		(9)
#define TAS2555_XXX_CLKIN_GPI1			(12)
#define TAS2555_XXX_CLKIN_GPI2			(13)
#define TAS2555_XXX_CLKIN_GPI3			(14)
#define TAS2555_NDIV_CLKIN_PLL			(15)
#define TAS2555_PLL_CLKIN_INT_OSC		(15)

#define TAS2555_MCLK_CLKIN_SRC_GPIO1       (0)
#define TAS2555_MCLK_CLKIN_SRC_GPIO2       (1)
#define TAS2555_MCLK_CLKIN_SRC_GPIO3       (2)
#define TAS2555_MCLK_CLKIN_SRC_GPIO4       (3)
#define TAS2555_MCLK_CLKIN_SRC_GPIO5       (4)
#define TAS2555_MCLK_CLKIN_SRC_GPIO6       (5)
#define TAS2555_MCLK_CLKIN_SRC_GPIO7       (6)
#define TAS2555_MCLK_CLKIN_SRC_GPIO8       (7)
#define TAS2555_MCLK_CLKIN_SRC_GPIO9       (8)
#define TAS2555_MCLK_CLKIN_SRC_GPIO10      (9)
#define TAS2555_MCLK_CLKIN_SRC_GPI1        (12)
#define TAS2555_MCLK_CLKIN_SRC_GPI2        (13)
#define TAS2555_MCLK_CLKIN_SRC_GPI3        (14)

#define TAS2555_FORMAT_I2S			(0x0 << 5)
#define TAS2555_FORMAT_DSP			(0x1 << 5)
#define TAS2555_FORMAT_RIGHT_J			(0x2 << 5)
#define TAS2555_FORMAT_LEFT_J			(0x3 << 5)
#define TAS2555_FORMAT_MONO_PCM			(0x4 << 5)
#define TAS2555_FORMAT_MASK			(0x7 << 5)

#define TAS2555_WORDLENGTH_16BIT		(0x0 << 3)
#define TAS2555_WORDLENGTH_20BIT		(0x1 << 3)
#define TAS2555_WORDLENGTH_24BIT		(0x2 << 3)
#define TAS2555_WORDLENGTH_32BIT		(0x3 << 3)
#define TAS2555_WORDLENGTH_MASK			TAS2555_WORDLENGTH_32BIT

/* B100P0R7 - TAS2555_SOFT_MUTE_REG */
#define TAS2555_PDM_SOFT_MUTE			(0x1 << 0)
#define TAS2555_VSENSE_SOFT_MUTE		(0x1 << 1)
#define TAS2555_ISENSE_SOFT_MUTE		(0x1 << 2)
#define TAS2555_CLASSD_SOFT_MUTE		(0x1 << 3)

/* B100P0R27 - TAS2555_PLL_P_VAL_REG */
#define TAS2555_PLL_P_VAL_MASK			(0x3f << 0)

/* B100P0R28 - TAS2555_PLL_J_VAL_REG */
#define TAS2555_PLL_J_VAL_MASK			((unsigned int ) (0x7f << 0))
#define TAS2555_PLL_J_VAL_MASKX	0x00

/* B100P0R29-30 - TAS2555_PLL_D_VAL_MSB/LSB_REG */
#define TAS2555_PLL_D_MSB_VAL(x)		((x >> 8) & 0x3f)
#define TAS2555_PLL_D_LSB_VAL(x)		(x & 0xff)

/* B100P0R31 - TAS2555_CLK_MISC_REG */
#define TAS2555_DSP_CLK_FROM_PLL		(0x1 << 5)

typedef struct {
	unsigned int mnType;
	unsigned int mnCommands;
	unsigned char *mpData;
} TBlock;

typedef struct {
	char mpName[64];
	char *mpDescription;
	unsigned int mnBlocks;
	TBlock *mpBlocks;
} TData;

typedef struct {
	char mpName[64];
	char *mpDescription;
	TData mData;
} TProgram;

typedef struct {
	char mpName[64];
	char *mpDescription;
	TBlock mBlock;
} TPLL;

typedef struct {
	char mpName[64];
	char *mpDescription;
	unsigned int mnProgram;
	unsigned int mnPLL;
	unsigned int mnSamplingRate;
	TData mData;
} TConfiguration;

typedef struct
{
	char mpName[64];
	char *mpDescription;
	unsigned int mnProgram;
	unsigned int mnConfiguration;
	TBlock mBlock;
} TCalibration;

typedef struct {
	unsigned int mnFWSize;
	unsigned int mnChecksum;
	unsigned int mnPPCVersion;
	unsigned int mnFWVersion;
	unsigned int mnTimeStamp;
	char mpDDCName[64];
	char *mpDescription;
	unsigned int mnDeviceFamily;
	unsigned int mnDevice;
	unsigned int mnPLLs;
	TPLL *mpPLLs;
	unsigned int mnPrograms;
	TProgram *mpPrograms;
	unsigned int mnConfigurations;
	TConfiguration *mpConfigurations;
	unsigned int mnCalibrations;
	TCalibration *mpCalibrations;
} TFirmware;

struct tas2555_register {
	int book;
	int page;
	int reg;
};

struct tas2555_priv {
	u8 i2c_regs_status;
	struct device *dev;
	struct regmap *mpRegmap;
	TFirmware *mpFirmware;
	TFirmware *mpCalFirmware;
	unsigned int mnCurrentProgram;
	unsigned int mnCurrentConfiguration;
	unsigned int mclk_clkin;
	int mclk_clk_id;
	int mnCurrentBook;
	int mnCurrentPage;
	int mnCurrentReg;
	int mnDBGCmd;
	bool mbTILoadActive;
	int reset_gpio;
	bool mbPowerUp;
	bool mbLoadConfigurationPostPowerUp;
	unsigned int mnPowerCtrl;
	bool mbCalibrationLoaded;
	int (*read) (struct tas2555_priv * pTAS2555, unsigned int reg,
		unsigned int *pValue);
	int (*write) (struct tas2555_priv * pTAS2555, unsigned int reg,
		unsigned int Value);
	int (*bulk_read) (struct tas2555_priv * pTAS2555, unsigned int reg,
		unsigned char *pData, unsigned int len);
	int (*bulk_write) (struct tas2555_priv * pTAS2555, unsigned int reg,
		unsigned char *pData, unsigned int len);
	int (*set_config) (struct tas2555_priv *pTAS2555, int config);
};
extern int msm8x16_quat_mi2s_clk_ctl(bool enable);

#define	FW_NAME_SIZE			64
#define	FW_DESCRIPTION_SIZE		256
#define	PROGRAM_BUF_SIZE 		(2 + FW_NAME_SIZE + FW_DESCRIPTION_SIZE)
#define	CONFIGURATION_BUF_SIZE 	(8 + FW_NAME_SIZE + FW_DESCRIPTION_SIZE)

#define	TIAUDIO_CMD_REG_WITE			1
#define	TIAUDIO_CMD_REG_READ			2
#define	TIAUDIO_CMD_DEBUG_ON			3
#define	TIAUDIO_CMD_PROGRAM				4
#define	TIAUDIO_CMD_CONFIGURATION		5
#define	TIAUDIO_CMD_FW_TIMESTAMP		6

#define	TAS2555_MAGIC_NUMBER	0x32353535	/* '2555' */

#define	SMARTPA_SPK_DAC_VOLUME	 			_IOWR(TAS2555_MAGIC_NUMBER, 1, unsigned long)
#define	SMARTPA_SPK_POWER_ON 				_IOWR(TAS2555_MAGIC_NUMBER, 2, unsigned long)
#define	SMARTPA_SPK_POWER_OFF 				_IOWR(TAS2555_MAGIC_NUMBER, 3, unsigned long)
#define	SMARTPA_SPK_SWITCH_PROGRAM 			_IOWR(TAS2555_MAGIC_NUMBER, 4, unsigned long)
#define	SMARTPA_SPK_SWITCH_CONFIGURATION 	_IOWR(TAS2555_MAGIC_NUMBER, 5, unsigned long)

#endif	/* _TAS2555_H */
