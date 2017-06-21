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

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/i2c-dev.h>
//#include <dt-bindings/sound/tas2555.h>

#include "tas2555_lenovo.h"


//set default PLL CLKIN to GPI2 (MCLK) = 0x00
#define TAS2555_DEFAULT_PLL_CLKIN 0x00
#define ENABLE_GPIO_RESET
#define ENABLE_TILOAD			//only enable this for in-system tuning or debug, not for production systems
#ifdef ENABLE_TILOAD
#include "tiload_lenovo.h"
#endif

#define TAS2555_FW_NAME     "tas2555_uCDSP.bin"
#define TAS2555_CAL_NAME    "/persist/audio/tas2555_cal.bin"

#define TAS2555_FW_FORMAT   0x01
const char *tas2555_fw_header = "TAS2555-uCDSP";

#define MAX_TUNINGS 16

#define TAS2555_UDELAY 0xFFFFFFFE

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2555_BLOCK_BASE_MAIN		0x01
#define TAS2555_BLOCK_CONF_COEFF	0x03
#define TAS2555_BLOCK_CONF_PRE		0x04
#define TAS2555_BLOCK_CONF_POST		0x05
#define TAS2555_BLOCK_CONF_POST_POWER	0x06
#define TAS2555_BLOCK_CONF_CAL		0x0A

#define TAS2555_REG_IS_VALID(book, page, reg) \
        ((book >= 0) && (book <= 255) &&\
        (page >= 0) && (page <= 255) &&\
        (reg >= 0) && (reg <= 127))

static struct tas2555_priv *g_tas2555 = NULL;
static int g_logEnable = 0;

extern int msm8x16_quin_mi2s_clk_ctl(bool enable);

static int fw_parse(TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize);
static void tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock);
static void tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType);
static void tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame);
static void tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName);

static void tas2555_change_book_page(struct tas2555_priv *pTAS2555, int nBook,
	int nPage)
{
	if ((pTAS2555->mnCurrentBook == nBook) && pTAS2555->mnCurrentPage == nPage)
		return;

	if (pTAS2555->mnCurrentBook != nBook) {
		regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_PAGE, 0);
		pTAS2555->mnCurrentPage = 0;
		regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_REG, nBook);
		pTAS2555->mnCurrentBook = nBook;
		if (nPage != 0) {
			regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_PAGE, nPage);
			pTAS2555->mnCurrentPage = nPage;
		}
	} else if (pTAS2555->mnCurrentPage != nPage) {
		regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_PAGE, nPage);
		pTAS2555->mnCurrentPage = nPage;
	}
}

static int tas2555_i2c_read(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int *pValue)
{
	int ret = 0;

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only reads from TILoad pass.
		nRegister &= ~0x80000000;
	}

	dev_dbg(pTAS2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
		TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister),
		TAS2555_PAGE_REG(nRegister));
	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
	ret = regmap_read(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister), pValue);

	return ret;
}

static int tas2555_i2c_write(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int nValue)
{
	if ((nRegister == 0xAFFEAFFE) && (nValue == 0xBABEBABE)) {
		pTAS2555->mbTILoadActive = true;
		return 0;
	}

	if ((nRegister == 0xBABEBABE) && (nValue == 0xAFFEAFFE)) {
		pTAS2555->mbTILoadActive = false;
		return 0;
	}

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}

	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
//  dev_err(codec->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
//      __func__, TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister),
//      TAS2555_PAGE_REG(nRegister), value);
	return regmap_write(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister),
		nValue);
}

static int tas2555_i2c_update_bits(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 nMask, u8 nValue)
{
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}
	
	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
	
	return regmap_update_bits(pTAS2555->mpRegmap, 
		TAS2555_PAGE_REG(nRegister), nMask, nValue);
}

static int tas2555_i2c_bulk_read(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}

	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
	return regmap_bulk_read(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister),
		pData, nLength);
}

static int tas2555_i2c_bulk_write(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}

	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
	return regmap_bulk_write(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister),
		pData, nLength);
}

static bool tas2555_volatile(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static bool tas2555_writeable(struct device *pDev, unsigned int nRegister)
{
	return true;
}


static const struct regmap_config tas2555_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2555_writeable,
	.volatile_reg = tas2555_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

#define TAS2555_PCTRL1_MASK (TAS2555_MADC_POWER_UP | \
                 TAS2555_MDAC_POWER_UP | \
                 TAS2555_DSP_POWER_UP)
#define TAS2555_PCTRL2_MASK (TAS2555_VSENSE_ENABLE | \
                 TAS2555_ISENSE_ENABLE | \
                 TAS2555_BOOST_ENABLE)
#define TAS2555_MUTE_MASK   (TAS2555_ISENSE_MUTE | TAS2555_CLASSD_MUTE)
#define TAS2555_SOFT_MUTE_MASK  (TAS2555_PDM_SOFT_MUTE | \
                 TAS2555_VSENSE_SOFT_MUTE | \
                 TAS2555_ISENSE_SOFT_MUTE | \
                 TAS2555_CLASSD_SOFT_MUTE)

static unsigned int p_tas2555_default_data[] = {
	TAS2555_ASI1_DAC_FORMAT_REG, 0x10,	//ASI1 DAC word length = 24 bits

	TAS2555_PLL_CLKIN_REG, 0x00,	//PLL_CLKIN = GPIO1 (BCLK)
	TAS2555_MAIN_CLKIN_REG, 0x0F,	//NDIV_MUX_CLKIN = PLL_CLK
	TAS2555_PLL_P_VAL_REG, 0x01,	//PLL P = 1
//  TAS2555_PLL_J_VAL_REG,      0x10, //PLL J = 16
	TAS2555_PLL_J_VAL_REG, 0x30,	//PLL J = 48 -> PLL_CLK = 1.536MHz * 48 = 73.728MHz
	TAS2555_PLL_D_VAL_MSB_REG, 0x00,	//PLL D = 0
	TAS2555_PLL_D_VAL_LSB_REG, 0x00,
	TAS2555_PLL_N_VAL_REG, 0x03,	//PLL N = 3 -> NDIV_CLK = 24.576MHz
	TAS2555_DAC_MADC_VAL_REG, 0x08,	//MDAC = 8
	TAS2555_CLK_MISC_REG, 0x20,	//DSP CLK = PLL out
//  TAS2555_ISENSE_DIV_REG,     0x40, //Isense div and MADC final divider configure auto
	TAS2555_ISENSE_DIV_REG, 0x00,	//Isense div and MADC final divider configure auto
//  TAS2555_RAMP_CLK_DIV_LSB_REG,   0x20, //ramp_clk divider = 32 so that 12.288MHz/32 = 384KHz
	TAS2555_RAMP_CLK_DIV_LSB_REG, 0x40,	//ramp_clk divider = 64 so that 24.576MHz/64 = 384KHz
	TAS2555_DSP_MODE_SELECT_REG, 0x22,	//DSP ROM mode 2, default coeffs

//  TAS2555_SPK_CTRL_REG,       0x74, //DAC channel gain
	TAS2555_SPK_CTRL_REG, 0x04,	//DAC channel gain
//  TAS2555_POWER_CTRL2_REG,    0xA3, //power up
//  TAS2555_POWER_CTRL1_REG,    0xF8, //power up
//  TAS2555_MUTE_REG,       0x00, //unmute
//  TAS2555_SOFT_MUTE_REG,      0x00, //soft unmute
//  TAS2555_CLK_ERR_CTRL,       0x09, //enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};

#define TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX 3
static unsigned int p_tas2555_startup_data[] = {
	TAS2555_CLK_ERR_CTRL, 0x00,	//disable clock error detection on PLL
	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,
	TAS2555_POWER_CTRL2_REG, 0xA0,	//Class-D, Boost power up
	TAS2555_POWER_CTRL2_REG, 0xA3,	//Class-D, Boost, IV sense power up
	TAS2555_POWER_CTRL1_REG, 0xF8,	//PLL, DSP, clock dividers power up
	TAS2555_UDELAY, 2000,
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_unmute_data[] = {
	TAS2555_MUTE_REG, 0x00,		//unmute
	TAS2555_SOFT_MUTE_REG, 0x00,	//soft unmute
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_shutdown_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	TAS2555_POWER_CTRL2_REG, 0x00,	//Class-D, Boost power down
	TAS2555_POWER_CTRL1_REG, 0x00,	//all power down
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_mute_DSP_down_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static void tas2555_i2c_load_data(struct tas2555_priv *pTAS2555,
	unsigned int *pData)
{
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
		if (nRegister == TAS2555_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF)
			tas2555_i2c_write(pTAS2555, nRegister, nData);
		n++;
	} while (nRegister != 0xFFFFFFFF);
}

static void tas2555_enable(struct tas2555_priv *pTAS2555, bool bEnable)
{
	dev_dbg(pTAS2555->dev, "Enable: %d\n", bEnable);
	if (bEnable) {
		if (!pTAS2555->mbPowerUp) {
			TConfiguration *pConfiguration;

			if (!pTAS2555->mbCalibrationLoaded) {
				tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
				pTAS2555->mbCalibrationLoaded = true;
			}
			dev_dbg(pTAS2555->dev, "Enable: load startup sequence\n");
			tas2555_i2c_load_data(pTAS2555, p_tas2555_startup_data);
			if (pTAS2555->mpFirmware->mpConfigurations != NULL) {
				pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
				tas2555_load_data(pTAS2555, &(pConfiguration->mData),
					TAS2555_BLOCK_CONF_POST_POWER);
				if (pTAS2555->mbLoadConfigurationPostPowerUp) {
					u8 data[4];
					dev_dbg(pTAS2555->dev,	"Enable: load configuration: %s, %s\n",
						pConfiguration->mpName, pConfiguration->mpDescription);
					tas2555_load_data(pTAS2555, &(pConfiguration->mData),
						TAS2555_BLOCK_CONF_COEFF);
					tas2555_i2c_bulk_read(pTAS2555, TAS2555_REG(0x8c, 0x2a,	0x60), data, 4);
					dev_dbg(pTAS2555->dev, "@@: REG[8c,2a,60] = 0x%x 0x%x 0x%x 0x%x\n",
						data[0], data[1], data[2], data[3]);
					tas2555_i2c_bulk_read(pTAS2555, TAS2555_REG(0x8c, 0x2a,	0x64), data, 4);
					dev_dbg(pTAS2555->dev, "@@: REG[8c,2a,64] = 0x%x 0x%x 0x%x 0x%x\n",
						data[0], data[1], data[2], data[3]);
					pTAS2555->mbLoadConfigurationPostPowerUp = false;
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
						tas2555_load_block(pTAS2555, &(pTAS2555->mpCalFirmware->mpCalibrations[0].mBlock));
					}
				}
			}
			dev_dbg(pTAS2555->dev, "Enable: load unmute sequence\n");
			tas2555_i2c_load_data(pTAS2555, p_tas2555_unmute_data);
			pTAS2555->mbPowerUp = true;
		}
	} else {
		if (pTAS2555->mbPowerUp) {
			dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
			tas2555_i2c_load_data(pTAS2555, p_tas2555_shutdown_data);
			//tas2555_i2c_load_data(pTAS2555, p_tas2555_shutdown_clk_err);
			pTAS2555->mbPowerUp = false;
		}
	}
}

/*
static int tas2555_set_sampling_rate(struct tas2555_priv *pTAS2555, unsigned int nSamplingRate)
{
	TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2555->dev, "tas2555_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((pTAS2555->mpFirmware->mpPrograms == NULL) ||
		(pTAS2555->mpFirmware->mpConfigurations == NULL)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_dbg(pTAS2555->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2555->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		if (pConfiguration->mnSamplingRate == nSamplingRate) {
			dev_dbg(pTAS2555->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			tas2555_load_configuration(pTAS2555, nConfiguration, false);
			return 0;
		}
	}

	dev_dbg(pTAS2555->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

	return -EINVAL;
}
*/

static void LOG(const char *pLine)
{
#ifdef DEBUG
	dev_dbg(g_tas2555->dev, "%s\n", pLine);
#endif
}

char pLog[256];

static void fw_print_header(TFirmware * pFirmware)
{
	LOG("Header:");
	sprintf(pLog, "  FW Size       = %d", pFirmware->mnFWSize);
	LOG(pLog);
	sprintf(pLog, "  Checksum      = 0x%04X", pFirmware->mnChecksum);
	LOG(pLog);
	sprintf(pLog, "  PPC Version   = 0x%04X", pFirmware->mnPPCVersion);
	LOG(pLog);
	sprintf(pLog, "  FW  Version   = 0x%04X", pFirmware->mnFWVersion);
	LOG(pLog);
	sprintf(pLog, "  Timestamp     = %d", pFirmware->mnTimeStamp);
	LOG(pLog);
	sprintf(pLog, "  DDC Name      = %s", pFirmware->mpDDCName);
	LOG(pLog);
	sprintf(pLog, "  Description   = %s", pFirmware->mpDescription);
	LOG(pLog);
	sprintf(pLog, "  Device Family = %d", pFirmware->mnDeviceFamily);
	LOG(pLog);
	sprintf(pLog, "  Device        = %d", pFirmware->mnDevice);
	LOG(pLog);
	LOG("");
}

static void fw_print_block(TBlock * pBlock)
{
	LOG("    Block:");
	sprintf(pLog, "    Type     = %d", pBlock->mnType);
	LOG(pLog);
	sprintf(pLog, "    Commands = %d", pBlock->mnCommands);
	LOG(pLog);
}

static void fw_print_data(TData * pData)
{
	unsigned int n;

	LOG("  Data:");
	sprintf(pLog, "    Name         = %s", pData->mpName);
	LOG(pLog);
	sprintf(pLog, "    Description  = %s", pData->mpDescription);
	LOG(pLog);
	sprintf(pLog, "    Blocks       = %d", pData->mnBlocks);
	LOG(pLog);

	for (n = 0; n < pData->mnBlocks; n++)
		fw_print_block(&(pData->mpBlocks[n]));
}

static void fw_print_PLL(TPLL * pPLL, unsigned int nPLL)
{
	sprintf(pLog, "  PLL: %d", nPLL);
	LOG(pLog);
	sprintf(pLog, "    Name         = %s", pPLL->mpName);
	LOG(pLog);
	sprintf(pLog, "    Description  = %s", pPLL->mpDescription);
	LOG(pLog);
	fw_print_block(&(pPLL->mBlock));
	LOG("");
}

void fw_print_configuration(TConfiguration * pConfiguration,
	unsigned int nConfiguration)
{
	sprintf(pLog, "Configuration: %d", nConfiguration);
	LOG(pLog);
	sprintf(pLog, "  Name          = %s", pConfiguration->mpName);
	LOG(pLog);
	sprintf(pLog, "  Description   = %s", pConfiguration->mpDescription);
	LOG(pLog);
	sprintf(pLog, "  Program       = %d", pConfiguration->mnProgram);
	LOG(pLog);
	sprintf(pLog, "  PLL           = %d", pConfiguration->mnPLL);
	LOG(pLog);
	sprintf(pLog, "  Sampling Rate = %d", pConfiguration->mnSamplingRate);
	LOG(pLog);
	fw_print_data(&(pConfiguration->mData));
	LOG("");
}

void fw_print_program(TProgram * pProgram, unsigned int nProgram)
{
	sprintf(pLog, "Program: %d", nProgram);
	LOG(pLog);
	sprintf(pLog, "  Name = %s", pProgram->mpName);
	LOG(pLog);
	sprintf(pLog, "  Description = %s", pProgram->mpDescription);
	LOG(pLog);

	fw_print_data(&(pProgram->mData));
	LOG("");
}

void fw_print_calibration(TCalibration * pCalibration,
	unsigned int nCalibration)
{
	LOG("Calibration:");
	sprintf(pLog, "  Name          = %s", pCalibration->mpName);
	LOG(pLog);
	sprintf(pLog, "  Description   = %s", pCalibration->mpDescription);
	LOG(pLog);
	sprintf(pLog, "  Program       = %d", pCalibration->mnProgram);
	LOG(pLog);
	sprintf(pLog, "  Configuration = %d", pCalibration->mnConfiguration);
	LOG(pLog);
	fw_print_block(&(pCalibration->mBlock));
	LOG("");
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };
	if (nSize < 102) {
		LOG("Firmware: Header too short");
		return -1;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		LOG("Firmware: Magic number doesn't match");
		return -1;
	}

	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;

	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	if ((pData - pDataStart) >= nSize) {
		LOG("Firmware: Header too short after DDC description");
		return -1;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	fw_print_header(pFirmware);

	return pData - pDataStart;
}

static int fw_parse_block_data(TBlock * pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);
	pData += n;

	return pData - pDataStart;
}

static int fw_parse_data(TData * pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;

	pImageData->mpBlocks =
		kmalloc(sizeof(TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(&(pImageData->mpBlocks[nBlock]), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_pll_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nPLL;
	TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPLLs = kmalloc(sizeof(TPLL) * pFirmware->mnPLLs, GFP_KERNEL);
	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_block_data(&(pPLL->mBlock), pData);
		pData += n;

		fw_print_PLL(pPLL, nPLL);
	}

	return pData - pDataStart;
}

static int fw_parse_program_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nProgram;
	TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPrograms =
		kmalloc(sizeof(TProgram) * pFirmware->mnPrograms, GFP_KERNEL);
	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_data(&(pProgram->mData), pData);
		pData += n;

		fw_print_program(pProgram, nProgram);
	}

	return pData - pDataStart;
}

static int fw_parse_configuration_data(TFirmware * pFirmware,
	unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nConfiguration;
	TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpConfigurations =
		kmalloc(sizeof(TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);
	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;

		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(&(pConfiguration->mData), pData);
		pData += n;

		fw_print_configuration(pConfiguration, nConfiguration);
	}

	return pData - pDataStart;
}

int fw_parse_calibration_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nCalibration;
	TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpCalibrations =
		kmalloc(sizeof(TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);
	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_block_data(&(pCalibration->mBlock), pData);
		pData += n;

		fw_print_calibration(pCalibration, nCalibration);
	}

	return pData - pDataStart;
}

static int fw_parse(TFirmware * pFirmware,
		unsigned char *pData,
		unsigned int nSize)
{
	int nPosition;

	sprintf(pLog, "fw size = %d", nSize);
	LOG(pLog);

	nPosition = fw_parse_header(pFirmware, pData, nSize);
	if (nPosition < 0) {
		LOG("Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		LOG("Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition = fw_parse_calibration_data(pFirmware, pData);

	return 0;
}

static void tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock)
{
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2555->dev, "TAS2555 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);
	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;

		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F)
			tas2555_i2c_write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset),
				nData);
		if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;
			dev_dbg(pTAS2555->dev,
				"TAS2555 load block: nOffset = 0x81 -> sleep %d [ms]\n",
				nSleep);
			msleep(nSleep);
			dev_dbg(pTAS2555->dev,
				"TAS2555 load block: just woke up from sleep %d [ms]\n",
				nSleep);
		}
		if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1)
				tas2555_i2c_bulk_write(pTAS2555, TAS2555_REG(nBook, nPage,
						nOffset), pData + 3, nLength);
			else
				tas2555_i2c_write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset),
					pData[3]);

			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}
}

static void tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType)
{
	unsigned int nBlock;
	TBlock *pBlock;

	dev_dbg(pTAS2555->dev,
		"TAS2555 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName,
		pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType)
			tas2555_load_block(pTAS2555, pBlock);
	}

	dev_dbg(pTAS2555->dev, "%s(), exit\n", __func__);
}

static void tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame)
{
	TConfiguration *pCurrentConfiguration;
	TConfiguration *pNewConfiguration;
	TPLL *pNewPLL;

	dev_dbg(pTAS2555->dev, "tas2555_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return;
	}

	if ((nConfiguration == pTAS2555->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_dbg(pTAS2555->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return;
	}

	pCurrentConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);

	if (pNewConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		return;
	}

	pNewPLL = &(pTAS2555->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);

	if (pTAS2555->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up -> mute and power down DSP before loading new configuration\n");
			//tas2555_i2c_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
			tas2555_i2c_load_data(pTAS2555, p_tas2555_shutdown_data);

			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			dev_dbg(pTAS2555->dev, "TAS2555: power up TAS2555\n");
			tas2555_i2c_load_data(pTAS2555, p_tas2555_startup_data);
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, post power up block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			dev_dbg(pTAS2555->dev, "TAS2555: unmute TAS2555\n");
			tas2555_i2c_load_data(pTAS2555, p_tas2555_unmute_data);
		} else {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
		}
		pTAS2555->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2555->dev,
			"TAS2555 was powered down -> set flag to load configuration data when OS powers up the TAS2555 the next time\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
		}
		pTAS2555->mbLoadConfigurationPostPowerUp = true;
	}

	pTAS2555->mnCurrentConfiguration = nConfiguration;
}

static void tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName)
{
	int nResult;
	int nFile;
	mm_segment_t fs;
	unsigned char pBuffer[512];
	int nSize = 0;

	dev_dbg(pTAS2555->dev, "%s:\n", __func__);

	fs = get_fs();
	set_fs(KERNEL_DS);
	nFile = sys_open(pFileName, O_RDONLY, 0);

	dev_info(pTAS2555->dev, "TAS2555 calibration file = %s, handle = %d\n",
		pFileName, nFile);

	if (nFile >= 0) {
		nSize = sys_read(nFile, pBuffer, 512);
		sys_close(nFile);
	} else {
		dev_err(pTAS2555->dev, "TAS2555 cannot open calibration file: %s\n",
			pFileName);
	}

	set_fs(fs);

	if (!nSize)
		return;

	dev_info(pTAS2555->dev, "TAS2555 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2555->mpCalFirmware, pBuffer, nSize);

	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 calibration file is corrupt\n");
		return;
	}

	dev_info(pTAS2555->dev, "TAS2555 calibration: %d calibrations\n",
		pTAS2555->mpCalFirmware->mnCalibrations);
}

static void tas2555_i2c_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *) pContext;
	TConfiguration *pConfiguration;
	TPLL *pPLL;
	int nResult;
	unsigned int Value;

	dev_info(pTAS2555->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_info(pTAS2555->dev, "%s firmware is not loaded.\n",
			TAS2555_FW_NAME);
		return;
	}

	nResult =
		fw_parse(pTAS2555->mpFirmware, (unsigned char *) (pFW->data),
		pFW->size);
	release_firmware(pFW);
	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 firmware is corrupt\n");
		return;
	}

	dev_info(pTAS2555->dev, "TAS2555 firmware: %d programs\n",
		pTAS2555->mpFirmware->mnPrograms);
	dev_info(pTAS2555->dev, "TAS2555 firmware: %d configurations\n",
		pTAS2555->mpFirmware->mnConfigurations);

	if (!pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555 firmware contains no programs\n");
		return;
	}

	if (!pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "TAS2555 firmware contains no configurations\n");
		return;
	}

	tas2555_i2c_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
	tas2555_i2c_write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	udelay(1000);

	pTAS2555->mnCurrentBook = 0;
	pTAS2555->mnCurrentPage = 0;

	tas2555_i2c_write(pTAS2555, TAS2555_CRC_RESET_REG, 0x01);
	dev_info(pTAS2555->dev, "TAS2555 load base image: %s main block\n",
		pTAS2555->mpFirmware->mpPrograms[0].mpName);
	tas2555_load_data(pTAS2555, &(pTAS2555->mpFirmware->mpPrograms[0].mData),
		TAS2555_BLOCK_BASE_MAIN);
	pTAS2555->mbLoadConfigurationPostPowerUp = true;
	pTAS2555->mnCurrentConfiguration = 0;
	pTAS2555->mnCurrentProgram = 0;

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[0]);
	if (pConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"TAS2555 Configuration #0 doesn't have a valid PLL index #%d, max = %d\n",
			pConfiguration->mnPLL, pTAS2555->mpFirmware->mnPLLs);
		return;
	} else {
		pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
		dev_info(pTAS2555->dev,
			"TAS2555 load PLL: %s block for Configuration %s\n",
			pPLL->mpName, pConfiguration->mpName);
		tas2555_load_block(pTAS2555, &(pPLL->mBlock));
	}

	nResult = tas2555_i2c_read(pTAS2555, TAS2555_CRC_CHECKSUM_REG, &Value);
	if (nResult < 0)
		dev_err(pTAS2555->dev, "%d, ERROR!\n", __LINE__);
	else
		dev_info(pTAS2555->dev, "uCDSP Checksum: 0x%02x\n", Value);

	tas2555_i2c_read(pTAS2555, TAS2555_PLL_CLKIN_REG, &Value);
	dev_info(pTAS2555->dev, "TAS2555 PLL_CLKIN = 0x%02X\n", Value);
	p_tas2555_startup_data[TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX] = Value;

	tas2555_load_data(pTAS2555, &(pConfiguration->mData),
		TAS2555_BLOCK_CONF_PRE);

	return;
}

static int tas2555_program_put(struct tas2555_priv *pTAS2555, unsigned int nProgram)
{
	TPLL *pPLL;
	TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	bool bFound = false;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -1;
	}
	if (nProgram >= pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555: Program %d doesn't exist\n",
			nConfiguration);
		return -1;
	}

	pTAS2555->mnCurrentProgram = nProgram;

	tas2555_i2c_write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);

	udelay(1000);
	pTAS2555->mnCurrentBook = 0;
	pTAS2555->mnCurrentPage = 0;

	tas2555_load_data(pTAS2555,
		&(pTAS2555->mpFirmware->mpPrograms[nProgram].mData),
		TAS2555_BLOCK_BASE_MAIN);

	nConfiguration = 0;
	while (!bFound && (nConfiguration < pTAS2555->mpFirmware->mnConfigurations)) {
		if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram ==
			nProgram)
			bFound = true;
		else
			nConfiguration++;
	}

	if (bFound) {
		pTAS2555->mnCurrentConfiguration = nConfiguration;

		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
		dev_dbg(pTAS2555->dev,
			"TAS2555 load PLL: %s block for Configuration %s\n",
			pPLL->mpName, pConfiguration->mpName);
		tas2555_load_block(pTAS2555, &(pPLL->mBlock));

		if (pTAS2555->mbPowerUp)
			tas2555_i2c_load_data(pTAS2555, p_tas2555_startup_data);
		tas2555_load_configuration(pTAS2555, nConfiguration, true);
		if (pTAS2555->mbPowerUp)
			tas2555_i2c_load_data(pTAS2555, p_tas2555_unmute_data);
	}

	dev_dbg(pTAS2555->dev,
		"tas2555_program_put = %d, found configuration = %d, %d\n",
		pTAS2555->mnCurrentProgram, bFound, nConfiguration);

	return 0;
}

static int tas2555_set_config(struct tas2555_priv *pTAS2555, int config)
{
	TConfiguration *pConfiguration;
	TProgram *pProgram;
	unsigned int nProgram = pTAS2555->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return -1;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2555->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		return -1;
	}

	tas2555_load_configuration(pTAS2555, nConfiguration, false);

	dev_dbg(pTAS2555->dev, "tas2555_configuration_put = %d\n",
		pTAS2555->mnCurrentConfiguration);

	return 0;
}



static int tas2555_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE)) return -ENODEV;

	file->private_data = (void*)g_tas2555;
	return 0;
}

static int tas2555_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void*)NULL;
	module_put(THIS_MODULE);

	return 0;
}

static ssize_t tas2555_file_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)file->private_data;
	int ret = 0;
	unsigned int value = 0;
	unsigned char *p_kBuf = NULL;
	
	switch(pTAS2555->mnDBGCmd)
	{
		case TIAUDIO_CMD_REG_READ:
		{
			if(g_logEnable) printk("current_reg = 0x%x, count=%d\n", pTAS2555->mnCurrentReg, (int)count);
			if(count == 1){
				ret = tas2555_i2c_read(pTAS2555, pTAS2555->mnCurrentReg, &value);
				if( 0 > ret) {
					dev_err(pTAS2555->dev, "dev read fail %d\n", ret);
					return ret;
				}
				
				ret = copy_to_user(buf, &value, 1);
				if (0 != ret) {
					/* Failed to copy all the data, exit */
					dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					return 0;
				}	
			}else if(count > 1){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					ret = tas2555_i2c_bulk_read(pTAS2555, pTAS2555->mnCurrentReg, p_kBuf, count);
					if( 0 > ret) {
						dev_err(pTAS2555->dev, "dev bulk read fail %d\n", ret);
						return ret;
					}
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
						return 0;
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
					return -ENOMEM;
				}
			}
		}
		break;
		
		case TIAUDIO_CMD_PROGRAM:
		{
			if(count == PROGRAM_BUF_SIZE){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					TProgram * pProgram = 
						&(pTAS2555->mpFirmware->mpPrograms[pTAS2555->mnCurrentProgram]);
					
					p_kBuf[0] = pTAS2555->mpFirmware->mnPrograms;							
					p_kBuf[1] = pTAS2555->mnCurrentProgram;					
					memcpy(&p_kBuf[2], pProgram->mpName, FW_NAME_SIZE);
					strcpy(&p_kBuf[2+FW_NAME_SIZE], pProgram->mpDescription);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
						return 0;
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
					return -ENOMEM;
				}				
			}else{
				dev_err(pTAS2555->dev, "read buffer not sufficient\n");
			}
		}
		break;
		
		case TIAUDIO_CMD_CONFIGURATION:
		{
			if(count == CONFIGURATION_BUF_SIZE){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					TConfiguration * pConfiguration = 
						&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);

					p_kBuf[0] = pTAS2555->mpFirmware->mnConfigurations;											
					p_kBuf[1] = pTAS2555->mnCurrentConfiguration;					
					memcpy(&p_kBuf[2], pConfiguration->mpName, FW_NAME_SIZE);
					p_kBuf[2+FW_NAME_SIZE] = pConfiguration->mnProgram;
					p_kBuf[3+FW_NAME_SIZE] = pConfiguration->mnPLL;
					p_kBuf[4+FW_NAME_SIZE] = (pConfiguration->mnSamplingRate&0x000000ff);
					p_kBuf[5+FW_NAME_SIZE] = ((pConfiguration->mnSamplingRate&0x0000ff00)>>8);
					p_kBuf[6+FW_NAME_SIZE] = ((pConfiguration->mnSamplingRate&0x00ff0000)>>16);
					p_kBuf[7+FW_NAME_SIZE] = ((pConfiguration->mnSamplingRate&0xff000000)>>24);
					strcpy(&p_kBuf[8+FW_NAME_SIZE], pConfiguration->mpDescription);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
						return 0;
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
					return -ENOMEM;
				}				
			}else{
				dev_err(pTAS2555->dev, "read buffer not sufficient\n");
			}
		}
		break;		
		
		case TIAUDIO_CMD_FW_TIMESTAMP:
		{
			if(count == 4){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					p_kBuf[0] = (pTAS2555->mpFirmware->mnTimeStamp&0x000000ff);
					p_kBuf[1] = ((pTAS2555->mpFirmware->mnTimeStamp&0x0000ff00)>>8);
					p_kBuf[2] = ((pTAS2555->mpFirmware->mnTimeStamp&0x00ff0000)>>16);
					p_kBuf[3] = ((pTAS2555->mpFirmware->mnTimeStamp&0xff000000)>>24);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
						return 0;
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
					return -ENOMEM;
				}	
			}
		}
		break;
	}
	 pTAS2555->mnDBGCmd = 0;


	return count;
}

static ssize_t tas2555_file_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)file->private_data;
	int ret = 0;
//	unsigned int value = 0;
	unsigned char *p_kBuf = NULL;
	unsigned int reg = 0;
	unsigned int len = 0;

	p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
	if(p_kBuf == NULL) {
		dev_err(pTAS2555->dev, "write no mem\n");
		return -ENOMEM;
	}
	
	ret = copy_from_user(p_kBuf, buf, count);
	if (0 != ret) {
		dev_err(pTAS2555->dev,"copy_from_user failed.\n");
		return 0;
	}

	pTAS2555->mnDBGCmd = p_kBuf[0];
	switch(pTAS2555->mnDBGCmd)
	{
		case TIAUDIO_CMD_REG_WITE:
		if(count > 5){
			reg = ((unsigned int)p_kBuf[1] << 24) + 
				((unsigned int)p_kBuf[2] << 16) + 
				((unsigned int)p_kBuf[3] << 8) + 
				(unsigned int)p_kBuf[4];
			len = count - 5;
			if(len == 1){
				ret = tas2555_i2c_write(pTAS2555, reg, p_kBuf[5]);
			}else{
				ret = tas2555_i2c_bulk_write(pTAS2555, reg, &p_kBuf[5], len);
			}
		}else{
			dev_err(pTAS2555->dev,"%s, write len fail, count=%d.\n", 
				__FUNCTION__, (int)count);
		}
		pTAS2555->mnDBGCmd = 0;
		break;
		
		case TIAUDIO_CMD_REG_READ:
		if(count == 5){
			pTAS2555->mnCurrentReg = ((unsigned int)p_kBuf[1] << 24) + 
				((unsigned int)p_kBuf[2] << 16) + 
				((unsigned int)p_kBuf[3] << 8) 	+ 
				(unsigned int)p_kBuf[4];		
			if(g_logEnable){
				printk("%s, data[1]=0x%x, data[2]=0x%x, data[3]=0x%x, data[4]=0x%x, whole=0x%x\n", 
					__FUNCTION__, p_kBuf[1], p_kBuf[2], p_kBuf[3],p_kBuf[4], 
					pTAS2555->mnCurrentReg);
			}	
		}else{
			dev_err(pTAS2555->dev,"read len fail.\n");
		}			
		break;
		
		case TIAUDIO_CMD_DEBUG_ON:
		{
			if(count == 2){
				g_logEnable = p_kBuf[1];
			}
			pTAS2555->mnDBGCmd = 0;
		}
		break;
		
		case TIAUDIO_CMD_PROGRAM:
		{
			if(count == 2){
				tas2555_program_put(pTAS2555, p_kBuf[1]);
				pTAS2555->mnDBGCmd = 0;
			}
		}
		break;
		
		case TIAUDIO_CMD_CONFIGURATION:
		{
			if(count == 2){
				tas2555_set_config(pTAS2555, p_kBuf[1]);
				pTAS2555->mnDBGCmd = 0;
			}
		}
		break;	
		
		case TIAUDIO_CMD_FW_TIMESTAMP:
		/*let go*/
		break;
		
		default:
			pTAS2555->mnDBGCmd = 0;
		break;
	}
	
	kfree(p_kBuf);

	return count;
}

static long tas2555_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tas2555_priv *pTAS2555 = file->private_data;
	//void __user *user_arg = (void __user *)arg;
	int ret = 0;
	
	switch (cmd) {
		case SMARTPA_SPK_DAC_VOLUME:
		{
			u8 volume = (arg & 0x0f);
			tas2555_i2c_update_bits(pTAS2555, 
				TAS2555_SPK_CTRL_REG, 0x78, volume<<3);
		}		
		break;
	
		case  ENABLE_MI2S_CLK:
		{
			if ((1 ==  arg))  {
			msm8x16_quin_mi2s_clk_ctl(true);
			msleep(5);	  
			 tas2555_set_config(pTAS2555, 0);	
			tas2555_enable(pTAS2555, true);	          		
			}  else if ((2 ==  arg)) {
			msm8x16_quin_mi2s_clk_ctl(true);
			msleep(5);	
			tas2555_set_config(pTAS2555, 1);
           		 tas2555_enable(pTAS2555, true);	          		 							
			} else {
			tas2555_enable(pTAS2555, false);
			msleep(5);
			msm8x16_quin_mi2s_clk_ctl(false);	
			}
		}		
		break;
		
		case SMARTPA_SPK_SWITCH_PROGRAM:
		{
			tas2555_program_put(pTAS2555, arg);
		}
		break;
		
		case SMARTPA_SPK_SWITCH_CONFIGURATION:
		{
			tas2555_set_config(pTAS2555, arg);
		}
		break;
	}
	
	return ret;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = tas2555_file_read,
	.write = tas2555_file_write,
	.unlocked_ioctl = tas2555_file_unlocked_ioctl,
	.open = tas2555_file_open,
	.release = tas2555_file_release,
};

#define MODULE_NAME	"i2c_smartpa"
static struct miscdevice tas2555_misc =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODULE_NAME,
	.fops = &fops,
};

static int tas2555_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2555_priv *pTAS2555;
	unsigned int n;
	int nResult;

	pTAS2555 = devm_kzalloc(&pClient->dev, sizeof(*pTAS2555), GFP_KERNEL);
	if (!pTAS2555)
		return -ENOMEM;
		
	pTAS2555->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2555);

	dev_set_drvdata(&pClient->dev, pTAS2555);

	pTAS2555->reset_gpio = of_get_named_gpio(pClient->dev.of_node, "ti,reset-gpio", 0);	
	if (gpio_is_valid(pTAS2555->reset_gpio)) {
		gpio_free(pTAS2555->reset_gpio)	;
		dev_info(pTAS2555->dev, "%s, reset gpio is (%d)\n", __FUNCTION__,pTAS2555->reset_gpio);
		gpio_request(pTAS2555->reset_gpio, "TAS2555_RST");
		gpio_direction_output(pTAS2555->reset_gpio,1);
		gpio_set_value(pTAS2555->reset_gpio, 0);
		//value  =  gpio_get_value(pTAS2555->reset_gpio);
		//dev_info(pTAS2555->dev, "shone gpio50  value is %d\n",pTAS2555->reset_gpio);
		msleep(10);
		gpio_set_value(pTAS2555->reset_gpio, 1);
		msleep(1);
	}else{
		nResult = -1;
		dev_err(pTAS2555->dev, "func(%s), line(%d), reset gpio is invalid (%d)\n", __FUNCTION__, __LINE__,pTAS2555->reset_gpio);
		return nResult;
	}
	  
	pTAS2555->mpRegmap = devm_regmap_init_i2c(pClient, &tas2555_i2c_regmap);
	if (IS_ERR(pTAS2555->mpRegmap)) {
		nResult = PTR_ERR(pTAS2555->mpRegmap);
		dev_err(&pClient->dev, "Failed to allocate register map: %d\n",
			nResult);
		return nResult;
	}

	/* Reset the chip */
	regmap_write(pTAS2555->mpRegmap, TAS2555_SW_RESET_REG, 0x01);
	udelay(1000);

	pTAS2555->mpFirmware =
		devm_kzalloc(&pClient->dev, sizeof(*(pTAS2555->mpFirmware)),
		GFP_KERNEL);
	if (!pTAS2555->mpFirmware)
		return -ENOMEM;

	pTAS2555->mpCalFirmware =
		devm_kzalloc(&pClient->dev, sizeof(*(pTAS2555->mpCalFirmware)),
		GFP_KERNEL);
	if (!pTAS2555->mpCalFirmware)
		return -ENOMEM;

	pTAS2555->mnCurrentPage = 0;
	pTAS2555->mnCurrentBook = 0;

	nResult = tas2555_i2c_read(pTAS2555, TAS2555_REV_PGID_REG, &n);
	dev_info(&pClient->dev, "TAS2555 PGID: 0x%02x\n", n);

	tas2555_i2c_load_data(pTAS2555, p_tas2555_default_data);

	nResult = request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
		&pClient->dev, GFP_KERNEL, pTAS2555, tas2555_i2c_fw_ready);

	pTAS2555->mbTILoadActive = false;

#ifdef ENABLE_TILOAD
	pTAS2555->read = tas2555_i2c_read;
	pTAS2555->write = tas2555_i2c_write;
	pTAS2555->bulk_read = tas2555_i2c_bulk_read;
	pTAS2555->bulk_write = tas2555_i2c_bulk_write;
	pTAS2555->set_config = tas2555_set_config;
	tiload_driver_init(pTAS2555);
#endif

	g_tas2555 = pTAS2555;

	nResult = misc_register(&tas2555_misc);
	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 misc fail: %d\n", nResult);
		return nResult;
	}	
	
	dev_info(pTAS2555->dev, "%s, success\n", __FUNCTION__);
    return nResult;
}

static int tas2555_i2c_remove(struct i2c_client *pClient)
{
	misc_deregister(&tas2555_misc);
	return 0;
}

static const struct i2c_device_id tas2555_i2c_id[] = {
	{"tas2555", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tas2555_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2555_of_match[] = {
	{.compatible = "ti,tas2555"},
	{},
};

MODULE_DEVICE_TABLE(of, tas2555_of_match);
#endif

static struct i2c_driver tas2555_i2c_driver = {
	.driver = {
			.name = "tas2555",
			.owner = THIS_MODULE,
#if defined(CONFIG_OF)
			.of_match_table = of_match_ptr(tas2555_of_match),
#endif
		},
	.probe = tas2555_i2c_probe,
	.remove = tas2555_i2c_remove,
	.id_table = tas2555_i2c_id,
};

module_i2c_driver(tas2555_i2c_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TAS2555 Smart Amplifier driver");
MODULE_LICENSE("GPLv2");
