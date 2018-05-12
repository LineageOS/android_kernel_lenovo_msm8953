/* Himax Android Driver Sample Code for Himax chipset
*
* Copyright (C) 2014 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#ifndef HIMAX_PLATFORM_H
#define HIMAX_PLATFORM_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>

//#if defined(CONFIG_HMX_DB)
#include <linux/regulator/consumer.h>
//#endif

#define QCT
//#define MTK

#ifdef MTK
#define HX_MTK_DMA                             // Support the MTK DMA Function.           Default is open.
#endif
#ifdef MTK
#include <linux/kthread.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>

#include <mach/board.h>
#include <mach/eint.h>

#include <cust_eint.h>
#include "cust_gpio_usage.h"
#include <linux/dma-mapping.h>
#include "tpd.h"
#define TPD_I2C_NUMBER				2
#endif

#if defined(CONFIG_HMX_DB)
/* Analog voltage @2.7 V */
#define HX_VTG_MIN_UV			2700000
#define HX_VTG_MAX_UV			3300000
#define HX_ACTIVE_LOAD_UA		15000
#define HX_LPM_LOAD_UA 			10
/* Digital voltage @1.8 V */
#define HX_VTG_DIG_MIN_UV		1800000
#define HX_VTG_DIG_MAX_UV		1800000
#define HX_ACTIVE_LOAD_DIG_UA	10000
#define HX_LPM_LOAD_DIG_UA 		10

#define HX_I2C_VTG_MIN_UV		1800000
#define HX_I2C_VTG_MAX_UV		1800000
#define HX_I2C_LOAD_UA 			10000
#define HX_I2C_LPM_LOAD_UA 		10
#else
#define HX_I2C_VTG_MIN_UV		1800000
#define HX_I2C_VTG_MAX_UV		1800000
#define HX_I2C_LOAD_UA 			10000

#endif

struct himax_i2c_platform_data {	
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	uint8_t fw_version;
	uint8_t tw_id;
	uint8_t powerOff3V3;
	uint8_t cable_config[2];
	uint8_t protocol_type;
	int gpio_irq;
	int gpio_reset;
	int gpio_3v3_en;
	int (*power)(int on);
	void (*reset)(void);
	struct himax_virtual_key *virtual_key;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;

	struct himax_config *hx_config;
	int hx_config_size;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;	
#if defined(CONFIG_HMX_DB)
	bool	i2c_pull_up;
	bool	digital_pwr_regulator;
	int reset_gpio;
	u32 reset_gpio_flags;
	int irq_gpio;
	u32 irq_gpio_flags;


	struct regulator *vcc_ana; //For Dragon Board
	struct regulator *vcc_dig; //For Dragon Board
	struct regulator *vcc_i2c; //For Dragon Board
#else	
	struct regulator *vcc_i2c;
#endif

};


extern int irq_enable_count;
extern int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry);
extern int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry);
extern int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry);
extern int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry);
extern int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry);
extern void himax_int_enable(int irqnum, int enable);
extern int himax_ts_register_interrupt(struct i2c_client *client);
extern void himax_rst_gpio_set(int pinnum, uint8_t value);
extern uint8_t himax_int_gpio_read(int pinnum);

extern int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata);

#endif
