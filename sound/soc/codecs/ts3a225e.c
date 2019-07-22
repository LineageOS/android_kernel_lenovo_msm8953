/* drivers/hwmon/mt6516/amit/IQS128.c - IQS128/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include "ts3a225e.h"

/******************************************************************************
 * configuration
*******************************************************************************/
#define TS3A225E_DEV_NAME     "TS3A225E"

static struct i2c_client *ts3a225e_i2c_client = NULL;
static const struct i2c_device_id ts3a225e_i2c_id[] = {{"TS3A225E",0},{}};

/* LCSH MOD for fix headset detect fail at first time when system boot @duanlongfei 20170317 Start */
/**
 * Used different VDDCX voltage values
 */
enum ts3a225e_vdd_value {
	VDD_NONE = 0,
	VDD_MIN,
	VDD_MAX,
	VDD_VAL_MAX,
};

static int ts3a225e_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	uint8_t devicve_id[1];
	struct regulator *ts3a225e_vdd;
	int ret = 0;
	int len;
	u32 tmp[3];
	int vdd_val[VDD_VAL_MAX];

    printk("%s:enter \n", __func__);

    ts3a225e_vdd = devm_regulator_get(&(client->dev),"ts3a225e_vdd");
    if (IS_ERR(ts3a225e_vdd)) {
        printk("%s:unable to get ts3a225e vdd\n", __func__);
    }

    if (client->dev.of_node) {
        of_get_property(client->dev.of_node,
    					"ts3a225e,vdd-voltage-level",
    					&len);
        if (len == sizeof(tmp)) {
            of_property_read_u32_array(client->dev.of_node,
                    "ts3a225e,vdd-voltage-level",
                    tmp, len/sizeof(*tmp));
            vdd_val[VDD_NONE] = tmp[0];
            vdd_val[VDD_MIN] = tmp[1];
            vdd_val[VDD_MAX] = tmp[2];
        } else {
            printk("%s:fail vdd voltage level\n", __func__);
        }
    } else {
        printk("%s:device tree not enabled\n", __func__);
    }

    ret = regulator_set_voltage(ts3a225e_vdd, vdd_val[VDD_MIN], vdd_val[VDD_MAX]);
    if (ret) {
        printk("%s:unable to set the voltage for ts3a225e vdd\n", __func__);
    }

    ret = regulator_enable(ts3a225e_vdd);
    if (ret) {
        printk("%s:unable to enable ts3a225e vdd\n", __func__);
    }

	ts3a225e_i2c_client = client;

	ts3a225e_read_byte(0x01, &devicve_id[0]);
	printk("ts3a225e_i2c_probe ID=%x \n", devicve_id[0]);
    ts3a225e_write_byte(0x02, 0x07);
    ts3a225e_read_byte(0x02, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 02=%x \n", devicve_id[0]);
    ts3a225e_write_byte(0x03, 0x01);
    ts3a225e_read_byte(0x03, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 03=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x04, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 04=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x05, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 05=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x06, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 06=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x07, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 07=%x \n", devicve_id[0]);

	return 0;
}
/* LCSH MOD for fix headset detect fail at first time when system boot @duanlongfei 20170317 End */

static DEFINE_MUTEX(ts3a225e_i2c_access);

int ts3a225e_read_byte(uint8_t reg_addr, uint8_t *data)
{
    struct i2c_client *client = ts3a225e_i2c_client;
    int retry;
    struct i2c_msg msg[] = {
			{
			 .addr = (client != NULL? client->addr : 0) ,
			 .flags = 0,
			 .len = 1,
			 .buf = &reg_addr,
			},

			{
			 .addr = (client != NULL? client->addr : 0),
			 .flags = I2C_M_RD,
			 .len = 1,
			 .buf = data,
			 },
		};
    if (client == NULL) {
        printk("ts3a225e client is null, maybe init fail  \n");
        return -EIO;
    }
    mutex_lock(&ts3a225e_i2c_access);
	//printk("ts3a225e_read_byte i2c_addr = 0x%x \n",msg[0].addr);
    for (retry = 0; retry < 5; retry++) {
    	if (i2c_transfer(client->adapter, msg,
    				ARRAY_SIZE(msg)) > 0)
    		break;
    	else
    		mdelay(1);
    }

    if (5 <= retry) {
    	dev_err(&client->dev, "I2C xfer error");
                 mutex_unlock(&ts3a225e_i2c_access);    
    	return -EIO;
    }
    mutex_unlock(&ts3a225e_i2c_access);    
    return 0;
}

int ts3a225e_write_byte(uint8_t reg_addr, uint8_t data)
{
    u8 buffer[2];
    int retry;
    struct i2c_client *client = ts3a225e_i2c_client;
    struct i2c_msg msg[] = {
    	{
    	 .addr = (client != NULL? client->addr : 0),
    	 .flags = 0,
    	 .len = 2,
    	 .buf = buffer,
    	 },
    };

    if (client == NULL) {
        printk("ts3a225e client is null, maybe init fail  \n");
        return -EIO;
    }
    mutex_lock(&ts3a225e_i2c_access);
    buffer[0] = reg_addr;
    buffer[1] = data;
	printk("ts3a225e_write_byte i2c_addr = 0x%x \n",msg[0].addr);
    for (retry = 0; retry < 5; retry++) {
    	if (i2c_transfer(client->adapter, msg,
    				ARRAY_SIZE(msg)) > 0) {
    		break;
    	} else {
    		mdelay(1);
    	}
    }
    if (5 <= retry) {
        dev_err(&client->dev, "I2C xfer error");
        mutex_unlock(&ts3a225e_i2c_access);
    	 return -EIO;
    }
    mdelay(2);
    mutex_unlock(&ts3a225e_i2c_access);
    return 0;
}

/******************************************************************************
 * extern functions
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id audio_switch_of_match[] = {
	{ .compatible = "hs_switch,ts3a225e", },
	{},
};
#endif
static struct i2c_driver ts3a225e_i2c_driver =
{
    .probe      = ts3a225e_i2c_probe,
    .id_table   = ts3a225e_i2c_id,
    .driver = {
        .name = TS3A225E_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = audio_switch_of_match,
    },
};

int ts3a225e_i2c_mode_init(void)
{
    u8 ts3a225e_reg[7] = {0};
    
    ts3a225e_write_byte(0x04, 0x01);
    msleep(500);
    ts3a225e_read_byte(0x02, &ts3a225e_reg[1]);
    ts3a225e_read_byte(0x03, &ts3a225e_reg[2]);
    ts3a225e_read_byte(0x05, &ts3a225e_reg[4]);
    ts3a225e_read_byte(0x06, &ts3a225e_reg[5]);
    printk("%s: ts3a225e reg 0x02 = 0x%x\n", __func__, ts3a225e_reg[1]);
    printk("%s: ts3a225e reg 0x03 = 0x%x\n", __func__, ts3a225e_reg[2]);
    printk("%s: ts3a225e reg 0x05 = 0x%x\n", __func__, ts3a225e_reg[4]);
    printk("%s: ts3a225e reg 0x06 = 0x%x\n", __func__, ts3a225e_reg[5]);
    if (ts3a225e_reg[5] == 0x01) {
        ts3a225e_write_byte(0x02, 0x07);
        ts3a225e_write_byte(0x03, 0x92);
        //goto exit;
    } else if (ts3a225e_reg[5] == 0x02) {
        if ((ts3a225e_reg[4]&0x07) == 0x07 || (ts3a225e_reg[4]&0x38) == 0x38) {
            printk("%s: ts3a225e is special headset", __func__);
        }
        //goto exit;
    } else {
        printk("%s: ts3a225e detect fail\n", __func__);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init ts3a225e_init(void)
{
	printk("ts3a225e_init \n");
	return i2c_add_driver(&ts3a225e_i2c_driver);
}
/*----------------------------------------------------------------------------*/
static void __exit ts3a225e_exit(void)
{
	printk("ts3a225e_exit \n");
       i2c_del_driver(&ts3a225e_i2c_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ts3a225e_init);
module_exit(ts3a225e_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("TS3A225E driver");
MODULE_LICENSE("GPL");

