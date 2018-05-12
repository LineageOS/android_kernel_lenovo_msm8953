/*! \file iqs263.c
 * \brief  iqs263 Driver
 *
 * Driver for the iqs263
 * Copyright (c) 2016 Longcheer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#define DRIVER_NAME "IQS263"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "iqs263.h" /* main struct, interrupt,init,pointers */

#define IDLE 0
#define ACTIVE 1

#define INVALID_GPIO        (-1)

/*power supply VDD 1.8V, VIO 1.8 */
//#define IQS263_VDD_MIN_UV       3300000
//#define IQS263_VDD_MAX_UV       3300000
#define IQS263_VDD_MIN_UV       1800000
#define IQS263_VDD_MAX_UV       1800000
#define IQS263_VIO_MIN_UV       1800000
#define IQS263_VIO_MAX_UV       1800000

static int init_try_count = 10;
static int iqs263_get_nirq_state(void);
//static int iqs263_get_nmov_state(void);
static int iqs263_power_ctl(struct iqs263_chip *iqs263, bool on);
static void iqs263_delay_work(struct work_struct *work);

static struct IQS263_platform_data *PIQS263_HW = NULL;

static irqreturn_t iqs263_interrupt_thread(int irq, void *data)
{
	struct iqs263_chip *iqs263_chip = NULL;
	iqs263_chip = data;
	//mutex_lock(&iqs263_chip->mutex);
	disable_irq_nosync(iqs263_chip->irq);
	//dev_info(iqs263_chip->pdev, "iqs263 irq\n");
	if (iqs263_chip->hw->get_is_nirq_low()) {
		cancel_delayed_work(&iqs263_chip->dworker);
		schedule_delayed_work(&iqs263_chip->dworker,msecs_to_jiffies(0));
	//dev_info(iqs263_chip->pdev,"Schedule Irq timer");
	}
	else {
		dev_err(iqs263_chip->pdev, "iqs263_irq - nirq read high\n");
		enable_irq(iqs263_chip->irq);
	}
	//mutex_unlock(&iqs263_chip->mutex);

	return IRQ_HANDLED;
}

static int iqs263_i2c_write(struct iqs263_chip *chip, uint8_t regaddr, uint8_t txbyte, uint8_t *data)
{
	struct iqs263_chip *iqs263 = chip;
	struct i2c_client *client = NULL;
	uint8_t buffer[8];
	int i, ret = -ENOMEM;;

	if (iqs263 && iqs263->pdev) {
		client = to_i2c_client(iqs263->pdev);

		mutex_lock(&iqs263->mutex);

		buffer[0] = regaddr;

		for(i=0; i< txbyte; i++) {
			buffer[i+1] = data[i];
			//dev_info(iqs263->pdev, "0x%x, ",buffer[i+1]);
		}

		ret = i2c_master_send(client, buffer, txbyte+1);
		//dev_info(iqs263->pdev,"iqs263_i2c_write Address: 0x%x Return: %d\n", regaddr, ret);

		mutex_unlock(&iqs263->mutex);
	}

	if (ret < 0) {
		dev_info( iqs263->pdev, "iqs263 i2c write %x error %d\n", regaddr, ret);
		return ret;
	}

	return ret;
}

#if 0
static int iqs263_i2c_read(struct iqs263_chip *chip, uint8_t regaddr, uint8_t rxbyte, uint8_t *data)
{
	struct iqs263_chip *iqs263 = chip;
	struct i2c_client *client = NULL;
	int ret = 0;
	struct i2c_msg msg[2];

	memset(&msg, 0, sizeof(struct i2c_msg));

	if (iqs263 && data && iqs263->pdev) {
		client = to_i2c_client(iqs263->pdev);

		mutex_lock(&iqs263->mutex);
#if 0
		dev_dbg(iqs263->pdev, "inside iqs263_i2c_read()\n");
		dev_dbg(iqs263->pdev,
			"going to call i2c_master_send i2c_client =0x%p, Reg: 0x%x\n",
			(void *)client, regaddr);
		ret = i2c_master_send(client, &regaddr, 1);
		if (ret >= 0) {
			dev_dbg(iqs263->pdev, "going to call i2c_master_recv i2c_client =0x%p, data =0x%p, rxbyte=%x\n",
				(void *)client, (void *)data, rxbyte);
			ret = i2c_master_recv(client, data, rxbyte);
	}
#endif

		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &regaddr;
		msg[1].addr = client->addr;
		msg[1].flags = 0;
		msg[1].len = rxbyte;
		msg[1].buf = data;

		ret = i2c_transfer(client->adapter, msg, 2);

		dev_info(iqs263->pdev, "iqs263_i2c_read Address: 0x%x Return: 0x%x with data[0]= 0x%x\n",regaddr, ret, data[0]);

		mutex_unlock(&iqs263->mutex);
	}

	if (ret != 2) {
		dev_info( iqs263->pdev, "iqs263_i2c_read ret=%d\n", ret);
	}
	return ret;
}
#else
static int read_register(struct iqs263_chip *chip, u8 address, u8 *value)
{
	struct iqs263_chip *iqs263 = chip;
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (iqs263 && value) {
	i2c = to_i2c_client(iqs263->pdev);
	returnValue = i2c_smbus_read_word_data(i2c,address);
	dev_info(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
	if (returnValue >= 0) {
		*value = returnValue;
		return 0;
	} else {
		return returnValue;
	}
	}
	return -ENOMEM;
}
#endif

static void touchProcess(struct iqs263_chip *chip)
{
	int counter = 0;
	u8 reg_value = 0;
	int numberOfButtons = 0;
	struct iqs263_chip *iqs263 = chip;
	struct iqs263_button_plat *buttons = NULL;
	struct input_dev *input = NULL;

	struct iqs263_button_plat *pCurrentButton  = NULL;

	if (iqs263) {
	dev_dbg(iqs263->pdev, "Inside touchProcess()\n");
	read_register(iqs263, TOUCH_STAT, &reg_value);

	buttons = PIQS263_HW->pbuttonInformation->buttons;
	input = PIQS263_HW->pbuttonInformation->input;
	numberOfButtons = PIQS263_HW->pbuttonInformation->buttonSize;

	if (unlikely( (buttons==NULL) || (input==NULL) )) {
		dev_err(iqs263->pdev, "ERROR!! buttons or input NULL!!!\n");
		return;
	}

	for (counter = 0; counter < numberOfButtons; counter++) {
		pCurrentButton = &buttons[counter];
		if (pCurrentButton==NULL) {
			dev_err(iqs263->pdev,"ERROR!! current button at index: %d NULL!!!\n",
				counter);
			return; // ERRORR!!!!
		}

		switch (pCurrentButton->state) {
			case IDLE: /* Button is not being touched! */
				if (((reg_value & pCurrentButton->mask) == pCurrentButton->mask)) {
					/* User pressed button */
					dev_info(iqs263->pdev, "cap button %d touched\n", counter);
					input_report_key(input, pCurrentButton->keycode, 1);
					pCurrentButton->state = ACTIVE;
				} else {
					dev_dbg(iqs263->pdev, "Button %d already released.\n",counter);
				}
				break;
			case ACTIVE: /* Button is being touched! */
				if (((reg_value & pCurrentButton->mask) != pCurrentButton->mask)) {
					/* User released button */
					dev_info(iqs263->pdev, "cap button %d released\n",counter);
					input_report_key(input, pCurrentButton->keycode, 0);
					pCurrentButton->state = IDLE;
				} else {
					dev_dbg(iqs263->pdev, "Button %d still touched.\n",counter);
				}
				break;
			default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
				break;
		};
	}
		input_sync(input);

		dev_dbg(iqs263->pdev, "Leaving touchProcess()\n");
	}
}

static int captouch_init_step = 0;
static bool captouch_init_flag = false;
static void iqs263_delay_work(struct work_struct *work)
{
	struct iqs263_chip *iqs263 = NULL;
	int err=0;
	//u8 reg_value = 0;
	uint8_t buffer[8]={0};

	iqs263 = container_of((struct work_struct *)work, struct iqs263_chip, dworker.work);

	if(!captouch_init_flag) {
		pr_info("iqs263 captouch sensor controller init captouch_init_step=%d \n", captouch_init_step);

		if (0 == captouch_init_step) {
			buffer[0] = 0x07;		//0x02;
			err = iqs263_i2c_write(iqs263, 0x0d, 1, buffer);
			if (err >= 0) {
				captouch_init_step++;
			} else
				init_try_count--;
		} else if (1 == captouch_init_step) {
			buffer[0] = 0x0c;		//0x04
			buffer[1] = 0x02;		//cs0 2.4G
			buffer[2] = 0x03;		//cs1 5.0G
			buffer[3] = 0x14;		//cs2 main
			buffer[4] = 0x0c;		//0x03;
			buffer[5] = 0x00;
			buffer[6] = 0x0F;		//0xff;
			buffer[7] = 0x65;		//0x04;

			err = iqs263_i2c_write(iqs263, 0x0a, 7, buffer);

			if (err >= 0) {
				captouch_init_step++;
			}else
				init_try_count--;
		} else if (2 == captouch_init_step) {
			buffer[0] = 0x00;
			buffer[1] = 0xC8;		//0xDF;
			buffer[2] = 0xC8;		//0xDD;

			err = iqs263_i2c_write(iqs263, 0x0b, 3, buffer);
			if (err >= 0) {
				captouch_init_step++;
			}else
				init_try_count--;
		} else if (3 == captouch_init_step) {
			buffer[0] = 0x3C;		//0x3f;
			buffer[1] = 0x2D;		//0x2a;
			buffer[2] = 0x2F;		//0x37;
			buffer[3] = 0x00;
			buffer[4] = 0x11;		//0x14;

			err = iqs263_i2c_write(iqs263, 0x07, 5, buffer);

			if (err >= 0) {
				captouch_init_step++;
			}else
				init_try_count--;
		} else if (4 == captouch_init_step) {
			buffer[0] = 0x12;		//0x10;
			buffer[1] = 0xd3;		//0x41;
			buffer[2] = 0x14;		//0x00;
			buffer[3] = 0x87;		//0x00;
			buffer[4] = 0x02;

			err = iqs263_i2c_write(iqs263, 0x09, 5, buffer);

			if (err >= 0) {
				captouch_init_step++;
				captouch_init_flag = true;
			}else
				init_try_count--;
		}
	}else if ((5 == captouch_init_step) || captouch_init_flag) {

		//err = read_register(iqs263, 0x03, &reg_value);
		//pr_info("iqs263_i2c_read reg_value=%x \n", reg_value);

		//err = iqs263_i2c_read(iqs263, 0x03, 1, buffer);
		//pr_info("iqs263_i2c_read buffer[0]=%x \n", buffer[0]);
		touchProcess(iqs263);
#if 0
		if (err >= 0) {
			if (buffer[0] & 0x02) {
				captouch_eint_status = 1;
			} else {
				captouch_eint_status = 0;
			}
		}
#endif
	}
	if ((err >= 0) || (init_try_count > 0))
		enable_irq(iqs263->irq);
}

static unsigned int iqs263_enable = 1;
static ssize_t iqs263_enable_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct iqs263_chip *iqs263 = dev_get_drvdata(dev);

	dev_info(iqs263->pdev, "Reading iqs263 enable status.\n");
	return sprintf(buf, "%u\n", iqs263_enable);
}

static ssize_t iqs263_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iqs263_chip *iqs263 = dev_get_drvdata(dev);
	unsigned int val; int err = 0;

	if (count >= sizeof(iqs263_enable))
		return -EINVAL;

	if (sscanf(buf, "%u", &val) != 1)
		return -EINVAL;

	if (val) {
		if (!iqs263_enable && iqs263) {
		dev_info( iqs263->pdev, "going to enable iqs263.\n");
			err = iqs263_power_ctl(iqs263, true);
			if (err) {
					dev_err(iqs263->pdev, "Failed to enable Capacitive Touch Controller power\n");
			}

			enable_irq(iqs263->irq);
			captouch_init_step=0;
		} else {
			if(iqs263)
				dev_info( iqs263->pdev, "iqs263 has already enabled.\n");
		}
	} else {
		if(iqs263_enable && iqs263) {
			dev_info( iqs263->pdev, "going to disable iqs263.\n");
			disable_irq(iqs263->irq);
			captouch_init_flag = false;
			err = iqs263_power_ctl(iqs263, false);
			if (err) {
				dev_err(iqs263->pdev, "Failed to disable Capacitive Touch Controller power\n");
			}
		} else {
			if(iqs263)
				dev_info( iqs263->pdev, "iqs263 has already disabled.\n");
		}
	}

	iqs263_enable = val;

	return count;
}

static DEVICE_ATTR(enable, 0664, iqs263_enable_show, iqs263_enable_store);

static struct attribute *iqs263_attributes[] = {
	&dev_attr_enable.attr,
	NULL,
};
static struct attribute_group iqs263_attr_group = {
	.attrs = iqs263_attributes,
};

static int iqs263_get_nirq_state(void)
{
	int value;

	if (!gpio_is_valid(PIQS263_HW->irq_gpio)){
		pr_err("iqs263 irq_gpio was not assigned properly");
	}
	value = gpio_get_value(PIQS263_HW->irq_gpio);
	pr_debug("iqs263 irq gpio status(%d)" , value);
	return !value;
}
#if 0
static int iqs263_get_nmov_state(void)
{
	int value;

	if (!gpio_is_valid(PIQS263_HW->mov_gpio)){
		pr_err("iqs263 mov_gpio was not assigned properly");
	}
	value = gpio_get_value(PIQS263_HW->mov_gpio);
	pr_err("iqs263 mov gpio status(%d)" , value);
	return !value;
}
#endif
static int iqs263_power_ctl(struct iqs263_chip *iqs263, bool on)
{
	int ret = 0;
	int err = 0;

	if (!on && iqs263->power_enabled) {
		ret = regulator_disable(iqs263->vdd);
		if (ret) {
			pr_err("Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(iqs263->vio);
		if (ret) {
			pr_err("Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(iqs263->vdd);
			return ret;
		}
		iqs263->power_enabled = on;
	} else if (on && !iqs263->power_enabled) {
		ret = regulator_enable(iqs263->vdd);
		if (ret) {
			pr_err("Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		msleep(8);////>=5ms OK.
		ret = regulator_enable(iqs263->vio);
		if (ret) {
			pr_err("Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(iqs263->vdd);
			return ret;
		}
		msleep(10); // wait 10ms
		iqs263->power_enabled = on;
	} else {
		pr_info("Power on=%d. enabled=%d\n", on, iqs263->power_enabled);
	}

	return ret;
}

static int iqs263_power_init(struct iqs263_chip *iqs263)
{
	int ret;
	struct i2c_client *client = to_i2c_client(iqs263->pdev);

	iqs263->vdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(iqs263->vdd)) {
		ret = PTR_ERR(iqs263->vdd);
		pr_err("Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(iqs263->vdd) > 0) {
		ret = regulator_set_voltage(iqs263->vdd,
				IQS263_VDD_MIN_UV,
				IQS263_VDD_MAX_UV);
		if (ret) {
			pr_err("Regulator set failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}

	iqs263->vio = regulator_get(&client->dev, "vio");
	if (IS_ERR(iqs263->vio)) {
		ret = PTR_ERR(iqs263->vio);
		pr_err("Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(iqs263->vio) > 0) {
		ret = regulator_set_voltage(iqs263->vio,
				IQS263_VIO_MIN_UV,
				IQS263_VIO_MAX_UV);
		if (ret) {
			pr_err("Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(iqs263->vio);
reg_vdd_set:
	if (regulator_count_voltages(iqs263->vdd) > 0)
		regulator_set_voltage(iqs263->vdd, 0, IQS263_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(iqs263->vdd);
	return ret;
}

static int iqs263_parse_dt(struct device *dev,
			iqs263_platform_data_t *pdata)
{
	//struct device_node *np = dev->of_node;
	//struct i2c_client *client = to_i2c_client(dev);

	pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
				"azopteq,irq-gpio", 0, NULL);

	if (pdata->irq_gpio < 0) {
		pr_debug("iqs263 irq gpio is not available\n");
		return -EINVAL;
	}

	//pdata->mov_gpio = of_get_named_gpio_flags(dev->of_node,
				//"azopteq,mov-gpio", 0, NULL);

	//if (pdata->mov_gpio < 0)
			//pr_debug("iqs263 mov gpio is not available\n");
	return 0;
}

static int iqs263_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0, err = 0;
	struct iqs263_chip *iqs263_chip;
	piqs263_platform_data_t pplatData = 0;
	struct input_dev *input = NULL;

	dev_info(&client->dev, "iqs263_probe()\n");
	dev_info(&client->dev, "IQS263 I2C Address: 0x%02x\n", client->addr);

	/* (1) allocation memory for piqs263_platform_data */
	PIQS263_HW = pplatData = kzalloc(sizeof(iqs263_platform_data_t), GFP_KERNEL);
	if (!pplatData) {
		dev_err(&client->dev, "memory alocation was failed");
		err = -ENOMEM;
		return err;
	}

	/* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
	pplatData->get_is_nirq_low = iqs263_get_nirq_state;

	//pplatData->get_is_nmov_low = iqs263_get_nmov_state;

	pplatData->irq_gpio = INVALID_GPIO;

	//pplatData->mov_gpio = INVALID_GPIO;

	pplatData->pbuttonInformation = &IQS263ButtonInformation;

	if (client->dev.of_node) {
		err = iqs263_parse_dt(&client->dev, pplatData);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto error_0;
		}
		dev_info(&client->dev, "iqs263 use gpio interrupt is %d\n", pplatData->irq_gpio);
		//dev_info(&client->dev, "iqs263 use movement gpio is %d\n", pplatData->mov_gpio);
	} else {
		pplatData = client->dev.platform_data;
		dev_err(&client->dev, "Use platform data\n");
		if (!pplatData) {
			dev_err(&client->dev, "platform data is required!\n");
			return -EINVAL;
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality was failed");
		err = -ENODEV;
		goto error_0;
	}

	if (gpio_is_valid(pplatData->irq_gpio)) {
		/* iqs263 interrupt gpio request */
		err = gpio_request(pplatData->irq_gpio, "iqs263_NIRQ");
		if (err < 0) {
			dev_err(&client->dev, "gpio req failed for iqs263 interrupt\n");
			pplatData->irq_gpio = INVALID_GPIO;
			goto error_0;
		}
		err = gpio_direction_input(pplatData->irq_gpio);
		if (err) {
			dev_err(&client->dev,"unable to set direction for rdy gpio " \
					"[%d]\n", pplatData->irq_gpio);
			goto error_0;
		}
		//gpio_free(pplatData->irq_gpio);
	}
#if 0
	if (gpio_is_valid(pplatData->mov_gpio)) {
		/* iqs263 interrupt gpio request */
		err = gpio_request(pplatData->mov_gpio, "iqs263_NMOV");
		if (err < 0) {
			dev_err(&client->dev, "gpio req failed for iqs263 movement\n");
			pplatData->mov_gpio = INVALID_GPIO;
			goto error_0;
		}
		err = gpio_direction_input(pplatData->mov_gpio);
		if (err) {
			dev_err(&client->dev,"unable to set direction for movement gpio " \
					"[%d]\n", pplatData->mov_gpio);
			goto error_0;
		}
		//gpio_free(pplatData->mov_gpio);
	}
#endif
	iqs263_chip = kzalloc(sizeof(struct iqs263_chip), GFP_KERNEL); /* create memory for main struct */
	dev_info(&client->dev, "\t Initialized Main Memory: 0x%p\n",iqs263_chip);

	if (iqs263_chip) {
		iqs263_chip->hw = pplatData;
		/* save irq in case we need to reference it */
		//iqs263_chip->irq = client->irq;
		iqs263_chip->irq = client->irq = gpio_to_irq(pplatData->irq_gpio);
		dev_info(&client->dev, "iqs263 gpio interrupt id is %d\n", iqs263_chip->irq);

		INIT_DELAYED_WORK(&iqs263_chip->dworker, iqs263_delay_work);

		/* initialize mutex */
		mutex_init(&iqs263_chip->mutex);

		/* setup i2c communication */
		i2c_set_clientdata(client, iqs263_chip);

			/* record device struct */
		iqs263_chip->pdev = &client->dev;

		err = sysfs_create_group(&client->dev.kobj, &iqs263_attr_group);
		if (err) {
			dev_err(&client->dev, "failed to register sysfs. err: %d\n", err);
			goto error_1;
		}

		//err = iqs263_power_init(client, pplatData);
		err = iqs263_power_init(iqs263_chip);
		if (err) {
			dev_err(&client->dev, "Failed to get Capacitive Touch Controller regulators\n");
			err = -EINVAL;
			goto error_1;
		}

		//err = iqs263_power_ctl(pplatData, true);
		err = iqs263_power_ctl(iqs263_chip, true);
		if (err) {
			dev_err(&client->dev, "Failed to enable Capacitive Touch Controller power\n");
			err = -EINVAL;
			goto error_1;
		}

		/* Create the input device */
		input = input_allocate_device();
		if (!input) {
			err = -ENOMEM;
			goto error_1;
		}

		/* Set all the keycodes */
		__set_bit(EV_KEY, input->evbit);
		for (i = 0; i < pplatData->pbuttonInformation->buttonSize; i++) {
			__set_bit(pplatData->pbuttonInformation->buttons[i].keycode,
							input->keybit);
			pplatData->pbuttonInformation->buttons[i].state = IDLE;
		}
		/* save the input pointer and finish initialization */
		pplatData->pbuttonInformation->input = input;
		input->name = "iqs263 Cap Touch";
		input->id.bustype = BUS_I2C;
		if(input_register_device(input)) {
			err= -ENOMEM;
			goto error_1;
		}

		err = request_threaded_irq(iqs263_chip->irq, NULL, iqs263_interrupt_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, iqs263_chip->pdev->driver->name,
			iqs263_chip);
		if (err) {
			dev_err(&client->dev, "irq request failed %d, error %d\n", iqs263_chip->irq, err);
			goto error_1;
		} else
			dev_info(&client->dev, "registered with threaded irq (%d)\n", iqs263_chip->irq);
		//disable_irq(iqs263_chip->irq);
	}

	return  0;

error_1:
	kfree(iqs263_chip);
error_0:
	PIQS263_HW = NULL;
	kfree(pplatData);
	return err;
}

static int iqs263_remove(struct i2c_client *client)
{
	piqs263_platform_data_t pplatData =0;
	struct iqs263_chip *iqs263_chip = i2c_get_clientdata(client);

	if (iqs263_chip) {
		pplatData = iqs263_chip->hw;
		input_unregister_device(pplatData->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &iqs263_attr_group);

			kfree(pplatData);

		cancel_delayed_work_sync(&iqs263_chip->dworker); /* Cancel the Worker Func */
		/*destroy_workqueue(iqs263_chip->workq); */

		free_irq(iqs263_chip->irq, iqs263_chip);

		kfree(iqs263_chip);
			return 0;
	}
	return -ENOMEM;
}

/*====================================================*/
/***** Kernel Suspend *****/
static int iqs263_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct iqs263_chip *iqs263_chip = i2c_get_clientdata(client);
	if(iqs263_enable && iqs263_chip)
		disable_irq(iqs263_chip->irq);
	pr_info("%s\n", __func__);
	return 0;
}

/***** Kernel Resume *****/
static int iqs263_resume(struct i2c_client *client)
{
	struct iqs263_chip *iqs263_chip = i2c_get_clientdata(client);
	if(iqs263_enable && iqs263_chip) {
		enable_irq(iqs263_chip->irq);
	}
	pr_info("%s\n", __func__);
	return 0;
}

/*====================================================*/
static struct i2c_device_id iqs263_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, iqs263_idtable);

static const struct of_device_id iqs263_of_match[] = {
	{ .compatible = "azopteq,iqs263", },
	{ },
};

static struct i2c_driver iqs263_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME,
		.of_match_table = iqs263_of_match,
	},
	.id_table = iqs263_idtable,
	.probe	  = iqs263_probe,
	.remove	  = iqs263_remove,
//#if defined(USE_KERNEL_SUSPEND)
	.suspend  = iqs263_suspend,
	.resume   = iqs263_resume,
//#endif
};

static int __init iqs263_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&iqs263_driver);
}
static void __exit iqs263_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&iqs263_driver);
}

module_init(iqs263_init);
module_exit(iqs263_exit);

MODULE_AUTHOR("Xinlin Fu");
MODULE_DESCRIPTION("iqs263 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
