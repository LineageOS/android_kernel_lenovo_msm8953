/*
 * STMicroelectronics lsm303agr i2c acc driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>

#include "lsm303agr_core.h"

#define I2C_AUTO_INCREMENT			(0x80)

static int lsm303agr_i2c_read(struct lsm303agr_data *cdata, u8 reg_addr,
			      int len, u8 * data)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (len > 1)
		reg_addr |= I2C_AUTO_INCREMENT;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&cdata->tf->tf_lock);
	err = i2c_transfer(client->adapter, msg, 2);
	mutex_unlock(&cdata->tf->tf_lock);

	return err;
}

static int lsm303agr_i2c_write(struct lsm303agr_data *cdata, u8 reg_addr,
			       int len, u8 * data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	mutex_lock(&cdata->tf->tf_lock);
	err = i2c_transfer(client->adapter, &msg, 1);
	mutex_unlock(&cdata->tf->tf_lock);

	return err;
}

static struct lsm303agr_transfer_function lsm303agr_i2c_tf = {
	.write = lsm303agr_i2c_write,
	.read = lsm303agr_i2c_read,
};

static int lsm303agr_i2c_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	int err;
	struct lsm303agr_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->tf = (struct lsm303agr_transfer_function *)&lsm303agr_i2c_tf;
	i2c_set_clientdata(client, cdata);

	err = strcmp(client->name, LSM303AGR_ACCEL_DEV_NAME);
	if (!err)
		cdata->id = ACCEL_ID;
	else {
		err = strcmp(client->name, LSM303AGR_MAGN_DEV_NAME);
		if (!err)
			cdata->id = MAGN_ID;
		else {
			kfree(cdata);
			return -EINVAL;
		}
	}

	switch (cdata->id) {
	case ACCEL_ID:
		err = lsm303agr_acc_probe(cdata, client->irq);
		if (err < 0) {
			kfree(cdata);
			return err;
		}
		break;

	case MAGN_ID:
		err = lsm303agr_mag_probe(cdata, client->irq);
		if (err < 0) {
			kfree(cdata);
			return err;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int lsm303agr_i2c_remove(struct i2c_client *client)
{
	int err;
	struct lsm303agr_data *cdata = i2c_get_clientdata(client);

	switch (cdata->id) {
	case ACCEL_ID:
		err = lsm303agr_acc_remove(cdata, client->irq);
		if (err < 0)
			return err;

		break;

	case MAGN_ID:
		err = lsm303agr_mag_remove(cdata, client->irq);
		if (err < 0)
			return err;

		break;

	default:
		return -EINVAL;
	}

	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303agr_i2c_suspend(struct device *dev)
{
	struct lsm303agr_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm303agr_common_suspend(cdata);
}

static int lsm303agr_i2c_resume(struct device *dev)
{
	struct lsm303agr_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm303agr_common_resume(cdata);
}

static const struct dev_pm_ops lsm303agr_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303agr_i2c_suspend, lsm303agr_i2c_resume)
};

#define LSM303AGR_PM_OPS		(&lsm303agr_i2c_pm_ops)
#else /* CONFIG_PM */
#define LSM303AGR_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303agr_ids[] = {
	{ "lsm303agr_accel", 0 },
	{ "lsm303agr_magn", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm303agr_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm303agr_id_table[] = {
	{ .compatible = "st,lsm303agr_accel", },
	{ .compatible = "st,lsm303agr_magn", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303agr_acc_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver lsm303agr_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "lsm303agr-i2c",
		   .pm = LSM303AGR_PM_OPS,
#ifdef CONFIG_OF
		   .of_match_table = lsm303agr_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = lsm303agr_i2c_probe,
	.remove = lsm303agr_i2c_remove,
	.id_table = lsm303agr_ids,
};
module_i2c_driver(lsm303agr_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm303agr i2c driver");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_LICENSE("GPL v2");
