/*
 * STMicroelectronics lsm303agr spi driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#include "lsm303agr_core.h"

static int lsm303agr_spi_read(struct lsm303agr_data *cdata,
						u8 reg_addr, int len, u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers[] = {
		{
			.tx_buf = cdata->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = cdata->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	mutex_lock(&cdata->tf->tf_lock);
	cdata->tb.tx_buf[0] = reg_addr | 0x80;
	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);
	if (err) {
		mutex_unlock(&cdata->tf->tf_lock);
		return err;
	}

	memcpy(data, cdata->tb.rx_buf, len * sizeof(u8));
	mutex_unlock(&cdata->tf->tf_lock);

	return len;
}

static int lsm303agr_spi_write(struct lsm303agr_data *cdata,
						u8 reg_addr, int len, u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	mutex_lock(&cdata->tf->tf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);

	mutex_unlock(&cdata->tf->tf_lock);

	return err;
}

static const struct lsm303agr_transfer_function lsm303agr_spi_tf = {
	.write = lsm303agr_spi_write,
	.read = lsm303agr_spi_read,
};

static int lsm303agr_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm303agr_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->tf = (struct lsm303agr_transfer_function *)&lsm303agr_spi_tf;
	spi_set_drvdata(spi, cdata);

	err = strcmp(spi->modalias, LSM303AGR_ACCEL_DEV_NAME);
	if (!err)
		cdata->id = ACCEL_ID;
	else {
		err = strcmp(spi->modalias, LSM303AGR_MAGN_DEV_NAME);
		if (!err)
			cdata->id = MAGN_ID;
		else {
			kfree(cdata);
			return -EINVAL;
		}
	}

	switch (cdata->id) {
	case ACCEL_ID:
		err = lsm303agr_acc_probe(cdata, spi->irq);
		if (err < 0) {
			kfree(cdata);
			return err;
		}
		break;

	case MAGN_ID:
		err = lsm303agr_mag_probe(cdata, spi->irq);
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

static int lsm303agr_spi_remove(struct spi_device *spi)
{
	int err;
	struct lsm303agr_data *cdata = spi_get_drvdata(spi);

	switch (cdata->id) {
	case ACCEL_ID:
		err = lsm303agr_acc_remove(cdata, spi->irq);
		if (err < 0)
			return err;

		break;

	case MAGN_ID:
		err = lsm303agr_mag_remove(cdata, spi->irq);
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
static int lsm303agr_spi_suspend(struct device *dev)
{
	struct lsm303agr_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm303agr_common_suspend(cdata);
}

static int lsm303agr_spi_resume(struct device *dev)
{
	struct lsm303agr_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm303agr_common_resume(cdata);
}

static const struct dev_pm_ops lsm303agr_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303agr_spi_suspend, lsm303agr_spi_resume)
};

#define LSM303AGR_PM_OPS		(&lsm303agr_spi_pm_ops)
#else /* CONFIG_PM */
#define LSM303AGR_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id lsm303agr_ids[] = {
	{ "lsm303agr_accel", 0 },
	{ "lsm303agr_magn", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm303agr_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm303agr_id_table[] = {
	{ .compatible = "st,lsm303agr_accel", },
	{ .compatible = "st,lsm303agr_magn", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303agr_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lsm303agr_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "lsm303agr-spi",
		   .pm = LSM303AGR_PM_OPS,
#ifdef CONFIG_OF
		   .of_match_table = lsm303agr_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = lsm303agr_spi_probe,
	.remove = lsm303agr_spi_remove,
	.id_table = lsm303agr_ids,
};
module_spi_driver(lsm303agr_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm303agr spi driver");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_LICENSE("GPL v2");
