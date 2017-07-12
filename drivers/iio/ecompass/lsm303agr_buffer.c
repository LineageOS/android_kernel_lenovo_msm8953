/*
 * STMicroelectronics lsm303agr driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.0.0
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "lsm303agr_core.h"

int lsm303agr_push_data(struct lsm303agr_sensor_data *sdata)
{
	int err;
	u64 *data;
	struct iio_chan_spec const *chs = sdata->iio_sensors_dev->channels;

	if (!sdata->buffer)
		return -EINVAL;

	err = lsm303agr_read_register(sdata->cdata, chs[0].address,
					sdata->cdata->sensor_conf->output_size,
					sdata->buffer);
	if (err < 0)
		goto err_push_data;

	data = (u64 *)sdata->buffer;
	data[DIV_ROUND_UP(sdata->cdata->sensor_conf->output_size, 8)] =
							sdata->cdata->timestamp;

	iio_push_to_buffers(sdata->iio_sensors_dev, sdata->buffer);

err_push_data:
	return 0;
}
EXPORT_SYMBOL(lsm303agr_push_data);

static inline irqreturn_t lsm303agr_trigger_handler(int irq, void *p)
{
	return IRQ_HANDLED;
}

int lsm303agr_trig_set_state(struct iio_trigger *trig, bool state)
{
	u8 val;
	int err;
	struct lsm303agr_sensor_data *sdata;

	if (state)
		val = LSM303AGR_EN_BIT;
	else
		val = LSM303AGR_DIS_BIT;

	sdata = iio_priv(iio_trigger_get_drvdata(trig));

	err = lsm303agr_write_register(sdata->cdata,
					sdata->cdata->sensor_conf->irq_addr,
					sdata->cdata->sensor_conf->irq_mask,
					val);

	return (err < 0) ? err : 0;
}

static int lsm303agr_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm303agr_sensor_data *sdata = iio_priv(indio_dev);

	err = lsm303agr_core_set_enable(sdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int lsm303agr_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm303agr_sensor_data *sdata = iio_priv(indio_dev);

	flush_workqueue(sdata->cdata->wq);

	err = lsm303agr_core_set_enable(sdata, false);
	if (err < 0)
		return err;

	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0)
		return err;

	return 0;
}

static const struct iio_buffer_setup_ops lsm303agr_buffer_setup_ops = {
	.preenable = &lsm303agr_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &lsm303agr_buffer_predisable,
};

int lsm303agr_allocate_buffer(struct lsm303agr_sensor_data *sdata)
{
	int err;

	err = iio_triggered_buffer_setup(sdata->iio_sensors_dev,
					&lsm303agr_trigger_handler,
					NULL,
					&lsm303agr_buffer_setup_ops);
	if (err < 0)
		iio_triggered_buffer_cleanup(sdata->iio_sensors_dev);

	return (err < 0) ? err : 0;
}
EXPORT_SYMBOL(lsm303agr_allocate_buffer);

void lsm303agr_deallocate_buffer(struct lsm303agr_sensor_data *sdata)
{
	iio_triggered_buffer_cleanup(sdata->iio_sensors_dev);
}
EXPORT_SYMBOL(lsm303agr_deallocate_buffer);

MODULE_DESCRIPTION("STMicroelectronics lsm303agr driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
