/*
 * STMicroelectronics lsm303agr driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.0.0
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include "lsm303agr_core.h"

irqreturn_t lsm303agr_save_timestamp(int irq, void *private)
{
	struct timespec ts;
	struct lsm303agr_data *cdata = private;

	get_monotonic_boottime(&ts);
	cdata->timestamp = timespec_to_ns(&ts);
	queue_work(cdata->wq, &cdata->work_s);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static void lsm303agr_irq_management(struct work_struct *work_s)
{
	struct lsm303agr_data *cdata;

	cdata = container_of((struct work_struct *)work_s,
						struct lsm303agr_data, work_s);

	lsm303agr_push_data(cdata->sdata);

	enable_irq(cdata->irq);
}

int lsm303agr_allocate_triggers(struct lsm303agr_sensor_data *sdata,
				const struct iio_trigger_ops *trigger_ops)
{
	int err;

	sdata->cdata->wq = create_workqueue(sdata->cdata->sensor_conf->name);
	if (!sdata->cdata->wq)
		return -EINVAL;

	INIT_WORK(&sdata->cdata->work_s, lsm303agr_irq_management);

	sdata->iio_trig = iio_trigger_alloc("%s-trigger",
					sdata->iio_sensors_dev->name);
	if (sdata->iio_trig == NULL) {
		err = -ENOMEM;
		dev_err(&sdata->iio_sensors_dev->dev,
					"failed to allocate iio trigger.\n");

		return -ENOMEM;
	}

	err = request_threaded_irq(sdata->cdata->irq,
					lsm303agr_save_timestamp,
					NULL,
					IRQF_TRIGGER_HIGH,
					sdata->cdata->sensor_conf->name,
					sdata->cdata);
	if (err)
		goto request_irq_error;

	iio_trigger_set_drvdata(sdata->iio_trig, sdata->iio_sensors_dev);
	sdata->iio_trig->ops = trigger_ops;
	sdata->iio_trig->dev.parent = sdata->cdata->dev;

	err = iio_trigger_register(sdata->iio_trig);
	if (err < 0) {
		dev_err(&sdata->iio_sensors_dev->dev,
					"failed to register iio trigger.\n");
		goto trigger_register_error;
	}
	sdata->iio_sensors_dev->trig = sdata->iio_trig;

	return 0;

trigger_register_error:
	free_irq(sdata->cdata->irq, sdata->iio_trig);

request_irq_error:
	iio_trigger_free(sdata->iio_trig);

	return err;
}
EXPORT_SYMBOL(lsm303agr_allocate_triggers);

void lsm303agr_deallocate_triggers(struct lsm303agr_sensor_data *sdata)
{
	destroy_workqueue(sdata->cdata->wq);
	iio_trigger_unregister(sdata->iio_trig);
	free_irq(sdata->cdata->irq, sdata->iio_trig);
	iio_trigger_free(sdata->iio_trig);
}
EXPORT_SYMBOL(lsm303agr_deallocate_triggers);

MODULE_DESCRIPTION("STMicroelectronics lsm303agr driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
