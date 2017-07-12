/*
 * STMicroelectronics lis2ds12 driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>

#include "lis2ds12_core.h"

static struct workqueue_struct *lis2ds12_workqueue;

void lis2ds12_flush_works()
{
	flush_workqueue(lis2ds12_workqueue);
}
EXPORT_SYMBOL(lis2ds12_flush_works);

irqreturn_t lis2ds12_save_timestamp(int irq, void *private)
{
	struct timespec ts;
	struct lis2ds12_data *cdata = private;

	get_monotonic_boottime(&ts);
	cdata->timestamp = timespec_to_ns(&ts);
	queue_work(lis2ds12_workqueue, &cdata->data_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

void lis2ds12_event_management(struct lis2ds12_data *cdata, u8 int_reg_val,
			     u8 ck_gate_val)
{
	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_TAP) &&
				(int_reg_val & LIS2DS12_TAP_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_TAP],
					IIO_UNMOD_EVENT_CODE(IIO_TAP, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_DOUBLE_TAP) &&
				(int_reg_val & LIS2DS12_DOUBLE_TAP_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_DOUBLE_TAP],
					IIO_UNMOD_EVENT_CODE(IIO_TAP_TAP, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

	if (ck_gate_val & LIS2DS12_FUNC_CK_GATE_STEP_D_MASK) {
		if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_STEP_D))
			iio_push_event(cdata->iio_sensors_dev[LIS2DS12_STEP_D],
					IIO_UNMOD_EVENT_CODE(IIO_STEP_DETECTOR, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

		if(CHECK_BIT(cdata->enabled_sensor, LIS2DS12_STEP_C))
			lis2ds12_read_step_c(cdata);
	}

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_TILT) &&
			(ck_gate_val & LIS2DS12_FUNC_CK_GATE_TILT_INT_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_TILT],
					IIO_UNMOD_EVENT_CODE(IIO_TILT, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_SIGN_M) &&
			(ck_gate_val & LIS2DS12_FUNC_CK_GATE_SIGN_M_DET_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_SIGN_M],
					IIO_UNMOD_EVENT_CODE(IIO_SIGN_MOTION, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);
}

void lis2ds12_irq_management(struct work_struct *data_work)
{
	struct lis2ds12_data *cdata;
	u8 status[4], func[2], fifo;

	cdata = container_of((struct work_struct *)data_work,
			     struct lis2ds12_data, data_work);

	cdata->tf->read(cdata, LIS2DS12_STATUS_DUP_ADDR, 4, status);
	cdata->tf->read(cdata, LIS2DS12_FUNC_CK_GATE_ADDR, 2, func);
	cdata->tf->read(cdata, LIS2DS12_FIFO_SRC_ADDR, 1, &fifo);

	if (fifo & LIS2DS12_FIFO_SRC_FTH_MASK)
		lis2ds12_read_fifo(cdata, true);

	if (status[0] & (LIS2DS12_EVENT_MASK | LIS2DS12_FUNC_CK_GATE_MASK))
		lis2ds12_event_management(cdata, status[0], func[0]);

	enable_irq(cdata->irq);
}

int lis2ds12_allocate_triggers(struct lis2ds12_data *cdata,
			     const struct iio_trigger_ops *trigger_ops)
{
	int err, i, n;

	if (!lis2ds12_workqueue)
		lis2ds12_workqueue = create_workqueue(cdata->name);

	if (!lis2ds12_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->data_work, lis2ds12_irq_management);

	for (i = 0; i < LIS2DS12_SENSORS_NUMB; i++) {
		cdata->iio_trig[i] = iio_trigger_alloc("%s-trigger",
						cdata->iio_sensors_dev[i]->name);
		if (!cdata->iio_trig[i]) {
			dev_err(cdata->dev, "failed to allocate iio trigger.\n");
			err = -ENOMEM;

			goto deallocate_trigger;
		}
		iio_trigger_set_drvdata(cdata->iio_trig[i],
						cdata->iio_sensors_dev[i]);
		cdata->iio_trig[i]->ops = trigger_ops;
		cdata->iio_trig[i]->dev.parent = cdata->dev;
	}

	err = request_threaded_irq(cdata->irq, lis2ds12_save_timestamp, NULL,
				   IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		goto deallocate_trigger;

	for (n = 0; n < LIS2DS12_SENSORS_NUMB; n++) {
		err = iio_trigger_register(cdata->iio_trig[n]);
		if (err < 0) {
			dev_err(cdata->dev, "failed to register iio trigger.\n");

			goto free_irq;
		}
		cdata->iio_sensors_dev[n]->trig = cdata->iio_trig[n];
	}

	return 0;

free_irq:
	free_irq(cdata->irq, cdata);
	for (n--; n >= 0; n--)
		iio_trigger_unregister(cdata->iio_trig[n]);
deallocate_trigger:
	for (i--; i >= 0; i--)
		iio_trigger_free(cdata->iio_trig[i]);

	return err;
}
EXPORT_SYMBOL(lis2ds12_allocate_triggers);

void lis2ds12_deallocate_triggers(struct lis2ds12_data *cdata)
{
	int i;

	free_irq(cdata->irq, cdata);

	for (i = 0; i < LIS2DS12_SENSORS_NUMB; i++)
		iio_trigger_unregister(cdata->iio_trig[i]);
}
EXPORT_SYMBOL(lis2ds12_deallocate_triggers);

MODULE_DESCRIPTION("STMicroelectronics lis2ds12 i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
