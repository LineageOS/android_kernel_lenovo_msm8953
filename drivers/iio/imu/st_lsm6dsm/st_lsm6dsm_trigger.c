/*
 * STMicroelectronics lsm6dsm driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Armando Visconti <armando.visconti@st.com>
 *
 * v. 1.0.0
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>

#include "st_lsm6dsm.h"

#define ST_LSM6DSM_FUNC_SRC_ADDR			0x53
#define ST_LSM6DSM_FUNC_SRC_STEP_DETECTED_MASK		0x10
#define ST_LSM6DSM_FUNC_SRC_TILT_IA_MASK		0x20
#define ST_LSM6DSM_FUNC_SRC_SIGN_MOTION_MASK		0x40
#define ST_LSM6DSM_FUNC_SRC_STEP_COUNT_IA_MASK		0x80

#define ST_LSM6DSM_FIFO_STATUS2_ADDR			0x3b
#define ST_LSM6DSM_FIFO_STATUS2_WATERM_MASK		0x80
#define ST_LSM6DSM_FIFO_STATUS2_FULL_MASK		0x60

static struct workqueue_struct *st_lsm6dsm_wq;

void st_lsm6dsm_flush_works()
{
	flush_workqueue(st_lsm6dsm_wq);
}
EXPORT_SYMBOL(st_lsm6dsm_flush_works);

irqreturn_t st_lsm6dsm_save_timestamp(int irq, void *private)
{
	struct timespec ts;
	struct lsm6dsm_data *cdata = private;

	get_monotonic_boottime(&ts);
	cdata->timestamp = timespec_to_ns(&ts);
	queue_work(st_lsm6dsm_wq, &cdata->data_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static void st_lsm6dsm_irq_management(struct work_struct *data_work)
{
	struct lsm6dsm_data *cdata;
	u8 func_src, src_fifo;

	cdata = container_of((struct work_struct *)data_work,
						struct lsm6dsm_data, data_work);

	cdata->tf->read(cdata, ST_LSM6DSM_FUNC_SRC_ADDR, 1, &func_src, true);
	cdata->tf->read(cdata, ST_LSM6DSM_FIFO_STATUS2_ADDR, 1, &src_fifo,
									true);

	if (src_fifo & ST_LSM6DSM_FIFO_STATUS2_WATERM_MASK) {
		if (src_fifo & ST_LSM6DSM_FIFO_STATUS2_FULL_MASK) {
			st_lsm6dsm_set_fifo_mode(cdata, BYPASS);
			st_lsm6dsm_set_fifo_mode(cdata, CONTINUOS);
			dev_err(cdata->dev,
				"data fifo overrun, reduce fifo size.\n");
		} else
			st_lsm6dsm_read_fifo(cdata, false);
	}

	if ((cdata->sensors_enabled & BIT(ST_INDIO_DEV_STEP_COUNTER)) &&
			(func_src & ST_LSM6DSM_FUNC_SRC_STEP_COUNT_IA_MASK))
		st_lsm6dsm_read_step_counter(cdata);

	if ((cdata->sensors_enabled & BIT(ST_INDIO_DEV_STEP_DETECTOR)) &&
			(func_src & ST_LSM6DSM_FUNC_SRC_STEP_DETECTED_MASK))
		st_lsm6dsm_push_data_with_timestamp(cdata,
					ST_INDIO_DEV_STEP_DETECTOR, NULL,
					cdata->timestamp);

	if ((cdata->smd_event_ready) &&
		(cdata->sensors_enabled & BIT(ST_INDIO_DEV_SIGN_MOTION)) &&
		(func_src & ST_LSM6DSM_FUNC_SRC_SIGN_MOTION_MASK)) {

		iio_push_event(cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION],
				IIO_UNMOD_EVENT_CODE(IIO_SIGN_MOTION,
						0,
						IIO_EV_TYPE_THRESH,
						IIO_EV_DIR_RISING),
				cdata->timestamp);

		cdata->smd_event_ready = false;
	}

	if (func_src & ST_LSM6DSM_FUNC_SRC_TILT_IA_MASK)
		st_lsm6dsm_push_data_with_timestamp(cdata,
					ST_INDIO_DEV_TILT, NULL,
					cdata->timestamp);

	enable_irq(cdata->irq);
	return;
}

int st_lsm6dsm_allocate_triggers(struct lsm6dsm_data *cdata,
				const struct iio_trigger_ops *trigger_ops)
{
	int err, i, n;

	if (!st_lsm6dsm_wq)
		st_lsm6dsm_wq = create_workqueue(cdata->name);

	if (!st_lsm6dsm_wq)
		return -EINVAL;

	INIT_WORK(&cdata->data_work, st_lsm6dsm_irq_management);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		cdata->trig[i] = iio_trigger_alloc("%s-trigger",
				cdata->indio_dev[i]->name);
		if (!cdata->trig[i]) {
			dev_err(cdata->dev,
					"failed to allocate iio trigger.\n");
			err = -ENOMEM;
			goto deallocate_trigger;
		}
		iio_trigger_set_drvdata(cdata->trig[i], cdata->indio_dev[i]);
		cdata->trig[i]->ops = trigger_ops;
		cdata->trig[i]->dev.parent = cdata->dev;
	}

	err = request_threaded_irq(cdata->irq, st_lsm6dsm_save_timestamp, NULL,
					IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		goto deallocate_trigger;

	for (n = 0; n < ST_INDIO_DEV_NUM; n++) {
		err = iio_trigger_register(cdata->trig[n]);
		if (err < 0) {
			dev_err(cdata->dev,
					"failed to register iio trigger.\n");
			goto free_irq;
		}
		cdata->indio_dev[n]->trig = cdata->trig[n];
	}

	return 0;

free_irq:
	free_irq(cdata->irq, cdata);
	for (n--; n >= 0; n--)
		iio_trigger_unregister(cdata->trig[n]);
deallocate_trigger:
	for (i--; i >= 0; i--)
		iio_trigger_free(cdata->trig[i]);

	return err;
}
EXPORT_SYMBOL(st_lsm6dsm_allocate_triggers);

void st_lsm6dsm_deallocate_triggers(struct lsm6dsm_data *cdata)
{
	int i;

	free_irq(cdata->irq, cdata);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_trigger_unregister(cdata->trig[i]);
}
EXPORT_SYMBOL(st_lsm6dsm_deallocate_triggers);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("STMicroelectronics lsm6dsm trigger driver");
MODULE_LICENSE("GPL v2");
