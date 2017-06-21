/*
 * STMicroelectronics lis2ds12 driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
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

#include "lis2ds12_core.h"

#define LIS2DS12_ACCEL_BUFFER_SIZE \
		ALIGN(LIS2DS12_FIFO_BYTE_FOR_SAMPLE + LIS2DS12_TIMESTAMP_SIZE, \
		      LIS2DS12_TIMESTAMP_SIZE)
#define LIS2DS12_STEP_C_BUFFER_SIZE \
		ALIGN(LIS2DS12_FIFO_BYTE_X_AXIS + LIS2DS12_TIMESTAMP_SIZE, \
		      LIS2DS12_TIMESTAMP_SIZE)

void lis2ds12_push_accel_fifo_data(struct lis2ds12_data *cdata, u16 read_length)
{
	size_t offset;
	uint16_t i, j, k;
	u8 buffer[LIS2DS12_ACCEL_BUFFER_SIZE], out_buf_index;
	struct iio_dev *indio_dev = cdata->iio_sensors_dev[LIS2DS12_ACCEL];

	for (i = 0; i < read_length; i += LIS2DS12_FIFO_BYTE_FOR_SAMPLE) {
		for (j = 0, out_buf_index = 0; j < LIS2DS12_FIFO_NUM_AXIS;
									j++) {
			k = i + LIS2DS12_FIFO_BYTE_X_AXIS * j;
			if (test_bit(j, indio_dev->active_scan_mask)) {
				memcpy(&buffer[out_buf_index],
						&cdata->fifo_data[k],
						LIS2DS12_FIFO_BYTE_X_AXIS);
				out_buf_index += LIS2DS12_FIFO_BYTE_X_AXIS;
			}
		}

		if (indio_dev->scan_timestamp) {
			offset = indio_dev->scan_bytes / sizeof(s64) - 1;
			((s64 *)buffer)[offset] = cdata->accel_timestamp;
			cdata->accel_timestamp += cdata->accel_deltatime;
		}

		iio_push_to_buffers(indio_dev, buffer);
	}
}

void lis2ds12_read_fifo(struct lis2ds12_data *cdata, bool check_fifo_len)
{
	int err;
	u8 fifo_src[2];
	u16 read_len = cdata->fifo_size;

	if (!cdata->fifo_data)
		return;

	if (check_fifo_len) {
		err = lis2ds12_read_register(cdata, LIS2DS12_FIFO_SRC, 2,
								fifo_src);
		if (err < 0)
			return;

		read_len = 0;
		read_len = (fifo_src[0] & LIS2DS12_FIFO_SRC_DIFF_MASK) ?
								(1 << 8) : 0;
		read_len |= fifo_src[1];
		read_len *= LIS2DS12_FIFO_BYTE_FOR_SAMPLE;

		if (read_len > cdata->fifo_size)
			read_len = cdata->fifo_size;
	}
	if (read_len == 0)
		return;

	err = lis2ds12_read_register(cdata, LIS2DS12_OUTX_L_ADDR, read_len,
							cdata->fifo_data);
	if (err < 0)
		return;

	cdata->accel_timestamp = cdata->timestamp;
	lis2ds12_push_accel_fifo_data(cdata, read_len);
}

void lis2ds12_read_step_c(struct lis2ds12_data *cdata)
{
	int err;
	int64_t timestamp = 0;
	char buffer[LIS2DS12_STEP_C_BUFFER_SIZE];
	struct iio_dev *indio_dev = cdata->iio_sensors_dev[LIS2DS12_STEP_C];

	err = lis2ds12_read_register(cdata, (u8)indio_dev->channels[0].address,
								2, buffer);
	if (err < 0)
		goto lis2ds12_step_counter_done;

	timestamp = cdata->timestamp;
	if (indio_dev->scan_timestamp)
		*(s64 *) ((u8 *) buffer +
			ALIGN(LIS2DS12_FIFO_BYTE_X_AXIS, sizeof(s64))) =
								timestamp;

	iio_push_to_buffers(indio_dev, buffer);

lis2ds12_step_counter_done:
	iio_trigger_notify_done(indio_dev->trig);
}

static inline irqreturn_t lis2ds12_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

int lis2ds12_trig_set_state(struct iio_trigger *trig, bool state)
{
	int err;
	struct lis2ds12_sensor_data *sdata;

	sdata = iio_priv(iio_trigger_get_drvdata(trig));
	err = lis2ds12_update_drdy_irq(sdata, state);

	return (err < 0) ? err : 0;
}

static int lis2ds12_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lis2ds12_sensor_data *sdata = iio_priv(indio_dev);

	if (sdata->sindex == LIS2DS12_ACCEL) {
		err = lis2ds12_update_fifo(sdata->cdata);
		if (err < 0)
			return err;
	}

	err = lis2ds12_set_enable(sdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int lis2ds12_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err, indx;
	struct lis2ds12_sensor_data *sdata = iio_priv(indio_dev);

	err = lis2ds12_set_enable(sdata, false);
	if (err < 0)
		return err;

	indx = sdata->sindex;
	if ((indx == LIS2DS12_ACCEL) || (indx == LIS2DS12_STEP_C)) {
		kfree(sdata->cdata->fifo_data);
		sdata->cdata->fifo_data = 0;
	}

	return 0;
}

static const struct iio_buffer_setup_ops lis2ds12_buffer_setup_ops = {
	.preenable = &lis2ds12_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &lis2ds12_buffer_postdisable,
};

int lis2ds12_allocate_rings(struct lis2ds12_data *cdata)
{
	int err, i;

	for (i = 0; i < LIS2DS12_SENSORS_NUMB; i++) {
		err = iio_triggered_buffer_setup(
				cdata->iio_sensors_dev[i],
				&lis2ds12_handler_empty,
				NULL,
				&lis2ds12_buffer_setup_ops);
		if (err < 0)
			goto buffer_cleanup;
	}

	return 0;

buffer_cleanup:
	for (i--; i >= 0; i--)
		iio_triggered_buffer_cleanup(cdata->iio_sensors_dev[i]);

	return err;
}

void lis2ds12_deallocate_rings(struct lis2ds12_data *cdata)
{
	int i;

	for (i = 0; i < LIS2DS12_SENSORS_NUMB; i++)
		iio_triggered_buffer_cleanup(cdata->iio_sensors_dev[i]);
}

MODULE_DESCRIPTION("STMicroelectronics lis2ds12 i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
