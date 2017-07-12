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

#include "st_lsm6dsm.h"

#define ST_LSM6DSM_ENABLE_AXIS			0x07
#define ST_LSM6DSM_FIFO_STATUS1_ADDR		0x3a
#define ST_LSM6DSM_FIFO_DIFF_MASK		0x0fff
#define ST_LSM6DSM_FIFO_DATA_OUT_L_ADDR		0x3e
#define ST_LSM6DSM_FIFO_DATA_OVR_2REGS_VAL	0x4000
#define ST_LSM6DSM_STEP_CUNTER_L_ADDR		0x4b

void st_lsm6dsm_push_data_with_timestamp(struct lsm6dsm_data *cdata,
					u8 index, u8 *data, int64_t timestamp)
{
	int i, n = 0;
	struct iio_chan_spec const *chs = cdata->indio_dev[index]->channels;
	uint16_t bfch, bfchs_out = 0, bfchs_in = 0;
	struct lsm6dsm_sensor_data *sdata = iio_priv(cdata->indio_dev[index]);

	for (i = 0; i < sdata->num_data_channels; i++) {
		bfch = chs[i].scan_type.storagebits >> 3;

		if (test_bit(i, cdata->indio_dev[index]->active_scan_mask)) {
			memcpy(&sdata->buffer_data[bfchs_out],
							&data[bfchs_in], bfch);
			n++;
			bfchs_out += bfch;
		}

		bfchs_in += bfch;
	}

	if (cdata->indio_dev[index]->scan_timestamp) {
		size_t offset = cdata->indio_dev[index]->scan_bytes /
							sizeof(int64_t) - 1;
		((int64_t *)sdata->buffer_data)[offset] = timestamp;
	}

	iio_push_to_buffers(cdata->indio_dev[index], sdata->buffer_data);
}

static void st_lsm6dsm_parse_fifo_data(struct lsm6dsm_data *cdata, u16 read_len)
{
	u16 fifo_offset = 0;
	u8 i, samples_in_pattern[ST_FIFO_DEV_NUM];
	u16 total_samples;
	struct st_lsm6dsm_fifo_cfg *pfifo_cfg;

	while (fifo_offset < read_len) {
		for (i = 0, total_samples = 0; i < ST_FIFO_DEV_NUM; i++)
			samples_in_pattern[i] =
					cdata->fifo_cfg[i].samples_in_pattern;

		total_samples = cdata->total_samples_in_pattern;

		do {
			for (i = 0; i < ST_FIFO_DEV_NUM; i++) {
				if (samples_in_pattern[i] == 0)
					continue;

				pfifo_cfg = &cdata->fifo_cfg[i];
				if (pfifo_cfg->samples_to_discard > 0) {
						pfifo_cfg->samples_to_discard--;
				} else {
					pfifo_cfg->timestamp +=
						pfifo_cfg->deltatime;
					st_lsm6dsm_push_data_with_timestamp(
						cdata, pfifo_cfg->sindex,
						&cdata->fifo_data[fifo_offset],
						pfifo_cfg->timestamp);
				}
				fifo_offset += ST_LSM6DSM_FIFO_ELEMENT_LEN_BYTE;
				samples_in_pattern[i]--;
				total_samples--;
			}
		} while (total_samples > 0);
	}

	return;
}

int st_lsm6dsm_read_fifo(struct lsm6dsm_data *cdata, bool check_fifo_len)
{
	int err;
#if (CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO > 0)
	u16 data_remaining, data_to_read;
#endif /* CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO */
	u16 read_len = cdata->fifo_threshold, byte_in_pattern;

	if (!cdata->fifo_data)
		return -EINVAL;

	if (check_fifo_len) {
		err = cdata->tf->read(cdata, ST_LSM6DSM_FIFO_STATUS1_ADDR,
						2, (u8 *)&read_len, true);
		if (err < 0)
			return err;

		if (read_len & ST_LSM6DSM_FIFO_DATA_OVR_2REGS_VAL) {
			dev_err(cdata->dev,
				"data fifo overrun, failed to read it.\n");
			return -EINVAL;
		}

		read_len &= ST_LSM6DSM_FIFO_DIFF_MASK;
		read_len *= ST_LSM6DSM_BYTE_FOR_CHANNEL;

		byte_in_pattern =  cdata->total_samples_in_pattern *
					ST_LSM6DSM_FIFO_ELEMENT_LEN_BYTE;

		read_len = (read_len / byte_in_pattern) * byte_in_pattern;

		if (read_len > cdata->fifo_threshold)
			read_len = cdata->fifo_threshold;
	}

	if (read_len == 0)
		return -EINVAL;

#if (CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO == 0)
	err = cdata->tf->read(cdata, ST_LSM6DSM_FIFO_DATA_OUT_L_ADDR,
					read_len, cdata->fifo_data, true);
	if (err < 0)
		return err;
#else /* CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO */
	data_remaining = read_len;

	do {
		if (data_remaining > CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO)
			data_to_read = CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO;
		else
			data_to_read = data_remaining;

		err = cdata->tf->read(cdata, ST_LSM6DSM_FIFO_DATA_OUT_L_ADDR,
				data_to_read,
				&cdata->fifo_data[read_len - data_remaining],
				true);
		if (err < 0)
			return err;

		data_remaining -= data_to_read;
	} while (data_remaining > 0);
#endif /* CONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO */

	st_lsm6dsm_parse_fifo_data(cdata, read_len);

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_read_fifo);

int st_lsm6dsm_read_step_counter(struct lsm6dsm_data *cdata)
{
	int err;
	struct lsm6dsm_sensor_data *sdata;

	sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
	err = cdata->tf->read(cdata, ST_LSM6DSM_STEP_CUNTER_L_ADDR,
					ST_LSM6DSM_BYTE_FOR_CHANNEL,
					sdata->buffer_data, true);
	if (err < 0)
		return err;

	st_lsm6dsm_push_data_with_timestamp(cdata, ST_INDIO_DEV_STEP_COUNTER,
							sdata->buffer_data,
							cdata->timestamp);

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_read_step_counter);

static inline irqreturn_t st_lsm6dsm_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

int st_lsm6dsm_trig_set_state(struct iio_trigger *trig, bool state)
{
	int err;
	struct lsm6dsm_sensor_data *sdata;

	sdata = iio_priv(iio_trigger_get_drvdata(trig));

	err = st_lsm6dsm_set_drdy_irq(sdata, state);

	return err < 0 ? err : 0;
}
EXPORT_SYMBOL(st_lsm6dsm_trig_set_state);

static int st_lsm6dsm_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	if ((sdata->sindex == ST_INDIO_DEV_ACCEL) &&
				(sdata->cdata->injection_mode))
		return -EBUSY;
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */

	err = st_lsm6dsm_set_enable(sdata, true);
	if (err < 0)
		return err;

	err = st_lsm6dsm_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int st_lsm6dsm_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	if (sdata->hwfifo_enabled && (indio_dev->buffer->length <
					2 * ST_LSM6DSM_MAX_FIFO_LENGHT))
		return -EINVAL;

	sdata->buffer_data = kmalloc(indio_dev->scan_bytes *
					indio_dev->buffer->length, GFP_KERNEL);
	if (!sdata->buffer_data)
		return -ENOMEM;

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0)
		goto free_buffer_data;

	if (sdata->sindex == ST_INDIO_DEV_SIGN_MOTION)
		sdata->cdata->smd_event_ready = true;

	return 0;

free_buffer_data:
	kfree(sdata->buffer_data);

	return err;
}

static int st_lsm6dsm_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	err = st_lsm6dsm_set_enable(sdata, false);
	if (err < 0)
		return err;

	err = st_lsm6dsm_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	if (sdata->sindex == ST_INDIO_DEV_SIGN_MOTION)
		sdata->cdata->smd_event_ready = false;

	kfree(sdata->buffer_data);

	return 0;
}

static const struct iio_buffer_setup_ops st_lsm6dsm_buffer_setup_ops = {
	.preenable = &st_lsm6dsm_buffer_preenable,
	.postenable = &st_lsm6dsm_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &st_lsm6dsm_buffer_postdisable,
};

int st_lsm6dsm_allocate_rings(struct lsm6dsm_data *cdata)
{
	int err;
	unsigned int i;

	for(i = 0; i < ST_INDIO_DEV_NUM; i++) {
		err = iio_triggered_buffer_setup(cdata->indio_dev[i],
						&st_lsm6dsm_handler_empty, NULL,
						&st_lsm6dsm_buffer_setup_ops);
		if (err < 0)
			goto buffer_cleanup;
	}

	return 0;

buffer_cleanup:
	for (i--; i >= 0; i--)
		iio_triggered_buffer_cleanup(cdata->indio_dev[i]);

	return err;
}
EXPORT_SYMBOL(st_lsm6dsm_allocate_rings);

void st_lsm6dsm_deallocate_rings(struct lsm6dsm_data *cdata)
{
	unsigned int i;

	for(i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_triggered_buffer_cleanup(cdata->indio_dev[i]);
}
EXPORT_SYMBOL(st_lsm6dsm_deallocate_rings);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("STMicroelectronics lsm6dsm buffer driver");
MODULE_LICENSE("GPL v2");
