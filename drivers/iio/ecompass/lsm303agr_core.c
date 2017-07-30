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
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <asm/unaligned.h>
#include "lsm303agr_core.h"

enum {
	LSM303AGR_LP_MODE = 0,
	LSM303AGR_HR_MODE,
	LSM303AGR_MODE_COUNT,
};

int lsm303agr_read_register(struct lsm303agr_data *cdata, u8 reg_addr,
							int data_len, u8 *data)
{
	return cdata->tf->read(cdata, reg_addr, data_len, data);
}
EXPORT_SYMBOL(lsm303agr_read_register);

int lsm303agr_write_register(struct lsm303agr_data *cdata, u8 reg_addr, u8 mask,
								u8 data)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = lsm303agr_read_register(cdata, reg_addr, 1, &old_data);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data);
}
EXPORT_SYMBOL(lsm303agr_write_register);

int lsm303agr_set_axis_enable(struct lsm303agr_sensor_data *sdata, u8 axis_enable)
{
	return sdata->cdata->func->set_axis(sdata, axis_enable);
}
EXPORT_SYMBOL(lsm303agr_set_axis_enable);

int lsm303agr_core_set_fs(struct lsm303agr_sensor_data *sdata, unsigned int fs)
{
	int err, i;

	for (i = 0; i < sdata->cdata->sensor_conf->fs.table_size; i++) {
		if (sdata->cdata->sensor_conf->fs.fs_table[i].urv == fs)
			break;
	}

	if (i == sdata->cdata->sensor_conf->fs.table_size)
		return -EINVAL;

	err = lsm303agr_write_register(sdata->cdata,
			sdata->cdata->sensor_conf->fs.fs_addr,
			sdata->cdata->sensor_conf->fs.fs_mask,
			sdata->cdata->sensor_conf->fs.fs_table[i].reg_val);
	if (err < 0)
		return err;

	sdata->gain = sdata->cdata->sensor_conf->fs.fs_table[i].gain;

	return 0;
}

int lsm303agr_core_write_odr(struct lsm303agr_sensor_data *sdata, uint32_t odr) {
	int err, i;

	for (i = 0; i < sdata->cdata->sensor_conf->odr.table_size; i++) {
		if (sdata->cdata->sensor_conf->odr.odr_table[i].hz >= odr)
			break;
	}
	if (i == sdata->cdata->sensor_conf->odr.table_size)
		return -EINVAL;

	err = lsm303agr_write_register(sdata->cdata,
			sdata->cdata->sensor_conf->odr.odr_addr,
			sdata->cdata->sensor_conf->odr.odr_mask,
			sdata->cdata->sensor_conf->odr.odr_table[i].reg_val);
	if (err < 0)
		return err;

	sdata->odr = sdata->cdata->sensor_conf->odr.odr_table[i].hz;

	return 0;
}

int lsm303agr_core_set_enable(struct lsm303agr_sensor_data *sdata, bool state)
{
	int err = 0;

	err = sdata->cdata->func->set_enable(sdata, state);
	if (err < 0)
		return err;

	sdata->enabled = state;

	return 0;
}
EXPORT_SYMBOL(lsm303agr_core_set_enable);

ssize_t lsm303agr_core_get_sampling_frequency(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lsm303agr_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->odr);
}
EXPORT_SYMBOL(lsm303agr_core_get_sampling_frequency);

ssize_t lsm303agr_core_set_sampling_frequency(struct device * dev,
					struct device_attribute * attr,
					const char *buf, size_t count)
{
	int err;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm303agr_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	if (sdata->odr == odr)
		return count;

	mutex_lock(&indio_dev->mlock);
	err = sdata->cdata->func->set_odr(sdata, odr);
	mutex_unlock(&indio_dev->mlock);

	return (err < 0) ? err : count;
}
EXPORT_SYMBOL(lsm303agr_core_set_sampling_frequency);

ssize_t lsm303agr_core_get_sampling_frequency_avail(struct device *dev,
						struct device_attribute
						*attr, char *buf)
{
	int i, len = 0;
	struct lsm303agr_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	for (i = 0; i < sdata->cdata->sensor_conf->odr.table_size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				sdata->cdata->sensor_conf->odr.odr_table[i].hz);

	buf[len - 1] = '\n';

	return len;
}
EXPORT_SYMBOL(lsm303agr_core_get_sampling_frequency_avail);

ssize_t lsm303agr_core_get_scale_avail(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm303agr_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	for (i = 0; i < sdata->cdata->sensor_conf->fs.table_size; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"0.%06u ", sdata->cdata->sensor_conf->fs.fs_table[i].gain);
	}
	buf[len - 1] = '\n';

	return len;
}
EXPORT_SYMBOL(lsm303agr_core_get_scale_avail);

int lsm303agr_core_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *ch, int *val,
				int *val2, long mask)
{
	int err;
	u8 outdata[2];
	struct lsm303agr_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = lsm303agr_core_set_enable(sdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		msleep(sdata->cdata->sensor_conf->turn_on_time_ms);

		err = lsm303agr_read_register(sdata->cdata, ch->address,
								2, outdata);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		err = lsm303agr_core_set_enable(sdata, false);
		mutex_unlock(&indio_dev->mlock);

		if (err < 0)
			return err;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->gain;

		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(lsm303agr_core_read_raw);

int lsm303agr_core_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;
	struct lsm303agr_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);
		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = lsm303agr_core_set_fs(sdata, val2);
		mutex_unlock(&indio_dev->mlock);

		break;

	default:
		return -EINVAL;
	}

	return err;
}
EXPORT_SYMBOL(lsm303agr_core_write_raw);

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops lsm303agr_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = (&lsm303agr_trig_set_state),
};
#define LSM303AGR_TRIGGER_OPS (&lsm303agr_trigger_ops)
#else
#define LSM303AGR_TRIGGER_OPS NULL
#endif


int lsm303agr_common_probe(struct lsm303agr_data *cdata, int irq,
					const struct iio_info *lsm303agr_info)
{
	u8 wai = 0;
	int32_t err;
	struct iio_dev *piio_dev;

	mutex_init(&cdata->tf->tf_lock);

	err = lsm303agr_read_register(cdata, cdata->sensor_conf->wai_addr, 1, &wai);
	if (err < 0) {
		dev_err(cdata->dev, "Failed to read Who-Am-I register.\n");
		return err;
	}

	if (wai != cdata->sensor_conf->wai_val) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	piio_dev = iio_device_alloc(sizeof(struct lsm303agr_sensor_data *));
	if (piio_dev == NULL) {
		err = -ENOMEM;
		goto err_lsm303agr_device_free;
	}

	cdata->sdata = iio_priv(piio_dev);
	cdata->sdata->iio_sensors_dev = piio_dev;
	cdata->sdata->enabled = false;
	cdata->sdata->cdata = cdata;
	cdata->sdata->iio_sensors_dev->name = cdata->sensor_conf->name;
	cdata->sdata->odr = cdata->sensor_conf->odr.default_odr;
	cdata->irq = irq;
	piio_dev->channels = cdata->sensor_conf->chan_spec;
	piio_dev->num_channels = cdata->sensor_conf->chan_spec_size;
	piio_dev->info = lsm303agr_info;
	piio_dev->modes = INDIO_DIRECT_MODE;
	piio_dev->name = kasprintf(GFP_KERNEL, "%s", cdata->sensor_conf->name);

	cdata->sdata->buffer = (u8 *)kmalloc(
				ALIGN(cdata->sensor_conf->output_size,
				sizeof(u64)) + sizeof(u64), GFP_KERNEL);
	if (!cdata->sdata->buffer)
		goto err_lsm303agr_device_free;

	if (cdata->func->init) {
		err = cdata->func->init(cdata);
		if (err < 0)
			goto err_lsm303agr_device_free;
	}

	err = lsm303agr_allocate_buffer(cdata->sdata);
	if (err)
		goto err_lsm303agr_deallocate_buffer;

	if (irq > 0) {
		err = lsm303agr_allocate_triggers(cdata->sdata,
							LSM303AGR_TRIGGER_OPS);
		if (err < 0)
			goto err_lsm303agr_deallocate_buffer;
	}

	err = iio_device_register(cdata->sdata->iio_sensors_dev);
	if (err)
		goto err_lsm303agr_unregister_and_deallocate;

	dev_info(cdata->dev, "%s: probed\n", cdata->sensor_conf->name);
	return 0;

err_lsm303agr_unregister_and_deallocate:
	iio_device_unregister(cdata->sdata->iio_sensors_dev);
	if (cdata->irq > 0)
		lsm303agr_deallocate_triggers(cdata->sdata);

err_lsm303agr_deallocate_buffer:
	lsm303agr_deallocate_buffer(cdata->sdata);
	kfree(cdata->sdata->buffer);

err_lsm303agr_device_free:
	iio_device_free(cdata->sdata->iio_sensors_dev);

	return err;
}
EXPORT_SYMBOL(lsm303agr_common_probe);

void lsm303agr_common_remove(struct lsm303agr_data *cdata, int irq)
{
	iio_device_unregister(cdata->sdata->iio_sensors_dev);

	if (cdata->irq > 0)
		lsm303agr_deallocate_triggers(cdata->sdata);

	lsm303agr_deallocate_buffer(cdata->sdata);
	iio_device_free(cdata->sdata->iio_sensors_dev);
	kfree(cdata->sdata->buffer);
}
EXPORT_SYMBOL(lsm303agr_common_remove);

#ifdef CONFIG_PM
int lsm303agr_common_suspend(struct lsm303agr_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(lsm303agr_common_suspend);

int lsm303agr_common_resume(struct lsm303agr_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(lsm303agr_common_resume);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("STMicroelectronics lsm303agr driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
