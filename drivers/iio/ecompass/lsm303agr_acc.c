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
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "lsm303agr_core.h"

/**
 * Accelerometer specific registers
 */
#define LSM303AGR_WHO_AM_I_ACC_ADDR		0x0f
#define LSM303AGR_WHO_AM_I_ACC_DEF		0x33

/* Accelerometer control registers */
#define LSM303AGR_CTRL1_ACC_ADDR		0x20
#define LSM303AGR_CTRL1_ACC_ODR_MASK		0xf0
#define LSM303AGR_CTRL1_ACC_ODR_PD		0x00
#define LSM303AGR_CTRL1_ACC_ODR_1HZ		0x01
#define LSM303AGR_CTRL1_ACC_ODR_10HZ		0x02
#define LSM303AGR_CTRL1_ACC_ODR_25HZ		0x03
#define LSM303AGR_CTRL1_ACC_ODR_50HZ		0x04
#define LSM303AGR_CTRL1_ACC_ODR_100HZ		0x05
#define LSM303AGR_CTRL1_ACC_ODR_200HZ		0x06
#define LSM303AGR_CTRL1_ACC_ODR_400HZ		0x07
#define LSM303AGR_CTRL1_ACC_AXIS_MASK		0x07

#define LSM303AGR_CTRL2_ACC_ADDR		0x21
#define LSM303AGR_CTRL3_ACC_ADDR		0x22
#define LSM303AGR_CTRL3_ACC_I1_DRDY1_MASK	0x10

#define LSM303AGR_CTRL4_ACC_ADDR		0x23
#define LSM303AGR_CTRL4_ACC_BDU_MASK		0x80
#define LSM303AGR_CTRL4_ACC_FS_MASK		0x30
#define LSM303AGR_CTRL4_ACC_FS_2G		0x00
#define LSM303AGR_CTRL4_ACC_FS_4G		0x01
#define LSM303AGR_CTRL4_ACC_FS_8G		0x02
#define LSM303AGR_CTRL4_ACC_FS_16G		0x03
#define LSM303AGR_CTRL4_ACC_FS_COUNT		4

#define LSM303AGR_CTRL5_ACC_ADDR		0x24
#define LSM303AGR_CTRL6_ACC_ADDR		0x25

/* Accelerometer output registers */
#define LSM303AGR_OUTX_L_ACC_ADDR		0x28
#define LSM303AGR_OUTY_L_ACC_ADDR		0x2A
#define LSM303AGR_OUTZ_L_ACC_ADDR		0x2C

/* Accelerometer interrupt control registers */
#define LSM303AGR_INT1_CFG_ACC_ADDR		0x30
#define LSM303AGR_INT1_SRC_ACC_ADDR		0x31
#define LSM303AGR_INT2_CFG_ACC_ADDR		0x34
#define LSM303AGR_INT2_SRC_ACC_ADDR		0x35

#define LSM303AGR_ACC_FS_2G_GAIN		IIO_G_TO_M_S_2(980)
#define LSM303AGR_ACC_FS_4G_GAIN		IIO_G_TO_M_S_2(1950)
#define LSM303AGR_ACC_FS_8G_GAIN		IIO_G_TO_M_S_2(3900)
#define LSM303AGR_ACC_FS_16G_GAIN		IIO_G_TO_M_S_2(11720)

#define LSM303AGR_DEFAULT_ACC_FS		4
#define LSM303AGR_ACC_ODR_HZ_DEF		10

#define LSM303AGR_ACC_ODR_TABLE_SIZE		7
#define LSM303AGR_ACC_FS_TABLE_SIZE		4
#define LSM303AGR_CHANNEL_SPECS_SIZE		4

#define LSM303AGR_ACC_TURNON_TIME_MS		2
#define LSM303AGR_AXIS_COUNT			3
#define LSM303AGR_BYTE_X_AXIS			2
#define LSM303AGR_OUT_DATA_SIZE			(LSM303AGR_AXIS_COUNT * \
						LSM303AGR_BYTE_X_AXIS)

static struct iio_chan_spec lsm303agr_acc_chan_spec[LSM303AGR_CHANNEL_SPECS_SIZE] = {
	LSM303AGR_ADD_CHANNEL(IIO_ACCEL, 1, 0, IIO_MOD_X, IIO_LE, 16, 12,
				LSM303AGR_OUTX_L_ACC_ADDR, 's'),
	LSM303AGR_ADD_CHANNEL(IIO_ACCEL, 1, 1, IIO_MOD_Y, IIO_LE, 16, 12,
				LSM303AGR_OUTY_L_ACC_ADDR, 's'),
	LSM303AGR_ADD_CHANNEL(IIO_ACCEL, 1, 2, IIO_MOD_Z, IIO_LE, 16, 12,
				LSM303AGR_OUTZ_L_ACC_ADDR, 's'),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static struct lsm303agr_odr_table_t lsm303agr_odr_table[LSM303AGR_ACC_ODR_TABLE_SIZE] = {
	[0] = {
		.hz = 1,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_1HZ,
	},
	[1] = {
		.hz = 10,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_10HZ,
	},
	[2] = {
		.hz = 25,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_25HZ,
	},
	[3] = {
		.hz = 50,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_50HZ,
	},
	[4] = {
		.hz = 100,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_100HZ,
	},
	[5] = {
		.hz = 200,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_200HZ,
	},
	[6] = {
		.hz = 400,
		.reg_val = LSM303AGR_CTRL1_ACC_ODR_400HZ,
	},
};

static struct lsm303agr_fs_table_t lsm303agr_fs_table[LSM303AGR_ACC_FS_TABLE_SIZE] = {
	[0] = {
		.gain = LSM303AGR_ACC_FS_2G_GAIN,
		.reg_val = LSM303AGR_CTRL4_ACC_FS_2G,
		.urv = 2,
	},
	[1] = {
		.gain = LSM303AGR_ACC_FS_4G_GAIN,
		.reg_val = LSM303AGR_CTRL4_ACC_FS_4G,
		.urv = 4,
	},
	[2] = {
		.gain = LSM303AGR_ACC_FS_8G_GAIN,
		.reg_val = LSM303AGR_CTRL4_ACC_FS_8G,
		.urv = 8,
	},
	[3] = {
		.gain = LSM303AGR_ACC_FS_16G_GAIN,
		.reg_val = LSM303AGR_CTRL4_ACC_FS_16G,
		.urv = 16,
	},
};

static struct lsm303agr_sensor_conf lsm303agr_acc_conf = {
	.name = LSM303AGR_ACCEL_DEV_NAME,
	.wai_addr = LSM303AGR_WHO_AM_I_ACC_ADDR,
	.wai_val = LSM303AGR_WHO_AM_I_ACC_DEF,
	.odr = {
		.odr_addr = LSM303AGR_CTRL1_ACC_ADDR,
		.odr_mask = LSM303AGR_CTRL1_ACC_ODR_MASK,
		.odr_table = lsm303agr_odr_table,
		.table_size = LSM303AGR_ACC_ODR_TABLE_SIZE,
		.default_odr = LSM303AGR_ACC_ODR_HZ_DEF,
	},
	.fs = {
		.fs_addr = LSM303AGR_CTRL4_ACC_ADDR,
		.fs_mask = LSM303AGR_CTRL4_ACC_FS_MASK,
		.default_fs = LSM303AGR_DEFAULT_ACC_FS,
		.table_size = LSM303AGR_ACC_FS_TABLE_SIZE,
		.fs_table = lsm303agr_fs_table,
	},
	.chan_spec = lsm303agr_acc_chan_spec,
	.chan_spec_size = LSM303AGR_CHANNEL_SPECS_SIZE,
	.irq_addr = LSM303AGR_CTRL3_ACC_ADDR,
	.irq_mask = LSM303AGR_CTRL3_ACC_I1_DRDY1_MASK,
	.turn_on_time_ms = LSM303AGR_ACC_TURNON_TIME_MS,
	.output_size = LSM303AGR_OUT_DATA_SIZE,
};

static int lsm303agr_acc_set_enable(struct lsm303agr_sensor_data *sdata,
								bool state)
{
	int err = 0;

	if (state)
		err = lsm303agr_core_write_odr(sdata, sdata->odr);
	else
		err = lsm303agr_write_register(sdata->cdata,
					LSM303AGR_CTRL1_ACC_ADDR,
					LSM303AGR_CTRL1_ACC_ODR_MASK,
					LSM303AGR_CTRL1_ACC_ODR_PD);

	return err;
}

static int lsm303agr_acc_set_axis(struct lsm303agr_sensor_data *sdata,
								u8 axis_enable)
{
	return lsm303agr_write_register(sdata->cdata,
					LSM303AGR_CTRL1_ACC_ADDR,
					LSM303AGR_CTRL1_ACC_AXIS_MASK,
					axis_enable);
}

static int lsm303agr_acc_init(struct lsm303agr_data *cdata)
{
	int err;
	

	err = lsm303agr_core_set_enable(cdata->sdata, false);
	if (err < 0)
		return err;
		

	err = lsm303agr_core_set_fs(cdata->sdata,
					cdata->sensor_conf->fs.default_fs);
	if (err < 0)
		return err;

	/*
	 * Enable block data update feature.
	 */
	err = lsm303agr_write_register(cdata,
					LSM303AGR_CTRL4_ACC_ADDR,
					LSM303AGR_CTRL4_ACC_BDU_MASK,
					LSM303AGR_EN_BIT);
	if (err < 0)
		return err;

	return 0;
}


static LSM303AGR_CORE_SAMPLE_FREQ_ATTR();
static LSM303AGR_CORE_SAMPLE_FREQ_AVAIL_ATTR();
static LSM303AGR_CORE_SCALE_AVAIL_ATTR(in_accel_scale_available);

static struct attribute *lsm303agr_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	NULL,
};

static const struct attribute_group lsm303agr_acc_attribute_group = {
	.attrs = lsm303agr_acc_attributes,
};

static const struct iio_info lsm303agr_acc_info = {
	.driver_module = THIS_MODULE,
	.attrs = &lsm303agr_acc_attribute_group,
	.read_raw = &lsm303agr_core_read_raw,
	.write_raw = &lsm303agr_core_write_raw,
};

static struct lsm303agr_ctm_func lsm303agr_acc_ctm_func = {
	.set_enable = lsm303agr_acc_set_enable,
	.set_odr = lsm303agr_core_write_odr,
	.set_axis = lsm303agr_acc_set_axis,
	.set_fs = lsm303agr_core_set_fs,
	.init = lsm303agr_acc_init,
};

int lsm303agr_acc_probe(struct lsm303agr_data *cdata, int irq)
{
	cdata->func = &lsm303agr_acc_ctm_func;
	cdata->sensor_conf = &lsm303agr_acc_conf;

	return lsm303agr_common_probe(cdata, irq, &lsm303agr_acc_info);
}

int lsm303agr_acc_remove(struct lsm303agr_data *cdata, int irq)
{
	lsm303agr_common_remove(cdata, irq);

	dev_info(cdata->dev, "%s: removed\n", cdata->sensor_conf->name);

	return 0;
}

MODULE_DESCRIPTION("STMicroelectronics lsm303agr accel driver");
MODULE_AUTHOR("Giuseppe Barba <giuseppe.barba@st.com>");
MODULE_LICENSE("GPL v2");
