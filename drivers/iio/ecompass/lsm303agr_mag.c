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

#define LSM303AGR_WHO_AM_I_MAG_ADDR		0x4f
#define LSM303AGR_WHO_AM_I_MAG_DEF		0x40

/* Magnetometer control registers */
#define LSM303AGR_CFG_REG_A_MAG_ADDR		0x60
#define LSM303AGR_CFG_REG_A_MAG_ODR_MASK	0x0c
#define LSM303AGR_CFG_REG_A_MAG_ODR_10Hz	0x00
#define LSM303AGR_CFG_REG_A_MAG_ODR_20Hz	0x01
#define LSM303AGR_CFG_REG_A_MAG_ODR_50Hz	0x02
#define LSM303AGR_CFG_REG_A_MAG_ODR_100Hz	0x03
#define LSM303AGR_CFG_REG_A_MAG_ODR_COUNT	4
#define LSM303AGR_CFG_REG_A_MAG_MD_MASK		0x03
#define LSM303AGR_CFG_REG_A_MAG_MD_CONT		0x00
#define LSM303AGR_CFG_REG_A_MAG_MD_IDLE		0x03

#define LSM303AGR_CFG_REG_B_MAG_ADDR		0x61
#define LSM303AGR_CFG_REG_B_OFF_CANC_MASK	0x02
#define LSM303AGR_CFG_REG_C_MAG_ADDR		0x62
#define LSM303AGR_CFG_REG_C_MAG_BDU_MASK	0x10
#define LSM303AGR_CFG_REG_C_INT_MAG_MASK	0x01

/* Magnetometer output registers */
#define LSM303AGR_OUTX_L_MAG_ADDR		0x68
#define LSM303AGR_OUTY_L_MAG_ADDR		0x6A
#define LSM303AGR_OUTZ_L_MAG_ADDR		0x6C

#define LSM303AGR_MAG_FS_50GAUSS_GAIN		1500
#define LSM303AGR_MAG_FS_DEFAULT		50

#define LSM303AGR_CHANNEL_SPECS_SIZE		4
#define LSM303AGR_MAG_ODR_TABLE_SIZE		4
#define LSM303AGR_MAG_FS_TABLE_SIZE		1
#define LSM303AGR_MAG_ODR_HZ_DEF		10
#define LSM303AGR_MAG_TURNON_TIME_MS		2

static const
struct iio_chan_spec lsm303agr_mag_chan_spec[LSM303AGR_CHANNEL_SPECS_SIZE] = {
	LSM303AGR_ADD_CHANNEL(IIO_MAGN, 1, 0, IIO_MOD_X, IIO_LE, 16, 16,
				LSM303AGR_OUTX_L_MAG_ADDR, 's'),
	LSM303AGR_ADD_CHANNEL(IIO_MAGN, 1, 1, IIO_MOD_Y, IIO_LE, 16, 16,
				LSM303AGR_OUTY_L_MAG_ADDR, 's'),
	LSM303AGR_ADD_CHANNEL(IIO_MAGN, 1, 2, IIO_MOD_Z, IIO_LE, 16, 16,
				LSM303AGR_OUTZ_L_MAG_ADDR, 's'),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const
struct lsm303agr_odr_table_t lsm303agr_odr_table[LSM303AGR_MAG_ODR_TABLE_SIZE] = {
	[0] = {
		.hz = 10,
		.reg_val = LSM303AGR_CFG_REG_A_MAG_ODR_10Hz,
	},
	[1] = {
		.hz = 20,
		.reg_val = LSM303AGR_CFG_REG_A_MAG_ODR_20Hz,
	},
	[2] = {
		.hz = 50,
		.reg_val = LSM303AGR_CFG_REG_A_MAG_ODR_50Hz,
	},
	[3] = {
		.hz = 100,
		.reg_val = LSM303AGR_CFG_REG_A_MAG_ODR_100Hz,
	},
};

static struct lsm303agr_fs_table_t lsm303agr_fs_table[LSM303AGR_MAG_FS_TABLE_SIZE] = {
	[0] = {
		.gain = LSM303AGR_MAG_FS_50GAUSS_GAIN,
		.urv = 50,
	},
};

static struct lsm303agr_sensor_conf lsm303agr_mag_conf = {
	.name = LSM303AGR_MAGN_DEV_NAME,
	.wai_addr = LSM303AGR_WHO_AM_I_MAG_ADDR,
	.wai_val = LSM303AGR_WHO_AM_I_MAG_DEF,
	.odr = {
		.odr_addr = LSM303AGR_CFG_REG_A_MAG_ADDR,
		.odr_mask = LSM303AGR_CFG_REG_A_MAG_ODR_MASK,
		.odr_table = lsm303agr_odr_table,
		.table_size = LSM303AGR_MAG_ODR_TABLE_SIZE,
		.default_odr = LSM303AGR_MAG_ODR_HZ_DEF,
	},
	.fs = {
		.default_fs = LSM303AGR_MAG_FS_DEFAULT,
		.table_size = LSM303AGR_MAG_FS_TABLE_SIZE,
		.fs_table = lsm303agr_fs_table,
	},
	.chan_spec = lsm303agr_mag_chan_spec,
	.chan_spec_size = LSM303AGR_CHANNEL_SPECS_SIZE,
	.irq_addr = LSM303AGR_CFG_REG_C_MAG_ADDR,
	.irq_mask = LSM303AGR_CFG_REG_C_INT_MAG_MASK,
	.turn_on_time_ms = LSM303AGR_MAG_TURNON_TIME_MS,
	.output_size = LSM303AGR_OUT_DATA_SIZE,
};

static int lsm303agr_mag_set_enable(struct lsm303agr_sensor_data *sdata, bool state)
{
	u8 mode;

	if (state)
		mode = LSM303AGR_CFG_REG_A_MAG_MD_CONT;
	else
		mode = LSM303AGR_CFG_REG_A_MAG_MD_IDLE;

	return lsm303agr_write_register(sdata->cdata,
					LSM303AGR_CFG_REG_A_MAG_ADDR,
					LSM303AGR_CFG_REG_A_MAG_MD_MASK,
					mode);
}

static int lsm303agr_mag_set_axis(struct lsm303agr_sensor_data *sdata, u8 axis_enable)
{
	return 0;
}

static int lsm303agr_mag_set_fs(struct lsm303agr_sensor_data *sdata, u32 fs)
{
	return -EINVAL;
}

static int lsm303agr_mag_init(struct lsm303agr_data *cdata)
{
	int err;
	

	err = lsm303agr_core_set_enable(cdata->sdata, false);
	if (err < 0)
		return err;

	/*
	 * Enable block data update feature.
	 */
	err = lsm303agr_write_register(cdata,
					LSM303AGR_CFG_REG_C_MAG_ADDR,
					LSM303AGR_CFG_REG_C_MAG_BDU_MASK,
					LSM303AGR_EN_BIT);
	if (err < 0)
		return err;

	err = lsm303agr_write_register(cdata,
					LSM303AGR_CFG_REG_B_MAG_ADDR,
					LSM303AGR_CFG_REG_B_OFF_CANC_MASK,
					LSM303AGR_EN_BIT);
	if (err < 0)
		return err;

	return 0;
}

static LSM303AGR_CORE_SAMPLE_FREQ_ATTR();
static LSM303AGR_CORE_SAMPLE_FREQ_AVAIL_ATTR();
static LSM303AGR_CORE_SCALE_AVAIL_ATTR(in_magn_scale_available);

static struct attribute *lsm303agr_mag_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_magn_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	NULL,
};

static const struct attribute_group lsm303agr_mag_attribute_group = {
	.attrs = lsm303agr_mag_attributes,
};

static const struct iio_info lsm303agr_mag_info = {
	.driver_module = THIS_MODULE,
	.attrs = &lsm303agr_mag_attribute_group,
	.read_raw = &lsm303agr_core_read_raw,
	.write_raw = &lsm303agr_core_write_raw,
};


static struct lsm303agr_ctm_func lsm303agr_mag_ctm_func = {
	.set_enable = lsm303agr_mag_set_enable,
	.set_odr = lsm303agr_core_write_odr,
	.set_axis = lsm303agr_mag_set_axis,
	.set_fs = lsm303agr_mag_set_fs,
	.init = lsm303agr_mag_init,
};

int lsm303agr_mag_probe(struct lsm303agr_data *cdata, int irq)
{
	cdata->func = &lsm303agr_mag_ctm_func;
	cdata->sensor_conf = &lsm303agr_mag_conf;

	return lsm303agr_common_probe(cdata, irq, &lsm303agr_mag_info);
}

int lsm303agr_mag_remove(struct lsm303agr_data *cdata, int irq)
{
	lsm303agr_common_remove(cdata, irq);

	dev_info(cdata->dev, "%s: removed\n", cdata->sensor_conf->name);

	return 0;
}

MODULE_DESCRIPTION("STMicroelectronics lsm303agr magn driver");
MODULE_AUTHOR("Giuseppe Barba <giuseppe.barba@st.com>");
MODULE_LICENSE("GPL v2");
