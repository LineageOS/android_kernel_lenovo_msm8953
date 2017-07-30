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
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <asm/unaligned.h>

#include <linux/iio/common/st_sensors.h>
#include "st_lsm6dsm.h"

#define MS_TO_NS(msec)				((msec) * 1000 * 1000)

#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#define MIN_BNZ(a, b)				(((a) < (b)) ? ((a == 0) ? \
						(b) : (a)) : ((b == 0) ? \
						(a) : (b)))

#define ST_LSM6DSM_EN_BIT			0x01
#define ST_LSM6DSM_DIS_BIT			0x00

#define ST_LSM6DSM_FUNC_CFG_ACCESS_ADDR		0x01
#define ST_LSM6DSM_FUNC_CFG_EN_MASK		0x80

#define ST_LSM6DSM_FIFO_CTRL1_ADDR		0x06

#define ST_LSM6DSM_FIFO_CTRL2_ADDR		0x07
#define ST_LSM6DSM_FIFO_CTRL2_FTH_MASK		0x07

#define ST_LSM6DSM_FIFO_CTRL3_ADDR		0x08
#define ST_LSM6DSM_FIFO_CTRL3_DEC_XL_MASK	0x07
#define ST_LSM6DSM_FIFO_CTRL3_DEC_G_MASK	0x38

#define ST_LSM6DSM_FIFO_CTRL4_ADDR		0x09
#define ST_LSM6DSM_FIFO_CTRL4_DEC_DS3_MASK	0x07
#define ST_LSM6DSM_FIFO_CTRL4_DEC_DS4_MASK	0x38

#define ST_LSM6DSM_FIFO_CTRL5_ADDR		0x0a
#define ST_LSM6DSM_FIFO_CTRL5_MODE_MASK		0x07
#define ST_LSM6DSM_FIFO_CTRL5_ODR_MASK		0x78

#define ST_LSM6DSM_INT1_ADDR			0x0d
#define ST_LSM6DSM_INT1_STEP_DETECTOR_MASK	0x80
#define ST_LSM6DSM_INT1_SIGN_MOT_MASK		0x40
#define ST_LSM6DSM_INT1_FTH_MASK		0x08

#define ST_LSM6DSM_INT2_ADDR			0x0e
#define ST_LSM6DSM_INT2_STEP_C_MASK		0x80

#define ST_LSM6DSM_WHO_AM_I_ADDR		0x0f

#define ST_LSM6DSM_CTRL1_XL_ADDR		0x10
#define ST_LSM6DSM_CTRL1_XL_ODR_MASK		0xf0
#define ST_LSM6DSM_CTRL1_XL_FS_MASK		0x0c

#define ST_LSM6DSM_CTRL2_G_ADDR			0x11
#define ST_LSM6DSM_CTRL2_G_ODR_MASK		0xf0
#define ST_LSM6DSM_CTRL2_G_FS_MASK		0x0c

#define ST_LSM6DSM_CTRL3_ADDR			0x12
#define ST_LSM6DSM_CTRL3_RESET_MASK		0x01
#define ST_LSM6DSM_CTRL3_BDU_MASK		0x40

#define ST_LSM6DSM_CTRL4_ADDR			0x13
#define ST_LSM6DSM_CTRL4_INT2_ON_1_MASK		0x20

#define ST_LSM6DSM_CTRL5_ADDR			0x14
#define ST_LSM6DSM_CTRL5_ST1_XL_MASK		0x03
#define ST_LSM6DSM_CTRL5_ST1_G_MASK		0x0c

#define ST_LSM6DSM_CTRL7_ADDR			0x16
#define ST_LSM6DSM_CTRL7_ROUNDING_MASK		0x04

#define ST_LSM6DSM_CTRL10_ADDR			0x19
#define ST_LSM6DSM_CTRL10_SIGNMOTION_MASK	0x01
#define ST_LSM6DSM_CTRL10_PEDO_RST_STEP_MASK	0x02
#define ST_LSM6DSM_CTRL10_FUNC_MASK		0x04
#define ST_LSM6DSM_CTRL10_TILT_MASK		0x08
#define ST_LSM6DSM_CTRL10_PEDO_MASK		0x10

#define ST_LSM6DSM_M_CFG_ADDR			0x1a
#define ST_LSM6DSM_M_CFG_PASS_TROUGH_MASK	0x04
#define ST_LSM6DSM_M_CFG_PULLUP_MASK		0x08
#define ST_LSM6DSM_M_CFG_MASTER_MASK		0x01
#define ST_LSM6DSM_M_CFG_START_CFG_MASK		0x10

#define ST_LSM6DSM_OUTX_L_G_ADDR		0x22
#define ST_LSM6DSM_OUTY_L_G_ADDR		0x24
#define ST_LSM6DSM_OUTZ_L_G_ADDR		0x26
#define ST_LSM6DSM_OUTX_L_XL_ADDR		0x28
#define ST_LSM6DSM_OUTY_L_XL_ADDR		0x2a
#define ST_LSM6DSM_OUTZ_L_XL_ADDR		0x2c
#define ST_LSM6DSM_STEP_COUNTER_L_ADDR		0x4b

#define ST_LSM6DSM_TAPCFG_ADDR			0x58
#define ST_LSM6DSM_TAPCFG_LIR_MASK		0x01
#define ST_LSM6DSM_TAPCFG_INTEN_MASK		0x80

#define ST_LSM6DSM_MD1_ADDR			0x5e
#define ST_LSM6DSM_MD1_INT1_TILT_MASK		0x02

#define ST_LSM6DSM_STEP_COUNT_DELTA_ADDR	0x15
#define ST_LSM6DSM_STEP_COUNT_DELTA_MASK	0xff

#define ST_LSM6DSM_WHO_AM_I_VAL			0x6a
#define ST_LSM6DSM_FIFO_ODR_MAX_VAL		0x08
#define ST_LSM6DSM_FIFO_ODR_OFF_VAL		0x00
#define ST_LSM6DSM_FIFO_MODE_BYPASS_VAL		0x00
#define ST_LSM6DSM_FIFO_MODE_CONTINUOS_VAL	0x06
#define ST_LSM6DSM_SELF_TEST_DISABLED_VAL	0x00
#define ST_LSM6DSM_SELF_TEST_POS_SIGN_VAL	0x01
#define ST_LSM6DSM_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define ST_LSM6DSM_SELF_TEST_NEG_GYRO_SIGN_VAL	0x03
#define ST_LSM6DSM_ODR_POWER_OFF_VAL		0x00
#define ST_LSM6DSM_ODR_1HZ_VAL			0x0e
#define ST_LSM6DSM_ODR_13HZ_VAL			0x01
#define ST_LSM6DSM_ODR_26HZ_VAL			0x02
#define ST_LSM6DSM_ODR_52HZ_VAL			0x03
#define ST_LSM6DSM_ODR_104HZ_VAL		0x04
#define ST_LSM6DSM_ODR_208HZ_VAL		0x05
#define ST_LSM6DSM_ODR_416HZ_VAL		0x06
#define ST_LSM6DSM_FS_LIST_NUM			4
#define ST_LSM6DSM_ACCEL_FS_2G_VAL		0x00
#define ST_LSM6DSM_ACCEL_FS_4G_VAL		0x02
#define ST_LSM6DSM_ACCEL_FS_8G_VAL		0x03
#define ST_LSM6DSM_ACCEL_FS_16G_VAL		0x01
#define ST_LSM6DSM_ACCEL_FS_2G_GAIN		IIO_G_TO_M_S_2(61)
#define ST_LSM6DSM_ACCEL_FS_4G_GAIN		IIO_G_TO_M_S_2(122)
#define ST_LSM6DSM_ACCEL_FS_8G_GAIN		IIO_G_TO_M_S_2(244)
#define ST_LSM6DSM_ACCEL_FS_16G_GAIN		IIO_G_TO_M_S_2(488)
#define ST_LSM6DSM_ACCEL_STD			3
#define ST_LSM6DSM_GYRO_FS_245_VAL		0x00
#define ST_LSM6DSM_GYRO_FS_500_VAL		0x01
#define ST_LSM6DSM_GYRO_FS_1000_VAL		0x02
#define ST_LSM6DSM_GYRO_FS_2000_VAL		0x03
#define ST_LSM6DSM_GYRO_FS_245_GAIN		IIO_DEGREE_TO_RAD(4375)
#define ST_LSM6DSM_GYRO_FS_500_GAIN		IIO_DEGREE_TO_RAD(8750)
#define ST_LSM6DSM_GYRO_FS_1000_GAIN		IIO_DEGREE_TO_RAD(17500)
#define ST_LSM6DSM_GYRO_FS_2000_GAIN		IIO_DEGREE_TO_RAD(70000)
#define ST_LSM6DSM_GYRO_STD			6
#define ST_LSM6DSM_STEP_COUNTER_DEFUALT_VAL	0
#define ST_LSM6DSM_FUNC_MIN_FREQ_HZ		26
#define ST_LSM6DSM_POWER_OFF_HZ			0
#define ST_LSM6DSM_MAX_CHANNEL_SPEC		5
#define ST_LSM6DSM_MIN_DURATION_MS		1638

#define ST_LSM6DSM_TEST_REG_ADDR		0x00
#define ST_LSM6DSM_START_INJECT_XL_MASK		0x08
#define ST_LSM6DSM_INJECT_XL_X_ADDR		0x06

#define ST_LSM6DSM_26HZ_INJECT_NS_UP		(ULLONG_MAX)
#define ST_LSM6DSM_26HZ_INJECT_NS_DOWN		(25641026LL)
#define ST_LSM6DSM_52HZ_INJECT_NS_UP		ST_LSM6DSM_26HZ_INJECT_NS_DOWN
#define ST_LSM6DSM_52HZ_INJECT_NS_DOWN		(12820512LL)
#define ST_LSM6DSM_104HZ_INJECT_NS_UP		ST_LSM6DSM_52HZ_INJECT_NS_DOWN
#define ST_LSM6DSM_104HZ_INJECT_NS_DOWN		(6410256LL)
#define ST_LSM6DSM_208HZ_INJECT_NS_UP		ST_LSM6DSM_104HZ_INJECT_NS_DOWN
#define ST_LSM6DSM_208HZ_INJECT_NS_DOWN		(0)

#define ST_LSM6DSM_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
			st_lsm6dsm_sysfs_get_sampling_frequency, \
			st_lsm6dsm_sysfs_set_sampling_frequency)

#define ST_LSM6DSM_DEV_ATTR_SAMP_FREQ_AVAIL() \
		IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
			st_lsm6dsm_sysfs_sampling_frequency_avail)

#define ST_LSM6DSM_DEV_ATTR_SCALE_AVAIL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			st_lsm6dsm_sysfs_scale_avail, NULL , 0);


static struct st_lsm6dsm_selftest_table {
	char *string_mode;
	u8 accel_value;
	u8 gyro_value;
	u8 accel_mask;
	u8 gyro_mask;
} st_lsm6dsm_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_LSM6DSM_SELF_TEST_DISABLED_VAL,
		.gyro_value = ST_LSM6DSM_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_LSM6DSM_SELF_TEST_POS_SIGN_VAL,
		.gyro_value = ST_LSM6DSM_SELF_TEST_POS_SIGN_VAL
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_LSM6DSM_SELF_TEST_NEG_ACCEL_SIGN_VAL,
		.gyro_value = ST_LSM6DSM_SELF_TEST_NEG_GYRO_SIGN_VAL
	},
};

struct st_lsm6dsm_odr_reg {
	unsigned int hz;
	u8 value;
};

static struct st_lsm6dsm_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct st_lsm6dsm_odr_reg odr_avl[5];
} st_lsm6dsm_odr_table = {
	.addr[ST_INDIO_DEV_ACCEL] = ST_LSM6DSM_CTRL1_XL_ADDR,
	.mask[ST_INDIO_DEV_ACCEL] = ST_LSM6DSM_CTRL1_XL_ODR_MASK,
	.addr[ST_INDIO_DEV_GYRO] = ST_LSM6DSM_CTRL2_G_ADDR,
	.mask[ST_INDIO_DEV_GYRO] = ST_LSM6DSM_CTRL2_G_ODR_MASK,
	.odr_avl = {
// 		{ .hz = 1, .value = ST_LSM6DSM_ODR_1HZ_VAL },
// 		{ .hz = 13, .value = ST_LSM6DSM_ODR_13HZ_VAL },
		{ .hz = 26, .value = ST_LSM6DSM_ODR_26HZ_VAL },
		{ .hz = 52, .value = ST_LSM6DSM_ODR_52HZ_VAL },
		{ .hz = 104, .value = ST_LSM6DSM_ODR_104HZ_VAL },
		{ .hz = 208, .value = ST_LSM6DSM_ODR_208HZ_VAL },
		{ .hz = 416, .value = ST_LSM6DSM_ODR_416HZ_VAL },
	},
};

struct st_lsm6dsm_fs_reg {
	unsigned int gain;
	u8 value;
};

static struct st_lsm6dsm_fs_table {
	u8 addr;
	u8 mask;
	struct st_lsm6dsm_fs_reg fs_avl[ST_LSM6DSM_FS_LIST_NUM];
} st_lsm6dsm_fs_table[ST_INDIO_DEV_NUM] = {
	[ST_INDIO_DEV_ACCEL] = {
		.addr = ST_LSM6DSM_CTRL1_XL_ADDR,
		.mask = ST_LSM6DSM_CTRL1_XL_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DSM_ACCEL_FS_2G_GAIN,
					.value = ST_LSM6DSM_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DSM_ACCEL_FS_4G_GAIN,
					.value = ST_LSM6DSM_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DSM_ACCEL_FS_8G_GAIN,
					.value = ST_LSM6DSM_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DSM_ACCEL_FS_16G_GAIN,
					.value = ST_LSM6DSM_ACCEL_FS_16G_VAL },
	},
	[ST_INDIO_DEV_GYRO] = {
		.addr = ST_LSM6DSM_CTRL2_G_ADDR,
		.mask = ST_LSM6DSM_CTRL2_G_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DSM_GYRO_FS_245_GAIN,
					.value = ST_LSM6DSM_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DSM_GYRO_FS_500_GAIN,
					.value = ST_LSM6DSM_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DSM_GYRO_FS_1000_GAIN,
					.value = ST_LSM6DSM_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DSM_GYRO_FS_2000_GAIN,
					.value = ST_LSM6DSM_GYRO_FS_2000_VAL },
	}
};

static const struct iio_event_spec singol_thr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
};

const struct iio_event_spec lsm6dsm_fifo_flush_event = {
	.type = IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct st_lsm6dsm_sensors_table {
	const char *name;
	const u8 iio_channel_size;
	const struct iio_chan_spec iio_channel[ST_LSM6DSM_MAX_CHANNEL_SPEC];
} st_lsm6dsm_sensors_table[ST_INDIO_DEV_NUM] = {
	[ST_INDIO_DEV_ACCEL] = {
		.name = "accel",
		.iio_channel = {
			ST_LSM6DSM_LSM_CHANNELS(IIO_ACCEL, 0, IIO_MOD_X,
				16, 16, ST_LSM6DSM_OUTX_L_XL_ADDR, 's'),
			ST_LSM6DSM_LSM_CHANNELS(IIO_ACCEL, 1, IIO_MOD_Y,
				16, 16, ST_LSM6DSM_OUTY_L_XL_ADDR, 's'),
			ST_LSM6DSM_LSM_CHANNELS(IIO_ACCEL, 2, IIO_MOD_Z,
				16, 16, ST_LSM6DSM_OUTZ_L_XL_ADDR, 's'),
			ST_LSM6DSM_FLUSH_CHANNEL(IIO_ACCEL),
			IIO_CHAN_SOFT_TIMESTAMP(3)
		},
		.iio_channel_size = ST_LSM6DSM_MAX_CHANNEL_SPEC,
	},
	[ST_INDIO_DEV_GYRO] = {
		.name = "gyro",
		.iio_channel = {
			ST_LSM6DSM_LSM_CHANNELS(IIO_ANGL_VEL, 0, IIO_MOD_X,
				16, 16, ST_LSM6DSM_OUTX_L_G_ADDR, 's'),
			ST_LSM6DSM_LSM_CHANNELS(IIO_ANGL_VEL, 1, IIO_MOD_Y,
				16, 16, ST_LSM6DSM_OUTY_L_G_ADDR, 's'),
			ST_LSM6DSM_LSM_CHANNELS(IIO_ANGL_VEL, 2, IIO_MOD_Z,
				16, 16, ST_LSM6DSM_OUTZ_L_G_ADDR, 's'),
			ST_LSM6DSM_FLUSH_CHANNEL(IIO_ANGL_VEL),
			IIO_CHAN_SOFT_TIMESTAMP(3)
		},
		.iio_channel_size = ST_LSM6DSM_MAX_CHANNEL_SPEC,
	},
	[ST_INDIO_DEV_SIGN_MOTION] = {
		.name = "sign_motion",
		.iio_channel = {
			{
				.type = IIO_SIGN_MOTION,
				.modified = 0,
				.scan_index = -1,
				.indexed = -1,
				.event_spec = &singol_thr_event,
				.num_event_specs = 1,
			},
			IIO_CHAN_SOFT_TIMESTAMP(0),
		},
		.iio_channel_size = 2,
	},
	[ST_INDIO_DEV_STEP_COUNTER] = {
		.name = "step_c",
		.iio_channel = {
			{
				.type = IIO_STEP_COUNTER,
				.modified = 0,
				.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
				.address = ST_LSM6DSM_STEP_COUNTER_L_ADDR,
				.scan_type = {
					.sign = 'u',
					.realbits = 16,
					.storagebits = 16,
					.endianness = IIO_LE,
				},
			},
			ST_LSM6DSM_FLUSH_CHANNEL(IIO_STEP_COUNTER),
			IIO_CHAN_SOFT_TIMESTAMP(1)
		},
		.iio_channel_size = 3,
	},
	[ST_INDIO_DEV_STEP_DETECTOR] = {
		.name = "step_d",
		.iio_channel = {
			ST_LSM6DSM_FLUSH_CHANNEL(IIO_STEP_DETECTOR),
			IIO_CHAN_SOFT_TIMESTAMP(0),
		},
		.iio_channel_size = 2,
	},
	[ST_INDIO_DEV_TILT] = {
		.name = "tilt",
		.iio_channel = {
			ST_LSM6DSM_FLUSH_CHANNEL(IIO_TILT),
			IIO_CHAN_SOFT_TIMESTAMP(0),
		},
		.iio_channel_size = 2,
	},
};

int st_lsm6dsm_write_data_with_mask(struct lsm6dsm_data *cdata, u8 reg_addr,
						u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}
EXPORT_SYMBOL(st_lsm6dsm_write_data_with_mask);

static int st_lsm6dsm_write_func_reg(struct lsm6dsm_data *cdata, u8 reg_addr, u8 mask,
							u8 data)
{
	int err, err2;

	mutex_lock(&cdata->bank_registers_lock);
	err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_FUNC_CFG_ACCESS_ADDR,
						ST_LSM6DSM_FUNC_CFG_EN_MASK,
						ST_LSM6DSM_EN_BIT, false);
	if (err < 0)
		goto st_lsm6dsm_write_func_reg_mutex_unlock;

	err = st_lsm6dsm_write_data_with_mask(cdata, reg_addr, mask, data,
									false);

	do {
		err2 = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_FUNC_CFG_ACCESS_ADDR,
						ST_LSM6DSM_FUNC_CFG_EN_MASK,
						ST_LSM6DSM_DIS_BIT, false);

		if (err2 < 0)
			msleep(50);
	} while (err2 < 0);
	mutex_unlock(&cdata->bank_registers_lock);
	return (err < 0) ? err : 0;

st_lsm6dsm_write_func_reg_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
static int st_lsm6dsm_enable_sensor_hub(struct lsm6dsm_data *cdata, bool enable)
{
	int err;

	if (enable) {
		if (cdata->sensors_enabled & ST_LSM6DSM_EXT_SENSORS) {
			err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_M_CFG_ADDR,
						ST_LSM6DSM_M_CFG_START_CFG_MASK,
						ST_LSM6DSM_DIS_BIT, true);
			if (err < 0)
				return err;
		}

		if (cdata->sensors_enabled & ST_LSM6DSM_EMBED_FUNCTIONS) {
			err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_CTRL10_ADDR,
						ST_LSM6DSM_CTRL10_FUNC_MASK,
						ST_LSM6DSM_EN_BIT, true);
			if (err < 0)
				return err;
		}

		if (cdata->sensors_enabled & ST_LSM6DSM_EXT_SENSORS) {
			err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_M_CFG_ADDR,
						ST_LSM6DSM_M_CFG_MASTER_MASK,
						ST_LSM6DSM_EN_BIT, true);
			if (err < 0)
				return err;
		}
	} else {
		err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_M_CFG_ADDR,
						ST_LSM6DSM_M_CFG_START_CFG_MASK,
						ST_LSM6DSM_EN_BIT, true);
		if (err < 0)
			return err;

		usleep_range(1500, 4000);

		err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_M_CFG_ADDR,
						ST_LSM6DSM_M_CFG_MASTER_MASK,
						ST_LSM6DSM_DIS_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_CTRL10_ADDR,
						ST_LSM6DSM_CTRL10_FUNC_MASK,
						ST_LSM6DSM_DIS_BIT, true);
		if (err < 0)
			return err;
	}

	return 0;
}

int st_lsm6dsm_enable_passthrough(struct lsm6dsm_data *cdata, bool enable)
{
	int err;
	u8 reg_value;

	if (enable)
		reg_value = ST_LSM6DSM_EN_BIT;
	else
		reg_value = ST_LSM6DSM_DIS_BIT;

	if (enable) {
		err = st_lsm6dsm_enable_sensor_hub(cdata, false);
		if (err < 0)
			return err;

#ifdef CONFIG_ST_LSM6DSM_ENABLE_INTERNAL_PULLUP
		err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_M_CFG_ADDR,
						ST_LSM6DSM_M_CFG_PULLUP_MASK,
						ST_LSM6DSM_DIS_BIT, true);
		if (err < 0)
			return err;
#endif /* CONFIG_ST_LSM6DSM_ENABLE_INTERNAL_PULLUP */
	}

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_M_CFG_ADDR,
					ST_LSM6DSM_M_CFG_PASS_TROUGH_MASK,
					reg_value, enable);
	if (err < 0)
		return err;

	if (!enable) {
#ifdef CONFIG_ST_LSM6DSM_ENABLE_INTERNAL_PULLUP
		err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_M_CFG_ADDR,
						ST_LSM6DSM_M_CFG_PULLUP_MASK,
						ST_LSM6DSM_EN_BIT, true);
		if (err < 0)
			return err;
#endif /* CONFIG_ST_LSM6DSM_ENABLE_INTERNAL_PULLUP */

		err = st_lsm6dsm_enable_sensor_hub(cdata, true);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_enable_passthrough);
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

int st_lsm6dsm_set_fifo_mode(struct lsm6dsm_data *cdata, enum fifo_mode fm)
{
	int err, i;
	u8 reg_value;
	bool enable_fifo;
	struct timespec ts;

	switch (fm) {
	case BYPASS:
		reg_value = ST_LSM6DSM_FIFO_MODE_BYPASS_VAL;
		enable_fifo = false;
		break;
	case CONTINUOS:
		reg_value = ST_LSM6DSM_FIFO_MODE_CONTINUOS_VAL;
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_FIFO_CTRL5_ADDR,
						ST_LSM6DSM_FIFO_CTRL5_MODE_MASK,
						reg_value, true);
	if (err < 0)
		return err;

	if (enable_fifo) {
		get_monotonic_boottime(&ts);

		for (i = 0; i < ST_FIFO_DEV_NUM; i++)
			cdata->fifo_cfg[i].timestamp = timespec_to_ns(&ts);
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_set_fifo_mode);

static int st_lsm6dsm_lookup_odr_value(unsigned int odr)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsm_odr_table.odr_avl); i++) {
		if (st_lsm6dsm_odr_table.odr_avl[i].hz == odr)
			break;
	}
	if (i == ARRAY_SIZE(st_lsm6dsm_odr_table.odr_avl))
		return -1;

	return st_lsm6dsm_odr_table.odr_avl[i].value;
}

static int st_lsm6dsm_write_odr(struct lsm6dsm_sensor_data *sdata,
							unsigned int odr)
{
	int ret;
	u8 val = 0x0;

	if (odr == ST_LSM6DSM_POWER_OFF_HZ) {
		val = ST_LSM6DSM_ODR_POWER_OFF_VAL;
	} else {
		ret = st_lsm6dsm_lookup_odr_value(odr);
		if (ret < 0)
			return -EINVAL;

		val = (u8)ret;
	}

	return st_lsm6dsm_write_data_with_mask(sdata->cdata,
				st_lsm6dsm_odr_table.addr[sdata->sindex],
				st_lsm6dsm_odr_table.mask[sdata->sindex],
				val, true);
}

static int st_lsm6dsm_set_fifo_params(struct lsm6dsm_data *cdata)
{
	int err;
	u8 decimator, decimator_addr, decimator_mask;
	unsigned int min_odr = 416, max_odr = 0;
	u16 fifo_len, fifo_threshold;
	u16 min_num_pattern, max_num_pattern, j;
	struct lsm6dsm_sensor_data *sdata;
	struct st_lsm6dsm_fifo_cfg *fifo_cfg;
#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	bool force_accel_odr = false;
#endif

	/*
	 * Search for min and max odr values
	 */
	for (j = 0; j < ST_FIFO_DEV_NUM; j++) {
		fifo_cfg = &cdata->fifo_cfg[j];
		if (BIT(fifo_cfg->sindex) & cdata->sensors_enabled) {
			sdata = iio_priv(cdata->indio_dev[fifo_cfg->sindex]);
			if (min_odr > sdata->c_odr)
				min_odr = sdata->c_odr;

			if (max_odr < sdata->c_odr) {
				max_odr = sdata->c_odr;
#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
				if ((j == ST_FIFO_DEV_EXT0) ||
						(j == ST_FIFO_DEV_EXT1))
					force_accel_odr = true;
#endif
			}

			fifo_cfg->fifo_len = sdata->hwfifo_watermark;
		}
	}

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	if (force_accel_odr) {
		/*
		 * Force accel odr to drive sensor hub at the external
		 * frequency requested value.
		 */
		err = st_lsm6dsm_write_odr(sdata, max_odr);
		if (err < 0)
			return err;
	} else {
		/*
		 * Write accel odr to eventually reset previous forced odr value
		 * driven from an external sensor.
		 */
		if (cdata->sensors_enabled & BIT(sdata->sindex)) {
			sdata->fifo_cfg->samples_to_discard =
							ST_LSM6DSM_ACCEL_STD;

			err = st_lsm6dsm_write_odr(sdata, sdata->c_odr);
			if (err < 0)
				return err;
		}
	}
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

	cdata->total_samples_in_pattern = 0;
	for (j = 0, min_num_pattern = 0; j < ST_FIFO_DEV_NUM; j++) {
		fifo_cfg = &cdata->fifo_cfg[j];
		if (BIT(fifo_cfg->sindex) & cdata->sensors_enabled) {
			sdata = iio_priv(cdata->indio_dev[fifo_cfg->sindex]);
			fifo_cfg->samples_in_pattern = sdata->c_odr / min_odr;
			fifo_cfg->num_pattern = MAX(fifo_cfg->fifo_len /
						fifo_cfg->samples_in_pattern,
						1);
			fifo_cfg->deltatime = (1000000000ULL / sdata->c_odr);
			decimator = max_odr / sdata->c_odr;
		} else {
			fifo_cfg->samples_in_pattern = 0;
			decimator = 0;
		}

		switch(j) {
		case ST_FIFO_DEV_ACCEL:
			decimator_addr = ST_LSM6DSM_FIFO_CTRL3_ADDR;
			decimator_mask = ST_LSM6DSM_FIFO_CTRL3_DEC_XL_MASK;
			break;

		case ST_FIFO_DEV_GYRO:
			decimator_addr = ST_LSM6DSM_FIFO_CTRL3_ADDR;
			decimator_mask = ST_LSM6DSM_FIFO_CTRL3_DEC_G_MASK;
			break;

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
		case ST_FIFO_DEV_EXT0:
			decimator_addr = ST_LSM6DSM_FIFO_CTRL4_ADDR;
			decimator_mask = ST_LSM6DSM_FIFO_CTRL4_DEC_DS3_MASK;
			break;

		case ST_FIFO_DEV_EXT1:
			decimator_addr = ST_LSM6DSM_FIFO_CTRL4_ADDR;
			decimator_mask = ST_LSM6DSM_FIFO_CTRL4_DEC_DS4_MASK;
			break;
#endif
		}

		err = st_lsm6dsm_write_data_with_mask(cdata,
							decimator_addr,
							decimator_mask,
							decimator, true);
		if (err < 0)
			return err;

		min_num_pattern = MIN_BNZ(min_num_pattern,
						fifo_cfg->num_pattern);
		cdata->total_samples_in_pattern += fifo_cfg->samples_in_pattern;
	}
	if (cdata->total_samples_in_pattern > 0) {
		max_num_pattern = ST_LSM6DSM_MAX_FIFO_SIZE /
					(cdata->total_samples_in_pattern *
					ST_LSM6DSM_FIFO_ELEMENT_LEN_BYTE);
		if (min_num_pattern > max_num_pattern)
			min_num_pattern = max_num_pattern;
	}

	/*
	 * Now it's time to compute fifo size
	 */
	fifo_len = cdata->total_samples_in_pattern * min_num_pattern *
					ST_LSM6DSM_FIFO_ELEMENT_LEN_BYTE;

	if (fifo_len > 0) {
		fifo_threshold = fifo_len / 2;

		err = cdata->tf->write(cdata, ST_LSM6DSM_FIFO_CTRL1_ADDR,
					1, (u8 *)&fifo_threshold, true);
		if (err < 0)
			return err;

		err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_FIFO_CTRL2_ADDR,
					ST_LSM6DSM_FIFO_CTRL2_FTH_MASK,
					*(((u8 *)&fifo_threshold) + 1), true);
		if (err < 0)
			return err;

		cdata->fifo_threshold = fifo_len;
	}
	kfree(cdata->fifo_data);
	cdata->fifo_data = 0;

	if (fifo_len > 0) {
		cdata->fifo_data = kmalloc(cdata->fifo_threshold, GFP_KERNEL);
		if (!cdata->fifo_data)
			return -ENOMEM;
	}

	return fifo_len;
}

int st_lsm6dsm_reconfigure_fifo(struct lsm6dsm_data *cdata,
						bool disable_irq_and_flush)
{
	int err, fifo_len;

	if (disable_irq_and_flush) {
		disable_irq(cdata->irq);
		st_lsm6dsm_flush_works();
	}

	mutex_lock(&cdata->fifo_lock);
	st_lsm6dsm_read_fifo(cdata, true);

	err = st_lsm6dsm_set_fifo_mode(cdata, BYPASS);
	if (err < 0)
		goto reconfigure_fifo_irq_restore;

	fifo_len = st_lsm6dsm_set_fifo_params(cdata);
	if (fifo_len < 0) {
		err = fifo_len;
		goto reconfigure_fifo_irq_restore;
	}

	if (fifo_len > 0) {
		err = st_lsm6dsm_set_fifo_mode(cdata, CONTINUOS);
		if (err < 0)
			goto reconfigure_fifo_irq_restore;
	}

reconfigure_fifo_irq_restore:
	mutex_unlock(&cdata->fifo_lock);

	if (disable_irq_and_flush)
		enable_irq(cdata->irq);

	return err;
}
EXPORT_SYMBOL(st_lsm6dsm_reconfigure_fifo);

int st_lsm6dsm_set_drdy_irq(struct lsm6dsm_sensor_data *sdata, bool state)
{
	u8 reg_addr, mask, value;

	if (state)
		value = ST_LSM6DSM_EN_BIT;
	else
		value = ST_LSM6DSM_DIS_BIT;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if (sdata->cdata->sensors_enabled & BIT(ST_INDIO_DEV_GYRO))
			return 0;

		reg_addr = ST_LSM6DSM_INT1_ADDR;
		mask = ST_LSM6DSM_INT1_FTH_MASK;
		break;

	case ST_INDIO_DEV_GYRO:
		if (sdata->cdata->sensors_enabled & BIT(ST_INDIO_DEV_ACCEL))
			return 0;

		reg_addr = ST_LSM6DSM_INT1_ADDR;
		mask = ST_LSM6DSM_INT1_FTH_MASK;
		break;

	case ST_INDIO_DEV_SIGN_MOTION:
		reg_addr = ST_LSM6DSM_INT1_ADDR;
		mask = ST_LSM6DSM_INT1_SIGN_MOT_MASK;
		break;

	case ST_INDIO_DEV_STEP_COUNTER:
		reg_addr = ST_LSM6DSM_INT2_ADDR;
		mask = ST_LSM6DSM_INT2_STEP_C_MASK;
		break;

	case ST_INDIO_DEV_STEP_DETECTOR:
		reg_addr = ST_LSM6DSM_INT1_ADDR;
		mask = ST_LSM6DSM_INT1_STEP_DETECTOR_MASK;
		break;

	case ST_INDIO_DEV_TILT:
		reg_addr = ST_LSM6DSM_MD1_ADDR;
		mask = ST_LSM6DSM_MD1_INT1_TILT_MASK;
		break;

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	case ST_INDIO_DEV_EXT0:
		reg_addr = ST_LSM6DSM_INT1_ADDR;
		mask = ST_LSM6DSM_INT1_FTH_MASK;
		break;

	case ST_INDIO_DEV_EXT1:
		reg_addr = ST_LSM6DSM_INT1_ADDR;
		mask = ST_LSM6DSM_INT1_FTH_MASK;
		break;

#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */
	default:
		return -EINVAL;
	}

	return st_lsm6dsm_write_data_with_mask(sdata->cdata,
						reg_addr, mask, value, true);
}
EXPORT_SYMBOL(st_lsm6dsm_set_drdy_irq);

int st_lsm6dsm_enable_accel_dependency(struct lsm6dsm_sensor_data *sdata,
								bool enable)
{
	int err;
	struct lsm6dsm_sensor_data *acc_sdata;

	if ((BIT(sdata->sindex) & ST_LSM6DSM_HW_FUNCTIONS) &&
				(!(sdata->cdata->sensors_enabled &
						BIT(ST_INDIO_DEV_ACCEL)))) {
		acc_sdata =
			iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
		if (enable) {
#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
			if (!sdata->cdata->injection_mode) {
				err = st_lsm6dsm_write_odr(acc_sdata,
						ST_LSM6DSM_FUNC_MIN_FREQ_HZ);
				if (err < 0)
					return err;
			}
#else /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
			err = st_lsm6dsm_write_odr(acc_sdata,
						ST_LSM6DSM_FUNC_MIN_FREQ_HZ);
			if (err < 0)
				return err;
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
		} else {
			err = st_lsm6dsm_write_odr(acc_sdata,
						ST_LSM6DSM_POWER_OFF_HZ);
			if (err < 0)
				return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_enable_accel_dependency);

static int st_lsm6dsm_enable_embed_func(struct lsm6dsm_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!((sdata->cdata->sensors_enabled & ST_LSM6DSM_EMBED_FUNCTIONS) &
						~BIT(sdata->sindex))) {
		if (enable) {
			err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
						ST_LSM6DSM_CTRL10_ADDR,
						ST_LSM6DSM_CTRL10_FUNC_MASK,
						ST_LSM6DSM_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
						ST_LSM6DSM_CTRL10_ADDR,
						ST_LSM6DSM_CTRL10_FUNC_MASK,
						ST_LSM6DSM_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	return st_lsm6dsm_enable_accel_dependency(sdata, enable);
}

static int st_lsm6dsm_enable_pedometer(struct lsm6dsm_sensor_data *sdata,
								bool enable)
{
	u8 value = ST_LSM6DSM_DIS_BIT;

	if ((sdata->cdata->sensors_enabled & ~BIT(sdata->sindex)) &
						ST_LSM6DSM_PEDOMETER_DEPENDENCY)
		return 0;

	if (enable)
		value = ST_LSM6DSM_EN_BIT;

	return st_lsm6dsm_write_data_with_mask(sdata->cdata,
						ST_LSM6DSM_CTRL10_ADDR,
						ST_LSM6DSM_CTRL10_PEDO_MASK,
						value, true);

}

static int st_lsm6dsm_enable_sensors(struct lsm6dsm_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
	case ST_INDIO_DEV_GYRO:
		err = st_lsm6dsm_write_odr(sdata, sdata->c_odr);
		if (err < 0)
			return err;

		if (sdata->sindex == ST_INDIO_DEV_ACCEL)
			sdata->fifo_cfg->samples_to_discard =
							ST_LSM6DSM_ACCEL_STD;
		else
			sdata->fifo_cfg->samples_to_discard =
							ST_LSM6DSM_GYRO_STD;
		break;

	case ST_INDIO_DEV_SIGN_MOTION:
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_CTRL10_ADDR,
					ST_LSM6DSM_CTRL10_SIGNMOTION_MASK,
					ST_LSM6DSM_EN_BIT, true);
		if (err < 0)
			return err;
		break;

	case ST_INDIO_DEV_STEP_COUNTER:
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_TAPCFG_ADDR,
					ST_LSM6DSM_TAPCFG_INTEN_MASK,
					ST_LSM6DSM_EN_BIT, true);
		if (err < 0)
			return err;
	case ST_INDIO_DEV_STEP_DETECTOR:
		err = st_lsm6dsm_enable_pedometer(sdata, true);
		if (err < 0)
			return err;
		break;

	case ST_INDIO_DEV_TILT:
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_CTRL10_ADDR,
					ST_LSM6DSM_CTRL10_TILT_MASK,
					ST_LSM6DSM_EN_BIT, true);
		if (err < 0)
			return err;
		break;

	default:
		return -EINVAL;
	}

	err = st_lsm6dsm_enable_embed_func(sdata, true);
	if (err < 0)
		return err;

	sdata->cdata->sensors_enabled |= BIT(sdata->sindex);

	return 0;
}

static int st_lsm6dsm_disable_sensors(struct lsm6dsm_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if (sdata->cdata->sensors_enabled & ST_LSM6DSM_EMBED_FUNCTIONS)
			err = st_lsm6dsm_write_odr(sdata,
					ST_LSM6DSM_FUNC_MIN_FREQ_HZ);
		else
			err = st_lsm6dsm_write_odr(sdata,
					ST_LSM6DSM_POWER_OFF_HZ);
		if (err < 0)
			return err;
		break;

	case ST_INDIO_DEV_GYRO:
		err = st_lsm6dsm_write_odr(sdata, ST_LSM6DSM_POWER_OFF_HZ);
		if (err < 0)
			return err;
		break;

	case ST_INDIO_DEV_SIGN_MOTION:
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_CTRL10_ADDR,
					ST_LSM6DSM_CTRL10_SIGNMOTION_MASK,
					ST_LSM6DSM_DIS_BIT, true);
		if (err < 0)
			return err;
		break;

	case ST_INDIO_DEV_STEP_COUNTER:
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_TAPCFG_ADDR,
					ST_LSM6DSM_TAPCFG_INTEN_MASK,
					ST_LSM6DSM_DIS_BIT, true);
		if (err < 0)
			return err;
	case ST_INDIO_DEV_STEP_DETECTOR:
		err = st_lsm6dsm_enable_pedometer(sdata, false);
		if (err < 0)
			return err;
		break;

	case ST_INDIO_DEV_TILT:
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_CTRL10_ADDR,
					ST_LSM6DSM_CTRL10_TILT_MASK,
					ST_LSM6DSM_DIS_BIT, true);
		if (err < 0)
			return err;
		break;

	default:
		return -EINVAL;
	}

	err = st_lsm6dsm_enable_embed_func(sdata, false);
	if (err < 0)
		return err;

	sdata->cdata->sensors_enabled &= ~BIT(sdata->sindex);

	return 0;
}

int st_lsm6dsm_set_enable(struct lsm6dsm_sensor_data *sdata, bool enable)
{
	if (enable)
		return st_lsm6dsm_enable_sensors(sdata);
	else
		return st_lsm6dsm_disable_sensors(sdata);
}
EXPORT_SYMBOL(st_lsm6dsm_set_enable);

static int st_lsm6dsm_set_odr(struct lsm6dsm_sensor_data *sdata,
							unsigned int odr)
{
	int ret;

	ret = st_lsm6dsm_lookup_odr_value(odr);
	if (ret < 0)
		return -EINVAL;

	if (sdata->cdata->sensors_enabled & BIT(sdata->sindex)) {
		disable_irq(sdata->cdata->irq);
		st_lsm6dsm_flush_works();

		ret = st_lsm6dsm_write_odr(sdata, odr);
		if (ret < 0) {
			enable_irq(sdata->cdata->irq);
			return ret;
		}

		if (sdata->sindex == ST_INDIO_DEV_ACCEL)
			sdata->fifo_cfg->samples_to_discard =
							ST_LSM6DSM_ACCEL_STD;
		else
			sdata->fifo_cfg->samples_to_discard =
							ST_LSM6DSM_GYRO_STD;

		sdata->c_odr = odr;

		st_lsm6dsm_reconfigure_fifo(sdata->cdata, false);
		enable_irq(sdata->cdata->irq);
	} else
		sdata->c_odr = odr;

	return 0;
}

static int st_lsm6dsm_set_fs(struct lsm6dsm_sensor_data *sdata,
							unsigned int gain)
{
	int err, i;

	for (i = 0; i < ST_LSM6DSM_FS_LIST_NUM; i++) {
		if (st_lsm6dsm_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}
	if (i == ST_LSM6DSM_FS_LIST_NUM)
		return -EINVAL;

	err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
		st_lsm6dsm_fs_table[sdata->sindex].addr,
		st_lsm6dsm_fs_table[sdata->sindex].mask,
		st_lsm6dsm_fs_table[sdata->sindex].fs_avl[i].value, true);
	if (err < 0)
		return err;

	sdata->c_gain[0] = gain;

	return 0;
}

static int st_lsm6dsm_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	u8 outdata[ST_LSM6DSM_BYTE_FOR_CHANNEL];
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6dsm_set_enable(sdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		if (sdata->sindex == ST_INDIO_DEV_ACCEL)
			msleep(40);

		if (sdata->sindex == ST_INDIO_DEV_GYRO)
			msleep(120);

		err = sdata->cdata->tf->read(sdata->cdata, ch->address,
				ST_LSM6DSM_BYTE_FOR_CHANNEL, outdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		err = st_lsm6dsm_set_enable(sdata, false);

		mutex_unlock(&indio_dev->mlock);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->c_gain[0];
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int st_lsm6dsm_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6dsm_set_fs(sdata, val2);
		mutex_unlock(&indio_dev->mlock);
		break;
	default:
		return -EINVAL;
	}

	return err;
}

static int st_lsm6dsm_reset_steps(struct lsm6dsm_data *cdata)
{
	int err;

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_CTRL10_ADDR,
					ST_LSM6DSM_CTRL10_PEDO_RST_STEP_MASK,
					ST_LSM6DSM_EN_BIT, true);
	if (err < 0)
		return err;

	return st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_CTRL10_ADDR,
					ST_LSM6DSM_CTRL10_PEDO_RST_STEP_MASK,
					ST_LSM6DSM_DIS_BIT, true);
}

static int st_lsm6dsm_init_sensor(struct lsm6dsm_data *cdata)
{
	int err, i;
	struct lsm6dsm_sensor_data *sdata;

	mutex_init(&cdata->tb.buf_lock);

	cdata->sensors_enabled = 0;
	cdata->smd_event_ready = false;

#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	cdata->injection_mode = false;
	cdata->last_injection_timestamp = 0;
	cdata->injection_odr = 0;
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */

	err = st_lsm6dsm_write_data_with_mask(cdata,
						ST_LSM6DSM_CTRL3_ADDR,
						ST_LSM6DSM_CTRL3_RESET_MASK,
						ST_LSM6DSM_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_FIFO_CTRL5_ADDR,
					ST_LSM6DSM_FIFO_CTRL5_ODR_MASK,
					ST_LSM6DSM_FIFO_ODR_MAX_VAL, true);
	if (err < 0)
		return err;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		sdata = iio_priv(cdata->indio_dev[i]);

		err = st_lsm6dsm_set_enable(sdata, false);
		if (err < 0)
			return err;

		err = st_lsm6dsm_set_drdy_irq(sdata, false);
		if (err < 0)
			return err;

		switch (sdata->sindex) {
		case ST_INDIO_DEV_ACCEL:
		case ST_INDIO_DEV_GYRO:
			sdata->num_data_channels = 3;
			sdata->c_odr = ST_LSM6DSM_FUNC_MIN_FREQ_HZ;
			sdata->c_gain[0] =
					st_lsm6dsm_fs_table[i].fs_avl[0].gain;
			if (sdata->sindex == ST_INDIO_DEV_ACCEL) {
				sdata->fifo_cfg =
					&cdata->fifo_cfg[ST_FIFO_DEV_ACCEL];
				sdata->fifo_cfg->sindex = ST_INDIO_DEV_ACCEL;
			} else {
				sdata->fifo_cfg =
					&cdata->fifo_cfg[ST_FIFO_DEV_GYRO];
				sdata->fifo_cfg->sindex = ST_INDIO_DEV_GYRO;
			}
			sdata->fifo_cfg->samples_in_pattern = 0;
			sdata->fifo_cfg->samples_to_discard = 0;
			err = st_lsm6dsm_set_fs(sdata, sdata->c_gain[0]);
			if (err < 0)
				return err;

			break;

		case ST_INDIO_DEV_STEP_COUNTER:
			sdata->num_data_channels = 1;
			break;

		case ST_INDIO_DEV_STEP_DETECTOR:
		case ST_INDIO_DEV_SIGN_MOTION:
		case ST_INDIO_DEV_TILT:
			sdata->num_data_channels = 0;
			break;

		default:
			sdata->c_odr = 0;

			break;
		}
	}

	cdata->gyro_selftest_status = 0;
	cdata->accel_selftest_status = 0;

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_TAPCFG_ADDR,
					ST_LSM6DSM_TAPCFG_LIR_MASK,
					ST_LSM6DSM_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_CTRL3_ADDR,
					ST_LSM6DSM_CTRL3_BDU_MASK,
					ST_LSM6DSM_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_CTRL7_ADDR,
					ST_LSM6DSM_CTRL7_ROUNDING_MASK,
					ST_LSM6DSM_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6dsm_write_data_with_mask(cdata,
					ST_LSM6DSM_CTRL4_ADDR,
					ST_LSM6DSM_CTRL4_INT2_ON_1_MASK,
					ST_LSM6DSM_EN_BIT, true);
	if (err < 0)
		return err;

	/*
	 * Reset and configure Step Counter
	 */
	err = st_lsm6dsm_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	err = st_lsm6dsm_write_func_reg(sdata->cdata,
					ST_LSM6DSM_STEP_COUNT_DELTA_ADDR,
					ST_LSM6DSM_STEP_COUNT_DELTA_MASK,
					ST_LSM6DSM_STEP_COUNTER_DEFUALT_VAL);
	if (err < 0)
		return err;

	return 0;
}

static int st_lsm6dsm_set_selftest(struct lsm6dsm_sensor_data *sdata, int index)
{
	int err;
	u8 mode, mask;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		mask = ST_LSM6DSM_CTRL5_ST1_XL_MASK;
		mode = st_lsm6dsm_selftest_table[index].accel_value;
		break;
	case ST_INDIO_DEV_GYRO:
		mask = ST_LSM6DSM_CTRL5_ST1_G_MASK;
		mode = st_lsm6dsm_selftest_table[index].gyro_value;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
					ST_LSM6DSM_CTRL5_ADDR,
					mask, mode, true);
	if (err < 0)
		return err;

	return 0;
}

static ssize_t st_lsm6dsm_sysfs_set_max_delivery_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 duration;
	int err;
	unsigned int max_delivery_rate;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtouint(buf, 10, &max_delivery_rate);
	if (err < 0)
		return -EINVAL;

	if (max_delivery_rate == sdata->c_odr)
		return size;

	duration = max_delivery_rate / ST_LSM6DSM_MIN_DURATION_MS;

	st_lsm6dsm_write_func_reg(sdata->cdata,
					ST_LSM6DSM_STEP_COUNT_DELTA_ADDR,
					ST_LSM6DSM_STEP_COUNT_DELTA_MASK,
					duration);
	if (err < 0)
		return err;

	sdata->c_odr = max_delivery_rate;

	return size;
}

static ssize_t st_lsm6dsm_sysfs_get_max_delivery_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6dsm_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t st_lsm6dsm_sysfs_reset_counter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct lsm6dsm_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	err = st_lsm6dsm_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	return size;
}

static ssize_t st_lsm6dsm_sysfs_get_sampling_frequency(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6dsm_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t st_lsm6dsm_sysfs_set_sampling_frequency(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&indio_dev->mlock);

#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	if (!((sdata->sindex & ST_INDIO_DEV_ACCEL) &&
					sdata->cdata->injection_mode)) {
		if (sdata->c_odr != odr)
			err = st_lsm6dsm_set_odr(sdata, odr);
	}
#else /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
	if (sdata->c_odr != odr)
		err = st_lsm6dsm_set_odr(sdata, odr);
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */

	mutex_unlock(&indio_dev->mlock);

	return err < 0 ? err : size;
}

static ssize_t st_lsm6dsm_sysfs_sampling_frequency_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsm_odr_table.odr_avl); i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
					st_lsm6dsm_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6dsm_sysfs_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm6dsm_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	for (i = 0; i < ST_LSM6DSM_FS_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
			st_lsm6dsm_fs_table[sdata->sindex].fs_avl[i].gain);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6dsm_sysfs_get_selftest_available(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s, %s, %s\n",
				st_lsm6dsm_selftest_table[0].string_mode,
				st_lsm6dsm_selftest_table[1].string_mode,
				st_lsm6dsm_selftest_table[2].string_mode);
}

static ssize_t st_lsm6dsm_sysfs_get_selftest_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 index;
	struct lsm6dsm_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		index = sdata->cdata->accel_selftest_status;
		break;
	case ST_INDIO_DEV_GYRO:
		index = sdata->cdata->gyro_selftest_status;
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%s\n",
				st_lsm6dsm_selftest_table[index].string_mode);
}

static ssize_t st_lsm6dsm_sysfs_set_selftest_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, i;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsm_selftest_table); i++) {
		if (strncmp(buf, st_lsm6dsm_selftest_table[i].string_mode,
								size - 2) == 0)
			break;
	}
	if (i == ARRAY_SIZE(st_lsm6dsm_selftest_table))
		return -EINVAL;

	err = st_lsm6dsm_set_selftest(sdata, i);
	if (err < 0)
		return err;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		sdata->cdata->accel_selftest_status = i;
		break;
	case ST_INDIO_DEV_GYRO:
		sdata->cdata->gyro_selftest_status = i;
		break;
	default:
		return -EINVAL;
	}

	return size;
}

ssize_t st_lsm6dsm_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u64 sensor_last_timestamp, event_type = 0;
	int stype = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		disable_irq(sdata->cdata->irq);
		st_lsm6dsm_flush_works();
	} else {
		mutex_unlock(&indio_dev->mlock);
		return 0;
	}

	sensor_last_timestamp = sdata->fifo_cfg->timestamp;
	mutex_unlock(&indio_dev->mlock);

	mutex_lock(&sdata->cdata->fifo_lock);
	st_lsm6dsm_read_fifo(sdata->cdata, true);

	if (sensor_last_timestamp == sdata->fifo_cfg->timestamp)
		event_type = IIO_EV_DIR_FIFO_EMPTY;
	else
		event_type = IIO_EV_DIR_FIFO_DATA;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		stype = IIO_ACCEL;
		break;

         case ST_INDIO_DEV_GYRO:
		stype = IIO_ANGL_VEL;
		break;

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	case ST_INDIO_DEV_EXT0:
		stype = IIO_MAGN;
		break;

	case ST_INDIO_DEV_EXT1:
		stype = IIO_PRESSURE;

		break;
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

	}

	iio_push_event(indio_dev, IIO_UNMOD_EVENT_CODE(stype,
							-1,
							IIO_EV_TYPE_FIFO_FLUSH,
							event_type),
						sdata->fifo_cfg->timestamp);

	mutex_unlock(&sdata->cdata->fifo_lock);
	enable_irq(sdata->cdata->irq);

	return size;
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_flush_fifo);

ssize_t st_lsm6dsm_sysfs_get_hwfifo_enabled(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", sdata->hwfifo_enabled);
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_get_hwfifo_enabled);

ssize_t st_lsm6dsm_sysfs_set_hwfifo_enabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	bool enable = false;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		mutex_unlock(&indio_dev->mlock);
		return -EBUSY;
	}

	err = strtobool(buf, &enable);
	if (err < 0) {
		mutex_unlock(&indio_dev->mlock);
		return err;
	}

	sdata->hwfifo_enabled = enable;
	if (!enable)
		sdata->hwfifo_watermark = 1;

	mutex_unlock(&indio_dev->mlock);

	return size;
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_set_hwfifo_enabled);

ssize_t st_lsm6dsm_sysfs_get_hwfifo_watermark(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", sdata->hwfifo_watermark);
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_get_hwfifo_watermark);

ssize_t st_lsm6dsm_sysfs_set_hwfifo_watermark(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, watermark = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	if (!sdata->hwfifo_enabled)
		return -EINVAL;

	err = kstrtoint(buf, 10, &watermark);
	if (err < 0)
		return err;

	if ((watermark < 1) || (watermark > ST_LSM6DSM_MAX_FIFO_LENGHT))
		return -EINVAL;

	if (sdata->cdata->sensors_enabled & BIT(sdata->sindex)) {
		disable_irq(sdata->cdata->irq);
		st_lsm6dsm_flush_works();

		sdata->hwfifo_watermark = watermark;

		st_lsm6dsm_reconfigure_fifo(sdata->cdata, false);

		enable_irq(sdata->cdata->irq);
	} else
		sdata->hwfifo_watermark = watermark;

	return size;
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_set_hwfifo_watermark);

ssize_t st_lsm6dsm_sysfs_get_hwfifo_watermark_max(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ST_LSM6DSM_MAX_FIFO_LENGHT);
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_get_hwfifo_watermark_max);

ssize_t st_lsm6dsm_sysfs_get_hwfifo_watermark_min(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}
EXPORT_SYMBOL(st_lsm6dsm_sysfs_get_hwfifo_watermark_min);

#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
static ssize_t st_lsm6dsm_sysfs_set_injection_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, start;
	u8 reg_value = ST_LSM6DSM_POWER_OFF_HZ;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	err = kstrtoint(buf, 10, &start);
	if (err < 0) {
		mutex_unlock(&indio_dev->mlock);
		return err;
	}

	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		mutex_unlock(&indio_dev->mlock);
		return -EBUSY;
	}

	if (start == 0) {
		/* End injection */
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
				ST_LSM6DSM_TEST_REG_ADDR,
				ST_LSM6DSM_START_INJECT_XL_MASK, 0, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		if (sdata->cdata->sensors_enabled & ST_LSM6DSM_EMBED_FUNCTIONS)
			reg_value = ST_LSM6DSM_FUNC_MIN_FREQ_HZ;

		err = st_lsm6dsm_write_odr(sdata, reg_value);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		sdata->cdata->injection_mode = false;
	} else {
		sdata->cdata->last_injection_timestamp = 0;
		sdata->cdata->injection_odr = 0;

		/* Set start injection */
		err = st_lsm6dsm_write_data_with_mask(sdata->cdata,
				ST_LSM6DSM_TEST_REG_ADDR,
				ST_LSM6DSM_START_INJECT_XL_MASK, 1, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		sdata->cdata->injection_mode = true;
	}

	mutex_unlock(&indio_dev->mlock);

	return err;
}

static ssize_t st_lsm6dsm_sysfs_get_injection_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", sdata->cdata->injection_mode);
}

static ssize_t st_lsm6dsm_sysfs_upload_xl_data(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, i;
	s64 timestamp, deltatime;
	u8 sample[3], current_odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsm_sensor_data *sdata = iio_priv(indio_dev);

	for (i = 0; i < 3; i++)
		sample[i] = *(s16 *)(&buf[i * 2]) >> 8;

	timestamp = *(s64 *)(buf + ALIGN(6, sizeof(s64)));

	if (sdata->cdata->last_injection_timestamp > 0) {
		deltatime = timestamp - sdata->cdata->last_injection_timestamp;
		if ((deltatime > ST_LSM6DSM_208HZ_INJECT_NS_DOWN) &&
				(deltatime <= ST_LSM6DSM_208HZ_INJECT_NS_UP)) {
			current_odr = 208;
		} else if ((deltatime > ST_LSM6DSM_104HZ_INJECT_NS_DOWN) &&
				(deltatime <= ST_LSM6DSM_104HZ_INJECT_NS_UP)) {
			current_odr = 104;
		} else if ((deltatime > ST_LSM6DSM_52HZ_INJECT_NS_DOWN) &&
				(deltatime <= ST_LSM6DSM_52HZ_INJECT_NS_UP)) {
			current_odr = 52;
		} else if ((deltatime > ST_LSM6DSM_26HZ_INJECT_NS_DOWN) &&
				(deltatime <= ST_LSM6DSM_26HZ_INJECT_NS_UP)) {
			current_odr = 26;
		} else
			return -EINVAL;

		if (sdata->cdata->injection_odr != current_odr) {
			err = st_lsm6dsm_write_odr(sdata, current_odr);
			if (err < 0)
				return err;

			sdata->cdata->injection_odr = current_odr;
		}
	}

	sdata->cdata->last_injection_timestamp = timestamp;

	err = sdata->cdata->tf->write(sdata->cdata,
			ST_LSM6DSM_INJECT_XL_X_ADDR, 3, (u8 *)sample, false);
	if (err < 0)
		return err;

	usleep_range(1000, 2000);

	return size;
}

static ssize_t st_lsm6dsm_sysfs_get_injection_sensors(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "lsm6dsm_accel");
}

static IIO_DEVICE_ATTR(injection_mode, S_IWUSR | S_IRUGO,
				st_lsm6dsm_sysfs_get_injection_mode,
				st_lsm6dsm_sysfs_set_injection_mode, 0);

static IIO_DEVICE_ATTR(in_accel_injection_raw, S_IWUSR, NULL,
				st_lsm6dsm_sysfs_upload_xl_data, 0);

static IIO_DEVICE_ATTR(injection_sensors, S_IRUGO,
				st_lsm6dsm_sysfs_get_injection_sensors,
				NULL, 0);
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */

static ST_LSM6DSM_DEV_ATTR_SAMP_FREQ();
static ST_LSM6DSM_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_LSM6DSM_DEV_ATTR_SCALE_AVAIL(in_accel_scale_available);
static ST_LSM6DSM_DEV_ATTR_SCALE_AVAIL(in_anglvel_scale_available);

static ST_LSM6DSM_HWFIFO_ENABLED();
static ST_LSM6DSM_HWFIFO_WATERMARK();
static ST_LSM6DSM_HWFIFO_WATERMARK_MIN();
static ST_LSM6DSM_HWFIFO_WATERMARK_MAX();
static ST_LSM6DSM_HWFIFO_FLUSH();

static IIO_DEVICE_ATTR(reset_counter, S_IWUSR,
				NULL, st_lsm6dsm_sysfs_reset_counter, 0);

static IIO_DEVICE_ATTR(max_delivery_rate, S_IWUSR | S_IRUGO,
				st_lsm6dsm_sysfs_get_max_delivery_rate,
				st_lsm6dsm_sysfs_set_max_delivery_rate, 0);

static IIO_DEVICE_ATTR(self_test_available, S_IRUGO,
				st_lsm6dsm_sysfs_get_selftest_available,
				NULL, 0);

static IIO_DEVICE_ATTR(self_test, S_IWUSR | S_IRUGO,
				st_lsm6dsm_sysfs_get_selftest_status,
				st_lsm6dsm_sysfs_set_selftest_status, 0);

static struct attribute *st_lsm6dsm_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test_available.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	&iio_dev_attr_injection_mode.dev_attr.attr,
	&iio_dev_attr_in_accel_injection_raw.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6dsm_accel_attribute_group = {
	.attrs = st_lsm6dsm_accel_attributes,
};

static struct attribute *st_lsm6dsm_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test_available.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsm_gyro_attribute_group = {
	.attrs = st_lsm6dsm_gyro_attributes,
};

static struct attribute *st_lsm6dsm_sign_motion_attributes[] = {
#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6dsm_sign_motion_attribute_group = {
	.attrs = st_lsm6dsm_sign_motion_attributes,
};

static struct attribute *st_lsm6dsm_step_c_attributes[] = {
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_max_delivery_rate.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6dsm_step_c_attribute_group = {
	.attrs = st_lsm6dsm_step_c_attributes,
};

static struct attribute *st_lsm6dsm_step_d_attributes[] = {
#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6dsm_step_d_attribute_group = {
	.attrs = st_lsm6dsm_step_d_attributes,
};

static struct attribute *st_lsm6dsm_tilt_attributes[] = {
#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6dsm_tilt_attribute_group = {
	.attrs = st_lsm6dsm_tilt_attributes,
};

static const struct iio_info st_lsm6dsm_info_table[ST_INDIO_DEV_NUM] = {
	[ST_INDIO_DEV_ACCEL] = {
		.driver_module = THIS_MODULE,
		.attrs = &st_lsm6dsm_accel_attribute_group,
		.read_raw = &st_lsm6dsm_read_raw,
		.write_raw = &st_lsm6dsm_write_raw,
	},
	[ST_INDIO_DEV_GYRO] = {
		.driver_module = THIS_MODULE,
		.attrs = &st_lsm6dsm_gyro_attribute_group,
		.read_raw = &st_lsm6dsm_read_raw,
		.write_raw = &st_lsm6dsm_write_raw,
	},
	[ST_INDIO_DEV_SIGN_MOTION] = {
		.driver_module = THIS_MODULE,
		.attrs = &st_lsm6dsm_sign_motion_attribute_group,
	},
	[ST_INDIO_DEV_STEP_COUNTER] = {
		.driver_module = THIS_MODULE,
		.attrs = &st_lsm6dsm_step_c_attribute_group,
		.read_raw = &st_lsm6dsm_read_raw,
	},
	[ST_INDIO_DEV_STEP_DETECTOR] = {
		.driver_module = THIS_MODULE,
		.attrs = &st_lsm6dsm_step_d_attribute_group,
	},
	[ST_INDIO_DEV_TILT] = {
		.driver_module = THIS_MODULE,
		.attrs = &st_lsm6dsm_tilt_attribute_group,
	},
};

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_lsm6dsm_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = ST_LSM6DSM_TRIGGER_SET_STATE,
};
#define ST_LSM6DSM_TRIGGER_OPS (&st_lsm6dsm_trigger_ops)
#else
#define ST_LSM6DSM_TRIGGER_OPS NULL
#endif

int st_lsm6dsm_common_probe(struct lsm6dsm_data *cdata, int irq)
{
	u8 wai = 0x00;
	int i, n, err;
	struct lsm6dsm_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->fifo_lock);

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	mutex_init(&cdata->passthrough_lock);
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

	cdata->fifo_data = 0;

	err = cdata->tf->read(cdata, ST_LSM6DSM_WHO_AM_I_ADDR, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != ST_LSM6DSM_WHO_AM_I_VAL) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	memset(cdata->fifo_cfg, 0, sizeof(cdata->fifo_cfg));
	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		cdata->indio_dev[i] = iio_device_alloc(sizeof(*sdata));
		if (cdata->indio_dev[i] == NULL) {
			err = -ENOMEM;
			goto iio_device_free;
		}
		sdata = iio_priv(cdata->indio_dev[i]);
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->hwfifo_watermark = 1;
		sdata->hwfifo_enabled = false;
		cdata->indio_dev[i]->modes = INDIO_DIRECT_MODE;
		cdata->indio_dev[i]->name = kasprintf(GFP_KERNEL, "%s_%s",
					cdata->name,
					st_lsm6dsm_sensors_table[i].name);
		cdata->indio_dev[i]->channels =
				st_lsm6dsm_sensors_table[i].iio_channel;
		cdata->indio_dev[i]->num_channels =
				st_lsm6dsm_sensors_table[i].iio_channel_size;
		cdata->indio_dev[i]->info = &st_lsm6dsm_info_table[i];
	}

	if (irq > 0) {
		cdata->irq = irq;
		dev_info(cdata->dev, "driver use DRDY int pin 1\n");
	}

	err = st_lsm6dsm_init_sensor(cdata);
	if (err < 0)
		goto iio_device_free;

	err = st_lsm6dsm_allocate_rings(cdata);
	if (err < 0)
		goto iio_device_free;

	if (irq > 0) {
		err = st_lsm6dsm_allocate_triggers(cdata,
							ST_LSM6DSM_TRIGGER_OPS);
		if (err < 0)
			goto deallocate_ring;
	}

	for (n = 0; n < ST_INDIO_DEV_NUM; n++) {
		err = iio_device_register(cdata->indio_dev[n]);
		if (err)
			goto iio_device_unregister_and_trigger_deallocate;
	}

	err = st_lsm6dsm_i2c_master_probe(cdata);
	if (err < 0)
		goto iio_device_unregister_and_trigger_deallocate;

	device_init_wakeup(cdata->dev, true);

	return 0;

iio_device_unregister_and_trigger_deallocate:
	for (n--; n >= 0; n--)
		iio_device_unregister(cdata->indio_dev[n]);

	if (irq > 0)
		st_lsm6dsm_deallocate_triggers(cdata);
deallocate_ring:
	st_lsm6dsm_deallocate_rings(cdata);
iio_device_free:
	for (i--; i >= 0; i--)
		iio_device_free(cdata->indio_dev[i]);

	return err;
}
EXPORT_SYMBOL(st_lsm6dsm_common_probe);

void st_lsm6dsm_common_remove(struct lsm6dsm_data *cdata, int irq)
{
	int i;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_unregister(cdata->indio_dev[i]);

	if (irq > 0)
		st_lsm6dsm_deallocate_triggers(cdata);

	st_lsm6dsm_deallocate_rings(cdata);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_free(cdata->indio_dev[i]);

	st_lsm6dsm_i2c_master_exit(cdata);
}
EXPORT_SYMBOL(st_lsm6dsm_common_remove);

#ifdef CONFIG_PM
int st_lsm6dsm_common_suspend(struct lsm6dsm_data *cdata)
{
#ifndef CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP
	int err, i;
	u8 tmp_sensors_enabled;
	struct lsm6dsm_sensor_data *sdata;

	tmp_sensors_enabled = cdata->sensors_enabled;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		if ((i == ST_INDIO_DEV_SIGN_MOTION) || (i == ST_INDIO_DEV_TILT))
			continue;

		sdata = iio_priv(cdata->indio_dev[i]);

		err = st_lsm6dsm_set_enable(sdata, false);
		if (err < 0)
			return err;
	}
	cdata->sensors_enabled = tmp_sensors_enabled;
#endif /* CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP */

	if (cdata->sensors_enabled & ST_LSM6DSM_WAKE_UP_SENSORS) {
		if (device_may_wakeup(cdata->dev))
			enable_irq_wake(cdata->irq);
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_common_suspend);

int st_lsm6dsm_common_resume(struct lsm6dsm_data *cdata)
{
#ifndef CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP
	int err, i;
	struct lsm6dsm_sensor_data *sdata;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		if ((i == ST_INDIO_DEV_SIGN_MOTION) || (i == ST_INDIO_DEV_TILT))
			continue;

		sdata = iio_priv(cdata->indio_dev[i]);

		if (BIT(sdata->sindex) & cdata->sensors_enabled) {
			err = st_lsm6dsm_set_enable(sdata, true);
			if (err < 0)
				return err;
		}
	}
#endif /* CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP */

	if (cdata->sensors_enabled & ST_LSM6DSM_WAKE_UP_SENSORS) {
		if (device_may_wakeup(cdata->dev))
			disable_irq_wake(cdata->irq);
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsm_common_resume);
#endif /* CONFIG_PM */

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("STMicroelectronics lsm6dsm core driver");
MODULE_LICENSE("GPL v2");
