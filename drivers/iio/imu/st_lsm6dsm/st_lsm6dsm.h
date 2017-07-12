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

#ifndef ST_LSM6DSM_H
#define ST_LSM6DSM_H

#include <linux/types.h>
#include <linux/iio/trigger.h>

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
#include <linux/i2c.h>
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

#define LSM6DSM_DEV_NAME			"lsm6dsm"

enum {
	ST_INDIO_DEV_ACCEL = 0,
	ST_INDIO_DEV_GYRO,
	ST_INDIO_DEV_SIGN_MOTION,
	ST_INDIO_DEV_STEP_COUNTER,
	ST_INDIO_DEV_STEP_DETECTOR,
	ST_INDIO_DEV_TILT,
	ST_INDIO_DEV_NUM,
};

#define ST_INDIO_DEV_EXT0			ST_INDIO_DEV_NUM
#define ST_INDIO_DEV_EXT1			(ST_INDIO_DEV_NUM + 1)

enum {
	ST_FIFO_DEV_GYRO = 0,
	ST_FIFO_DEV_ACCEL,
#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	ST_FIFO_DEV_EXT0,
	ST_FIFO_DEV_EXT1,
#endif
	ST_FIFO_DEV_NUM,
};

#define ST_LSM6DSM_EXT_SENSORS		(BIT(ST_INDIO_DEV_EXT0) | \
					BIT(ST_INDIO_DEV_EXT1))

#define ST_LSM6DSM_HW_FUNCTIONS		(BIT(ST_INDIO_DEV_STEP_COUNTER) | \
					BIT(ST_INDIO_DEV_TILT) | \
					BIT(ST_INDIO_DEV_SIGN_MOTION) | \
					BIT(ST_INDIO_DEV_STEP_DETECTOR))

#define ST_LSM6DSM_EMBED_FUNCTIONS	((ST_LSM6DSM_HW_FUNCTIONS) | \
					(ST_LSM6DSM_EXT_SENSORS))

#define ST_LSM6DSM_ACCEL_DEPENDENCY	(BIT(ST_INDIO_DEV_ACCEL) | \
					(ST_LSM6DSM_EMBED_FUNCTIONS))

#define ST_LSM6DSM_PEDOMETER_DEPENDENCY (BIT(ST_INDIO_DEV_STEP_COUNTER) | \
					BIT(ST_INDIO_DEV_STEP_DETECTOR))

#define ST_LSM6DSM_USE_BUFFER		(BIT(ST_INDIO_DEV_ACCEL) | \
					BIT(ST_INDIO_DEV_GYRO) | \
					(ST_LSM6DSM_HW_FUNCTIONS))

#ifdef CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP
#define ST_LSM6DSM_WAKE_UP_SENSORS	(BIT(ST_INDIO_DEV_SIGN_MOTION) | \
					BIT(ST_INDIO_DEV_TILT))
#else /* CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP */
#define ST_LSM6DSM_WAKE_UP_SENSORS	(BIT(ST_INDIO_DEV_ACCEL) | \
					BIT(ST_INDIO_DEV_GYRO) | \
					(ST_LSM6DSM_HW_FUNCTIONS) | \
					(ST_LSM6DSM_EXT_SENSORS))
#endif /* CONFIG_ST_LSM6DSM_IIO_SENSORS_WAKEUP */

#define ST_LSM6DSM_TX_MAX_LENGTH		12
#define ST_LSM6DSM_RX_MAX_LENGTH		8193

#define ST_LSM6DSM_BYTE_FOR_CHANNEL		2
#define ST_LSM6DSM_FIFO_ELEMENT_LEN_BYTE	6

/* ST_LSM6DSM_MAX_FIFO_SIZE maximum 4096 byte */
#define ST_LSM6DSM_MAX_FIFO_SIZE		3498
#define ST_LSM6DSM_MAX_FIFO_LENGHT		(ST_LSM6DSM_MAX_FIFO_SIZE / \
					ST_LSM6DSM_FIFO_ELEMENT_LEN_BYTE)

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
#define ST_LSM6DSM_NUM_CLIENTS			2
#else /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */
#define ST_LSM6DSM_NUM_CLIENTS			0
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

#define ST_LSM6DSM_LSM_CHANNELS(device_type, index, mod, sbits, \
							rbits, addr, s) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = s, \
		.realbits = rbits, \
		.shift = sbits - rbits, \
		.storagebits = sbits, \
		.endianness = IIO_LE, \
	}, \
}

extern const struct iio_event_spec lsm6dsm_fifo_flush_event;

#define ST_LSM6DSM_FLUSH_CHANNEL(device_type) \
{ \
	.type = device_type, \
	.modified = 0, \
	.scan_index = -1, \
	.indexed = -1, \
	.event_spec = &lsm6dsm_fifo_flush_event,\
	.num_event_specs = 1, \
}

#define ST_LSM6DSM_HWFIFO_ENABLED() \
	IIO_DEVICE_ATTR(hwfifo_enabled, S_IWUSR | S_IRUGO, \
			st_lsm6dsm_sysfs_get_hwfifo_enabled,\
			st_lsm6dsm_sysfs_set_hwfifo_enabled, 0);

#define ST_LSM6DSM_HWFIFO_WATERMARK() \
	IIO_DEVICE_ATTR(hwfifo_watermark, S_IWUSR | S_IRUGO, \
			st_lsm6dsm_sysfs_get_hwfifo_watermark,\
			st_lsm6dsm_sysfs_set_hwfifo_watermark, 0);

#define ST_LSM6DSM_HWFIFO_WATERMARK_MIN() \
	IIO_DEVICE_ATTR(hwfifo_watermark_min, S_IRUGO, \
			st_lsm6dsm_sysfs_get_hwfifo_watermark_min, NULL, 0);

#define ST_LSM6DSM_HWFIFO_WATERMARK_MAX() \
	IIO_DEVICE_ATTR(hwfifo_watermark_max, S_IRUGO, \
			st_lsm6dsm_sysfs_get_hwfifo_watermark_max, NULL, 0);

#define ST_LSM6DSM_HWFIFO_FLUSH() \
	IIO_DEVICE_ATTR(hwfifo_flush, S_IWUSR, \
			NULL, st_lsm6dsm_sysfs_flush_fifo, 0);

enum fifo_mode {
	BYPASS = 0,
	CONTINUOS,
};

struct st_lsm6dsm_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ST_LSM6DSM_RX_MAX_LENGTH];
	u8 tx_buf[ST_LSM6DSM_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_lsm6dsm_fifo_cfg {
	u8 sindex;
	u8 samples_in_pattern;
	u8 samples_to_discard;
	u16 fifo_len;
	u16 num_pattern;
	int64_t deltatime;
	int64_t timestamp;
};

struct lsm6dsm_data {
	const char *name;

	bool smd_event_ready;

	u8 *fifo_data;
	u8 sensors_enabled;
	u8 gyro_selftest_status;
	u8 accel_selftest_status;
	u16 fifo_threshold;
	u16 total_samples_in_pattern;
	int irq;
	s64 timestamp;

	struct work_struct data_work;
	struct device *dev;
	struct iio_dev *indio_dev[ST_INDIO_DEV_NUM + ST_LSM6DSM_NUM_CLIENTS];
	struct iio_trigger *trig[ST_INDIO_DEV_NUM + ST_LSM6DSM_NUM_CLIENTS];
	struct mutex bank_registers_lock;
	struct mutex fifo_lock;
	struct st_lsm6dsm_fifo_cfg fifo_cfg[ST_FIFO_DEV_NUM];

#ifdef CONFIG_ST_LSM6DSM_XL_DATA_INJECTION
	bool injection_mode;
	s64 last_injection_timestamp;
	u8 injection_odr;
#endif /* CONFIG_ST_LSM6DSM_XL_DATA_INJECTION */

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
	struct i2c_client *master_client[ST_LSM6DSM_NUM_CLIENTS];
	struct mutex passthrough_lock;
	bool ext0_available;
	bool ext1_available;
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

	const struct st_lsm6dsm_transfer_function *tf;
	struct st_lsm6dsm_transfer_buffer tb;
};

struct st_lsm6dsm_transfer_function {
	int (*write) (struct lsm6dsm_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock);
	int (*read) (struct lsm6dsm_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock);
};

struct lsm6dsm_sensor_data {
	struct lsm6dsm_data *cdata;
	struct st_lsm6dsm_fifo_cfg *fifo_cfg;

	unsigned int c_odr;
	unsigned int c_gain[3];

	bool hwfifo_enabled;
	u16 hwfifo_watermark;

	u8 num_data_channels;
	u8 sindex;
	u8 *buffer_data;
};

int st_lsm6dsm_write_data_with_mask(struct lsm6dsm_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock);

void st_lsm6dsm_push_data_with_timestamp(struct lsm6dsm_data *cdata,
					u8 index, u8 *data, int64_t timestamp);

int st_lsm6dsm_common_probe(struct lsm6dsm_data *cdata, int irq);
void st_lsm6dsm_common_remove(struct lsm6dsm_data *cdata, int irq);

int st_lsm6dsm_set_enable(struct lsm6dsm_sensor_data *sdata, bool enable);
int st_lsm6dsm_set_drdy_irq(struct lsm6dsm_sensor_data *sdata, bool state);
int st_lsm6dsm_set_fifo_mode(struct lsm6dsm_data *cdata, enum fifo_mode fm);
int st_lsm6dsm_reconfigure_fifo(struct lsm6dsm_data *cdata,
						bool disable_irq_and_flush);

ssize_t st_lsm6dsm_sysfs_get_hwfifo_enabled(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t st_lsm6dsm_sysfs_set_hwfifo_enabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
ssize_t st_lsm6dsm_sysfs_get_hwfifo_watermark(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t st_lsm6dsm_sysfs_set_hwfifo_watermark(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
ssize_t st_lsm6dsm_sysfs_get_hwfifo_watermark_max(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t st_lsm6dsm_sysfs_get_hwfifo_watermark_min(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t st_lsm6dsm_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

#ifdef CONFIG_IIO_BUFFER
int st_lsm6dsm_allocate_rings(struct lsm6dsm_data *cdata);
void st_lsm6dsm_deallocate_rings(struct lsm6dsm_data *cdata);
int st_lsm6dsm_trig_set_state(struct iio_trigger *trig, bool state);
int st_lsm6dsm_read_fifo(struct lsm6dsm_data *cdata, bool check_fifo_len);
int st_lsm6dsm_read_step_counter(struct lsm6dsm_data *cdata);
#define ST_LSM6DSM_TRIGGER_SET_STATE (&st_lsm6dsm_trig_set_state)
#else /* CONFIG_IIO_BUFFER */
static inline int st_lsm6dsm_allocate_rings(struct lsm6dsm_data *cdata)
{
	return 0;
}
static inline void st_lsm6dsm_deallocate_rings(struct lsm6dsm_data *cdata)
{
}
#define ST_LSM6DSM_TRIGGER_SET_STATE NULL
#endif /* CONFIG_IIO_BUFFER */

#ifdef CONFIG_IIO_TRIGGER
void st_lsm6dsm_flush_works(void);
int st_lsm6dsm_allocate_triggers(struct lsm6dsm_data *cdata,
				const struct iio_trigger_ops *trigger_ops);

void st_lsm6dsm_deallocate_triggers(struct lsm6dsm_data *cdata);

#else /* CONFIG_IIO_TRIGGER */
static inline int st_lsm6dsm_allocate_triggers(struct lsm6dsm_data *cdata,
			const struct iio_trigger_ops *trigger_ops, int irq)
{
	return 0;
}
static inline void st_lsm6dsm_deallocate_triggers(struct lsm6dsm_data *cdata,
								int irq)
{
	return;
}
static inline void st_lsm6dsm_flush_works()
{
	return;
}
#endif /* CONFIG_IIO_TRIGGER */

#ifdef CONFIG_PM
int st_lsm6dsm_common_suspend(struct lsm6dsm_data *cdata);
int st_lsm6dsm_common_resume(struct lsm6dsm_data *cdata);
#endif /* CONFIG_PM */

#ifdef CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT
int st_lsm6dsm_i2c_master_probe(struct lsm6dsm_data *cdata);
int st_lsm6dsm_i2c_master_exit(struct lsm6dsm_data *cdata);
int st_lsm6dsm_enable_passthrough(struct lsm6dsm_data *cdata, bool enable);
int st_lsm6dsm_enable_accel_dependency(struct lsm6dsm_sensor_data *sdata,
								bool enable);
#else /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */
static inline int st_lsm6dsm_i2c_master_probe(struct lsm6dsm_data *cdata)
{
	return 0;
}
static inline int st_lsm6dsm_i2c_master_exit(struct lsm6dsm_data *cdata)
{
	return 0;
}
#endif /* CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT */

#endif /* ST_LSM6DSM_H */
