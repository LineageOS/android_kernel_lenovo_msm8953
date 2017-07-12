/*
 * STMicroelectronics lsm303agr driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.0.0
 * Licensed under the GPL-2.
 */

#ifndef __LSM303AGR_H
#define __LSM303AGR_H

#include <linux/types.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>

#define LSM303AGR_ACCEL_DEV_NAME		"lsm303agr_accel"
#define LSM303AGR_MAGN_DEV_NAME			"lsm303agr_magn"

#define LSM303AGR_EN_BIT			1
#define LSM303AGR_DIS_BIT			0
#define LSM303AGR_AXIS_COUNT			3
#define LSM303AGR_BYTE_X_AXIS			2
#define LSM303AGR_OUT_DATA_SIZE			(LSM303AGR_AXIS_COUNT * \
						LSM303AGR_BYTE_X_AXIS)
#define LSM303AGR_TX_MAX_LENGTH			12
#define LSM303AGR_RX_MAX_LENGTH			8193

#define LSM303AGR_ADD_CHANNEL(device_type, modif, index, mod, endian, sbits, rbits, addr, s) \
{ \
	.type = device_type, \
	.modified = modif, \
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
		.endianness = endian, \
	}, \
}

#define LSM303AGR_CORE_SAMPLE_FREQ_ATTR() \
				IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
				lsm303agr_core_get_sampling_frequency, \
				lsm303agr_core_set_sampling_frequency)

#define LSM303AGR_CORE_SAMPLE_FREQ_AVAIL_ATTR() \
				IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
				lsm303agr_core_get_sampling_frequency_avail)

#define LSM303AGR_CORE_SCALE_AVAIL_ATTR(x) \
				IIO_DEVICE_ATTR(x, S_IRUGO, \
				lsm303agr_core_get_scale_avail, \
				NULL, 0)

struct lsm303agr_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LSM303AGR_RX_MAX_LENGTH];
	u8 tx_buf[LSM303AGR_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lsm303agr_odr_table_t {
	u32 hz;
	u8 reg_val;
};

struct lsm303agr_odr_conf_t {
	u8 odr_addr;
	u8 odr_mask;
	u32 default_odr;
	u32 table_size;
	const struct lsm303agr_odr_table_t *odr_table;
};

struct lsm303agr_fs_table_t {
	u32 gain;
	u8 reg_val;
	int urv;
};

struct lsm303agr_fs_conf_t {
	u8 fs_addr;
	u8 fs_mask;
	u32 table_size;
	u32 default_fs;
	const struct lsm303agr_fs_table_t *fs_table;
};

struct lsm303agr_sensor_conf {
	const char *name;
	u8 wai_addr;
	u8 wai_val;
	u8 irq_addr;
	u8 irq_mask;
	const struct lsm303agr_odr_conf_t odr;
	const struct lsm303agr_fs_conf_t fs;
	const struct iio_chan_spec *chan_spec;
	u32 chan_spec_size;
	u32 turn_on_time_ms;
	u32 output_size;
};

struct lsm303agr_data;

struct lsm303agr_transfer_function {
	struct mutex tf_lock;
	int (*write)(struct lsm303agr_data *cdata, u8 reg_addr, int len, u8 *data);
	int (*read)(struct lsm303agr_data *cdata, u8 reg_addr, int len, u8 *data);
};

struct lsm303agr_sensor_data {
	u8 enabled;
	u8 sample_to_discard;
	u8 *buffer;
	u32 odr;
	u32 gain;
	struct lsm303agr_data *cdata;
	struct iio_trigger *iio_trig;
	struct iio_dev *iio_sensors_dev;
};

struct lsm303agr_ctm_func {
	int (*set_enable)(struct lsm303agr_sensor_data *sdata, bool state);
	int (*set_odr)(struct lsm303agr_sensor_data *sdata, uint32_t odr);
	int (*set_axis)(struct lsm303agr_sensor_data *sdata, u8 axis_enable);
	int (*set_fs)(struct lsm303agr_sensor_data *sdata, u32 fs);
	int (*init)(struct lsm303agr_data *cdata);
	int (*set_trigger)(struct lsm303agr_sensor_data *sdata, bool state);
};

enum lsm303agr_sensor_id {
	ACCEL_ID = 0,
	MAGN_ID
};

struct lsm303agr_data {
	u8 drdy_int_pin;
	int irq;
	int drdy_pin;
	void *priv_data;
	s64 timestamp;
	enum lsm303agr_sensor_id id;
	struct work_struct work_s;
	struct device *dev;
	struct lsm303agr_sensor_data *sdata;
	struct workqueue_struct *wq;
	struct lsm303agr_transfer_function *tf;
	struct lsm303agr_sensor_conf *sensor_conf;
	struct lsm303agr_ctm_func *func;
	struct lsm303agr_transfer_buffer tb;
};

int lsm303agr_acc_probe(struct lsm303agr_data *cdata, int irq);
int lsm303agr_acc_remove(struct lsm303agr_data *cdata, int irq);

int lsm303agr_mag_probe(struct lsm303agr_data *cdata, int irq);
int lsm303agr_mag_remove(struct lsm303agr_data *cdata, int irq);

int lsm303agr_common_probe(struct lsm303agr_data *cdata, int irq,
					const struct iio_info *lsm303agr_info);
#ifdef CONFIG_PM
int lsm303agr_common_suspend(struct lsm303agr_data *cdata);
int lsm303agr_common_resume(struct lsm303agr_data *cdata);
#endif
int lsm303agr_allocate_buffer(struct lsm303agr_sensor_data *sdata);
int lsm303agr_allocate_triggers(struct lsm303agr_sensor_data *sdata,
				const struct iio_trigger_ops *trigger_ops);
int lsm303agr_trig_set_state(struct iio_trigger *trig, bool state);
int lsm303agr_read_register(struct lsm303agr_data *cdata, u8 reg_addr, int data_len,
							u8 *data);
int lsm303agr_core_set_enable(struct lsm303agr_sensor_data *sdata, bool enable);
int lsm303agr_set_axis_enable(struct lsm303agr_sensor_data *sdata, u8 axis_enable);
int lsm303agr_core_set_fs(struct lsm303agr_sensor_data *sdata, unsigned int fs);
void lsm303agr_common_remove(struct lsm303agr_data *cdata, int irq);
void lsm303agr_flush_works(void);
void lsm303agr_deallocate_buffer(struct lsm303agr_sensor_data *sdata);
void lsm303agr_deallocate_triggers(struct lsm303agr_sensor_data *sdata);
int lsm303agr_write_register(struct lsm303agr_data *cdata, u8 reg_addr, u8 mask,
								u8 data);
int lsm303agr_read_register(struct lsm303agr_data *cdata, u8 reg_addr,
							int data_len, u8 *data);
int lsm303agr_core_write_odr(struct lsm303agr_sensor_data *sdata, uint32_t odr);
int lsm303agr_push_data(struct lsm303agr_sensor_data *sdata);

ssize_t lsm303agr_core_get_sampling_frequency(struct device *dev,
					      struct device_attribute *attr,
					      char *buf);
ssize_t lsm303agr_core_set_sampling_frequency(struct device * dev,
					      struct device_attribute * attr,
					      const char *buf, size_t count);
ssize_t lsm303agr_core_get_sampling_frequency_avail(struct device *dev,
						    struct device_attribute
						    *attr, char *buf);
ssize_t lsm303agr_core_get_scale_avail(struct device *dev,
				       struct device_attribute *attr, char *buf);
int lsm303agr_core_read_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *ch, int *val,
					int *val2, long mask);
int lsm303agr_core_write_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int val, int val2, long mask);
#endif /* __LSM303AGR_H */
