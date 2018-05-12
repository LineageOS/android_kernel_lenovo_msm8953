/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef IQS263_H
#define IQS263_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

/*	Addresses of Registers on IQS263	*/
#define DEV_INF				0x00
#define SYSFLAGS			0x01
#define COORDINATE   		0x02
#define TOUCH_STAT			0x03
#define COUNTS				0x04
#define LTA 				0x05
#define DELTAS				0x06
#define MULTIPLIERS			0x07
#define COMPENSATION		0x08
#define PROXSETTING			0x09
#define THRESHOLD			0x0a
#define TIMING_TARGET		0x0b
#define GESTURE_TIMER		0x0c
#define ACTIVE_CHANNEL		0x0d

/**************************************
*			define platform data
*
**************************************/

struct iqs263_button_plat {
  /*! The Key to send to the input */
  int keycode;
  /*! Mask to look for on Touch Status */
  int mask;
  /*! Current state of button. */
  int state;
};

struct iqs263_button_drv {
	struct iqs263_button_plat *buttons;
	int buttonSize;
	struct input_dev *input;
};

typedef struct iqs263_button_drv iqs263_button_t;
typedef struct iqs263_button_drv *piqs263_button_t;

struct IQS263_platform_data {
	piqs263_button_t pbuttonInformation;
	int	irq_gpio;
	//int	mov_gpio;
	int (*get_is_nirq_low)(void);
	//int (*get_is_nmov_low)(void);
};

typedef struct IQS263_platform_data iqs263_platform_data_t;
typedef struct IQS263_platform_data *piqs263_platform_data_t;

/***************************************
*		define data struct/interrupt
*
***************************************/

/* CpsStat  */
#define IQS263_TCHCMPSTAT_TCHSTAT2_FLAG   0x08
#define IQS263_TCHCMPSTAT_TCHSTAT1_FLAG   0x04
#define IQS263_TCHCMPSTAT_TCHSTAT0_FLAG   0x02

#define KEY_CAPSENSOR_IQS263_CS0 0x278
#define KEY_CAPSENSOR_IQS263_CS1 0x279
#define KEY_CAPSENSOR_IQS263_CS2 0x280

static struct iqs263_button_plat piqs263Buttons[] = {
  {
    .keycode = KEY_CAPSENSOR_IQS263_CS0,
    .mask = IQS263_TCHCMPSTAT_TCHSTAT0_FLAG,
  },
  {
    .keycode = KEY_CAPSENSOR_IQS263_CS1,
    .mask = IQS263_TCHCMPSTAT_TCHSTAT1_FLAG,
  },
  {
    .keycode = KEY_CAPSENSOR_IQS263_CS2,
    .mask = IQS263_TCHCMPSTAT_TCHSTAT2_FLAG,
  },
};

static struct iqs263_button_drv IQS263ButtonInformation = {
	.buttons = piqs263Buttons,
	.buttonSize = ARRAY_SIZE(piqs263Buttons),
};

struct iqs263_chip {
	struct device *pdev;
	struct IQS263_platform_data *hw;

	struct regulator *vdd;
	struct regulator *vio;
	int power_enabled;

	int irq;

	struct mutex mutex;

	struct delayed_work dworker;
};

#endif
