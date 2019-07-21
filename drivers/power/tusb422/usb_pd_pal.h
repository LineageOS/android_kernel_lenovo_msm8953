/*
 * Texas Instruments TUSB422 Power Delivery
 *
 * Author: Brian Quach <brian.quach@ti.com>
 *
 * Copyright: (C) 2016 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __USB_PD_PAL_H__
#define __USB_PD_PAL_H__

#include "tcpm.h"
#include "usb_pd_policy_engine.h"

//#include <linux/bitops.h>
//#include <linux/err.h>
#include <linux/gpio.h>
//#include <linux/interrupt.h>
//#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/usb/class-dual-role.h>

enum cc_line_state {
	CC_1,
	CC_2,
	OPEN,
};

struct typec_wakeup_source {
	struct wakeup_source	source;
	unsigned long		enabled;
};

struct qpnp_typec_chip {
	struct device		*dev;
	struct spmi_device	*spmi;
	struct power_supply	*batt_psy;
	struct power_supply	type_c_psy;
	struct regulator	*ss_mux_vreg;
	struct mutex		typec_lock;
	spinlock_t		rw_lock;

	u16			base;

	/* IRQs */
	int			vrd_changed;
	int			ufp_detach;
	int			ufp_detect;
	int			dfp_detach;
	int			dfp_detect;
	int			vbus_err;
	int			vconn_oc;

	/* Configurations */
	int			cc_line_state;
	int			current_ma;
	int			ssmux_gpio;
	enum of_gpio_flags	gpio_flag;
	int			typec_state;

	/* Dual role support */
	bool				role_reversal_supported;
	bool				in_force_mode;
	int				force_mode;
	struct dual_role_phy_instance	*dr_inst;
	struct dual_role_phy_desc	dr_desc;
	struct delayed_work		role_reversal_check;
	struct typec_wakeup_source	role_reversal_wakeup_source;
};

extern void external_gpio_direction_output(int value);
extern void tusb422_shedule_work(void);
extern struct qpnp_typec_chip *tusb_typec_chip;
extern  int set_property_on_battery(struct qpnp_typec_chip *chip,
				enum power_supply_property prop);
void usb_pd_pal_disable_vbus(unsigned int port);
void usb_pd_pal_source_vbus(unsigned int port, bool usb_pd, uint16_t mv, uint16_t ma);
void usb_pd_pal_sink_vbus(unsigned int port, bool usb_pd, uint16_t mv, uint16_t ma);
void usb_pd_pal_sink_vbus_batt(unsigned int port, uint16_t min_mv, uint16_t max_mv, uint16_t mw);
void usb_pd_pal_sink_vbus_vari(unsigned int port, uint16_t min_mv, uint16_t max_mv, uint16_t ma);

void usb_pd_pal_notify_connect_state(unsigned int port, tcpc_state_t state, bool polarity);
void usb_pd_pal_notify_pd_state(unsigned int port, usb_pd_pe_state_t state);

void usb_pd_pal_power_role_swap(unsigned int port, uint8_t new_role);
void usb_pd_pal_data_role_swap(unsigned int port, uint8_t new_role);

#endif
