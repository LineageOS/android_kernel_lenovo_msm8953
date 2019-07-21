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

#include "usb_pd_pal.h"
//#include "tcpm.h"
//#include "tusb422_common.h"
//#include "usb_pd_policy_engine.h"
//#include <huawei_platform/usb/hw_pd_dev.h>

#define TCP_VBUS_CTRL_PD_DETECT (1 << 7)

// TODO: add port number to device tree and use that for port.

void usb_pd_pal_source_vbus(unsigned int port, bool usb_pd, uint16_t mv, uint16_t ma)
{
	PRINT("%s: %u mV, %s\n", __func__, mv, (usb_pd) ? "USB_PD" : "TYPE-C");
	
	if(!usb_pd){
		external_gpio_direction_output(0);
	}
	return;
}

void usb_pd_pal_disable_vbus(unsigned int port)
{
	PRINT("%s\n", __func__);
	external_gpio_direction_output(1);
	return;
}

void usb_pd_pal_sink_vbus(unsigned int port, bool usb_pd, uint16_t mv, uint16_t ma)
{
	struct qpnp_typec_chip *chip = tusb_typec_chip;
	PRINT("%s: %u mV, %u mA %s\n", __func__, mv, ma, (usb_pd) ? "USB_PD" : "TYPE-C");

	if (usb_pd)
	{
		chip->current_ma = ma;
		set_property_on_battery(chip,POWER_SUPPLY_PROP_CURRENT_CAPABILITY);
	}
	return;
}

/* For battery supplies */
void usb_pd_pal_sink_vbus_batt(unsigned int port, uint16_t min_mv, uint16_t max_mv, uint16_t mw)
{
	PRINT("%s: %u - %u mV, %u mW\n", __func__, min_mv, max_mv, mw);

	return;
}

/* For variable supplies */
void usb_pd_pal_sink_vbus_vari(unsigned int port, uint16_t min_mv, uint16_t max_mv, uint16_t ma)
{
	PRINT("%s: %u - %u mV, %u mA\n", __func__, min_mv, max_mv, ma);

	return;
}

void usb_pd_pal_notify_pd_state(unsigned int port, usb_pd_pe_state_t state)
{
	PRINT("%s: %s\n", __func__, 
		  (state == PE_SRC_READY) ? "PE_SRC_READY" : 
		  (state == PE_SNK_READY) ? "PE_SNK_READY" : "?");

	switch (state)
	{
		case PE_SRC_READY:
			//pd_state.connected = PD_CONNECT_PE_READY_SRC;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_PD_STATE, (void *)&pd_state);
			break;

		case PE_SNK_READY:
			tusb422_shedule_work();
			break;

		default:
			break;
	}

	return;
}

void usb_pd_pal_notify_connect_state(unsigned int port, tcpc_state_t state, bool polarity)
{
	int rc;
	struct qpnp_typec_chip *chip = tusb_typec_chip;
	switch (state)
	{
		case TCPC_STATE_UNATTACHED_SRC:
		case TCPC_STATE_UNATTACHED_SNK:
			PRINT("%s: TYPEC_UNATTACHED, polarity = 0x%x\n", __func__, polarity);
			chip->cc_line_state = OPEN;
			chip->current_ma = 0;
			chip->typec_state = POWER_SUPPLY_TYPE_UNKNOWN;
			chip->type_c_psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
			rc = set_property_on_battery(chip, POWER_SUPPLY_PROP_TYPEC_MODE);
			if (rc)
				pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", rc);

			pr_info("CC_line state = %d current_ma = %d in_force_mode = %d\n",
					chip->cc_line_state, chip->current_ma,
					chip->in_force_mode);
			break;

		case TCPC_STATE_ATTACHED_SRC:
			PRINT("%s: TYPEC_ATTACHED_SRC, polarity = 0x%x\n", __func__, polarity);
			chip->typec_state = POWER_SUPPLY_TYPE_DFP;
			chip->type_c_psy.type = POWER_SUPPLY_TYPE_DFP;
			chip->current_ma = 0;
			rc = set_property_on_battery(chip,
					POWER_SUPPLY_PROP_TYPEC_MODE);
			if (rc)
				pr_err("failed to set TYPEC MODE on battery psy rc=%d\n",
						rc);

			break;

		case TCPC_STATE_ATTACHED_SNK:
			PRINT("%s: TYPEC_ATTACHED_SNK, polarity = 0x%x\n", __func__, polarity);
			chip->current_ma = 2000;
			chip->typec_state = POWER_SUPPLY_TYPE_UFP;
			chip->type_c_psy.type = POWER_SUPPLY_TYPE_UFP;
			rc = set_property_on_battery(chip, POWER_SUPPLY_PROP_TYPEC_MODE);
			if (rc)
				pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", rc);
			break;

		default:
			break;
	}

	return;
}

void usb_pd_pal_data_role_swap(unsigned int port, uint8_t new_role)
{
	//struct pd_dpm_swap_state swap_state;

	PRINT("%s: new_role = %x\n", __func__, new_role);

	//swap_state.new_role = new_role;

	//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DR_SWAP, (void *)&swap_state);

	return;
}

void usb_pd_pal_power_role_swap(unsigned int port, uint8_t new_role)
{
	//pd_dpm_handle_pe_event();

	return;
}






