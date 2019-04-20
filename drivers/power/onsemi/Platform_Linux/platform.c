#include <linux/printk.h>                                                       // pr_err, printk, etc
#include "fusb30x_global.h"                                                     // Chip structure
#include "platform_helpers.h"                                                   // Implementation details
#include "../core/platform.h"

#include "../core/PDProtocol.h"
#include "../core/PD_Types.h"

#include "../core/PDPolicy.h"
#include "../core/TypeC.h"
#include "../core/fusb30X.h"
#include "../core/vendor_info.h"

#include <linux/delay.h>

extern int fusb_enable_vbus(unsigned int on);
FSC_BOOL bVbusEnable = FALSE;

/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i, ret;

    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
		if(bVbusEnable != blnEnable){
			ret = fusb_enable_vbus(blnEnable);
			pr_info("FUSB %s: ret=%d, bVbusEnable=%d, blnEnable=%d\n", __func__, ret, bVbusEnable, blnEnable);
			if(0 == ret)
			bVbusEnable = blnEnable;
		}
        break;
    case VBUS_LVL_12V:
        // Enable/Disable the 12V Source
        //fusb_GPIO_Set_VBusOther(blnEnable == TRUE ? true : false);
        break;
    default:
        // Otherwise, do nothing.
        break;
    }

    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return fusb_GPIO_Get_VBus5v() ? TRUE : FALSE;

    case VBUS_LVL_12V:
        // Return the state of the 12V VBUS Source.
        return fusb_GPIO_Get_VBusOther() ? TRUE : FALSE;

    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    if (Data == NULL)
    {
        pr_err("%s - Error: Write data buffer is NULL!\n", __func__);
        ret = TRUE;
    }
    else if (fusb_I2C_WriteData((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = FALSE;
    }
    else  // I2C Write failure
    {
        ret = TRUE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    FSC_S32 i = 0;
    FSC_U8 temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        pr_err("FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return TRUE;
    }

    if (Data == NULL)
    {
        pr_err("%s - Error: Read data buffer is NULL!\n", __func__);
        ret = TRUE;
    }
    else if (DataLength > 1 && chip->use_i2c_blocks)    // Do block reads if able and necessary
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            ret = TRUE;
        }
        else
        {
            ret = FALSE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((FSC_U8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = FALSE;
            }
            else
            {
                ret = TRUE;
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{
	pr_info("FUSB %s:enable=%d, ConnState=%d, ProtocolState=%d\n", __func__, enable, ConnState, ProtocolState);
}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_Delay10us(delayCount);
}

/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/
int typeC_orientation = 0;
void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
    // Optional: Notify platform of CC orientation
    pr_err("dhx-----typec is %d\n",orientation);
    typeC_orientation = orientation;
}


/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
int is_pd_5v_insertion;
void platform_notify_pd_contract(FSC_BOOL contract)
{
#if ADDED_BY_HQ_WWM
    FSC_U8 pdstatus = 0;
    FSC_U8 aby_data[64] = {0};
    doDataObject_t contract_obj = {0};
    sopMainHeader_t received_caps_header = {0};
    doDataObject_t  received_caps_data[7] = {{0}}   ;
    FSC_U8 i = 0, j = 0;
    FSC_U8 intIndex = 5;
    FSC_U16 current_ma = 0;
    FSC_U16 voltage_mv = 0;
    pdstatus = GetUSBPDStatusOverview();

    pr_info("FUSB  %s : contract=%d, pdstatus=%d\n", __func__, contract, pdstatus);
    if (contract == 0)
        return;
    if (!(pdstatus&0x04)) {
        //PAD is sink
        //charging
        //Get voltage and Current
        GetUSBPDStatus(aby_data);

        for (i=0;i<4;i++){
            contract_obj.byte[i] = aby_data[intIndex++];
            pr_err("dhx---byte = %d\n",contract_obj.byte[i]);
        }
        received_caps_header.byte[0] = aby_data[intIndex++];
        received_caps_header.byte[1] = aby_data[intIndex++];
        for (i=0;i<7;i++)                                                       // Loop through each data object
        {
            for (j=0;j<4;j++)                                                   // Loop through each byte of the data object
                received_caps_data[i].byte[j] =  aby_data[intIndex++];
        } 
        current_ma = contract_obj.FVRDO.OpCurrent*10;
        voltage_mv = received_caps_data[contract_obj.FVRDO.ObjectPosition-1].FPDOSupply.Voltage*50;   //跟充电器协商好的电压， 单位为mV
        if (current_ma == 500)
            is_pd_5v_insertion = 2; 
        if (current_ma == 2000)
            is_pd_5v_insertion = 1;      
        pr_err("dhx---is_pd_5v_insertiong = %d\n",is_pd_5v_insertion);
        pr_err("FUSB  %s : dhx--fpdosupply--current_ma=%d, voltage_mv=%d\n", __func__, current_ma, voltage_mv);
        pr_err("dhx-----FPDOSink mv = %d\n",contract_obj.FPDOSink.Voltage*10);
        pr_err("dhx----bffo maxmv = %d minmv = %d maxpower = %d\n",contract_obj.BPDO.MaxVoltage ,contract_obj.BPDO.MinVoltage ,contract_obj.BPDO.MaxPower);
    }
#endif // ADDED_BY_HQ_WWM
} 

/*******************************************************************************
* Function:        platform_notify_unsupported_accessory
* Input:           None
* Return:          None
* Description:     A callback used by the core to report entry to the
*                  Unsupported Accessory state. The platform may implement
*                  USB Billboard.
*******************************************************************************/
void platform_notify_unsupported_accessory(void)
{
    // Optional: Implement USB Billboard
}

/*******************************************************************************
* Function:        platform_set_data_role
* Input:           PolicyIsDFP - Current data role
* Return:          None
* Description:     A callback used by the core to report the new data role after
*                  a data role swap.
*******************************************************************************/
extern struct dwc3_msm *fusb_dwc3_msm;
void platform_set_data_role(FSC_BOOL PolicyIsDFP)
{
    // Optional: Control Data Direction

	if (PolicyIsDFP)
	{
		//host
		int retry = 0;
		
		do{
			pr_info("FUSB  %s fusb_dwc3_msm=%p, retry=%d\n", __func__, fusb_dwc3_msm, retry);
			if (fusb_dwc3_msm == NULL) {
				msleep(1000);
				retry++;
			}
		} while(fusb_dwc3_msm == NULL && retry < 100);

		fusb_GPIO_Set_UsbId(0);
	}
	else
	{
		//device
		fusb_GPIO_Set_UsbId(1);
	}
}

/*******************************************************************************
* Function:        platform_notify_bist
* Input:           bistEnabled - TRUE when BIST enabled, FALSE when disabled
* Return:          None
* Description:     A callback that may be used to limit current sinking during
*                  BIST
*******************************************************************************/
void platform_notify_bist(FSC_BOOL bistEnabled)
{
    // Do something
}

void platform_set_timer(TIMER *timer, FSC_U16 timeout)
{
    timer->start_time = get_system_time();
    timer->timeout = timeout;
}

FSC_BOOL platform_check_timer(TIMER *timer)
{
    return (((FSC_U16)(get_system_time() - timer->start_time) > timer->timeout) ? TRUE: FALSE);
}

FSC_U16 platform_get_system_time()
{
    return get_system_time();
}
