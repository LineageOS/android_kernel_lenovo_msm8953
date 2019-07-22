#ifndef _LINUX_ELAN_TS_H
#define _LINUX_ELAN_TS_H

/****************************customer info****************************/
#define LCM_X_MAX 1280
#define LCM_Y_MAX 800

/****************************elan data info****************************/

//i2c info
#define ELAN_TS_NAME "elan_ts"

#define ELAN_7BITS_ADDR 0x10
#define ELAN_8BITS_ADDR (ELAN_7BITS_ADDR<<1)

//sleep  mod 	
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK		BIT(3)

//cmd or paket head
#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54
#define HELLO_PKT			0x55
#define RAM_PKT				0xcc
#define BUFF_PKT			0x63

//elan IC series(only choose one)
//#define ELAN_2K_XX
#define ELAN_3K_XX
//#define ELAN_RAM_XX

/**********************fingers number macro switch**********************/
//#define TWO_FINGERS
//#define FIVE_FINGERS
#define TEN_FINGERS

//#define ELAN_BUFFER_MODE
//#define ELAN_ICS_SLOT_REPORT

#ifdef TWO_FINGERS
	#define FINGERS_PKT				0x5A
	#define PACKET_SIZE				8
	#define FINGERS_NUM				2
#endif

#ifdef FIVE_FINGERS
	#define FINGERS_PKT				0x5D
	#define PACKET_SIZE				18
	#define FINGERS_NUM				5
#endif

#ifdef TEN_FINGERS
	#define FINGERS_PKT				0x62
	#ifdef ELAN_BUFFER_MODE
	#define PACKET_SIZE				55
	#else
	#define PACKET_SIZE				35
	#endif
	#define FINGERS_NUM				10
#endif


/***********************debug info macro switch***********************/
//#define PRINT_INT_INFO
#define PRINT_INT_INFO1 

#ifdef PRINT_INT_INFO1 
	static bool debug_flage = true;
	#define elan_info(fmt, args...) do{\
		if(debug_flage)\
                printk("[elan debug]:"fmt"\n", ##args);\
	}while(0);
#else
	#define elan_info(fmt, args...)
#endif


/*************************have button macro switch*********************/
static const int key_value[] = {KEY_MENU, KEY_HOME, KEY_BACK};

//#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON
	static const int virtual_button[][2] = {
			{20,   80},
			{120, 180},
			{220, 280},
			{320, 380}
		};
#endif

#define TPD_HAVE_KEY
#ifdef TPD_HAVE_KEY

//#ifdef TWO_LAYER
	#define ELAN_KEY_NONE 0x01
	#define ELAN_KEY_BACK 0x21
	#define ELAN_KEY_HOME 0x41
	#define ELAN_KEY_MENU 0x81
//#endif

#ifdef ONE_LAYER
	#define FLAG_MENU	0x04
    #define FLAG_HOME	0x08
    #define FLAG_BACK	0x10
#endif

#endif

/*************************dev file macro switch********************/
#define ELAN_IAP_DEV

#ifdef ELAN_IAP_DEV
// For Firmware Update ****
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)
#define IOCTL_VIAROM	_IOR(ELAN_IOCTLID, 20, int) 
#define IOCTL_VIAROM_CHECKSUM	_IOW(ELAN_IOCTLID, 21, unsigned long)

#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

#endif


/**********************update firmware macro switch*******************/

#define IAP_PORTION

#if defined IAP_PORTION || defined ELAN_RAM_XX
	/*The newest firmware, if update must be changed here*/
	static uint8_t file_fw_data[] = {
		#include "fw_data_5517.i"
	};

	enum
	{
		PageSize = 132,
		ACK_Fail = 0x00,
		ACK_OK = 0xAA,
		ACK_REWRITE= 0x55,
		E_FD = -1,
	};
#endif


/**********************elan attr file macro switch*******************/
//#define SYS_ATTR_FILE
#ifdef SYS_ATTR_FILE
	static struct kobject *android_touch_kobj;
#endif

#endif /* _LINUX_ELAN_TS_H */

