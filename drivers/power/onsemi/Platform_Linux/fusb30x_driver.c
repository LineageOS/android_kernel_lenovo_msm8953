/* 
 * File:   fusb30x_driver.c
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */

#define DEBUG

/* Standard Linux includes */
#include <linux/init.h>				// __init, __initdata, etc
#include <linux/module.h>			// Needed to be a module
#include <linux/kernel.h>			// Needed to be a kernel module
#include <linux/i2c.h>				// I2C functionality
#include <linux/slab.h>				// devm_kzalloc
#include <linux/types.h>			// Kernel datatypes
#include <linux/errno.h>			// EINVAL, ERANGE, etc
#include <linux/of_device.h>			// Device tree functionality

/* Driver-specific includes */
#include "fusb30x_global.h"			// Driver-specific structures/types
#include "platform_helpers.h"			// I2C R/W, GPIO, misc, etc

#ifdef FSC_DEBUG
#include "../core/core.h"			// GetDeviceTypeCStatus
#endif // FSC_DEBUG

#include "fusb30x_driver.h"
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#define FUSB30X_VDD_VTG_MIN_UV		3300000
#define FUSB30X_VDD_VTG_MAX_UV		3300000
#define FUSB30X_I2C_VTG_MIN_UV		1800000
#define FUSB30X_I2C_VTG_MAX_UV		1800000

#if 0
struct fusb30x_data {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
};
#endif

extern void fusb_GPIO_Cleanup_usbid(void);

/******************************************************************************
* Driver functions
******************************************************************************/
static int __init fusb30x_init(void)
{
	pr_debug("FUSB  %s - Start driver initialization...\n", __func__);

	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
	pr_debug("FUSB  %s - Driver deleted...\n", __func__);
}

static int fusb30x_i2c_resume(struct device* dev)
{
	struct fusb30x_chip *chip;
		struct i2c_client *client = to_i2c_client(dev);

		if (client) {
			chip = i2c_get_clientdata(client);
		if (chip)
		up(&chip->suspend_lock);
		}
	return 0;
}

static int fusb30x_i2c_suspend(struct device* dev)
{
	struct fusb30x_chip* chip;
		struct i2c_client* client =  to_i2c_client(dev);

		if (client) {
			chip = i2c_get_clientdata(client);
		if (chip)
			down(&chip->suspend_lock);
		}
		return 0;
}

#if ADDED_BY_HQ_WWM
static int fusb30x_power_init(struct fusb30x_chip* chip, bool on)
{
	int rc;
	struct i2c_client* client = chip->client;

	if (!on)
		goto pwr_deinit;

	chip->vdd = regulator_get(&chip->client->dev, "vdd");
	if (IS_ERR(chip->vdd)) {
		rc = PTR_ERR(chip->vdd);
		dev_err(&chip->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	dev_info(&client->dev, "%s():vdd count = %d\n", __func__, regulator_count_voltages(chip->vdd));
	if (regulator_count_voltages(chip->vdd) > 0) {
		rc = regulator_set_voltage(chip->vdd, FUSB30X_VDD_VTG_MIN_UV,
			FUSB30X_VDD_VTG_MAX_UV);
		if (rc) {
			dev_err(&chip->client->dev,
		"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	chip->vcc_i2c = regulator_get(&chip->client->dev, "vcc_i2c");
	if (IS_ERR(chip->vcc_i2c)) {
		rc = PTR_ERR(chip->vcc_i2c);
		dev_err(&chip->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	dev_info(&client->dev, "%s():vcc_i2c count = %d\n", __func__, regulator_count_voltages(chip->vcc_i2c));
	if (regulator_count_voltages(chip->vcc_i2c) > 0) {
		rc = regulator_set_voltage(chip->vcc_i2c, FUSB30X_I2C_VTG_MIN_UV,
			FUSB30X_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&chip->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}


	dev_info(&client->dev, "%s():vdd power on.\n", __func__);
	rc = regulator_enable(chip->vdd);
	msleep(50);
	dev_info(&client->dev, "%s():vcc_i2c power on.\n", __func__);
	rc = regulator_enable(chip->vcc_i2c);
	msleep(50);

	return 0;

reg_vcc_i2c_put:
	regulator_put(chip->vcc_i2c);

reg_vdd_set_vtg:
	if (regulator_count_voltages(chip->vdd) > 0)
		regulator_set_voltage(chip->vdd, 0, FUSB30X_VDD_VTG_MAX_UV);

reg_vdd_put:
	regulator_put(chip->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(chip->vdd) > 0)
		regulator_set_voltage(chip->vdd, 0, FUSB30X_VDD_VTG_MAX_UV);

	dev_info(&client->dev, "%s():vdd power off.\n", __func__);
	rc = regulator_disable(chip->vdd);

	regulator_put(chip->vdd);

	if (regulator_count_voltages(chip->vcc_i2c) > 0)
		regulator_set_voltage(chip->vcc_i2c, 0, FUSB30X_I2C_VTG_MAX_UV);

	dev_info(&client->dev, "%s():vcc_i2c power off.\n", __func__);
	rc = regulator_disable(chip->vcc_i2c);

	regulator_put(chip->vcc_i2c);

	return 0;
}
#endif

#if 0
#define GTP_ADDR_LENGTH		2

FSC_BOOL fusb_i2c_read(struct i2c_client *client, FSC_U8 *buf, FSC_U8 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

	dev_info(&client->dev, "%s()\n", __func__);

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }

#if 0
    if((retries >= 5))
    {
    #if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif
    
    #if GTP_GESTURE_WAKEUP
        // reset chip would quit doze mode
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        { 
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 10);  
        }
    }
#endif
    return ret;
}

static FSC_BOOL fusb_i2c_test(struct i2c_client *client)
{
    FSC_U8 test[3] = {0x01};
    FSC_U8 retry = 0;
    FSC_S8 ret = -1;
  
    dev_info(&client->dev, "%s()\n", __func__);
  
    while(retry++ < 5)
    {
        ret = fusb_i2c_read(client, test, 1);
        if (ret > 0)
        {
            return ret;
        }
	dev_err(&client->dev, "FUSB i2c test failed time %d.\n", retry);
        msleep(1);
    }

    return ret;
}
#endif

bool have_fusb302 = 0;
static int fusb30x_probe (struct i2c_client* client,
		const struct i2c_device_id* id)
{
	int ret = 0;
	struct fusb30x_chip* chip; 
	struct i2c_adapter* adapter;

	if (!client)
	{
		pr_err("FUSB  %s - Error: Client structure is NULL!\n", __func__);
		return -EINVAL;
	}
	dev_info(&client->dev, "%s\n", __func__);

	/* Make sure probe was called on a compatible device */
	if (!of_match_device(fusb30x_dt_match, &client->dev))
	{
		dev_err(&client->dev, "FUSB  %s - Error: Device tree mismatch!\n", __func__);
		return -EINVAL;
	}
	pr_debug("FUSB  %s - Device tree matched!\n", __func__);

	/* Allocate space for our chip structure (devm_* is managed by the device) */
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to allocate memory for g_chip!\n", __func__);
		return -ENOMEM;
	}
	chip->client = client;			// Assign our client handle to our chip

#if ADDED_BY_HQ_WWM
	ret = fusb30x_power_init(chip, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto free_chip;
	}
#endif

	fusb30x_SetChip(chip);			// Set our global chip's address to the newly allocated memory
	pr_debug("FUSB  %s - Chip structure is set! Chip: %p ... g_chip: %p\n", __func__, chip, fusb30x_GetChip());

	/* Initialize the chip lock */
	mutex_init(&chip->lock);

	/* Initialize the chip's data members */
	fusb_InitChipData();
	pr_debug("FUSB  %s - Chip struct data initialized!\n", __func__);

	/* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
	adapter = to_i2c_adapter(client->dev.parent);
	if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
	{
		chip->use_i2c_blocks = true;
	}
	else
	{
		// If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
		// NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
		dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
		if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
		{
			dev_err(&client->dev, "FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
			dev_err(&client->dev, "FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
			return -EIO;
		}
	}
	pr_debug("FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

	/* Assign our struct as the client's driverdata */
	i2c_set_clientdata(client, chip);
	pr_debug("FUSB  %s - I2C client data set!\n", __func__);

	/* Verify that our device exists and that it's what we expect */
	if (!fusb_IsDeviceValid())
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to communicate with device!\n", __func__);
		pr_err("FUSB  %s - fusb_GPIO_Cleanup_usbid()\n", __func__);
		fusb_GPIO_Cleanup_usbid();
		return -EIO;
	}
	pr_debug("FUSB  %s - Device check passed!\n", __func__);

	/* reset fusb30x*/
	fusb_reset();

	/* Initialize semaphore*/
	sema_init(&chip->suspend_lock, 1);

	/* Initialize the platform's GPIO pins and IRQ */
	ret = fusb_InitializeGPIO();
	if (ret)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
		return ret;
	}
	pr_debug("FUSB  %s - GPIO initialized!\n", __func__);

#ifdef FSC_DEBUG
	/* Initialize debug sysfs file accessors */
	fusb_Sysfs_Init();
	pr_debug("FUSB  %s - Sysfs device file created!\n", __func__);
#endif // FSC_DEBUG

#ifdef FSC_INTERRUPT_TRIGGERED
	/* Enable interrupts after successful core/GPIO initialization */
	ret = fusb_EnableInterrupts();
	if (ret)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
		return -EIO;
	}

	/* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
	*  Interrupt must be enabled before starting 302 initialization */
	fusb_InitializeCore();
	pr_debug("FUSB  %s - Core is initialized!\n", __func__);
#else
	/* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now) */
	fusb_InitializeCore();
	pr_debug("FUSB  %s - Core is initialized!\n", __func__);

	/* Init our workers, but don't start them yet */
	fusb_InitializeWorkers();
	/* Start worker threads after successful initialization */
	fusb_ScheduleWork();
	pr_debug("FUSB  %s - Workers initialized and scheduled!\n", __func__);
#endif  // ifdef FSC_POLLING elif FSC_INTERRUPT_TRIGGERED

	dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);
	have_fusb302 = 1;
	return ret;

#if ADDED_BY_HQ_WWM
free_chip:
	kfree(chip);
	return ret;
#endif
}

static int fusb30x_remove(struct i2c_client* client)
{
	pr_debug("FUSB  %s - Removing fusb30x device!\n", __func__);

#ifndef FSC_INTERRUPT_TRIGGERED // Polling mode by default
	fusb_StopThreads();
#endif  // !FSC_INTERRUPT_TRIGGERED

	fusb_GPIO_Cleanup();
	pr_debug("FUSB  %s - FUSB30x device removed from driver...\n", __func__);
	return 0;
}

static void fusb30x_shutdown(struct i2c_client *client)
{
	fusb_reset();
		pr_debug("FUSB	%s - fusb30x shutdown\n", __func__);
}


/*******************************************************************************
 * Driver macros
 ******************************************************************************/
//#if ADDED_BY_HQ_WWM
//late_initcall(fusb30x_init);			// Defines the module's entrance function
//#else
module_init(fusb30x_init);			// Defines the module's entrance function
//#endif
module_exit(fusb30x_exit);			// Defines the module's exit function

MODULE_LICENSE("GPL");				// Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");	// Exposed on call to modinfo
MODULE_AUTHOR("Fairchild");			// Exposed on call to modinfo
