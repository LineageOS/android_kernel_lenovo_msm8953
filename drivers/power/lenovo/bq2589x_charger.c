/*
 * bq2589x_charger.c - Charger driver for TI BQ25892/25890
 *
 */

//#define DEBUG

#ifndef __BQ2589X_CHARGER_C_
#define __BQ2589X_CHARGER_C_
#endif

#define EXT_CHARGER_POWER_SUPPLY

#ifdef CONFIG_DEBUG_FS
#undef CONFIG_DEBUG_FS
#endif

#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/sfi.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/version.h>
#include <linux/usb/otg.h>
#include <linux/rpmsg.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/reboot.h>
#include "bq2589x_charger.h"

#define DEV_NAME "bq2589x"

#define CHARGER_TASK_JIFFIES		(HZ * 15)/* 150sec */
#define CHARGER_HOST_JIFFIES		(HZ * 30) /* 60sec */
#define CHARGER_CHECK_JIFFIES		(HZ * 2)

/* Max no. of tries to i2c operation */
#define MAX_TRY		3

/* Max no. of tries to reset the bq2589xi WDT */
#define MAX_RESET_WDT_RETRY 8

#ifdef CONFIG_DEBUG_FS
#define bq2589x_MAX_MEM		12
#endif

#define TYPEC_PSY_NAME "typec"

struct bq2589x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq2589x_chip {
	struct i2c_client *client;
	struct bq2589x_platform_data *pdata;
	enum bq2589x_chip_type chip_type;
#ifdef EXT_CHARGER_POWER_SUPPLY
	struct power_supply charger_psy;
	int temp_debug_flag;
#endif
	struct delayed_work chrg_task_wrkr;
	struct delayed_work fault_work;
	struct delayed_work charger_check_work;
	struct work_struct otg_evt_work;
	struct mutex event_lock;
	struct bq2589x_otg_regulator	otg_vreg;
	/* Wake lock to prevent platform from going to S3 when charging */
	struct wake_lock wakelock;
	/* timeout Wake lock when charger status changed*/
	struct wake_lock irq_wk;
	struct regulator *vdd;
	int chgr_stat;
	int cc;
	int cv;
	int iusb_max;
	int ibat_normal;
	int vbat_normal;
	int vbat_cool;
	int vbat_warm;
	int ibat_warm;
	int ibat_cool;
	int warm_temp;
	int cool_temp;
	int iterm;
	int batt_temp;
	int board_temp;	
	int batt_voltage;
	int batt_status;
	int irq_gpio;
	int irq;
	int chg_en_gpio;
	int otg_en_gpio;
#ifdef LENOVO_OTG_USB_SHORT
	int otg_usb_short_gpio;
	bool otg_usb_short_state;
#endif
	char ic_name[10];
	bool bat_is_warm;
	bool bat_is_cool;
	bool is_charging_enabled;
	bool online;
	bool sfttmr_expired;
	bool sfi_tabl_present;
	bool is_first_start;
#ifdef CONFIG_OTG_SUPPORT
	bool boost_mode;
#endif
	bool is_factory_mode;
	bool is_factory_cable;
	bool	typec_dfp;
};

static struct power_supply *fg_psy = NULL;	
static struct power_supply *usb_psy = NULL;
static struct power_supply *typec_psy = NULL;

#ifdef CONFIG_DEBUG_FS
static struct dentry *bq2589x_dbgfs_root;
static char bq2589x_dbg_regs[bq2589x_MAX_MEM][4];
#endif

static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);

//extern int tlmm_set_config_pullup(unsigned gpio, unsigned enable);

static int bq2589x_get_chip_version(struct bq2589x_chip *chip);

static int retry_sleep_ms[] = {
	10, 20, 30, 40, 50
};

/*
 * Genenric register read/write interfaces to access registers in charger ic
 */
static int bq2589x_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;
	int retry_count = 0;

retry:
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0 && retry_count < MAX_TRY) {
		/* sleep for few ms before retrying */
		msleep(retry_sleep_ms[retry_count++]);
		goto retry;
	}
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int bq2589x_read_reg(struct i2c_client *client, u8 reg)
{
	s32 ret;
	int retry_count = 0;
	u8 val;

retry:
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0 && retry_count < MAX_TRY) {
		/* sleep for few ms before retrying */
		msleep(retry_sleep_ms[retry_count++]);
		goto retry;
	}
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		val = (u8) ret;
	}

	return val;
}

static int bq2589x_reg_read_mask(struct i2c_client *client, u8 reg, u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2589x_read_reg(client, reg);
	if (ret < 0)
		return ret;

	return (ret & (mask<<shift)) >> shift;
}

static int bq2589x_reg_write_mask(struct i2c_client *client, u8 reg, u8 val, u8 shift,  u8 mask)
{
	int ret;

	if (shift > 8)
	{
		pr_err("%s shift out range %d\n", __func__, shift);
		return -EINVAL;
	}

	ret = bq2589x_read_reg(client, reg);
	if (ret < 0)
	{
		pr_err("%s read err %d\n", __func__, ret);	
		return ret;
	}

	ret &= ~(mask<<shift);
	ret |= val << shift;

	if((reg==BQ2589X_REG_14)&&(shift!=BQ2589X_REG_14_REG_RST))
		ret &= 0x7f;

	return bq2589x_write_reg(client, reg, ret);
}

//#define DEBUG
/*
 * This function dumps the bq2589x registers
 */
static void bq2589x_dump_registers(struct bq2589x_chip *chip)
{
	int ret;

//	dev_info(&chip->client->dev, "%s\n", __func__);
#ifndef DEBUG
	pr_warn("%s :", __func__);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_00);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Input Src Ctrl reg read fail\n");
	pr_warn( "00 %x; ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_01);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Pwr On Cfg reg read fail\n");
	pr_warn( "01 %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_03);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Chrg Curr Ctrl reg read fail\n");
	pr_warn( "03 %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_04);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Pre-Chrg Term reg read fail\n");
	pr_warn( "04 %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_05);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Chrg Volt Ctrl reg read fail\n");
	pr_warn( "05 %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_06);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Thermal Regulation reg read fail\n");
	pr_warn( "06 %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_09);
	if (ret < 0)
		dev_warn(&chip->client->dev, "09 reg read fail\n");
	pr_warn( "09 %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0A);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	pr_warn( "0A %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0B);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	pr_warn( "0B %x;  ", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0C);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	pr_warn( "0C %x;  ", ret);

	//bq2589x_reg_write_mask(chip->client, BQ2589X_REG_02, 0x3, 6, 0x3);
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0D);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	pr_warn( "0D %x \n", ret);
/*
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0E);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	pr_warn( "0E %x \n", ret);
	
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_11);
	if (ret < 0)
		dev_warn(&chip->client->dev, "11 reg read fail\n");
	pr_warn( "11 %x \n", ret);

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_12);
	if (ret < 0)
		dev_warn(&chip->client->dev, "12 reg read fail\n");
	pr_warn( "12 %x \n", ret);
*/
#else
	int i;

	bq2589x_reg_write_mask(chip->client, BQ2589X_REG_02, 0x3, 6, 0x3);

	pr_warn("bq25892 REG dump");	
	for(i=0;i<0x14;i++)
	{
		ret = bq2589x_read_reg(chip->client, BQ2589X_REG_00+i);
		if (ret < 0)
			pr_warn("%s reg %x read fail\n", __func__, i);
		pr_warn("[%x]=%x ", i, ret);		
	}

	//ret = gpio_get_value(chip->chg_en_gpio);
	//dev_info(&chip->client->dev, "en io %d\n", ret);
#endif
}

/*
 * This function verifies if the bq2589xi charger chip is in Hi-Z
 * If yes, then clear the Hi-Z to resume the charger operations
 */
static int bq2589x_clear_hiz(struct bq2589x_chip *chip)
{
	int ret;

	pr_debug("%s\n", __func__);

	/*
	 * Read the bq2589xi REG00 register for charger Hi-Z mode.
	 * If it is in Hi-Z, then clear the Hi-Z to resume the charging
	 * operations.
	 */
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_00);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
				"Input src cntl read failed\n");
		goto i2c_error;
	}

	if (ret & (1<<BQ2589X_REG_00_EN_HIZ)) {
		dev_warn(&chip->client->dev, "Charger IC in Hi-Z mode\n");
		ret &= ~(1<<BQ2589X_REG_00_EN_HIZ);
		ret = bq2589x_write_reg(chip->client, BQ2589X_REG_00, ret);
		if (ret < 0) {
			dev_warn(&chip->client->dev, 	"Input src cntl write failed\n");
			goto i2c_error;
		}
		msleep(150);
	}

	return ret;
i2c_error:
	dev_err(&chip->client->dev, "%s\n", __func__);
	return ret;
}

static int bq2589x_reset_regs(struct bq2589x_chip *chip)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_14, 1, BQ2589X_REG_14_REG_RST, BQ2589X_REG_14_REG_RST_MASK);
	if (ret < 0) {
		dev_warn(&chip->client->dev,	"Input src cntl read failed\n");
		goto i2c_error;
	}

	//bq2589x_dump_registers(chip);
	return ret;
i2c_error:
	dev_err(&chip->client->dev, "%s\n", __func__);
	return ret;
}

static int fg_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!fg_psy)
		fg_psy = power_supply_get_by_name("battery");
	if (fg_psy) {
		ret = fg_psy->get_property(fg_psy, psp, &val);
		if (!ret)
			return val.intval;
	}
	return ret;
}


static int usb_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!usb_psy)
		usb_psy = power_supply_get_by_name("usb");

	if (usb_psy) {
		ret = usb_psy->get_property(usb_psy, psp, &val);
		if (!ret)
			return val.intval;
	}
	return ret;
}

static int usb_chip_set_property(enum power_supply_property psp, int value)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!usb_psy)
		usb_psy = power_supply_get_by_name("usb");

	if (usb_psy) {
		val.intval = value;
		ret = usb_psy->set_property(usb_psy, psp, &val);
		if (!ret)
			return val.intval;
	}else
		pr_err("%s usb psy not available\n", __func__);

	return ret;
}

static int typec_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!typec_psy)
		typec_psy = power_supply_get_by_name(TYPEC_PSY_NAME);

	if (typec_psy) {
		ret = typec_psy->get_property(typec_psy, psp, &val);
		if (!ret)
			return val.intval;
	}
	return ret;
}

static void typec_update_otg_status(struct bq2589x_chip *chip, int mode, bool force)
{
	pr_err("typec mode = %d\n", mode);

	if (mode == POWER_SUPPLY_TYPE_DFP) {
		chip->typec_dfp = true;
		power_supply_set_usb_otg(usb_psy, chip->typec_dfp);
		/* update FG */
		//set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS, get_prop_batt_status(chip));
	} else if (force || chip->typec_dfp) {
		chip->typec_dfp = false;
		power_supply_set_usb_otg(usb_psy, chip->typec_dfp);
		/* update FG */
		//set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS, get_prop_batt_status(chip));
	}
}

/*
static void typec_update_status(struct bq2589x_chip *chip)
{
	union power_supply_propval type = {0, };
	union power_supply_propval capability = {0, };
	int rc;

	get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
	if (type.intval != POWER_SUPPLY_TYPE_UNKNOWN) {
		get_property_from_typec(chip,
				POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
				&capability);
		chip->typec_current_ma = capability.intval;

		if (!chip->skip_usb_notification) {
			rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
				&capability);
			if (rc)
				pr_err("typec failed to set current max rc=%d\n",
					rc);
			pr_smb(PR_TYPEC, "SMB Type-C mode = %d, current=%d\n",
					type.intval, capability.intval);
		}
	} else {
		pr_smb(PR_TYPEC,
			"typec detection not completed continuing with USB update\n");
	}
}*/

#ifdef CONFIG_CHARGER_DUAL
static int charger_dual_chip_set_property(enum power_supply_property psp, int value)
{
	union power_supply_propval val;
	int ret = -ENODEV;
	static struct power_supply *charger_dual_psy = NULL;

	if (!charger_dual_psy)
		charger_dual_psy = power_supply_get_by_name("ext-charger-dual");;

	if (charger_dual_psy) {
		val.intval = value;
		ret = charger_dual_psy->set_property(charger_dual_psy, psp, &val);
		if (!ret)
			return val.intval;
	}else
		pr_err("%s usb psy not available\n", __func__);

	return ret;
}
#endif

static int chrg_ilim_to_reg(int ilim)
{
	int reg;

	if(ilim<=100)
		reg = 0;
	else if(ilim>=3250)
		reg = BQ2589X_REG_00_ILIM_MASK;
	else
		reg = (ilim - 100)/50;

	return reg;
}

static u8 chrg_iterm_to_reg(int iterm)
{
	u8 reg;

	if (iterm <= 64)
		reg = 0;
	else if(iterm>=1024)
		reg = BQ2589X_REG_05_ITERM_MASK;
	else
		reg = (iterm - 64) /64;

	return reg;
}

static u8 chrg_cur_to_reg(int cur)
{
	u8 reg;

	if (cur <= 0)
		reg = 0;
	else if(cur>=5056)
		reg = BQ2589X_REG_04_ICHG_MASK;
	else
		reg = cur /64;

	return reg;
}

static u8 chrg_volt_to_reg(int volt)
{
	u8 reg;

	if (volt <= 3840)
		reg = 0;
	else if(volt>=4608)
		reg = BQ2589X_REG_06_VREG_MASK;
	else
		reg = (volt - 3840) /16 + 1;

	return reg;
}

static u8 vindpm_to_reg(int vindpm)
{
	u8 reg;

	if (vindpm <= 3900)
		reg = 0;
	else if(vindpm>=7000)
		reg = BQ2589X_REG_01_VINDPM_MASK;
	else
		reg = (vindpm - 3900) /100;

	return reg;
}

static int program_timers(struct bq2589x_chip *chip, int wdt_duration,
				bool sfttmr_enable)
{
	int ret;

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_07);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "TIMER CTRL reg read failed\n");
		return ret;
	}

	ret |=  wdt_duration;
	if (sfttmr_enable)
		ret |= CHRG_TIMER_EXP_CNTL_EN_TIMER;
	else
		ret &= ~CHRG_TIMER_EXP_CNTL_EN_TIMER;

	ret = bq2589x_write_reg(chip->client, BQ2589X_REG_07, ret);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER CTRL I2C write failed\n");

	return ret;
}

/* This function should be called with the mutex held */
static int reset_wdt_timer(struct bq2589x_chip *chip)
{
	int ret = 0, i;

	for (i = 0; i < MAX_RESET_WDT_RETRY; i++) {
		ret = bq2589x_reg_write_mask(chip->client,
						BQ2589X_REG_03, 1,
						BQ2589X_REG_03_WD_RST, BQ2589X_REG_03_WD_RST_MASK);
		if (ret < 0)
			pr_warn("I2C write failed:%s\n", __func__);
		else
			break;
	}

	return ret;
}

/*
 *This function will modify the VINDPM as per the battery voltage
 */
static int bq2589x_modify_vindpm(struct bq2589x_chip *chip, u8 vindpm)
{
	int ret;
	static u8 vindpm_prev = 0xff;

	if(vindpm_prev==vindpm) {
		dev_warn(&chip->client->dev, "same vindpm val, ignored\n");
		return 0;
	}

	/* Get the input src ctrl values programmed */
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_01);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "INPUT CTRL reg read failed\n");
		return ret;
	}

	/* Assign the return value of REG00 to vindpm_prev */
	vindpm_prev = (ret & (BQ2589X_REG_01_VINDPM_MASK<<BQ2589X_REG_01_VINDPM));
	ret &= ~(BQ2589X_REG_01_VINDPM_MASK<<BQ2589X_REG_01_VINDPM);

	/*
	 * If both the previous and current values are same do not program
	 * the register.
	*/
	vindpm |= ret;
	ret = bq2589x_write_reg(chip->client, BQ2589X_REG_01, vindpm);
	if (ret < 0) {
		dev_info(&chip->client->dev, "VINDPM failed\n");
		return ret;
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
#define DBGFS_REG_BUF_LEN	3

static int bq2589x_show(struct seq_file *seq, void *unused)
{
	u16 val;
	long addr;

	if (kstrtol((char *)seq->private, 16, &addr))
		return -EINVAL;

	val = bq2589x_read_reg(bq2589x_client, addr);
	seq_printf(seq, "%x\n", val);

	return 0;
}

static int bq2589x_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq2589x_show, inode->i_private);
}

static ssize_t bq2589x_dbgfs_reg_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[DBGFS_REG_BUF_LEN];
	long addr;
	unsigned long value;
	int ret;
	struct seq_file *seq = file->private_data;

	if (!seq || kstrtol((char *)seq->private, 16, &addr))
		return -EINVAL;

//	if (copy_from_user(buf, user_buf, DBGFS_REG_BUF_LEN-1))
//		return -EFAULT;

	buf[DBGFS_REG_BUF_LEN-1] = '\0';
	if (kstrtoul(buf, 16, &value))
		return -EINVAL;

	dev_info(&bq2589x_client->dev, "[dbgfs write] Addr:0x%x Val:0x%x\n", (u32)addr, (u32)value);

	ret = bq2589x_write_reg(bq2589x_client, addr, value);
	if (ret < 0)
		dev_warn(&bq2589x_client->dev, "I2C write failed\n");

	return count;
}

static const struct file_operations bq2589x_dbgfs_fops = {
	.owner		= THIS_MODULE,
	.open		= bq2589x_dbgfs_open,
	.read		= seq_read,
	.write		= bq2589x_dbgfs_reg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int bq2589x_create_debugfs(struct bq2589x_chip *chip)
{
	int i;
	struct dentry *entry;

	bq2589x_dbgfs_root = debugfs_create_dir(DEV_NAME, NULL);
	if (IS_ERR(bq2589x_dbgfs_root)) {
		dev_warn(&chip->client->dev, "DEBUGFS DIR create failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < bq2589x_MAX_MEM; i++) {
		sprintf((char *)&bq2589x_dbg_regs[i], "%x", i);
		entry = debugfs_create_file(
					(const char *)&bq2589x_dbg_regs[i],
					S_IRUGO,
					bq2589x_dbgfs_root,
					&bq2589x_dbg_regs[i],
					&bq2589x_dbgfs_fops);
		if (IS_ERR(entry)) {
			debugfs_remove_recursive(bq2589x_dbgfs_root);
			bq2589x_dbgfs_root = NULL;
			dev_warn(&chip->client->dev, 	"DEBUGFS entry Create failed\n");
			return -ENOMEM;
		}
	}

	return 0;
}
static inline void bq2589x_remove_debugfs(struct bq2589x_chip *chip)
{
	if (bq2589x_dbgfs_root)
		debugfs_remove_recursive(bq2589x_dbgfs_root);
}
#else
static inline int bq2589x_create_debugfs(struct bq2589x_chip *chip)
{
	return 0;
}
static inline void bq2589x_remove_debugfs(struct bq2589x_chip *chip)
{
}
#endif

static inline int bq2589x_set_cv(struct bq2589x_chip *chip, int cv)
{
	u8 regval;

	dev_info(&chip->client->dev, "%s: %d\n", __func__,  cv);
	chip->cv = cv;
	regval = chrg_volt_to_reg(cv);

	return bq2589x_reg_write_mask(chip->client, BQ2589X_REG_06, regval, BQ2589X_REG_06_VREG, BQ2589X_REG_06_VREG_MASK);	
}

static int bq2589x_set_appropriate_cv(struct bq2589x_chip *chip)
{
	int cv = 0;
	if(chip->bat_is_cool)
		cv = chip->vbat_cool;
	else if(chip->bat_is_warm)
		cv = chip->vbat_warm;
	else
		cv = chip->vbat_normal;

	printk("charger : set cv %d\n", cv);

	return bq2589x_set_cv(chip,cv);
}

static inline int bq2589x_get_appropriate_cv(struct bq2589x_chip *chip)
{
	if(chip->bat_is_cool)
		return chip->vbat_cool;
	else if(chip->bat_is_warm)
		return chip->vbat_warm;
	else
		return chip->vbat_normal;
}

static void bq2589x_cv_check_and_set(struct bq2589x_chip *chip)
{
	int ret,cv,cv_reg;
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_06);
	if (ret < 0)
	{
		dev_warn(&chip->client->dev, "Chrg Volt Ctrl reg read fail\n");
		return;
	}

	ret = (ret>>2);
	cv = bq2589x_get_appropriate_cv(chip);
	cv_reg = chrg_volt_to_reg(cv);
	if(ret != cv_reg)
	{
		printk("Charger:cv value 0x%x get from charger chip is not correct :0x%x(%d)\n",ret,cv_reg, cv);
		bq2589x_set_appropriate_cv(chip);
	}
}

static inline int bq2589x_set_cc(struct bq2589x_chip *chip, int cc)
{
	u8 regval;

	//dev_info(&chip->client->dev, "%s: %d\n", __func__,  cc);
	regval = chrg_cur_to_reg(cc);
	printk("%s: %d 0x%x\n", __func__,  cc, regval);

	return bq2589x_reg_write_mask(chip->client, BQ2589X_REG_04,regval, BQ2589X_REG_04_ICHG, BQ2589X_REG_04_ICHG_MASK);
}

static inline int bq2589x_set_iusb(struct bq2589x_chip *chip, int iusb_max)
{
	int regval;

	chip->iusb_max = iusb_max;
	regval = chrg_ilim_to_reg(iusb_max);

	pr_info("fast_charger: %s:%d %x\n", __func__,  iusb_max,regval);
	if (regval < 0)
		return regval;

	return bq2589x_reg_write_mask(chip->client, BQ2589X_REG_00, regval, BQ2589X_REG_00_ILIM, BQ2589X_REG_00_ILIM_MASK);	
}

static void bq2589x_set_ibat_max(struct bq2589x_chip *chip, int ibat_max)
{
	static int pre_val = -1;

	pr_info("charger: %s, %d\n", __func__, ibat_max);
	if(pre_val!=ibat_max)
	{
		pre_val = ibat_max;
		chip->cc = ibat_max;
		if(chip->cc==0)
			chip->is_charging_enabled = false;
		else
			chip->is_charging_enabled = true;
		
		bq2589x_set_cc(chip, ibat_max);
	}else
		pr_info("%s ignore same val(%d)\n", __func__, pre_val);
}

static void bq2589x_set_iusb_max(struct bq2589x_chip *chip, int iusb_max)
{
	static int pre_val = -1;

	if(pre_val==iusb_max)
	{
		pr_info("%s ignore same val(%d)\n", __func__, iusb_max);
		return;
	}

	pr_info("charger: %s, %d\n", __func__, iusb_max);
	chip->iusb_max = iusb_max;
	pre_val = iusb_max;

	bq2589x_set_iusb(chip, chip->iusb_max);

//	bq2589x_dump_registers(chip);
}

static int bq2589x_set_appropriate_cc(struct bq2589x_chip *chip)
{
	int cc = 0;

	if(chip->bat_is_cool)
		cc = chip->ibat_cool;
	else if(chip->bat_is_warm)
		cc = chip->ibat_warm;
	else
		cc = chip->ibat_normal;

	printk("charger : set cc %d\n", cc);	
	bq2589x_set_ibat_max(chip, cc);

	return 0;
}

static int bq2589x_set_iterm(struct bq2589x_chip *chip, int iterm)
{
	u8 reg_val;

	if (iterm > bq2589x_CHRG_ITERM_OFFSET)
		dev_info(&chip->client->dev, "%s ITERM set for%d >128mA", __func__, iterm);

	reg_val = chrg_iterm_to_reg(iterm);
	msleep(500);

	return bq2589x_reg_write_mask(chip->client, BQ2589X_REG_05, reg_val, BQ2589X_REG_05_ITERM, BQ2589X_REG_05_ITERM_MASK);	
}

#if 0
bool is_charging_enabled(void)
{
	struct bq2589x_chip *chip = NULL;
	
	if(bq2589x_client != NULL)
		chip = i2c_get_clientdata(bq2589x_client);
	else
		return 1;
	return chip->is_charging_enabled;
}

bool is_in_otg_mode(void)
{
	struct bq2589x_chip *chip = NULL;
	
	if(bq2589x_client != NULL)
		chip = i2c_get_clientdata(bq2589x_client);
	else
		return 0;
	return chip->boost_mode;
}
#endif

static int bq2589x_enable_charging(struct bq2589x_chip *chip, bool val)
{
	int ret;

	dev_info(&chip->client->dev, "%s: %d\n", __func__, val);
	ret = program_timers(chip, CHRG_TIMER_EXP_CNTL_WDT160SEC, true);
	if (ret < 0) {
		dev_err(&chip->client->dev,
				"program_timers failed: %d\n", ret);
		return ret;
	}

#ifndef CONFIG_CHARGING_APP_CONTROL
	{
		int regval;
		/*
		 * Program the iusb_max here in case we are asked to resume the charging
		 * framework would send only set CC/CV commands and not the iusb_max. This
		 * would make sure that we program the last set iusb_max into the register
		 * in case for some reasons WDT expires
		 */
		regval = chrg_ilim_to_reg(chip->iusb_max);

		if (regval < 0) {
			dev_err(&chip->client->dev, "read ilim failed: %d\n", regval);
			return regval;
		}

		printk("%s ichg max is %d,reg is %x\n", __func__, chip->iusb_max,regval);
		ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_00, regval, BQ2589X_REG_00_ILIM, BQ2589X_REG_00_ILIM_MASK);
		if (ret < 0) {
			dev_err(&chip->client->dev, "iusb_max programming failed: %d\n", ret);
			return ret;
		}
	}
#endif

	/*
	 * check if we have the battery emulator connected. We do not start
	 * charging if the emulator is connected. Disable the charging
	 * explicitly.
	 */
	if (chip->sfi_tabl_present) {
		bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 0, BQ2589X_REG_03_CHG_CONFIG, BQ2589X_REG_03_CHG_CONFIG_MASK);
		pr_err("%s sfi_tabl_present is not true, disable charging\n", __func__);
		return ret;
	}

	if (chip->sfttmr_expired)
	{
		pr_info("%s Safety timer expired\n", __func__);		
		return ret;
	}

	if(val)
		ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 1, BQ2589X_REG_03_CHG_CONFIG, BQ2589X_REG_03_CHG_CONFIG_MASK);
	else
		ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 0, BQ2589X_REG_03_CHG_CONFIG, BQ2589X_REG_03_CHG_CONFIG_MASK);
		
	if (ret < 0)
		pr_warn("%s charger enable/disable failed\n", __func__);
	else {
		if (val)
			chip->online = true;
		else
			chip->online = false;
	}

	return ret;
}

static int bq2589x_enable_charger(struct bq2589x_chip *chip, int val)
{
	int ret = 0;

	/*stop charger, by putting it in HiZ mode*/
	if (val == 0) {
		ret =  bq2589x_reg_write_mask(chip->client, BQ2589X_REG_00, 1, BQ2589X_REG_00_EN_HIZ, BQ2589X_REG_00_EN_HIZ_MASK);
		if (ret < 0)
			dev_warn(&chip->client->dev, 	"Input src cntl write failed\n");
		else
			chip->is_charging_enabled = false;			
	}else
	{
		bq2589x_clear_hiz(chip);
		bq2589x_enable_charging(chip, true);
		chip->is_charging_enabled = true;	
	}

	pr_warn("%s: %d, %d\n", __func__,  val, chip->is_charging_enabled);
	return ret;
}

static int bq2589x_get_charging_status(struct bq2589x_chip *chip)
{
	int ret;

	if(chip->is_first_start) {
		pr_err("%s i2c did not init, return defalut val\n", __func__);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0B);
	ret &= SYSTEM_STAT_CHRG_MASK;
	switch (ret) {
		case SYSTEM_STAT_NOT_CHRG:
			chip->chgr_stat = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case SYSTEM_STAT_CHRG_DONE:
			chip->chgr_stat = POWER_SUPPLY_STATUS_FULL;
			break;
		case SYSTEM_STAT_PRE_CHRG:
		case SYSTEM_STAT_FAST_CHRG:
			chip->chgr_stat = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			chip->chgr_stat = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
	}

	return chip->chgr_stat;
}

static bool bq2589x_is_charging_done(struct bq2589x_chip *chip)
{
	int batt_voltage = 0;

	batt_voltage = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	if (batt_voltage < 0) {
		dev_err(&chip->client->dev, "Can't read voltage from FG\n");
		return false;
	}
	chip->batt_voltage = batt_voltage;

	if((POWER_SUPPLY_STATUS_FULL== bq2589x_get_charging_status(chip)) && (chip->batt_voltage > 4250))
		return true;

	if(chip->bat_is_warm && chip->batt_voltage > 4050)
		return true;

	return false;
}

/* IRQ handler for charger Interrupts configured to GPIO pin */
static irqreturn_t bq2589x_irq_isr(int irq, void *devid)
{
//	struct bq2589x_chip *chip = (struct bq2589x_chip *)devid;

	/**TODO: This hanlder will be used for charger Interrupts */
//	dev_dbg(&chip->client->dev,
//		"IRQ Handled for charger interrupt: %d\n", irq);
	return IRQ_WAKE_THREAD;
}

/* IRQ handler for charger Interrupts configured to GPIO pin */
static irqreturn_t bq2589x_irq_thread(int irq, void *devid)
{
	struct bq2589x_chip *chip = (struct bq2589x_chip *)devid;
	int reg_status;
	static int usb_state = -1;

	msleep(100);

	wake_lock_timeout(&chip->irq_wk,HZ);
	/*
	 * check the bq2589x status/fault registers to see what is the
	 * source of the interrupt
	 */
	reg_status = bq2589x_read_reg(chip->client, BQ2589X_REG_0B);
	if (reg_status < 0)
		dev_err(&chip->client->dev, "STATUS register read failed:\n");

	pr_err("ww_debug %s reg0x%x\n", __func__, reg_status);

	if(((reg_status & SYSTEM_STAT_VBUS_HOST) == SYSTEM_STAT_VBUS_HOST) ||
		((reg_status & SYSTEM_STAT_VBUS_CDP) == SYSTEM_STAT_VBUS_CDP))
	{
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);

		if (1)/*(usb_state!=1)*/ {
			pr_info("%s, set usb present!\n", __func__);
			usb_state = 1;
			usb_chip_set_property(POWER_SUPPLY_PROP_DP_DM, POWER_SUPPLY_DP_DM_DPF_DMF);
			usb_chip_set_property(POWER_SUPPLY_PROP_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
			usb_chip_set_property(POWER_SUPPLY_PROP_TYPE, POWER_SUPPLY_TYPE_USB);
			usb_chip_set_property(POWER_SUPPLY_PROP_PRESENT, 1);
			if(chip->is_factory_mode) {
				pr_info("CHG is factory mode, schedule charger det work\n");
				schedule_delayed_work(&chip->charger_check_work, CHARGER_CHECK_JIFFIES);
			}
		}
	}else{
		msleep(150);
		/* Release the wake lock */
		if (wake_lock_active(&chip->wakelock))
			wake_unlock(&chip->wakelock);

		if (usb_state!=0) {
			pr_info("%s, set usb not present!\n", __func__);
			usb_state = 0;
			usb_chip_set_property(POWER_SUPPLY_PROP_TYPE, POWER_SUPPLY_TYPE_UNKNOWN);
			usb_chip_set_property(POWER_SUPPLY_PROP_PRESENT, 0);
			//usb_chip_set_property(POWER_SUPPLY_PROP_DP_DM, POWER_SUPPLY_DP_DM_DPR_DMR);
#ifndef CONFIG_CHARGING_APP_CONTROL
			bq2589x_set_iusb_max(chip, 500);
#endif

			if((chip->is_factory_cable)&&(chip->is_factory_mode)) {
				schedule_delayed_work(&chip->charger_check_work, CHARGER_CHECK_JIFFIES);
			}
		}
	}

	reg_status &= SYSTEM_STAT_CHRG_DONE;
	if (reg_status == SYSTEM_STAT_CHRG_DONE) {
/*		mutex_lock(&chip->event_lock);
		bq2589x_enable_hw_term(chip, false);
		mutex_unlock(&chip->event_lock);
		bq2589x_hw_init(chip);
		*/
		/* schedule the thread to let the framework know about FULL */
	}
	
//	if(fg_psy != NULL)
//		power_supply_changed(fg_psy);

	schedule_delayed_work(&chip->fault_work, 0);

	return IRQ_HANDLED;
}

static void bq2589x_hw_init(struct bq2589x_chip *chip)
{
	int ret = 0;

	ret = program_timers(chip, CHRG_TIMER_EXP_CNTL_WDT160SEC,true);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER enable failed\n");

	bq2589x_set_appropriate_cv(chip);

	if(chip->iterm)
		bq2589x_set_iterm(chip,chip->iterm);

	bq2589x_reg_write_mask(chip->client, BQ2589X_REG_02, 0, BQ2589X_REG_02_ICO_EN, BQ2589X_REG_02_ICO_EN_MASK);
	bq2589x_reg_write_mask(chip->client, BQ2589X_REG_00, 0, BQ2589X_REG_00_EN_ILIM, BQ2589X_REG_00_EN_ILIM_MASK);
	bq2589x_write_reg(chip->client,BQ2589X_REG_0D,0x93);

	//bq2589x_set_iusb_max(chip, 500);
	if(chip->is_factory_mode) {
		bq2589x_set_iusb_max(chip, 3000);
		bq2589x_set_ibat_max(chip, 0);		
	 } else {
		bq2589x_set_iusb_max(chip, 500);
		bq2589x_set_cc(chip, 500);
	}

	bq2589x_enable_charging(chip, chip->is_charging_enabled);
}

static void bq2589x_fault_worker(struct work_struct *work)
{
	struct bq2589x_chip *chip = container_of(work, struct bq2589x_chip, fault_work.work);
	int reg_fault;

	/* Check if battery fault condition occured. Reading the register
	   value two times to get reliable reg value, recommended by vendor*/
	reg_fault = bq2589x_read_reg(chip->client, BQ2589X_REG_0C);
	if (reg_fault < 0)
		dev_err(&chip->client->dev, "FAULT register read failed:\n");

	if(reg_fault!=0x00)
		dev_info(&chip->client->dev, "FAULT reg %x\n", reg_fault);
	
	if (reg_fault & FAULT_STAT_WDT_TMR_EXP) {
		dev_warn(&chip->client->dev, "WDT expiration fault\n");
		if (chip->is_charging_enabled) {
			/*when WDT expiration ,bq2589x register will be reset,so we need reconfig it*/
			bq2589x_hw_init(chip);
		} else
			dev_info(&chip->client->dev, "No charger connected\n");
	}

	if ((reg_fault & FAULT_STAT_CHRG_TMR_FLT) == FAULT_STAT_CHRG_TMR_FLT) {
		chip->sfttmr_expired = true;
		pr_info("%s Safety timer expired\n", __func__);
	}
}

static void bq2589x_temp_monitor(struct bq2589x_chip *chip) 
{
	bool bat_cool = false;
	bool bat_warm = false;

	if(chip->batt_temp  < chip->cool_temp){
		bat_cool = true;
		bat_warm = false;
	}else if(chip->batt_temp > chip->cool_temp && chip->batt_temp < chip->warm_temp){
		bat_cool = false;
		bat_warm = false;
	}else if(chip->batt_temp > chip->warm_temp){
		bat_warm = true;
		bat_cool = false;
	}

	if (chip->bat_is_cool ^ bat_cool || chip->bat_is_warm ^ bat_warm) {
		chip->bat_is_cool = bat_cool;
		chip->bat_is_warm = bat_warm;
#ifndef CONFIG_CHARGING_APP_CONTROL   
		bq2589x_set_appropriate_cc(chip);
#endif
		bq2589x_set_appropriate_cv(chip);
	}
}

static void bq2589x_task_worker(struct work_struct *work)
{
	struct bq2589x_chip *chip =
	    container_of(work, struct bq2589x_chip, chrg_task_wrkr.work);
	int ret, jiffy = CHARGER_TASK_JIFFIES;
	int soc = fg_chip_get_property(POWER_SUPPLY_PROP_CAPACITY);
	int cur;
	int vbus;

	if(fg_psy != NULL)
		power_supply_changed(fg_psy);

	mutex_lock(&chip->event_lock);
	ret = reset_wdt_timer(chip);
	mutex_unlock(&chip->event_lock);
	if (ret < 0)
		pr_err("%s WDT reset failed:\n", __func__);

	if (chip->is_first_start) {
		int type;

		//check vbus status after prob
		chip->is_first_start = false;
		if (typec_psy) {
			pr_err("ww_debug typec operation\n");
			type = typec_chip_get_property(POWER_SUPPLY_PROP_TYPE);
			typec_update_otg_status(chip, type, true);
		}
		bq2589x_irq_thread(0, chip);
	}

	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_0C);
	if (ret < 0)
		pr_err("%s FAULT register read failed:\n", __func__);
	else 
	{
		if(ret!=0x00)
		{
			pr_info("%s FAULT reg %x\n", __func__, ret);
		
			if (ret & FAULT_STAT_WDT_TMR_EXP) {
				pr_warn("%s WDT expiration fault\n", __func__);
				if (chip->is_charging_enabled) {
					/*when WDT expiration ,bq2589x register will be reset,so we need reconfig it*/
					bq2589x_hw_init(chip);
				} else
					pr_info("%s No charger connected\n", __func__);
			}
		}
	}
	
	/*
	 * If we have an OTG device connected, no need to modify the VINDPM
	 * check for Hi-Z
	 */
	if ((chip->boost_mode) ||(chip->is_factory_mode)) {
		jiffy = CHARGER_HOST_JIFFIES;
		goto sched_task_work;
	}

	/*sometimes WDT interrupt won't assert, so charger parameters will be reseted to 
	 * default value. we do check here, if the CV value is not correct, we reinit the
	 * charger parameters*/
	bq2589x_cv_check_and_set(chip);
	/* Modify the VINDPM */

sched_task_work:
	vbus = fg_chip_get_property(POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION);
	cur = fg_chip_get_property(POWER_SUPPLY_PROP_CURRENT_NOW);
	chip->batt_status = bq2589x_get_charging_status(chip);//fg_chip_get_property(POWER_SUPPLY_PROP_STATUS);
	chip->batt_temp = fg_chip_get_property(POWER_SUPPLY_PROP_TEMP);
	chip->batt_voltage  = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000;
	if (chip->batt_voltage < 0) {
		dev_err(&chip->client->dev, "Can't read voltage from FG\n");
	}

	/* convert voltage into millivolts */
	if (POWER_SUPPLY_STATUS_FULL== chip->batt_status) {
		if(soc != 100)
		{
			dev_warn(&chip->client->dev, 	"%s battery full,but soc is not 100", __func__);
		}
	}

	bq2589x_write_reg(chip->client,BQ2589X_REG_0D, 0x93);//vindpm 4.5V

	bq2589x_temp_monitor(chip);

	ret  = usb_chip_get_property(POWER_SUPPLY_PROP_TYPE);
	pr_warn("bq2589x battery voltage is %d, capacity is %d, temp is %d, status is %d, cur %d, vbus %d type %d\n",
		chip->batt_voltage, soc,chip->batt_temp,chip->batt_status, cur, vbus, ret);

	bq2589x_dump_registers(chip);

	if(soc < 5 || chip->batt_voltage < 3400)
		jiffy = jiffy /3;
	
	schedule_delayed_work(&chip->chrg_task_wrkr, jiffy);
}

static void bq2589x_charger_check_worker(struct work_struct *work)
{
	struct bq2589x_chip *chip = container_of(work, struct bq2589x_chip, charger_check_work.work);
	enum power_supply_type charger_type;

	if (!chip->is_factory_cable) {
		charger_type = usb_chip_get_property(POWER_SUPPLY_PROP_TYPE);
		if((charger_type==POWER_SUPPLY_TYPE_USB)&&(chip->is_factory_mode))
			chip->is_factory_cable = true;
		else
			chip->is_factory_cable = false;

		pr_info("CHG %s, charger type %d is_factorymode %d is_cable %d\n", __func__, 
			charger_type, chip->is_factory_mode, chip->is_factory_cable);
	} else {
		//if (!factory_kill_disable && !reboot_in_progress()) {
		if (!factory_kill_disable ) {
			pr_err("CHG - Factory Cable removed, power-off\n");
			kernel_power_off();
		} else
			pr_err("CHG - Factory cable removed - kill disabled\n");

		chip->is_factory_cable = false;
	}
}

static int bq2589x_get_chip_version(struct bq2589x_chip *chip)
{
	int ret;

	/* check chip model number */
	ret = bq2589x_read_reg(chip->client, BQ2589X_REG_14);
	if (ret < 0) {
		dev_err(&chip->client->dev, "i2c read err:%d\n", ret);
		return -EIO;
	}
	dev_info(&chip->client->dev, "version reg: 0x%x\n", ret);

	ret = (ret&0x38) >> 3;
	switch (ret) {
	case BQ25852_IC_VERSION:
		chip->chip_type = BQ25852;
		memcpy(chip->ic_name, "BQ25852", sizeof("BQ25852"));
		break;
	case BQ25890_IC_VERSION:
		chip->chip_type = BQ25890;
		memcpy(chip->ic_name, "BQ25890", sizeof("BQ25890"));
		break;
	case bq25895_IC_VERSION:
		chip->chip_type = BQ25895;
		memcpy(chip->ic_name, "BQ25895", sizeof("BQ25895"));
		break;
	default:
		dev_err(&chip->client->dev, "device version mismatch: 0x%x set default\n", ret);
		chip->chip_type = BQ25852;
		memcpy(chip->ic_name, "BQ25852", sizeof("BQ25852"));
		break;
	}

	dev_info(&chip->client->dev, "chip type:%x\n", chip->chip_type);
	return 0;
}

static ssize_t  charger_ver_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bq2589x_chip *chip = i2c_get_clientdata(client);

	ret = sprintf(buf, "%s\n",chip->ic_name);
    
    return ret;
}

static DEVICE_ATTR(chrg_version, S_IRUGO|S_IWUSR, charger_ver_get,NULL);

#ifdef LENOVO_OTG_USB_SHORT
int bq2589x_turn_otg_vbus(bool votg_on);

int bq2589x_otg_short_config(struct bq2589x_chip *chip, int en)
{
	int value;

	if(en==1)
	{
		if(gpio_is_valid(chip->otg_usb_short_gpio)) {
			//ret = gpio_request(chip->otg_usb_short_gpio, "otg_usb_short_gpio");
			//bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 0, BQ2589X_REG_03_OTG_CONFIG, BQ2589X_REG_03_OTG_CONFIG_MASK);
			bq2589x_turn_otg_vbus(0);
			usleep(1000*1000);
			gpio_set_value(chip->otg_usb_short_gpio,1);
			usleep(1000*1000);
			//bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 1, BQ2589X_REG_03_OTG_CONFIG, BQ2589X_REG_03_OTG_CONFIG_MASK);
			bq2589x_turn_otg_vbus(1);
			
			chip->otg_usb_short_state = true;
		}else
			chip->otg_usb_short_state = false;		
	}else
	{
		if(gpio_is_valid(chip->otg_usb_short_gpio)) {
			//ret = gpio_request(chip->otg_usb_short_gpio, "otg_usb_short_gpio");
			bq2589x_turn_otg_vbus(0);
			gpio_set_value(chip->otg_usb_short_gpio,0);
			usleep(2000*1000);
			bq2589x_turn_otg_vbus(1);
			
			chip->otg_usb_short_state = false;
		}else
			chip->otg_usb_short_state = false;		
	}	

	value = gpio_get_value(chip->otg_usb_short_gpio);
		
	return value;
}

static ssize_t otg_usb_short_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;
	int value = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bq2589x_chip *chip = i2c_get_clientdata(client);

	sscanf(buf, "%x", &cmd);

	if(chip->otg_usb_short_state==cmd)
	{
		pr_err("%s new cmd is as same as the old value(%d)\n", __func__, chip->otg_usb_short_state);
		return ret;
	}

	value = bq2589x_otg_short_config(chip, cmd);

	printk("otg usb short state  is %d, cmd %d, ret %d\n",chip->otg_usb_short_state, cmd, value);
	
	return ret;
}


ssize_t  otg_usb_short_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
   	 int ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bq2589x_chip *chip = i2c_get_clientdata(client);

	printk("otg usb short state  is %d\n",chip->otg_usb_short_state);
	ret = sprintf(buf, "%d\n", chip->otg_usb_short_state);

    return ret;
}

static DEVICE_ATTR(otg_usb_short, S_IRUGO|S_IWUSR,otg_usb_short_get,otg_usb_short_set);
#endif

static struct attribute *fs_attrs[] = {
	&dev_attr_chrg_version.attr,
#ifdef LENOVO_OTG_USB_SHORT
	&dev_attr_otg_usb_short.attr,
#endif
	NULL,
};

static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static int bq2589x_parse_dt(struct device *dev,struct bq2589x_chip *chip)
{
	struct device_node *np = dev->of_node;
	
	chip->irq_gpio = of_get_named_gpio_flags(np, "charger,irq-gpio", 0, 0);
	chip->chg_en_gpio = of_get_named_gpio_flags(np, "charger,en-gpio", 0, 0);
	chip->otg_en_gpio = of_get_named_gpio_flags(np, "charger,otg-en-gpio", 0, 0);
#ifdef LENOVO_OTG_USB_SHORT
	chip->otg_usb_short_gpio = of_get_named_gpio_flags(np,
			"charger,otg-short-gpio", 0, 0);	
#endif

	of_property_read_u32(np,"charger,ichg-max" ,&chip->iusb_max);	
	of_property_read_u32(np,"charger,iterm-ma" ,&chip->iterm);	
	of_property_read_u32(np,"charger,ibat-max" ,&chip->ibat_normal);	
	of_property_read_u32(np,"charger,vbat-mv" ,&chip->vbat_normal);	
	of_property_read_u32(np,"charger,warm-temp" ,&chip->warm_temp);	
	of_property_read_u32(np,"charger,ibat-warm" ,&chip->ibat_warm);	
	of_property_read_u32(np,"charger,ibat-cool" ,&chip->ibat_cool);	
	of_property_read_u32(np,"charger,cool-temp" ,&chip->cool_temp);	
	of_property_read_u32(np,"charger,vbat-warm" ,&chip->vbat_warm);	
	of_property_read_u32(np,"charger,vbat-cool" ,&chip->vbat_cool);
#ifdef CONFIG_FAKE_BATTERY
	chip->sfi_tabl_present = 1;
#else
	chip->sfi_tabl_present = 0;
#endif
	chip->is_charging_enabled = true;
	chip->bat_is_warm = false;
	chip->bat_is_cool = false;
	chip->is_first_start = true;

	return 0;
}

#ifdef CONFIG_OTG_SUPPORT
static int bq2589x_turn_otg_vbus(struct bq2589x_chip* chip, bool votg_on)
{
	int ret = 0;

	pr_info("turn on otg vbus %d\n", votg_on);

	if (votg_on) {
			/* Program the timers */
			ret = program_timers(chip,
						CHRG_TIMER_EXP_CNTL_WDT80SEC,
						false);
			if (ret < 0) {
				dev_warn(&chip->client->dev, 	"TIMER enable failed %s\n", __func__);
				goto i2c_write_fail;
			}

#ifdef CONFIG_CHARGER_DUAL
			charger_dual_chip_set_property(POWER_SUPPLY_PROP_CHARGE_ENABLED, 0);
#endif
			
			ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_0A, 0x6, BQ2589X_REG_0A_BOOST_LIM, BQ2589X_REG_0A_BOOST_LIM_MASK);
			if (ret < 0) {
				dev_warn(&chip->client->dev, 	"otg boost lim write 1.3A failed\n");
				goto i2c_write_fail;
			}

			ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_0A, 0xc, BQ2589X_REG_0A_BOOSTV, BQ2589X_REG_0A_BOOSTV_MASK);
			if (ret < 0) {
				dev_warn(&chip->client->dev, 	"otg boost lim write 1.3A failed\n");
				goto i2c_write_fail;
			}		
			
			ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 2, BQ2589X_REG_03_CHG_CONFIG, 0x3);
			if (ret < 0) {
				dev_warn(&chip->client->dev, 	"otg set config failed\n");
				goto i2c_write_fail;
			}			
			
			chip->boost_mode = true;
	} else {
			ret = bq2589x_reg_write_mask(chip->client, BQ2589X_REG_03, 0, BQ2589X_REG_03_OTG_CONFIG, BQ2589X_REG_03_OTG_CONFIG_MASK);
			if (ret < 0) {
				dev_warn(&chip->client->dev, 	"otg set config failed\n");
				goto i2c_write_fail;
			}

			chip->boost_mode = false;

#ifdef CONFIG_CHARGER_DUAL
			charger_dual_chip_set_property(POWER_SUPPLY_PROP_CHARGE_ENABLED, 1);
#endif
			
			//restart charger when come back from otg mode
			if( chip->is_charging_enabled )
				bq2589x_enable_charging(chip, true);
	}

	return ret;
i2c_write_fail:
	dev_err(&chip->client->dev, "%s: Failed\n", __func__);
	return ret;
}

static int bq2589x_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct bq2589x_chip *chip = rdev_get_drvdata(rdev);
	
	return bq2589x_turn_otg_vbus(chip, true);
}

static int bq2589x_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct bq2589x_chip *chip = rdev_get_drvdata(rdev);
	
#ifdef LENOVO_OTG_USB_SHORT
	{
		int val;

		val = bq2589x_otg_short_config(chip, 0);
		pr_info("bq2589x_otg_short_config = %d", val);
	}
#endif

	return bq2589x_turn_otg_vbus(chip, false);
}

static int bq2589x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq2589x_chip *chip = rdev_get_drvdata(rdev);
	return chip->boost_mode;
}

struct regulator_ops bq2589x_otg_reg_ops = {
	.enable		= bq2589x_otg_regulator_enable,
	.disable	= bq2589x_otg_regulator_disable,
	.is_enabled	= bq2589x_otg_regulator_is_enable,
};

static int bq2589x_regulator_init(struct device *dev,struct bq2589x_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(dev, dev->of_node);
	if (!init_data) {
		dev_err(dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &bq2589x_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(dev, "OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}
#endif

//dummy function for unused static functions
void dummy_function(void)
{
	bq2589x_reg_read_mask(NULL, 0, 0, 0);	
	bq2589x_modify_vindpm(NULL, 0);
	vindpm_to_reg(0);
	bq2589x_is_charging_done(NULL);
#ifdef CONFIG_CHARGING_APP_CONTROL   
	bq2589x_set_appropriate_cc(NULL);
#endif
}

#ifdef EXT_CHARGER_POWER_SUPPLY
//static char *bq2589x_chip_name = "ext-charger-dual";
static char *bq2589x_chip_name = "ext-charger";

static enum power_supply_property bq2589x_power_supply_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,	
//	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,	
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_USB_OTG,
};

static int bq2589x_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq2589x_chip *chip = container_of(psy, struct bq2589x_chip, charger_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_get_charging_status(chip);
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_get_charging_status(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->batt_temp;
		break;
/*	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->model;
		break;*/
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = (chip->is_charging_enabled)? 1:0;		
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->iusb_max;
       		 break;
	default:
		pr_err("%s unsupported psy %d\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}

static int bq2589x_power_supply_set_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     const union power_supply_propval *val)
{
	struct bq2589x_chip *chip = container_of(psy, struct bq2589x_chip, charger_psy);
	int value;
	
	switch (psp) {
		//just for charging temp protect function debug 
		case POWER_SUPPLY_PROP_TEMP:
			pr_err("%s debug charging temp, intval = %d\n", __func__, val->intval);
			if((val->intval<-20)&&(val->intval>60))
				chip->temp_debug_flag = false;
			else
			{
				chip->temp_debug_flag = true;
				chip->batt_temp = val->intval;
			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
			bq2589x_enable_charger(chip, val->intval);
#ifdef CONFIG_CHARGER_DUAL
			charger_dual_chip_set_property(POWER_SUPPLY_PROP_CHARGE_ENABLED, val->intval);
#endif
			break;
		case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
			bq2589x_enable_charging(chip, val->intval);
#ifdef CONFIG_CHARGER_DUAL
			charger_dual_chip_set_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, val->intval);
#endif
			break;			
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			value = val->intval;
			bq2589x_set_iusb_max(chip, value);
	       		 break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
			value = val->intval;
			bq2589x_set_ibat_max(chip, value);
			break;			
		case POWER_SUPPLY_PROP_FLASH_ACTIVE:
			pr_warn("%s not support POWER_SUPPLY_PROP_FLASH_ACTIVE \n", __func__);
			break;
		case POWER_SUPPLY_PROP_USB_OTG:
			pr_warn("%s property set otg %d \n", __func__, val->intval);
			bq2589x_turn_otg_vbus(chip, (val->intval?true:false));
			break;
		default:
			pr_err("%s not support power_supply property cmd\n", __func__);
			return -EINVAL;
	}
	return 0;
}

static int bq2589x_power_supply_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:	
		case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		case POWER_SUPPLY_PROP_USB_OTG:
			return 1;
	        		break;
		default:
			break;
	}

	return 0;
}

static int bq2589x_power_supply_init(struct bq2589x_chip *chip)
{
	int ret;

	chip->charger_psy.name = bq2589x_chip_name;//chip->name;
	chip->charger_psy.type = POWER_SUPPLY_TYPE_USB;
	chip->charger_psy.properties = bq2589x_power_supply_props;
	chip->charger_psy.num_properties = ARRAY_SIZE(bq2589x_power_supply_props);
	chip->charger_psy.get_property = bq2589x_power_supply_get_property;
	chip->charger_psy.set_property = bq2589x_power_supply_set_property;
	chip->charger_psy.property_is_writeable = bq2589x_power_supply_property_is_writeable;	

	ret = power_supply_register(&chip->client->dev, &chip->charger_psy);
	if (ret) 
	{
		return ret;
	}

	return 0;
}


static void bq2589x_power_supply_exit(struct bq2589x_chip *chip)
{
	//cancel_delayed_work_sync(&chip->work);
	power_supply_unregister(&chip->charger_psy);

}
#endif

static int bq2589x_regulator_get(struct bq2589x_chip *chip)
{
	int ret;

	chip->vdd = regulator_get(&chip->client->dev, "vdd");
	if (IS_ERR_OR_NULL(chip->vdd)) {
		pr_err("%s: fail to get 1.8v LDO\n", __func__);
		return -3;
	}

	if (regulator_count_voltages(chip->vdd) > 0) {
		ret = regulator_set_voltage(chip->vdd, 1800000, 1800000);
		if (ret) {
			pr_err("%s: regulator set_vtg vdd_reg failed rc=%d\n", __func__, ret);
			regulator_put(chip->vdd);
			return -4;
		}
	}
	
	ret = regulator_enable(chip->vdd);
	if (ret) {
		pr_err("%s: Regulator vdd_reg enable failed rc=%d\n", __func__, ret);
	}
	
    return 0;
}

static irqreturn_t bq2589x_usbid_change_handler(int irq, void *_chip)
{
	//struct bq2589x_chip *chip = _chip;
	bool otg_present = 0;

	//need to be judged by io level
	//otg_present = chip->boost_mode;
	usb_chip_set_property(POWER_SUPPLY_PROP_USB_OTG, otg_present ? 1 : 0);
	if (otg_present)
		pr_info( "OTG detected\n");

	return IRQ_HANDLED;
}

static bool bq2589x_charger_is_factory_mode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

static int bq2589x_charger_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bq2589x_chip *chip;
	int ret;

	printk("%s \n", __func__);
	
	fg_psy = power_supply_get_by_name("battery");
	if (!fg_psy) {
		pr_err("bq25892  fg supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	chip = kzalloc(sizeof(struct bq2589x_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "bq25892 mem alloc failed\n");
		return -ENOMEM;
	}

#ifdef CONFIG_OF	
	if (client->dev.of_node) {
		ret = bq2589x_parse_dt(&client->dev, chip);
		if (ret)
			return ret;

		if (of_property_read_bool(client->dev.of_node, "charger,external-typec")) {
			typec_psy = power_supply_get_by_name(TYPEC_PSY_NAME);
			if (!typec_psy) {
				pr_err("Type-C supply not found, deferring probe\n");
				return -EPROBE_DEFER;
			}
		}
	}
#endif
	chip->client = client;

	bq2589x_regulator_get(chip);

	/*assigning default value for min and max temp*/
	i2c_set_clientdata(client, chip);
	chip->chgr_stat = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->is_factory_cable = false;
	chip->is_factory_mode = bq2589x_charger_is_factory_mode();
	if (chip->is_factory_mode) {
		dev_warn(&client->dev,
			 "Entering Factory Mode\n");
	}

	/* check chip model number */
	ret = bq2589x_get_chip_version(chip);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read err:%d\n", ret);
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		regulator_disable(chip->vdd);
		return -EIO;
	}

	printk("%s chip version %s \n", __func__, chip->ic_name);

	INIT_DELAYED_WORK(&chip->chrg_task_wrkr, bq2589x_task_worker);
	INIT_DELAYED_WORK(&chip->fault_work, bq2589x_fault_worker);
	if(chip->is_factory_mode)
		INIT_DELAYED_WORK(&chip->charger_check_work, bq2589x_charger_check_worker);
	mutex_init(&chip->event_lock);

	/* Initialize the wakelock */
	wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND, "ctp_charger_wakelock");
	wake_lock_init(&chip->irq_wk, WAKE_LOCK_SUSPEND, "charger_irq_wakelock");

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&chip->client->dev);
	pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

	/* create debugfs for maxim registers */
	ret = bq2589x_create_debugfs(chip);
	if (ret < 0) {
		dev_err(&client->dev, "debugfs create failed\n");
	}

#ifdef EXT_CHARGER_POWER_SUPPLY
	ret = bq2589x_power_supply_init(chip);
	if (ret) {
		pr_err("failed to register power supply: %d\n", ret);
	}	
	ret = sysfs_create_group(&chip->charger_psy.dev->kobj,&fs_attr_group);
#else
	ret = sysfs_create_group(&client->dev.kobj,&fs_attr_group);
#endif

	if (ret) {
		dev_err(&client->dev, "failed to setup sysfs ret = %d\n", ret);
	}

	bq2589x_usbid_change_handler(0, chip);
	usb_chip_set_property(POWER_SUPPLY_PROP_DP_DM, POWER_SUPPLY_DP_DM_DPR_DMR); //enable usb dp dm ldo

	/*
	 * Request for charger chip gpio.This will be used to
	 * register for an interrupt handler for servicing charger
	 * interrupts
	 */
	if (gpio_is_valid(chip->irq_gpio)) {
		ret = gpio_request(chip->irq_gpio, "chg_irq_gpio");
		ret = gpio_direction_input(chip->irq_gpio);
		chip->irq = gpio_to_irq(chip->irq_gpio);
	} else {
		pr_err("invalid irq gpio\n");	
		chip->irq = -1;
	}

	if (chip->irq < 0) {
		pr_err("%s chgr_int_n GPIO is not available\n", __func__);
	} else {
		//tlmm_set_config_pullup(chip->irq_gpio, 1);
		ret = request_threaded_irq(chip->irq,
				bq2589x_irq_isr, bq2589x_irq_thread,
				IRQF_TRIGGER_FALLING, "bq2589x", chip);
//				IRQF_TRIGGER_HIGH, "bq2589x", chip);
		if (ret) {
			pr_warn("%s failed to register irq for pin %d as %d\n", __func__, chip->irq_gpio, IRQF_TRIGGER_FALLING);
		} else {
			pr_warn("%s registered charger irq for pin %d\n", __func__, chip->irq_gpio);
		}
	}

#if 1
	if (gpio_is_valid(chip->chg_en_gpio)) {
        		ret = gpio_request(chip->chg_en_gpio, "chg_en_gpio");
		gpio_direction_output(chip->chg_en_gpio,0);
		gpio_set_value(chip->chg_en_gpio,0);
	}
	else
		pr_err("%s chg_en_gpio is invalid\n", __func__);
#endif
#ifdef LENOVO_OTG_USB_SHORT	
	if(gpio_is_valid(chip->otg_usb_short_gpio)) {
	ret = gpio_request(chip->otg_usb_short_gpio, "otg_usb_short_gpio");	
		gpio_direction_output(chip->otg_usb_short_gpio,0);
		gpio_set_value(chip->otg_usb_short_gpio,0);
	}
#endif

	bq2589x_hw_init(chip);

#ifdef CONFIG_OTG_SUPPORT
	bq2589x_regulator_init(&client->dev, chip);
#endif
	bq2589x_enable_charger(chip, 1);

	bq2589x_dump_registers(chip);
	
	schedule_delayed_work(&chip->chrg_task_wrkr, msecs_to_jiffies(1000));

	pr_info("%s prob success\n", __func__);
	
	return 0;
}

static int bq2589x_remove(struct i2c_client *client)
{
	struct bq2589x_chip *chip = i2c_get_clientdata(client);

	bq2589x_remove_debugfs(chip);

#ifdef EXT_CHARGER_POWER_SUPPLY	
	bq2589x_power_supply_exit(chip);
#endif

	if (chip->irq > 0)
		free_irq(chip->irq, chip);

	i2c_set_clientdata(client, NULL);
	wake_lock_destroy(&chip->wakelock);

	kfree(chip);
	return 0;
}

static void bq2589x_shutdown(struct i2c_client *client)
{
	struct bq2589x_chip *chip = i2c_get_clientdata(client);

	printk("bq2589x_shutdown \r\n");

	wake_lock_destroy(&chip->wakelock);

	if (chip->irq > 0) {
		free_irq(chip->irq, chip);
	}
	cancel_delayed_work_sync(&chip->chrg_task_wrkr);
	cancel_delayed_work_sync(&chip->fault_work);
	
	if(1)
	{
		if(gpio_is_valid(chip->chg_en_gpio)) {		
			printk("disable charging for shutting down\r\n");
			gpio_set_value(chip->chg_en_gpio,1);
		}
		
		bq2589x_set_iusb_max(chip, 0);
		
		msleep(250);
	}
	
	bq2589x_reset_regs(chip);

	printk("bq2589x_shutdown complete\r\n");
}

#ifdef CONFIG_PM
static int bq2589x_suspend(struct device *dev)
{
	struct bq2589x_chip *chip = dev_get_drvdata(dev);
	
	if (chip->irq > 0) {
		/*
		 * Once the WDT is expired all bq2589x registers gets
		 * set to default which means WDT is programmed to 40s
		 * and if there is no charger connected, no point
		 * feeding the WDT. Since reg07[1] is set to default,
		 * charger will interrupt SOC every 40s which is not
		 * good for S3. In this case we need to free chgr_int_n
		 * interrupt so that no interrupt from charger wakes
		 * up the platform in case of S3. Interrupt will be
		 * re-enabled on charger connect.
		 */
		free_irq(chip->irq, chip);
		//enable_irq_wake(chip->irq);
	}
	cancel_delayed_work_sync(&chip->chrg_task_wrkr);
	cancel_delayed_work_sync(&chip->fault_work);

	//regulator operation
#if 1	
	{
		int ret= regulator_disable(chip->vdd);
		if (ret) {
			pr_err("%s: Regulator vdd_reg disable failed rc=%d\n", __func__, ret);
		}
	}
#endif

	dev_dbg(&chip->client->dev, "bq2589x suspend,cancel charger worker\n");
	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct bq2589x_chip *chip = dev_get_drvdata(dev);
	int ret;

	//regulator operation
#if 1	
	ret = regulator_enable(chip->vdd);
	if (ret) {
		pr_err("%s: Regulator vdd_reg enable failed rc=%d\n", __func__, ret);
	}
#endif

	if (chip->irq > 0) {
		//tlmm_set_config_pullup(chip->irq_gpio, 1);		
		ret = request_threaded_irq(chip->irq,
				bq2589x_irq_isr, bq2589x_irq_thread,
				IRQF_TRIGGER_FALLING, "bq2589x", chip);
		if (ret) {
			dev_warn(dev, "failed to register irq for pin %d\n",
				chip->irq_gpio);
		} else {
			dev_warn(dev, "registered charger irq for pin %d\n",
				chip->irq_gpio);
		}
		//disable_irq_wake(chip->irq);
	}
	schedule_delayed_work(&chip->fault_work, 0);
	schedule_delayed_work(&chip->chrg_task_wrkr, msecs_to_jiffies(20));

	dev_dbg(&chip->client->dev, "bq2589x resume,shecdule charger work and fault work\n");
	return 0;
}
#else
#define bq2589x_suspend NULL
#define bq2589x_resume NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bq2589x_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq2589x_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq2589x_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bq2589x_runtime_suspend	NULL
#define bq2589x_runtime_resume		NULL
#define bq2589x_runtime_idle		NULL
#endif

MODULE_DEVICE_TABLE(i2c, bq2589x_id);

static const struct dev_pm_ops bq2589x_pm_ops = {
	.suspend		= bq2589x_suspend,
	.resume			= bq2589x_resume,
	.runtime_suspend	= bq2589x_runtime_suspend,
	.runtime_resume		= bq2589x_runtime_resume,
	.runtime_idle		= bq2589x_runtime_idle,
};

#ifdef CONFIG_OF
static const struct of_device_id bq2589x_match[] = {
	{ .compatible = "ti,bq2589x" },
	{ },
};
#endif

static const struct i2c_device_id bq2589x_id[] = {
	{ DEV_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2589x_id);

static struct i2c_driver bq2589x_battery_driver = {
	.driver = {
		.name = DEV_NAME,
		.owner	= THIS_MODULE,
		.pm = &bq2589x_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(bq2589x_match),
#endif
	},
	.probe = bq2589x_charger_probe,
	.remove = bq2589x_remove,
	.shutdown = bq2589x_shutdown,
	.id_table = bq2589x_id,
};

static inline int bq2589x_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq2589x_battery_driver);
	printk("%s:bq2589x register_i2c driver\n",__func__);
	if (ret)
		printk(KERN_ERR "Unable to register bq2589x i2c driver\n");

	return ret;
}

static inline void bq2589x_battery_i2c_exit(void)
{
	i2c_del_driver(&bq2589x_battery_driver);
}

static int __init bq2589x_battery_init(void)
{
	int ret;
	ret = bq2589x_battery_i2c_init();

	pr_notice("bq2589x %s %d\n", __func__, ret);
	
	return ret;
}
module_init(bq2589x_battery_init);

static void __exit bq2589x_battery_exit(void)
{
	bq2589x_battery_i2c_exit();
}
module_exit(bq2589x_battery_exit);

