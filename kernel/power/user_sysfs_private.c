/*
* Copyright (C) 2012 lenovo, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/uaccess.h> /* sys_sync */
#include <linux/rtc.h> /* sys_sync */
/* yangjq, 2011-12-16, Add for vreg, START */
#include <linux/platform_device.h>
/* yangjq, 2011-12-16, Add for vreg, END */
#include <linux/err.h>
////#include <mach/pmic.h>
#include <linux/regulator/consumer.h>

#include <linux/io.h>
#include <linux/of_gpio.h>

static u32 *tz_config = NULL;
static int tz_pin_num = 0;
static struct kobject *sysfs_private_kobj;

#define GPIO_CFG(gpio, func, dir, pull, drvstr) \
	((((gpio) & 0x3FF) << 4)        |	  \
	 ((func) & 0xf)                  |	  \
	 (((dir) & 0x1) << 14)           |	  \
	 (((pull) & 0x3) << 15)          |	  \
	 (((drvstr) & 0xF) << 17))

/* GP PIN TYPE REG MASKS */
#define TLMM_GP_DRV_SHFT		6
#define TLMM_GP_DRV_MASK		0x7
#define TLMM_GP_PULL_SHFT		0
#define TLMM_GP_PULL_MASK		0x3
#define TLMM_GP_DIR_SHFT		9
#define TLMM_GP_DIR_MASK		1
#define TLMM_GP_FUNC_SHFT		2
#define TLMM_GP_FUNC_MASK		0xF
#define GPIO_OUT_BIT			1
#define GPIO_IN_BIT			0
#define GPIO_OE_BIT			9
/**
 * extract GPIO pin from bit-field used for gpio_tlmm_config
 */
#define GPIO_PIN(gpio_cfg)    (((gpio_cfg) >>  4) & 0x3ff)
#define GPIO_FUNC(gpio_cfg)   (((gpio_cfg) >>  0) & 0xf)
#define GPIO_DIR(gpio_cfg)    (((gpio_cfg) >> 14) & 0x1)
#define GPIO_PULL(gpio_cfg)   (((gpio_cfg) >> 15) & 0x3)
#define GPIO_DRVSTR(gpio_cfg) (((gpio_cfg) >> 17) & 0xf)
#define TLMM_GP_CFG(reg_base, pin)	(reg_base + 0x0 + \
						 0x1000* (pin))
#define TLMM_GP_INOUT(reg_base, pin)	(reg_base + 0x4 + \
				 0x1000 * (pin))
/* chenyb1, 20130515, Add sysfs for gpio's debug, START */
#define TLMM_NUM_GPIO 141

#define HAL_OUTPUT_VAL(config)    \
         (((config)&0x40000000)>>30)

extern void *tlmm_reg_base;
static int tlmm_get_cfg(unsigned gpio, unsigned* cfg)
{
	unsigned flags;

	if(tlmm_reg_base == NULL)
		return -1;
	BUG_ON(gpio >= TLMM_NUM_GPIO);
	//printk("%s(), gpio=%d, addr=0x%08x\n", __func__, gpio, (unsigned int)GPIO_CONFIG(gpio));

#if 0
	flags = ((GPIO_DIR(config) << 9) & (0x1 << 9)) |
		((GPIO_DRVSTR(config) << 6) & (0x7 << 6)) |
		((GPIO_FUNC(config) << 2) & (0xf << 2)) |
		((GPIO_PULL(config) & 0x3));
#else
	flags = readl_relaxed(TLMM_GP_CFG(tlmm_reg_base, gpio));
#endif
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	*cfg = GPIO_CFG(gpio, (flags >> TLMM_GP_FUNC_SHFT) & 0xf, (flags >> TLMM_GP_DIR_SHFT) & 0x1, flags & 0x3, (flags >> TLMM_GP_DRV_SHFT) & 0x7);

	return 0;
}

int tlmm_set_config(unsigned config)
{
	unsigned int flags;
	unsigned gpio = GPIO_PIN(config);
	void __iomem *cfg_reg = TLMM_GP_CFG(tlmm_reg_base, gpio );
	
	if(tlmm_reg_base == NULL)
		return -1;
	if (gpio > TLMM_NUM_GPIO)
		return -EINVAL;

	printk("%s(), %d,gpio=%d\n", __func__, __LINE__, gpio);

	config = (config & ~0x40000000);
	flags = readl_relaxed(cfg_reg);
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	
	flags = ((GPIO_DIR(config) & TLMM_GP_DIR_MASK) << TLMM_GP_DIR_SHFT) |
		((GPIO_DRVSTR(config) & TLMM_GP_DRV_MASK) << TLMM_GP_DRV_SHFT) |
		((GPIO_FUNC(config) & TLMM_GP_FUNC_MASK) << TLMM_GP_FUNC_SHFT) |
		((GPIO_PULL(config) & TLMM_GP_PULL_MASK));

	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

#if 0
	/*set func*/
	cfg_reg = TLMMV4_GP_CFG(tlmm_reg_base, gpio);
	flags = readl_relaxed(cfg_reg);
	flags &= ~(TLMMV4_GP_FUNC_MASK << TLMMV4_GP_FUNC_SHFT);
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	
	flags |= (GPIO_FUNC(config) << TLMMV4_GP_FUNC_SHFT);
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

	/* set DIR */
	cfg_reg = TLMMV4_GP_CFG(tlmm_reg_base, gpio);
	flags = readl_relaxed(cfg_reg);
	if (GPIO_DIR(config))
	{
		flags |= BIT(GPIO_OE_BIT);
	}
	else
	{
		flags &= ~BIT(GPIO_OE_BIT);
	}
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

	/* set PULL */
	flags = readl_relaxed(cfg_reg);
	flags |= GPIO_PULL(config) & 0x3;
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);

	/* set DRVSTR */
	flags = readl_relaxed(cfg_reg);
	flags |= drv_str_to_rval(GPIO_DRVSTR(config));
	printk("%s(), %d, flags=%x\n", __func__, __LINE__, flags);
	writel_relaxed(flags, cfg_reg);
#endif
	return 0;
}
static int tlmm_dump_cfg(char* buf,unsigned gpio, unsigned cfg, int output_val)
{
	static char* drvstr_str[] = { "2", "4", "6", "8", "10", "12", "14", "16" }; // mA
	static char*   pull_str[] = { "N", "D", "K", "U" };	 // "NO_PULL", "PULL_DOWN", "KEEPER", "PULL_UP"
	static char*    dir_str[] = { "I", "O" }; // "Input", "Output"	 
	char func_str[20];
	
	char* p = buf;

	int drvstr   = GPIO_DRVSTR(cfg);
	int pull     = GPIO_PULL(cfg);
	int dir      = GPIO_DIR(cfg);
	int func     = GPIO_FUNC(cfg);

	//printk("%s(), drvstr=%d, pull=%d, dir=%d, func=%d\n", __func__, drvstr, pull, dir, func);
	sprintf(func_str, "%d", func);

	p += sprintf(p, "%d:0x%x %s%s%s%s", gpio, cfg,
			func_str, pull_str[pull], dir_str[dir], drvstr_str[drvstr]);

	p += sprintf(p, " = %d", output_val);

	p += sprintf(p, "\n");	
			
	return p - buf;		
}

static int tlmm_dump_header(char* buf)
{
	char* p = buf;
	p += sprintf(p, "bit   0~3: function. (0 is GPIO)\n");
	p += sprintf(p, "bit  4~13: gpio number\n");
	p += sprintf(p, "bit    14: 0: input, 1: output\n");
	p += sprintf(p, "bit 15~16: pull: NO_PULL, PULL_DOWN, KEEPER, PULL_UP\n");
	p += sprintf(p, "bit 17~20: driver strength. \n");
	p += sprintf(p, "0:GPIO\n");
	p += sprintf(p, "N:NO_PULL  D:PULL_DOWN  K:KEEPER  U:PULL_UP\n");
	p += sprintf(p, "I:Input  O:Output\n");
	p += sprintf(p, "2:2, 4, 6, 8, 10, 12, 14, 16 mA (driver strength)\n\n");
	return p - buf;
}

static int tlmm_get_inout(unsigned gpio)
{
	void __iomem *inout_reg = TLMM_GP_INOUT(tlmm_reg_base, gpio);

	if(tlmm_reg_base == NULL)
		return -1;
	return readl_relaxed(inout_reg) & BIT(GPIO_IN_BIT);
}

void tlmm_set_inout(unsigned gpio, unsigned val)
{
	void __iomem *inout_reg = TLMM_GP_INOUT(tlmm_reg_base, gpio);

	if(tlmm_reg_base == NULL)
		return;
	writel_relaxed(val ? BIT(GPIO_OUT_BIT) : 0, inout_reg);
}

int tlmm_dump_info(char* buf, int tlmm_num)
{
	unsigned i, j;
	char* p = buf;
	unsigned cfg;
	int output_val = 0;
	int tz_flag = 0;

	if(tlmm_num >= 0 && tlmm_num < TLMM_NUM_GPIO) {
		tlmm_get_cfg(tlmm_num, &cfg);
		output_val = tlmm_get_inout(tlmm_num);
			
		p += tlmm_dump_cfg(p, tlmm_num, cfg, output_val);
	} else {

		p += tlmm_dump_header(p);
		p += sprintf(p, "Standard Format: gpio_num  function  pull  direction  strength [output_value]\n");
		p += sprintf(p, "Shortcut Format: gpio_num  output_value\n");
		p += sprintf(p, " e.g.  'echo  20 0 D O 2 1'  ==> set pin 20 as GPIO output and the output = 1 \n");
		p += sprintf(p, " e.g.  'echo  20 1'  ==> set output gpio pin 20 output = 1 \n");
		printk("%s(), %d, TLMM_BASE=%lx\n", __func__, __LINE__, (unsigned long int)(void *)tlmm_reg_base);
		for(i = 0; i < TLMM_NUM_GPIO; ++i) {
			for(j = 0; j < tz_pin_num; j++) {
				if(i == tz_config[j]) {
					tz_flag = 1;
					continue;
				}
			}
			if(tz_flag == 1) {
				tz_flag = 0;
				continue;
			}
			tlmm_get_cfg(i, &cfg);
			output_val = tlmm_get_inout(i);
			
			p += tlmm_dump_cfg(p, i, cfg, output_val);
		}
		printk("%s(), %d\n", __func__, __LINE__);
		p+= sprintf(p, "(%ld)\n", (unsigned long)(p - buf)); // only for debug reference
	}
	return p - buf;	
}

/* save tlmm config before sleep */
static int before_sleep_fetched;
module_param(before_sleep_fetched,int,0644);
static unsigned before_sleep_configs[TLMM_NUM_GPIO];
void tlmm_before_sleep_save_configs(void)
{
	unsigned i;

	//only save tlmm configs when it has been fetched
	if (!before_sleep_fetched)
		return;

	printk("%s(), before_sleep_fetched=%d\n", __func__, before_sleep_fetched);
	before_sleep_fetched = false;
	for(i = 0; i < TLMM_NUM_GPIO; ++i) {
		unsigned cfg;
		int output_val = 0;

		tlmm_get_cfg(i, &cfg);
		output_val = tlmm_get_inout(i);

		before_sleep_configs[i] = cfg | (output_val << 30);
	}
}

int tlmm_before_sleep_dump_info(char* buf)
{
	unsigned i;
	char* p = buf;

	p += sprintf(p, "tlmm_before_sleep:\n");
	if (!before_sleep_fetched) {
		before_sleep_fetched = true;

		p += tlmm_dump_header(p);
		
		for(i = 0; i < TLMM_NUM_GPIO; ++i) {
			unsigned cfg;
			int output_val = 0;

			cfg = before_sleep_configs[i];
			output_val = HAL_OUTPUT_VAL(cfg);
			//cfg &= ~0x40000000;
			p += tlmm_dump_cfg(p, i, cfg, output_val);
		}
		p+= sprintf(p, "(%ld)\n", (unsigned long)(p - buf)); // only for debug reference
	}
	return p - buf;	
}

/* set tlmms config before sleep */
static int before_sleep_table_enabled = 0;
module_param(before_sleep_table_enabled,int,0644);
static unsigned before_sleep_table_configs[TLMM_NUM_GPIO];
void tlmm_before_sleep_set_configs(void)
{
	int res;
	unsigned i;

	//only set tlmms before sleep when it's enabled
	if (!before_sleep_table_enabled)
		return;

	printk("%s(), before_sleep_table_enabled=%d\n", __func__, before_sleep_table_enabled);
	for(i = 0; i < TLMM_NUM_GPIO; ++i) {
		unsigned cfg;
		int gpio;
		int dir;
		int func;
		int output_val = 0;

		cfg = before_sleep_table_configs[i];

		gpio = GPIO_PIN(cfg);
		if(gpio != i)//(cfg & ~0x20000000) == 0 || 
			continue;

		output_val = HAL_OUTPUT_VAL(cfg);
		//Clear the output value
		//cfg &= ~0x40000000;
		dir = GPIO_DIR(cfg);
		func = GPIO_FUNC(cfg);

		printk("%s(), [%d]: 0x%x\n", __func__, i, cfg);
		res = tlmm_set_config(cfg & ~0x40000000);
		if(res < 0) {
			printk("Error: Config failed.\n");
		}
		
		if((func == 0) && (dir == 1)) // gpio output
			tlmm_set_inout(i, output_val);
	}
}

int tlmm_before_sleep_table_set_cfg(unsigned gpio, unsigned cfg)
{
	//BUG_ON(gpio >= TLMM_NUM_GPIO && GPIO_PIN(cfg) != 0xff);
	if (gpio >= TLMM_NUM_GPIO && gpio != 255 && gpio != 256) {
		printk("gpio >= TLMM_NUM_GPIO && gpio != 255 && gpio != 256!\n");
		return -1;
	}

	if(gpio < TLMM_NUM_GPIO)
	{
		before_sleep_table_configs[gpio] = cfg;// | 0x20000000
		before_sleep_table_enabled = true;
	}
	else if(gpio == 255)
		before_sleep_table_enabled = true;
	else if(gpio == 256)
		before_sleep_table_enabled = false;

	return 0;
}

int tlmm_before_sleep_table_dump_info(char* buf)
{
	unsigned i;
	char* p = buf;

	p += tlmm_dump_header(p);
	p += sprintf(p, "Format: gpio_num  function  pull  direction  strength [output_value]\n");
	p += sprintf(p, " e.g.  'echo  20 0 D O 2 1'  ==> set pin 20 as GPIO output and the output = 1 \n");
	p += sprintf(p, " e.g.  'echo  20'  ==> disable pin 20's setting \n");
	p += sprintf(p, " e.g.  'echo  255'  ==> enable sleep table's setting \n");
	p += sprintf(p, " e.g.  'echo  256'  ==> disable sleep table's setting \n");

	for(i = 0; i < TLMM_NUM_GPIO; ++i) {
		unsigned cfg;
		int output_val = 0;

		cfg = before_sleep_table_configs[i];
		output_val = HAL_OUTPUT_VAL(cfg);
		//cfg &= ~0x40000000;
		p += tlmm_dump_cfg(p, i, cfg, output_val);
	}
	p+= sprintf(p, "(%ld)\n", (unsigned long)(p - buf)); // only for debug reference
	return p - buf;
}
/* yangjq, 20130515, Add sysfs for gpio's debug, END */


#define private_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#ifdef CONFIG_LENOVO_PM_LOG_TLMM//TBD
//chenyb1, 2015-2-3, Add a sysfs interface for modem's sim card checking, START
#define TLMM_GPIO_SIM 60
static ssize_t tlmm_sim_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *p = buf;
	int output_val = 0;

	output_val = tlmm_get_inout(TLMM_GPIO_SIM);
	p += sprintf(p, "%d", output_val);

	return (p - buf);
}

static ssize_t tlmm_sim_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support yet.\n", __func__);

	return -EPERM;
}
//yangjq, 2015-2-3, Add a sysfs interface for modem's sim card checking, END

static int tlmm_num = -1;
static ssize_t tlmm_num_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += sprintf(p, "A single gpio[0, %d] to be checked by cat tlmm\n", TLMM_NUM_GPIO);
	p += sprintf(p, "-1 to check all %d gpios by cat tlmm\n", TLMM_NUM_GPIO+1);
	p += sprintf(p, "%d\n", tlmm_num);
	p += sprintf(p, "TLMM_BASE=%lx\n",  (unsigned long int)(void *)tlmm_reg_base);

	printk("%s(), %d, TLMM_BASE=%lx\n", __func__, __LINE__, (unsigned long int)(void *)tlmm_reg_base);
	return p - buf;
}

static ssize_t tlmm_num_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int gpio;
	int res;
	
	res = sscanf(buf, "%d", &gpio);
	printk("res=%d. %d\n", res, gpio);
	
	if(res != 1)
		goto tlmm_num_store_wrong_para;

	if(gpio >= TLMM_NUM_GPIO)
		goto tlmm_num_store_wrong_para;

	tlmm_num = gpio;
	printk("tlmm_num: %d\n", tlmm_num);

	goto tlmm_num_store_ok;
		
tlmm_num_store_wrong_para:
	printk("Wrong Input.\n");	
	printk("Format: gpio_num\n");
	printk("      gpio_num: 0 ~ 145\n");

tlmm_num_store_ok:	
	return n;
}

static ssize_t tlmm_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
#if 1 //TBD
	p += tlmm_dump_info(buf, tlmm_num);
#endif
	return p - buf;
}

static ssize_t tlmm_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	char pull_c, dir_c;
	int gpio, func, pull, dir, drvstr, output_val;
	unsigned cfg;
	int res;
	
	res = sscanf(buf, "%d %d %c %c %d %d", &gpio, &func, &pull_c, &dir_c, &drvstr, &output_val);
	printk("res=%d.  %d %d %c %c %d %d\n", res, gpio, func, pull_c, dir_c, drvstr, output_val);
	
	//Add a shortcut wrting format to change an output gpio's value
	if(res == 2 && gpio < TLMM_NUM_GPIO && (func == 0 || func == 1)) {
		output_val = func;
		goto tlmm_store_only_output_val;
	}
	if((res != 5) && (res != 6))
		goto tlmm_store_wrong_para;

	if(gpio >= TLMM_NUM_GPIO)
		goto tlmm_store_wrong_para;
		
	if('N' == pull_c)
		pull = 0;
	else if('D' == pull_c)
		pull = 1;
	else if('K' == pull_c)
		pull = 2;
	else if('U' == pull_c)
		pull = 3;
	else 
		goto tlmm_store_wrong_para;

	if('I' == dir_c)
		dir = 0;
	else if('O' == dir_c)
		dir = 1;
	else 
		goto tlmm_store_wrong_para;
	
	drvstr = drvstr/2 - 1; // 2mA -> 0, 4mA -> 1, 6mA -> 2, ...	
	if(drvstr > 7)
		goto tlmm_store_wrong_para;
	
	if(output_val > 1)
		goto tlmm_store_wrong_para;
		
	printk("final set: %d %d %d %d %d %d\n", gpio, func, pull, dir, drvstr, output_val);

	cfg = GPIO_CFG(gpio, func, dir, pull, drvstr);	
#if 1 
	res = tlmm_set_config(cfg);
	if(res < 0) {
		printk("Error: Config failed.\n");
		goto tlmm_store_wrong_para;
	}
#endif
	printk("final set: %d %d %d %d %d %d\n", gpio, func, pull, dir, drvstr, output_val);
	if((func == 0)  && (dir == 1)) // gpio output
tlmm_store_only_output_val:
		tlmm_set_inout(gpio, output_val);
	
	goto tlmm_store_ok;
		
tlmm_store_wrong_para:
	printk("Wrong Input.\n");	
	printk("Standard Format: gpio_num  function  pull  direction  strength [output_value]\n");
	printk("Shortcut Format: gpio_num  output_value\n");
	printk("      gpio_num: 0 ~ 145\n");
	printk("      function: number, where 0 is GPIO\n");	
	printk("      pull: 'N': NO_PULL, 'D':PULL_DOWN, 'K':KEEPER, 'U': PULL_UP\n");	
	printk("      direction: 'I': Input, 'O': Output\n");	
	printk("      strength:  2, 4, 6, 8, 10, 12, 14, 16\n");	
	printk("      output_value:  Optional. 0 or 1. vaild if GPIO output\n");	
	printk(" e.g.  'echo  20 0 D I 2'  ==> set pin 20 as GPIO input \n");	
	printk(" e.g.  'echo  20 0 D O 2 1'  ==> set pin 20 as GPIO output and the output = 1 \n");	
	printk(" e.g.  'echo  20 1'  ==> set output gpio pin 20 output = 1 \n");

tlmm_store_ok:	
	return n;
}

/* Set GPIO's sleep config from sysfs */
static ssize_t tlmm_before_sleep_table_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
#if 1 //TBD
	p += tlmm_before_sleep_table_dump_info(buf);
#endif
	return p - buf;
}

static ssize_t tlmm_before_sleep_table_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	char pull_c, dir_c;
	int gpio, func = 0, pull = 0, dir = 0, drvstr = 0, output_val = 0;
	int ignore;
	unsigned cfg;
	int res;
	
	res = sscanf(buf, "%d %d %c %c %d %d", &gpio, &func, &pull_c, &dir_c, &drvstr, &output_val);
	printk("res=%d.  %d %d %c %c %d %d\n", res, gpio, func, pull_c, dir_c, drvstr, output_val);
	
	if(1 == res) { // if only gpio, means ingore(disable) the gpio's sleep config 
		ignore = 1;
		printk("final set: to disable gpio %d sleep config\n", gpio);
	}
	else {
		ignore = 0;
	
		if((res != 5) && (res != 6)) 
			goto tlmm_before_sleep_table_store_wrong_para;

		if(gpio >= TLMM_NUM_GPIO)
			goto tlmm_before_sleep_table_store_wrong_para;
			
		if('N' == pull_c)
			pull = 0;
		else if('D' == pull_c)
			pull = 1;
		else if('K' == pull_c)
			pull = 2;
		else if('U' == pull_c)
			pull = 3;
		else 
			goto tlmm_before_sleep_table_store_wrong_para;

		if('I' == dir_c)
			dir = 0;
		else if('O' == dir_c)
			dir = 1;
		else 
			goto tlmm_before_sleep_table_store_wrong_para;
		
		drvstr = drvstr/2 - 1; // 2mA -> 0, 4mA -> 1, 6mA -> 2, ...	
		if(drvstr > 7)
			goto tlmm_before_sleep_table_store_wrong_para;
				
		printk("final set: %d %d %d %d %d\n", gpio, func, pull, dir, drvstr);
	}
		 
	cfg = GPIO_CFG(ignore ? 0xff : gpio, func, dir, pull, drvstr);
#if 1 //TBD
	res = tlmm_before_sleep_table_set_cfg(gpio, cfg | (output_val << 30));
	if(res < 0) {
		printk("Error: Config failed.\n");
		goto tlmm_before_sleep_table_store_wrong_para;
	}
#endif
	
	goto tlmm_before_sleep_table_store_ok;
		
tlmm_before_sleep_table_store_wrong_para:
	printk("Wrong Input.\n");	
	printk("Format: refer to tlmm's format except  'echo gpio_num > xxx' to disable the gpio's setting\n");	

tlmm_before_sleep_table_store_ok:
	return n;
}

extern int tlmm_before_sleep_dump_info(char* buf);
static ssize_t tlmm_before_sleep_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
#if 1 //TBD
	p += tlmm_before_sleep_dump_info(buf);
#endif
	return p - buf;
}

static ssize_t tlmm_before_sleep_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);
	return n;
}
#endif
extern int vreg_dump_info(char* buf);
static ssize_t vreg_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += vreg_dump_info(buf);
	return p - buf;
}

//extern void vreg_config(struct vreg *vreg, unsigned on, unsigned mv);
#if 0
extern void regulator_config(struct regulator *reg, unsigned on, unsigned mv);
#endif
static ssize_t vreg_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);
	return n;
}

extern int vreg_before_sleep_dump_info(char* buf);
static ssize_t vreg_before_sleep_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += vreg_before_sleep_dump_info(buf);
	return p - buf;
}

static ssize_t vreg_before_sleep_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);
	return n;
}

static ssize_t clk_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
#if 0
	extern int clk_dump_info(char* buf);
#endif //0
	char *s = buf;

	// show all enabled clocks
#if 0
	//s += sprintf(s, "\nEnabled Clocks:\n");
	s += clk_dump_info(s);
#else
	//Use interface /sys/kernel/debug/clk/enabled_clocks provided by krait instead
	s += sprintf(s, "cat /sys/kernel/debug/clk/enabled_clocks to show Enabled Clocks\n");
#endif //0
	
	return (s - buf);
}

static ssize_t clk_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);

	return -EPERM;
}
/* chenyb1 add thermal config for benchmark 20150612 begin*/
unsigned int thermal_bm_flag = 0;
static ssize_t thermal_bm_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	printk(KERN_ERR "%s,thermal_bm_flag=%d\n", __func__, thermal_bm_flag);
	
	return snprintf(buf, 10, "%d\n", thermal_bm_flag);
}

static ssize_t thermal_bm_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	const char *s = buf;
	
	thermal_bm_flag = s[0] - '0';
	sysfs_notify(sysfs_private_kobj, NULL, "thermal_bm");
	printk(KERN_ERR "%s,thermal_bm_flag=%d\n", __func__, thermal_bm_flag);

	return n;
}
/* chenyb1 add thermal config for benchmark 20150612 begin*/

extern unsigned long acpu_clk_get_rate(int cpu);
extern int wakelock_dump_info(char* buf);
static ssize_t pm_status_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	char *s = buf;
	unsigned long rate; // khz
	int cpu;

	// show CPU clocks
	for (cpu = 0; cpu < nr_cpu_ids; cpu++) {
		s += sprintf(s, "APPS[%d]:", cpu);
		if (cpu_online(cpu)) {
#if 0
			//acpuclk_get_rate doesn't work because acpuclk_data is no longer available in krait
			rate = acpuclk_get_rate(cpu); // khz
			s += sprintf(s, "(%3lu MHz); \n", rate / 1000);
#else
			//Call acpu_clk_get_rate added in clock-krait-8974.c
			rate = acpu_clk_get_rate(cpu); // hz
			s += sprintf(s, "(%3lu MHz); \n", rate / 1000000);
#endif
		} else {
			s += sprintf(s, "sleep; \n");
		}
	}

	s += wakelock_dump_info(s);
	
	return (s - buf);
}

static ssize_t pm_status_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support yet.\n", __func__);

	return -EPERM;
}

static unsigned pm_wakeup_fetched = true;
static ssize_t pm_wakeup_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	char *s = buf;

	if (!pm_wakeup_fetched) {
		pm_wakeup_fetched = true;
		s += sprintf(s, "true");
	} else
		s += sprintf(s, "false");
	
	return (s - buf);
}

static ssize_t pm_wakeup_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support yet.\n", __func__);

	return -EPERM;
}

// create a sys interface for power monitor 
#define PM_MONITOR_BUF_LEN 128
static char pm_monitor_buf[PM_MONITOR_BUF_LEN] = {0};
static ssize_t pm_monitor_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return snprintf(buf, PM_MONITOR_BUF_LEN, "%s", pm_monitor_buf);
}

static ssize_t pm_monitor_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	if (n>(PM_MONITOR_BUF_LEN-1) || n<=0)
	{
		printk(KERN_ERR "%s: %d\n", __func__, (int)n);
		return -EPERM;
	}
	
	snprintf(pm_monitor_buf, PM_MONITOR_BUF_LEN, "%s", buf);
	pm_monitor_buf[n] = '\0';
	printk(KERN_ERR "%s: %s,%d\n", __func__, pm_monitor_buf, (int)n);

	return n;
}

#ifdef CONFIG_LENOVO_PM_LOG_TLMM//TBD
private_attr(tlmm_sim);
private_attr(tlmm_num);
private_attr(tlmm);
private_attr(tlmm_before_sleep_table);
private_attr(tlmm_before_sleep);
#endif
private_attr(vreg_before_sleep);
private_attr(vreg);
private_attr(clk);
private_attr(thermal_bm);
private_attr(pm_status);
private_attr(pm_wakeup);
private_attr(pm_monitor);

static struct attribute *g_private_attr[] = {
#ifdef CONFIG_LENOVO_PM_LOG_TLMM//TBD
	&tlmm_sim_attr.attr,
	&tlmm_num_attr.attr,
	&tlmm_attr.attr,
	&tlmm_before_sleep_table_attr.attr,
	&tlmm_before_sleep_attr.attr,
#endif
	&vreg_attr.attr,
	&vreg_before_sleep_attr.attr,
	&clk_attr.attr,
	&thermal_bm_attr.attr,
	&pm_status_attr.attr,
	&pm_wakeup_attr.attr,
	&pm_monitor_attr.attr,
	NULL,
};

static struct attribute_group private_attr_group = {
	.attrs = g_private_attr,
};

#define SLEEP_LOG
#ifdef SLEEP_LOG
#define WRITE_SLEEP_LOG
#define MAX_WAKEUP_IRQ 8

enum {
	DEBUG_SLEEP_LOG = 1U << 0,
	DEBUG_WRITE_LOG = 1U << 1,
	DEBUG_WAKEUP_IRQ = 1U << 2,
	DEBUG_RPM_SPM_LOG = 1U << 3,
	DEBUG_RPM_CXO_LOG = 1U << 4,
	DEBUG_ADSP_CXO_LOG = 1U << 5,
	DEBUG_MODEM_CXO_LOG = 1U << 6,
	DEBUG_WCNSS_CXO_LOG = 1U << 7,
};
static int debug_mask;// = DEBUG_WRITE_LOG;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct sleep_log_t {
	char time[18];
	long timesec;
	unsigned int log;
	uint32_t maoints[2];
	int wakeup_irq[MAX_WAKEUP_IRQ];
	int wakeup_gpio;
//31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
//bit1-0=00 :try to sleep; bit 1-0 = 01 : leave from sleep    ;bit1-0=10:fail to sleep
//bit31-bit24 : return value
};

struct rpm_smem_state_t {
	uint32_t wakeup_ints[2];
};
struct rpm_smem_state_t rpm_smem_state_data;

#define TRY_TO_SLEEP  (0)
#define LEAVE_FORM_SLEEP  (1)
#define FAIL_TO_SLEEP  (2)

#define SLEEP_LOG_LENGTH 80

struct sleep_log_t sleep_log_array[SLEEP_LOG_LENGTH];
int sleep_log_pointer = 0;
int sleep_log_count = 0;
int enter_times = 0;

static int irq_wakeup_saved = MAX_WAKEUP_IRQ;
static int irq_wakeup_irq[MAX_WAKEUP_IRQ];
static int irq_wakeup_gpio;

char sleep_log_name[60];
struct file *sleep_log_file = NULL;

#ifdef WRITE_SLEEP_LOG
static int sleep_log_write(void)
{
	char buf[256];
	char *p, *p0;
	int i, j, pos;
	mm_segment_t old_fs;
	p = buf;
	p0 = p;

	if (sleep_log_file == NULL)
		sleep_log_file = filp_open(sleep_log_name, O_RDWR | O_APPEND | O_CREAT,
				0644);
	if (IS_ERR(sleep_log_file)) {
		printk("error occured while opening file %s, exiting...\n",
				sleep_log_name);
		return 0;
	}

	if (sleep_log_count > 1) {
		for (i = 0; i < 2; i++) {
			if (sleep_log_pointer == 0)
				pos = SLEEP_LOG_LENGTH - 2 + i;
			else
				pos = sleep_log_pointer - 2 + i;
			switch (sleep_log_array[pos].log & 0xF) {
			case TRY_TO_SLEEP:
				p += sprintf(p, ">[%ld]%s\n", sleep_log_array[pos].timesec,
						sleep_log_array[pos].time);
				break;
			case LEAVE_FORM_SLEEP:
				p += sprintf(p, "<[%ld]%s(0x%x,0x%x,",
						sleep_log_array[pos].timesec,
						sleep_log_array[pos].time,
						sleep_log_array[pos].maoints[0],
						sleep_log_array[pos].maoints[1]);
				for (j = 0; j < MAX_WAKEUP_IRQ && sleep_log_array[pos].wakeup_irq[j]; j++)
					p += sprintf(p, " %d", sleep_log_array[pos].wakeup_irq[j]);

				if (sleep_log_array[pos].wakeup_gpio)
					p += sprintf(p, ", gpio %d", sleep_log_array[pos].wakeup_gpio);

				p += sprintf(p, ")\n");
				break;
			case FAIL_TO_SLEEP:
				p += sprintf(p, "^[%ld]%s(%d)\n", sleep_log_array[pos].timesec,
						sleep_log_array[pos].time,
						(char) (sleep_log_array[pos].log >> 24));
				break;
			}
		}
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	vfs_write(sleep_log_file, p0, p - p0,
			&sleep_log_file->f_pos);
	set_fs(old_fs);

	if (sleep_log_file != NULL) {
		filp_close(sleep_log_file, NULL);
		sleep_log_file = NULL;
	}
	return 0;
}
#else //WRITE_SLEEP_LOG
static int sleep_log_write(void)
{
	return 0;
}
#endif //WRITE_SLEEP_LOG

static int save_irq_wakeup_internal(int irq)
{
	int i;
	int ret;

	ret = 0;
	if (irq_wakeup_saved < MAX_WAKEUP_IRQ) {
		for (i = 0; i < irq_wakeup_saved; i++) {
			if (irq == irq_wakeup_irq[i])
				break;
		}
		if (i == irq_wakeup_saved)
			ret = irq_wakeup_irq[irq_wakeup_saved++] = irq;
	}
	return ret;
}

int save_irq_wakeup_gpio(int irq, int gpio)
{
	struct irq_desc *desc;
	int ret;

	ret = 0;
	if (debug_mask & DEBUG_WAKEUP_IRQ) {
		desc = irq_to_desc(irq);
		if (desc != NULL) {
			//if (irqd_is_wakeup_set(&desc->irq_data)) {
				ret = save_irq_wakeup_internal(irq);
				if (ret) {
					if (gpio != 0 && irq_wakeup_gpio == 0) {
						irq_wakeup_gpio = gpio;
						irq_wakeup_saved = MAX_WAKEUP_IRQ;
					}
#ifdef CONFIG_KALLSYMS
					printk("%s(), irq=%d, gpio=%d, %s, handler=(%pS)\n", __func__, irq, gpio, 
						desc->action && desc->action->name ? desc->action->name : "",
						desc->action ? (void *)desc->action->handler : 0);
#else
					printk("%s(), irq=%d, gpio=%d, %s, handler=0x%08x\n", __func__, irq, gpio, 
						desc->action && desc->action->name ? desc->action->name : "",
						desc->action ? (unsigned int)desc->action->handler : 0);
#endif
				}
//			}//if (irqd_is_wakeup_set(&desc->irq_data)) {
		}
	}

	return ret;
}

static void clear_irq_wakeup_saved(void)
{
	if (debug_mask & DEBUG_WAKEUP_IRQ) {
		memset(irq_wakeup_irq, 0, sizeof(irq_wakeup_irq));
		irq_wakeup_gpio = 0;
		irq_wakeup_saved = 0;
	}
}

static void set_irq_wakeup_saved(void)
{
	if (debug_mask & DEBUG_WAKEUP_IRQ)
		irq_wakeup_saved = MAX_WAKEUP_IRQ;
}

void log_suspend_enter(void)
{
	extern void smem_set_reserved(int index, int data);
	struct timespec ts_;
	struct rtc_time tm_;

	//Turn on/off the share memory flag to inform RPM to record spm logs
	//smem_set_reserved(6, debug_mask & DEBUG_WAKEUP_IRQ ? 1 : 0);
//	smem_set_reserved(6, debug_mask);

	if (debug_mask & DEBUG_SLEEP_LOG) {
		printk("%s(), APPS try to ENTER sleep mode>>>\n", __func__);

		getnstimeofday(&ts_);
		rtc_time_to_tm(ts_.tv_sec + 8 * 3600, &tm_);

		sprintf(sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].time,
				"%d-%02d-%02d %02d:%02d:%02d", tm_.tm_year + 1900, tm_.tm_mon + 1,
				tm_.tm_mday, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

		if (strlen(sleep_log_name) < 1) {
			sprintf(sleep_log_name,
					"/data/local/log/aplog/sleeplog%d%02d%02d_%02d%02d%02d.txt",
					tm_.tm_year + 1900, tm_.tm_mon + 1, tm_.tm_mday, tm_.tm_hour,
					tm_.tm_min, tm_.tm_sec);
			printk("%s(), sleep_log_name = %s \n", __func__, sleep_log_name);
		}

		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].timesec = ts_.tv_sec;
		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log = TRY_TO_SLEEP;
		sleep_log_pointer++;
		sleep_log_count++;
		if (sleep_log_pointer == SLEEP_LOG_LENGTH)
			sleep_log_pointer = 0;
	}

	clear_irq_wakeup_saved();
	pm_wakeup_fetched = false;
}

void log_suspend_exit(int error)
{
#if 0
	extern int smem_get_reserved(int index);
#else
	extern void msm_rpmstats_get_reverved(u32 reserved[][4]);
	u32 reserved[4][4];
#endif
	struct timespec ts_;
	struct rtc_time tm_;
	uint32_t smem_value;
	int i;

	if (debug_mask & DEBUG_SLEEP_LOG) {
		getnstimeofday(&ts_);
		rtc_time_to_tm(ts_.tv_sec + 8 * 3600, &tm_);
		sprintf(sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].time,
				"%d-%02d-%02d %02d:%02d:%02d", tm_.tm_year + 1900, tm_.tm_mon + 1,
				tm_.tm_mday, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].timesec = ts_.tv_sec;

		if (error == 0) {
#if 0
			rpm_smem_state_data.wakeup_ints[0] = smem_get_reserved(0);
			rpm_smem_state_data.wakeup_ints[1] = smem_get_reserved(1);
#elif 0
			if (debug_mask & DEBUG_RPM_SPM_LOG) {
				for(i = 0; i <= 5; i++) {
					smem_value = ((uint32_t)smem_get_reserved(i)) & 0xffff;
					if(smem_value > 0)
						printk("rpm: %s[%d] = %d\n", "spm_active" , i, smem_value);
				}
			}
			if (debug_mask & DEBUG_RPM_CXO_LOG) {
				for(i = 0; i <= 5; i++) {
					smem_value = ((uint32_t)smem_get_reserved(i)) >> 16;
					if(smem_value > 0)
						printk("rpm: %s[%d] = %d\n", "cxo_voter" , i, smem_value);
				}
			}
#else
			printk("%s, debug_mask=%x\n", __func__, debug_mask);
			if (debug_mask & (DEBUG_RPM_SPM_LOG | DEBUG_RPM_CXO_LOG)) {
				memset(reserved, 0, sizeof(reserved));
				msm_rpmstats_get_reverved(reserved);
#if 1
				for(i = 0; i < 3; i++)
					printk("reserved[0][%d]=0x%08x\n", i, reserved[0][i]);
				for(i = 0; i < 3; i++)
					printk("reserved[1][%d]=0x%08x\n", i, reserved[1][i]);
#endif
			}

			if (debug_mask & DEBUG_RPM_SPM_LOG) {
				for(i = 0; i <= 5; i++) {
					smem_value = (reserved[1][i/2] >> (16 * (i % 2))) & 0xffff;
					if(smem_value > 0)
						printk("rpm: %s[%d] = %d\n", "spm_active" , i, smem_value);
				}
			}
			if (debug_mask & DEBUG_RPM_CXO_LOG) {
				for(i = 0; i <= 5; i++) {
					smem_value = (reserved[0][i/2] >> (16 * (i % 2))) & 0xffff;
					if(smem_value > 0)
						printk("rpm: %s[%d] = %d\n", "cxo_voter" , i, smem_value);
				}
			}
#endif

			printk("%s(), APPS Exit from sleep<<<: wakeup ints=0x%x, 0x%x\n", __func__ ,
					rpm_smem_state_data.wakeup_ints[0],
					rpm_smem_state_data.wakeup_ints[1]);

			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log =
					LEAVE_FORM_SLEEP;
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].maoints[0] =
					rpm_smem_state_data.wakeup_ints[0];
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].maoints[1] =
					rpm_smem_state_data.wakeup_ints[1];
			for (i = 0; i < (irq_wakeup_gpio == 0 ? irq_wakeup_saved : 1); i++)
				sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].wakeup_irq[i] =
					irq_wakeup_irq[i];
			for (; i < MAX_WAKEUP_IRQ; i++)
				sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].wakeup_irq[i] = 0;
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].wakeup_gpio =
					irq_wakeup_gpio;
		} else {
			printk("%s(), APPS FAIL to enter sleep^^^\n", __func__);

			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log =
					FAIL_TO_SLEEP | (error << 24);
		}

		sleep_log_pointer++;
		sleep_log_count++;

		if (sleep_log_pointer == SLEEP_LOG_LENGTH)
			sleep_log_pointer = 0;

		if (debug_mask & DEBUG_WRITE_LOG) {
			enter_times++;
			if (enter_times < 5000)
				sleep_log_write();
		}
	}

	set_irq_wakeup_saved();
}
#else //SLEEP_LOG
void log_suspend_enter(void)
{
	clear_irq_wakeup_saved();
	pm_wakeup_fetched = false;
}

void log_suspend_exit(int error)
{
	set_irq_wakeup_saved();
}
#endif //SLEEP_LOG

static const struct of_device_id sysfs_private_tlmm_dt_match[] = {
	{ .compatible = "qcom,msmtitanium-pinctrl", },
	{ },
};

static int sysfs_private_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;
	int size = 0;
	int ret = 0;
	struct regulator	*vdd_l16;
	u32 voltage_low, voltage_high;
	//int i = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		return -ENOENT;
	}
/*
*	Request same resource could cause ioremap failed and 
*	return -EBUSY.Therefore we remove it.
*/ 
/*
	tlmm_reg_base =  devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(tlmm_reg_base))
		return PTR_ERR(tlmm_reg_base);
*/
	of_get_property(node, "lenovo,tz_gpio", &size);
	//printk("%s(), %d, size=%d\n", __func__, __LINE__, size);
	if (size) {
		tz_pin_num = size / sizeof(u32);
		tz_config = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
		ret = of_property_read_u32_array(node,
				"lenovo,tz_gpio", tz_config,
				tz_pin_num);
/*
		for(i = 0; i < tz_pin_num; i++)
		{
			pr_debug("%s(), %d, tz_config[%d]=%d\n", __func__, __LINE__, i, tz_config[i]);
		}
*/
	}
	printk("%s(), %d, TLMM_BASE=%lx\n", __func__, __LINE__, (unsigned long int)(void *)tlmm_reg_base);

	// enable l16
	if (of_find_property(node, "vdd-supply", NULL)) {
		vdd_l16 = devm_regulator_get(&pdev->dev, "vdd");
		if (IS_ERR(vdd_l16))
			return PTR_ERR(vdd_l16);
	}
	
	ret = of_property_read_u32(node, "vdd-low-microvolt",
				   &voltage_low);
	if (ret) {
		dev_err(&pdev->dev, "no vdd-low-microvolt property set\n");
		return ret;
	}
	ret = of_property_read_u32(node, "vdd-low-microvolt",
				   &voltage_high);
	if (ret) {
		dev_err(&pdev->dev, "no vdd-low-microvolt property set\n");
		return ret;
	}
	if (regulator_count_voltages(vdd_l16) <= 0)
		return 0;

	printk("%s(), %d, voltage_low=%u, voltage_high=%u\n", __func__, __LINE__, voltage_low, voltage_high);
	
	regulator_set_voltage(vdd_l16, voltage_low, voltage_high);

	ret = regulator_enable(vdd_l16);
	pr_err("enable l16 regulator rc=%d\n", ret);
	if (ret) {
		pr_err("failed to enable l16 regulator rc=%d\n", ret);
		return ret;
	}
	
	return 0;
}

static const struct of_device_id msmtitanium_pinctrl_of_match[] = {
	{ .compatible = "qcom,sysfs_private", },
	{ },
};

static struct platform_driver sysfs_private_drv = {
	.probe		= sysfs_private_probe,
	.driver = {
		.name	= "sysfs_private",
		.owner	= THIS_MODULE,
		.of_match_table = msmtitanium_pinctrl_of_match,
	},
};


MODULE_DEVICE_TABLE(of, msm_tlmm_dt_match);



static int __init sysfs_private_init(void)
{
	int result;

	printk("%s(), %d\n", __func__, __LINE__);

	sysfs_private_kobj = kobject_create_and_add("private", NULL);
	if (!sysfs_private_kobj)
		return -ENOMEM;

	result = sysfs_create_group(sysfs_private_kobj, &private_attr_group);
	printk("%s(), %d, result=%d\n", __func__, __LINE__, result);

#ifdef SLEEP_LOG
	strcpy (sleep_log_name, "");
	sleep_log_pointer = 0;
	sleep_log_count = 0;
	enter_times = 0;
#endif

	platform_driver_register(&sysfs_private_drv);
	//tlmm_reg_base = ioremap((resource_size_t )0x1000000, (unsigned long)0x300000);

	return 0;//platform_driver_register(&sysfs_private_drv);
}

static void __exit sysfs_private_exit(void)
{
	printk("%s(), %d\n", __func__, __LINE__);
	sysfs_remove_group(sysfs_private_kobj, &private_attr_group);

	kobject_put(sysfs_private_kobj);

	platform_driver_unregister(&sysfs_private_drv);
}

module_init(sysfs_private_init);
module_exit(sysfs_private_exit);
