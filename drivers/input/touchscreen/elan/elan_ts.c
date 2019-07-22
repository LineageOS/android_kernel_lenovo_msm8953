#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/wakelock.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif

#include <linux/miscdevice.h>

#include "elan_ts.h"

/********竖屏关闭，横屏开启********/
//#define SWAP_X_Y_RESOLUTION

/********X 轴反转********/
//#define ROTATE_X

/********Y 轴反转********/
//#define ROTATE_Y

static struct workqueue_struct *init_elan_ic_wq = NULL;
static struct delayed_work init_work;
static unsigned long delay = 2*HZ;
extern int compare_tp_id;

//#define ELAN_ESD_CHECK
#ifdef ELAN_ESD_CHECK
    static struct workqueue_struct *esd_wq = NULL;
    static struct delayed_work esd_work;
    static atomic_t elan_cmd_response = ATOMIC_INIT(0);
#endif
/*********************************platform data********************************************/
//must be init first
static unsigned int gpio_3v3 = 0;
static unsigned int gpio_1v8 = 0;
static unsigned int reset_gpio  = 0;
static unsigned int intr_gpio       = 0;
static int elan_irq                 = 0;

int elan_flag = 0;	//lct--lyh--add for tp firmware update

static const struct i2c_device_id elan_ts_id[] = {
    {ELAN_TS_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, elan_ts_id);

#ifdef I2C_BOARD_REGISTER
static struct i2c_board_info __initdata elan_i2c_info={
    I2C_BOARD_INFO(ELAN_TS_NAME, (ELAN_7BITS_ADDR))
};

static const unsigned short normal_i2c[2] = {
    ELAN_7BITS_ADDR,
    I2C_CLIENT_END
};
#endif

/**********************************elan struct*********************************************/

struct elan_ts_i2c_platform_data {
    const char *name;
    struct regulator * vcc_i2c;
	#if 0
    struct regulator * pm8917_l10_switch;
    struct regulator * pm8917_l17_switch;
	#endif
    u32 irqflags;
    u32 intr_gpio;
    u32 intr_gpio_flags;
    u32 rst_gpio;
    u32 rst_gpio_flags;
    u32 gpio_3v3;
    u32 gpio_3v3_flags;
    u32 gpio_1v8;
    u32 gpio_1v8_flags;
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 panel_minx;
    u32 panel_miny;
    u32 panel_maxx;
    u32 panel_maxy;
    int (*init_platform_hw)(void);
    int (*power_init) (bool);
    int (*power_on) (bool);
};

struct elan_ts_data{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct  work;
#ifdef CONFIG_HAS_EARLYSUSPEND
//used for early_suspend
    struct early_suspend early_suspend;
#endif
    
//Firmware Information
    int fw_ver;
    int fw_id;
    int fw_bcd;
    int x_resolution;
    int y_resolution;
    int recover;//for iap mod
//for suspend or resum lock
    int power_lock;
    int circuit_ver;
//for button state
    int button_state;
//For Firmare Update 
    struct miscdevice firmware;
    struct proc_dir_entry *p;
};

/************************************global elan data*************************************/
static struct elan_ts_data *private_ts;

/*********************************global elan function*************************************/
static int __hello_packet_handler(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ts_resume(struct i2c_client *client);
static int elan_ts_rough_calibrate(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ts_early_suspend(struct early_suspend *h);
static void elan_ts_late_resume(struct early_suspend *h);
#endif

#if defined IAP_PORTION
static void check_update_flage(struct elan_ts_data *ts);
#endif

/************************************** function list**************************************/


static void elan_reset(void)
{
    elan_info("%s enter\n", __func__);
    gpio_set_value(reset_gpio, 1);
    msleep(10);
    gpio_set_value(reset_gpio, 0);
    msleep(10);
    gpio_set_value(reset_gpio, 1);
    msleep(10);
}

static void elan_switch_irq(int on)
{
    elan_info("%s enter, irq = %d, on = %d\n", __func__, elan_irq, on);
    if(on){
        enable_irq(elan_irq);
    }
    else{
        disable_irq(elan_irq);
    }
}

static int elan_ts_poll(void)
{
    int status = 0, retry = 20;

    do {
        status = gpio_get_value(intr_gpio);
        elan_info("%s: status = %d\n", __func__, status);
        if(status == 0)
		break;
        retry--;
        msleep(40);
    } while (status == 1 && retry > 0);

    elan_info("%s: poll interrupt status %s\n", __func__, status == 1 ? "high" : "low");
    
    return status == 0 ? 0 : -ETIMEDOUT;
}

static int elan_i2c_send(const struct i2c_client *client, const char *buf, int count)
{
    int ret;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg;

    msg.addr = client->addr;
    msg.flags = client->flags & I2C_M_TEN;
    msg.len = count;
    msg.buf = (char *)buf;

    ret = i2c_transfer(adap, &msg, 1);

    /*
     * If everything went ok (i.e. 1 msg transmitted), return #bytes
     * transmitted, else error code.
     */
    return (ret == 1) ? count : ret;        
}

static int elan_i2c_recv(const struct i2c_client *client, char *buf, int count)
{
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg;
    int ret;

    msg.addr = client->addr;
    msg.flags = client->flags & I2C_M_TEN;
    msg.flags |= I2C_M_RD;
    msg.len = count;
    msg.buf = buf;

    ret = i2c_transfer(adap, &msg, 1);

    /*
     * If everything went ok (i.e. 1 msg received), return #bytes received,
     * else error code.
     */
    return (ret == 1) ? count : ret;
}

static int elan_ts_send_cmd(struct i2c_client *client, uint8_t *cmd, size_t size)
{
    elan_info("dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);
    if (elan_i2c_send(client, cmd, size) != size) {
        printk("[elan error]%s: elan_ts_send_cmd failed\n", __func__);
        return -EINVAL;
    }
    else{
        elan_info("[elan] elan_ts_send_cmd ok");
    }
    return size;
}

static int elan_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t size)
{
    int rc;

    if (buf == NULL){
        return -EINVAL;
    }
    if (elan_ts_send_cmd(client, cmd, size) != size){
        return -EINVAL;
    }
    msleep(2);
    
    rc = elan_ts_poll();
    if (rc < 0){
        return -EINVAL;
    }
    else {
        rc = elan_i2c_recv(client, buf, size);
        elan_info("%s: respone packet %2x:%2X:%2x:%2x\n", __func__, buf[0], buf[1], buf[2], buf[3]); 
        if(buf[0] != CMD_S_PKT || rc != size){ 
            printk("[elan error]%s: cmd respone error\n", __func__);
            return -EINVAL;
        }
    }
    
    return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
    struct elan_ts_data *ts = i2c_get_clientdata(client);
    int rc;
    uint8_t buf_recv[8] = { 0 };
    uint8_t cmd[4] = {0x53, 0x00, 0x00, 0x01};
    
    rc = elan_ts_poll();
    if(rc != 0){
        printk("[elan error] %s: Int poll 55 55 55 55 failed!\n", __func__);
    }
    
    rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
    if(rc != sizeof(buf_recv)){
        printk("[elan error] __hello_packet_handler recv error, retry\n");
    }
    elan_info("%s: hello packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
    
    if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80){
        elan_info("%s: boot code packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
        ts->recover = 0x80;
    }
    else if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x55 && buf_recv[3]==0x55){
        elan_info("__hello_packet_handler recv ok\n");
        ts->recover = 0x0;
    }
    else{
        if(rc != sizeof(buf_recv)){
            rc = elan_i2c_send(client, cmd, sizeof(cmd));
            if(rc != sizeof(cmd)){
                msleep(5);
                rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
            }
        }
        ts->recover = rc;
    }
    
    return ts->recover;
}

static int __fw_packet_handler(struct i2c_client *client)
{
    struct elan_ts_data *ts = i2c_get_clientdata(client);
    int rc;
    int major, minor;
    uint8_t cmd[]           = {0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
    
    uint8_t cmd_id[]        = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
    uint8_t cmd_bc[]        = {0x53, 0x10, 0x00, 0x01};/* Get BootCode Version*/
#if 0
    int x, y;
    uint8_t cmd_getinfo[] = {0x5B, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t adcinfo_buf[17]={0};
#else
    uint8_t cmd_x[]         = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
    uint8_t cmd_y[]         = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
#endif  
    uint8_t buf_recv[4]     = {0};
// Firmware version
    rc = elan_ts_get_data(client, cmd, buf_recv, 4);
    if (rc < 0)
        return rc;
    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
    ts->fw_ver = major << 8 | minor;
    
// Firmware ID
    rc = elan_ts_get_data(client, cmd_id, buf_recv, 4);
    if (rc < 0)
        return rc;
    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
    ts->fw_id = major << 8 | minor;
#if 1
    // X Resolution
    rc = elan_ts_get_data(client, cmd_x, buf_recv, 4);
    if (rc < 0)
        return rc;
    minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
#ifdef SWAP_X_Y_RESOLUTION
    ts->y_resolution = minor;
#else   
    ts->x_resolution = minor;
#endif

    // Y Resolution 
    rc = elan_ts_get_data(client, cmd_y, buf_recv, 4);
    if (rc < 0)
        return rc;
    minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
#ifdef SWAP_X_Y_RESOLUTION
    ts->x_resolution = minor;
#else   
    ts->y_resolution = minor;
#endif
    
#else
    
    elan_i2c_send(client, cmd_getinfo, sizeof(cmd_getinfo));
    msleep(10);
    elan_i2c_recv(client, adcinfo_buf, 17);
    x  = adcinfo_buf[2]+adcinfo_buf[6]+adcinfo_buf[10]+adcinfo_buf[14];
    y  = adcinfo_buf[3]+adcinfo_buf[7]+adcinfo_buf[11]+adcinfo_buf[15];

    printk( "[elan] %s: x= %d, y=%d\n",__func__,x,y);

    ts->x_resolution=(x-1)*64;
    ts->y_resolution=(y-1)*64;
#endif
    
// Firmware BC
    rc = elan_ts_get_data(client, cmd_bc, buf_recv, 4);
    if (rc < 0)
        return rc;
    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
    ts->fw_bcd = major << 8 | minor;    
    
    elan_info( "%s: firmware version: 0x%4.4x\n",__func__, ts->fw_ver);
    elan_info( "%s: firmware ID: 0x%4.4x\n",__func__, ts->fw_id);
    elan_info( "%s: firmware BC: 0x%4.4x\n",__func__, ts->fw_bcd);
    elan_info( "%s: x resolution: %d, y resolution: %d\n",__func__, ts->x_resolution, ts->y_resolution);
    
    return 0;
}


//lct--lyh--add tp info start
static char dev_info[100];

static void lct_tp_info(char *module)
{
    char info[50] = {0};
    static bool initialized = false;
    if(module && initialized)
	return;

   snprintf(info, sizeof(info), "[fw]0x%4.4x,[ic]EKTH3668", private_ts->fw_ver); 
   snprintf(dev_info, sizeof(dev_info), "[Vendor]%s,[fw]0x%4.4x,[ic]EKTH3668", module, private_ts->fw_ver);
   if(!module)
	update_tp_fm_info(info);
   else {
	init_tp_fm_info(0, info, module);
        initialized = true;
   }
	
}

static void lct_get_tp_info(char *tp_info)
{
	if(tp_info == NULL)
		return;
	sprintf(tp_info,"%s", dev_info);
}
//lct--lyh--add tp info end
static int elan_ts_rough_calibrate(struct i2c_client *client)
{
    uint8_t flash_key[] = {CMD_W_PKT, 0xc0, 0xe1, 0x5a};
    uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};
    
    elan_info("%s: enter\n", __func__);
    dev_info(&client->dev,
             "[elan] dump flash_key : %02x, %02x, %02x, %02x\n",
             flash_key[0], flash_key[1], flash_key[2], flash_key[3]);

    if((i2c_master_send(client, flash_key, sizeof(flash_key))) != sizeof(flash_key))
    {
        dev_err(&client->dev,
                "[elan] %s: i2c_master_send failed\n", __func__);
        return -EINVAL;
    }
    
    dev_info(&client->dev,
             "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
             cmd[0], cmd[1], cmd[2], cmd[3]);

    if((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd))
    {
        dev_err(&client->dev,
                "[elan] %s: i2c_master_send failed\n", __func__);
        return -EINVAL;
    }

    return 0;
}

int WritePage(struct i2c_client *client, uint8_t * szPage, int byte, int which)
{
    int len = 0;
    
    len = elan_i2c_send(client, szPage,  byte);
    if (len != byte) {
        printk("[elan] ERROR: write the %d th page error, write error. len=%d\n", which, len);
        return -1;
    }
    
    return 0;
}

/*every page write to recv 2 bytes ack */
int GetAckData(struct i2c_client *client, uint8_t *ack_buf)
{
    int len = 0;

    
    len=elan_i2c_recv(client, ack_buf, 2);

    if (len != 2) {
        printk("[elan] ERROR: GetAckData. len=%d\r\n", len);
        return -1;
    }
    
    if (ack_buf[0] == 0xaa && ack_buf[1] == 0xaa) {
        return ACK_OK;
    }
    else if (ack_buf[0] == 0x55 && ack_buf[1] == 0x55){
        return ACK_REWRITE;
    }
    else{
        return ACK_Fail;
    }
    return 0;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(struct i2c_client *client)
{
    char buff[4] = {0};
    int rc = 0;
    
    rc = elan_i2c_recv(client, buff, 4);
    if (rc != 4) {
        printk("[elan] ERROR: CheckIapMode. len=%d\r\n", rc);
        return -1;
    }
    else
        elan_info("Mode= 0x%x 0x%x 0x%x 0x%x\r\n", buff[0], buff[1], buff[2], buff[3]);
        
    return 0;   
}

int check_2k_recover_mode(struct i2c_client *client)
{
	uint8_t buf_recv[8] = { 0 };
	int rc = 0;
	int retry_cnt = 0;
	
retry_to_check_mode:	 
	elan_reset();
	rc = elan_ts_poll();
	if(rc != 0){
		printk("[elan error] %s: Int poll 55 55 55 55 failed!\n", __func__);
	}
	
	rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
	if(rc != sizeof(buf_recv)){
		printk("[elan error] __hello_packet_handler recv error, retry\n");
	}
	elan_info("%s: hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	
	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80){
		elan_info("%s: boot code packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
		msleep(5);
		rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
		elan_info("%s: enter mode check %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[0], buf_recv[0], buf_recv[0]);
		msleep(5);
		
		return 0x80;
	}
	else if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x55 && buf_recv[3]==0x55){
		elan_info(" __hello_packet_handler recv ok\n");
		return 0;
	}
	else {
		retry_cnt++;
		if(retry_cnt < 3){
			goto retry_to_check_mode;
		}
	}
	
	return -1;
}


int update_fw_one(struct i2c_client *client)
{
    uint8_t ack_buf[2] = {0};
    struct elan_ts_data *ts = i2c_get_clientdata(client);
    
    int res = 0;
    int iPage = 0;
    
    uint8_t data;

    const int PageSize = 132;
    const int PageNum = sizeof(file_fw_data)/PageSize;
    
    const int PAGERETRY = 10;
    const int IAPRESTART = 5;
    
    int restartCnt = 0; // For IAP_RESTART
    int rewriteCnt = 0;// For IAP_REWRITE
    
    int iap_mod;
    
    uint8_t *szBuff = NULL;
    int curIndex = 0;

#ifdef ELAN_2K_XX   
    uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};    //54 00 12 34
    iap_mod = 2;
#endif

#ifdef ELAN_3K_XX 
    uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};    //45 49 41 50
    iap_mod = 3;    
#endif

#ifdef ELAN_RAM_XX 
    uint8_t isp_cmd[] = {0x22, 0x22, 0x22, 0x22};    //22 22 22 22
    iap_mod = 1;    
#endif

    elan_flag = 1;	//lct--lyh--add for tp firmware update

    elan_switch_irq(0);
    ts->power_lock = 1;
    
    data=ELAN_7BITS_ADDR;
    elan_info( "%s: address data=0x%x iap_mod=%d PageNum = %d\r\n", __func__, data, iap_mod, PageNum);
    
IAP_RESTART:
    //reset tp
    if (iap_mod == 3){
        elan_reset();
        res = elan_ts_send_cmd(client, isp_cmd, sizeof(isp_cmd));
		msleep(5);
		res = CheckIapMode(client);
    }
    else if (iap_mod == 2){
    	res = check_2k_recover_mode(client);
    	if(res != 0x80){
        	res = elan_ts_send_cmd(client, isp_cmd, sizeof(isp_cmd));
        	msleep(5);
        	res = CheckIapMode(client);
        }
   	}
    //Step 3 Send Dummy Byte
    res = elan_i2c_send(client, &data,  sizeof(data));
    if(res!=sizeof(data)){
        elan_info(" dummy error code = %d\n",res);
	elan_flag = 0;		//lct--lyh--add for tp firmware update
	lct_set_ctp_upgrade_status("Upgrade failed!");
        return res;
    }
    else{
        elan_info("send Dummy byte sucess data:%x", data);
    }
    
    msleep(10);
    

    //Step 4 Start IAP
    for( iPage = 1; iPage <= PageNum; iPage++ ) {
            
        szBuff = file_fw_data + curIndex;
        curIndex =  curIndex + PageSize;

PAGE_REWRITE:
        res = WritePage(client, szBuff, PageSize, iPage);
        
        if(iPage==PageNum || iPage==1){
            msleep(300);             
        }
        else{
            msleep(50);              
        }
        
        res = GetAckData(client, ack_buf);
        if (ACK_OK != res) {
            
            msleep(50); 
            elan_info("%d page ack error: ack0:%x ack1:%x\n",  iPage, ack_buf[0], ack_buf[1]);
            
 //           if ( res == ACK_REWRITE ){
                rewriteCnt = rewriteCnt + 1;
                if (rewriteCnt < PAGERETRY){
                    elan_info("---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
                    goto PAGE_REWRITE;
                }
 //           }
            
            restartCnt = restartCnt + 1;
            if (restartCnt == IAPRESTART){
                elan_info("ReStart %d times fails!\n", restartCnt);
                elan_switch_irq(1);
                ts->power_lock = 0;
		elan_flag = 0;		//lct--lyh--add for tp firmware update
		lct_set_ctp_upgrade_status("Upgrade failed!");
                return -1;
            }
            else{
                elan_info("try to ReStart %d times !\n", restartCnt);
                curIndex = 0;
                rewriteCnt=0;
                goto IAP_RESTART;
            }
        }
        else{
            printk(KERN_ERR "[elan]---%d--- page flash ok\n", iPage);
            rewriteCnt=0;
        }
    }

    elan_reset();
    elan_switch_irq(1);
    ts->power_lock = 0;
    
    elan_flag = 0;	//lct--lyh--add for tp firmware update
    elan_info("Update ALL Firmware successfully!\n");
    lct_set_ctp_upgrade_status("Upgrade success!");
    
    return 0;
}


//lct--lyh--add for factory upgrade firmware start 
static int lct_ctp_upgrade_func(void)
{
/*
	int ret;
	ret = update_fw_one(private_ts->client);
	if(ret)
	{
		printk("[lct_ctp] Upgrate failed!");
	}
	return ret;
*/
	return 0;
}
//lct--lyh--add for factory upgrade firmware end

static inline int elan_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
    *x = *y = 0;

    *x = (data[0] & 0xf0);
    *x <<= 4;
    *x |= data[1];

    *y = (data[0] & 0x0f);
    *y <<= 8;
    *y |= data[2];

    return 0;
}

static int elan_ts_setup(struct i2c_client *client)
{
    int rc = 0;
    elan_reset();
    msleep(200);
    
    rc = __hello_packet_handler(client);
    if (rc < 0){
        printk("[elan error] %s, hello_packet_handler fail, rc = %d\n", __func__, rc);
    }
    return rc;
}


/*
static int elan_ts_set_power_state(struct i2c_client *client, int state)
{
    uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
    int size = sizeof(cmd);
    
    cmd[1] |= (state << 3);
    if (elan_ts_send_cmd(client, cmd, size) != size){
        return -EINVAL;
    }   

    return 0;
}
*/


static void elan_ts_touch_down(struct elan_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
    if (ts->input_dev == NULL) {
        elan_info("input dev is null\n");
        return;    
    }
    
#ifdef ROTATE_X    
    x = ts->x_resolution-x;
#endif
#ifdef ROTATE_Y 
    y = ts->y_resolution-y;
#endif
        
#ifdef ELAN_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
    input_mt_sync(ts->input_dev);
#endif

    elan_info("Touch ID:%d, X:%d, Y:%d, W:%d down", id, x, y, w);
}

static void elan_ts_touch_up(struct elan_ts_data* ts,s32 id,s32 x,s32 y)
{   
    if (ts->input_dev == NULL) {
        elan_info("input dev is null\n");
        return;    
    }
    
#ifdef ELAN_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
    input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
    elan_info("Touch id[%2d] release!", id);
#else
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, ~id);
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    //input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
    input_mt_sync(ts->input_dev);
    elan_info("Touch all release!");
#endif
}

static void elan_ts_report_key(struct elan_ts_data *ts, uint8_t button_data)
{   
    static unsigned int key_value = 0;
    static unsigned int x = 0, y = 0;
    
    switch (button_data) {
        case ELAN_KEY_BACK:
            key_value = KEY_BACK;
            input_report_key(ts->input_dev, key_value, 1);
            break;
        case ELAN_KEY_HOME:
            key_value = KEY_HOME;
            input_report_key(ts->input_dev, key_value, 1);
            break;
        case ELAN_KEY_MENU:
            key_value = KEY_MENU;
            input_report_key(ts->input_dev, key_value, 1);
            break;
        default:
            if(key_value != 0){
                input_report_key(ts->input_dev, key_value, 0);
                key_value = 0;
            }
            else{
                elan_ts_touch_up(ts, 0, x, y);
            }   
            break;
    }
    
}

#if defined ELAN_RAM_XX
static void elan_ts_iap_ram_continue(struct i2c_client *client)
{
    uint8_t cmd[] = { 0x33, 0x33, 0x33, 0x33 };
    int size = sizeof(cmd);

    elan_ts_send_cmd(client, cmd, size);
}
#endif

static void elan_ts_handler_event(struct elan_ts_data *ts, uint8_t *buf)
{
    if(buf[0] == 0x55){
        if(buf[2] == 0x55){
            ts->recover = 0;
        }
        else if(buf[2] == 0x80){
            ts->recover = 0x80;
        }
    }
    
#ifdef ELAN_ESD_CHECK   
    else if(buf[0] == 0x78){
        atomic_set(&elan_cmd_response, 1);
    }
#endif  
}

#ifdef ELAN_IAP_DEV
int elan_iap_open(struct inode *inode, struct file *filp)
{ 
    elan_info("%s enter", __func__);
    if (private_ts == NULL){
        printk("[elan error]private_ts is NULL~~~");
    }   
    return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{    
    elan_info("%s enter", __func__);
    return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{  
    int ret;
    char *tmp;
    struct i2c_client *client = private_ts->client;
    
    elan_info("%s enter", __func__);
    if (count > 8192){
        count = 8192;
    }
    tmp = kmalloc(count, GFP_KERNEL);
    if (tmp == NULL){
        return -ENOMEM;
    }
    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
    
    ret = elan_i2c_send(client, tmp, count);
    if (ret != count){ 
        printk("[elan error]elan elan_i2c_send fail, ret=%d \n", ret);
    }
    kfree(tmp);
    
    return ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{    
    char *tmp;
    int ret;  
    long rc;
    
    struct i2c_client *client = private_ts->client;
    
    elan_info("%s enter", __func__);
    
    if (count > 8192){
        count = 8192;
    }
    tmp = kmalloc(count, GFP_KERNEL);
    if (tmp == NULL){
        return -ENOMEM;
    }
    ret = elan_i2c_recv(client, tmp, count);
    if (ret != count){ 
        printk("[elan error]elan elan_i2c_recv fail, ret=%d \n", ret);
    }
    if (ret == count){
        rc = copy_to_user(buff, tmp, count);
    }
    kfree(tmp);
    return ret;
}

static long elan_iap_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
    int __user *ip = (int __user *)arg;
    
    elan_info("%s enter, cmd value %x\n",__func__, cmd);

    switch (cmd) {        
        case IOCTL_I2C_SLAVE:
            elan_info("pre addr is %X\n",  private_ts->client->addr); 
            private_ts->client->addr = (int __user)arg;
            elan_info("new addr is %X\n",  private_ts->client->addr); 
            break;
        case IOCTL_RESET:
            elan_reset();
            break;
        case IOCTL_IAP_MODE_LOCK:
            if(private_ts->power_lock == 0){
                private_ts->power_lock = 1;
                elan_switch_irq(0);
            }
            break;
        case IOCTL_IAP_MODE_UNLOCK:
            if(private_ts->power_lock == 1){
                private_ts->power_lock = 0;
                elan_switch_irq(1);
            }
            break;
        case IOCTL_CHECK_RECOVERY_MODE:
            return private_ts->recover;;
            break;
	case IOCTL_ROUGH_CALIBRATE:
            return elan_ts_rough_calibrate(private_ts->client); 
        case IOCTL_I2C_INT:
            put_user(gpio_get_value(intr_gpio), ip);
            break;
        default:            
            break;   
    }       
    return 0;
}

struct file_operations elan_touch_fops = {    
    .open = elan_iap_open,    
    .write = elan_iap_write,    
    .read = elan_iap_read,    
    .release =  elan_iap_release,    
    .unlocked_ioctl = elan_iap_ioctl, 
    .compat_ioctl = elan_iap_ioctl, 
 };

#endif

#ifdef ELAN_ESD_CHECK
static void elan_touch_esd_func(struct work_struct *work)
{   
    int res;
    struct i2c_client *client = private_ts->client; 

    elan_info("esd %s: enter.......", __FUNCTION__);
    
    if(private_ts->power_lock == 1){
        goto elan_esd_check_out;
    }
    
    if(atomic_read(&elan_cmd_response) == 0){
        printk("[elan esd] %s:  response 0x78 failed \n", __func__);
    }
    else{
        elan_info("esd %s: response ok!!!", __func__);
        goto elan_esd_check_out;
    }
    
    elan_reset();
elan_esd_check_out: 
    
    atomic_set(&elan_cmd_response, 0);
    queue_delayed_work(esd_wq, &esd_work, delay);
    elan_info("[elan esd] %s: out.......", __FUNCTION__);   
    return;
}
#endif  


#ifdef SYS_ATTR_FILE
static int i2c_read_len = 0;
static int i2c_retrun_status = 0;

static ssize_t elan_ioctl_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    
    ret = gpio_get_value(intr_gpio);
    return ret;
}

static ssize_t elan_ioctl_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t size)
{
    int cmd = 0, value = 0;
    
    if(sscanf(buf, "%d:%d", &cmd, &value) != 2){
        printk("[elan error] parameter error\n");
    }
    
    if(cmd == 1){
        elan_reset();
    }
    else if(cmd == 2){
        
        if(private_ts->power_lock == 0 && value == 0){
            private_ts->power_lock = 1;
            elan_switch_irq(0);
        }
        else if(private_ts->power_lock == 1 && value == 1){
            private_ts->power_lock = 0;
            elan_switch_irq(1); 
        }
    }
    else if(cmd == 3){
        i2c_read_len = value;
    }
    else if(cmd == 4){
        elan_info("pre addr is 0x%X\n",  private_ts->client->addr); 
        private_ts->client->addr = value;
        elan_info("new addr is 0x%X\n",  private_ts->client->addr); 
    }
    
    return size;;
}
static DEVICE_ATTR(ioctl, 0666, elan_ioctl_show, elan_ioctl_store);

static ssize_t elani2c_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int i = 0;
    char data[512];
    
    if(i2c_read_len > 512){
        i2c_read_len = 512; 
    }
    
    i2c_retrun_status = elan_i2c_recv(private_ts->client, data, i2c_read_len);
    for(i=0; i<i2c_read_len; i++)
        sprintf(buf+i*2, "%02x", data[i]);
    
    ret = strlen(buf);
    return ret;
}

static ssize_t elani2c_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t size)
{
    i2c_retrun_status = elan_i2c_send(private_ts->client, buf, size);
    return size;
}
static DEVICE_ATTR(elani2c, 0666, elani2c_show, elani2c_store);

static ssize_t elan_i2c_return_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%08x\n", i2c_retrun_status);
}
static DEVICE_ATTR(i2c_return_status, 0444, elan_i2c_return_status_show, NULL);

static ssize_t elan_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
#ifdef PRINT_INT_INFO   
    debug_flage = !debug_flage;
    if(debug_flage)
        elan_info("elan debug switch open\n");
    else
        elan_info("elan debug switch close\n");
#endif  
    return ret;
}
static DEVICE_ATTR(debug, 0444, elan_debug_show, NULL);

static ssize_t elan_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    struct elan_ts_data *ts = private_ts;
    elan_switch_irq(0);
    __fw_packet_handler(ts->client);
    elan_switch_irq(1);
    sprintf(buf, "elan fw ver:%X,id:%X,x:%d,y:%d\n", ts->fw_ver, ts->fw_id, ts->x_resolution, ts->y_resolution);
    ret = strlen(buf) + 1;
    return ret;
}
static DEVICE_ATTR(info, 0444, elan_info_show, NULL);

static ssize_t send_cmd_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t size)
{
    char cmd[4] = {0};
    
    if (sscanf(buf, "%02x:%02x:%02x:%02x\n", (int *)&cmd[0], (int *)&cmd[1], (int *)&cmd[2], (int *)&cmd[3]) != 4){
        printk("[elan error]elan cmd format error\n");
        return -EINVAL;
    }
    elan_ts_send_cmd(private_ts->client, cmd, 4);
    return size;
}
static DEVICE_ATTR(send_cmd, 0222, NULL, send_cmd_store);

static struct attribute *sysfs_attrs_ctrl[] = {
    &dev_attr_ioctl.attr,
    &dev_attr_elani2c.attr,
    &dev_attr_i2c_return_status.attr,
    &dev_attr_debug.attr,
    &dev_attr_info.attr,
    &dev_attr_send_cmd.attr,
    NULL
};
static struct attribute_group elan_attribute_group[] = {
    {.attrs = sysfs_attrs_ctrl },
};
#endif

static void elan_touch_node_init(void)
{
#ifdef SYS_ATTR_FILE  
    int ret ;
#endif
    struct elan_ts_data *ts = private_ts;
    
#ifdef SYS_ATTR_FILE        
    android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
    if (android_touch_kobj == NULL) {
        printk(KERN_ERR "[elan]%s: kobject_create_and_add failed\n", __func__);
        return;
    }
    
    ret = sysfs_create_group(android_touch_kobj, elan_attribute_group);
    if (ret < 0) {
        printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
    }
#endif
    
#ifdef ELAN_IAP_DEV 
    ts->firmware.minor = MISC_DYNAMIC_MINOR;
    ts->firmware.name = "elan-iap";
    ts->firmware.fops = &elan_touch_fops;
    ts->firmware.mode = S_IFREG|S_IRWXUGO; 

    if (misc_register(&ts->firmware) < 0)
        elan_info("misc_register failed!!\n");
    
    ts->p = proc_create("elan-iap", 0666, NULL, &elan_touch_fops);
    if (ts->p == NULL){
        printk("[elan error] proc_create failed!!\n");        
    }
    else{
        elan_info("proc_create ok!!\n");        
    }
#endif
    
    return;
}

#ifdef SYS_ATTR_FILE
static void elan_touch_node_deinit(void)
{

    if(android_touch_kobj){
        sysfs_remove_group(android_touch_kobj, elan_attribute_group);
        kobject_put(android_touch_kobj);
    }   


#ifdef ELAN_IAP_DEV 
    misc_deregister(&private_ts->firmware);
    remove_proc_entry("elan-iap", NULL);
#endif
    
}
#endif

static int elan_ts_recv_data(struct elan_ts_data *ts, uint8_t *buf)
{
    int rc;
#ifdef PRINT_INT_INFO
    int i=0;
#endif  
    rc = elan_i2c_recv(ts->client, buf, PACKET_SIZE);
    if(PACKET_SIZE != rc){
        printk("[elan error] elan_ts_recv_data\n");
        return -1;
    }
#ifdef PRINT_INT_INFO
    for(i = 0; i < (PACKET_SIZE+7)/8; i++){
        elan_info("%02x %02x %02x %02x %02x %02x %02x %02x", buf[i*8+0],buf[i*8+1],buf[i*8+2],buf[i*8+3],buf[i*8+4],buf[i*8+5],buf[i*8+6],buf[i*8+7]);
    }
#endif
    
    if(FINGERS_PKT != buf[0]){
#ifndef PRINT_INT_INFO      
        elan_info("other event packet:%02x %02x %02x %02x %02x %02x %02x %02x\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
#endif
        elan_ts_handler_event(ts, buf);
        return -1;
    }
    
#ifdef ELAN_ESD_CHECK
    atomic_set(&elan_cmd_response, 1);
#endif
    
    return 0;
}

static void elan_ts_report_data(struct elan_ts_data *ts, uint8_t *buf)
{
    uint16_t fbits=0;
#ifdef ELAN_ICS_SLOT_REPORT 
    static uint16_t pre_fbits=0;
    uint16_t fbits_tmp=0;
#else
    int reported = 0;
#endif
    uint8_t idx;
    int finger_num;
    int num = 0;
    uint16_t x = 0;
    uint16_t y = 0;
    int position = 0;
    uint8_t button_byte = 0;

    finger_num = FINGERS_NUM;   
#ifdef TWO_FINGERS
    num = buf[7] & 0x03; 
    fbits = buf[7] & 0x03;
    idx=1;
    button_byte = buf[PACKET_SIZE-1];
#endif
    
#ifdef FIVE_FINGERS
    num = buf[1] & 0x07; 
    fbits = buf[1] >>3;
    idx=2;
    button_byte = buf[PACKET_SIZE-1];
#endif
    
#ifdef TEN_FINGERS
    fbits = buf[2] & 0x30;  
    fbits = (fbits << 4) | buf[1];  
    num = buf[2] &0x0f;
    idx=3;
    button_byte = buf[PACKET_SIZE-1];
#endif
    
#ifdef ELAN_ICS_SLOT_REPORT
    fbits_tmp = fbits;
    if(fbits || pre_fbits){
        for(position=0; position<finger_num;position++){
            if(fbits&0x01){
                #ifdef SWAP_X_Y_RESOLUTION
                    elan_ts_parse_xy(&buf[idx], &y, &x);
                #else
                    elan_ts_parse_xy(&buf[idx], &x, &y);
                #endif   
                elan_ts_touch_down(ts, position, x, y, 255);
            }
            else if(pre_fbits&0x01){
                elan_ts_touch_up(ts, position, x, y);
            }
            fbits >>= 1;
            pre_fbits >>= 1;
            idx += 3;   
        }
    }   
    else{
        elan_ts_report_key(ts, button_byte);
    }   
    pre_fbits = fbits_tmp;
#else
    if (num == 0){
        elan_ts_report_key(ts, button_byte);
    } 
    else{
//        elan_info( "[elan] %d fingers", num);
                
        for(position=0; (position<finger_num) && (reported < num);position++){
            if((fbits & 0x01)){
                #ifdef SWAP_X_Y_RESOLUTION
                    elan_ts_parse_xy(&buf[idx], &y, &x);
                #else
                    elan_ts_parse_xy(&buf[idx], &x, &y);
                #endif
                elan_ts_touch_down(ts, position, x, y, 8);
                reported++;
            }
            fbits = fbits >> 1;
            idx += 3;
        }
    }
#endif
    
    input_sync(ts->input_dev);
    return;
}

static irqreturn_t elan_ts_irq_handler(int irq, void *dev_id)
{
    int rc = 0;
    uint8_t buf[64] = {0}; 

    rc = elan_ts_recv_data(private_ts, buf);
    if(rc < 0 || private_ts->input_dev == NULL){
        return IRQ_HANDLED;
    }
    elan_ts_report_data(private_ts, buf);

    return IRQ_HANDLED;
}

static int elan_ts_register_interrupt(struct elan_ts_data *ts )
{
    int err = 0;
    
    err = request_threaded_irq(elan_irq, NULL, elan_ts_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ELAN_TS_NAME, ts);
    
    if (err < 0){
        printk("[elan error] %s: request_irq %d failed, err = %d\n",__func__, ts->client->irq, err);
    }
    return err;
}

#if defined IAP_PORTION
static void check_update_flage(struct elan_ts_data *ts)
{
    int NEW_FW_VERSION = 0;
    int New_FW_ID = 0;
    int rc = 0;
    int retry_cnt = 0;
    
    if(ts->recover == 0x80){
        elan_info("***fw is miss, force update!!!!***\n");
        goto update_elan_fw;    
    }
    
#ifdef ELAN_2K_XX
#if 0
    New_FW_ID = file_fw_data[0x7DB3]<<8  | file_fw_data[0x7DB2];        
    NEW_FW_VERSION = file_fw_data[0x7DB1]<<8  | file_fw_data[0x7DB0];
#else
    New_FW_ID = file_fw_data[0x7BD3]<<8 | file_fw_data[0x7BD2];                 
    NEW_FW_VERSION = file_fw_data[0x7BD1]<<8 | file_fw_data[0x7BD0];
#endif
#endif

#ifdef ELAN_3K_XX
#if 1
    New_FW_ID  = file_fw_data[0xE2CF]<<8  | file_fw_data[0xE2CE];
    NEW_FW_VERSION = file_fw_data[0xDEC3]<<8  | file_fw_data[0xDEC2];
#else
	New_FW_ID  = file_fw_data[0xBDD1]<<8  | file_fw_data[0xBDD0];
	NEW_FW_VERSION = file_fw_data[0xBDD3]<<8  | file_fw_data[0xBDD2];
#endif
#endif
    
    elan_info("FW_ID=0x%x, New_FW_ID=0x%x \n",ts->fw_id, New_FW_ID);
    elan_info("FW_VERSION=0x%x,New_FW_VER=0x%x \n",ts->fw_ver,NEW_FW_VERSION);

    if((ts->fw_id&0xff) != (New_FW_ID&0xff)){
        printk("[elan error] ***fw id is different, can not update !***\n");
        goto no_update_elan_fw;
    }
    else{
        elan_info("fw id is same !\n");
    }
    
    if((ts->fw_ver&0xff) >= (NEW_FW_VERSION&0xff)){
        elan_info("fw version is newest!!\n");
        goto no_update_elan_fw;
    }
   
update_elan_fw:

    update_fw_one(ts->client);
    msleep(500);
    elan_switch_irq(0);
    
    for(retry_cnt=0; retry_cnt<3; retry_cnt++){     
        rc = __fw_packet_handler(private_ts->client);
        if (rc < 0){
            printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
        }
        else{
            break;    
        }
    }   
    
    elan_switch_irq(1);

	msleep(200);
	elan_ts_rough_calibrate(ts->client);
    
no_update_elan_fw:
    elan_info("%s, fw check end..............\n", __func__);
                
    return;
}
#endif

static int elan_request_input_dev(struct elan_ts_data *ts)
{
    int err = 0;
    int i = 0;
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        err = -ENOMEM;
        printk("[elan error] Failed to allocate input device\n");
        return err;
    }
    
    ts->input_dev->evbit[0] = BIT(EV_KEY)|BIT_MASK(EV_REP);

    //key setting
    for (i = 0; i < ARRAY_SIZE(key_value); i++){
        __set_bit(key_value[i], ts->input_dev->keybit);
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
#ifdef ELAN_ICS_SLOT_REPORT
    input_mt_init_slots(ts->input_dev, FINGERS_NUM);
#else
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
    
    elan_info( "%s: x resolution: %d, y resolution: %d\n",__func__, ts->x_resolution, ts->y_resolution);
    input_set_abs_params(ts->input_dev,ABS_MT_POSITION_X,  0, 1472, 0, 0);
    input_set_abs_params(ts->input_dev,ABS_MT_POSITION_Y,  0, 2368, 0, 0);
    
    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);  
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);

    ts->input_dev->name = ELAN_TS_NAME;
    ts->input_dev->phys = "input/ts"; 
    ts->input_dev->id.bustype = BUS_I2C; 
    ts->input_dev->id.vendor = 0xDEAD; 
    ts->input_dev->id.product = 0xBEEF; 
    ts->input_dev->id.version = 2013;

    err = input_register_device(ts->input_dev);
    if (err) {
        input_free_device(ts->input_dev);
        printk("[elan error]%s: unable to register %s input device\n", __func__, ts->input_dev->name);
        return err;
    }
    return 0;
}

static void elan_ic_init_work(struct work_struct *work)
{
    
    int rc = 0;
    int retry_cnt = 0;
    
    if(private_ts->recover == 0){
        elan_switch_irq(0);
        for(retry_cnt=0; retry_cnt<3; retry_cnt++){     
            rc = __fw_packet_handler(private_ts->client);
            if (rc < 0){
                printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
            }
            else{
                break;    
            }
        }   
        elan_switch_irq(1);
    }
//lct--lyh--add tp info
    lct_tp_info("ELAN");

//lct--lyh--add for factory compare
    lct_ctp_upgrade_int(lct_ctp_upgrade_func, lct_get_tp_info);
    
#if defined IAP_PORTION 
    check_update_flage(private_ts);
#endif
    
    //init input devices
    rc = elan_request_input_dev(private_ts);
    if (rc < 0) {
        printk("[elan error]: %s elan_request_input_dev\n", __func__);
    }
    
#ifdef ELAN_ESD_CHECK
    INIT_DELAYED_WORK(&esd_work, elan_touch_esd_func);
    esd_wq = create_singlethread_workqueue("esd_wq");   
    if (!esd_wq) {
        return;
    }
    queue_delayed_work(esd_wq, &esd_work, delay);
#endif
} 

#ifdef CONFIG_OF
static int elan_ts_parse_dt(struct device *dev, struct elan_ts_i2c_platform_data *pdata)
{
    int rc;
    struct device_node *np = dev->of_node;
    
    rc = of_property_read_string(np, "elan,name", &pdata->name);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read name\n");
        return rc;
    }
    
    /* reset, irq, power gpio info */
    pdata->rst_gpio = of_get_named_gpio_flags(np, "elan,rst-gpio", 0, &pdata->rst_gpio_flags);
    if (pdata->rst_gpio < 0){
        return pdata->rst_gpio;
    }
    pdata->intr_gpio = of_get_named_gpio_flags(np, "elan,intr-gpio", 0, &pdata->intr_gpio_flags);
    if (pdata->intr_gpio < 0){
        return pdata->intr_gpio;
    }
	
    pdata->gpio_3v3 = of_get_named_gpio_flags(np, "elan,3v3-gpio", 0, &pdata->gpio_3v3_flags);
    if (pdata->gpio_3v3 < 0){
        return pdata->gpio_3v3;
    }

    pdata->gpio_1v8 = of_get_named_gpio_flags(np, "elan,1v8-gpio", 0, &pdata->gpio_1v8_flags);
    if (pdata->gpio_1v8 < 0){
        return pdata->gpio_1v8;
    }
    return 0;
}
#endif

static int elan_gpio_init(struct i2c_client *client, struct elan_ts_i2c_platform_data *pdata)
{
//    struct regulator * vcc_i2c = pdata->vcc_i2c;
    int rc = 0;
    //pm8917_l10_switch(1);
	#if 0 //mike_zhu
    struct regulator * pm8917_l10_switch = pdata->pm8917_l10_switch;
    struct regulator * pm8917_l17_switch = pdata->pm8917_l17_switch;	
    pr_err("[elan] pm8917_l10_switch enter. \n");
    pm8917_l17_switch = regulator_get(&client->dev, "pm8917_l17_switch");
    if (IS_ERR(pm8917_l17_switch)) {
		rc = PTR_ERR(pm8917_l17_switch);
		pr_err("Regulator get failed pm8917_l17_switch rc=%d\n", rc);
		return rc;
	}
	pdata->pm8917_l17_switch = pm8917_l17_switch;
	if (regulator_count_voltages(pm8917_l17_switch) > 0) {
		rc = regulator_set_voltage(pm8917_l17_switch, 2800000, 2800000); // set 1.8v
		if (rc) {
			pr_err( "Regulator set_vtg failed pm8917_l17_switch rc=%d\n", rc);
			regulator_put(pm8917_l10_switch);
		}
	}		
        rc = regulator_enable(pm8917_l17_switch);
		
    pr_err("[elan] pm8917_l10_switch enter. \n");
    pm8917_l10_switch = regulator_get(&client->dev, "pm8917_l10_switch");
    if (IS_ERR(pm8917_l10_switch)) {
		rc = PTR_ERR(pm8917_l10_switch);
		pr_err("Regulator get failed pm8917_l10_switch rc=%d\n", rc);
		return rc;
	}
	pdata->pm8917_l10_switch = pm8917_l10_switch;
	if (regulator_count_voltages(pm8917_l10_switch) > 0) {
		rc = regulator_set_voltage(pm8917_l10_switch, 2800000, 2800000); // set 1.8v
		if (rc) {
			pr_err( "Regulator set_vtg failed pm8917_l10_switch rc=%d\n", rc);
			regulator_put(pm8917_l10_switch);
		}
	}		
        rc = regulator_enable(pm8917_l10_switch);
#endif
    elan_info("elan_gpio_init enter. \n");
/*
    vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
    if (IS_ERR(vcc_i2c)) {
		rc = PTR_ERR(vcc_i2c);
		dev_err(&client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
		return rc;
	}
	pdata->vcc_i2c = vcc_i2c;
	if (regulator_count_voltages(vcc_i2c) > 0) {
		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000); // set 1.8v
		if (rc) {
			dev_err(&client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			regulator_put(vcc_i2c);
		}
	}
		
        rc = regulator_enable(vcc_i2c);
*/
    //set 3v3
/*
    if (gpio_3v3 >= 0) {
    	rc = gpio_request(gpio_3v3, NULL);
	if (rc < 0) {
		gpio_free(gpio_3v3);
		pr_err("%s: request gpio_3v3 pin failed\n", __func__);
		return rc;
	}
	gpio_direction_output(gpio_3v3, 1);
	pr_err("3v1_en pin =%d\n", gpio_get_value(pdata->gpio_3v3));
} 
*/
/*
	//set 3v3
    if (gpio_1v8 >= 0) {
    	rc = gpio_request(gpio_1v8, NULL);
	if (rc < 0) {
		gpio_free(gpio_1v8);
		pr_err("%s: request gpio_1v8 pin failed\n", __func__);
		return rc;
	}
	gpio_direction_output(gpio_1v8, 1);
	pr_err("1v8_en pin =%d\n", gpio_get_value(pdata->gpio_1v8));
}
*/
    //set reset output high
	if (gpio_3v3 >= 0) {
         rc = gpio_request(gpio_3v3, NULL);
         if (rc < 0) {
                 gpio_free(gpio_3v3);
                 pr_err("%s: request gpio_3v3 pin failed\n", __func__);
                 return rc;
         }
         gpio_direction_output(gpio_3v3, 1);
         pr_err("3v1_en pin =%d\n", gpio_get_value(pdata->gpio_3v3));
 }
    rc = gpio_request(reset_gpio,"tp_reset");
    if (rc < 0) {
		gpio_free(reset_gpio);
		pr_err("%s: request reset_gpio pin failed\n", __func__);
		return rc;
	}
    gpio_direction_output(reset_gpio,1);
    
    //set int pin input
    rc = gpio_request(intr_gpio,"tp_irq");
    if (rc < 0) {
		gpio_free(intr_gpio);
		pr_err("%s: request intr_gpio pin failed\n", __func__);
		return rc;
	}
    gpio_direction_input(intr_gpio);
    return rc;
}

static int elan_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct elan_ts_data *ts;
    struct elan_ts_i2c_platform_data *pdata;
        
    elan_info("%s enter i2c addr %x\n", __func__, client->addr);
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("[elan error] %s: i2c check functionality error\n", __func__);
        return -ENODEV;
    }
    
    ts = kzalloc(sizeof(struct elan_ts_data), GFP_KERNEL);
    if (ts == NULL) {
        printk("[elan error] %s: allocate elan_ts_data failed\n", __func__);
        return -ENOMEM;
    }
    
    /***********platform gpio&irq**********/
    //board file cofing
    if(client->dev.platform_data){
        pdata = client->dev.platform_data;
        if(pdata->init_platform_hw != NULL){
            pdata->init_platform_hw();
        }
    }
#ifdef CONFIG_OF
	//DTS file config
    else if (client->dev.of_node) {
        pdata = kzalloc(sizeof(struct elan_ts_i2c_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allocate elan_ts_i2c_platform_data memory\n");
            return -ENOMEM;
        }
        
        ret = elan_ts_parse_dt(&client->dev, pdata);
        if (ret) {
            dev_err(&client->dev, "DT parsing failed\n");
            return -EINVAL;
        }
    }
#endif
    //user config
    else{
        printk("[elan error] must be set gpio value\n");
        pdata = kzalloc(sizeof(struct elan_ts_i2c_platform_data), GFP_KERNEL);
    }
    
    if (pdata){
	gpio_3v3   =	pdata->gpio_3v3;
	gpio_1v8   =	pdata->gpio_1v8;
        reset_gpio =    pdata->rst_gpio;
        intr_gpio  =    pdata->intr_gpio;
        elan_irq   =    gpio_to_irq(pdata->intr_gpio);
    }
    elan_gpio_init(client, pdata);
    
    elan_info("[elan_reset] %d\n", reset_gpio);
    elan_info("[elan_intr] %d\n", intr_gpio);
    elan_info("[elan_irq] %d\n", elan_irq);
    
    /***********platform gpio&irq**********/
    
    ts->client = client;
    i2c_set_clientdata(client, ts);
    private_ts = ts;
    
    ret = elan_ts_setup(client);
    if (ret < 0) {
        printk("[elan error]: %s No Elan chip inside, return now\n", __func__);
    	return -ENODEV;    
    }
    
    elan_touch_node_init();
    
    ret = elan_ts_register_interrupt(ts);
    if (ret < 0) {
        printk("[elan error]: %s elan_ts_register_interrupt\n", __func__);
        return -EINVAL;
    }
    
    INIT_DELAYED_WORK(&init_work, elan_ic_init_work);
    init_elan_ic_wq = create_singlethread_workqueue("init_elan_ic_wq"); 
    if (!init_elan_ic_wq) {
        return -EINVAL;
    }
    queue_delayed_work(init_elan_ic_wq, &init_work, delay);
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = elan_ts_early_suspend;
    ts->early_suspend.resume = elan_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif
    
    elan_info("+++++++++end porbe+++++++++!\n");
    
    return 0;
}

static int elan_ts_remove(struct i2c_client *client)
{
    struct elan_ts_data *ts = i2c_get_clientdata(client);
    elan_info( "%s: enter\n", __func__);
    
#ifdef SYS_ATTR_FILE
    elan_touch_node_deinit();
#endif
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif
    free_irq(client->irq, ts);

    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}

static int elan_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct elan_ts_data *ts = private_ts;
//    int rc = 0;
//    int i = 0;

    elan_info( "%s: enter\n", __func__);
    if(ts->power_lock==0){
        elan_switch_irq(0);
	gpio_direction_output(reset_gpio,0);
	gpio_direction_output(gpio_3v3, 0);

 //       rc = elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);
    }
    
#ifdef ELAN_ESD_CHECK   
    cancel_delayed_work_sync(&esd_work);
#endif

/*
#ifdef ELAN_ICS_SLOT_REPORT
    for(i=0; i<FINGERS_NUM; i++){
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
    }   
#else
    i = 0;
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    //input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
    input_mt_sync(ts->input_dev);
#endif
    input_sync(ts->input_dev);
  */  
    return 0;
}

static int elan_ts_resume(struct i2c_client *client)
{
    struct elan_ts_data *ts = private_ts;
    int i = 0;


    printk("[elan] %s: enter\n", __func__);
    if(ts->power_lock==0){
        printk("[elan] reset gpio to resum tp\n"); 
	gpio_direction_output(reset_gpio,1);     
        elan_reset();
        elan_switch_irq(1);
    }
#ifdef ELAN_ESD_CHECK
    queue_delayed_work(esd_wq, &esd_work, delay);   
#endif
 #ifdef ELAN_ICS_SLOT_REPORT
    for(i=0; i<FINGERS_NUM; i++){
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
    }   
#else
    i = 0;
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    //input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
    input_mt_sync(ts->input_dev);
#endif
    input_sync(ts->input_dev);
     return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ts_early_suspend(struct early_suspend *h)
{
    struct elan_ts_data *ts =  private_ts;
    elan_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ts_late_resume(struct early_suspend *h)
{
    struct elan_ts_data *ts =  private_ts;
    elan_ts_resume(ts->client);
}
#endif

#if defined(CONFIG_OF)
static struct of_device_id elants_match_table[] = {
	{ .compatible = "elan,ts",},
	{ },
};
#endif

static struct i2c_driver elan_ts_driver = {
    .class = I2C_CLASS_HWMON,
    .probe = elan_ts_probe,
    .remove = elan_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = elan_ts_suspend,
    .resume = elan_ts_resume,
#endif
    .id_table = elan_ts_id,
    .driver = {
        .name = ELAN_TS_NAME,
        .owner = THIS_MODULE,
        #if defined(CONFIG_OF)
        .of_match_table = elants_match_table,
        #endif
    },
};

static int __init elan_ts_init(void)
{
    int ret = -1;

    if(compare_tp_id != 1)
    {
	elan_info("%s failed, compare_tp_id = %d\n", __func__, compare_tp_id);
	return ret;
    }
    elan_info("%s driver 004 version : auto-mapping resolution\n", __func__);   
    ret = i2c_add_driver(&elan_ts_driver);
    elan_info("%s add do i2c_add_driver and the return value=%d\n",__func__,ret);
    return ret;
}

static void __exit elan_ts_exit(void)
{
    elan_info("%s remove driver\n", __func__);
    i2c_del_driver(&elan_ts_driver);
    return;
}

module_init(elan_ts_init);
module_exit(elan_ts_exit);

MODULE_DESCRIPTION("elan KTF Touchscreen Driver");
MODULE_LICENSE("GPL");
