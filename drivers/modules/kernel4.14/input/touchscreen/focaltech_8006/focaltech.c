/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
//#include <linux/i2c/ft53x6_ts.h>
//#include <soc/sprd/regulator.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
//#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>

#include <linux/proc_fs.h>
//#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <asm/io.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <uapi/linux/sched/types.h>

#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
//#include <soc/sprd/i2c-sprd.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//#include <soc/sprd/board.h>
#include "touchscreen.h"

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#endif

#if defined(CONFIG_PM_SLEEP) && defined(TS_USE_ADF_NOTIFIER)
#include <video/adf_notifier.h>
#endif

#if defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)
#include <linux/suspend.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
//#include "ft_gesture_lib.h"

static struct wake_lock tp_wakelock;//add by xieyuehong 20161027

#endif

#if defined(CONFIG_PM_SLEEP) && defined(TS_USE_ADF_NOTIFIER)
static struct notifier_block adf_event_block;
#endif
#if defined(CTP_WORK_WITH_CHARGER_PLUGIN)
extern int USB_PLUGIN_FOR_TP;
#endif
#ifdef WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP

#define GESTURE_FUNCTION_SWITCH 	"/data/misc/gesture/gesture_switch"
static int s_gesture_switch = 1;	// Defaultly, the macro is open
#define GESTURE_FUNCTION_CALL_SWITCH 		"/data/misc/gesture/call_status_switch"
#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
static int s_gesture_call_switch = 1;
#endif
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_V		    0x54
#define GESTURE_Z			0x41
#define GESTURE_2			0x65
#define GESTURE_S			0x46
#define WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP_POINTS	255
static int is_sleep = 0;

//add wake lock for gestrue
#define GESTURE_WAKE_LOCK 1
#ifdef GESTURE_WAKE_LOCK
static struct wake_lock suspend_gestrue_lock;
#endif

static short pointnum = 0;
extern suspend_state_t get_suspend_state(void);
#endif

//#define FT53X6_DBG
#ifdef FT53X6_DBG
#define ENTER printk(KERN_INFO "[FT53X6_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[FT53X6_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FT53X6_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FT53X6_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FT53X6_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FT53X6_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FT53X6_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FT53X6_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

#define APK_DEBUG
//#define SPRD_AUTO_UPGRADE
#define SYSFS_DEBUG
#define FTS_CTL_IIC
#include  "focaltech.h"
#include  "focaltech_ex_fun.h"
#include  "focaltech_ctl.h"

#define	USE_WAIT_QUEUE	1
#define	USE_THREADED_IRQ	0
#define	USE_WORK_QUEUE	0

#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0
#define	TS_MAX_FINGER		5

#define	FTS_PACKET_LENGTH	128

#if USE_WAIT_QUEUE
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;
#endif

#if 0
static unsigned char FT5316_FW[]=
{
#include "ft5316_720p.i"
};

static unsigned char FT5306_FW[]=
{
#include "ft5306_qHD.i"
};

static unsigned char *CTPM_FW = FT5306_FW;
#endif
//static int fw_size;

static struct ft5x0x_ts_data *g_ft5x0x_ts;
static struct i2c_client *this_client;

//static unsigned char suspend_flag = 0;

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
static u8 ft5x0x_ts_proximity_enable = 0;
static u8 ft5x0x_ts_proximity_value = 0;
static u8 ft5x0x_ts_proximity_enable_status = 0;
static u8 ft5x0x_ts_proximity_disable_status = 0;
static u8 ft5x0x_ts_status = 0;
#endif

static u8 ft5x0x_ts_debug = 0;

#if 1
struct sprd_i2c_setup_data {
 unsigned i2c_bus;	//the same number as i2c->adap.nr in adapter probe function
 unsigned short i2c_address;
 int irq;
 char type[I2C_NAME_SIZE];
};
//static struct sprd_i2c_setup_data ft5x0x_ts_setup={3, FOCALTECH_TS_ADDR, 0, FOCALTECH_TS_NAME};

static struct ft5x0x_ts_platform_data ft5x0x_ts_info = {
	.irq_gpio_number    = GPIO_TOUCH_IRQ,
	.reset_gpio_number  = GPIO_TOUCH_RESET,
#if defined(WW6_DRV_TOUCHSCREEN_VDD_NAME)
	.vdd_name           = WW6_DRV_TOUCHSCREEN_VDD_NAME,
#else
	.vdd_name           = "vdd28",
#endif
	.TP_MAX_X = CTP_MAX_WIDTH - 1,
	.TP_MAX_Y = CTP_MAX_HEIGHT - 1,
};

struct ft5x0x_key_setup_data{
	u16 key_y;
	u16 key_menu_x;
	u16 key_home_x;
	u16 key_back_x;
	u16 key_search_x;
};
const static struct ft5x0x_key_setup_data ft5x0x_key_setup[]={
	//key_y		key_menu_x		key_home_x		key_back_x		key_search_x
#if defined(WW6_DRV_TP_FT6X0X_KEY_Y) && defined(WW6_DRV_TP_FT6X0X_KEY_MENU_X) && defined(WW6_DRV_TP_FT6X0X_KEY_HOME_X) && defined(WW6_DRV_TP_FT6X0X_KEY_BACK_X)
	{WW6_DRV_TP_FT6X0X_KEY_Y, WW6_DRV_TP_FT6X0X_KEY_MENU_X, WW6_DRV_TP_FT6X0X_KEY_HOME_X, WW6_DRV_TP_FT6X0X_KEY_BACK_X, 0},
#else

#if defined(CONFIG_LCD_QHD)
	{1100,		60,				180,			300,			0},
	{1010,		90,				270,			360,			0},
	{980,		40,				120,			200,			0},
	{1030,		60, 			170,			280,			0},
	{1400,		40, 			120,			200,			0},
#elif defined(CONFIG_LCD_720P)
	{1350,		120, 			360,			600,			0},
	{1400,		40, 			120,			200,			0},
#else
	{900,		80,				240,			400,			0},
	{980,		40,				120,			200,			0},
#endif
#endif

};
#endif



 struct Upgrade_Info fts_updateinfo[] =
{
    {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
	{0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x64,"FT6336GU",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 1300}, //,"FT6336GU"
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000}, //,"FT6x36"
	{0x54,"FT5446U",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 1300}, //,"FT5446U"
};

struct Upgrade_Info fts_updateinfo_curr;
#if 0//dennis
/* Attribute */
static ssize_t ft5x0x_show_suspend(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t ft5x0x_show_version(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_update(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
#endif
//static unsigned char ft5x0x_read_fw_ver(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler);
static void ft5x0x_ts_resume(struct early_suspend *handler);
#endif
//static int fts_ctpm_fw_update(void);
//static int fts_ctpm_fw_upgrade_with_i_file(void);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct ts_event	event;
#if USE_WORK_QUEUE
	struct work_struct	pen_event_work;
	struct workqueue_struct	*ts_workqueue;
#endif
#if defined(CONFIG_PM_SLEEP)
	struct work_struct		 resume_work;
	struct workqueue_struct *ts_resume_workqueue;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct work_struct       resume_work;
	struct workqueue_struct *ts_resume_workqueue;
	struct early_suspend	early_suspend;
#endif
	struct ft5x0x_ts_platform_data	*platform_data;
	spinlock_t irq_lock;		// add by oujx
	s32 irq_is_disable;			// add by oujx
};

#if 0//dennis
static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR, ft5x0x_show_suspend, ft5x0x_store_suspend);
static DEVICE_ATTR(update, S_IRUGO | S_IWUSR, ft5x0x_show_version, ft5x0x_update);

static ssize_t ft5x0x_show_suspend(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(suspend_flag==1)
		sprintf(buf, "FT5x0x Suspend\n");
	else
		sprintf(buf, "FT5x0x Resume\n");

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	suspend_flag = on_off;

	if(on_off==1)
	{
		pr_info("FT5x0x Entry Suspend\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND
		ft5x0x_ts_suspend(NULL);
	#endif
	}
	else
	{
		pr_info("FT5x0x Entry Resume\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND
		ft5x0x_ts_resume(NULL);
	#endif
	}

	return len;
}

static ssize_t ft5x0x_show_version(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;
	unsigned char uc_reg_value;

	uc_reg_value = ft5x0x_read_fw_ver();

	sprintf(buf, "ft5x0x firmware id is V%x\n", uc_reg_value);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ft5x0x_update(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	unsigned char uc_reg_value;

	uc_reg_value = ft5x0x_read_fw_ver();

	if(on_off==1)
	{
		pr_info("ft5x0x update, current firmware id is V%x\n", uc_reg_value);
		//fts_ctpm_fw_update();
		fts_ctpm_fw_upgrade_with_i_file();
	}

	return len;
}

static int ft5x0x_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_suspend);
	err = device_create_file(dev, &dev_attr_update);

	return err;
}
#endif

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	return ret;
}
#if  defined(WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR)//||defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)||defined(CONFIG_HAS_EARLYSUSPEND)

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msg, 1) != 1) {
		ret = -EIO;
		pr_err("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}
#endif
/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret = 0;
	u8 buf[2] = {addr, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf+1,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	*pdata = buf[1];
	return ret;
}

/*******************************************************
Function:
    Disable irq function
Input:
    ts: ft5x0x i2c_client private data
Output:
    None.
*********************************************************/
void ft5x0x_irq_disable(struct ft5x0x_ts_data *ts)
{
	unsigned long irqflags;
	#if 1
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
	        ts->irq_is_disable = 1;
	        disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
	#else
	disable_irq(ts->client->irq);
	#endif
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: ft5x0x i2c_client private data
Output:
    None.
*********************************************************/
void ft5x0x_irq_enable(struct ft5x0x_ts_data *ts)
{
	unsigned long irqflags = 0;

	#if 1
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
	        enable_irq(ts->client->irq);
	        ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
	#else
	enable_irq(ts->client->irq);
	#endif
}


#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
/*static int ft5x0x_get_ps_value(void)
{
	//printk("ft5x0x_get_ps_value %d!\n", ft5x0x_ts_proximity_value);
	return ft5x0x_ts_proximity_value;
}
*/
static int ft5x0x_enable_ps(int enable)
{
	printk("ft5x0x_enable_ps %d!\n", enable);
	if(enable==1)
	{
		if(ft5x0x_ts_status)
		{
			ft5x0x_ts_proximity_enable_status = 0;
			ft5x0x_ts_proximity_enable = 1;
			ft5x0x_write_reg(FT_REG_PS_CTL, 0x1);
		}
		else
		{
			ft5x0x_ts_proximity_enable_status = 1;
		}
	}
	else
	{
		if(ft5x0x_ts_status)
		{
			ft5x0x_ts_proximity_disable_status = 0;
			ft5x0x_ts_proximity_enable = 0;
			ft5x0x_write_reg(FT_REG_PS_CTL, 0x0);
		}
		else
		{
			ft5x0x_ts_proximity_disable_status = 1;
		}
	}

    return 1;
}


static long ft5x0x_ioctl_operate(struct file *file, unsigned int cmd, unsigned long arg)
{

	printk("ft5x0x_ioctl_operate %d!\n", cmd);

	switch(cmd)
	{
		case CTP_IOCTL_PROX_ON:
			ft5x0x_enable_ps(1);
			break;

		case CTP_IOCTL_PROX_OFF:
			ft5x0x_enable_ps(0);
			break;

		default:
			pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
			return -EINVAL;
	}
	return 0;

}

static struct file_operations ft5x0x_proximity_fops = {
	.owner = THIS_MODULE,
	.open = NULL,
	.release = NULL,
	.unlocked_ioctl = ft5x0x_ioctl_operate
};

struct miscdevice ft5x0x_proximity_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = CTP_PROXIMITY_DEVICE_NAME,		//match the hal's name
	.fops = &ft5x0x_proximity_fops,
	.mode = 0x666,
};
#endif


#ifdef TOUCH_VIRTUAL_KEYS
static void virtual_keys_check(u16 *x, u16 *y)
{
	int i;
	if(CTP_MAX_HEIGHT < *y){
		printk("ft5x0x virtual_keys_check x = %d  y = %d\n", *x, *y);
	}else{
		return;
	}
	for(i=0; i < sizeof(ft5x0x_key_setup)/sizeof(struct ft5x0x_key_setup_data); i++){
		if(*y == ft5x0x_key_setup[i].key_y){
			if(*x == ft5x0x_key_setup[i].key_menu_x)
				*x = MENU_CTP_BUTTON_X;
			else if(*x == ft5x0x_key_setup[i].key_home_x)
				*x = HOME_CTP_BUTTON_X;
			else if(*x == ft5x0x_key_setup[i].key_back_x)
				*x = BACK_CTP_BUTTON_X;
			else if(*x == ft5x0x_key_setup[i].key_search_x)
				*x = SEARCH_CTP_BUTTON_X;

			*y = CTP_BUTTON_KEY_Y;
			break;
		}
	}
}

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	//struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	//struct ft5x0x_ts_platform_data *pdata = data->platform_data;
	//return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
	//	,__stringify(EV_KEY), __stringify(KEY_MENU),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
	//	,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
	//	,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
	return sprintf(buf,
		"0x%02x:%d:%d:%d:%d:%d:0x%02x:%d:%d:%d:%d:%d:0x%02x:%d:%d:%d:%d:%d:0x%02x:%d:%d:%d:%d:%d\n",
		EV_KEY, KEY_MENU, MENU_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT,
		EV_KEY, KEY_HOMEPAGE, HOME_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT,
		EV_KEY, KEY_BACK, BACK_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT,
		EV_KEY, KEY_SEARCH, SEARCH_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.focaltech_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void ft5x0x_ts_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("[FST] %s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif

#ifdef WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP
static int ft5x0x_gesture_handle(void)
{
   // poll=0;
    unsigned char buf[WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP_POINTS * 2] = { 0 };
    int ret = -1;
    //int i = 0;
	int gesture_key = 0;
	int gesture_id = 0;
    unsigned char ruby;
	//printk("ft5x0x_gesture_handle:%s\n",__func__);
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
    buf[0] = 0xd3;

	//ft5x0x_write_reg(0xd0, 0x1);
	msleep(10);

    //ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP_POINTS);
    ret = ft5x0x_i2c_rxdata(buf, WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP_POINTS);
    if (ret < 0)
    {
        dev_err(&data->client->dev, "===error===   %s read touchdata failed.\n",            __func__);
        return ret;
    }
    /* FW Ö±\BDÓ¸\F8\B3\F6\CA\D6\CA\C6 */

	//for(i=0;i<10;i++)
	//	printk("[ruby][tpd_gesture_handle]  buf[%d]=%x\n",i,buf[i]);

	ft5x0x_read_reg(0xd0, &ruby);
	//printk("ft5x0x_gesture_handle ruby 0x%x\n",ruby);
/*
    if (0x24 == buf[0])
    {
		//printk("double click\n");
       gesture_id = 0x24;
	   input_report_key(data->input_dev, KEY_U, 1); //KEY_U
       input_sync(data->input_dev);
	   input_report_key(data->input_dev, KEY_U, 0); //KEY_U
       input_sync(data->input_dev);
       return 0;
    }
*/
	if(0xFE == buf[0])
	{
    	pointnum = (short)(buf[1]) & 0xff;
    	//gesture_id = fetch_object_sample(buf,pointnum);
	}
	else
	{
		gesture_id = buf[0];
	}
	switch(gesture_id)
	{
		case 0x24:
			gesture_key =KEY_U;
			break;
		case GESTURE_LEFT:
			gesture_key =KEY_LEFT;
			break;
		case GESTURE_RIGHT:
			gesture_key = KEY_RIGHT;
			break;
		case GESTURE_UP:
			gesture_key = KEY_UP;
			break;
		case GESTURE_DOWN:
			gesture_key = KEY_DOWN;
			break;
		case GESTURE_O:
			gesture_key = KEY_O;
			break;
		case GESTURE_W:
			gesture_key = KEY_W;
			break;
		case GESTURE_M:
			gesture_key = KEY_M;
			break;
		case GESTURE_E:
			gesture_key = KEY_E;
			break;
		case GESTURE_C:
			gesture_key = KEY_C;
			break;
		case GESTURE_V:
			gesture_key = KEY_V;
			break;
		case GESTURE_S:
			gesture_key = KEY_S;
			break;
		case GESTURE_Z:
		case GESTURE_2:
			gesture_key = KEY_Z;
			break;
		case KEY_POWER:
			gesture_key = KEY_POWER;
			break;
		default:
			break;
	}

	printk("ft5x0x_gesture_handle gesture_key=%d,gesture_id=%x\n",gesture_key,gesture_id);

	if(gesture_key > 0){
	   input_report_key(data->input_dev, gesture_key, 1);
       input_sync(data->input_dev);
	   input_report_key(data->input_dev, gesture_key, 0);
       input_sync(data->input_dev);
    }
	msleep(200);

	wake_lock_timeout(&tp_wakelock, msecs_to_jiffies(200));	//add by xieyuehong 20161027
	return 0;
}

#endif

#ifdef __WW6_CDC_COM__

/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)||defined(CONFIG_PM_SLEEP)
static void ft5x0x_clear_report_data(struct ft5x0x_ts_data *ft5x0x_ts)
{
	int i;

	for(i = 0; i < TS_MAX_FINGER; i++) {
	#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(ft5x0x_ts->input_dev, i);
		input_mt_report_slot_state(ft5x0x_ts->input_dev, MT_TOOL_FINGER, false);
	#endif
	}
	input_report_key(ft5x0x_ts->input_dev, BTN_TOUCH, 0);
	#if !MULTI_PROTOCOL_TYPE_B
		input_mt_sync(ft5x0x_ts->input_dev);
	#endif
	input_sync(ft5x0x_ts->input_dev);
}
#endif

static int ft5x0x_update_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[33] = {0};
	int ret = -1;
	int i;
	u16 x , y;
	u8 ft_pressure , ft_size;

#ifdef WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP
	if(is_sleep == 1)
	{
#ifdef GESTURE_WAKE_LOCK
		wake_lock_timeout(&suspend_gestrue_lock, msecs_to_jiffies(2500));
#endif
		ft5x0x_gesture_handle();
		return 0;
	}
#endif

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	if(ft5x0x_ts_proximity_enable)
	{
		u8 proximity_value=0xFF;
		//  0x01-->C0 ¨º????¨¹  E0 ¨º???¨¤?
		ft5x0x_read_reg(0x01, &proximity_value);

		if ((0xE0 == proximity_value)||(0 == proximity_value))// leave
		{
			proximity_value = 1;
		}
		else if (0xC0 == proximity_value)// close to
		{
			proximity_value = 0;
		}

		//printk("proximity_value =0x%x ft5x0x_ts_proximity_value %d\n", proximity_value, ft5x0x_ts_proximity_value);

		if(proximity_value != ft5x0x_ts_proximity_value)
		{
			ft5x0x_ts_proximity_value = proximity_value;
			input_report_abs(data->input_dev, ABS_DISTANCE, ft5x0x_ts_proximity_value);
			input_mt_sync(data->input_dev);
			input_sync(data->input_dev);
			return 0;
		}
	}
#endif

	ret = ft5x0x_i2c_rxdata(buf, 33);

	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	for(i = 0; i < event->touch_point/*TS_MAX_FINGER*/; i++) {
		if((buf[6*i+3] & 0xc0) == 0xc0)
			continue;
		x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];
		y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];
#ifdef TOUCH_VIRTUAL_KEYS
		virtual_keys_check(&x, &y);
	#ifdef FT53X6_DBG
	printk("TOUCH_VIRTUAL_KEYS  x==%d, y==%d\n ", x,y);
	#endif
#endif
#ifdef CTP_NEGATIVE_X

  if((y ==CTP_BUTTON_KEY_Y) && ((x ==MENU_CTP_BUTTON_X) ||(x ==BACK_CTP_BUTTON_X) ||(x ==HOME_CTP_BUTTON_X)))
  {
  }
  else
  {
	x = (CTP_MAX_WIDTH-x >= 0) ? (CTP_MAX_WIDTH-x): x;
  }
#endif
		ft_pressure = buf[6*i+7];
		if(ft_pressure > 127 || ft_pressure == 0)
			ft_pressure = 127;
		ft_size = (buf[6*i+8]>>4) & 0x0F;
		ft_size = (ft_size == 0 ? 1 : ft_size);
		if((buf[6*i+3] & 0x40) == 0x0) {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
		#else
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6*i+5]>>4);
		#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, ft_pressure);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif
			pr_debug("===x%d = %d,y%d = %d ====",i, x, i, y);
			//printk("ft5x0x touch_point %d ===x%d = %d,y%d = %d ====\n",event->touch_point,i, x, i, y);
		}
		else {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
		}
	}
	if(0 == event->touch_point) {
		for(i = 0; i < TS_MAX_FINGER; i ++) {
			#if MULTI_PROTOCOL_TYPE_B
                            input_mt_slot(data->input_dev, i);
                            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			#endif
		}
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif

	}
	input_sync(data->input_dev);

	return 0;
}

#if USE_WAIT_QUEUE
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, (0 != tpd_flag));
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		ft5x0x_update_data();

	} while (!kthread_should_stop());

	return 0;
}
#endif

#if USE_WORK_QUEUE
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	ft5x0x_update_data();
	//enable_irq(this_client->irq);
	ft5x0x_irq_enable(g_ft5x0x_ts);
}
#endif

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
#if defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)
	#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	if (s_gesture_switch && s_gesture_call_switch)
	#else
	if (s_gesture_switch)
	#endif
	{
	irq_set_irq_type(this_client->irq,IRQF_TRIGGER_FALLING);
	}
#endif

#if USE_WAIT_QUEUE
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
#endif

#if USE_WORK_QUEUE
//	disable_irq_nosync(fts_i2c_client->irq);
	struct ft5x0x_ts_data *ft5x0x_ts = (struct ft5x0x_ts_data *)dev_id;

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}
	return IRQ_HANDLED;
#endif

#if USE_THREADED_IRQ
	ft5x0x_update_data();
	return IRQ_HANDLED;
#endif

}

static void ft5x0x_ts_reset(void)
{
	struct ft5x0x_ts_platform_data *pdata = g_ft5x0x_ts->platform_data;
	int retval = 0;

	retval = gpio_direction_output(pdata->reset_gpio_number, 1);
	if (retval < 0){
		pr_warn("%s: Failed get gpio_direction_output\n", __func__);
		goto free_reset_request;
	}
	msleep(1);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(200);

	free_reset_request:
	gpio_free(pdata->reset_gpio_number);
}

#if defined(CTP_WORK_WITH_CHARGER_PLUGIN)
//extern unsigned char usb_flag;
void ft5x0x_work_ac_usb_plugin(int plugin)
{
	if(plugin)
	  ft5x0x_write_reg(0x8b, 0x01);
	else
	  ft5x0x_write_reg(0x8b, 0x00);

}
void ft5x0x_work_with_ac_usb_plugin(int plugin)
{
	//u8 val = 0;
	//return ft5x0x_write_reg(data->client, 0x8B, val);
	//ft5x0x_ts_reset();
	if(plugin)
	  ft5x0x_write_reg(0x8b, 0x01);
	else
	  ft5x0x_write_reg(0x8b, 0x00);

	//ft5x0x_read_reg(0x8b, &usb_flag);
	//usb_flag=USB_PLUGIN_FOR_TP;

}
#endif

#if 0
//for future use

struct regulator *vdd28 = NULL;

static void ft53x6_power_off(void)
{
	if(vdd28 != NULL)
		regulator_force_disable(vdd28);
	PRINT_INFO("power off\n");
}

static void ft53x6_power_on(void)
{
	int err = 0;

	if(vdd28 == NULL) {
		vdd28 = regulator_get(NULL, "vdd28");
		if (IS_ERR(vdd28)) {
			PRINT_ERR("regulator_get failed\n");
			return;
		}
		err = regulator_set_voltage(vdd28,2800000,2800000);
		if (err)
			PRINT_ERR("regulator_set_voltage failed\n");
	}
	regulator_enable(vdd28);

	PRINT_INFO("power on\n");
}
#endif
#if 0
//static void ft5x0x_suspend(void)
static int ft5x0x_suspend(struct device *dev)
{
	int ret = -1;
	pr_info("==%s==\n", __FUNCTION__);

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	if(ft5x0x_ts_proximity_disable_status)
	{
		ft5x0x_enable_ps(0);
	}

	if(ft5x0x_ts_proximity_enable)
	{
		return;
	}
#endif

#if defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)
{
	struct file *fp = NULL; 
	#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	fp = filp_open(GESTURE_FUNCTION_CALL_SWITCH, O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		printk("open file %s error!\n", GESTURE_FUNCTION_CALL_SWITCH);
		s_gesture_call_switch = 1;	
	} 
	else 
	{
		printk("open file %s success!\n", GESTURE_FUNCTION_CALL_SWITCH);
		s_gesture_call_switch = 0;	
		filp_close(fp, NULL); 
	}
	
	#endif

	fp = NULL;
	fp = filp_open(GESTURE_FUNCTION_SWITCH, O_RDONLY , 0); 
	if (!IS_ERR(fp)) 
	{ 
		printk("open file %s success!\n", GESTURE_FUNCTION_SWITCH);
		s_gesture_switch = 1;
		filp_close(fp, NULL); 	
	} 
	else 
	{
		printk("open file %s error!\n", GESTURE_FUNCTION_SWITCH);
		s_gesture_switch = 0;	
	}

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	if (s_gesture_switch && s_gesture_call_switch)
#else
	if (s_gesture_switch)
#endif
	{
		is_sleep = 1;
		ft5x0x_write_reg(0xd0, 0x1);
		irq_set_irq_type(this_client->irq,IRQF_TRIGGER_LOW);
		return;
	}
}
#endif

	ret = ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	if(ret){
		PRINT_ERR("==ft5x0x_ts_suspend==  ft5x0x_write_reg fail\n");
	}
	//disable_irq(this_client->irq);
	ft5x0x_irq_disable(g_ft5x0x_ts);
	ft5x0x_clear_report_data(g_ft5x0x_ts);
	//gpio_set_value(g_ft5x0x_ts->platform_data->reset_gpio_number, 0);//for future use
#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR	
	ft5x0x_ts_status = 0;
#endif
return 0;
}

//static void ft5x0x_resume(void)
static int ft5x0x_resume(struct device *dev)
{
	struct ft5x0x_ts_data  *ft5x0x_ts = (struct ft5x0x_ts_data *)i2c_get_clientdata(this_client);
	queue_work(ft5x0x_ts->ts_resume_workqueue, &ft5x0x_ts->resume_work);
	return 0;
}
static SIMPLE_DEV_PM_OPS(ft5x0x_pm_ops, ft5x0x_suspend, ft5x0x_resume);
#endif
#if defined(CTP_WORK_WITH_HALL_SUPPORT)
//extern unsigned char usb_flag;
extern int WW6_HALL_STATUS;
void ft5x0x_work_hall_work(int status)
{
	if(status)
	  ft5x0x_write_reg(0xc1, 0x01);
	else
	  ft5x0x_write_reg(0xc1, 0x00);
	
	//ft5x0x_read_reg(0xc1, &usb_flag);
	//usb_flag=USB_PLUGIN_FOR_TP;

}
void ft5x0x_work_hall_support(void)
{
	ft5x0x_suspend();
	ft5x0x_resume();
}
#endif

static void ft5x0x_ts_resume_work(struct work_struct *work)
{
	pr_info("==%s==\n", __FUNCTION__);

#if defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)
	is_sleep = 0;
	ft5x0x_write_reg(0xd0, 0x00);
	irq_set_irq_type(this_client->irq,IRQF_TRIGGER_FALLING);
#endif
#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	if(ft5x0x_ts_proximity_enable)
	{
		//ft5x0x_enable_ps(1);
		return;
	}
#endif

	ft5x0x_ts_reset();
	//ft5x0x_write_reg(FT5X0X_REG_PERIODACTIVE, 7);
	//enable_irq(this_client->irq);
	ft5x0x_irq_enable(g_ft5x0x_ts);
	msleep(2);
	ft5x0x_clear_report_data(g_ft5x0x_ts);
#if defined(CTP_WORK_WITH_CHARGER_PLUGIN)
	ft5x0x_work_ac_usb_plugin(USB_PLUGIN_FOR_TP);
#endif
#if defined(CTP_WORK_WITH_HALL_SUPPORT)
    ft5x0x_work_hall_work(WW6_HALL_STATUS);
#endif

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	ft5x0x_ts_status = 1;
	if(ft5x0x_ts_proximity_enable_status)
	{
		ft5x0x_enable_ps(1);
	}
#endif
}
#if defined(CONFIG_PM_SLEEP) && defined(TS_USE_ADF_NOTIFIER)
static int ts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{
	struct adf_notifier_event *event = data;
	int adf_event_data;


	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;
	//PRINT_INFO("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		ft5x0x_resume();
		break;
	case DRM_MODE_DPMS_OFF:
		ft5x0x_suspend();
		break;
	default:
		//TS_WARN("receive adf event with error data, adf_event_data=%d",adf_event_data);
		break;
	}

	return NOTIFY_OK;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	ft5x0x_suspend();
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	ft5x0x_resume();
}
#endif

static int ft5x0x_ts_hw_init(struct ft5x0x_ts_data *ft5x0x_ts)
{
	struct regulator *reg_vdd;
	struct i2c_client *client = ft5x0x_ts->client;
	struct ft5x0x_ts_platform_data *pdata = ft5x0x_ts->platform_data;
	int ret = 0;

	pr_info("[FST] %s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
	ret = gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	if (ret < 0) {
		pr_info("[FST] Failed to request irq(%d) gpio\n", pdata->irq_gpio_number);
		return -EINVAL;
	}

	ret = gpio_direction_input(pdata->irq_gpio_number);
        if (ret < 0) {
                pr_info("[FST] Failed to request irq(%d) gpio\n", pdata->irq_gpio_number);
                goto free_irq_request;
        }

	ret = gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	if (ret < 0) {
		pr_info("[FST] Failed to request reset(%d) gpio\n", pdata->reset_gpio_number);
		goto free_irq_request;
	}

	ret = gpio_direction_output(pdata->reset_gpio_number, 1);
	if (ret < 0) {
		pr_info("[FST] Failed to request reset(%d) gpio\n", pdata->reset_gpio_number);
		goto free_reset_request;
	}

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[FST] ft5x0x_ts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		regulator_set_voltage(reg_vdd, 2800000, 2800000);
		ret=regulator_enable(reg_vdd);
		if (ret) {
			printk("ft5x0x_ts_hw_init:regulator_enable fail\n");
		}
	}
	msleep(100);
	ft5x0x_ts_reset();
	return ret;

	free_reset_request:
	gpio_free(pdata->reset_gpio_number);

	free_irq_request:
	gpio_free(pdata->irq_gpio_number);

	return ret;
}

unsigned char FT6XX6_CHIP_ID;

void focaltech_get_upgrade_array(struct i2c_client *client)
{
	u8 chip_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(client,FT_REG_CHIP_ID,1,&chip_id);
    chip_id = FT6XX6_CHIP_ID;//0x64;
	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

#ifdef CONFIG_OF
static struct ft5x0x_ts_platform_data *ft5x0x_ts_parse_dt(struct device *dev)
{

	struct ft5x0x_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	
	dev_err(dev, "hx_ft5x0x_ts_parse_dt");
	printk("%s ft5x0x_ts_parse_dt .\n", __func__);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	} 
	ret = of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}
	pdata->TP_MAX_Y = CTP_MAX_HEIGHT;//ZCFG_LCM_HEIGHT;
	pdata->TP_MAX_X = CTP_MAX_WIDTH;//ZCFG_LCM_WIDTH;
	//if(pdata)
	//{
	//dev_err(dev, "hx_ft5x0x_ts_parse_dt_return %s\n",pdata);
	//printk("ft5x0x_ts_parse_dt_return:\n", pdata);
	//}
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static ssize_t ft5x0x_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ft5x0x debug %d\n", ft5x0x_ts_debug);
}


/* Allow users to enable/disable the device */
static ssize_t ft5x0x_debug_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	//int						ret;
	unsigned long			enable;

	enable = simple_strtoul(buf, NULL, 10);    
	//printk("%s:enable=0x%lx\n", __FUNCTION__, enable);
	ft5x0x_ts_debug = (enable > 0) ? 1 : 0;
	return count;

}



#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
static ssize_t ft5x0x_ps_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ft5x0x ps enable %d\n", ft5x0x_ts_proximity_enable);
}

/* Allow users to enable/disable the device */
static ssize_t ft5x0x_ps_enable_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
//	int						ret;
	unsigned long			enable;

	enable = simple_strtoul(buf, NULL, 10);    
	//printk("%s:enable=0x%lx\n", __FUNCTION__, enable);
	enable = (enable > 0) ? 1 : 0;
	
	ft5x0x_enable_ps(enable);
		
	return count;

}
#endif

/*===================================================================
The definitions are located in src/include/stat.h
Possible options include:
S_IRUSR	/ S_IWUSR	enable read/write permission for user
S_IRGRP / S_IWGRP	enable read/write permission for group
S_IROTH / S_IWOTH	enable read/write permission for other
S_IRUGO / S_IWUGO	enable read/write perimission for user+group+other
Alternatively, you can directly use the number to indicate the permission, where S_IRUSR = 00400 and etc.
Current setting, S_IRUGO|S_IWUSR means 644.
=====================================================================*/
#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
//static DEVICE_ATTR(ps_enable, S_IRUGO|S_IWUGO, ft5x0x_ps_enable_show, ft5x0x_ps_enable_store);
static struct device_attribute ctp_ps_enable_attribute = __ATTR(ps_enable,0664,ft5x0x_ps_enable_show,ft5x0x_ps_enable_store);
#endif
//static DEVICE_ATTR(debug, S_IRUGO|S_IWUGO, ft5x0x_debug_show, ft5x0x_debug_store);
static struct device_attribute ctp_dev_debug_attribute = __ATTR(debug,0664,ft5x0x_debug_show,ft5x0x_debug_store);


static struct attribute *ft5x0x_attributes[] = {
#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR	
	&ctp_ps_enable_attribute.attr,
#endif
	&ctp_dev_debug_attribute.attr,
	NULL
};
static const struct attribute_group ft5x0x_attr_group = {
	.attrs = ft5x0x_attributes,
};


#ifdef __WW6_CDC_COM__
extern char TP_NAME[20];
extern  unsigned char TP_FW_VER;
#endif

static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	unsigned char uc_reg_value;
#ifdef __WW6_CDC_COM__
	u8 i;
#endif
#ifdef CONFIG_OF
	struct device_node		*np = client->dev.of_node;
#endif

	pr_info("[FST] %s: probe\n",__func__);

#ifdef __WW6_CDC_COM__
	if(cdc_tp_device_id(0) != 0)
	{
		pr_err("CTP(0x%x)Exist!", cdc_tp_device_id(0));
		return -ENODEV;
	}	
#endif

#ifdef CONFIG_OF
	if (np && !pdata){
		pdata = ft5x0x_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_ft5x0x_ts = ft5x0x_ts;
	if(pdata){
		ft5x0x_ts->platform_data = pdata;
	}else{
		ft5x0x_ts->platform_data = &ft5x0x_ts_info;
		pdata = &ft5x0x_ts_info;
	}
	this_client = client;
	ft5x0x_ts->client = client;

	err = ft5x0x_ts_hw_init(ft5x0x_ts);
	if (err < 0) {
		pr_info("%s: Failed to request gpio\n", __func__);
		goto exit_chip_check_failed;
	}

	i2c_set_clientdata(client, ft5x0x_ts);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	#ifdef CONFIG_I2C_SPRD
	//sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
	#endif

	err = ft5x0x_read_reg(FT5X0X_REG_CIPHER, &uc_reg_value);
	if (err < 0)
	{
		pr_err("[FST] read chip id error %x\n", uc_reg_value);
		err = -ENODEV;
		goto exit_chip_check_failed;
	}
#ifdef __WW6_CDC_COM__
{
	TP_FW_VER = ft5x0x_read_fw_ver();//fw id
	
	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(uc_reg_value==fts_updateinfo[i].CHIP_ID)
		{
		  FT6XX6_CHIP_ID=uc_reg_value;
		  strcpy(TP_NAME,fts_updateinfo[i].FTS_NAME);//ID name
		}
    }
}
#endif

#ifdef __WW6_CDC_COM__
	cdc_tp_lock_mutex();
	cdc_tp_device_id(0x6206);
	cdc_tp_unlock_mutex();	
#endif

	/* set report rate, about 70HZ */
	//ft5x0x_write_reg(FT5X0X_REG_PERIODACTIVE, 7);
#if USE_WORK_QUEUE
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue("focal-work-queue");
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)||defined(CONFIG_PM_SLEEP)
	INIT_WORK(&ft5x0x_ts->resume_work, ft5x0x_ts_resume_work);
	ft5x0x_ts->ts_resume_workqueue = create_singlethread_workqueue("ft5x0x_ts_resume_work");
	if (!ft5x0x_ts->ts_resume_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif
#if 0//def WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	err = misc_register(&ft5x0x_proximity_misc);
	if (err < 0)
	{
		pr_err("%s: could not register misc device\n", __func__);
		goto err_mis_reg;
	}
#endif

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[FST] failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	ft5x0x_ts_virtual_keys_init();
#endif
	ft5x0x_ts->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit); //for google img 2018 04 23

#ifdef WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_U);
	input_set_capability(input_dev, EV_KEY, KEY_O);
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_Z);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	//init_para(CTP_MAX_WIDTH,CTP_MAX_HEIGHT,100,0,0);
	
#ifdef GESTURE_WAKE_LOCK
	wake_lock_init(&suspend_gestrue_lock, WAKE_LOCK_SUSPEND, "suspend_gestrue");
#endif	

#endif	

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER,0);
#endif
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_PRESSURE, 0, 127, 0, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif
	#if 0
	/*ft5306's firmware is qhd, ft5316's firmware is 720p*/
	if (uc_reg_value == 0x0a || uc_reg_value == 0x0) {
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, 720, 0, 0);
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, 1280, 0, 0);
	} else {
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, 540, 0, 0);
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, 960, 0, 0);
	}
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

#ifdef WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR
	//err = misc_register(&ft5x0x_proximity_misc);
	//if (err < 0)
	//{
	//	pr_err("%s: could not register misc device\n", __func__);
	//	goto exit_input_register_misc_device_failed;
	//}	
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	ft5x0x_ts_status = 1;
#endif		

	input_dev->name = FOCALTECH_TS_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"[FST] ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	spin_lock_init(&ft5x0x_ts->irq_lock); // add by oujx

	err = sysfs_create_group(&(ft5x0x_ts->input_dev->dev.kobj), &ft5x0x_attr_group);
	if(err < 0)
	{
		pr_err("%s: could not sysfs_create_group\n", __func__);
		goto exit_sysfs_create_group_failed;
	}
 
#if defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)//||defined(WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR) 
#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, ft5x0x_ts);
#else
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, ft5x0x_ts);
#endif
#if defined(WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP)
	wake_lock_init(&tp_wakelock,WAKE_LOCK_SUSPEND, "tp_input_wakelock");//add by xieyuehong 20161027
#endif

#else ////////////

#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ft5x0x_ts);
#else
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ft5x0x_ts);
#endif
#endif
	if (err < 0) {
		dev_err(&client->dev, "[FST] ft5x0x_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}

	//ft5x0x_ts->irq_is_disable = 1;
	//ft5x0x_irq_enable(ft5x0x_ts);
#if defined(CONFIG_PM_SLEEP) && defined(TS_USE_ADF_NOTIFIER)
	adf_event_block.notifier_call = ts_adf_event_handler;
	adf_register_client(&adf_event_block);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

focaltech_get_upgrade_array(client);

#ifdef SYSFS_DEBUG	
fts_create_sysfs_ww(client);
#endif

#ifdef FTS_CTL_IIC	
if (ft_rw_iic_drv_init(client) < 0)	
{
	dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",	__func__);
}
#endif

#if defined(SPRD_AUTO_UPGRADE) || defined(WW6_DRV_TP_FW_AUTO_UPGRADE) 
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(client);
#endif   

#ifdef APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
#endif

#if USE_WAIT_QUEUE
	thread = kthread_run(touch_event_handler, 0, "focal-wait-queue");
	if (IS_ERR(thread))
	{
		err = PTR_ERR(thread);
		PRINT_ERR("failed to create kernel thread: %d\n", err);
	}
#endif

	return 0;

exit_sysfs_create_group_failed:
	sysfs_remove_group(&(ft5x0x_ts->input_dev->dev.kobj), &ft5x0x_attr_group);

exit_irq_request_failed:
	input_unregister_device(input_dev);
#if 0//def WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR	
exit_input_register_misc_device_failed:
	//misc_deregister(&ft5x0x_proximity_misc);
#endif	
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
#if 0//def WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR	
	misc_deregister(&ft5x0x_proximity_misc);
	err_mis_reg:
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (ft5x0x_ts->ts_resume_workqueue) {
		destroy_workqueue(ft5x0x_ts->ts_resume_workqueue);
	}
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)||(USE_WORK_QUEUE)||defined(CONFIG_PM_SLEEP)
exit_create_singlethread:
#endif
exit_chip_check_failed:
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	ft5x0x_ts = NULL;
	i2c_set_clientdata(client, ft5x0x_ts);
exit_alloc_platform_data_failed:

#ifdef __WW6_CDC_COM__
	cdc_tp_lock_mutex();
	cdc_tp_device_id(0xFFFF);
	cdc_tp_unlock_mutex();	
#endif	
	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	pr_info("==ft5x0x_ts_remove=\n");
	
	#ifdef SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC	
	ft_rw_iic_drv_exit();
	#endif
	#ifdef APK_DEBUG
	ft5x0x_release_apk_debug_channel();
	#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	input_free_device(ft5x0x_ts->input_dev);
#if USE_WORK_QUEUE
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
#endif
#if 0//def WW6_DRV_TOUCHSCREEN_PROXIMITY_SENSOR	
	misc_deregister(&ft5x0x_proximity_misc);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&ft5x0x_ts->resume_work);
	destroy_workqueue(ft5x0x_ts->ts_resume_workqueue);
#endif
	kfree(ft5x0x_ts);
	ft5x0x_ts = NULL;
	i2c_set_clientdata(client, ft5x0x_ts);
#ifdef WW6_DRV_TOUCHSCREEN_GESTURE_WAKEUP	
	wake_lock_destroy(&tp_wakelock);//add by xieyuehong 20161027
#endif
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FOCALTECH_TS_NAME, 0 },{ }
};

/*
static int ft5x0x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	PRINT_INFO("ft5x0x_suspend\n");
	return 0;
}
static int ft5x0x_resume(struct i2c_client *client)
{
	PRINT_INFO("ft5x0x_resume\n");
	return 0;
}
*/
#if 0
static int sprd_add_i2c_device(struct sprd_i2c_setup_data *i2c_set_data, struct i2c_driver *driver)
{
	struct i2c_board_info	info;
	struct i2c_adapter		*adapter;
	struct i2c_client		*client;
	int						ret,err;

	printk("%s : i2c_bus=%d; slave_address=0x%x; i2c_name=%s",__func__,i2c_set_data->i2c_bus, \
		i2c_set_data->i2c_address, i2c_set_data->type);

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = i2c_set_data->i2c_address;
	strlcpy(info.type, i2c_set_data->type, I2C_NAME_SIZE);
	if(i2c_set_data->irq > 0)
		info.irq = i2c_set_data->irq;

	adapter = i2c_get_adapter( i2c_set_data->i2c_bus);
	if (!adapter) {
		printk("%s: can't get i2c adapter %d\n",
			__func__,  i2c_set_data->i2c_bus);
		err = -ENODEV;
		goto err_adapter;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		printk("%s:  can't add i2c device at 0x%x\n",
			__func__, (unsigned int)info.addr);
		err = -ENODEV;
		goto err_device;
	}

	i2c_put_adapter(adapter);

	ret = i2c_add_driver(driver);
	if (ret != 0) {
		printk("%s: can't add i2c driver\n", __func__);
		err = -ENODEV;
		goto err_driver;
	}	

	return 0;

err_driver:
	i2c_unregister_device(client);
	return err;
err_device:
	i2c_put_adapter(adapter);
err_adapter:
	return err;
}

static void sprd_del_i2c_device(struct i2c_client *client, struct i2c_driver *driver)
{
	printk("%s : slave_address=0x%x; i2c_name=%s",__func__, client->addr, client->name);
	if (client)
		i2c_unregister_device(client);
	i2c_del_driver(driver);
}
#endif


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static const struct of_device_id focaltech_of_match[] = {
       { .compatible = "focaltech,focaltech_ts", },
       { }
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);
static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FOCALTECH_TS_NAME,
		.owner	= THIS_MODULE,
		//.pm	= &ft5x0x_pm_ops,
		.of_match_table = focaltech_of_match,
	},
	//.suspend = ft5x0x_suspend,
	//.resume = ft5x0x_resume,
};

static int __init ft5x0x_ts_init(void)
{
#if 1
/*	if(cdc_tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", cdc_tp_device_id(0));
		return -ENODEV;
	}*/

	return i2c_add_driver(&ft5x0x_ts_driver);
#else
	return sprd_add_i2c_device(&ft5x0x_ts_setup, &ft5x0x_ts_driver);
#endif
}

static void __exit ft5x0x_ts_exit(void)
{
#if 1
	i2c_del_driver(&ft5x0x_ts_driver);
#else
	sprd_del_i2c_device(this_client, &ft5x0x_ts_driver);	
#endif
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
