/************************************************************************
* 
* File Name: touch.h
*
* Author: likaoshan
*
* Created: 2021-02-19
*
* Abstract: for tp Compatibility
*
************************************************************************/
	
#ifndef __LINUX_TOUCH_H__
#define __LINUX_TOUCH_H__

#include <soc/oplus/device_info.h>

enum{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
	MSM_BOOT_MODE__CHARGE,
	MSM_BOOT_MODE__SILENCE,
	MSM_BOOT_MODE__SAU,
	MSM_BOOT_MODE__CALI,
};

enum project_board_id{
	S19610JA1_ID = 1,
	S19615NA1_ID,
};

enum tp_ic_type
{
   HIMAX_8311A = 1,
   FTS_8006S = 2,
   FTS_8722 = 3,
};

struct touch_panel {
	void (*headset_switch_status)(int status);
	void (*charger_mode_switch_status)(int status);
	void (*tp_inferface_fw_upgrade)(char *fw_name,int count);
	void (*tp_inferface_edge_mode)(char *buf,int count);
	enum tp_ic_type tp_type;
};

#define TP_INFO(fmt, args...) do { \
    printk(KERN_INFO "[TP_interface/I]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define TP_ERROR(fmt, args...) do { \
    printk(KERN_ERR "[TP_interface/E]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define MAX_DEVICE_VERSION_LENGTH 32
#define MAX_DEVICE_MANU_LENGTH 16

int tp_create_sysfs(struct device *dev);
int tp_remove_sysfs(struct device *dev);

int tp_register_devinfo(int version);


#endif


