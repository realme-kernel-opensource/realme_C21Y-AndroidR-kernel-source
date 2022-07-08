/************************************************************************
* 
* File Name: touch.c
*
* Author: likaoshan
*
* Created: 2021-02-19
*
* Abstract: for tp Compatibility
*
************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>

#include "touch.h"


#define FW_NAME_LEN                    128

int mode_name = 0;
const char *lcd_str;

struct touch_panel tp_interface;

static int __init get_boot_mode(char *str)
{
	if (!str)
		return 0;

	if (!strncmp(str, "charger", strlen("charger"))){
		mode_name = MSM_BOOT_MODE__CHARGE;
	} else if (!strncmp(str, "cali", strlen("cali"))){
		mode_name = MSM_BOOT_MODE__CALI;
	} else {
		mode_name = MSM_BOOT_MODE__NORMAL;
	}

	printk("mode name from uboot: %s\n", mode_name);

	return 0;
}
__setup("androidboot.mode=", get_boot_mode);


static int __init lcd_name_get(char *str)
{
        if (str != NULL)
                lcd_str = str;
        printk("lcd name from uboot: %s\n", lcd_str);

        return 0;
}
__setup("lcd_name=", lcd_name_get);

void tp_charge_status_switch(int status)
{
	
   if(tp_interface.charger_mode_switch_status){
   	  printk("kaoshan tp charge status switch status = %d \n",status);
      tp_interface.charger_mode_switch_status(status);
   }else{
   	printk("tp charge status switch not func\n");
   	}
   
}
//EXPORT_SYMBOL(tp_charge_stats_switch);

void tp_headset_status_switch(int status)
{
   
   if(tp_interface.headset_switch_status){
   	 printk("kaoshan tp headset status switch status =%d \n",status);
      tp_interface.headset_switch_status(status);
   	}else{
			 printk("tp headset status switch not func\n");
   		}
}
//EXPORT_SYMBOL(tp_headset_stats_switch);


/*add tp interface */
static ssize_t tp_fw_upgrade_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

static ssize_t tp_fw_upgrade_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
   char fwname[FW_NAME_LEN];
   int cnt = count;
   memset(fwname, 0, sizeof(fwname));
   
   snprintf(fwname, count, "%s", buf);
   
   TP_INFO("fw_name = %s ",fwname);
   
   if(tp_interface.tp_inferface_fw_upgrade)
      tp_interface.tp_inferface_fw_upgrade(fwname,cnt);
   else
   	  TP_INFO("tp_inferface_fw_upgrade not func\n");

   
   return -EPERM;
   
}

static ssize_t tp_edge_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

static ssize_t tp_edge_mode_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
   char buf_edge_mode[4];
   int cnt = count;
   memset(buf_edge_mode, 0, sizeof(buf_edge_mode));
   
   snprintf(buf_edge_mode, 4, "%s", buf);


   TP_INFO("buf = %s ",buf);
   
   TP_INFO("buf_edge_mode = %s ",buf_edge_mode);
   
   if(tp_interface.tp_inferface_edge_mode)
      tp_interface.tp_inferface_edge_mode(buf_edge_mode,cnt);
   else
   	  TP_INFO("tp_inferface_fw_upgrade not func\n");

   
   return -EPERM;
   
}

static DEVICE_ATTR(tp_fw_upgrade, S_IRUGO | S_IWUSR, tp_fw_upgrade_show, tp_fw_upgrade_store);
static DEVICE_ATTR(tp_edge_mode, S_IRUGO | S_IWUSR, tp_edge_mode_show,tp_edge_mode_store);

static struct attribute *tp_attributes[] = {
    &dev_attr_tp_fw_upgrade.attr,
	&dev_attr_tp_edge_mode.attr,
    NULL
};

static struct attribute_group tp_attribute_group = {
    .attrs = tp_attributes
};

int tp_create_sysfs( struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &tp_attribute_group);
    if (ret) {
        TP_ERROR("[EX]: sysfs_create_group() failed!!");
        sysfs_remove_group(&dev->kobj, &tp_attribute_group);
        return -ENOMEM;
    } else {
        TP_INFO("[EX]: sysfs_create_group() succeeded!!");
    }

    return ret;
}

int tp_remove_sysfs( struct device *dev)
{
    sysfs_remove_group(&dev->kobj, &tp_attribute_group);
    return 0;
}

struct manufacture_info manufacture_information;

int tp_register_devinfo(int version)
{
	char dev_version[MAX_DEVICE_VERSION_LENGTH], dev_vendor[MAX_DEVICE_MANU_LENGTH];

	if (version <= 0) {
		TP_ERROR("%s version is invalied\n", __func__);
		return -1;
	}
	manufacture_information.version = kzalloc(MAX_DEVICE_VERSION_LENGTH, GFP_KERNEL);
    if (manufacture_information.version == NULL) {
        TP_ERROR("manufacture_information.version kzalloc error\n");
        return -ENOMEM;
    }

	manufacture_information.manufacture = kzalloc(MAX_DEVICE_MANU_LENGTH, GFP_KERNEL);
    if (manufacture_information.manufacture == NULL) {
        TP_ERROR("manufacture_information.manufacture kzalloc error\n");
        return -ENOMEM;
    }
	
	if (!(strncmp(lcd_str,"lcd_icnl9911c_boe_mipi_hd",strlen("lcd_icnl9911c_boe_mipi_hd")))) {
		snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "LSICNL9911C%X", version);
		snprintf(dev_vendor, MAX_DEVICE_MANU_LENGTH, "LIANSI");
	} else if (!(strncmp(lcd_str,"lcd_ft8006s_holitech_mipi_hd",strlen("lcd_ft8006s_holitech_mipi_hd")))) {
		snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "HLTFTS8006%X", version);
		snprintf(dev_vendor, MAX_DEVICE_MANU_LENGTH, "HOLITECH");
	} else if (!(strncmp(lcd_str,"lcd_hx83102d_truly_mipi_hd",strlen("lcd_hx83102d_truly_mipi_hd")))) {
		snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "XLHX83102D%X", version);
		snprintf(dev_vendor, MAX_DEVICE_MANU_LENGTH, "TRULY");
	}  else if (!(strncmp(lcd_str,"lcd_hx83102d_skyworth_mipi_hd",strlen("lcd_hx83102d_skyworth_mipi_hd")))) {
		snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "SKYHX83102D%X", version);
		snprintf(dev_vendor, MAX_DEVICE_MANU_LENGTH, "SKYWORTH");
	} else if (!(strncmp(lcd_str,"lcd_ili7806s_txd_mipi_hd",strlen("lcd_ili7806s_txd_mipi_hd")))) {
		snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "TXDILI7806S%X", version);
		snprintf(dev_vendor, MAX_DEVICE_MANU_LENGTH, "TXD");
	} else {
		TP_ERROR("nothing match lcd name\n");
		return -1;
	}

	sprintf(manufacture_information.version, dev_version);
	sprintf(manufacture_information.manufacture, dev_vendor);

	register_device_proc("tp", manufacture_information.version, manufacture_information.manufacture);
	return 0;
}

__attribute__((weak)) int register_devinfo(char *name, struct manufacture_info *info)
{
	return -1;
}

__attribute__((weak)) int register_device_proc(char *name, char *version, char *vendor)
{
	return -1;
}


