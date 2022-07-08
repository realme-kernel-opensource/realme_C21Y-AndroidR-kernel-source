// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <soc/oplus/device_info.h>
//#include <soc/oplus/system/oplus_project.h>
#include <linux/hardware_info.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "../../../../fs/proc/internal.h"
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/iio/consumer.h>
#include <linux/of_fdt.h>

#define DEVINFO_NAME "devinfo"

#define dev_msg(msg, arg...) pr_err("devinfo:" msg, ##arg);

#define BOARD_GPIO_SUPPORT 4
#define MAIN_BOARD_SUPPORT 256

static struct proc_dir_entry *g_parent = NULL;
struct device_info {
	struct device *dev;
	struct pinctrl *p_ctrl;
	struct pinctrl_state *active[BOARD_GPIO_SUPPORT], *sleep[BOARD_GPIO_SUPPORT];
	struct list_head dev_list;
};

static struct device_info *g_dev_info = NULL;

static int match_board(struct manufacture_info *info)
{
	dev_msg("before info->manufacture = %s\n", info->manufacture);
	info->manufacture = sub_match_info_get();
	dev_msg("after  info->manufacture = %s\n", info->manufacture);
	return 0;
}

bool check_id_match(const char *label, const char *id_match, int id)
{
	struct o_hw_id *pos = NULL;

	list_for_each_entry(pos, & (g_dev_info->dev_list), list)
	{
		if (sizeof(label) != sizeof(pos->label)) {
			continue;
		}
		if (!strcasecmp(pos->label, label)) {
			if (id_match) {
				if (!strcasecmp(pos->match, id_match)) {
					return true;
				}
			} else {
				if (pos->id == id) {
					return true;
				}
			}
		}
	}

	return false;
}

static int devinfo_read_func(struct seq_file *s, void *v)
{
	struct manufacture_info *info = (struct manufacture_info *) s->private;

	if (strcmp(info->name, "audio_mainboard") == 0) {
		match_board(info);
	}

	if (info->version) {
		seq_printf(s, "Device version:\t\t%s\n", info->version);
	}

	if (info->manufacture) {
		seq_printf(s, "Device manufacture:\t\t%s\n", info->manufacture);
	}

	if (info->fw_path) {
		seq_printf(s, "Device fw_path:\t\t%s\n", info->fw_path);
	}

	return 0;
}

static int device_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, devinfo_read_func, PDE_DATA(inode));
}

static const struct file_operations device_node_fops = {
	.owner = THIS_MODULE,
	.open = device_info_open,
	.read = seq_read,
	.release = single_release,
};

static int devinfo_read_ufsplus_func(struct seq_file *s, void *v)
{
	struct o_ufsplus_status *ufsplus_status  = (struct o_ufsplus_status *)s->private;
	if (!ufsplus_status) {
		return -EINVAL;
	}
	seq_printf(s, "HPB status: %d\n", * (ufsplus_status->hpb_status));
	seq_printf(s, "TW status: %d\n", * (ufsplus_status->tw_status));
	return 0;
}

static int device_info_for_ufsplus_open(struct inode *inode, struct file *file)
{
	return single_open(file, devinfo_read_ufsplus_func, PDE_DATA(inode));
}


static const struct file_operations device_node_for_ufsplus_fops = {
	.owner = THIS_MODULE,
	.open = device_info_for_ufsplus_open,
	.read = seq_read,
	.release = single_release,
};

static int deviceid_read_func(struct seq_file *s, void *v)
{
	struct o_hw_id *info = (struct o_hw_id *) s->private;

	if (info->match) {
		seq_printf(s, "%s", info->match);
	} else {
		seq_printf(s, "%d", info->id);
	}

	return 0;
}

static int device_id_open(struct inode *inode, struct file *file)
{
	return single_open(file, deviceid_read_func, PDE_DATA(inode));
}

static const struct file_operations device_id_fops = {
	.owner = THIS_MODULE,
	.open = device_id_open,
	.read = seq_read,
	.release = single_release,
};

int register_device_id(struct device_info *dev_info, const char *label, const char *id_match, int id)
{
	struct o_hw_id *hw_id = NULL;

	hw_id = (struct o_hw_id *) kzalloc(sizeof(*hw_id), GFP_KERNEL);
	if (!hw_id) {
		return -ENOMEM;
	}

	hw_id->label = label;
	hw_id->match = id_match;
	hw_id->id = id;

	list_add(& (hw_id->list), & (dev_info->dev_list));

	if (!proc_create_data(label, S_IRUGO, g_parent, &device_id_fops, hw_id)) {
		dev_msg("failed to create entry %s \n", label);
	}

	return 0;
}

int register_devinfo(char *name, struct manufacture_info *info)
{
	struct proc_dir_entry *d_entry;

	if (!info) {
		return -EINVAL;
	}

	memcpy(info->name, name, strlen(name) > INFO_LEN-1?INFO_LEN-1:strlen(name));

	d_entry = proc_create_data(name, S_IRUGO, g_parent, &device_node_fops, info);
	if (!d_entry) {
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(register_devinfo);

int register_device_proc_for_ufsplus(char *name, int *hpb_status, int *tw_status)
{
	struct proc_dir_entry *d_entry;
	struct o_ufsplus_status *ufsplus_status;
	ufsplus_status = (struct o_ufsplus_status *)kzalloc(sizeof(*ufsplus_status), GFP_KERNEL);
	if (!ufsplus_status) {
		return -ENOMEM;
	}

	ufsplus_status->hpb_status = hpb_status;
	ufsplus_status->tw_status = tw_status;

	d_entry = proc_create_data(name, S_IRUGO, g_parent, &device_node_for_ufsplus_fops, ufsplus_status);
	if (!d_entry) {
		kfree(ufsplus_status);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(register_device_proc_for_ufsplus);

int register_device_proc(char *name, char *version, char *vendor)
{
	struct manufacture_info *info;

	if (!g_parent) {
		return -ENOMEM;
	}
	info = (struct manufacture_info *) kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	if (version) {
		info->version = (char *) kzalloc(32, GFP_KERNEL);
		if (!info->version) {
			kfree(info);
			return -ENOMEM;
		}
		memcpy(info->version, version, strlen(version) > 31?31:strlen(version));
	}
	if (vendor) {
		info->manufacture = (char *) kzalloc(32, GFP_KERNEL);
		if (!info->manufacture) {
			kfree(info->version);
			kfree(info);
			return -ENOMEM;
		}
		memcpy(info->manufacture, vendor, strlen(vendor) > 31?31:strlen(vendor));
	}

	return register_devinfo(name, info);
}
EXPORT_SYMBOL(register_device_proc);

static int init_aboard_info(struct device_info *dev_info)
{
	struct manufacture_info *info = NULL;

	info = (struct manufacture_info *) kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}
	dev_msg("begin to create audio_mainboard proc\n");
	return register_devinfo("audio_mainboard", info);
}

static int devinfo_probe(struct platform_device *pdev)
{
	struct device_info *dev_info;

	dev_info = kzalloc(sizeof(struct device_info), GFP_KERNEL);
	if (!dev_info) {
		dev_msg("failed to alloc memory\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&dev_info->dev_list);

	g_dev_info = dev_info;
	g_dev_info->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev_info);

	init_aboard_info(g_dev_info);

	return 0;
}

static int
devinfo_remove(struct platform_device *dev)
{
	if (g_parent) {
		remove_proc_entry("ufsplus_status", g_parent);
	}
	remove_proc_entry(DEVINFO_NAME, NULL);
	return 0;
}

static struct of_device_id devinfo_id[] = {
	{.compatible = "oplus-devinfo", },
};

static struct platform_driver devinfo_platform_driver = {
	.probe = devinfo_probe,
	.remove = devinfo_remove,
	.driver = {
		.name = DEVINFO_NAME,
		.of_match_table = devinfo_id,
	},
};

static int __init
device_info_init(void)
{
	g_parent = proc_mkdir("devinfo", NULL);

	if (!g_parent) {
		return -ENODEV;
	}

	return platform_driver_register(&devinfo_platform_driver);
}

device_initcall(device_info_init);

MODULE_LICENSE("GPL v2");
