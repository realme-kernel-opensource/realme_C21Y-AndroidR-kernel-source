/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

#include <sprd_mm.h>
#include "sprd_img.h"
#include <sprd_fd.h>

#include "cam_types.h"
#include "cam_buf.h"
#include "fd_drv.h"
#include "fd_core.h"

#define FD_DEVICE_NAME "sprd_fd"


static int fd_ioctl_write(struct fd_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_fd_cfg_param  cfg_param;

	mutex_lock(&module->mod_lock);
	ret = copy_from_user(&cfg_param, (struct sprd_fd_cfg_param *)arg,
			sizeof(struct sprd_fd_cfg_param));

	if (ret) {
		pr_err("FD_ERR: ioctl write copy user\n");
		mutex_unlock(&module->mod_lock);
		return ret;
	}
	ret = fd_drv_reg_write_handler(module->drv_handle, &cfg_param);
	mutex_unlock(&module->mod_lock);
	return ret;
}


static int fd_ioctl_multi_write(struct fd_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_fd_multi_reg_cfg_param  param;

	mutex_lock(&module->mod_lock);
	ret = copy_from_user(&param, (struct sprd_fd_multi_reg_cfg_param *)arg,
			    sizeof(struct sprd_fd_multi_reg_cfg_param));
	if (ret) {
		pr_err("FD_ERR: ioctl multi write copy user\n");
		mutex_unlock(&module->mod_lock);
		return ret;
	}
	ret = fd_drv_reg_multi_write_handler(module->drv_handle, &param);
	mutex_unlock(&module->mod_lock);
	return ret;
}

static int fd_ioctl_read(struct fd_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_fd_cfg_param  cfg_param;

	mutex_lock(&module->mod_lock);
	ret = copy_from_user(&cfg_param,
		(struct sprd_fd_cfg_param * __user)arg,
		    sizeof(struct sprd_fd_cfg_param));
	if (ret) {
		pr_err("FD_ERR: ioctl read copy user\n");
		mutex_unlock(&module->mod_lock);
		return ret;
	}
	ret = fd_drv_reg_read_handler(module->drv_handle, arg);
	mutex_unlock(&module->mod_lock);
	return ret;
}

static int fd_ioctl_multi_read(struct fd_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_fd_multi_reg_cfg_param  param;

	mutex_lock(&module->mod_lock);
	ret = copy_from_user(&param,
			    (struct sprd_fd_multi_reg_cfg_param *)arg,
			    sizeof(struct sprd_fd_multi_reg_cfg_param));

	if (ret) {
		pr_err("FD_ERR: ioctl multi read copy user\n");
		mutex_unlock(&module->mod_lock);
		return ret;
	}
	ret = fd_drv_reg_multi_read_handler(module->drv_handle, &param);
	mutex_unlock(&module->mod_lock);
	return ret;
}

static int fd_ioctl_dvfs_work_clk_cfg(struct fd_module *module,
					unsigned long arg)
{
	int ret = 0;
	unsigned int index = 0;

	mutex_lock(&module->mod_lock);
	ret = copy_from_user(&index, (unsigned int *)arg,
			    sizeof(unsigned int));
	if (ret) {
		pr_err("FD_ERR: ioctl dvfs workd clk copy user\n");
		mutex_unlock(&module->mod_lock);
		return ret;
	}
	pr_debug("FD_core, dvfs work clk index %d", index);
	ret = fd_drv_dvfs_work_clk_cfg(module->drv_handle, index);
	mutex_unlock(&module->mod_lock);
	return ret;
}

static int fd_ioctl_dvfs_idle_clk_cfg(struct fd_module *module,
					unsigned long arg)
{
	int ret = 0;
	unsigned int index = 0;

	mutex_lock(&module->mod_lock);
	ret = copy_from_user(&index, (unsigned int *)arg,
			    sizeof(unsigned int));

	if (ret) {
		pr_err("FD_ERR: ioctl dvfs idle clk copy user\n");
		mutex_unlock(&module->mod_lock);
		return ret;
	}
	pr_debug("FD_core, dvfs idle clk index %d", index);
	ret = fd_drv_dvfs_idle_clk_cfg(module->drv_handle, index);
	mutex_unlock(&module->mod_lock);
	return ret;
}

static int fd_ioctl_get_iommu_status(struct fd_module *module,
					unsigned long arg)
{
	int ret = 0;
	unsigned int status;

	mutex_lock(&module->mod_lock);
	if (cam_buf_iommu_status_get(CAM_IOMMUDEV_FD) ==  0)
		status = SPRD_FD_IOMMU_ENABLED;
	else
		status = SPRD_FD_IOMMU_DISABLED;

	pr_debug("FD_core, iommu enable %d\n", status);

	ret = copy_to_user((unsigned int  __user *)arg,	&status,
				sizeof(unsigned int)
				);
	mutex_unlock(&module->mod_lock);
	return ret;
}
static struct fd_ioctl_cmd fd_ioctl_cmds_table[] = {

	[_IOC_NR(SPRD_FD_IO_WRITE)] = {SPRD_FD_IO_WRITE,
						fd_ioctl_write},
	[_IOC_NR(SPRD_FD_IO_WRITE_WITHBIT)] = {SPRD_FD_IO_WRITE_WITHBIT,
						fd_ioctl_write},
	[_IOC_NR(SPRD_FD_IO_MULTI_WRITE)] = {SPRD_FD_IO_MULTI_WRITE,
						fd_ioctl_multi_write},
	[_IOC_NR(SPRD_FD_IO_READ)] = {SPRD_FD_IO_READ,
						fd_ioctl_read},
	[_IOC_NR(SPRD_FD_IO_MULTI_READ)] = {SPRD_FD_IO_MULTI_READ,
						fd_ioctl_multi_read},
	[_IOC_NR(SPRD_FD_IO_WORK_CLOCK_SEL)] = {SPRD_FD_IO_WORK_CLOCK_SEL,
						fd_ioctl_dvfs_work_clk_cfg},
	[_IOC_NR(SPRD_FD_IO_IDLE_CLOCK_SEL)] = {SPRD_FD_IO_IDLE_CLOCK_SEL,
						fd_ioctl_dvfs_idle_clk_cfg},
	[_IOC_NR(SPRD_FD_IO_GET_IOMMU_STATUS)] = {SPRD_FD_IO_GET_IOMMU_STATUS,
						fd_ioctl_get_iommu_status},
};

static int sprd_fd_open(struct inode *node, struct file *file)
{
	int ret = 0;
	struct fd_module *module = NULL;
	struct miscdevice *md = file->private_data;

	pr_info("%s start\n", __func__);

	module  = md->this_device->platform_data;
	if (module == NULL) {
		pr_err("FD_CORE err fd open\n");
		return -EINVAL;
	}

	mutex_lock(&module->mod_lock);
	ret = sprd_fd_drv_open(module->drv_handle);
	mutex_unlock(&module->mod_lock);
	pr_info("%s end ret %d\n", __func__, ret);
	return ret;
}

static int sprd_fd_release(struct inode *node, struct file *file)
{
	int ret = 0;
	struct fd_module *module = NULL;
	struct miscdevice *md = file->private_data;


	pr_info("%s start\n", __func__);
	module  = md->this_device->platform_data;
	mutex_lock(&module->mod_lock);
	ret = sprd_fd_drv_close(module->drv_handle);
	mutex_unlock(&module->mod_lock);
	pr_info("%s end ret %d\n", __func__, ret);

	return ret;
}

#ifdef CONFIG_COMPAT
static long compat_sprd_fd_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	long ret = 0L;

	ret = file->f_op->unlocked_ioctl(file, cmd, arg);
	return ret;
}
#endif


static long sprd_fd_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	int ret = 0;
	struct fd_module *module = NULL;
	struct fd_ioctl_cmd *p_ioctl = NULL;
	int ioctl_nr = _IOC_NR(cmd);
	struct miscdevice *md = file->private_data;

	pr_debug("%s start %d\n", __func__, ioctl_nr);
	module  = md->this_device->platform_data;

	if (unlikely(!(ioctl_nr >= 0 && ioctl_nr < ARRAY_SIZE(fd_ioctl_cmds_table)))) {
		pr_info("invalid cmd: 0x%xn", cmd);
		return -EINVAL;
	}
	p_ioctl = &fd_ioctl_cmds_table[ioctl_nr];
	if (!p_ioctl->cmd_proc) {
		pr_err("SPRD_FD cmd function not defined %d", ioctl_nr);
		return -EINVAL;
	}
	ret  = p_ioctl->cmd_proc(module, arg);
	if (ret)
		pr_err("%s err for cmd %d ret %d\n", __func__, ioctl_nr, ret);
	pr_debug("%s end\n", __func__);

	return ret;
}

static const struct file_operations fd_fops = {
	.open = sprd_fd_open,
	.unlocked_ioctl = sprd_fd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_sprd_fd_ioctl,
#endif
	.release = sprd_fd_release,
};

static struct miscdevice fd_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FD_DEVICE_NAME,
	.fops = &fd_fops,
};

static int sprd_fd_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fd_module *module = NULL;

	pr_info("%s start\n", __func__);

	module = kzalloc(sizeof(struct fd_module), GFP_KERNEL);
	if (module == NULL) {
		pr_err("FD_CORE err, probe no memory for module\n");
		return -ENOMEM;
	}
	ret = misc_register(&fd_dev);
	if (ret) {
		pr_err("FD_CORE err, register misc driver ret %d\n", ret);
		kfree(module);
		return -EINVAL;
	}
	fd_dev.this_device->of_node = pdev->dev.of_node;
	fd_dev.this_device->platform_data = (void *)module;
	module->md = &fd_dev;
	module->pdev = pdev;
	ret = sprd_fd_drv_init(&module->drv_handle, pdev->dev.of_node);
	if (ret) {
		pr_err("FD_CORE err, driver init failed ret %d\n", ret);
		misc_deregister(&fd_dev);
		kfree(module);
		return -EINVAL;
	}
	mutex_init(&module->mod_lock);
	pr_info("%s end ret %d\n", __func__, ret);

	return ret;
}

static int sprd_fd_remove(struct platform_device *pdev)
{
	struct fd_module *module = NULL;
	int ret = 0;

	module = (struct fd_module *)fd_dev.this_device->platform_data;

	if (module == NULL) {
		pr_err("FD_CORE err remove\n");
		return -EINVAL;
	}
	ret = sprd_fd_drv_deinit(module->drv_handle);

	if (ret) {
		pr_err("FD_CORE err, removing driver deinit\n");
		return ret;
	}
	module->drv_handle = NULL;
	misc_deregister(&fd_dev);
	kfree(module);

	return 0;
}

static const struct of_device_id sprd_fd_of_match[] = {
	{ .compatible = "sprd,fd", },
	{},
};

static struct platform_driver sprd_fd_driver = {
	.probe = sprd_fd_probe,
	.remove = sprd_fd_remove,
	.driver = {
		.name = FD_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_fd_of_match),
	},
};

module_platform_driver(sprd_fd_driver);

MODULE_DESCRIPTION("Sprd Face Detect Driver");
MODULE_AUTHOR("Multimedia_Camera@Spreadtrum");
MODULE_LICENSE("GPL");

