/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * Author: Haibing.Yang <haibing.yang@spreadtrum.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
struct class *vdsp_class;
extern int32_t vdsp_wait_xm6_done(void);
int vdsp_class_init(void)
{
	vdsp_class = class_create(THIS_MODULE, "vdsp");
	if (IS_ERR(vdsp_class)) {
		pr_err("Unable to create vdsp class\n");
		return PTR_ERR(vdsp_class);
	}

	return 0;
}

//postcore_initcall(vdsp_class_init);


static DEFINE_MUTEX(xm6_state_lock);
int xm6_state_changed = 0;
static ssize_t xm6_state_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int ret = 0;
	vdsp_wait_xm6_done();
	mutex_lock(&xm6_state_lock);
	ret = sprintf(buf,
                "%d\n",
                xm6_state_changed);
	xm6_state_changed = 0;
	mutex_unlock(&xm6_state_lock);
	return ret;
}
extern int xm6_close(void);
extern int xm6_open(void);
static ssize_t xm6_state_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	int enable;

	ret = kstrtoint(buf, 10, &enable);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}
	return count;
}



static DEVICE_ATTR_RW(xm6_state);

static struct attribute *vdsp_attrs[] = {
	&dev_attr_xm6_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(vdsp);

int sprd_vdsp_sysfs_init(struct device *dev)
{
	int rc;

	rc = sysfs_create_groups(&dev->kobj, vdsp_groups);
	if (rc)
		pr_err("create dsi attr node failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(sprd_vdsp_sysfs_init);

