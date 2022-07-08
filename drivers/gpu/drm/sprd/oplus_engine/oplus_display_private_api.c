#include <drm/drmP.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include "oplus_display_private_api.h"

static bool support_mipi_cmd = true;
unsigned long lcd_id;
static int __init lcd_id_get(char *str)
{
	int ret;
	if (str != NULL)
		ret = kstrtoul(str, 16, &lcd_id);

	DRM_INFO("lcd_id from uboot: %u, ret: %d\n", lcd_id, ret);
	return 0;
}
__setup("lcd_id=ID", lcd_id_get);

static ssize_t lcd_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	if (support_mipi_cmd) {
		ret = scnprintf(buf, PAGE_SIZE, "LCM ID[%x]: 0x%x 0x%x\n", 0, lcd_id, 0);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "LCM ID[00]: 0x00 0x00\n");
	}

	return ret;
}

static ssize_t lcd_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	return num;
}
static DEVICE_ATTR(panel_id, S_IRUGO|S_IWUSR, lcd_id_show, lcd_id_store);


static struct attribute *oplus_display_attrs[] = {
	&dev_attr_panel_id.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oplus_display_attr_group = {
	.attrs = oplus_display_attrs,
};

static struct kobject *oplus_display_kobj;
int oplus_display_private_api_init(void)
{
	static struct kobject *oplus_display_kobj;
	int retval;

	oplus_display_kobj = kobject_create_and_add("oplus_display", kernel_kobj);
	if (!oplus_display_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(oplus_display_kobj, &oplus_display_attr_group);
	if (retval)
		kobject_put(oplus_display_kobj);

	return retval;
}

void oplus_display_private_api_exit(void)
{
	kobject_put(oplus_display_kobj);
}