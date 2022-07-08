/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#define pr_fmt(__fmt) "[sprd-vdsp][%20s] "__fmt, __func__

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "vdsp_lib.h"

unsigned int vdsp_debug = 7;
module_param_named(debug, vdsp_debug, int, 0600);


void vdsp_printk(unsigned int category,
                const char *format, ...)
{
        struct va_format vaf;
        va_list args;

        if (category != VDSP_UT_NONE && !(vdsp_debug & category))
                return;

        va_start(args, format);
        vaf.fmt = format;
        vaf.va = &args;

        printk(" [vdsp:%ps] %pV",__builtin_return_address(0), &vaf);

        va_end(args);
}
EXPORT_SYMBOL(vdsp_printk);

void *vdsp_ops_attach(const char *str, struct list_head *head)
{
	struct ops_list *list;
	const char *ver;

	list_for_each_entry(list, head, head) {
		ver = list->entry->ver;
		if (!strcmp(str, ver)){
			VDSP_INFO("attach vdsp ops %s success\n", str);
			return list->entry->ops;
		}
	}

	VDSP_ERROR("attach vdsp ops %s failed\n", str);
	return NULL;
}
EXPORT_SYMBOL_GPL(vdsp_ops_attach);

int vdsp_ops_register(struct ops_entry *entry, struct list_head *head)
{
	struct ops_list *list;

	list = kzalloc(sizeof(struct ops_list), GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	list->entry = entry;
	list_add(&list->head, head);
	VDSP_INFO("register vdsp ops %s\n", entry->ver);

	return 0;
}
EXPORT_SYMBOL_GPL(vdsp_ops_register);

