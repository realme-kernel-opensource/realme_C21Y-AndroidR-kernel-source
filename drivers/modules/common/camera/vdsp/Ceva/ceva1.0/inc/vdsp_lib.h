
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

#ifndef _VDSP_LIB_H_
#define _VDSP_LIB_H_

#include <linux/list.h>

void vdsp_printk(unsigned int category,
                const char *format, ...);
#define VDSP_UT_NONE              0x00 
#define VDSP_UT_ERROR             0x01 
#define VDSP_UT_INFO              0x02
#define VDSP_UT_DEBUG             0x04

#define VDSP_DEBUG(fmt, ...)                                             \
        vdsp_printk(VDSP_UT_DEBUG, fmt, ##__VA_ARGS__)

#define VDSP_INFO(fmt, ...)                                             \
        vdsp_printk(VDSP_UT_INFO, fmt, ##__VA_ARGS__)


#define VDSP_ERROR(fmt, ...)                                             \
        vdsp_printk(VDSP_UT_ERROR, fmt,  ##__VA_ARGS__)



struct ops_entry {
	const char *ver;
	void *ops;
};

struct ops_list {
	struct list_head head;
	struct ops_entry *entry;
};

void *vdsp_ops_attach(const char *str, struct list_head *head);
int vdsp_ops_register(struct ops_entry *entry, struct list_head *head);

#endif
