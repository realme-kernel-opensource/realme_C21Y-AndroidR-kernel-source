/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * Authors:
 * Keguang Zhang <keguang.zhang@spreadtrum.com>
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

#ifndef __SPRDWL_INTF_H__
#define __SPRDWL_INTF_H__

#include "sprdwl.h"
#include "msg.h"
//#if defined(SPRDWL_INTF_SDIO)
//#include "sdio.h"
//#elif defined(SPRDWL_INTF_SIPC)
#include "sipc.h"
//#endif

struct sprdwl_priv;

struct sprdwl_if_ops {
	struct sprdwl_msg_buf *(*get_msg_buf)(struct sprdwl_intf *intf,
					      enum sprdwl_head_type type,
					      enum sprdwl_mode mode);
	void (*free_msg_buf)(struct sprdwl_intf *intf, struct sprdwl_msg_buf *msg);
	int (*tx)(struct sprdwl_intf *intf, struct sprdwl_msg_buf *msg);
	void (*force_exit)(void);
	int (*is_exit)(void);
	void (*debugfs)(struct dentry *dir);
	void (*tcp_drop_msg)(struct sprdwl_msg_buf *msg);
	void (*set_qos)(enum sprdwl_mode mode,
			int enable);
	int (*reserv_len)(void);
};

struct sprdwl_intf *sprdwl_intf_create(void);
void sprdwl_intf_free(struct sprdwl_intf *intf);
int sprdwl_intf_init(struct sprdwl_intf *intf,
				struct platform_device *pdev,
				struct sprdwl_priv *priv);
void sprdwl_intf_predeinit(struct sprdwl_intf *intf);
void sprdwl_intf_deinit(struct sprdwl_intf *intf);

#endif/*__INTF_H__*/
