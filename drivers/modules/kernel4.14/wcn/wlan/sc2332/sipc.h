/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Authors	:
 * Keguang Zhang <keguang.zhang@spreadtrum.com>
 * Jingxiang Li <Jingxiang.li@spreadtrum.com>
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

#ifndef __SPRDWL_SIPC_H__
#define __SPRDWL_SIPC_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ieee80211.h>
#include <linux/slab.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include "intf.h"
#include "sprdwl.h"

/*FIXME*/
#define WLAN_CP_ID			3
#define SPRDWL_SWCNBLK_CMD_EVENT	7
#define SPRDWL_SWCNBLK_DATA0		8
#define SPRDWL_SWCNBLK_DATA1		9

#define SPRDWL_SIPC_HEAD_RESERV	32

/* tx len less than cp len 4 byte as sdiom 4 bytes align */
#define SPRDWL_MAX_CMD_TXLEN	1396
#define SPRDWL_MAX_CMD_RXLEN	1088
#define SPRDWL_MAX_DATA_TXLEN	1624
#define SPRDWL_MAX_DATA_RXLEN	1592

struct sprdwl_priv;
struct sprdwl_if_ops;

struct sprdwl_intf {
	struct platform_device *pdev;
	/* priv use void *, after MCC adn priv->flags,
	 * and change txrx intf pass priv to void later
	 */
	struct sprdwl_priv *priv;

	/* if nedd more flags which not only exit, fix it*/
	/* unsigned int exit:1; */
	int exit;

	unsigned long cmd_timeout;
	unsigned long data_timeout;
	/* lock for do_tx */
	spinlock_t lock;
	unsigned long do_tx;
	wait_queue_head_t waitq;
	unsigned int net_stop_cnt;
	unsigned int net_start_cnt;
	unsigned int drop_cmd_cnt;
	/* sta */
	unsigned int drop_data1_cnt;
	/* p2p */
	unsigned int drop_data2_cnt;

	/* 1 for send data; 0 for not send data */
	int driver_status;

	/* list not included in sprdwl_vif */
	/* just as our driver support less port */
	/* for cmd */
	struct sprdwl_msg_list tx_list0;
	/* for STA/SOFTAP data */
	struct sprdwl_msg_list tx_list1;
	/* for P2P data */
	struct sprdwl_msg_list tx_list2;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	struct wakeup_source tx_wakelock;
#else
	struct wake_lock tx_wakelock;
#endif
	/* off screen tx may into deepsleep */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	struct wakeup_source keep_wake;
#else
	struct wake_lock keep_wake;
#endif
	unsigned long wake_last_time;
	unsigned long wake_timeout;
	unsigned long wake_pre_timeout;

	struct work_struct tx_work;
	struct workqueue_struct *tx_queue;

	struct sprdwl_if_ops *if_ops;
};

#endif/*__SPRDWL_SIPC_H__*/
