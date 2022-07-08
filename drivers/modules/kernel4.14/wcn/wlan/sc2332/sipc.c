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

#include <linux/platform_device.h>
#include <linux/utsname.h>
#include <linux/sipc.h>
#include <linux/debugfs.h>
#include <linux/swcnblk.h>
#include <misc/wcn_integrate_platform.h>

#include "sprdwl.h"
#include "sipc.h"
#include "msg.h"
#include "txrx.h"
#include "qos.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "sprdwl sipc:%s " fmt, __func__

#define SPRDWL_TX_CMD_TIMEOUT	3000
#define SPRDWL_TX_DATA_TIMEOUT	4000
#define SPRDWL_WAKE_TIMEOUT	240
#define SPRDWL_WAKE_PRE_TIMEOUT	80

enum sprdwl_sipc_port {
	SPRDWL_SIPC_CMD_TX,
	SPRDWL_SIPC_CMD_RX = 0,
	SPRDWL_SIPC_DATA_TX = 1,
	SPRDWL_SIPC_DATA_RX = 1,
};

#define SPRDWL_TX_MSG_CMD_NUM 4
#define SPRDWL_TX_MSG_DATA_NUM 128
#define SPRDWL_TX_DATA_START_NUM (SPRDWL_TX_MSG_DATA_NUM - 3)

#define SPRDWL_SIPC_MASK_LIST0	0x1
#define SPRDWL_SIPC_MASK_LIST1	0x2
#define SPRDWL_SIPC_MASK_LIST2	0x4
/* sipc reserv more len than Mac80211 HEAD */
#define SPRDWL_SIPC_RESERV_LEN	32

static struct sprdwl_intf *p_sprdwl_intf;

static void sprdwl_sipc_wakeup_tx(void);

static const char *sprdwl_sipc_channel_tostr(int channel)
{
	switch (channel) {
	case SPRDWL_SWCNBLK_CMD_EVENT:
		return "cmd";
	case SPRDWL_SWCNBLK_DATA0:
		return "data0";
	case SPRDWL_SWCNBLK_DATA1:
		return "data1";
	default:
		return "unknown channel";
	}
}

static void sprdwl_sipc_wake_net_ifneed(struct sprdwl_intf *intf,
					struct sprdwl_msg_list *list,
					enum sprdwl_mode mode)
{
	if (atomic_read(&list->flow)) {
		if (atomic_read(&list->ref) <= SPRDWL_TX_DATA_START_NUM) {
			atomic_set(&list->flow, 0);
			intf->net_start_cnt++;
			if (intf->driver_status)
				sprdwl_net_flowcontrl(intf->priv, mode, true);
		}
	}
}

static int sprdwl_sipc_txmsg(struct sprdwl_intf *intf,
				struct sprdwl_msg_buf *msg)
{
	u16 len;
	unsigned char *info;

	if (msg->msglist == &intf->tx_list0) {
		len = SPRDWL_MAX_CMD_TXLEN;
		info = "cmd";
		msg->timeout = jiffies + intf->cmd_timeout;
	} else {
		len = SPRDWL_MAX_DATA_TXLEN;
		info = "data";
		msg->timeout = jiffies + intf->data_timeout;
	}

	if (msg->len > len) {
		dev_err(&intf->pdev->dev,
			"%s err:%s too long:%d > %d,drop it\n",
			__func__, info, msg->len, len);
		WARN_ON(1);
		dev_kfree_skb(msg->skb);
		sprdwl_free_msg_buf(msg, msg->msglist);
		return 0;
	}

	sprdwl_queue_msg_buf(msg, msg->msglist);
	sprdwl_sipc_wakeup_tx();
	queue_work(intf->tx_queue, &intf->tx_work);

	return 0;
}

static struct
sprdwl_msg_buf *sprdwl_sipc_get_msg_buf(struct sprdwl_intf *intf,
					enum sprdwl_head_type type,
					enum sprdwl_mode mode)
{
	u8 sipc_type;
	struct sprdwl_msg_buf *msg;
	struct sprdwl_msg_list *list;

	if (unlikely(intf->exit))
		return NULL;
	if (type == SPRDWL_TYPE_DATA) {
		sipc_type = SPRDWL_SIPC_DATA_TX;
		if (mode <= SPRDWL_MODE_AP)
			list = &intf->tx_list1;
		else
			list = &intf->tx_list2;
	} else {
		sipc_type = SPRDWL_SIPC_CMD_TX;
		list = &intf->tx_list0;
	}
	msg = sprdwl_alloc_msg_buf(list);
	if (msg) {
		msg->type = sipc_type;
		msg->msglist = list;
		msg->mode = mode;
		return msg;
	}

	if (type == SPRDWL_TYPE_DATA) {
		intf->net_stop_cnt++;
		sprdwl_net_flowcontrl(intf->priv, mode, false);
		atomic_set(&list->flow, 1);
	}
	printk_ratelimited("%s no more msgbuf for %s\n",
			   __func__, type == SPRDWL_TYPE_DATA ?
			   "data" : "cmd");

	return NULL;
}

static void sprdwl_sipc_free_msg_buf(struct sprdwl_intf *intf,
				struct sprdwl_msg_buf *msg)
{
	sprdwl_free_msg_buf(msg, msg->msglist);
}

static void sprdwl_sipc_force_exit(void)
{
	p_sprdwl_intf->exit = 1;
	wake_up_all(&p_sprdwl_intf->waitq);
}

static int sprdwl_sipc_is_exit(void)
{
	return p_sprdwl_intf->exit;
}

#define SPRDWL_SIPC_DEBUG_BUFLEN 128
static ssize_t sprdwl_sipc_read_info(struct file *file,
				     char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	unsigned int buflen, len;
	size_t ret;
	unsigned char *buf;
	struct sprdwl_intf *intf;

	intf = (struct sprdwl_intf *)file->private_data;
	buflen = SPRDWL_SIPC_DEBUG_BUFLEN;
	buf = kzalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len = 0;
	len += scnprintf(buf, buflen, "net: stop %u, start %u\n"
			 "drop cnt: cmd %u, sta %u, p2p %u\n",
			 intf->net_stop_cnt, intf->net_start_cnt,
			 intf->drop_cmd_cnt, intf->drop_data1_cnt,
			 intf->drop_data2_cnt);
	if (len > buflen)
		len = buflen;

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret;
}

static ssize_t sprdwl_sipc_write(struct file *file,
				 const char __user *__user_buf,
				 size_t count, loff_t *ppos)
{
	char buf[20];

	if (!count || count >= sizeof(buf)) {
		dev_err(&p_sprdwl_intf->pdev->dev,
			"write len too long:%zu >= %u\n", count,
			(unsigned int)sizeof(buf));
		return -EINVAL;
	}
	if (copy_from_user(buf, __user_buf, count))
		return -EFAULT;
	buf[count] = '\0';
	dev_err(&p_sprdwl_intf->pdev->dev, "write info:%s\n", buf);

	return count;
}

static const struct file_operations sprdwl_sipc_debug_fops = {
	.read = sprdwl_sipc_read_info,
	.write = sprdwl_sipc_write,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static void sprdwl_sipc_debugfs(struct dentry *dir)
{
	debugfs_create_file("sipcinfo", S_IRUSR,
			    dir, p_sprdwl_intf, &sprdwl_sipc_debug_fops);
}

static void sprdwl_sipc_tcp_drop_msg(struct sprdwl_msg_buf *msgbuf)
{
	enum sprdwl_mode mode;
	struct sprdwl_msg_list *list;

	dev_kfree_skb(msgbuf->skb);
	mode = msgbuf->mode;
	list = msgbuf->msglist;
	sprdwl_free_msg_buf(msgbuf, list);
	sprdwl_sipc_wake_net_ifneed(p_sprdwl_intf, list, mode);
}

static void sprdwl_sipc_set_qos(enum sprdwl_mode mode, int enable)
{
	struct sprdwl_msg_list *list;

	if (mode <= SPRDWL_MODE_AP)
		list = &p_sprdwl_intf->tx_list1;
	else
		list = &p_sprdwl_intf->tx_list2;
	dev_info(&p_sprdwl_intf->pdev->dev, "%s set qos:%d\n",
		 __func__, enable);
	list->qos.enable = enable ? 1 : 0;
	list->qos.change = 1;
}

static int sprdwl_sipc_reserv_len(void)
{
	return SPRDWL_SIPC_RESERV_LEN;
}

struct sprdwl_if_ops sprdwl_sipc_ops = {
	.get_msg_buf = sprdwl_sipc_get_msg_buf,
	.free_msg_buf = sprdwl_sipc_free_msg_buf,
	.tx = sprdwl_sipc_txmsg,
	.force_exit = sprdwl_sipc_force_exit,
	.is_exit = sprdwl_sipc_is_exit,
	.debugfs = sprdwl_sipc_debugfs,
	.tcp_drop_msg = sprdwl_sipc_tcp_drop_msg,
	.set_qos = sprdwl_sipc_set_qos,
	.reserv_len = sprdwl_sipc_reserv_len,
};

static void sprdwl_rx_process(unsigned char *data, unsigned int len)
{
	struct sprdwl_priv *priv;
	struct sprdwl_intf *intf;

	intf = p_sprdwl_intf;
	priv = intf->priv;

	switch (SPRDWL_HEAD_GET_TYPE(data)) {
	case SPRDWL_TYPE_DATA:
		if (len > SPRDWL_MAX_DATA_RXLEN)
			dev_err(&intf->pdev->dev,
				"err rx data too long:%d > %d\n",
				len, SPRDWL_MAX_DATA_RXLEN);
		sprdwl_rx_data_process(priv, data);
		break;
	case SPRDWL_TYPE_CMD:
		if (len > SPRDWL_MAX_CMD_RXLEN)
			dev_err(&intf->pdev->dev,
				"err rx cmd too long:%d > %d\n",
				len, SPRDWL_MAX_CMD_RXLEN);
		sprdwl_rx_rsp_process(priv, data);
		break;
	case SPRDWL_TYPE_EVENT:
		if (len > SPRDWL_MAX_CMD_RXLEN)
			dev_err(&intf->pdev->dev,
				"err rx event too long:%d > %d\n",
				len, SPRDWL_MAX_CMD_RXLEN);
		sprdwl_rx_event_process(priv, data);
		break;
	default:
		dev_err(&intf->pdev->dev, "rx unkonow type:%d\n",
			SPRDWL_HEAD_GET_TYPE(data));
			break;
	}
}

static int sprdwl_sipc_msg_send(void *xmit_data, u16 xmit_len, int channel)
{
	int ret;
	struct swcnblk_blk blk;
	u8 *addr = NULL;

	/* Get a free swcnblk. */
	ret = swcnblk_get(WLAN_CP_ID, channel, &blk, 0);
	if (ret) {
		wl_err("%s Failed to get free swcnblk(%d)!\n",
		       sprdwl_sipc_channel_tostr(channel), ret);
		return -ENOMEM;
	}

	if (blk.length < xmit_len) {
		wl_err("%s The size of swcnblk is so tiny!\n",
		       sprdwl_sipc_channel_tostr(channel));
		swcnblk_put(WLAN_CP_ID, channel, &blk);
		WARN_ON(1);
		return 0;
	}

	addr = (u8 *)blk.addr + SPRDWL_SIPC_HEAD_RESERV;
	blk.length = xmit_len + SPRDWL_SIPC_HEAD_RESERV;
	memcpy(((u8 *)addr), xmit_data, xmit_len);
	ret = swcnblk_send_prepare(WLAN_CP_ID, channel, &blk);
	if (ret) {
		wl_err("%s err:%d\n",
		       sprdwl_sipc_channel_tostr(channel), ret);
		swcnblk_put(WLAN_CP_ID, channel, &blk);
	}

	return ret;
}

static int sprdwl_tx_cmd(struct sprdwl_intf *intf, struct sprdwl_msg_list *list)
{
	struct sprdwl_msg_buf *msgbuf;

	while ((msgbuf = sprdwl_peek_msg_buf(list))) {
		if (unlikely(intf->exit)) {
			dev_kfree_skb(msgbuf->skb);
			sprdwl_dequeue_msg_buf(msgbuf, list);
			continue;
		}
		if (time_after(jiffies, msgbuf->timeout)) {
			intf->drop_cmd_cnt++;
			dev_err(&intf->pdev->dev,
				"tx drop cmd msg,dropcnt:%u\n",
				intf->drop_cmd_cnt);
			dev_kfree_skb(msgbuf->skb);
			sprdwl_dequeue_msg_buf(msgbuf, list);
			continue;
		}

		sprdwl_sipc_msg_send(msgbuf->tran_data, msgbuf->len,
				     SPRDWL_SWCNBLK_CMD_EVENT);
		dev_kfree_skb(msgbuf->skb);
		sprdwl_dequeue_msg_buf(msgbuf, list);
	}

	return 0;
}

static int sprdwl_tx_data(struct sprdwl_intf *intf,
			  struct sprdwl_msg_list *list,
			  int num, int channel)
{
	unsigned char mode;
	int ret, pkts;
	unsigned int cnt;
	struct sprdwl_msg_buf *msgbuf;

	pkts = -1;
	sprdwl_qos_reorder(&list->qos);
	while ((msgbuf = sprdwl_qos_peek_msg(&list->qos, &pkts))) {
		if (unlikely(intf->exit)) {
			dev_kfree_skb(msgbuf->skb);
			sprdwl_qos_update(&list->qos, msgbuf, &msgbuf->list);
			sprdwl_dequeue_msg_buf(msgbuf, list);
			sprdwl_need_resch(&list->qos);
			continue;
		}
		if (time_after(jiffies, msgbuf->timeout)) {
			char *pinfo;

			if (list == &intf->tx_list1) {
				pinfo = "data1";
				cnt = intf->drop_data1_cnt++;
			} else {
				pinfo = "data2";
				cnt = intf->drop_data2_cnt++;
			}
			dev_err(&intf->pdev->dev,
				"tx drop %s msg,dropcnt:%u\n", pinfo, cnt);
			dev_kfree_skb(msgbuf->skb);
			mode = msgbuf->mode;
			sprdwl_qos_update(&list->qos, msgbuf, &msgbuf->list);
			sprdwl_dequeue_msg_buf(msgbuf, list);
			sprdwl_need_resch(&list->qos);
			sprdwl_sipc_wake_net_ifneed(intf, list, mode);
			continue;
		}
		if (num-- == 0)
			break;

		ret = sprdwl_sipc_msg_send(msgbuf->tran_data,
					   msgbuf->len, channel);
		if (!ret) {
			dev_kfree_skb(msgbuf->skb);
			mode = msgbuf->mode;
			sprdwl_qos_update(&list->qos, msgbuf, &msgbuf->list);
			sprdwl_dequeue_msg_buf(msgbuf, list);
			sprdwl_need_resch(&list->qos);
			sprdwl_sipc_wake_net_ifneed(intf, list, mode);
		} else {
			break;
		}
	}

	return 0;
}

static void sprdwl_sipc_flush_txlist(struct sprdwl_msg_list *list)
{
	struct sprdwl_msg_buf *msgbuf;

	while ((msgbuf = sprdwl_peek_msg_buf(list))) {
		dev_kfree_skb(msgbuf->skb);
		sprdwl_dequeue_msg_buf(msgbuf, list);
		continue;
	}
}

static void sprdwl_sipc_flush_all_txlist(struct sprdwl_intf *intf)
{
	sprdwl_sipc_flush_txlist(&intf->tx_list0);
	sprdwl_sipc_flush_txlist(&intf->tx_list1);
	sprdwl_sipc_flush_txlist(&intf->tx_list2);
}

static void sprdwl_sipc_keep_wakeup(struct sprdwl_intf *intf)
{
	if (time_after(jiffies, intf->wake_last_time)) {
		intf->wake_last_time = jiffies + intf->wake_pre_timeout;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		__pm_wakeup_event(&intf->keep_wake, intf->wake_timeout);
#else
		wake_lock_timeout(&intf->keep_wake, intf->wake_timeout);
#endif
	}
}

static void sprdwl_tx_work_queue(struct work_struct *work)
{
	unsigned int send_list, needsleep;
	struct sprdwl_intf *intf;
	int send_num0, send_num1, send_num2;
	int wake_state = 0;

	send_num0 = 0;
	send_num1 = 0;
	send_num2 = 0;
	intf = container_of(work, struct sprdwl_intf, tx_work);
RETRY:
	if (unlikely(intf->exit)) {
		if (wake_state)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			__pm_relax(&intf->tx_wakelock);
#else
			wake_unlock(&intf->tx_wakelock);
#endif

		sprdwl_sipc_flush_all_txlist(intf);
		return;
	}
	send_list = 0;
	needsleep = 0;
	intf->do_tx = 0;
	if (sprdwl_msg_tx_pended(&intf->tx_list0))
		send_list |= SPRDWL_SIPC_MASK_LIST0;
	if (intf->driver_status) {
		if (sprdwl_msg_tx_pended(&intf->tx_list1)) {
			send_num1 =
				swcnblk_get_free_count(WLAN_CP_ID,
						       SPRDWL_SWCNBLK_DATA0);
			if (send_num1 > 0)
				send_list |= SPRDWL_SIPC_MASK_LIST1;
			else
				needsleep |= SPRDWL_SIPC_MASK_LIST1;
		}
		if (sprdwl_msg_tx_pended(&intf->tx_list2)) {
			send_num2 =
				swcnblk_get_free_count(WLAN_CP_ID,
						       SPRDWL_SWCNBLK_DATA1);
			if (send_num2 > 0)
				send_list |= SPRDWL_SIPC_MASK_LIST2;
			else
				needsleep |= SPRDWL_SIPC_MASK_LIST2;
		}
	}

	/*send_list = 0 & needsleep = 0 means tx_list is empty!
	 *send_list = 0 & needsleep != 0 means cp hasn't enough space for new message!
	 */
	if (!send_list) {
		if (!needsleep) {
			if (wake_state)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
				__pm_relax(&intf->tx_wakelock);
#else
				wake_unlock(&intf->tx_wakelock);
#endif
			sprdwl_sipc_keep_wakeup(intf);
			return;
		}
		printk_ratelimited("%s need sleep  -- 0x%x %d %d %d\n",
				   __func__, needsleep, send_num0,
				   send_num1, send_num2);
		if (wake_state) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			__pm_relax(&intf->tx_wakelock);
#else
			wake_unlock(&intf->tx_wakelock);
#endif
			wake_state = 0;
		}
		sprdwl_sipc_keep_wakeup(intf);
		wait_event(intf->waitq,
			   (intf->do_tx || intf->exit));
		goto RETRY;
	}

	/*send list is not empty and cp has more buff, then not permit sleep!
	 */
	if (!wake_state) {
		wake_state = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		__pm_stay_awake(&intf->tx_wakelock);
#else
		wake_lock(&intf->tx_wakelock);
#endif
	}

	if (send_list & SPRDWL_SIPC_MASK_LIST0)
		sprdwl_tx_cmd(intf, &intf->tx_list0);
	if (intf->driver_status) {
		if (send_list & SPRDWL_SIPC_MASK_LIST1)
			sprdwl_tx_data(intf, &intf->tx_list1,
				       send_num1, SPRDWL_SWCNBLK_DATA0);
		if (send_list & SPRDWL_SIPC_MASK_LIST2)
			sprdwl_tx_data(intf, &intf->tx_list2,
				       send_num2, SPRDWL_SWCNBLK_DATA1);
	}

	goto RETRY;
}

static void sprdwl_sipc_wakeup_tx(void)
{
	p_sprdwl_intf->do_tx = 1;
	wake_up(&p_sprdwl_intf->waitq);
}

static int sprdwl_sipc_rx_handle(void *data, unsigned int len)
{
	if (!data || !len) {
		dev_err(&p_sprdwl_intf->pdev->dev,
			"%s param erro:%p %d\n", __func__, data, len);
		return -EFAULT;
	}

	if (unlikely(p_sprdwl_intf->exit))
		return -EFAULT;

	sprdwl_rx_process(data, len);

	return 0;
}

static void sprdwl_sipc_msg_receive(int channel)
{
	struct swcnblk_blk blk;
	u32 length = 0;
	int ret;

	while (!swcnblk_receive(WLAN_CP_ID, channel, &blk, 0)) {
		length = blk.length - SPRDWL_SIPC_HEAD_RESERV;
		sprdwl_sipc_rx_handle((u8 *)blk.addr +
				      SPRDWL_SIPC_HEAD_RESERV, length);
		ret = swcnblk_release(WLAN_CP_ID, channel, &blk);
		if (ret)
			wl_err("release swcnblk[%d] err:%d\n",
			       channel, ret);
	}
}

static void sprdwl_sipc_data0_handler(int event, void *data)
{
	switch (event) {
	case SBLOCK_NOTIFY_RECV:
		sprdwl_sipc_msg_receive(SPRDWL_SWCNBLK_DATA0);
		break;
	case SBLOCK_NOTIFY_GET:
		sprdwl_sipc_wakeup_tx();
		break;
	case SBLOCK_NOTIFY_STATUS:
		break;
	default:
		wl_err("Invalid data0 swcnblk notify:%d\n", event);
		break;
	}
}

static void sprdwl_sipc_data1_handler(int event, void *data)
{
	switch (event) {
	case SBLOCK_NOTIFY_GET:
		sprdwl_sipc_wakeup_tx();
		break;
	case SBLOCK_NOTIFY_RECV:
	case SBLOCK_NOTIFY_STATUS:
		break;
	default:
		wl_err("Invalid data1 swcnblk notify:%d\n", event);
		break;
	}
}

static void sprdwl_sipc_event_handler(int event, void *data)
{
	switch (event) {
	case SBLOCK_NOTIFY_RECV:
		sprdwl_sipc_msg_receive(SPRDWL_SWCNBLK_CMD_EVENT);
		break;
	/* SBLOCK_NOTIFY_GET cmd not need process it */
	case SBLOCK_NOTIFY_GET:
	case SBLOCK_NOTIFY_STATUS:
		break;
	default:
		wl_err("Invalid event swcnblk notify:%d\n", event);
		break;
	}
}

static void sprdwl_sipc_clean_swcnblk(void)
{
	int ret = 0, block_num;
	struct swcnblk_blk blk;

	block_num = 0;
	do {
		ret = swcnblk_receive(WLAN_CP_ID,
				      SPRDWL_SWCNBLK_CMD_EVENT, &blk, 0);
		if (!ret) {
			swcnblk_release(WLAN_CP_ID,
					SPRDWL_SWCNBLK_CMD_EVENT, &blk);
			block_num++;
		}
	} while (!ret);
	if (block_num)
		wl_err("release event rubbish num:%d\n",
		       block_num);

	block_num = 0;
	do {
		ret = swcnblk_receive(WLAN_CP_ID,
				      SPRDWL_SWCNBLK_DATA0, &blk, 0);
		if (!ret) {
			swcnblk_release(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA0, &blk);
			block_num++;
		}
	} while (!ret);
	if (block_num)
		wl_err("release data0 rubbish num:%d\n",
		       block_num);

	block_num = 0;
	do {
		ret = swcnblk_receive(WLAN_CP_ID,
				      SPRDWL_SWCNBLK_DATA1, &blk, 0);
		if (!ret) {
			swcnblk_release(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA1, &blk);
			block_num++;
		}
	} while (!ret);
	if (block_num)
		wl_err("release data1 rubbish num:%d\n",
		       block_num);
}

static int sprdwl_sipc_swcnblk_init(struct sprdwl_intf *intf)
{
	int ret = 0;
	int channel[3];
	int i = 0;
	unsigned long timeout;

	channel[0] = SPRDWL_SWCNBLK_CMD_EVENT;
	channel[1] = SPRDWL_SWCNBLK_DATA0;
	channel[2] = SPRDWL_SWCNBLK_DATA1;
	timeout = jiffies + msecs_to_jiffies(1000);
	while (time_before(jiffies, timeout)) {
		ret = swcnblk_query(WLAN_CP_ID, channel[i]);
		if (ret) {
			usleep_range(8000, 10000);
			continue;
		} else {
			i++;
			if (i == 3)
				break;
		}
	}
	if (i != 3) {
		wl_err("cp swcnblk not ready (%d %d)!\n",
		       i, ret);
		return ret;
	}
	sprdwl_sipc_clean_swcnblk();

	ret = swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_CMD_EVENT,
					sprdwl_sipc_event_handler, intf->priv);
	if (ret) {
		wl_err("Failed to regitster event swcnblk notifier (%d)!\n",
		       ret);
		return ret;
	}

	ret = swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA0,
					sprdwl_sipc_data0_handler, intf->priv);
	if (ret) {
		wl_err("Failed to regitster data0 swcnblk notifier(%d)!\n",
		       ret);
		goto err_data0;
	}

	ret = swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA1,
					sprdwl_sipc_data1_handler, intf->priv);
	if (ret) {
		wl_err("Failed to regitster data1 swcnblk notifier(%d)!\n",
		       ret);
		goto err_data1;
	}

	return 0;

err_data1:
	swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA0,
				  NULL, NULL);
err_data0:
	swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_CMD_EVENT,
				  NULL, NULL);

	return ret;
}

struct sprdwl_intf *sprdwl_intf_create()
{
	struct sprdwl_intf *intf;

	intf = kzalloc(sizeof(*intf), GFP_ATOMIC);
	if (IS_ERR(intf)) {
		wl_err("allocate fail\n");
		return NULL;
	}
	intf->if_ops = &sprdwl_sipc_ops;
	return intf;
}

void sprdwl_intf_free(struct sprdwl_intf *intf)
{
	kfree(intf);
}

void sprdwl_intf_predeinit(struct sprdwl_intf *intf)
{
	intf->driver_status = 0;
}

int sprdwl_intf_init(struct sprdwl_intf *intf,
				struct platform_device *pdev,
				struct sprdwl_priv *priv)
{
	int ret;

	ret = start_marlin(WCN_MARLIN_WIFI);
	if (ret) {
		wl_err("start wcn err\n");
		return -ENXIO;
	}

	intf->pdev = pdev;
	intf->driver_status = 1;
	intf->priv = priv;
	p_sprdwl_intf = intf;

	spin_lock_init(&intf->lock);
	init_waitqueue_head(&intf->waitq);
	intf->cmd_timeout = msecs_to_jiffies(SPRDWL_TX_CMD_TIMEOUT);
	intf->data_timeout = msecs_to_jiffies(SPRDWL_TX_DATA_TIMEOUT);
	intf->wake_timeout = msecs_to_jiffies(SPRDWL_WAKE_TIMEOUT);
	intf->wake_pre_timeout = msecs_to_jiffies(SPRDWL_WAKE_PRE_TIMEOUT);
	intf->wake_last_time = jiffies;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	wakeup_source_init(&intf->keep_wake, "sprdwl_keep_wakelock");
#else
	wake_lock_init(&intf->keep_wake,
			WAKE_LOCK_SUSPEND, "sprdwl_keep_wakelock");
#endif
	ret = sprdwl_msg_init(SPRDWL_TX_MSG_CMD_NUM, &intf->tx_list0);
	if (ret) {
		dev_err(&intf->pdev->dev, "%s no tx_list0\n", __func__);
		goto err_tx_list0;
	}
	ret = sprdwl_msg_init(SPRDWL_TX_MSG_DATA_NUM, &intf->tx_list1);
	if (ret) {
		dev_err(&intf->pdev->dev, "%s no tx_list1\n", __func__);
		goto err_tx_list1;
	}
	ret = sprdwl_msg_init(SPRDWL_TX_MSG_DATA_NUM, &intf->tx_list2);
	if (ret) {
		dev_err(&intf->pdev->dev, "%s no tx_list2\n", __func__);
		goto err_tx_list2;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	wakeup_source_init(&intf->tx_wakelock, "sprdwl_tx_wakelock");
#else
	wake_lock_init(&intf->tx_wakelock,
			WAKE_LOCK_SUSPEND, "sprdwl_tx_wakelock");
#endif
	intf->tx_queue = alloc_ordered_workqueue("SPRDWL_TX_QUEUE",
						 WQ_MEM_RECLAIM |
						 WQ_HIGHPRI | WQ_CPU_INTENSIVE);
	if (!intf->tx_queue) {
		dev_err(&intf->pdev->dev,
			"%s SPRDWL_TX_QUEUE create failed", __func__);
		ret = -ENOMEM;
		goto err_tx_work;
	}
	INIT_WORK(&intf->tx_work, sprdwl_tx_work_queue);

	ret = sprdwl_sipc_swcnblk_init(intf);
	if (ret)
		goto err_swcnblk_init;

	return 0;

err_swcnblk_init:
	destroy_workqueue(intf->tx_queue);
err_tx_work:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	wakeup_source_trash(&intf->tx_wakelock);
#else
	wake_lock_destroy(&intf->tx_wakelock);
#endif
	sprdwl_msg_deinit(&intf->tx_list2);
err_tx_list2:
	sprdwl_msg_deinit(&intf->tx_list1);
err_tx_list1:
	sprdwl_msg_deinit(&intf->tx_list0);
err_tx_list0:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	wakeup_source_trash(&intf->keep_wake);
#else
	wake_lock_destroy(&intf->keep_wake);
#endif
	stop_marlin(WCN_MARLIN_WIFI);
	return ret;
}

void sprdwl_intf_deinit(struct sprdwl_intf *intf)
{
	intf->exit = 1;

	swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA1,
				  NULL, NULL);
	swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_DATA0,
				  NULL, NULL);
	swcnblk_register_notifier(WLAN_CP_ID, SPRDWL_SWCNBLK_CMD_EVENT,
				  NULL, NULL);

	wake_up_all(&intf->waitq);
	flush_workqueue(intf->tx_queue);
	destroy_workqueue(intf->tx_queue);

	sprdwl_sipc_flush_all_txlist(intf);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	wakeup_source_trash(&intf->tx_wakelock);
	wakeup_source_trash(&intf->keep_wake);
#else
	wake_lock_destroy(&intf->tx_wakelock);
	wake_lock_destroy(&intf->keep_wake);
#endif
	sprdwl_msg_deinit(&intf->tx_list0);
	sprdwl_msg_deinit(&intf->tx_list1);
	sprdwl_msg_deinit(&intf->tx_list2);

	stop_marlin(WCN_MARLIN_WIFI);
	pr_info("%s\t"
		"net: stop %u, start %u\t"
		"drop cnt: cmd %u, sta %u, p2p %u\t",
		__func__,
		intf->net_stop_cnt, intf->net_start_cnt,
		intf->drop_cmd_cnt, intf->drop_data1_cnt,
		intf->drop_data2_cnt);
}
