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

#include <linux/skbuff.h>

#include "sprdwl.h"
#include "txrx.h"
#include "cfg80211.h"
#include "cmdevt.h"
#include "intf_ops.h"
#include "work.h"
#if defined(SC2355_FTR)
#include "if_sc2355.h"
#endif
#include "rx_msg_sc2355.h"
#include "tcp_ack.h"
#include "core_sc2355.h"
#include "tx_msg_sc2355.h"

/* if err, the caller judge the skb if need free,
 * here just free the msg buf to the freelist
 */
int sprdwl_send_data(struct sprdwl_vif *vif, struct sprdwl_msg_buf *msg,
		     struct sk_buff *skb, u8 offset)
{
	int ret;
	unsigned char *buf = NULL;
/*TODO temp for MARLIN2*/
#ifndef SC2355_FTR
	struct sprdwl_data_hdr *hdr;
#endif
	struct sprdwl_intf *intf;
	unsigned int plen = cpu_to_le16(skb->len);

	intf = (struct sprdwl_intf *)vif->priv->hw_priv;

	buf = skb->data;
/*TODO temp for MARLIN2*/
#ifndef SC2355_FTR
	skb_push(skb, sizeof(*hdr) + offset);
	hdr = (struct sprdwl_data_hdr *)skb->data;
	memset(hdr, 0, sizeof(*hdr));
	hdr->common.type = SPRDWL_TYPE_DATA;
	hdr->common.ctx_id = vif->ctx_id;
	hdr->plen = cpu_to_le16(skb->len);
#else
	if (sprdwl_intf_fill_msdu_dscr(vif, skb, SPRDWL_TYPE_DATA, offset))
		return -EPERM;
#endif /* SC2355_FTR */

	sprdwl_fill_msg(msg, skb, skb->data, skb->len);

	if(intf->priv->hw_type == SPRDWL_HW_SC2355_PCIE)
		buf = skb->data + MSDU_DSCR_RSVD + DSCR_LEN;

	if (sprdwl_filter_send_tcp_ack(vif->priv, msg, buf, plen))
		return 0;

	ret = sprdwl_intf_tx(vif->priv, msg);
	if (ret)
		wl_err("%s TX data Err: %d\n", __func__, ret);

	if (intf->tdls_flow_count_enable == 1 &&
		vif->sm_state == SPRDWL_CONNECTED) {
		if (intf->priv->hw_type == SPRDWL_HW_SC2355_PCIE) {
			count_tdls_flow(vif,
					buf,
					skb->len);
		} else {
			count_tdls_flow(vif,
					skb->data + offset + DSCR_LEN,
					skb->len - offset - DSCR_LEN);
		}
	}

	return ret;
}

#if defined FPGA_LOOPBACK_TEST
int sprdwl_send_data_fpga_test(struct sprdwl_priv *priv,
				struct sprdwl_msg_buf *msg,
				struct sk_buff *skb, u8 type, u8 offset)
{
	int ret = 0;

	sprdwl_intf_fill_msdu_dscr_test(priv, skb, type, offset);
	msg->type = type;

	sprdwl_fill_msg(msg, skb, skb->data, skb->len);
	ret = sprdwl_intf_tx(priv, msg);

	return ret;
}
#endif

int sprdwl_send_cmd(struct sprdwl_priv *priv, struct sprdwl_msg_buf *msg)
{
	int ret;
	struct sk_buff *skb;

	skb = msg->skb;
	ret = sprdwl_intf_tx(priv, msg);
	if (ret) {
		wl_err("%s TX cmd Err: %d\n", __func__, ret);
		/* now cmd msg droped */
#if !defined(SC2355_FTR)
		dev_kfree_skb(skb);
#endif
	}

	return ret;
}

void sprdwl_rx_send_cmd_process(struct sprdwl_priv *priv, void *data, int len,
				unsigned char id, unsigned char ctx_id)
{
	struct sprdwl_vif *vif;
	struct sprdwl_work *misc_work = NULL;

	if (unlikely(!priv)) {
		wl_err("%s priv not init.\n", __func__);
	} else if (ctx_id > STAP_MODE_P2P_DEVICE) {
		wl_err("%s [ctx_id %d]RX err\n", __func__, ctx_id);
	} else {
		vif = ctx_id_to_vif(priv, ctx_id);
		if (!vif) {
			wl_err("%s cant't get vif from ctx_id%d\n",
			       __func__, ctx_id);
		} else {
			misc_work = sprdwl_alloc_work(len);
			if (!misc_work) {
				wl_err("%s out of memory", __func__);
			} else {
				misc_work->vif = vif;
				misc_work->id = id;
				memcpy(misc_work->data, data, len);
				sprdwl_queue_work(vif->priv, misc_work);
			}
			sprdwl_put_vif(vif);
		}
	}
}

void sprdwl_rx_skb_process(struct sprdwl_priv *priv, struct sk_buff *skb)
{
	struct sprdwl_vif *vif = NULL;
	struct net_device *ndev = NULL;
	struct rx_msdu_desc *msdu_desc = NULL;
	struct sk_buff *tx_skb = NULL;
	struct sprdwl_intf *intf;
	struct ethhdr *eth;

	intf = (struct sprdwl_intf *)priv->hw_priv;

	if (unlikely(!priv)) {
		wl_err("%s priv not init.\n", __func__);
		goto err;
	}

	msdu_desc = (struct rx_msdu_desc *)skb->data;
	if (msdu_desc->ctx_id >= SPRDWL_MAC_INDEX_MAX) {
		wl_err("%s [ctx_id %d]RX err\n", __func__, msdu_desc->ctx_id);
		goto err;
	}

	vif = ctx_id_to_vif(priv, msdu_desc->ctx_id);
	if (!vif) {
		wl_err("%s cannot get vif, ctx_id: %d\n",
		       __func__, msdu_desc->ctx_id);
		goto err;
	}

	/* Sanity check for bug 846454 */
	if (vif->ndev == NULL) {
		wl_err("%s ndev is NULL, ctx_id = %d\n",
		       __func__, msdu_desc->ctx_id);
		BUG_ON(1);
	}

	ndev = vif->ndev;
	skb_reserve(skb, msdu_desc->msdu_offset);
	skb_put(skb, msdu_desc->msdu_len);

	eth = (struct ethhdr *)skb->data;
	if (eth->h_proto == htons(ETH_P_IPV6))
		if(ether_addr_equal(skb->data, skb->data + ETH_ALEN)) {
			wl_err("%s, drop loopback pkt, macaddr:%02x:%02x:%02x:%02x:%02x:%02x\n",
				__func__, skb->data[0], skb->data[1], skb->data[2],
				skb->data[3], skb->data[4], skb->data[5]);
			goto err;
		}

	if (intf->tdls_flow_count_enable == 1)
		count_tdls_flow(vif,
				skb->data + ETH_ALEN,
				skb->len - ETH_ALEN);

	/* FIXME: We would remove mode in furture, how to modify? */
	if (((vif->mode == SPRDWL_MODE_AP) ||
	     (vif->mode == SPRDWL_MODE_P2P_GO)) && msdu_desc->uc_w2w_flag) {
		skb->dev = ndev;
		dev_queue_xmit(skb);
	} else {
		if (((vif->mode == SPRDWL_MODE_AP) ||
		     (vif->mode == SPRDWL_MODE_P2P_GO)) &&
		    msdu_desc->bc_mc_w2w_flag) {
			struct ethhdr *eth = (struct ethhdr *)skb->data;

			if (eth->h_proto != ETH_P_IP &&
			    eth->h_proto != ETH_P_IPV6) {
				tx_skb = pskb_copy(skb, GFP_ATOMIC);
				if (likely(tx_skb)) {
					tx_skb->dev = ndev;
					dev_queue_xmit(tx_skb);
				}
			}
		}

		/* skb->data MUST point to ETH HDR */
		sprdwl_filter_rx_tcp_ack(priv, skb->data, msdu_desc->msdu_len);
		if (intf->priv->hw_type == SPRDWL_HW_SC2355_PCIE) {
			sprdwl_count_rx_tp(intf, msdu_desc->msdu_len);
		}
		sprdwl_netif_rx(skb, ndev);
	}

	sprdwl_put_vif(vif);

	return;

err:
	dev_kfree_skb(skb);
#if defined(MORE_DEBUG)
	intf->stats.rx_errors++;
	intf->stats.rx_dropped++;
#endif
}
unsigned short sprdwl_rx_data_process(struct sprdwl_priv *priv,
				      unsigned char *msg)
{
	return 0;
}
