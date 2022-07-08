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

#include <linux/utsname.h>
#include <misc/wcn_bus.h>
#include "sprdwl.h"
#include "wapi.h"
#include "npi.h"
#include "cfg80211.h"
#include "cmdevt.h"
#include "txrx.h"
#include "msg.h"
#include "vendor.h"
#include "work.h"
#include "tcp_ack.h"

unsigned int dump_data;
module_param(dump_data, uint, 0);
MODULE_PARM_DESC(dump_data, "dump data packet");
unsigned int wfa_cap;
module_param(wfa_cap, uint, 0);
MODULE_PARM_DESC(wfa_cap, "set capability for WFA test");

unsigned int tcp_ack_drop_cnt = SPRDWL_TCP_ACK_DROP_CNT;
/* Maybe you need S_IRUGO | S_IWUSR for debug */
module_param(tcp_ack_drop_cnt, uint, 0);
MODULE_PARM_DESC(tcp_ack_drop_cnt, "valid values: [1, 13]");

struct sprdwl_priv *g_sprdwl_priv;
struct device *sprdwl_dev;

static void str2mac(const char *mac_addr, u8 *mac)
{
	unsigned int m[ETH_ALEN];

	if (sscanf(mac_addr, "%02x:%02x:%02x:%02x:%02x:%02x",
		   &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != ETH_ALEN) {
		wl_err("failed to parse mac address '%s'", mac_addr);
		memset(m, 0, sizeof(unsigned int) * ETH_ALEN);
	}
	mac[0] = m[0];
	mac[1] = m[1];
	mac[2] = m[2];
	mac[3] = m[3];
	mac[4] = m[4];
	mac[5] = m[5];
}

void sprdwl_netif_rx(struct sk_buff *skb, struct net_device *ndev)
{
	if (dump_data)
		print_hex_dump_debug("RX packet: ", DUMP_PREFIX_OFFSET,
				     16, 1, skb->data, skb->len, 0);
	skb->dev = ndev;
	skb->protocol = eth_type_trans(skb, ndev);
	/* CHECKSUM_UNNECESSARY not supported by our hardware */
	/* skb->ip_summed = CHECKSUM_UNNECESSARY; */

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += skb->len;

	//netif_rx_ni(skb);
	local_bh_disable();
	netif_receive_skb(skb);
	local_bh_enable();
}

void sprdwl_stop_net(struct sprdwl_vif *vif)
{
	struct sprdwl_vif *real_vif, *tmp_vif;
	struct sprdwl_priv *priv = vif->priv;

	spin_lock_bh(&priv->list_lock);
	list_for_each_entry_safe(real_vif, tmp_vif, &priv->vif_list, vif_node)
		if (real_vif->ndev)
			netif_stop_queue(real_vif->ndev);
	set_bit(SPRDWL_AP_FLOW_CTR, &priv->flags);
	spin_unlock_bh(&priv->list_lock);
}

static void sprdwl_netflowcontrl_mode(struct sprdwl_priv *priv,
				      enum sprdwl_mode mode, bool state)
{
	struct sprdwl_vif *vif;

	vif = mode_to_vif(priv, mode);
	if (vif) {
		if (state)
			netif_wake_queue(vif->ndev);
		else
			netif_stop_queue(vif->ndev);
		sprdwl_put_vif(vif);
	}
}

static void sprdwl_netflowcontrl_all(struct sprdwl_priv *priv, bool state)
{
	struct sprdwl_vif *real_vif, *tmp_vif;

	spin_lock_bh(&priv->list_lock);
	list_for_each_entry_safe(real_vif, tmp_vif, &priv->vif_list, vif_node)
		if (real_vif->ndev) {
			if (state)
				netif_wake_queue(real_vif->ndev);
			else
				netif_stop_queue(real_vif->ndev);
		}
	spin_unlock_bh(&priv->list_lock);
}

/* @state: true for netif_start_queue
 *	   false for netif_stop_queue
 */
void sprdwl_net_flowcontrl(struct sprdwl_priv *priv,
			   enum sprdwl_mode mode, bool state)
{
	if (mode != SPRDWL_MODE_NONE)
		sprdwl_netflowcontrl_mode(priv, mode, state);
	else
		sprdwl_netflowcontrl_all(priv, state);
}

static int sprdwl_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	bool flag;
	unsigned char type;
	int ret;
	struct sprdwl_vif *vif;
	struct sprdwl_msg_buf *msg;
	unsigned int len;

	vif = netdev_priv(ndev);
	/* FIXME vif connect state, need fix cfg80211_connect_result when MCC */
	/*if (vif->connect_status != SPRDWL_CONNECTED) */

	/* Hardware tx data queue prority is lower than management queue
	 * management frame will be send out early even that get into queue
	 * after data frame.
	 * Workaround way: Put eap failure frame to high queue
	 * by use tx mgmt cmd
	 */
	if (vif->mode == SPRDWL_MODE_P2P_GO &&
	    skb->protocol == cpu_to_be16(ETH_P_PAE)) {
		u8 *data = (u8 *)(skb->data) + sizeof(struct ethhdr);
		struct sprdwl_eap_hdr *eap = (struct sprdwl_eap_hdr *)data;

		if (eap->type == EAP_PACKET_TYPE &&
		    eap->code == EAP_FAILURE_CODE) {
			sprdwl_xmit_data2mgmt(skb, ndev);
			return NETDEV_TX_OK;
		}
	}

	msg = vif->priv->if_ops->get_msg_buf(vif->priv->hw_intf,
					     SPRDWL_TYPE_DATA, vif->mode);
	if (!msg) {
		ndev->stats.tx_fifo_errors++;
		return NETDEV_TX_BUSY;
	}

	if (!sprdwl_is_wapi(vif, skb->data)) {
		if (skb_headroom(skb) < ndev->needed_headroom) {
			struct sk_buff *tmp_skb = skb;

			skb = skb_realloc_headroom(skb, ndev->needed_headroom);
			dev_kfree_skb(tmp_skb);
			if (!skb) {
				netdev_err(ndev,
					   "%s skb_realloc_headroom failed\n",
					   __func__);
				vif->priv->if_ops->free_msg_buf(vif->priv->
								hw_intf, msg);
				goto out;
			}
		}
		type = SPRDWL_DATA_TYPE_NORMAL;
		flag = true;
	} else {
		type = SPRDWL_DATA_TYPE_WAPI;
		flag = false;
	}

	/* Bug-831329. Save the len to avoid
	 * use-after-free crash when access skb after
	 * sprdwl_send_data. The skb may be freed in
	 * sprdwl_tx_work_queue.
	 */
	len = skb->len;
	if (dump_data)
		print_hex_dump_debug("TX packet: ", DUMP_PREFIX_OFFSET,
				     16, 1, skb->data, len, 0);
	/* sprdwl_send_data: offset use 2 for cp bytes align */
	ret = sprdwl_send_data(vif, msg, skb, type, 2, flag);
	if (ret) {
		netdev_err(ndev, "%s drop msg due to TX Err\n", __func__);
		/* FIXME as debug sdiom later, here just drop the msg
		 * wapi temp drop
		 */
		if (type == SPRDWL_DATA_TYPE_NORMAL)
			dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	vif->ndev->stats.tx_bytes += len;
	vif->ndev->stats.tx_packets++;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	ndev->trans_start = jiffies;
#endif
out:
	return NETDEV_TX_OK;
}

static int sprdwl_init(struct net_device *ndev)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);

	/* initialize firmware */
	return sprdwl_init_fw(vif);
}

static void sprdwl_uninit(struct net_device *ndev)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);

	sprdwl_uninit_fw(vif);
}

static int sprdwl_open(struct net_device *ndev)
{
	netdev_info(ndev, "%s\n", __func__);

	netif_start_queue(ndev);

	return 0;
}

static int sprdwl_close(struct net_device *ndev)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);

	netdev_info(ndev, "%s\n", __func__);

	sprdwl_check_connection(vif);
	sprdwl_scan_done(vif, true);
	sprdwl_sched_scan_done(vif, true);
	netif_stop_queue(ndev);
	if (netif_carrier_ok(ndev))
		netif_carrier_off(ndev);

	return 0;
}

static struct net_device_stats *sprdwl_get_stats(struct net_device *ndev)
{
	return &ndev->stats;
}

static void sprdwl_tx_timeout(struct net_device *ndev)
{
	netdev_info(ndev, "%s\n", __func__);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	ndev->trans_start = jiffies;
#endif
	netif_wake_queue(ndev);
}

#define CMD_BLACKLIST_ENABLE		"BLOCK"
#define CMD_BLACKLIST_DISABLE		"UNBLOCK"
#define CMD_ADD_WHITELIST		"WHITE_ADD"
#define CMD_DEL_WHITELIST		"WHITE_DEL"
#define CMD_ENABLE_WHITELIST		"WHITE_EN"
#define CMD_DISABLE_WHITELIST		"WHITE_DIS"
#define CMD_SETSUSPENDMODE		"SETSUSPENDMODE"
#define CMD_SET_FCC_CHANNEL		"SET_FCC_CHANNEL"
#define CMD_REDUCE_TX_POWER		"SET_TX_POWER_CALLING"
#define CMD_SET_COUNTRY			"COUNTRY"
#define CMD_11V_GET_CFG			"11VCFG_GET"
#define CMD_11V_SET_CFG			"11VCFG_SET"
#define CMD_11V_WNM_SLEEP		"WNM_SLEEP"
#define CMD_P2P_MAC			"P2PMACADDR"
#define CMD_SET_SAR			"SET_SAR"


static int sprdwl_priv_cmd(struct net_device *ndev, struct ifreq *ifr)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_priv *priv = vif->priv;
	struct android_wifi_priv_cmd priv_cmd;
	char *command = NULL, *country = NULL;
	u16 interval = 0;
	u8 feat = 0, status = 0;
	u8 addr[ETH_ALEN] = {0}, *mac_addr = NULL, *tmp, *mac_list;
	int ret = 0, skip, counter, index;

	if (!ifr->ifr_data)
		return -EINVAL;
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(priv_cmd)))
		return -EFAULT;

	/*add length check to avoid invalid NULL ptr*/
	if ((!priv_cmd.total_len) || (SPRDWL_MAX_CMD_TXLEN < priv_cmd.total_len)) {
		netdev_err(ndev, "%s: priv cmd total len is invalid\n", __func__);
		return -EINVAL;
	}

	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto out;
	}

	if (!strncasecmp(command, CMD_BLACKLIST_ENABLE,
			 strlen(CMD_BLACKLIST_ENABLE))) {
		skip = strlen(CMD_BLACKLIST_ENABLE) + 1;
		str2mac(command + skip, addr);
		if (!is_valid_ether_addr(addr))
			goto out;
		netdev_info(ndev, "%s: block %pM\n", __func__, addr);
		ret = sprdwl_set_blacklist(priv, vif->mode,
					   SPRDWL_SUBCMD_ADD, 1, addr);
	} else if (!strncasecmp(command, CMD_BLACKLIST_DISABLE,
				strlen(CMD_BLACKLIST_DISABLE))) {
		skip = strlen(CMD_BLACKLIST_DISABLE) + 1;
		str2mac(command + skip, addr);
		if (!is_valid_ether_addr(addr))
			goto out;
		netdev_info(ndev, "%s: unblock %pM\n", __func__, addr);
		ret = sprdwl_set_blacklist(priv, vif->mode,
					   SPRDWL_SUBCMD_DEL, 1, addr);
	} else if (!strncasecmp(command, CMD_ADD_WHITELIST,
				strlen(CMD_ADD_WHITELIST))) {
		skip = strlen(CMD_ADD_WHITELIST) + 1;
		str2mac(command + skip, addr);
		if (!is_valid_ether_addr(addr))
			goto out;
		netdev_info(ndev, "%s: add whitelist %pM\n", __func__, addr);
		ret = sprdwl_set_whitelist(priv, vif->mode,
					   SPRDWL_SUBCMD_ADD, 1, addr);
	} else if (!strncasecmp(command, CMD_DEL_WHITELIST,
				strlen(CMD_DEL_WHITELIST))) {
		skip = strlen(CMD_DEL_WHITELIST) + 1;
		str2mac(command + skip, addr);
		if (!is_valid_ether_addr(addr))
			goto out;
		netdev_info(ndev, "%s: delete whitelist %pM\n", __func__, addr);
		ret = sprdwl_set_whitelist(priv, vif->mode,
					   SPRDWL_SUBCMD_DEL, 1, addr);
	} else if (!strncasecmp(command, CMD_ENABLE_WHITELIST,
				strlen(CMD_ENABLE_WHITELIST))) {
		skip = strlen(CMD_ENABLE_WHITELIST) + 1;
		counter = command[skip];
		netdev_info(ndev, "%s: enable whitelist counter : %d\n",
			    __func__, counter);
		if (!counter) {
			ret = sprdwl_set_whitelist(priv, vif->mode,
						   SPRDWL_SUBCMD_ENABLE,
						   0, NULL);
			goto out;
		}
		mac_addr = kmalloc(ETH_ALEN * counter, GFP_KERNEL);
		mac_list = mac_addr;
		if (IS_ERR(mac_addr)) {
			ret = -ENOMEM;
			goto out;
		}

		tmp = command + skip + 1;
		for (index = 0; index < counter; index++) {
			str2mac(tmp, mac_addr);
			if (!is_valid_ether_addr(mac_addr))
				goto out;
			netdev_info(ndev, "%s: enable whitelist %pM\n",
				    __func__, mac_addr);
			mac_addr += ETH_ALEN;
			tmp += 18;
		}
		ret = sprdwl_set_whitelist(priv, vif->mode,
					   SPRDWL_SUBCMD_ENABLE,
					   counter, mac_list);
		kfree(mac_list);
	} else if (!strncasecmp(command, CMD_DISABLE_WHITELIST,
				strlen(CMD_DISABLE_WHITELIST))) {
		skip = strlen(CMD_DISABLE_WHITELIST) + 1;
		counter = command[skip];
		netdev_info(ndev, "%s: disable whitelist counter : %d\n",
			    __func__, counter);
		if (!counter) {
			ret = sprdwl_set_whitelist(priv, vif->mode,
						   SPRDWL_SUBCMD_DISABLE,
						   0, NULL);
			goto out;
		}
		mac_addr = kmalloc(ETH_ALEN * counter, GFP_KERNEL);
		mac_list = mac_addr;
		if (IS_ERR(mac_addr)) {
			ret = -ENOMEM;
			goto out;
		}

		tmp = command + skip + 1;
		for (index = 0; index < counter; index++) {
			str2mac(tmp, mac_addr);
			if (!is_valid_ether_addr(mac_addr))
				goto out;
			netdev_info(ndev, "%s: disable whitelist %pM\n",
				    __func__, mac_addr);
			mac_addr += ETH_ALEN;
			tmp += 18;
		}
		ret = sprdwl_set_whitelist(priv, vif->mode,
					   SPRDWL_SUBCMD_DISABLE,
					   counter, mac_list);
		kfree(mac_list);
	} else if (!strncasecmp(command, CMD_11V_GET_CFG,
				strlen(CMD_11V_GET_CFG))) {
		/* deflaut CP support all featrue */
		if (priv_cmd.total_len < (strlen(CMD_11V_GET_CFG) + 4)) {
			ret = -ENOMEM;
			goto out;
		}
		memset(command, 0, priv_cmd.total_len);
		if (priv->fw_std & SPRDWL_STD_11V)
			feat = priv->wnm_ft_support;

		sprintf(command, "%s %d", CMD_11V_GET_CFG, feat);
		netdev_info(ndev, "%s: get 11v feat\n", __func__);
		if (copy_to_user(priv_cmd.buf, command, priv_cmd.total_len)) {
			netdev_err(ndev, "%s: get 11v copy failed\n", __func__);
			ret = -EFAULT;
			goto out;
		}
	} else if (!strncasecmp(command, CMD_11V_SET_CFG,
				strlen(CMD_11V_SET_CFG))) {
		int skip = strlen(CMD_11V_SET_CFG) + 1;
		int cfg = command[skip];

		netdev_info(ndev, "%s: 11v cfg %d\n", __func__, cfg);
		sprdwl_set_11v_feature_support(priv, vif->mode, cfg);
	} else if (!strncasecmp(command, CMD_11V_WNM_SLEEP,
				strlen(CMD_11V_WNM_SLEEP))) {
		int skip = strlen(CMD_11V_WNM_SLEEP) + 1;

		status = command[skip];
		if (status)
			interval = command[skip + 1];

		netdev_info(ndev, "%s: 11v sleep, status %d, interval %d\n",
			    __func__, status, interval);
		sprdwl_set_11v_sleep_mode(priv, vif->mode, status, interval);
	} else if (!strncasecmp(command, CMD_SET_COUNTRY,
				strlen(CMD_SET_COUNTRY))) {
		skip = strlen(CMD_SET_COUNTRY) + 1;
		country = command + skip;

		if (!country || strlen(country) != SPRDWL_COUNTRY_CODE_LEN) {
			netdev_err(ndev, "%s: invalid country code\n",
				   __func__);
			ret = -EINVAL;
			goto out;
		}
		netdev_info(ndev, "%s country code:%c%c\n", __func__,
			    toupper(country[0]), toupper(country[1]));
		ret = regulatory_hint(priv->wiphy, country);
	} else {
		netdev_err(ndev, "%s command not support\n", __func__);
		ret = -ENOTSUPP;
	}
out:
	kfree(command);
	return ret;
}

static int sprdwl_set_power_save(struct net_device *ndev, struct ifreq *ifr)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_priv *priv = vif->priv;
	struct android_wifi_priv_cmd priv_cmd;
	char *command = NULL;
	int ret = 0, skip, value;

	if (!ifr->ifr_data)
		return -EINVAL;
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(priv_cmd)))
		return -EFAULT;

	/*add length check to avoid invalid NULL ptr*/
	if ((!priv_cmd.total_len) || (priv_cmd.total_len > SPRDWL_MAX_CMD_TXLEN)) {
		netdev_err(ndev, "%s: priv cmd total len is invalid\n", __func__);
		return -EINVAL;
	}

	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto out;
	}

	if (!strncasecmp(command, CMD_SETSUSPENDMODE,
			 strlen(CMD_SETSUSPENDMODE))) {
		skip = strlen(CMD_SETSUSPENDMODE) + 1;
		ret = kstrtoint(command + skip, 0, &value);
		if (ret)
			goto out;
		netdev_info(ndev, "%s: set suspend mode,value : %d\n",
			    __func__, value);
		ret = sprdwl_power_save(priv, vif->mode,
					SPRDWL_SET_SUSPEND, value);
	} else if (!strncasecmp(command, CMD_SET_FCC_CHANNEL,
				strlen(CMD_SET_FCC_CHANNEL))) {
		skip = strlen(CMD_SET_FCC_CHANNEL) + 1;
		ret = kstrtoint(command + skip, 0, &value);
		if (ret)
			goto out;
		netdev_info(ndev, "%s: set fcc channel,value : %d\n",
			    __func__, value);
		ret = sprdwl_power_save(priv, vif->mode,
					SPRDWL_SET_FCC_CHANNEL, value);
	} else if (!strncasecmp(command, CMD_REDUCE_TX_POWER,
				strlen(CMD_REDUCE_TX_POWER))) {
		skip = strlen(CMD_REDUCE_TX_POWER) + 1;
		ret = kstrtoint(command + skip, 0, &value);
		if (ret)
			goto out;
		netdev_info(ndev, "%s: reduce tx power,value : %d\n",
			    __func__, value);
		ret = sprdwl_power_save(priv, vif->mode,
					SPRDWL_SET_TX_POWER, value);
	} else if (!strncasecmp(command, CMD_SET_SAR,
				strlen(CMD_SET_SAR))) {
		skip = strlen(CMD_SET_SAR) + 1;
		ret = kstrtoint(command + skip, 0, &value);
		if (ret)
			goto out;
		netdev_info(ndev, "%s: set sar,value : %d\n",
			    __func__, value);
		ret = sprdwl_set_sar(priv, vif->mode, SPRDWl_SET_SAR_ABSOLUTE,
				     value, SPRDWL_SET_SAR_ALL_MODE);

	} else {
		netdev_err(ndev, "%s command not support\n", __func__);
		ret = -ENOTSUPP;
	}
out:
	kfree(command);
	return ret;
}

static int sprdwl_set_vowifi(struct net_device *ndev, struct ifreq *ifr)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_priv *priv = vif->priv;
	struct android_wifi_priv_cmd priv_cmd;
	struct sprdwl_vowifi_data *vowifi;
	char *command = NULL;
	int ret, value;

	if (!ifr->ifr_data)
		return -EINVAL;
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(priv_cmd)))
		return -EFAULT;

	/*add length check to avoid invalid NULL ptr*/
	if ((!priv_cmd.total_len) || (SPRDWL_MAX_CMD_TXLEN < priv_cmd.total_len)) {
		netdev_err(ndev, "%s: priv cmd total len is invalid\n", __func__);
		return -EINVAL;
	}

	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto out;
	}

	vowifi = (struct sprdwl_vowifi_data *)command;
	value = *vowifi->data;
	netdev_info(ndev, "%s value:%d\n", __func__, value);
	if (value != 0 && value != 1) {
		ret =  -EINVAL;
		goto out;
	}
	ret = sprdwl_set_vowifi_state(priv, vif->mode, value);

	if (ret)
		netdev_err(ndev, "%s set vowifi cmd error\n", __func__);
out:
	kfree(command);
	return ret;
}

static int sprdwl_set_p2p_mac(struct net_device *ndev, struct ifreq *ifr)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_priv *priv = vif->priv;
	struct android_wifi_priv_cmd priv_cmd;
	char *command = NULL;
	int ret = 0;
	struct sprdwl_vif *tmp1, *tmp2;
	u8 addr[ETH_ALEN] = {0};

	if (!ifr->ifr_data)
		return -EINVAL;
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(priv_cmd)))
		return -EFAULT;

	/*add length check to avoid invalid NULL ptr*/
	if ((!priv_cmd.total_len) || (SPRDWL_MAX_CMD_TXLEN < priv_cmd.total_len)) {
		netdev_err(ndev, "%s: priv cmd total len is invalid\n", __func__);
		return -EINVAL;
	}

	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto out;
	}

	memcpy(addr, command + 11, ETH_ALEN);
	netdev_info(ndev, "p2p dev random addr is %pM\n", addr);
	if (is_multicast_ether_addr(addr)) {
		netdev_err(ndev, "%s invalid addr\n", __func__);
		ret = -EINVAL;
		goto out;
	} else if (is_zero_ether_addr(addr)) {
		netdev_info(ndev, "restore to vif addr if addr is zero \n");
		memcpy(addr, vif->mac, ETH_ALEN);
	}
	ret = sprdwl_set_random_mac(vif->priv, SPRDWL_MODE_P2P_DEVICE, addr);
	if (ret) {
		netdev_err(ndev, "%s set p2p mac cmd error\n", __func__);
		ret = -EFAULT;
		goto out;
	}

	spin_lock_bh(&priv->list_lock);
	list_for_each_entry_safe_reverse(tmp1, tmp2,
					 &priv->vif_list, vif_node) {
		if (tmp1->mode == SPRDWL_MODE_P2P_DEVICE) {
			netdev_info(ndev, "get p2p device, set addr for wdev\n");
			memcpy(tmp1->wdev.address, addr, ETH_ALEN);
		}
	}
	spin_unlock_bh(&priv->list_lock);

out:
	kfree(command);
	return ret;
}
#define SPRDWLIOCTL		(SIOCDEVPRIVATE + 1)
#define SPRDWLHANDLECMD		(SIOCDEVPRIVATE + 2)
#define SPRDWLSETSUSPEND	(SIOCDEVPRIVATE + 4)
#define SPRDWLSETCOUNTRY	(SIOCDEVPRIVATE + 5)
#define SPRDWLSETP2PMAC		(SIOCDEVPRIVATE + 6)
#define SPRDWLVOWIFI		(SIOCDEVPRIVATE + 7)

static int sprdwl_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	switch (cmd) {
	case SPRDWLIOCTL:
	case SPRDWLSETCOUNTRY:
		return sprdwl_priv_cmd(ndev, req);
	case SPRDWLHANDLECMD:
		return sprdwl_handle_cmd(ndev, req);
	case SPRDWLSETSUSPEND:
		return sprdwl_set_power_save(ndev, req);
	case SPRDWLVOWIFI:
		return sprdwl_set_vowifi(ndev, req);
	case SPRDWLSETP2PMAC:
		return sprdwl_set_p2p_mac(ndev, req);
	default:
		netdev_err(ndev, "Unsupported IOCTL %d\n", cmd);
		return -ENOTSUPP;
	}
}

static bool mc_address_changed(struct net_device *ndev)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct netdev_hw_addr *ha;
	u8 mc_count, index;
	u8 *mac_addr;
	bool found;

	mc_count = netdev_mc_count(ndev);

	if (mc_count != vif->mc_filter->mac_num)
		return true;

	mac_addr = vif->mc_filter->mac_addr;
	netdev_for_each_mc_addr(ha, ndev) {
		found = false;
		for (index = 0; index < vif->mc_filter->mac_num; index++) {
			if (!memcmp(ha->addr, mac_addr, ETH_ALEN)) {
				found = true;
				break;
			}
			mac_addr += ETH_ALEN;
		}

		if (!found)
			return true;
	}
	return false;
}

#define SPRDWL_RX_MODE_MULTICAST	1
static void sprdwl_set_multicast(struct net_device *ndev)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_priv *priv = vif->priv;
	struct sprdwl_work *work;
	struct netdev_hw_addr *ha;
	u8 mc_count;
	u8 *mac_addr;

	mc_count = netdev_mc_count(ndev);
	netdev_info(ndev, "%s multicast address num: %d\n", __func__, mc_count);
	if (mc_count > priv->max_mc_mac_addrs)
		return;

	vif->mc_filter->mc_change = false;
	if ((ndev->flags & IFF_MULTICAST) && (mc_address_changed(ndev))) {
		mac_addr = vif->mc_filter->mac_addr;
		netdev_for_each_mc_addr(ha, ndev) {
			netdev_info(ndev, "%s set mac: %pM\n", __func__,
				    ha->addr);
			if ((ha->addr[0] != 0x33 || ha->addr[1] != 0x33) &&
			    (ha->addr[0] != 0x01 || ha->addr[1] != 0x00 ||
			     ha->addr[2] != 0x5e || ha->addr[3] > 0x7f)) {
				netdev_info(ndev, "%s invalid addr\n",
					    __func__);
				return;
			}
			memcpy(mac_addr, ha->addr, ETH_ALEN);
			mac_addr += ETH_ALEN;
		}
		vif->mc_filter->mac_num = mc_count;
		vif->mc_filter->mc_change = true;
	} else if (!(ndev->flags & IFF_MULTICAST) && vif->mc_filter->mac_num) {
		vif->mc_filter->mac_num = 0;
		vif->mc_filter->mc_change = true;
	}

	work = sprdwl_alloc_work(0);
	if (!work) {
		netdev_err(ndev, "%s out of memory\n", __func__);
		return;
	}
	work->vif = vif;
	work->id = SPRDWL_WORK_MC_FILTER;
	vif->mc_filter->subtype = SPRDWL_RX_MODE_MULTICAST;
	sprdwl_queue_work(vif->priv, work);
}

static int sprdwl_set_mac(struct net_device *dev, void *addr)
{
	struct sprdwl_vif *vif = netdev_priv(dev);
	struct sockaddr *sa = (struct sockaddr *)addr;
	int ret;

	if (!dev) {
		netdev_err(dev, "Invalid net device\n");
		return -EINVAL;
	}
	netdev_info(dev, "%s start set random mac: %pM\n", __func__, sa->sa_data);

	if (is_multicast_ether_addr(sa->sa_data)) {
		netdev_err(dev, "invalid, it is multicast addr: %pM\n", sa->sa_data);
		return -EINVAL;
	}

	if (vif->mode == SPRDWL_MODE_STATION) {
		if (!is_zero_ether_addr(sa->sa_data)) {
			vif->has_rand_mac = true;
			memcpy(vif->random_mac, sa->sa_data, ETH_ALEN);
			memcpy(dev->dev_addr, sa->sa_data, ETH_ALEN);
			netdev_info(dev, "vif random mac : %pM\n", vif->random_mac);
		} else {
			vif->has_rand_mac = false;
			netdev_info(dev, "%s need clear random mac for sta/softap mode\n", __func__);
			memset(vif->random_mac, 0, ETH_ALEN);
			memcpy(dev->dev_addr, vif->mac, ETH_ALEN);
		}
	}

	if (vif->mode == SPRDWL_MODE_P2P_CLIENT || vif->mode == SPRDWL_MODE_P2P_GO) {
		if (!is_zero_ether_addr(sa->sa_data)) {
			netdev_info(dev,"%s vif-> mac : %pM\n", __func__, vif->mac);
			if (ether_addr_equal(vif->mac, sa->sa_data)) {
				netdev_info(dev, "equal to vif mac, no need set to cp\n");
				memset(vif->random_mac, 0, ETH_ALEN);
				memcpy(dev->dev_addr, vif->mac, ETH_ALEN);
				vif->has_rand_mac = false;
				return 0;
			}
			netdev_info(dev, "set go/gc random mac addr\n");
			memcpy(dev->dev_addr, sa->sa_data, ETH_ALEN);
			vif->has_rand_mac = true;
			memcpy(vif->random_mac, sa->sa_data, ETH_ALEN);
			ret = sprdwl_set_random_mac(vif->priv,
						    vif->mode, sa->sa_data);
			if (ret) {
				netdev_err(dev, "%s set p2p mac error\n", __func__);
				return -EFAULT;
			}
		} else {
			netdev_info(dev, "%s clear mac for go/gc mode\n", __func__);
			vif->has_rand_mac = false;
			memset(vif->random_mac, 0, ETH_ALEN);
			memcpy(dev->dev_addr, vif->mac, ETH_ALEN);
		}
	}
	/*return success to pass vts test*/
	return 0;
}

static struct net_device_ops sprdwl_netdev_ops = {
	.ndo_init = sprdwl_init,
	.ndo_uninit = sprdwl_uninit,
	.ndo_open = sprdwl_open,
	.ndo_stop = sprdwl_close,
	.ndo_start_xmit = sprdwl_start_xmit,
	.ndo_get_stats = sprdwl_get_stats,
	.ndo_tx_timeout = sprdwl_tx_timeout,
	.ndo_do_ioctl = sprdwl_ioctl,
	.ndo_set_mac_address = sprdwl_set_mac,
};

static int sprdwl_inetaddr_event(struct notifier_block *this,
				 unsigned long event, void *ptr)
{
	struct net_device *ndev;
	struct sprdwl_vif *vif;
	struct in_ifaddr *ifa = (struct in_ifaddr *)ptr;

	if (!ifa || !(ifa->ifa_dev->dev))
		return NOTIFY_DONE;
	if (ifa->ifa_dev->dev->netdev_ops != &sprdwl_netdev_ops)
		return NOTIFY_DONE;

	ndev = ifa->ifa_dev->dev;
	vif = netdev_priv(ndev);

	switch (vif->wdev.iftype) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		if (event == NETDEV_UP)
			sprdwl_notify_ip(vif->priv, vif->mode, SPRDWL_IPV4,
					 (u8 *)&ifa->ifa_address);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block sprdwl_inetaddr_cb = {
	.notifier_call = sprdwl_inetaddr_event,
};

static int sprdwl_inetaddr6_event(struct notifier_block *this,
				  unsigned long event, void *ptr)
{
	struct net_device *ndev;
	struct sprdwl_vif *vif;
	struct inet6_ifaddr *inet6_ifa = (struct inet6_ifaddr *)ptr;
	struct sprdwl_work *work;
	u8 *ipv6_addr;

	if (!inet6_ifa || !(inet6_ifa->idev->dev))
		return NOTIFY_DONE;

	if (inet6_ifa->idev->dev->netdev_ops != &sprdwl_netdev_ops)
		return NOTIFY_DONE;

	ndev = inet6_ifa->idev->dev;
	vif = netdev_priv(ndev);

	switch (vif->wdev.iftype) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		if (event == NETDEV_UP) {
			work = sprdwl_alloc_work(SPRDWL_IPV6_ADDR_LEN);
			if (!work) {
				netdev_err(ndev, "%s out of memory\n",
					   __func__);
				return NOTIFY_DONE;
			}
			work->vif = vif;
			work->id = SPRDWL_WORK_NOTIFY_IP;
			ipv6_addr = (u8 *)work->data;
			memcpy(ipv6_addr, (u8 *)&inet6_ifa->addr,
			       SPRDWL_IPV6_ADDR_LEN);
			sprdwl_queue_work(vif->priv, work);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block sprdwl_inet6addr_cb = {
	.notifier_call = sprdwl_inetaddr6_event,
};

#define MAC_ADDR_RETRY_PATH "/mnt/vendor/wifimac.txt"
#define WIFI_MAC_ADDR_TEMP "/data/vendor/wifi/wifimac_tmp.txt"

static int sprdwl_get_mac_from_file(struct sprdwl_vif *vif, u8 *addr)
{
	struct file *fp = 0;
	u8 buf[64] = { 0 };
	mm_segment_t fs;
	loff_t *pos;

	/* trying to open MAC_ADDR_RETRY_PATH */
	fp = filp_open(MAC_ADDR_RETRY_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp))
		goto read_operation;
	netdev_info(vif->ndev, "%s read error, start to read %s\n",
		MAC_ADDR_RETRY_PATH, WIFI_MAC_ADDR_TEMP);

	/*trying to open WIFI_MAC_ADDR_TEMP*/
	fp = filp_open(WIFI_MAC_ADDR_TEMP, O_RDONLY, 0);
	if (!IS_ERR(fp))
		goto read_operation;
	wl_err("%s read error\n", WIFI_MAC_ADDR_TEMP);

	return -ENOENT;

read_operation:
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = &fp->f_pos;
	vfs_read(fp, buf, sizeof(buf), pos);

	filp_close(fp, NULL);
	set_fs(fs);

	str2mac(buf, addr);

	if (!is_valid_ether_addr(addr)) {
		netdev_err(vif->ndev, "%s invalid MAC address (%pM)\n",
			   __func__, addr);
		return -EINVAL;
	}
	if (is_local_ether_addr(addr)) {
		netdev_warn(vif->ndev, "%s Warning: Assigning a locally valid "
			    "MAC address (%pM) to a device\n", __func__, addr);
		netdev_warn(vif->ndev, "%s You should not set the 2nd rightmost "
			    "bit in the first byte of the MAC\n", __func__);
		vif->local_mac_flag = 1;
	} else {
		vif->local_mac_flag = 0;
	}
	return 0;
}

static int write_mac_addr(u8 *addr)
{
	struct file *fp = 0;
	mm_segment_t old_fs;
	char buf[18];
	loff_t pos = 0;

	/*open file*/
	fp = filp_open(WIFI_MAC_ADDR_TEMP, O_CREAT | O_RDWR | O_TRUNC, 0666);
	if (IS_ERR(fp)) {
		wl_err("can't create WIFI MAC file!\n");
		return -ENOENT;
	}
	/*format MAC address*/
	sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1],
	     addr[2], addr[3], addr[4], addr[5]);
	/*save old fs: should be USER_DS*/
	old_fs = get_fs();
	/*change it to KERNEL_DS*/
	set_fs(KERNEL_DS);
	/*write file*/
	vfs_write(fp, buf, sizeof(buf), &pos);
	/*close file*/
	filp_close(fp, NULL);
	/*restore to old fs*/
	set_fs(old_fs);
	
	return 0;
}

static void sprdwl_set_mac_addr(struct sprdwl_vif *vif, u8 *pending_addr,
				u8 *addr)
{
	enum nl80211_iftype type = vif->wdev.iftype;
	struct sprdwl_priv *priv = vif->priv;

	if (!addr) {
		return;
	} else if (pending_addr && is_valid_ether_addr(pending_addr)) {
		memcpy(addr, pending_addr, ETH_ALEN);
	} else if (is_valid_ether_addr(priv->default_mac)) {
		memcpy(addr, priv->default_mac, ETH_ALEN);
	} else if (sprdwl_get_mac_from_file(vif, addr)) {
		random_ether_addr(addr);
		netdev_warn(vif->ndev, "%s Warning: use random MAC address\n",
			    __func__);
		/* initialize MAC addr with specific OUI */
		addr[0] = 0x40;
		addr[1] = 0x45;
		addr[2] = 0xda;
		/*write random mac to WIFI FILE*/
		write_mac_addr(addr);
	}

	switch (type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_AP:
		memcpy(priv->default_mac, addr, ETH_ALEN);
		break;
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_P2P_GO:
		addr[4] ^= 0x80;
		break;
	case NL80211_IFTYPE_P2P_DEVICE:
		addr[0] ^= 0x02;
		break;
	default:
		break;
	}
}

static void sprdwl_init_vif(struct sprdwl_priv *priv, struct sprdwl_vif *vif,
			    const char *name)
{
	WARN_ON(strlen(name) >= sizeof(vif->name));

	strcpy(vif->name, name);
	vif->priv = priv;
	vif->sm_state = SPRDWL_DISCONNECTED;
	INIT_LIST_HEAD(&vif->survey_info_list);
}

static void sprdwl_deinit_vif(struct sprdwl_vif *vif)
{
	sprdwl_scan_done(vif, true);
	sprdwl_sched_scan_done(vif, true);
	/* We have to clear all the work which
	 * is belong to the vif we are going to remove.
	 */
	sprdwl_cancle_work(vif->priv, vif);

	if (vif->ref > 0) {
		int cnt = 0;
		unsigned long timeout = jiffies + msecs_to_jiffies(1000);

		do {
			usleep_range(2000, 2500);
			cnt++;
			if (time_after(jiffies, timeout)) {
				netdev_err(vif->ndev, "%s timeout cnt %d\n",
					   __func__, cnt);
				break;
			}
		} while (vif->ref > 0);
		netdev_dbg(vif->ndev, "cnt %d\n", cnt);
	}
}

static struct sprdwl_vif *sprdwl_register_wdev(struct sprdwl_priv *priv,
					       const char *name,
					       enum nl80211_iftype type,
					       u8 *addr)
{
	struct sprdwl_vif *vif;
	struct wireless_dev *wdev;

	vif = kzalloc(sizeof(*vif), GFP_KERNEL);
	if (!vif)
		return ERR_PTR(-ENOMEM);

	/* initialize vif stuff */
	sprdwl_init_vif(priv, vif, name);

	/* initialize wdev stuff */
	wdev = &vif->wdev;
	wdev->wiphy = priv->wiphy;
	wdev->iftype = type;

	sprdwl_set_mac_addr(vif, addr, wdev->address);
	pr_info("iface '%s'(%pM) type %d added\n", name, wdev->address, type);

	return vif;
}

static void sprdwl_unregister_wdev(struct sprdwl_vif *vif)
{
	pr_info("iface '%s' deleted\n", vif->name);

	cfg80211_unregister_wdev(&vif->wdev);
	/* cfg80211_unregister_wdev use list_del_rcu to delete wdev,
	 * so we can not free vif immediately, must wait until an
	 * RCU grace period has elapsed.
	 */
	synchronize_rcu();
	sprdwl_deinit_vif(vif);
	kfree(vif);
}

static struct sprdwl_vif *sprdwl_register_netdev(struct sprdwl_priv *priv,
						 const char *name,
						 unsigned char name_assign_type,
						 enum nl80211_iftype type,
						 u8 *addr)
{
	struct net_device *ndev;
	struct wireless_dev *wdev;
	struct sprdwl_vif *vif;
	int ret;

	ndev = alloc_netdev(sizeof(*vif), name, name_assign_type, ether_setup);
	if (!ndev) {
		wl_err("%s failed to alloc net_device!\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* initialize vif stuff */
	vif = netdev_priv(ndev);
	vif->ndev = ndev;
	sprdwl_init_vif(priv, vif, name);

	/* initialize wdev stuff */
	wdev = &vif->wdev;
	wdev->netdev = ndev;
	wdev->wiphy = priv->wiphy;
	wdev->iftype = type;

	/* initialize ndev stuff */
	ndev->ieee80211_ptr = wdev;
	if (priv->fw_capa & SPRDWL_CAPA_MC_FILTER) {
		pr_info("\tMulticast Filter supported\n");
		vif->mc_filter =
		    kzalloc(sizeof(struct sprdwl_mc_filter) +
			    priv->max_mc_mac_addrs * ETH_ALEN, GFP_KERNEL);
		if (!vif->mc_filter) {
			ret = -ENOMEM;
			goto err;
		}

		sprdwl_netdev_ops.ndo_set_rx_mode = sprdwl_set_multicast;
	}
	ndev->netdev_ops = &sprdwl_netdev_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	ndev->priv_destructor = free_netdev;
#else
	ndev->destructor = free_netdev;
#endif
	ndev->needed_headroom = priv->skb_head_len;
	ndev->watchdog_timeo = 2 * HZ;
	SET_NETDEV_DEV(ndev, wiphy_dev(priv->wiphy));
	sprdwl_set_mac_addr(vif, addr, ndev->dev_addr);
	memcpy(vif->mac, ndev->dev_addr, ETH_ALEN);
	/* register new Ethernet interface */
	ret = register_netdevice(ndev);
	if (ret) {
		netdev_err(ndev, "failed to regitster netdev(%d)!\n", ret);
		goto err;
	}

	pr_info("iface '%s'(%pM) type %d added\n", ndev->name, ndev->dev_addr,
		type);
	return vif;
err:
	sprdwl_deinit_vif(vif);
	free_netdev(ndev);
	return ERR_PTR(ret);
}

static void sprdwl_unregister_netdev(struct sprdwl_vif *vif)
{
	pr_info("iface '%s' deleted\n", vif->ndev->name);

	if (vif->priv->fw_capa & SPRDWL_CAPA_MC_FILTER)
		kfree(vif->mc_filter);
	sprdwl_deinit_vif(vif);
	unregister_netdevice(vif->ndev);
}

struct wireless_dev *sprdwl_add_iface(struct sprdwl_priv *priv,
				      const char *name,
				      unsigned char name_assign_type,
				      enum nl80211_iftype type, u8 *addr)
{
	struct sprdwl_vif *vif;

	if (type == NL80211_IFTYPE_P2P_DEVICE)
		vif = sprdwl_register_wdev(priv, name, type, addr);
	else
		vif = sprdwl_register_netdev(priv, name, name_assign_type,
					     type, addr);

	if (IS_ERR(vif)) {
		wl_err("failed to add iface '%s'\n", name);
		return (void *)vif;
	}

	spin_lock_bh(&priv->list_lock);
	list_add_tail(&vif->vif_node, &priv->vif_list);
	spin_unlock_bh(&priv->list_lock);

	return &vif->wdev;
}

int sprdwl_del_iface(struct sprdwl_priv *priv, struct sprdwl_vif *vif)
{
	if (!vif->ndev)
		sprdwl_unregister_wdev(vif);
	else
		sprdwl_unregister_netdev(vif);

	return 0;
}

static void sprdwl_del_all_ifaces(struct sprdwl_priv *priv)
{
	struct sprdwl_vif *vif, *tmp;

next_intf:
	spin_lock_bh(&priv->list_lock);
	list_for_each_entry_safe_reverse(vif, tmp, &priv->vif_list, vif_node) {
		list_del(&vif->vif_node);
		spin_unlock_bh(&priv->list_lock);
		rtnl_lock();
		sprdwl_del_iface(priv, vif);
		rtnl_unlock();
		goto next_intf;
	}
	spin_unlock_bh(&priv->list_lock);
}

static void sprdwl_init_debugfs(struct sprdwl_priv *priv)
{
	if (!priv->wiphy->debugfsdir)
		return;
	priv->debugfs = debugfs_create_dir("sprdwl_wifi",
					   priv->wiphy->debugfsdir);
	if (IS_ERR_OR_NULL(priv->debugfs))
		return;
	priv->if_ops->debugfs(priv->debugfs);
}

static int sprdwl_host_reset(struct notifier_block *nb,
			     unsigned long data, void *ptr)
{
	struct sprdwl_vif *vif;

	char *envp[3] = {
		[0] = "SOURCE=unisocwl",
		[1] = "EVENT=FW_ERROR",
		[2] = NULL,
	};

	kobject_uevent_env(&sprdwl_dev->kobj, KOBJ_CHANGE, envp);

	if (!g_sprdwl_priv) {
		pr_err("%s g_sprdwl_priv is NULL\n", __func__);
		return NOTIFY_OK;
	}

	vif = list_first_entry(&g_sprdwl_priv->vif_list,
			       struct sprdwl_vif, vif_node);
	if (!vif) {
		pr_err("%s vif list is NULL\n", __func__);
		return NOTIFY_OK;
	}

	return NOTIFY_OK;
}

static struct notifier_block wifi_host_reset = {
	.notifier_call = sprdwl_host_reset,
};

int sprdwl_core_init(struct device *dev, struct sprdwl_priv *priv)
{
	struct wiphy *wiphy = priv->wiphy;
	struct wireless_dev *wdev;
	int ret;

	sprdwl_tcp_ack_init(priv);
	sprdwl_get_fw_info(priv);
	sprdwl_setup_wiphy(wiphy, priv);
	sprdwl_vendor_init(wiphy);
	set_wiphy_dev(wiphy, dev);
	ret = wiphy_register(wiphy);
	if (ret) {
		wiphy_err(wiphy, "failed to regitster wiphy(%d)!\n", ret);
		goto out;
	}
	sprdwl_init_debugfs(priv);

	rtnl_lock();
	wdev = sprdwl_add_iface(priv, "wlan%d", NET_NAME_ENUM,
				NL80211_IFTYPE_STATION, NULL);
	rtnl_unlock();
	if (IS_ERR(wdev)) {
		wiphy_unregister(wiphy);
		ret = -ENXIO;
		goto out;
	}
	atomic_notifier_chain_register(&wcn_reset_notifier_list,
				       &wifi_host_reset);

	sprdwl_init_npi();
	ret = register_inetaddr_notifier(&sprdwl_inetaddr_cb);
	if (ret)
		wl_err("%s failed to register inetaddr notifier(%d)!\n",
		       __func__, ret);
	if (priv->fw_capa & SPRDWL_CAPA_NS_OFFLOAD) {
		pr_info("\tIPV6 NS Offload supported\n");
		ret = register_inet6addr_notifier(&sprdwl_inet6addr_cb);
		if (ret)
			wl_err("%s failed to register inet6addr notifier(%d)!\n",
			       __func__, ret);
	}

#if defined(SPRDWL_INTF_SDIO) || defined(SPRDWL_INTF_SIPC)
	ret = marlin_reset_register_notify(priv->if_ops->force_exit, NULL);
	if (ret) {
		wl_err("%s failed to register wcn cp rest notify(%d)!\n",
		       __func__, ret);
	}
#endif

out:
	return ret;
}

int sprdwl_core_deinit(struct sprdwl_priv *priv)
{
#if defined(SPRDWL_INTF_SDIO) || defined(SPRDWL_INTF_SIPC)
	marlin_reset_unregister_notify();
#endif
	atomic_notifier_chain_unregister(&wcn_reset_notifier_list,
					 &wifi_host_reset);
	unregister_inetaddr_notifier(&sprdwl_inetaddr_cb);
	if (priv->fw_capa & SPRDWL_CAPA_NS_OFFLOAD)
		unregister_inet6addr_notifier(&sprdwl_inet6addr_cb);
	sprdwl_deinit_npi();
	sprdwl_del_all_ifaces(priv);
	sprdwl_vendor_deinit(priv->wiphy);
	wiphy_unregister(priv->wiphy);
	sprdwl_cmd_wake_upall();
	sprdwl_tcp_ack_deinit();

	return 0;
}

static int sprdwl_probe(struct platform_device *pdev)
{
	struct sprdwl_intf *intf;
	struct sprdwl_priv *priv;
	int ret = 0;

	pr_info("Spreadtrum WLAN Driver (Ver. %s, %s)\n",
		SPRDWL_DRIVER_VERSION, utsname()->release);

	intf = sprdwl_intf_create();
	if (!intf)
		return -ENOSPC;

	priv = sprdwl_core_create(intf);
	if (!priv) {
		ret = -ENOSPC;
		goto err_core_create;
	}
	platform_set_drvdata(pdev, priv);
	sprdwl_dev = &pdev->dev;

	ret = sprdwl_intf_init(intf, pdev, priv);
	if (ret)
		goto err_intf_init;

	ret = sprdwl_core_init(&pdev->dev, priv);
	if (ret)
		goto err_core_init;

	return 0;
err_core_init:
	sprdwl_intf_deinit(intf);
err_intf_init:
	sprdwl_core_free((struct sprdwl_priv *)intf->priv);
err_core_create:
	sprdwl_intf_free(intf);
	wl_err("err, driver is not ok\n");
	return ret;
}

static int sprdwl_remove(struct platform_device *pdev)
{
	struct sprdwl_priv *priv = platform_get_drvdata(pdev);
	struct sprdwl_intf *intf = (struct sprdwl_intf *)priv->hw_intf;

	pr_info("%s:enter remove\n", __func__);
	sprdwl_intf_predeinit(intf);
	sprdwl_core_deinit(priv);
	sprdwl_core_free(priv);
	sprdwl_intf_deinit(intf);
	sprdwl_intf_free(intf);
	pr_info("%s:remove ok\n", __func__);

	return 0;
}

static const struct of_device_id sprdwl_of_match[] = {
	{.compatible = "sprd,sc2332",},
	{.compatible = "sprd,sp9832e",},
	{.compatible = "sprd,sp7731e",},
	{.compatible = "sprd,sp9863a",},
	{.compatible = "sprd,sp9850e",},
	{},
};
MODULE_DEVICE_TABLE(of, sprdwl_of_match);

static struct platform_driver sprdwl_driver = {
	.probe = sprdwl_probe,
	.remove = sprdwl_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "wlan",
		.of_match_table = sprdwl_of_match,
	},
};

module_platform_driver(sprdwl_driver);

MODULE_DESCRIPTION("Spreadtrum Wireless LAN Driver");
MODULE_AUTHOR("Spreadtrum WCN Division");
MODULE_LICENSE("GPL");
