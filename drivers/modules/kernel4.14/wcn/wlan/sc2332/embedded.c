/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Authors	:
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
#include "sprdwl.h"
#include "sipc.h"
#include "cmdevt.h"

int sprdwl_handle_cmd(struct net_device *ndev, struct ifreq *ifr)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_priv *priv = vif->priv;
	struct android_wifi_priv_cmd priv_cmd;
	char *command = NULL;
	unsigned short subtype;
	int ret = 0, value;
	char max_sta_num;

	if (!ifr->ifr_data)
		return -EINVAL;
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(priv_cmd)))
		return -EINVAL;

	/*add length check to avoid invalid NULL ptr*/
	if ((!priv_cmd.total_len) || (SPRDWL_MAX_CMD_TXLEN < priv_cmd.total_len)) {
		netdev_err(ndev, "%s: priv cmd total len is invalid\n", __func__);
		return -EINVAL;
	}

	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
	    return -EINVAL;
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto out;
	}

	subtype = *(unsigned short *)command;
	if (subtype == 5) {
	value = *((int *)(command + 2 * sizeof(unsigned short)));
	netdev_info(ndev, "%s: set miracast value : %d\n",
		__func__, value);
	ret = sprdwl_enable_miracast(priv, vif->mode, value);
	} else if (subtype == 4) {
	max_sta_num = *(char *)(command + 2 * sizeof(unsigned short));
	netdev_info(ndev, "%s: set max station num : %d\n",
			__func__, max_sta_num);
	ret = sprdwl_set_max_sta(priv, vif->mode, max_sta_num);
	}

out:
	kfree(command);
	return ret;
}
