/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
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

#include <linux/uaccess.h>
#include <sprd_mm.h>

#include "isp_hw.h"
#include "isp_reg.h"
#include "cam_types.h"
#include "cam_block.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CSA: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_csa_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_csa_info *csa_info;

	csa_info = &isp_k_param->csa_info;

	ret = copy_from_user((void *)csa_info,
			param->property_param,
			sizeof(struct isp_dev_csa_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -EPERM;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_SATURATION))
		csa_info->bypass = 1;
	if (csa_info->bypass)
		return 0;

	val = ((csa_info->csa_factor_v & 0xFF) << 8) |
		((csa_info->csa_factor_u & 0xFF) << 16);
	ISP_REG_MWR(idx, ISP_CSA_PARAM, 0xFFFF00, val);

	return ret;
}

int isp_k_cfg_csa(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	if (!param) {
		pr_err("fail to get param\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_CSA_BLOCK:
		ret = isp_k_csa_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
