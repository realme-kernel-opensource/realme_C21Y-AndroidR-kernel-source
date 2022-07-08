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
#define pr_fmt(fmt) "GAMMA: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_gamma_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	uint32_t i, buf_sel = 0;
	uint32_t buf_addr, val;
	struct isp_dev_gamma_info *gamma_info = NULL;

	gamma_info = &isp_k_param->gamma_info;

	ret = copy_from_user((void *)gamma_info,
			param->property_param,
			sizeof(struct isp_dev_gamma_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return  ret;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_GAMC))
		gamma_info->bypass = 1;
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0,
			gamma_info->bypass);
	if (gamma_info->bypass)
		return 0;

	/* only cfg mode and buf0 is selected. */
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_1, buf_sel << 1);

	buf_addr = ISP_FGAMMA_R_BUF0;
	for (i = 0; i < ISP_FRGB_GAMMA_PT_NUM - 1; i++) {
		val = gamma_info->gain_r[i];
		val = ((val << 8) | gamma_info->gain_r[i + 1]);
		ISP_REG_WR(idx, buf_addr + i * 4, val);
	}

	buf_addr = ISP_FGAMMA_G_BUF0;
	for (i = 0; i < ISP_FRGB_GAMMA_PT_NUM - 1; i++) {
		val = gamma_info->gain_g[i];
		val = ((val << 8) | gamma_info->gain_g[i + 1]);
		ISP_REG_WR(idx, buf_addr + i * 4, val);
	}

	buf_addr = ISP_FGAMMA_B_BUF0;
	for (i = 0; i < ISP_FRGB_GAMMA_PT_NUM - 1; i++) {
		val = gamma_info->gain_b[i];
		val = ((val << 8) | gamma_info->gain_b[i + 1]);
		ISP_REG_WR(idx, buf_addr + i * 4, val);
	}

	return ret;
}

int isp_k_cfg_gamma(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_GAMMA_BLOCK:
		ret = isp_k_gamma_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
