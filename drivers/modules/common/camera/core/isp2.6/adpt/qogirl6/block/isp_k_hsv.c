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
#define pr_fmt(fmt) "HSV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


static int isp_k_hsv_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	int i = 0;
	uint32_t val = 0;
	uint32_t buf_sel = 0;
	unsigned long reg_addr = 0;
	struct isp_dev_hsv_info_v2 *hsv_info;

	hsv_info = &isp_k_param->hsv_info;

	ret = copy_from_user((void *)hsv_info,
			param->property_param,
			sizeof(struct isp_dev_hsv_info_v2));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_HSV))
		hsv_info->bypass = 1;
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, hsv_info->bypass);
	if (hsv_info->bypass)
		return 0;

	for (i = 0; i < 5; i++) {
		val = ((hsv_info->curve_info.hrange_left[i] & 0x1FF) << 23) |
			((hsv_info->curve_info.s_curve[i][1] & 0x7FF) << 11) |
			(hsv_info->curve_info.s_curve[i][0] & 0x7FF);
		ISP_REG_WR(idx, ISP_HSV_CFG0 + i * 12, val);

		val = ((hsv_info->curve_info.hrange_right[i] & 0x1FF) << 23) |
			((hsv_info->curve_info.s_curve[i][3] & 0x7FF) << 11) |
			(hsv_info->curve_info.s_curve[i][2] & 0x7FF);
		ISP_REG_WR(idx, ISP_HSV_CFG1 + i * 12, val);

		val = ((hsv_info->curve_info.v_curve[i][3] & 0xFF) << 24) |
			((hsv_info->curve_info.v_curve[i][2] & 0xFF) << 16) |
			((hsv_info->curve_info.v_curve[i][1] & 0xFF) << 8) |
			(hsv_info->curve_info.v_curve[i][0] & 0xFF);
		ISP_REG_WR(idx, ISP_HSV_CFG2 + i * 12, val);
	}

	/* new added from CFG15 ~ CFG21 */
	val = ((hsv_info->curve_info.r_s[1][0] << 20) |
			(hsv_info->curve_info.r_s[0][0] << 9) |
			(hsv_info->curve_info.r_v[0][0]));
	ISP_REG_WR(idx, ISP_HSV_CFG15, val);

	val = ((hsv_info->curve_info.r_s[3][0] << 20) |
			(hsv_info->curve_info.r_s[2][0] << 9) |
			(hsv_info->curve_info.r_v[1][0]));
	ISP_REG_WR(idx, ISP_HSV_CFG16, val);

	val = ((hsv_info->curve_info.r_v[3][0] << 20) |
			(hsv_info->curve_info.r_s[4][0] << 9) |
			(hsv_info->curve_info.r_v[2][0]));
	ISP_REG_WR(idx, ISP_HSV_CFG17, val);

	val = ((hsv_info->curve_info.r_s[1][1] << 20) |
			(hsv_info->curve_info.r_s[0][1] << 9) |
			(hsv_info->curve_info.r_v[0][1]));
	ISP_REG_WR(idx, ISP_HSV_CFG18, val);

	val = ((hsv_info->curve_info.r_s[3][1] << 20) |
			(hsv_info->curve_info.r_s[2][1] << 9) |
			(hsv_info->curve_info.r_v[1][1]));
	ISP_REG_WR(idx, ISP_HSV_CFG19, val);

	val = ((hsv_info->curve_info.r_v[3][1] << 20) |
			(hsv_info->curve_info.r_s[4][1] << 9) |
			(hsv_info->curve_info.r_v[2][1]));
	ISP_REG_WR(idx, ISP_HSV_CFG20, val);

	val = ((hsv_info->curve_info.r_v[4][1] << 9) |
			(hsv_info->curve_info.r_v[4][0]));
	ISP_REG_WR(idx, ISP_HSV_CFG21, val);

	/* only cfg mode will be supported for blocks. */
	/* cfg mode doesn't need ping-pong buffer. */
	buf_sel = 0;
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_1, buf_sel << 1);

	reg_addr = ISP_HSV_BUF0_ADDR;
	for(i = 0; i < 360; i++){
		val = ((hsv_info->d.hs.sat[i] & 0x7FF) << 9 |
				(hsv_info->d.hs.hue[i] & 0x1FF));
		ISP_REG_WR(idx, reg_addr + i * 4, val);
	}
	return ret;
}

int isp_k_cfg_hsv(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_HSV_BLOCK:
		ret = isp_k_hsv_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
