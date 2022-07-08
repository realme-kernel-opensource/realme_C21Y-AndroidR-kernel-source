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
#define pr_fmt(fmt) "CONTRAST: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_contrast_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	struct isp_dev_contrast_info *contrast_info;

	contrast_info = &isp_k_param->contrast_info;

	ret = copy_from_user((void *)contrast_info,
			param->property_param,
			sizeof(struct isp_dev_contrast_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -EPERM;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_CONTRAST))
		contrast_info->bypass = 1;
	if (contrast_info->bypass)
		return 0;

	ISP_REG_MWR(idx, ISP_CONTRAST_PARAM,
			0x1FE, (contrast_info->factor << 1));

	return ret;
}

int isp_k_cfg_contrast(struct isp_io_param *param,
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
	case ISP_PRO_CONTRAST_BLOCK:
		ret = isp_k_contrast_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
