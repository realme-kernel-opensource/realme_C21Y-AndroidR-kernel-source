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
#include "cam_block.h"
#include "isp_ltm.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "LTM MAP: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int isp_ltm_config_param(struct isp_ltm_ctx_desc *ctx,
		enum isp_ltm_region ltm_id)
{
	return 0;
}
