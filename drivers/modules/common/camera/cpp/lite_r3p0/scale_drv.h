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
#ifndef _SCALE_DRV_H_
#define _SCALE_DRV_H_

#include "cpp_core.h"

struct scale_drv_private {
	struct sprd_cpp_scale_cfg_parm cfg_parm;

	struct sprd_cpp_size sc_input_size;
	unsigned int sc_deci_val;

	spinlock_t *hw_lock;
	void *priv;

	struct cpp_iommu_info iommu_src;
	struct cpp_iommu_info iommu_dst;

	void __iomem *io_base;

	struct platform_device *pdev;
};

void get_cpp_max_size(unsigned int *max_width,
		unsigned int *max_height);
int cpp_scale_start(struct sprd_cpp_scale_cfg_parm *parm,
		    struct scale_drv_private *p);
void cpp_scale_stop(struct scale_drv_private *p);

int cpp_scale_capability(struct sprd_cpp_scale_capability *scale_param);

#endif /* _SCALE_DRV_H_ */
