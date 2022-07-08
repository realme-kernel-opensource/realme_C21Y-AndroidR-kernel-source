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

#ifndef _COMPAT_CAM_DRV_H_
#define _COMPAT_CAM_DRV_H_

#define COMPAT_SPRD_ISP_IO_CFG_PARAM \
	_IOWR(SPRD_IMG_IO_MAGIC, 41, struct compat_isp_io_param)

struct compat_isp_io_param {
	uint32_t scene_id;
	uint32_t sub_block;
	uint32_t property;
	compat_caddr_t property_param;
};

long compat_sprd_img_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long param);

#endif/* _COMPAT_CAM_DRV_H_ */
