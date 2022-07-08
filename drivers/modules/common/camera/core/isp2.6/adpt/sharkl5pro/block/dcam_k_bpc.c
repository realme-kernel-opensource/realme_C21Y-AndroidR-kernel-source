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

#include "dcam_reg.h"
#include "dcam_interface.h"
#include "cam_block.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "BPC: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int dcam_k_bpc_block(struct dcam_dev_param *param)
{
	int ret = 0;
	uint32_t idx;
	int i = 0;
	uint32_t val = 0;
	struct dcam_dev_bpc_info *p;

	idx = param->idx;
	p = &(param->bpc.bpc_param.bpc_info);

	/* debugfs bpc not bypass then write*/
	if (g_dcam_bypass[idx] & (1 << _E_BPC))
		p->bpc_bypass = 1;

	/* following bit can be 0 only if bpc_bypss is 0 */
	if (p->bpc_bypass == 1)
		p->bpc_double_bypass = 1;
	if (p->bpc_double_bypass == 1)
		p->bpc_three_bypass = 1;
	if (p->bpc_three_bypass == 1)
		p->bpc_four_bypass = 1;

	val = (p->bpc_bypass & 0x1) |
		((p->bpc_double_bypass & 0x1) << 1) |
		((p->bpc_three_bypass & 0x1) << 2) |
		((p->bpc_four_bypass & 0x1) << 3);
	DCAM_REG_MWR(idx, ISP_BPC_PARAM, 0xF, val);
	val = DCAM_REG_RD(idx, ISP_BPC_PARAM);
	if (p->bpc_bypass)
		return 0;

	val = ((p->bpc_mode & 0x3) << 4) |
		((p->bpc_is_mono_sensor & 0x1) << 6) |
		((p->bpc_ppi_en & 0x1) << 7) |
		((p->bpc_edge_hv_mode & 0x3) << 8) |
		((p->bpc_edge_rd_mode & 0x3) << 10) |
		((p->bpc_pos_out_en & 0x1) << 16) |
		((p->bpc_map_clr_en & 0x1) << 17) |
		((p->bpc_rd_max_len_sel & 0x1) << 18) |
		((p->bpc_wr_max_len_sel & 0x1) << 19) |
		((p->bpc_blk_mode & 0x1) << 20) |
		((p->bpc_mod_en & 0x1) << 30) |
		((p->bpc_cg_dis & 0x1) << 31);
	DCAM_REG_MWR(idx, ISP_BPC_PARAM, 0xC01F0FF0, val);

	for (i = 0; i < 4; i++) {
		val = (p->bpc_four_badpixel_th[i] & 0x3FF) |
			((p->bpc_three_badpixel_th[i] & 0x3FF) << 10) |
			((p->bpc_double_badpixel_th[i] & 0x3FF) << 20);
		DCAM_REG_WR(idx, ISP_BPC_BAD_PIXEL_TH0 + i * 4, val);
	}

	val = (p->bpc_texture_th & 0x3FF) |
		((p->bpc_flat_th & 0x3FF) << 10) |
		((p->bpc_shift[2] & 0xF) << 20) |
		((p->bpc_shift[1] & 0xF) << 24) |
		((p->bpc_shift[0] & 0xF) << 28);
	DCAM_REG_WR(idx, ISP_BPC_FLAT_TH, val);

	val = (p->bpc_edgeratio_hv & 0x1FF) |
		((p->bpc_edgeratio_rd & 0x1FF) << 16);
	DCAM_REG_WR(idx, ISP_BPC_EDGE_RATIO, val);

	val = (p->bpc_highoffset & 0xFF) |
		((p->bpc_lowoffset & 0xFF) << 8) |
		((p->bpc_highcoeff & 0x7) << 16) |
		((p->bpc_lowcoeff & 0x7) << 24);
	DCAM_REG_WR(idx, ISP_BPC_BAD_PIXEL_PARAM, val);

	val = (p->bpc_mincoeff & 0x1F) |
		((p->bpc_maxcoeff & 0x1F) << 16);
	DCAM_REG_WR(idx, ISP_BPC_BAD_PIXEL_COEFF, val);

	for (i = 0; i < 8; i++) {
		val = ((p->bpc_lut_level[i] & 0x3FF) << 20) |
			((p->bpc_slope_k[i] & 0x3FF) << 10) |
			(p->bpc_intercept_b[i] & 0x3FF);
		DCAM_REG_WR(idx, ISP_BPC_LUTWORD0 + i * 4, val);
	}

	val = p->bad_pixel_num;
	DCAM_REG_WR(idx, ISP_BPC_MAP_CTRL, val);

	return ret;
}

int dcam_k_bpc_ppe_param(struct dcam_dev_param *param)
{
	int ret = 0;
	int i;
	uint32_t offset;
	uint32_t idx = param->idx;
	uint32_t val = 0;
	struct dcam_bpc_ppi_info *p;

	p = &(param->bpc.ppi_info);

	val = ((p->ppi_phase_map_corr_en & 1) << 3) | (p->ppi_bypass & 1);
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_3 | BIT_0, val);

	if (!p->ppi_phase_map_corr_en)
		return 0;

	offset = PDAF_CORR_TABLE_START;
	for (i = 0; i < PDAF_PPI_GAIN_MAP_LEN; i++) {
		val = (p->ppi_l_gain_map[i] & 0x3FFF);
		val <<= 16;
		val |= (p->ppi_r_gain_map[i] & 0x3FFF);
		DCAM_REG_WR(idx, offset, val);
		offset += 4;
	}

	return ret;
}


int dcam_k_cfg_bpc(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;
	void *dst_ptr;
	ssize_t dst_size;
	FUNC_DCAM_PARAM sub_func = NULL;

	switch (param->property) {
	case DCAM_PRO_BPC_BLOCK:
	{
		dst_ptr = (void *)&p->bpc.bpc_param.bpc_info;
		dst_size = sizeof(struct dcam_dev_bpc_info);
		sub_func = dcam_k_bpc_block;
		break;
	}
	case DCAM_PRO_BPC_PPE_PARAM:
	{
		dst_ptr = (void *)&p->bpc.ppi_info;
		dst_size = sizeof(struct dcam_bpc_ppi_info);
		sub_func = dcam_k_bpc_ppe_param;
		break;
	}
	default:
		pr_err("fail to support property %d\n",
			param->property);
		ret = -EINVAL;
		return ret;
	}

	if (p->offline == 0) {
		ret = copy_from_user(dst_ptr,
				param->property_param,
				dst_size);
		if (ret) {
			pr_err("fail to copy from user, ret = %d\n", ret);
			goto exit;
		}
		if (sub_func)
			ret = sub_func(p);
	} else {
		mutex_lock(&p->param_lock);
		ret = copy_from_user(dst_ptr,
				param->property_param,
				dst_size);
		if (ret)
			pr_err("fail to copy from user, ret = %d\n", ret);

		mutex_unlock(&p->param_lock);
	}
exit:
	return ret;
}
