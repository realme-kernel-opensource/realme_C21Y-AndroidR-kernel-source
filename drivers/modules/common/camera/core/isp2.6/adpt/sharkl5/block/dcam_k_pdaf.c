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
#include "dcam_core.h"
#include "dcam_reg.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "PDAF: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


static void write_pd_table(struct pdaf_ppi_info *pdaf_info, enum dcam_id idx)
{
	int i = 0;
	int col = 0, row = 0, is_right = 0;
	int pd_num = 0;
	int reg_pos = 0;
	uint32_t pdafinfo[32] = {0};

	pd_num = pdaf_info->pd_pos_size * 2;
	for (i = 0; i < pd_num; i++) {
		col = pdaf_info->pattern_pixel_col[i];
		row = pdaf_info->pattern_pixel_row[i];
		is_right = pdaf_info->pattern_pixel_is_right[i] & 0x01;
		pr_debug("col %d, row %d, right %d\n", col, row, is_right);
		if (i%2 == 0) {
			pdafinfo[reg_pos] = col | (row << 6) | (is_right << 12);
		} else if (i % 2 == 1) {
			pdafinfo[reg_pos] |= (col << 16) | (row << 22) | (is_right << 28);
			reg_pos++;
		}
	}

	DCAM_REG_MWR(idx, ISP_PPI_PARAM, 0x007f0000, pd_num << 16);

	for (i = 0; i < pd_num/2; i++)
		DCAM_REG_MWR(idx, ISP_PPI_PATTERN01 + (i * 4), 0x1fff1fff, pdafinfo[i]);
}

static int isp_k_pdaf_type1_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	struct dev_dcam_vc2_control vch2_info;

	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}

static int isp_k_pdaf_type2_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	struct dev_dcam_vc2_control vch2_info;

	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}
static int isp_k_pdaf_type3_block(struct isp_io_param *param, void *in)
{
	int ret = 0;
	enum dcam_id idx;
	struct dev_dcam_vc2_control vch2_info;
	struct dcam_dev_param *p = (struct dcam_dev_param *)in;
	struct dcam_pipe_dev  *dev = (struct dcam_pipe_dev *)p->dev;

	idx = dev->idx;
	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	pr_debug("pdaf_info.vch2_mode = %d\n", vch2_info.vch2_mode);
	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}

static int isp_k_dual_pdaf_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	struct dev_dcam_vc2_control vch2_info;

	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}

static int isp_k_pdaf_type3_set_info(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_pdaf_info pdaf_info;
	struct pdaf_ppi_info ppi_info;

	memset(&pdaf_info, 0x00, sizeof(pdaf_info));
	memset(&ppi_info, 0x00, sizeof(ppi_info));
	ret = copy_from_user((void *)&pdaf_info,
		param->property_param, sizeof(pdaf_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	/* phase map corr en */
	val = pdaf_info.phase_map_corr_en;
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_3, val << 3);

	/* ppi grid mode */
	val = pdaf_info.grid_mode;
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_8, val << 8);

	return ret;
}

static int isp_k_pdaf_bypass(
	struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	bypass = !!bypass;
	p->pdaf.bypass = bypass;
	pr_info("dcam%d pdaf bypass %d\n", p->idx, bypass);

	return ret;
}

static int isp_k_pdaf_type3_set_ppi_info(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct pdaf_ppi_info pdaf_info;

	memset(&pdaf_info, 0x00, sizeof(pdaf_info));
	ret = copy_from_user((void *)&pdaf_info,
		param->property_param, sizeof(pdaf_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	pr_debug("idx %d, block area: (%d, %d) (%d, %d), block.w/h: %d, %d, ppi bypass %d\n", idx,
		pdaf_info.block.start_x,
		pdaf_info.block.start_y,
		pdaf_info.block.end_x,
		pdaf_info.block.end_y,
		pdaf_info.block_size.width,
		pdaf_info.block_size.height,
		pdaf_info.bypass);

	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_0, pdaf_info.bypass);
	if (pdaf_info.bypass)
		return 0;

	write_pd_table(&pdaf_info, idx);

	/* ppi block col&row start,end */
	val = pdaf_info.block.start_x
		| pdaf_info.block.end_x << 16;
	DCAM_REG_MWR(idx, ISP_PPI_BLOCK_COL, 0xffffffff, val);
	val = 0;
	val = pdaf_info.block.start_y
		| pdaf_info.block.end_y << 16;
	DCAM_REG_MWR(idx, ISP_PPI_BLOCK_ROW, 0xffffffff, val);

	val = pdaf_info.block.start_y
		| pdaf_info.block.end_y << 16;
	DCAM_REG_WR(idx, ISP_ZZBPC_PPI_RANG, val);

	val = pdaf_info.block.start_x
		| pdaf_info.block.end_x << 16;
	DCAM_REG_WR(idx, ISP_ZZBPC_PPI_RANG1, val);

	/* ppi block w*h */
	val = 0;
	val = (pdaf_info.block_size.width<<4)
		| (pdaf_info.block_size.height << 6);
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, 0x000000f0, val);

	return ret;
}

static int isp_k_pdaf_type3_set_roi(struct isp_io_param *param, enum dcam_id idx)
{

	int ret = 0;
	unsigned int val = 0;
	struct pdaf_roi_info roi_info;

	pr_debug("E\n");
	memset(&roi_info, 0x00, sizeof(roi_info));
	ret = copy_from_user((void *)&roi_info,
		param->property_param, sizeof(roi_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	pr_debug("win start:(%d, %d) end:(%d, %d)\n", roi_info.win.start_x,
		roi_info.win.start_y, roi_info.win.end_x, roi_info.win.end_y);
	val = ((roi_info.win.start_y & 0xFFFF) << 16) |
		(roi_info.win.start_x & 0xFFFF);
	DCAM_REG_WR(idx, ISP_PPI_AF_WIN_START, val);

	val = ((roi_info.win.end_y & 0xFFFF) << 16) |
		(roi_info.win.end_x & 0xFFFF);
	DCAM_REG_WR(idx, ISP_PPI_AF_WIN_END, val);

	return ret;
}

static int isp_k_pdaf_type3_set_skip_num(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	unsigned int skip_num = 0;

	ret = copy_from_user((void *)&skip_num,
			param->property_param, sizeof(skip_num));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, 0xF0,
		skip_num << 4);
	DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL1, BIT_1, BIT_1);

	return ret;
}

static int isp_k_pdaf_type3_set_mode(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	uint32_t mode = 0;

	ret = copy_from_user((void *)&mode,
			param->property_param, sizeof(mode));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	/* single/multi mode */
	mode = !!mode;
	DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, BIT_2, mode << 2);
	DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, BIT_3, mode << 3);

	return ret;
}

int dcam_k_cfg_pdaf(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;
	enum dcam_id idx;
	struct dcam_pipe_dev *dev = NULL;

	if (!param || !p) {
		pr_err("fail to get param\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -1;
	}

	dev = (struct dcam_pipe_dev *)p->dev;
	idx = p->idx;
	switch (param->property) {
	case DCAM_PDAF_BYPASS:
		ret = isp_k_pdaf_bypass(param, p);
		break;
	case DCAM_PDAF_TYPE3_SET_INFO:
		ret = isp_k_pdaf_type3_set_info(param, idx);
		break;
	case DCAM_PDAF_TYPE3_SET_MODE:
		ret = isp_k_pdaf_type3_set_mode(param, idx);
		break;
	case DCAM_PDAF_TYPE3_SET_SKIP_NUM:
		ret = isp_k_pdaf_type3_set_skip_num(param, idx);
		break;
	case DCAM_PDAF_TYPE3_SET_ROI:
		ret = isp_k_pdaf_type3_set_roi(param, idx);
		break;
	case DCAM_PDAF_TYPE3_SET_PPI_INFO:
		ret = isp_k_pdaf_type3_set_ppi_info(param, idx);
		break;
	case DCAM_PDAF_TYPE1_BLOCK:
		p->pdaf.pdaf_type = 1;
		ret = isp_k_pdaf_type1_block(param, idx);
		break;
	case DCAM_PDAF_TYPE2_BLOCK:
		p->pdaf.pdaf_type = 2;
		ret = isp_k_pdaf_type2_block(param, idx);
		break;
	case DCAM_PDAF_TYPE3_BLOCK:
		p->pdaf.pdaf_type = 3;
		ret = isp_k_pdaf_type3_block(param, p);
		break;
	case DCAM_DUAL_PDAF_BLOCK:
		p->pdaf.pdaf_type = 0;
		ret = isp_k_dual_pdaf_block(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}
	pr_info("idx %d, Sub %d, ret %d\n", idx, param->property, ret);

	return ret;
}
