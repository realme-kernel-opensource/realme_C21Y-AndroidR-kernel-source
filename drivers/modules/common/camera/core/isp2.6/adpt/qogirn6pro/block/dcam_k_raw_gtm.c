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
#include "cam_types.h"
#include "cam_block.h"
#include "dcam_core.h"


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "GTM: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


void dcam_k_raw_gtm_set_default(struct dcam_dev_raw_gtm_block_info *p)
{
	p->gtm_tm_out_bit_depth = 0xE;
	p->gtm_tm_in_bit_depth = 0xE;
	p->gtm_cur_is_first_frame = 0x0;
	p->gtm_log_diff = 0x0;
	p->gtm_log_diff_int = 0x25C9;
	p->gtm_log_max_int = 0x0;
	p->gtm_log_min_int = 0x496B;
	p->gtm_lr_int = 0x23F;
	p->gtm_tm_param_calc_by_hw = 0x1;
	p->gtm_yavg = 0x0;
	p->gtm_ymax = 0x0;
	p->gtm_ymin = 0x4;
	p->tm_lumafilter_c[0][0] = 0x4;
	p->tm_lumafilter_c[0][1] = 0x2;
	p->tm_lumafilter_c[0][2] = 0x4;
	p->tm_lumafilter_c[1][0] = 0x2;
	p->tm_lumafilter_c[1][1] = 0x28;
	p->tm_lumafilter_c[1][2] = 0x2;
	p->tm_lumafilter_c[2][0] = 0x4;
	p->tm_lumafilter_c[2][1] = 0x2;
	p->tm_lumafilter_c[2][2] = 0x4;
	p->tm_lumafilter_shift = 0x6;
	p->slice.gtm_slice_line_startpos = 0x0;
	p->slice.gtm_slice_line_endpos = 0x0;
	p->slice.gtm_slice_main = 0x0;
}

int dcam_k_raw_gtm_slice(uint32_t idx, struct dcam_dev_gtm_slice_info *gtm_slice)
{
	int ret = 0;
	unsigned int val = 0;

	/* for slice */
	val =((gtm_slice->gtm_slice_main & 0x1) <<7);
	DCAM_REG_MWR(idx, DCAM_GTM_GLB_CTRL, 0x80, val);

	val = (gtm_slice->gtm_slice_line_startpos & 0x1FFF);
	DCAM_REG_WR(idx, GTM_SLICE_LINE_STARTPOS, val);

	val = (gtm_slice->gtm_slice_line_endpos & 0x1FFF);
	DCAM_REG_WR(idx, GTM_SLICE_LINE_ENDPOS, val);

	return ret;
}

int dcam_k_raw_gtm_block(uint32_t gtm_param_idx,
	struct dcam_dev_param *param)
{
	int ret = 0;
	unsigned int i = 0;
	uint32_t idx = param->idx;
	unsigned int val = 0;
	struct dcam_dev_raw_gtm_block_info *p;
	struct dcam_dev_gtm_slice_info *gtm_slice;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_dev_gtm_param *gtm = NULL;

	gtm = &param->gtm[gtm_param_idx];
	if (!gtm->update_en)
                return 0;

	dev = (struct dcam_pipe_dev *)param->dev;
	p = &(gtm->gtm_info);
	dcam_k_raw_gtm_set_default(p);

	if (g_dcam_bypass[idx] & (1 << _E_GTM))
		p->gtm_mod_en = 0;
	gtm_slice = &(p->slice);
	DCAM_REG_MWR(idx, DCAM_GTM_GLB_CTRL, BIT_0, (p->gtm_mod_en & 0x1));
	if (!p->gtm_mod_en)
		return 0;

	if (atomic_read(&dev->state) != STATE_RUNNING)
		p->gtm_cur_is_first_frame = 1;

	val =((p->gtm_map_bypass & 0x1) <<1) |
		((p->gtm_hist_stat_bypass & 0x1) <<2) |
		((p->gtm_tm_param_calc_by_hw & 0x1) <<3) |
		((p->gtm_cur_is_first_frame &0x1)<<4) |
		((p->gtm_tm_luma_est_mode & 0x3)<<5) |
		((p->gtm_tm_in_bit_depth & 0xF) << 24) |
		((p->gtm_tm_out_bit_depth & 0xF) << 28);
	DCAM_REG_MWR(idx, DCAM_GTM_GLB_CTRL, 0xFF00007E, val);

	val = (p->gtm_imgkey_setting_mode & 0x1) | ((p->gtm_imgkey_setting_value & 0x7FFF) << 4);
	DCAM_REG_MWR(idx, GTM_HIST_CTRL0, 0x7FFF1, val);

	val = (p->gtm_target_norm_setting_mode & 0x1)
		| ((p->gtm_target_norm & 0x3FFF) << 2)
		| ((p->gtm_target_norm_coeff & 0x3FFF) << 16);
	DCAM_REG_MWR(idx, GTM_HIST_CTRL1, 0x3FFFFFFD, val);

	val = p->gtm_yavg_diff_thr & 0x3FFF;
	DCAM_REG_WR(idx, GTM_HIST_CTRL2, val);

	p->gtm_hist_total = dev->cap_info.cap_size.size_x * dev->cap_info.cap_size.size_y;
	val = p->gtm_hist_total & 0x3FFFFFF;
	DCAM_REG_WR(idx, GTM_HIST_CTRL5, val);

	val = ((p->gtm_min_per * p->gtm_hist_total) >> 16) & 0xFFFFF;
	DCAM_REG_WR(idx, GTM_HIST_CTRL6, val);

	val = ((p->gtm_max_per * p->gtm_hist_total) >> 16) & 0xFFFFF;
	DCAM_REG_WR(idx, GTM_HIST_CTRL7, val);

	val = p->gtm_log_diff & 0x1FFFFFFF;
	DCAM_REG_WR(idx, GTM_LOG_DIFF, val);

	val = (p->gtm_ymax_diff_thr & 0x3FFF)
		| ((p->gtm_cur_ymin_weight & 0x1FF) << 14)
		| ((p->gtm_pre_ymin_weight & 0x1FF) << 23);
	DCAM_REG_WR(idx, GTM_TM_YMIN_SMOOTH, val);

	val = (p->tm_lumafilter_c[0][0] & 0xFF)
		| ((p->tm_lumafilter_c[0][1] & 0xFF) << 8)
		| ((p->tm_lumafilter_c[0][2] & 0xFF) << 16)
		| ((p->tm_lumafilter_c[1][0] & 0xFF) << 24);
	DCAM_REG_WR(idx, GTM_TM_LUMAFILTER0, val);

	val = (p->tm_lumafilter_c[1][1] & 0xFF)
		| ((p->tm_lumafilter_c[1][2] & 0xFF) << 8)
		| ((p->tm_lumafilter_c[2][0] & 0xFF) << 16)
		| ((p->tm_lumafilter_c[2][1] & 0xFF) << 24);
	DCAM_REG_WR(idx, GTM_TM_LUMAFILTER1, val);

	val = (p->tm_lumafilter_c[2][2] & 0xFF) | ((p->tm_lumafilter_shift & 0xF) << 28);
	DCAM_REG_MWR(idx, GTM_TM_LUMAFILTER2, 0xF00000FF, val);

	val = (p->tm_rgb2y_r_coeff & 0x7FF) | ((p->tm_rgb2y_g_coeff & 0x7FF) << 16);
	DCAM_REG_MWR(idx, GTM_TM_RGB2YCOEFF0, 0x7FF07FF, val);

	val = (p->tm_rgb2y_b_coeff & 0x7FF);
	DCAM_REG_MWR(idx, GTM_TM_RGB2YCOEFF1, 0x7FF, val);

	for (i = 0; i < GTM_HIST_XPTS_CNT / 2; i+=2) {
		val = ((p->tm_hist_xpts[i] & 0x3FFF) << 16) | (p->tm_hist_xpts[i+1] & 0x3FFF);
		DCAM_REG_WR(idx, GTM_HIST_XPTS + i*2, val);
	}

	/* for slice */
	dcam_k_raw_gtm_slice(idx, gtm_slice);

	return ret;
}

int dcam_k_cfg_raw_gtm(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;
	struct dcam_dev_gtm_param *gtm = NULL;

	switch (param->property) {
	case DCAM_PRO_RAW_GTM_BLOCK:
		if (param->scene_id == PM_SCENE_CAP)
			gtm = &p->gtm[DCAM_GTM_PARAM_CAP];
		else
			gtm = &p->gtm[DCAM_GTM_PARAM_PRE];
		/* online mode not need mutex, response faster
		 * Offline need mutex to protect param
		 */
		if (p->offline == 0) {
			ret = copy_from_user((void *)&(gtm->gtm_info),
				param->property_param,
				sizeof(gtm->gtm_info));
			if (ret) {
				pr_err("fail to copy, ret=0x%x\n", (unsigned int)ret);
				return -EPERM;
			}
			if (param->scene_id != PM_SCENE_CAP)
				dcam_k_raw_gtm_block(DCAM_GTM_PARAM_PRE, p);
		} else {
			mutex_lock(&p->param_lock);
			ret = copy_from_user((void *)&(gtm->gtm_info),
				param->property_param,
				sizeof(gtm->gtm_info));
			if (ret) {
				mutex_unlock(&p->param_lock);
				pr_err("fail to copy, ret=0x%x\n", (unsigned int)ret);
				return -EPERM;
			}
			mutex_unlock(&p->param_lock);
		}
		break;
	default:
		pr_err("fail cmd id:%d, not supported.\n",
			param->property);
		break;
	}
	return ret;
}

