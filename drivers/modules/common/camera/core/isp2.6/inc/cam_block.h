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
#ifndef _CAM_BLOCK_H_
#define _CAM_BLOCK_H_

#include "cam_buf.h"
#include "dcam_blkparam.h"

struct isp_k_block {
	uint32_t src_w;
	uint32_t src_h;
	struct isp_dev_nlm_info_v2 nlm_info_base;
	struct isp_dev_nlm_imblance imbalance_info_v0_base;
	struct isp_dev_nlm_imblance_v1 imbalance_info_base;
	struct isp_dev_ynr_info_v2 ynr_info_v2_base;
	struct isp_dev_3dnr_info nr3_info_base;
	uint32_t seed0_for_mode1;
	uint32_t yrandom_mode;

	/* sharkl5/sharkl5pro  */
	struct isp_dev_grgb_info grgb_info;
	struct isp_dev_bchs_info bchs_info;
	struct isp_dev_rgb_ltm_info ltm_rgb_info;
	struct isp_dev_yuv_ltm_info ltm_yuv_info;
	struct isp_dev_nlm_imblance imblance_info_v0;
	struct isp_dev_nlm_imblance_v1 imblance_info;
	/* sharkl5/sharkl5pro diff blocks*/
	struct isp_dev_posterize_info_v2 pstrz_info_v2;
	struct isp_dev_uvd_info_v2 uvd_info_v2;
	struct isp_dev_ynr_info_v2 ynr_info_v2;

	/* sharkl3 only */
	struct isp_dev_brightness_info brightness_info;
	struct isp_dev_contrast_info contrast_info;
	struct isp_dev_csa_info csa_info;
	struct isp_dev_hue_info_l3 hue_info;
	/* sharkl3 diff blocks*/
	struct isp_dev_posterize_info pstrz_info;
	struct isp_dev_uvd_info uvd_info;
	struct isp_dev_ynr_info ynr_info;

	/* common */
	struct isp_dev_3dnr_info nr3d_info;
	struct isp_dev_cce_info cce_info;
	struct isp_dev_pre_cdn_info pre_cdn_info;
	struct isp_dev_cdn_info cdn_info;
	struct isp_dev_post_cdn_info post_cdn_info;
	struct isp_dev_cfa_info cfa_info;
	struct isp_dev_cmc10_info cmc10_info;
	struct isp_dev_edge_info_v2 edge_info;
	struct isp_dev_gamma_info gamma_info;
	struct isp_dev_hsv_info_v2 hsv_info;
	struct isp_dev_iircnr_info iircnr_info;
	struct isp_dev_nlm_info_v2 nlm_info;
	struct isp_dev_ygamma_info ygamma_info;
	struct isp_dev_yrandom_info yrandom_info;
	struct isp_dev_noise_filter_info nf_info;
	uint32_t vst_buf[ISP_VST_IVST_NUM2];
	uint32_t ivst_buf[ISP_VST_IVST_NUM2];
};

int dcam_init_lsc_slice(void *param, uint32_t online);
int dcam_init_lsc(void *param, uint32_t online);
int dcam_update_lsc(void *param);
int dcam_k_cfg_blc(struct isp_io_param *param,	struct dcam_dev_param *p);
int dcam_k_cfg_raw_gtm(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_raw_gtm_block(uint32_t gtm_param_idx, struct dcam_dev_param *p);
int dcam_k_raw_gtm_slice(uint32_t idx, struct dcam_dev_gtm_slice_info *gtm_slice);
int dcam_k_cfg_rgb_gain(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_rgb_dither(struct isp_io_param *param,
	struct dcam_dev_param *p);
int dcam_k_cfg_pdaf(struct isp_io_param *param,	struct dcam_dev_param *p);
int dcam_k_cfg_lsc(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_bayerhist(struct isp_io_param *param,
	struct dcam_dev_param *p);
int dcam_k_cfg_aem(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_lscm(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_afl(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_awbc(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_bpc(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_grgb(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_3dnr_me(struct isp_io_param *param, struct dcam_dev_param *p);
int dcam_k_cfg_afm(struct isp_io_param *param, struct dcam_dev_param *p);
void dcam_k_3dnr_set_roi(struct isp_img_rect rect,
			uint32_t project_mode, uint32_t idx);

/* for dcam driver internal */
int dcam_k_blc_block(struct dcam_dev_param *param);
int dcam_k_rgb_gain_block(struct dcam_dev_param *param);
int dcam_k_rgb_dither_random_block(struct dcam_dev_param *param);
int dcam_k_lsc_block(struct dcam_dev_param *param);
int dcam_k_bayerhist_block(struct dcam_dev_param *param);
int dcam_k_bayerhist_roi(struct dcam_dev_param *param);

int dcam_k_aem_bypass(struct dcam_dev_param *param);
int dcam_k_aem_mode(struct dcam_dev_param *param);
int dcam_k_aem_win(struct dcam_dev_param *param);
int dcam_k_aem_skip_num(struct dcam_dev_param *param);
int dcam_k_aem_rgb_thr(struct dcam_dev_param *param);

int dcam_k_afl_block(struct dcam_dev_param *param);
int dcam_k_afl_bypass(struct dcam_dev_param *param);
int dcam_k_awbc_block(struct dcam_dev_param *param);
int dcam_k_awbc_gain(struct dcam_dev_param *param);
int dcam_k_awbc_block(struct dcam_dev_param *param);

int dcam_k_bpc_block(struct dcam_dev_param *param);
int dcam_k_bpc_ppe_param(struct dcam_dev_param *param);

int dcam_k_3dnr_me(struct dcam_dev_param *param);

int dcam_k_afm_block(struct dcam_dev_param *param);
int dcam_k_afm_bypass(struct dcam_dev_param *param);
int dcam_k_afm_win(struct dcam_dev_param *param);
int dcam_k_afm_win_num(struct dcam_dev_param *param);
int dcam_k_afm_mode(struct dcam_dev_param *param);
int dcam_k_afm_skipnum(struct dcam_dev_param *param);
int dcam_k_afm_crop_eb(struct dcam_dev_param *param);
int dcam_k_afm_crop_size(struct dcam_dev_param *param);
int dcam_k_afm_done_tilenum(struct dcam_dev_param *param);

int dcam_k_lscm_bypass(struct dcam_dev_param *param);
int dcam_k_lscm_monitor(struct dcam_dev_param *param);

int dcam_k_pdaf(struct dcam_dev_param *param);
/* for dcam driver internal end */

int isp_k_cfg_nlm(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_ynr(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_3dnr(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_bchs(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_cce(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_cdn(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_cfa(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_brightness(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_contrast(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_csa(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_hue(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_cmc10(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_edge(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_gamma(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_grgb(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_hist(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_hist2(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_hsv(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_iircnr(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_rgb_ltm(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_yuv_ltm(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_post_cdn(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_pre_cdn(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_pstrz(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_uvd(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_ygamma(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfg_yrandom(struct isp_io_param *param,
	struct isp_k_block *isp_k_param,  uint32_t idx);
int isp_k_cfg_yuv_noisefilter(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);
void cam_block_noisefilter_seeds(uint32_t image_width,
	uint32_t seed0, uint32_t *seed1, uint32_t *seed2, uint32_t *seed3);

int isp_k_update_nlm(uint32_t idx,
	struct isp_k_block *isp_k_param,
	uint32_t new_width, uint32_t old_width,
	uint32_t new_height, uint32_t old_height);
int isp_k_update_ynr(uint32_t idx,
	struct isp_k_block *isp_k_param,
	uint32_t new_width, uint32_t old_width,
	uint32_t new_height, uint32_t old_height);
int isp_k_update_3dnr(uint32_t idx,
	struct isp_k_block *isp_k_param,
	uint32_t new_width, uint32_t old_width,
	uint32_t new_height, uint32_t old_height);
int isp_k_update_imbalance(uint32_t idx,
	struct isp_k_block *isp_k_param,
	uint32_t new_width, uint32_t old_width,
	uint32_t new_height, uint32_t old_height);

int init_dcam_pm(struct dcam_dev_param *blk_pm_ctx);
int init_isp_pm(struct isp_k_block *isp_k_param);

/* for param debug */
int dcam_k_dump_pm(void *pdst, void *psrc);
int isp_k_dump_pm(void *pdst, void *psrc);

/* for bypass dcam,isp sub-block */
enum block_bypass {
	_E_4IN1 = 0,
	_E_PDAF,
	_E_LSC,
	_E_AEM,
	_E_HIST,
	_E_AFL,
	_E_AFM,
	_E_BPC,
	_E_GRGB,
	_E_BLC,
	_E_RGB,
	_E_RAND, /* yuv random */
	_E_PPI,
	_E_AWBC,
	_E_NR3,
	_E_GTM,
};
extern uint32_t g_dcam_bypass[];

enum isp_bypass {
	_EISP_GC = 0, /* E:enum, ISP: isp */
	_EISP_NLM,
	_EISP_VST,
	_EISP_IVST,
	_EISP_CFA,
	_EISP_CMC,
	_EISP_GAMC, /* gamma correction */
	_EISP_HSV,
	_EISP_HIST,
	_EISP_HIST2,
	_EISP_PSTRZ,
	_EISP_PRECDN,
	_EISP_YNR,
	_EISP_EE,
	_EISP_GAMY, /* Y gamma */
	_EISP_CDN,
	_EISP_POSTCDN,
	_EISP_UVD,
	_EISP_IIRCNR,
	_EISP_YRAND,
	_EISP_BCHS,
	_EISP_CONTRAST,
	_EISP_BRIGHT,
	_EISP_SATURATION,
	_EISP_HUE,
	_EISP_YUVNF,

	_EISP_TOTAL, /* total before this */
	_EISP_CCE = 29,
	_EISP_LTM = 30,
	_EISP_NR3 = 31,
	/* Attention up to 31 */
};
extern uint32_t g_isp_bypass[];

struct bypass_tag {
	char *p;/* abbreviation */
	uint32_t addr;
	uint32_t bpos;/* bit position */
	uint32_t all;/* 1: all bypass except preview path */
};

#endif
