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

#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sprd_mm.h>

#include "cam_trusty.h"
#include "dcam_reg.h"
#include "defines.h"
#include "dcam_int.h"
#include "dcam_path.h"
#include "isp_reg.h"
#include "isp_core.h"
#include "isp_slice.h"
#include "isp_cfg.h"
#include "isp_path.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_HW_IF_L5: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static unsigned long coff_buf_addr[2][3][4] = {
	{
		{
			ISP_SCALER_PRE_LUMA_HCOEFF_BUF0,
			ISP_SCALER_PRE_CHROMA_HCOEFF_BUF0,
			ISP_SCALER_PRE_LUMA_VCOEFF_BUF0,
			ISP_SCALER_PRE_CHROMA_VCOEFF_BUF0,
		},
		{
			ISP_SCALER_VID_LUMA_HCOEFF_BUF0,
			ISP_SCALER_VID_CHROMA_HCOEFF_BUF0,
			ISP_SCALER_VID_LUMA_VCOEFF_BUF0,
			ISP_SCALER_VID_CHROMA_VCOEFF_BUF0,
		}
	},
	{
		{
			ISP_SCALER_PRE_LUMA_HCOEFF_BUF1,
			ISP_SCALER_PRE_CHROMA_HCOEFF_BUF1,
			ISP_SCALER_PRE_LUMA_VCOEFF_BUF1,
			ISP_SCALER_PRE_CHROMA_VCOEFF_BUF1,
		},
		{
			ISP_SCALER_VID_LUMA_HCOEFF_BUF1,
			ISP_SCALER_VID_CHROMA_HCOEFF_BUF1,
			ISP_SCALER_VID_LUMA_VCOEFF_BUF1,
			ISP_SCALER_VID_CHROMA_VCOEFF_BUF1,
		}
	},
};

#define CAM_HW_ADPT_LAYER
#include "dcam_hw.c"
#include "isp_hw.c"
#undef CAM_HW_ADPT_LAYER

static int camhwif_dcam_ioctl(void *handle,
	enum dcam_hw_cfg_cmd cmd, void *arg)
{
	int ret = 0;
	hw_ioctl_fun hw_ctrl = NULL;

	hw_ctrl = dcamhw_ioctl_fun_get(cmd);
	if (NULL != hw_ctrl)
		ret = hw_ctrl(handle, arg);
	else
		pr_debug("not support cmd %d", cmd);

	return ret;
}

static int camhwif_isp_ioctl(void *handle, enum isp_hw_cfg_cmd cmd, void *arg)
{
	int ret = 0;
	hw_ioctl_fun hw_ctrl = NULL;

	hw_ctrl = isphw_ioctl_fun_get(cmd);
	if (NULL != hw_ctrl)
		ret = hw_ctrl(handle, arg);
	else
		pr_debug("not support cmd %d", cmd);

	return ret;
}

const unsigned long slowmotion_store_addr[3][4] = {
	{
		DCAM_BIN_BASE_WADDR0,
		DCAM_BIN_BASE_WADDR1,
		DCAM_BIN_BASE_WADDR2,
		DCAM_BIN_BASE_WADDR3
	},
	{
		DCAM_AEM_BASE_WADDR,
		DCAM_AEM_BASE_WADDR1,
		DCAM_AEM_BASE_WADDR2,
		DCAM_AEM_BASE_WADDR3
	},
	{
		DCAM_HIST_BASE_WADDR,
		DCAM_HIST_BASE_WADDR1,
		DCAM_HIST_BASE_WADDR2,
		DCAM_HIST_BASE_WADDR3
	}
};

static uint32_t path_ctrl_id[DCAM_PATH_MAX] = {
	[DCAM_PATH_FULL] = DCAM_CTRL_FULL,
	[DCAM_PATH_BIN] = DCAM_CTRL_BIN,
	[DCAM_PATH_PDAF] = DCAM_CTRL_PDAF,
	[DCAM_PATH_VCH2] = DCAM_CTRL_VCH2,
	[DCAM_PATH_VCH3] = DCAM_CTRL_VCH3,
	[DCAM_PATH_AEM] = DCAM_CTRL_COEF,
	[DCAM_PATH_AFM] = DCAM_CTRL_COEF,
	[DCAM_PATH_AFL] = DCAM_CTRL_COEF,
	[DCAM_PATH_HIST] = DCAM_CTRL_COEF,
	[DCAM_PATH_3DNR] = DCAM_CTRL_COEF,
	[DCAM_PATH_BPC] = DCAM_CTRL_COEF,
	[DCAM_PATH_LSCM] = DCAM_CTRL_COEF,
};

static unsigned long dcam_store_addr[DCAM_PATH_MAX] = {
	[DCAM_PATH_FULL] = DCAM_FULL_BASE_WADDR,
	[DCAM_PATH_BIN] = DCAM_BIN_BASE_WADDR0,
	[DCAM_PATH_PDAF] = DCAM_PDAF_BASE_WADDR,
	[DCAM_PATH_VCH2] = DCAM_VCH2_BASE_WADDR,
	[DCAM_PATH_VCH3] = DCAM_VCH3_BASE_WADDR,
	[DCAM_PATH_AEM] = DCAM_AEM_BASE_WADDR,
	[DCAM_PATH_AFM] = ISP_AFM_BASE_WADDR,
	[DCAM_PATH_AFL] = ISP_AFL_GLB_WADDR,
	[DCAM_PATH_HIST] = DCAM_HIST_BASE_WADDR,
	[DCAM_PATH_3DNR] = ISP_NR3_WADDR,
	[DCAM_PATH_BPC] = ISP_BPC_OUT_ADDR,
};

static unsigned long dcam2_store_addr[DCAM_PATH_MAX] = {
	[DCAM_PATH_FULL] = DCAM2_PATH0_BASE_WADDR,
	[DCAM_PATH_BIN] = DCAM2_PATH1_BASE_WADDR,
};

static uint32_t isp_ctx_fmcu_support[ISP_CONTEXT_HW_NUM] = {
	[ISP_CONTEXT_HW_P0] = 1,
	[ISP_CONTEXT_HW_C0] = 1,
	[ISP_CONTEXT_HW_P1] = 1,
	[ISP_CONTEXT_HW_C1] = 1,
};

static struct cam_hw_soc_info dcam_soc_info;
static struct cam_hw_soc_info isp_soc_info;
static struct cam_hw_ip_info dcam[DCAM_ID_MAX] = {
	[DCAM_ID_0] = {
		.slm_path = BIT(DCAM_PATH_BIN) | BIT(DCAM_PATH_AEM)
			| BIT(DCAM_PATH_HIST),
		.lbuf_share_support = 1,
		.offline_slice_support = 0,
		.afl_gbuf_size = STATIS_AFL_GBUF_SIZE,
		.superzoom_support = 1,
		.dcam_full_fbc_mode = DCAM_FBC_DISABLE,
		.dcam_bin_fbc_mode = DCAM_FBC_DISABLE,
		.store_addr_tab = dcam_store_addr,
		.path_ctrl_id_tab = path_ctrl_id,
		.pdaf_type3_reg_addr = DCAM_PPE_RIGHT_WADDR,
		.rds_en = 1,
	},
	[DCAM_ID_1] = {
		.slm_path = BIT(DCAM_PATH_BIN) | BIT(DCAM_PATH_AEM)
			| BIT(DCAM_PATH_HIST),
		.lbuf_share_support = 1,
		.offline_slice_support = 1,
		.afl_gbuf_size = STATIS_AFL_GBUF_SIZE,
		.superzoom_support = 1,
		.dcam_full_fbc_mode = DCAM_FBC_DISABLE,
		.dcam_bin_fbc_mode = DCAM_FBC_DISABLE,
		.store_addr_tab = dcam_store_addr,
		.path_ctrl_id_tab = path_ctrl_id,
		.pdaf_type3_reg_addr = DCAM_PPE_RIGHT_WADDR,
		.rds_en = 1,
	},
	[DCAM_ID_2] = {
		.slm_path = BIT(DCAM_PATH_BIN) | BIT(DCAM_PATH_AEM)
			| BIT(DCAM_PATH_HIST),
		.lbuf_share_support = 0,
		.offline_slice_support = 0,
		.afl_gbuf_size = STATIS_AFL_GBUF_SIZE,
		.superzoom_support = 1,
		.dcam_full_fbc_mode = DCAM_FBC_DISABLE,
		.dcam_bin_fbc_mode = DCAM_FBC_DISABLE,
		.store_addr_tab = dcam2_store_addr,
		.path_ctrl_id_tab = path_ctrl_id,
		.pdaf_type3_reg_addr = DCAM_PPE_RIGHT_WADDR,
		.rds_en = 0,
	},
};
static struct cam_hw_ip_info isp = {
	.slm_cfg_support = 1,
	.ctx_fmcu_support = isp_ctx_fmcu_support,
};

struct cam_hw_info sharkl5_hw_info = {
	.prj_id = SHARKL5,
	.pdev = NULL,
	.soc_dcam = &dcam_soc_info,
	.soc_isp = &isp_soc_info,
	.ip_dcam[DCAM_ID_0] = &dcam[DCAM_ID_0],
	.ip_dcam[DCAM_ID_1] = &dcam[DCAM_ID_1],
	.ip_dcam[DCAM_ID_2] = &dcam[DCAM_ID_2],
	.ip_isp = &isp,
	.dcam_ioctl = camhwif_dcam_ioctl,
	.isp_ioctl = camhwif_isp_ioctl,

};
