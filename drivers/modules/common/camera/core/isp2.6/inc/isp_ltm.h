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

#ifndef _ISP_LTM_H_
#define _ISP_LTM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cam_queue.h"
#include "dcam_interface.h"
#include "isp_interface.h"

#define LTM_HIST_TABLE_NUM              128
#define LTM_MIN_TILE_WIDTH              128
#define LTM_MIN_TILE_HEIGHT             20
#define LTM_MAX_TILE_RANGE              65536
#define LTM_MAX_ROI_X                   240
#define LTM_MAX_ROI_Y                   180
#define LTM_ID_MAX                      3

typedef struct isp_ltm_hist_param {
	/* match ltm stat info */
	uint32_t bypass;
	uint32_t region_est_en;
	uint32_t text_point_thres;
	uint32_t text_proportion;
	uint32_t channel_sel;
	uint32_t buf_sel;
	/* input */
	uint32_t strength;
	uint32_t tile_num_auto;

	/* output / input */
	uint32_t tile_num_x;
	uint32_t tile_num_y;

	/* output */
	uint32_t tile_width;
	uint32_t tile_height;
	uint32_t tile_size;

	uint32_t frame_width;
	uint32_t frame_height;
	uint32_t clipLimit;
	uint32_t clipLimit_min;
	uint32_t binning_en;

	uint32_t cropUp;
	uint32_t cropDown;
	uint32_t cropLeft;
	uint32_t cropRight;
	uint32_t cropRows;
	uint32_t cropCols;
} ltm_param_t;

typedef struct isp_ltm_rtl_param {
	int tile_index_xs;
	int tile_index_ys;
	int tile_index_xe;
	int tile_index_ye;
	uint32_t tile_x_num_rtl;
	uint32_t tile_y_num_rtl;
	uint32_t tile_width_rtl;
	uint32_t tile_height_rtl;
	uint32_t tile_size_pro_rtl;
	uint32_t tile_start_x_rtl;
	uint32_t tile_start_y_rtl;
	uint32_t tile_left_flag_rtl;
	uint32_t tile_right_flag_rtl;
} ltm_map_rtl_t;

/*
 * DON'T CHANGE
 * LTM register, copy from SPEC.
 *
 */
struct isp_ltm_hists {
	/* ISP_LTM_PARAMETERS 0x0010 */
	uint32_t bypass;
	uint32_t binning_en;
	uint32_t region_est_en;
	uint32_t buf_full_mode;
	uint32_t buf_sel;
	uint32_t channel_sel;
	/* ISP_LTM_ROI_START 0x0014 */
	uint32_t roi_start_x;
	uint32_t roi_start_y;

	/* ISP_LTM_TILE_RANGE 0x0018 */
	uint32_t tile_width;
	uint32_t tile_num_x_minus;
	uint32_t tile_height;
	uint32_t tile_num_y_minus;

	/* ISP_LTM_CLIP_LIMIT 0x0020 */
	uint32_t clip_limit;
	uint32_t clip_limit_min;

	/* ISP_LTM_THRES 0x0024 */
	uint32_t texture_proportion;
	uint32_t text_point_thres;

	/* ISP_LTM_ADDR 0x0028 */
	unsigned long addr;

	/* ISP_LTM_PITCH 0x002C */
	uint32_t pitch;
	uint32_t wr_num;
	uint32_t ltm_hist_table[LTM_HIST_TABLE_NUM];
};

struct isp_ltm_map {
	/* ISP_LTM_MAP_PARAM0 0x0010 */
	uint32_t bypass;
	uint32_t burst8_en;
	uint32_t hist_error_en;
	uint32_t fetch_wait_en;
	uint32_t fetch_wait_line;

	/* ISP_LTM_MAP_PARAM1 0x0014 */
	uint32_t tile_width;
	uint32_t tile_height;
	uint32_t tile_x_num;
	uint32_t tile_y_num;

	/* ISP_LTM_MAP_PARAM2 0x0018 */
	uint32_t tile_size_pro;/* ltmap_tile_width x ltmap_tile_height */

	/* ISP_LTM_MAP_PARAM3 0x001c */
	uint32_t tile_start_x;
	uint32_t tile_left_flag;
	uint32_t tile_start_y;
	uint32_t tile_right_flag;

	/* ISP_LTM_MAP_PARAM4 0x0020 */
	unsigned long mem_init_addr;

	/* ISP_LTM_MAP_PARAM5 0x0024 */
	uint32_t hist_pitch;
};

struct isp_ltm_ctx_desc {
	uint32_t bypass;
	uint32_t type;
	uint32_t isp_pipe_ctx_id;
	uint32_t ltm_index;

	/*
	 * preview and capture frame has UNIFY and UNIQ frame ID
	 * match frame of preview and capture
	 */
	uint32_t fid;
	/*
	 * Origion frame size
	 */
	uint32_t frame_width;
	uint32_t frame_height;
	/*
	 * Histo calc frame size, binning
	 */
	uint32_t frame_width_stat;
	uint32_t frame_height_stat;

	struct isp_ltm_hists hists[LTM_MAX];
	struct isp_ltm_map   map[LTM_MAX];

	struct camera_buf *pbuf[LTM_MAX][ISP_LTM_BUF_NUM];
};


/*
 * Share data between context pre / cap
 */
struct isp_ltm_share_ctx_ops;
struct isp_ltm_share_ctx_param {
	/*
	 * status:
	 *	1: running
	 *	0: stop
	 */
	uint32_t pre_ctx_status;
	uint32_t cap_ctx_status;

	uint32_t pre_cid;
	uint32_t cap_cid;

	atomic_t pre_fid;
	atomic_t cap_fid;

	uint32_t pre_update;
	uint32_t cap_update;

	uint32_t pre_hist_bypass;

	uint32_t pre_frame_h;
	uint32_t pre_frame_w;
	uint32_t cap_frame_h;
	uint32_t cap_frame_w;

	uint32_t tile_num_x_minus;
	uint32_t tile_num_y_minus;
	uint32_t tile_width;
	uint32_t tile_height;

	/* uint32_t wait_completion; */
	atomic_t wait_completion[LTM_MAX];
	struct completion share_comp[LTM_MAX];

	struct mutex share_mutex;
};

struct isp_ltm_share_ctx_desc {
	struct isp_ltm_share_ctx_param *param[LTM_ID_MAX];
	struct isp_ltm_share_ctx_ops *ops;
};

struct isp_ltm_share_ctx_ops {
	int (*init)(uint32_t idx);
	int (*deinit)(uint32_t idx);
	int (*set_status)(int status, int context_idx, int type, uint32_t idx);
	int (*get_status)(int type, uint32_t idx);
	int (*set_update)(int update, int type, uint32_t idx);
	int (*get_update)(int type, uint32_t idx);
	int (*set_frmidx)(int frame_idx, uint32_t idx);
	int (*get_frmidx)(uint32_t idx);
	int (*set_completion)(int frame_idx, enum isp_ltm_region ltm_id, uint32_t idx);
	int (*get_completion)(enum isp_ltm_region ltm_id, uint32_t idx);
	int (*complete_completion)(enum isp_ltm_region ltm_id, uint32_t idx);
	int (*set_config)(struct isp_ltm_ctx_desc *ctx, struct isp_ltm_hists *hists, uint32_t idx);
	int (*get_config)(struct isp_ltm_ctx_desc *ctx, struct isp_ltm_hists *hists, uint32_t idx);
	int (*clear_status)(uint32_t idx);
};

struct isp_ltm_tilenum_minus1 {
	uint32_t tile_num_x;
	uint32_t tile_num_y;
};

struct isp_ltm_text {
	uint32_t text_point_alpha;
	uint32_t text_point_thres;
	uint32_t textture_proporion;
};

struct isp_ltm_stat_info {
	uint32_t bypass;/* bypass */
	struct isp_ltm_tilenum_minus1 tile_num;
	struct isp_ltm_text ltm_text;
	uint32_t strength;
	uint32_t tile_num_auto;
	uint32_t text_point_thres;/* text_point_thres */
	uint32_t region_est_en;/* region_est_en */
	uint32_t channel_sel;
	uint32_t ltm_hist_table[LTM_HIST_TABLE_NUM];
};

struct isp_ltm_map_info {
	uint32_t bypass;/* ltm map bypass */
	uint32_t ltm_map_video_mode;
};

struct isp_ltm_info {
	struct isp_ltm_stat_info ltm_stat;
	struct isp_ltm_map_info ltm_map;
};

/*
 * EXPORT function interface
 */
struct isp_ltm_share_ctx_desc *isp_ltm_share_ctx_desc_get(void);
int isp_ltm_share_ctx_desc_put(struct isp_ltm_share_ctx_desc *param);

int isp_ltm_frame_config_gen(struct isp_ltm_ctx_desc *ctx,
	enum isp_ltm_region ltm_id, struct isp_ltm_info *ltm_info);
int isp_ltm_map_slice_config_gen(struct isp_ltm_ctx_desc *ctx,
			enum isp_ltm_region ltm_id,
			struct isp_ltm_rtl_param *prtl,
			uint32_t *slice_info);

int isp_ltm_config_param(struct isp_ltm_ctx_desc *ctx,
		enum isp_ltm_region ltm_id);

#ifdef __cplusplus
}
#endif

#endif /* _ISP_LTM_H_ */
