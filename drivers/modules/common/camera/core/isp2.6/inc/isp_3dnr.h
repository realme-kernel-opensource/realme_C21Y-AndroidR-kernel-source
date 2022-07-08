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

#ifndef _ISP_3DNR_H_
#define _ISP_3DNR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cam_queue.h"
#include "dcam_interface.h"
#include "isp_interface.h"

#define FBC_NR3_Y_PAD_WIDTH            256
#define FBC_NR3_Y_PAD_HEIGHT           4
#define FBC_NR3_Y_WIDTH                128
#define FBC_NR3_Y_HEIGHT               2
#define FBC_NR3_BASE_ALIGN             256
#define FBD_NR3_Y_PAD_WIDTH            256
#define FBD_NR3_Y_PAD_HEIGHT           4
#define FBD_NR3_Y_WIDTH                128
#define FBD_NR3_Y_HEIGHT               2
#define FBD_NR3_BASE_ALIGN             256
#define FBD_BAYER_HEIGHT               4

#define NR3_BLEND_CNT                  5

enum nr3_func_type {
	NR3_FUNC_OFF,
	NR3_FUNC_PRE,
	NR3_FUNC_VID,
	NR3_FUNC_CAP,
	NR3_FUNC_MAX
};

struct isp_3dnr_mem_ctrl {
	uint32_t bypass;
	uint32_t nr3_done_mode;
	uint32_t nr3_ft_path_sel;
	uint32_t back_toddr_en;
	uint32_t chk_sum_clr_en;
	uint32_t data_toyuv_en;
	uint32_t roi_mode;
	uint32_t retain_num;
	uint32_t ref_pic_flag;
	uint32_t ft_max_len_sel;
	uint32_t first_line_mode;
	uint32_t last_line_mode;
	uint32_t start_row;
	uint32_t start_col;
	uint32_t global_img_width;
	uint32_t global_img_height;
	uint32_t img_width;
	uint32_t img_height;
	uint32_t ft_y_width;
	uint32_t ft_y_height;
	uint32_t ft_uv_width;
	uint32_t ft_uv_height;
	int mv_y;
	int mv_x;
	unsigned long ft_luma_addr;
	unsigned long ft_chroma_addr;
	uint32_t ft_pitch;
	uint32_t blend_y_en_start_row;
	uint32_t blend_y_en_start_col;
	uint32_t blend_y_en_end_row;
	uint32_t blend_y_en_end_col;
	uint32_t blend_uv_en_start_row;
	uint32_t blend_uv_en_start_col;
	uint32_t blend_uv_en_end_row;
	uint32_t blend_uv_en_end_col;
	uint32_t ft_hblank_num;
	uint32_t pipe_hblank_num;
	uint32_t pipe_flush_line_num;
	uint32_t pipe_nfull_num;
	uint32_t ft_fifo_nfull_num;
	struct img_addr frame_addr;
};

struct isp_3dnr_store {
	uint32_t chk_sum_clr_en;
	uint32_t shadow_clr_sel;
	uint32_t st_max_len_sel;
	uint32_t st_bypass;
	uint32_t img_width;
	uint32_t img_height;
	unsigned long st_luma_addr;
	unsigned long st_chroma_addr;
	uint32_t st_pitch;
	uint32_t shadow_clr;
};

struct isp_3dnr_fbd_fetch {
	uint32_t bypass;
	uint32_t fbdc_cr_ch0123_val0;
	uint32_t fbdc_cr_ch0123_val1;
	uint32_t fbdc_cr_y_val0;
	uint32_t fbdc_cr_y_val1;
	uint32_t fbdc_cr_uv_val0;
	uint32_t fbdc_cr_uv_val1;
	uint32_t y_tile_addr_init_x256;
	uint32_t y_tiles_num_pitch;
	uint32_t y_header_addr_init;
	uint32_t c_tile_addr_init_x256;
	uint32_t c_tiles_num_pitch;
	uint32_t c_header_addr_init;
	uint32_t y_pixel_size_in_hor;
	uint32_t y_pixel_size_in_ver;
	uint32_t c_pixel_size_in_hor;
	uint32_t c_pixel_size_in_ver;
	uint32_t y_pixel_start_in_hor;
	uint32_t y_pixel_start_in_ver;
	uint32_t c_pixel_start_in_hor;
	uint32_t c_pixel_start_in_ver;
	uint32_t y_tiles_num_in_hor;
	uint32_t y_tiles_num_in_ver;
	uint32_t c_tiles_num_in_hor;
	uint32_t c_tiles_num_in_ver;
	uint32_t c_tiles_start_odd;
	uint32_t y_tiles_start_odd;
	uint32_t y_rd_one_more_en;
	uint32_t rd_time_out_th;
};

struct isp_3dnr_fbc_store {
	uint32_t bypass;
	uint32_t slice_mode_en;
	uint32_t size_in_ver;
	uint32_t size_in_hor;
	unsigned long y_tile_addr_init_x256;
	unsigned long c_tile_addr_init_x256;
	uint32_t tile_number_pitch;
	unsigned long y_header_addr_init;
	unsigned long c_header_addr_init;
	uint32_t fbc_constant_yuv;
	uint32_t later_bits;
	uint32_t tile_number;
};

struct isp_3dnr_crop {
	uint32_t crop_bypass;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t dst_width;
	uint32_t dst_height;
	int start_x;
	int start_y;
};

struct  fast_mv {
	int mv_x;
	int mv_y;
};

struct mv_conversion {
	uint32_t nr3_channel_sel;
	uint32_t project_mode;
	uint32_t sub_me_bypass;
	int roi_start_x;
	int roi_start_y;
	uint32_t roi_width;
	uint32_t roi_height;
	uint32_t input_width;
	uint32_t input_height;
	uint32_t output_width;
	uint32_t output_height;
};

struct frame_size {
	uint32_t width;
	uint32_t height;
};

/* capture */
struct nr3_slice {
	uint32_t start_row;
	uint32_t end_row;
	uint32_t start_col;
	uint32_t end_col;
	uint32_t overlap_left;
	uint32_t overlap_right;
	uint32_t overlap_up;
	uint32_t overlap_down;
	uint32_t slice_num;
	uint32_t cur_frame_width;
	uint32_t cur_frame_height;
	uint32_t src_lum_addr;
	uint32_t src_chr_addr;
	uint32_t img_id;
	int mv_x;
	int mv_y;
};

struct nr3_slice_for_blending {
	uint32_t start_col;
	uint32_t start_row;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t ft_y_width;
	uint32_t ft_y_height;
	uint32_t ft_uv_width;
	uint32_t ft_uv_height;
	uint32_t src_lum_addr;
	uint32_t src_chr_addr;
	uint32_t first_line_mode;
	uint32_t last_line_mode;
	uint32_t dst_width;
	uint32_t dst_height;
	uint32_t dst_lum_addr;
	uint32_t dst_chr_addr;
	uint32_t offset_start_x;
	uint32_t offset_start_y;
	uint32_t crop_bypass;
};
/* capture */

struct isp_3dnr_ctx_desc {
	uint32_t bypass;
	uint32_t type;
	uint32_t blending_cnt;
	uint32_t mode;
	uint32_t width;
	uint32_t height;
	enum sprd_cam_sec_mode nr3_sec_mode;
	struct fast_mv mv;
	struct nr3_me_data *mvinfo;
	struct isp_3dnr_mem_ctrl mem_ctrl;
	struct isp_3dnr_store nr3_store;
	struct isp_3dnr_fbd_fetch nr3_fbd_fetch;
	struct isp_3dnr_fbc_store nr3_fbc_store;
	struct isp_3dnr_crop crop;
	struct camera_buf *buf_info[ISP_NR3_BUF_NUM];
};

int isp_3dnr_conversion_mv(struct isp_3dnr_ctx_desc *ctx);
int isp_3dnr_memctrl_slice_info_update(struct nr3_slice *in, struct nr3_slice_for_blending *out);

/*
 * Generate configuration of 3dnr,
 * incluing:
 * mem ctrl, crop, store
 */
int isp_3dnr_config_gen(struct isp_3dnr_ctx_desc *ctx);

void isp_3dnr_config_param(struct isp_3dnr_ctx_desc *ctx,
	struct isp_k_block *isp_k_param, uint32_t idx,
	enum nr3_func_type type_id);

void isp_3dnr_bypass_config(uint32_t idx);

#ifdef __cplusplus
}
#endif

#endif/* _ISP_3DNR_H_ */
