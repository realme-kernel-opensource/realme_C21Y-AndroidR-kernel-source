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

#ifndef _CAM_HW_H_
#define _CAM_HW_H_

#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "cam_types.h"

#define ISP_SC_COEFF_BUF_SIZE   (24 << 10)

extern struct cam_hw_info sharkl3_hw_info;
extern struct cam_hw_info sharkl5_hw_info;
extern struct cam_hw_info sharkl5pro_hw_info;
extern struct cam_hw_info qogirl6_hw_info;

typedef int (*hw_ioctl_fun)(void *handle, void *arg);

/*
 * Supported dcam_if index. Number 0&1 for dcam_if and 2 for dcam_if_lite.
 */
enum dcam_id {
	DCAM_ID_0 = 0,
	DCAM_ID_1,
	DCAM_ID_2,
	DCAM_ID_MAX,
};

/* The project id must keep same with the DT cfg
 * new added project should always added in the end
 */
enum cam_prj_id {
	SHARKL3,
	SHARKL5,
	ROC1,
	SHARKL5pro,
	QOGIRN6pro,
	QOGIRL6,
	PROJECT_MAX
};

enum isp_default_type {
	ISP_HW_PARA,
	ISP_CFG_PARA,
	ISP_MAX_PARA
};

enum cam_block_type {
	DCAM_BLOCK_TYPE,
	ISP_BLOCK_TYPE,
	MAX_BLOCK_TYPE
};

enum dcam_full_src_sel_type {
	ORI_RAW_SRC_SEL,
	PROCESS_RAW_SRC_SEL,
	MAX_RAW_SRC_SEL
};

enum cam_reg_trace_type {
	NORMAL_REG_TRACE,
	ABNORMAL_REG_TRACE,
	MAX_REG_TRACE
};

enum cam_bypass_type {
	DCAM_BYPASS_TYPE,
	ISP_BYPASS_TYPE,
	MAX_BYPASS_TYPE
};

enum dcam_fbc_mode_type {
	DCAM_FBC_DISABLE = 0,
	DCAM_FBC_FULL_10_BIT = 0x1,
	DCAM_FBC_BIN_10_BIT = 0x3,
	DCAM_FBC_FULL_14_BIT = 0x5,
	DCAM_FBC_BIN_14_BIT = 0x7,
};

enum isp_yuv_block_ctrl_type {
	ISP_YUV_BLOCK_CFG,
	ISP_YUV_BLOCK_DISABLE,
	ISP_YUV_BLOCK_ENABLE,
	ISP_YUV_BLOCK_MAX
};

enum isp_sub_path_id {
	ISP_SPATH_CP = 0,
	ISP_SPATH_VID,
	ISP_SPATH_FD,
	ISP_SPATH_NUM,
};

enum isp_afbc_path {
	AFBC_PATH_PRE = 0,
	AFBC_PATH_VID,
	AFBC_PATH_NUM,
};

enum dcam_hw_cfg_cmd {
	DCAM_HW_CFG_ENABLE_CLK,
	DCAM_HW_CFG_DISABLE_CLK,
	DCAM_HW_CFG_INIT_AXI,
	DCAM_HW_CFG_SET_QOS,
	DCAM_HW_CFG_RESET,
	DCAM_HW_CFG_START,
	DCAM_HW_CFG_STOP,
	DCAM_HW_CFG_STOP_CAP_EB,
	DCAM_HW_CFG_FETCH_START,
	DCAM_HW_CFG_AUTO_COPY,
	DCAM_HW_CFG_FORCE_COPY,
	DCAM_HW_CFG_PATH_START,
	DCAM_HW_CFG_PATH_STOP,
	DCAM_HW_CFG_PATH_CTRL,
	DCAM_HW_CFG_PATH_SRC_SEL,
	DCAM_HW_CFG_PATH_SIZE_UPDATE,
	DCAM_HW_CFG_CALC_RDS_PHASE_INFO,
	DCAM_HW_CFG_MIPI_CAP_SET,
	DCAM_HW_CFG_FETCH_SET,
	DCAM_HW_CFG_FETCH_BLOCK_SET,
	DCAM_HW_CFG_EBD_SET,
	DCAM_HW_CFG_BINNING_4IN1_SET,
	DCAM_HW_CFG_SRAM_CTRL_SET,
	DCAM_HW_CFG_LBUF_SHARE_SET,
	DCAM_HW_CFG_LBUF_SHARE_GET,
	DCAM_HW_CFG_SLICE_FETCH_SET,
	DCAM_HW_CFG_FBC_CTRL,
	DCAM_HW_CFG_FBC_ADDR_SET,
	DCAM_HW_CFG_GTM_STATUS_GET,
	DCAM_HW_CFG_GTM_LTM_EB,
	DCAM_HW_CFG_GTM_LTM_DIS,
	DCAM_HW_CFG_GTM_UPDATE,
	DCAM_HW_CFG_BLOCK_FUNC_GET,
	DCAM_HW_CFG_BLOCKS_SETALL,
	DCAM_HW_CFG_BLOCKS_SETSTATIS,
	DCAM_HW_CFG_MIPICAP,
	DCAM_HW_CFG_START_FETCH,
	DCAM_HW_CFG_BIN_MIPI,
	DCAM_HW_CFG_BIN_PATH,
	DCAM_HW_CFG_HIST_ROI_UPDATE,
	DCAM_HW_CFG_MAX
};

enum isp_hw_cfg_cmd {
	ISP_HW_CFG_ENABLE_CLK,
	ISP_HW_CFG_DISABLE_CLK,
	ISP_HW_CFG_RESET,
	ISP_HW_CFG_ENABLE_IRQ,
	ISP_HW_CFG_DISABLE_IRQ,
	ISP_HW_CFG_CLEAR_IRQ,
	ISP_HW_CFG_FETCH_SET,
	ISP_HW_CFG_FETCH_FBD_SET,
	ISP_HW_CFG_DEFAULT_PARA_SET,
	ISP_HW_CFG_BLOCK_FUNC_GET,
	ISP_HW_CFG_CFG_MAP_INFO_GET,
	ISP_HW_CFG_FMCU_VALID_GET,
	ISP_HW_CFG_BYPASS_DATA_GET,
	ISP_HW_CFG_BYPASS_COUNT_GET,
	ISP_HW_CFG_REG_TRACE,
	ISP_HW_CFG_ISP_CFG_SUBBLOCK,
	ISP_HW_CFG_SET_PATH_STORE,
	ISP_HW_CFG_SET_PATH_SCALER,
	ISP_HW_CFG_SET_PATH_THUMBSCALER,
	ISP_HW_CFG_SLICE_SCALER,
	ISP_HW_CFG_SLICE_STORE,
	ISP_HW_CFG_AFBC_PATH_SET,
	ISP_HW_CFG_FBD_SLICE_SET,
	ISP_HW_CFG_FBD_ADDR_SET,
	ISP_HW_CFG_AFBC_FMCU_ADDR_SET,
	ISP_HW_CFG_AFBC_PATH_SLICE_SET,
	ISP_HW_CFG_LTM_SLICE_SET,
	ISP_HW_CFG_NR3_FBC_SLICE_SET,
	ISP_HW_CFG_NR3_FBD_SLICE_SET,
	ISP_HW_CFG_SLW_FMCU_CMDS,
	ISP_HW_CFG_FMCU_CFG,
	ISP_HW_CFG_SLICE_FETCH,
	ISP_HW_CFG_SLICE_NR_INFO,
	ISP_HW_CFG_SLICE_FMCU_CMD,
	ISP_HW_CFG_SLICE_NOFILTER,
	ISP_HW_CFG_SLICE_3DNR_CROP,
	ISP_HW_CFG_SLICE_3DNR_STORE,
	ISP_HW_CFG_SLICE_3DNR_MEMCTRL,
	ISP_HW_CFG_SLICE_SPATH_STORE,
	ISP_HW_CFG_SLICE_SPATH_SCALER,
	ISP_HW_CFG_SLICE_SPATH_THUMBSCALER,
	ISP_HW_CFG_SET_SLICE_FETCH,
	ISP_HW_CFG_SET_SLICE_NR_INFO,
	ISP_HW_CFG_LTM_PARAM,
	ISP_HW_CFG_3DNR_PARAM,
	ISP_HW_CFG_GET_NLM_YNR,
	ISP_HW_CFG_STOP,
	ISP_HW_CFG_STORE_FRAME_ADDR,
	ISP_HW_CFG_FETCH_FRAME_ADDR,
	ISP_HW_CFG_MAP_INIT,
	ISP_HW_CFG_START_ISP,
	ISP_HW_CFG_UPDATE_HIST_ROI,
	ISP_HW_CFG_FETCH_START,
	ISP_HW_CFG_FMCU_CMD,
	ISP_HW_CFG_FMCU_START,
	ISP_HW_CFG_YUV_BLOCK_CTRL_TYPE,
	ISP_HW_CFG_MAX
};

enum dcam_path_ctrl {
	HW_DCAM_PATH_PAUSE = 0,
	HW_DCAM_PATH_RESUME,
	HW_DCAM_PATH_MAX
};

enum isp_store_format {
	ISP_STORE_UYVY = 0x00,
	ISP_STORE_YUV422_2FRAME,
	ISP_STORE_YVU422_2FRAME,
	ISP_STORE_YUV422_3FRAME,
	ISP_STORE_YUV420_2FRAME,
	ISP_STORE_YVU420_2FRAME,
	ISP_STORE_YUV420_3FRAME,
	ISP_STORE_FORMAT_MAX
};

enum isp_fetch_format {
	ISP_FETCH_YUV422_3FRAME = 0,
	ISP_FETCH_YUYV,
	ISP_FETCH_UYVY,
	ISP_FETCH_YVYU,
	ISP_FETCH_VYUY,
	ISP_FETCH_YUV422_2FRAME,
	ISP_FETCH_YVU422_2FRAME,
	ISP_FETCH_RAW10,
	ISP_FETCH_CSI2_RAW10,
	ISP_FETCH_FULL_RGB10,
	ISP_FETCH_YUV420_2FRAME,
	ISP_FETCH_YVU420_2FRAME,
	ISP_FETCH_FORMAT_MAX
};

struct isp_fbd_raw_info {
	uint32_t ctx_id;
	/* ISP_FBD_RAW_SEL */
	uint32_t pixel_start_in_hor:6;
	uint32_t pixel_start_in_ver:2;
	uint32_t chk_sum_auto_clr:1;
	uint32_t fetch_fbd_bypass:1;
	uint32_t fetch_fbd_4bit_bypass:1;
	/* ISP_FBD_RAW_SLICE_SIZE */
	uint32_t height;
	uint32_t width;
	/* ISP_FBD_RAW_PARAM0 */
	uint32_t tiles_num_in_ver:11;
	uint32_t tiles_num_in_hor:6;
	/* ISP_FBD_RAW_PARAM1 */
	uint32_t time_out_th:8;
	uint32_t tiles_start_odd:1;
	uint32_t tiles_num_pitch:8;
	/* ISP_FBD_RAW_PARAM2 */
	uint32_t header_addr_init;
	/* ISP_FBD_RAW_PARAM3 */
	uint32_t tile_addr_init_x256;
	/* ISP_FBD_RAW_PARAM4 */
	uint32_t fbd_cr_ch0123_val0;
	/* ISP_FBD_RAW_PARAM5 */
	uint32_t fbd_cr_ch0123_val1;
	/* ISP_FBD_RAW_PARAM6 */
	uint32_t fbd_cr_uv_val1:8;
	uint32_t fbd_cr_y_val1:8;
	uint32_t fbd_cr_uv_val0:8;
	uint32_t fbd_cr_y_val0:8;
	/* ISP_FBD_RAW_LOW_PARAM0 */
	uint32_t low_bit_addr_init;
	/* ISP_FBD_RAW_LOW_PARAM1 */
	uint32_t low_bit_pitch:16;
	/* ISP_FBD_RAW_HBLANK */
	uint32_t hblank_en:1;
	uint32_t hblank_num:16;
	/* ISP_FBD_RAW_LOW_4BIT_PARAM0 */
	uint32_t low_4bit_addr_init;
	/* ISP_FBD_RAW_LOW_4BIT_PARAM1 */
	uint32_t low_4bit_pitch:16;
	/*
	 * For ISP trim feature. In capture channel, DCAM FULL crop is not used
	 * in zoom. ISP fetch trim is used instead.
	 * @size is normally same as @width and @height above.
	 */
	struct img_size size;
	struct img_trim trim;
	struct compressed_addr hw_addr;
	uint32_t header_addr_offset;
	uint32_t tile_addr_offset_x256;
	uint32_t low_bit_addr_offset;
	uint32_t low_4bit_addr_offset;
};

struct isp_hw_fetch_info {
	uint32_t ctx_id;
	uint32_t dispatch_color;
	uint32_t fetch_path_sel;
	uint32_t pack_bits;
	uint32_t bayer_pattern;
	enum sprd_cam_sec_mode sec_mode;
	enum isp_fetch_format fetch_fmt;
	struct img_size src;
	struct img_trim in_trim;
	struct img_addr addr;
	struct img_addr trim_off;
	struct img_addr addr_hw;
	struct img_pitch pitch;
	uint32_t mipi_byte_rel_pos;
	uint32_t mipi_word_num;
};

struct store_border {
	uint32_t up_border;
	uint32_t down_border;
	uint32_t left_border;
	uint32_t right_border;
};

struct isp_afbc_store_info {
	uint32_t bypass;
	uint32_t endian;
	uint32_t mirror_en;
	uint32_t color_format;
	uint32_t tile_number_pitch;
	uint32_t yaddr;
	uint32_t yheader;
	uint32_t header_offset;
	struct img_size size;
	struct store_border border;
};

struct isp_hw_afbc_path {
	uint32_t ctx_id;
	enum isp_sub_path_id spath_id;
	struct isp_afbc_store_info afbc_store;
};

struct isp_store_info {
	uint32_t bypass;
	uint32_t endian;
	uint32_t speed_2x;
	uint32_t mirror_en;
	uint32_t max_len_sel;
	uint32_t shadow_clr;
	uint32_t store_res;
	uint32_t rd_ctrl;
	uint32_t shadow_clr_sel;
	uint32_t total_size;
	enum isp_store_format color_fmt;
	struct img_size size;
	struct img_addr addr;
	struct img_addr slice_offset;
	struct img_pitch pitch;
};

struct isp_hw_path_store {
	uint32_t ctx_id;
	enum isp_sub_path_id spath_id;
	struct isp_store_info store;
};

struct isp_scaler_info {
	uint32_t scaler_bypass;
	uint32_t odata_mode;
	uint32_t scaler_y_ver_tap;
	uint32_t scaler_uv_ver_tap;
	uint32_t scaler_ip_int;
	uint32_t scaler_ip_rmd;
	uint32_t scaler_cip_int;
	uint32_t scaler_cip_rmd;
	uint32_t scaler_factor_in;
	uint32_t scaler_factor_out;
	uint32_t scaler_ver_ip_int;
	uint32_t scaler_ver_ip_rmd;
	uint32_t scaler_ver_cip_int;
	uint32_t scaler_ver_cip_rmd;
	uint32_t scaler_ver_factor_in;
	uint32_t scaler_ver_factor_out;
	uint32_t scaler_out_width;
	uint32_t scaler_out_height;
	uint32_t coeff_buf[ISP_SC_COEFF_BUF_SIZE];
};

struct isp_regular_info {
	uint32_t regular_mode;
	uint32_t shrink_uv_dn_th;
	uint32_t shrink_uv_up_th;
	uint32_t shrink_y_dn_th;
	uint32_t shrink_y_up_th;
	uint32_t effect_v_th;
	uint32_t effect_u_th;
	uint32_t effect_y_th;
	uint32_t shrink_c_range;
	uint32_t shrink_c_offset;
	uint32_t shrink_y_range;
	uint32_t shrink_y_offset;
};

struct img_deci_info {
	uint32_t deci_y_eb;
	uint32_t deci_y;
	uint32_t deci_x_eb;
	uint32_t deci_x;
};

struct isp_hw_path_scaler {
	uint32_t ctx_id;
	uint32_t uv_sync_v;
	uint32_t frm_deci;
	enum isp_sub_path_id spath_id;
	struct img_deci_info deci;
	struct img_size src;
	struct img_trim in_trim;
	struct img_trim out_trim;
	struct img_size dst;
	struct isp_scaler_info scaler;
	struct isp_regular_info regular_info;
};

struct isp_hw_hist_roi {
	uint32_t ctx_id;
	struct isp_coord *hist_roi;
};

struct isp_hw_cfg_map {
	atomic_t map_cnt;
	struct isp_dev_cfg_info *s_cfg_settings;
};

struct isp_hw_set_slice_nr_info {
	uint32_t start_row_mod4;
	struct slice_nlm_info *slice_nlm;
	struct slice_ynr_info *slice_ynr;
	struct isp_fmcu_ctx_desc *fmcu;
};

struct isp_hw_set_slice_fetch {
	struct isp_fmcu_ctx_desc *fmcu;
	struct slice_fetch_info *fetch_info;
};

struct isp_hw_slice_3dnr_memctrl {
	struct isp_fmcu_ctx_desc *fmcu;
	struct slice_3dnr_memctrl_info *mem_ctrl;
};

struct isp_hw_slice_3dnr_store {
	struct isp_fmcu_ctx_desc *fmcu;
	struct slice_3dnr_store_info *store;
};

struct isp_hw_slice_3dnr_crop {
	struct isp_fmcu_ctx_desc *fmcu;
	struct slice_3dnr_crop_info *crop;
};

struct isp_hw_slice_nofilter {
	struct slice_noisefilter_info *noisefilter_info;
	struct isp_fmcu_ctx_desc *fmcu;
};

struct dcam_hw_fetch_block {
	uint32_t idx;
	uint32_t raw_fetch_count;
};

struct dcam_hw_cfg_mipicap {
	uint32_t idx;
	uint32_t reg_val;
};

struct cam_hw_gtm_update {
	uint32_t gtm_idx;
	uint32_t idx;
	spinlock_t glb_reg_lock;
	struct dcam_dev_param *blk_dcam_pm;
	struct cam_hw_info *hw;
};

struct dcam_hw_slice_fetch {
	uint32_t idx;
	uint32_t slice_count;
	uint32_t dcam_slice_mode;
	struct dcam_fetch_info *fetch;
	struct img_trim *cur_slice;
	struct img_trim slice_trim;
};

struct dcam_hw_sram_ctrl {
	uint32_t sram_ctrl_en;
	uint32_t idx;
};

struct dcam_hw_binning_4in1 {
	uint32_t idx;
	uint32_t binning_4in1_en;
};

struct dcam_hw_path_size {
	uint32_t idx;
	uint32_t auto_cpy_id;
	uint32_t size_x;
	uint32_t size_y;
	uint32_t path_id;
	uint32_t src_sel;
	uint32_t bin_ratio;
	uint32_t scaler_sel;
	uint32_t rds_coeff_size;
	uint32_t rds_init_phase_int1;
	uint32_t rds_init_phase_int0;
	uint32_t rds_init_phase_rdm1;
	uint32_t rds_init_phase_rdm0;
	void *rds_coeff_buf;
	struct img_size in_size;
	struct img_trim in_trim;
	struct img_size out_size;
};

struct dcam_hw_path_src_sel {
	uint32_t src_sel;
	uint32_t idx;
};

struct dcam_hw_auto_copy {
	uint32_t id;
	uint32_t idx;
	spinlock_t glb_reg_lock;
};

struct dcam_hw_path_stop {
	uint32_t idx;
	uint32_t path_id;
};

struct dcam_fetch_info {
	uint32_t pack_bits;
	uint32_t endian;
	uint32_t pattern;
	struct img_size size;
	struct img_trim trim;
	struct img_addr addr;
};

struct dcam_mipi_info {
	uint32_t sensor_if;
	uint32_t format;
	uint32_t mode;
	uint32_t data_bits;
	uint32_t pattern;
	uint32_t href;
	uint32_t frm_deci;
	uint32_t frm_skip;
	uint32_t x_factor;
	uint32_t y_factor;
	uint32_t is_4in1;
	uint32_t dcam_slice_mode;
	uint32_t is_cphy;
	struct img_trim cap_size;
};

struct dcam_hw_mipi_cap {
	uint32_t idx;
	struct dcam_mipi_info cap_info;
};

struct dcam_hw_path_start {
	uint32_t path_id;
	uint32_t idx;
	uint32_t slowmotion_count;
	uint32_t pdaf_path_eb;
	uint32_t pack_bits;
	uint32_t src_sel;
	uint32_t bayer_pattern;
	struct img_trim in_trim;
	struct dcam_mipi_info cap_info;
	struct img_endian endian;
};

struct dcam_hw_fetch_set {
	uint32_t idx;
	struct dcam_fetch_info *fetch_info;
};

struct dcam_hw_force_copy {
	uint32_t id;
	uint32_t idx;
	spinlock_t glb_reg_lock;
};

struct dcam_hw_start {
	uint32_t idx;
	uint32_t format;
};

struct reg_add_val_tag {
	unsigned int addr;
	unsigned int valid;
	unsigned int dvalue;
	unsigned int rw;
	unsigned int wc;
};

struct coeff_arg {
	uint32_t *h_coeff;
	uint32_t *v_coeff;
	uint32_t *v_chroma_coeff;
	uint32_t h_coeff_addr;
	uint32_t h_chroma_coeff_addr;
	uint32_t v_coeff_addr;
	uint32_t v_chroma_coeff_addr;
};

struct dcam_hw_path_ctrl {
	uint32_t path_id;
	uint32_t idx;
	enum dcam_path_ctrl type;
};

struct dcam_hw_ebd_set {
	struct sprd_ebd_control *p;
	uint32_t idx;
};

struct isp_hw_default_param {
	uint32_t type;
	uint32_t index;
};

struct isp_hw_block_func {
	struct isp_cfg_entry *isp_entry;
	uint32_t index;
};

struct dcam_hw_block_func_get {
	struct dcam_cfg_entry *dcam_entry;
	uint32_t index;
};

struct isp_hw_thumbscaler_info {
	uint32_t idx;
	uint32_t scaler_bypass;
	uint32_t odata_mode;
	uint32_t frame_deci;

	struct img_deci_info y_deci;
	struct img_deci_info uv_deci;

	struct img_size y_factor_in;
	struct img_size y_factor_out;
	struct img_size uv_factor_in;
	struct img_size uv_factor_out;

	struct img_size src0;
	struct img_trim y_trim;
	struct img_size y_src_after_deci;
	struct img_size y_dst_after_scaler;
	struct img_size y_init_phase;

	struct img_trim uv_trim;
	struct img_size uv_src_after_deci;
	struct img_size uv_dst_after_scaler;
	struct img_size uv_init_phase;
};

struct cam_hw_bypass_data {
	struct bypass_tag *tag;
	enum cam_bypass_type type;
	uint32_t i;
};

struct cam_hw_reg_trace {
	int type;
	uint32_t idx;
};

struct isp_hw_slice_store {
	uint32_t path_en;
	uint32_t ctx_id;
	enum isp_sub_path_id spath_id;
	struct slice_store_info *slc_store;
};

struct isp_hw_slice_scaler {
	uint32_t path_en;
	uint32_t ctx_id;
	enum isp_sub_path_id spath_id;
	struct slice_scaler_info *slc_scaler;
};

struct isp_hw_nr3_fbd_slice {
	struct isp_fmcu_ctx_desc *fmcu_handle;
	struct slice_3dnr_fbd_fetch_info *fbd_fetch;
};

struct isp_hw_nr3_fbc_slice {
	struct isp_fmcu_ctx_desc *fmcu_handle;
	struct slice_3dnr_fbc_store_info *fbc_store;
};

struct isp_hw_ltm_slice {
	struct isp_fmcu_ctx_desc *fmcu_handle;
	struct slice_ltm_map_info *map;
	uint32_t ltm_id;
};

struct isp_hw_afbc_path_slice {
	struct isp_fmcu_ctx_desc *fmcu_handle;
	struct slice_afbc_store_info *slc_afbc_store;
	uint32_t path_en;
	uint32_t ctx_idx;
	uint32_t spath_id;
};

struct isp_hw_slw_fmcu_cmds {
	uint32_t ctx_id;
	struct img_addr fetchaddr;
	struct isp_fmcu_ctx_desc *fmcu_handle;
	struct isp_afbc_store_info afbc_store[AFBC_PATH_NUM];
	struct isp_store_info store[ISP_SPATH_NUM];
	struct isp_path_desc *isp_path;
};

struct isp_hw_fmcu_cfg {
	uint32_t ctx_id;
	struct isp_fmcu_ctx_desc *fmcu;
};

struct isp_hw_slice_fetch {
	uint32_t ctx_id;
	struct slice_fetch_info *fetch_info;
};

struct isp_hw_slice_nr_info {
	uint32_t ctx_id;
	struct isp_slice_desc *cur_slc;
};

struct isp_hw_slices_fmcu_cmds {
	int hw_ctx_id;
	uint32_t wmode;
	struct isp_fmcu_ctx_desc *fmcu;
};

struct isp_hw_afbc_fmcu_addr {
	struct isp_fmcu_ctx_desc *fmcu;
	uint32_t yaddr;
	uint32_t yheader;
	int index;
};

struct isp_hw_afbc_addr {
	uint32_t idx;
	uint32_t spath_id;
	unsigned long *yuv_addr;
};

struct isp_hw_fbd_slice {
	struct isp_fmcu_ctx_desc *fmcu_handle;
	struct slice_fbd_raw_info *info;
};

struct cam_hw_gtm_ltm_dis {
	uint32_t dcam_idx;
	uint32_t isp_idx;
};

struct cam_hw_gtm_ltm_eb {
	uint32_t dcam_idx;
	uint32_t isp_idx;
};

struct dcam_hw_fbc_addr {
	uint32_t idx;
	unsigned long addr;
	struct compressed_addr *fbc_addr;
};

struct dcam_hw_fbc_ctrl {
	uint32_t idx;
	int fbc_mode;
};

struct cam_hw_lbuf_share {
	enum dcam_id idx;
	uint32_t width;
	uint32_t offline_flag;
};

struct dcam_hw_calc_rds_phase {
	struct dcam_rds_slice_ctrl *gphase;
	uint16_t slice_id;
	uint16_t slice_end0;
	uint16_t slice_end1;
};

struct isp_hw_slice_spath {
	uint32_t path_en;
	uint32_t ctx_idx;
	enum isp_sub_path_id spath_id;
	struct slice_store_info *slc_store;
	struct slice_scaler_info *slc_scaler;
	struct isp_fmcu_ctx_desc *fmcu;
};

struct isp_hw_slice_spath_thumbscaler {
	uint32_t path_en;
	uint32_t ctx_idx;
	struct isp_fmcu_ctx_desc *fmcu;
	struct slice_thumbscaler_info *slc_scaler;
};

struct isp_hw_ltm_3dnr_param {
	uint32_t idx;
	uint32_t val;
};

struct isp_hw_nlm_ynr {
	uint32_t val;
	uint32_t ctx_id;
	struct slice_cfg_input *slc_cfg_in;
};

struct dcam_hw_start_fetch {
	uint32_t start_x;
	uint32_t size_x;
	uint32_t fetch_pitch;
	uint32_t idx;
};

struct dcam_hw_cfg_bin_path {
	uint32_t idx;
	uint32_t start_x;
	uint32_t fetch_pitch;
};

struct isp_hw_fmcu_cmd {
	unsigned long base;
	unsigned long hw_addr;
	int cmd_num;
};

struct isp_hw_fmcu_start {
	unsigned long base;
	unsigned long hw_addr;
	int cmd_num;
};

struct isp_hw_fmcu_sel {
	uint32_t fmcu_id;
	uint32_t hw_idx;
};

struct isp_hw_yuv_block_ctrl {
	uint32_t idx;
	uint32_t type;
	struct isp_k_block *blk_param;
};

struct hw_io_ctrl_fun {
	uint32_t cmd;
	hw_ioctl_fun hw_ctrl;
};

struct glb_syscon {
	uint32_t rst;
	uint32_t rst_mask;
	uint32_t rst_ahb_mask;
	uint32_t rst_vau_mask;
	uint32_t all_rst;
	uint32_t all_rst_mask;
};

struct cam_hw_ip_info {
	uint32_t idx;
	uint32_t irq_no;
	uint32_t max_height;
	uint32_t max_width;
	unsigned long phy_base;
	unsigned long reg_base;
	struct glb_syscon syscon;

	/* For dcam support info */
	uint32_t slm_path;
	uint32_t lbuf_share_support;
	uint32_t offline_slice_support;
	uint32_t superzoom_support;
	uint32_t dcam_full_fbc_mode;
	uint32_t dcam_bin_fbc_mode;
	unsigned long *store_addr_tab;
	uint32_t *path_ctrl_id_tab;
	uint32_t afl_gbuf_size;
	unsigned long pdaf_type3_reg_addr;
	uint32_t rds_en;

	/* For isp support info */
	uint32_t slm_cfg_support;
	uint32_t *ctx_fmcu_support;
};

struct cam_hw_soc_info {
	struct platform_device *pdev;
	unsigned long axi_reg_base;

	struct regmap *cam_ahb_gpr;
	struct regmap *aon_apb_gpr;

	struct clk *clk;
	struct clk *clk_parent;
	struct clk *clk_default;
	struct clk *bpc_clk;
	struct clk *bpc_clk_parent;
	struct clk *bpc_clk_default;
	struct clk *core_eb;
	struct clk *axi_eb;
	struct clk *axi_clk;
	struct clk *axi_clk_parent;
	struct clk *axi_clk_default;

	uint32_t arqos_high;
	uint32_t arqos_low;
	uint32_t awqos_high;
	uint32_t awqos_low;
};

struct cam_hw_info {
	enum cam_prj_id prj_id;
	struct platform_device *pdev;
	struct cam_hw_soc_info *soc_dcam;
	struct cam_hw_soc_info *soc_isp;
	struct cam_hw_ip_info *ip_dcam[DCAM_ID_MAX];
	struct cam_hw_ip_info *ip_isp;
	int (*dcam_ioctl)(void *, uint32_t, void *);
	int (*isp_ioctl)(void *, uint32_t, void *);
};

#endif
