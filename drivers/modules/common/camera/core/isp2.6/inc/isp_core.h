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

#ifndef _ISP_CORE_H_
#define _ISP_CORE_H_

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sprd_ion.h>
#include "sprd_img.h"

#include "cam_types.h"
#include "cam_queue.h"
#include "cam_block.h"
#include "isp_interface.h"
#include "isp_3dnr.h"
#include "isp_ltm.h"

#define ISP_LINE_BUFFER_W          ISP_MAX_LINE_WIDTH
#define ISP_IN_Q_LEN               4
#define ISP_PROC_Q_LEN             2
#define ISP_RESULT_Q_LEN           2
#define ISP_SLW_IN_Q_LEN           12
#define ISP_SLW_PROC_Q_LEN         12
#define ISP_SLW_RESULT_Q_LEN       12
#define ISP_OUT_BUF_Q_LEN          32
#define ISP_RESERVE_BUF_Q_LEN      12
#define ISP_STREAM_STATE_Q_LEN     12
#define ISP_SW_CONTEXT_Q_LEN       ISP_SW_CONTEXT_NUM

#define ODATA_YUV420               1
#define ODATA_YUV422               0

#define ISP_PIXEL_ALIGN_WIDTH      4
#define ISP_PIXEL_ALIGN_HEIGHT     2

#define AFBC_PADDING_W_YUV420      32
#define AFBC_PADDING_H_YUV420      8
#define AFBC_HEADER_SIZE           16
#define AFBC_PAYLOAD_SIZE          84

#define ISP_FBD_TILE_WIDTH         64
#define ISP_FBD_TILE_HEIGHT        4
#define ISP_FBD_BASE_ALIGN         256

#define ISP_ALIGN_W(_a)            ((_a) & ~(ISP_PIXEL_ALIGN_WIDTH - 1))
#define ISP_ALIGN_H(_a)            ((_a) & ~(ISP_PIXEL_ALIGN_HEIGHT - 1))
#define ISP_DIV_ALIGN_W(_a, _b)    (((_a) / (_b)) & ~(ISP_PIXEL_ALIGN_WIDTH - 1))
#define ISP_DIV_ALIGN_H(_a, _b)    (((_a) / (_b)) & ~(ISP_PIXEL_ALIGN_HEIGHT - 1))

enum isp_work_mode {
	ISP_CFG_MODE,
	ISP_AP_MODE,
	ISP_WM_MAX
};

enum isp_raw_format {
	ISP_RAW_PACK10 = 0x00,
	ISP_RAW_HALF10 = 0x01,
	ISP_RAW_HALF14 = 0x02,
	ISP_RAW_FORMAT_MAX
};

enum isp_postproc_type {
	POSTPROC_FRAME_DONE,
	POSTPORC_HIST_DONE,
	POSTPROC_MAX,
};

struct isp_pipe_dev;
struct isp_sw_context;


typedef int (*func_isp_cfg_param)(
	struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx);

typedef int(*isp_irq_postproc)(void *handle, uint32_t idx,
	enum isp_postproc_type type);

struct isp_cfg_entry {
	uint32_t sub_block;
	func_isp_cfg_param cfg_func;
};

struct stream_ctrl_info {
	uint32_t src_fmt;
	struct img_size src;
	struct img_trim src_crop;
};

struct slice_cfg_input {
	uint32_t ltm_rgb_eb;
	uint32_t ltm_yuv_eb;
	struct img_size frame_in_size;
	struct img_size *frame_out_size[ISP_SPATH_NUM];
	struct isp_hw_fetch_info *frame_fetch;
	struct isp_fbd_raw_info *frame_fbd_raw;
	struct isp_hw_thumbscaler_info *thumb_scaler;
	struct isp_store_info *frame_store[ISP_SPATH_NUM];
	struct isp_afbc_store_info *frame_afbc_store[AFBC_PATH_NUM];
	struct isp_scaler_info *frame_scaler[ISP_SPATH_NUM];
	struct img_deci_info *frame_deci[ISP_SPATH_NUM];
	struct img_trim *frame_trim0[ISP_SPATH_NUM];
	struct img_trim *frame_trim1[ISP_SPATH_NUM];
	uint32_t nlm_center_x;
	uint32_t nlm_center_y;
	uint32_t ynr_center_x;
	uint32_t ynr_center_y;
	struct isp_3dnr_ctx_desc *nr3_ctx;
	struct isp_ltm_ctx_desc *ltm_ctx;
	struct isp_k_block *nofilter_ctx;
};

struct isp_path_desc {
	atomic_t user_cnt;
	atomic_t store_cnt;
	enum isp_sub_path_id spath_id;
	int32_t reserved_buf_fd;
	size_t reserve_buf_size[3];
	struct isp_sw_context *attach_ctx;
	struct cam_hw_info *hw;

	int q_init;
	struct camera_queue reserved_buf_queue;
	struct camera_queue out_buf_queue;
	struct camera_queue result_queue;
};

struct isp_pipe_info {
	struct isp_hw_fetch_info fetch;
	struct isp_fbd_raw_info fetch_fbd;
	struct isp_hw_path_scaler scaler[ISP_SPATH_NUM];
	struct isp_hw_thumbscaler_info thumb_scaler;
	struct isp_hw_path_store store[ISP_SPATH_NUM];
	struct isp_hw_afbc_path afbc[AFBC_PATH_NUM];
};

struct isp_path_uinfo {
	/* 1 for fbc store; 0 for normal store */
	uint32_t store_fbc;
	uint32_t out_fmt;
	uint32_t bind_type;
	uint32_t slave_path_id;
	uint32_t regular_mode;
	uint32_t uframe_sync;
	struct img_endian data_endian;
	struct img_size dst;
	struct img_trim in_trim;
};

struct isp_uinfo {
	/* common info from cam core */
	uint32_t in_fmt;
	uint32_t pack_bits;
	/* GR, R, B, Gb */
	uint32_t bayer_pattern;
	uint32_t ltm_rgb;
	uint32_t ltm_yuv;
	uint32_t mode_ltm;
	uint32_t mode_3dnr;
	uint32_t slw_state;
	uint32_t enable_slowmotion;
	uint32_t slowmotion_count;
	uint32_t uframe_sync;

	/* compression info from cam core */
	/* 1: fetch_fbd; 0: fetch */
	uint32_t fetch_path_sel;
	/* 0: 14bit; 1: 10bit */
	uint32_t fetch_fbd_4bit_bypass;
	/* 1: 3dnr compressed; 0: 3dnr plain data */
	uint32_t nr3_fbc_fbd;

	/* input info from cam core */
	struct img_size src;
	struct img_trim crop;
	struct img_scaler_info original;

	/* output info from cam core */
	struct isp_path_uinfo path_info[ISP_SPATH_NUM];
};

struct isp_sw_context {
	struct list_head list;
	atomic_t user_cnt;
	atomic_t state_user_cnt;
	uint32_t started;
	uint32_t ctx_id;
	uint32_t in_irq_handler;
	uint32_t iommu_status;
	uint32_t updated;
	enum camera_id attach_cam_id;
	enum cam_ch_id ch_id;

	struct isp_uinfo uinfo;
	struct isp_uinfo pipe_src;
	struct isp_pipe_info pipe_info;

	struct isp_path_desc isp_path[ISP_SPATH_NUM];
	struct isp_pipe_dev *dev;
	struct cam_hw_info *hw;
	void *slice_ctx;
	struct isp_k_block isp_k_param;
	struct isp_3dnr_ctx_desc nr3_ctx;
	struct isp_ltm_ctx_desc ltm_ctx;

	struct cam_thread_info thread;
	struct completion frm_done;
	struct completion slice_done;
	/* lock ctx/path param(size) updated from zoom */
	struct mutex param_mutex;
	/* lock block param to avoid acrossing frame */
	struct mutex blkpm_lock;

	struct camera_queue in_queue;
	struct camera_queue proc_queue;
	struct camera_queue stream_ctrl_in_q;
	struct camera_queue stream_ctrl_proc_q;

	struct camera_frame *postproc_buf;
	struct camera_frame *nr3_buf[ISP_NR3_BUF_NUM];
	struct camera_frame *ltm_buf[LTM_MAX][ISP_LTM_BUF_NUM];
	struct camera_buf statis_buf_array[STATIS_BUF_NUM_MAX];
	struct camera_queue hist2_result_queue;

	uint32_t multi_slice;
	uint32_t is_last_slice;
	uint32_t valid_slc_num;
	uint32_t sw_slice_num;
	uint32_t sw_slice_no;

	isp_irq_postproc postproc_func;
	isp_dev_callback isp_cb_func;
	void *cb_priv_data;
};

struct isp_hw_context {
	atomic_t user_cnt;
	uint32_t sw_ctx_id;
	uint32_t hw_ctx_id;
	uint32_t fmcu_used;
	void *fmcu_handle;
	struct isp_sw_context *pctx;
};

struct isp_pipe_dev {
	uint32_t irq_no[2];
	atomic_t user_cnt;
	atomic_t enable;
	struct mutex path_mutex;
	spinlock_t ctx_lock;
	enum isp_work_mode wmode;
	enum sprd_cam_sec_mode sec_mode;
	void *cfg_handle;
	struct isp_ltm_share_ctx_desc *ltm_handle;
	struct camera_queue sw_ctx_q;
	struct isp_sw_context *sw_ctx[ISP_SW_CONTEXT_NUM];
	struct isp_hw_context hw_ctx[ISP_CONTEXT_HW_NUM];
	struct cam_hw_info *isp_hw;
	struct isp_pipe_ops *isp_ops;
};

struct isp_statis_buf_size_info {
	enum isp_statis_buf_type buf_type;
	size_t buf_size;
	size_t buf_cnt;
};

int isp_core_hw_context_id_get(struct isp_sw_context *pctx);
int isp_core_sw_context_id_get(enum isp_context_hw_id hw_ctx_id, struct isp_pipe_dev *dev);
int isp_core_context_bind(struct isp_sw_context *pctx,  int fmcu_need);
int isp_core_context_unbind(struct isp_sw_context *pctx);

#endif
