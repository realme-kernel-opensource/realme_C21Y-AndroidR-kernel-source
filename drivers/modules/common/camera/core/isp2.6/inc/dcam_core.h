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

#ifndef _DCAM_CORE_H_
#define _DCAM_CORE_H_

#include "sprd_img.h"
#include <linux/sprd_ion.h>

#include "cam_types.h"
#include "cam_queue.h"
#include "dcam_interface.h"
#include "cam_block.h"
#include "cam_hw.h"

#define DCAM_IN_Q_LEN                     2
#define DCAM_PROC_Q_LEN                   12

/* TODO: extend these for slow motion dev */
#define DCAM_RESULT_Q_LEN                 12
#define DCAM_OUT_BUF_Q_LEN                25
#define DCAM_RESERVE_BUF_Q_LEN            12

#define DCAM_INTERNAL_RES_BUF_SIZE        0xC0000
#define DCAM_LSC_BUF_SIZE                 0x3000

#define DCAM_OFFLINE_TIMEOUT              msecs_to_jiffies(2000)
#define DCAM_OFFLINE_SLC_MAX              2

// TODO: how many helpers there should be?
#define DCAM_SYNC_HELPER_COUNT            20
/* DO NOT MODIFY!! */
#define DCAM_FRAME_TIMESTAMP_COUNT        0x40
/* get index of timestamp from frame index */
#define tsid(x)                           ((x) & (DCAM_FRAME_TIMESTAMP_COUNT - 1))
#define DCAM_FETCH_TWICE(dev)             (dev->raw_fetch_num > 1)
#define DCAM_FIRST_FETCH(dev)             (dev->raw_fetch_count == 1)
#define DCAM_LAST_FETCH(dev)              (dev->raw_fetch_count == 2)

struct dcam_pipe_dev;

typedef int (*func_dcam_cfg_param)(struct isp_io_param *param,
				struct dcam_dev_param *p);
struct dcam_cfg_entry {
	uint32_t sub_block;
	func_dcam_cfg_param cfg_func;
};

enum dcam_context_id {
	DCAM_CXT_0,
	DCAM_CXT_1,
	DCAM_CXT_2,
	DCAM_CXT_3,
	DCAM_CXT_NUM,
};

enum dcam_scaler_type {
	DCAM_SCALER_BINNING = 0,
	DCAM_SCALER_RAW_DOWNSISER,
	DCAM_SCALER_BYPASS,
	DCAM_SCALER_MAX,
};

enum dcam_path_state {
	DCAM_PATH_IDLE,
	DCAM_PATH_PAUSE,
	DCAM_PATH_RESUME,
};

struct dcam_rds_slice_ctrl {
	uint32_t rds_input_h_global;
	uint32_t rds_input_w_global;
	uint32_t rds_output_h_global;
	uint32_t rds_output_w_global;
	uint32_t rds_init_phase_int1;
	uint32_t rds_init_phase_int0;
	uint32_t rds_init_phase_rdm1;
	uint32_t rds_init_phase_rdm0;
};

struct dcam_path_desc {
	atomic_t user_cnt;
	enum dcam_path_id path_id;

	spinlock_t size_lock;
	uint32_t size_update;
	void *priv_size_data;

	spinlock_t state_lock;
	enum dcam_path_state state;
	uint32_t state_update;

	struct img_endian endian;
	struct img_size in_size;
	struct img_trim in_trim;
	struct img_size out_size;

	uint32_t base_update;
	uint32_t bayer_pattern;
	uint32_t out_fmt;
	uint32_t pack_bits;
	uint32_t is_4in1;

	/* full path source sel */
	uint32_t src_sel;

	/* for bin path */
	uint32_t is_slw;
	uint32_t slw_frm_num;
	uint32_t bin_ratio;
	uint32_t scaler_sel;/* 0: bining, 1: RDS, 2&3: bypass */
	void *rds_coeff_buf;
	uint32_t rds_coeff_size;

	uint32_t frm_deci;
	uint32_t frm_deci_cnt;

	uint32_t frm_skip;
	uint32_t frm_cnt;

	atomic_t set_frm_cnt;
	atomic_t is_shutoff;
	struct camera_frame *cur_frame;
	struct camera_queue reserved_buf_queue;
	struct camera_queue out_buf_queue;
	struct camera_queue alter_out_queue;
	struct camera_queue result_queue;
	struct dcam_rds_slice_ctrl gphase;
};

/*
 * state machine for DCAM
 *
 * @INIT:             initial state of this dcam_pipe_dev
 * @IDLE:             after hardware initialized, power/clk/int should be
 *                    prepared, which indicates registers are ready for
 *                    accessing
 * @RUNNING:          this dcam_pipe_dev is receiving data from CSI or DDR, and
 *                    writing data from one or more paths
 * @ERROR:            something is wrong, we should reset hardware right now
 */
enum dcam_state {
	STATE_INIT = 0,
	STATE_IDLE = 1,
	STATE_RUNNING = 2,
	STATE_ERROR = 3,
};

/*
 * Wrapper for dcam_frame_synchronizer. This helper uses kernel list to maintain
 * frame synchronizer data. Any operation on list should be protected by
 * helper_lock.
 *
 * @list:    support list operation
 * @sync:    underlaying synchronizer data
 * @enabled: enabled path for this exact frame, when it becomes zero, we should
 *           recycle this sync data
 * @dev:     pointer to the dcam_pipe_dev
 */
struct dcam_sync_helper {
	struct list_head list;
	struct dcam_frame_synchronizer sync;
	uint32_t enabled;
	struct dcam_pipe_dev *dev;
};

/* for multi dcam context (offline) */
struct dcam_pipe_context {
	atomic_t user_cnt;
	uint32_t ctx_id;
	struct dcam_dev_param blk_pm;
};

/*
 * A dcam_pipe_dev is a digital IP including one input for raw RGB or YUV
 * data, several outputs which usually be called paths. Input data can be
 * frames received from a CSI controller, or pure image data fetched from DDR.
 * Output paths contains a full path, a binning path and other data paths.
 * There may be raw domain processors in IP, each may or may NOT have data
 * output. Those who outputs data to some address in DDR behaves just same
 * as full or binning path. They can be treated as a special type of path.
 *
 * Each IP should implement a dcam_pipe_dev according to features it has. The
 * IP version may change, but basic function of DCAM IP is similar.
 *
 * @idx:               index of this device
 * @irq:               interrupt number for this device
 * @state:             working state of this device
 *
 * @frame_index:       frame index, tracked by CAP_SOF
 * @index_to_set:      index of next frame, or group of frames, updated in
 *                     CAP_SOF or initial phase
 * @need_fix:          leave fixing work to next CAP_SOF, only in slow motion
 *                     for now
 * @handled_bits:      mask bits that will not be handled this time
 * @iommu_status:      iommu status register
 * @frame_ts:          timestamp at SOF, time without suspend
 * @frame_ts_boot:     timestamp at SOF, ns counts from system boot
 *
 * @slowmotion_count:  frame count in a slow motion group, AKA slow motion rate
 *
 * @helper_enabled:    sync enabled path IDs
 * @helper_lock:       this lock protects synchronizer helper related data
 * @helper_list:       a list of sync helpers
 * @helpers:           memory for helpers
 *
 * @raw_proc_scene:    hwsim flag for offline process
 */
struct dcam_pipe_dev {
	uint32_t idx;
	uint32_t irq;
	atomic_t state;// TODO: use mutex to protect
	uint32_t auto_cpy_id;
	uint32_t base_fid;
	uint32_t frame_index;
	uint32_t index_to_set;
	bool need_fix;
	uint32_t handled_bits;
	uint32_t iommu_status;
	struct timespec frame_ts[DCAM_FRAME_TIMESTAMP_COUNT];
	ktime_t frame_ts_boot[DCAM_FRAME_TIMESTAMP_COUNT];
	uint32_t slowmotion_count;
	uint32_t helper_enabled;
	spinlock_t helper_lock;
	struct list_head helper_list;
	struct dcam_sync_helper helpers[DCAM_SYNC_HELPER_COUNT];
	struct dcam_image_replacer *replacer;
	struct cam_hw_info *hw;
	spinlock_t glb_reg_lock;
	bool dcamsec_eb;
	uint32_t err_status;// TODO: change to use state
	uint32_t err_count;/* iommu register dump count in dcam_err */
	uint32_t pack_bits;
	uint32_t is_4in1;
	uint32_t lowlux_4in1;/* 4in1 low lux mode capture */
	uint32_t skip_4in1;/* need skip 1 frame then change full source */
	uint32_t is_3dnr;
	uint32_t is_ebd;
	uint32_t offline;/* flag: set 1 for 4in1 go through dcam1 bin */
	uint32_t rps;/* raw_proc_scene 0:normal 1:hwsim*/
	uint32_t dcam_slice_mode;
	uint32_t slice_num;
	uint32_t slice_count;
	struct img_trim slice_trim;/* for sw slices */
	struct img_trim hw_slices[DCAM_OFFLINE_SLC_MAX];/* for offline hw slices */
	struct img_trim *cur_slice;
	uint32_t raw_cap;
	uint32_t raw_fetch_num;
	uint32_t raw_fetch_count;
	struct completion slice_done;
	struct completion frm_done;
	struct completion offline_complete;
	uint32_t zoom_ratio;
	struct img_trim next_roi;
	uint32_t iommu_enable;
	struct dcam_mipi_info cap_info;
	void *internal_reserved_buf;/* for statis path output */
	struct camera_buf statis_buf_array[STATIS_TYPE_MAX][STATIS_BUF_NUM_MAX];
	dcam_dev_callback dcam_cb_func;
	void *cb_priv_data;
	uint32_t cur_ctx_id;
	struct dcam_pipe_context ctx[DCAM_CXT_NUM];
	struct dcam_path_desc path[DCAM_PATH_MAX];
	struct dcam_fetch_info fetch;
	struct camera_queue in_queue;
	struct camera_queue proc_queue;
	struct cam_thread_info thread;
	struct dcam_pipe_ops *dcam_pipe_ops;
};

/*
 * Test if frame sync is enabled for path @path_id.
 */
#define is_sync_enabled(dev, path_id) (dev->helper_enabled & BIT(path_id))
/*
 * Get an empty dcam_sync_helper. Returns NULL if no empty helper remains.
 */
struct dcam_sync_helper *dcam_core_sync_helper_get(struct dcam_pipe_dev *dev);
/*
 * Put an empty dcam_sync_helper.
 *
 * In CAP_SOF, when the requested empty dcam_sync_helper finally not being used,
 * it should be returned to circle. This only happens when no buffer is
 * available and all paths are using reserved memory.
 */
void dcam_core_sync_helper_put(struct dcam_pipe_dev *dev,
			struct dcam_sync_helper *helper);
#endif
