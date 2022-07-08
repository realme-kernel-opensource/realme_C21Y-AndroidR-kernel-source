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

#ifndef _CAM_TYPES_H_
#define _CAM_TYPES_H_

#include <linux/completion.h>
#include "sprd_img.h"

#ifndef MAX
#define MAX(__a, __b) (((__a) > (__b)) ? (__a) : (__b))
#endif
#ifndef MIN
#define MIN(__a, __b) (((__a) < (__b)) ? (__a) : (__b))
#endif

#define ZOOM_RATIO_DEFAULT              1000
#define CAM_BUF_ALIGN_SIZE              4
#define ALIGN_OFFSET                    16
#define FBC_TILE_WIDTH                  64
#define FBC_TILE_HEIGHT                 4
#define FBC_TILE_ADDR_ALIGN             256
#define FBC_HEADER_REDUNDANT            64

/*************** for global debug starts********************/

/* for iommu
 * 0: auto - dts configured
 * 1: iommu off and reserved memory
 * 2: iommu on and reserved memory
 * 3: iommu on and system memory
 */

enum cam_iommu_mode {
	IOMMU_AUTO = 0,
	IOMMU_OFF,
	IOMMU_ON_RESERVED,
	IOMMU_ON,
};
extern int g_dbg_iommu_mode;
extern int g_dbg_set_iommu_mode;

/* for global camera control */
struct cam_global_ctrl {
	uint32_t dcam_zoom_mode;
	uint32_t dcam_rds_limit;
	uint32_t isp_wmode;
	uint32_t isp_linebuf_len;
};
extern struct cam_global_ctrl g_camctrl;

/* for memory leak debug */
struct cam_mem_dbg_info {
	atomic_t ion_alloc_cnt;
	atomic_t ion_kmap_cnt;
	atomic_t ion_dma_cnt;
	atomic_t iommu_map_cnt[6];
	atomic_t empty_frm_cnt;
	atomic_t empty_state_cnt;
	atomic_t isp_sw_context_cnt;
};
extern struct cam_mem_dbg_info *g_mem_dbg;

enum {
	DUMP_DISABLE = 0,
	DUMP_PATH_BOTH,
	DUMP_PATH_FULL,
	DUMP_PATH_BIN,
};

/* for raw picture dump */
struct cam_dbg_dump {
	uint32_t dump_en;/* see enumerations above */
	uint32_t dump_count;
	uint32_t dump_ongoing;
	struct mutex dump_lock;
	struct completion *dump_start[3];
};
extern struct cam_dbg_dump g_dbg_dump;

/*************** for global debug ends ********************/

enum camera_slice_mode {
	CAM_SLICE_NONE = 0,
	CAM_OFFLINE_SLICE_HW,
	CAM_OFFLINE_SLICE_SW
};

enum camera_cap_type {
	CAM_CAP_NORMAL = 0,
	CAM_CAP_RAW_FULL,
	CAM_CAP_RAW_BIN
};

enum camera_id {
	CAM_ID_0 = 0,
	CAM_ID_1,
	CAM_ID_2,
	CAM_ID_3,
	CAM_ID_MAX,
};

enum camera_cap_status {
	CAM_CAPTURE_STOP = 0,
	CAM_CAPTURE_START,
	CAM_CAPTURE_RAWPROC,
	CAM_CAPTURE_RAWPROC_DONE,
};

enum cam_ch_id {
	CAM_CH_RAW = 0,
	CAM_CH_PRE,
	CAM_CH_VID,
	CAM_CH_CAP,
	CAM_CH_PRE_THM,
	CAM_CH_CAP_THM,
	CAM_CH_MAX,
};

enum cam_3dnr_type {
	CAM_3DNR_OFF = 0,
	CAM_3DNR_HW,
	CAM_3DNR_SW,
};

enum cam_slw_state {
	CAM_SLOWMOTION_OFF = 0,
	CAM_SLOWMOTION_ON,
};

enum cam_data_endian {
	ENDIAN_LITTLE = 0,
	ENDIAN_BIG,
	ENDIAN_HALFBIG,
	ENDIAN_HALFLITTLE,
	ENDIAN_MAX
};

enum cam_zoom_type {
	ZOOM_DEFAULT = 0,
	ZOOM_BINNING2,
	ZOOM_BINNING4,
	ZOOM_RDS,
	ZOOM_ADAPTIVE,
	ZOOM_TYPEMAX,
};

enum cam_raw_format {
	CAM_RAW_PACK10 = 0x00,
	CAM_RAW_HALF10 = 0x01,
	CAM_RAW_HALF14 = 0x02,
	CAN_RAW_FORMAT_MAX
};

enum cam_frame_scene {
	CAM_FRAME_COMMON = 0,
	CAM_FRAME_FDRL,
	CAM_FRAME_FDRH,
};

struct camera_format {
	char *name;
	uint32_t fourcc;
	int depth;
};

struct img_endian {
	uint32_t y_endian;
	uint32_t uv_endian;
};

struct img_addr {
	uint32_t addr_ch0;
	uint32_t addr_ch1;
	uint32_t addr_ch2;
};

struct img_pitch {
	uint32_t pitch_ch0;
	uint32_t pitch_ch1;
	uint32_t pitch_ch2;
};

struct img_size {
	uint32_t w;
	uint32_t h;
};

struct img_trim {
	uint32_t start_x;
	uint32_t start_y;
	uint32_t size_x;
	uint32_t size_y;
};

struct img_scaler_info {
	struct img_size src_size;
	struct img_trim src_trim;
	struct img_size dst_size;
};

struct cam_thread_info {
	atomic_t thread_stop;
	void *ctx_handle;
	int (*proc_func)(void *param);
	uint8_t thread_name[32];
	struct completion thread_com;
	struct completion thread_stop_com;
	struct task_struct *thread_task;
};

enum cam_evt_type {
	CAM_EVT_ERROR,
};

enum isp_cb_type {
	ISP_CB_GET_PMBUF,
	ISP_CB_RET_SRC_BUF,
	ISP_CB_RET_DST_BUF,
	ISP_CB_DEV_ERR,
	ISP_CB_MMU_ERR,
	ISP_CB_STATIS_DONE,
};

enum dcam_cb_type {
	DCAM_CB_DATA_DONE,
	DCAM_CB_IRQ_EVENT,
	DCAM_CB_STATIS_DONE,

	DCAM_CB_RET_SRC_BUF,
	DCAM_CB_RET_DST_BUF,
	DCAM_CB_GET_PMBUF,

	DCAM_CB_DEV_ERR,
	DCAM_CB_MMU_ERR,

};

/*
 * @addr0: start address of this buffer
 * @addr1: tile address for raw, y tile address for yuv
 * @addr2: low 2 bit address for raw, uv tile address for yuv
 */
struct compressed_addr {
	uint32_t addr0;
	uint32_t addr1;
	uint32_t addr2;
	uint32_t addr3;
};

extern struct camera_queue *g_empty_frm_q;
extern struct camera_queue *g_empty_state_q;

typedef int(*isp_dev_callback)(enum isp_cb_type type, void *param,
				void *priv_data);
typedef int(*dcam_dev_callback)(enum dcam_cb_type type, void *param,
				void *priv_data);

#endif/* _CAM_TYPES_H_ */
