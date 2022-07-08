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

#ifndef _DCAM_INTERFACE_H_
#define _DCAM_INTERFACE_H_

#include <linux/platform_device.h>

#include "cam_hw.h"
#include "cam_types.h"

/*
 * dcam_if fbc capability limit
 * modification to these values may cause some function in isp_slice.c not
 * work, check @ispslice_slice_fbd_raw_cfg and all other symbol references for details
 */
#define DCAM_FBC_TILE_WIDTH             64
#define DCAM_FBC_TILE_HEIGHT            4

/* 4-pixel align for MIPI CAP input from CSI */
#define DCAM_MIPI_CAP_ALIGN             4

/* 2-pixel align for RDS output size */
#define DCAM_RDS_OUT_ALIGN              2

/* 16-pixel align for debug convenience */
#define DCAM_OUTPUT_DEBUG_ALIGN         16

/* align size for full/bin crop, use 4 for zzhdr sensor, 2 for normal sensor */
#define DCAM_CROP_SIZE_ALIGN            4

#define DCAM_SCALE_DOWN_MAX             4

/*
 * Quick function to check is @idx valid.
 */
#define is_dcam_id(idx)                 ((idx) < DCAM_ID_MAX)


/*
 * Enumerating output paths in dcam_if device.
 *
 * @DCAM_PATH_FULL: with biggest line buffer, full path is often used as capture
 *                  path. Crop function available on this path.
 * @DCAM_PATH_BIN:  bin path is used as preview path. Crop and scale function
 *                  available on this path.
 * @DCAM_PATH_PDAF: this path is used to receive PDAF data
 * @DCAM_PATH_VCH2: can receive data according to data_type or
 *                  virtual_channel_id in a MIPI packet
 * @DCAM_PATH_VCH3: receive all data left
 * @DCAM_PATH_AEM:  output exposure by blocks
 * @DCAM_PATH_AFM:  output focus related data
 * @DCAM_PATH_AFL:  output anti-flicker data, including global data and region
 *                  data
 * @DCAM_PATH_HIST: output bayer histogram data in RGB channel
 * @DCAM_PATH_3DNR: output noise reduction data
 * @DCAM_PATH_BPC:  output bad pixel data
 */
enum dcam_path_id {
	DCAM_PATH_FULL = 0,
	DCAM_PATH_BIN,
	DCAM_PATH_PDAF,
	DCAM_PATH_VCH2,
	DCAM_PATH_VCH3,
	DCAM_PATH_AEM,
	DCAM_PATH_AFM,
	DCAM_PATH_AFL,
	DCAM_PATH_HIST,
	DCAM_PATH_3DNR,
	DCAM_PATH_BPC,
	DCAM_PATH_LSCM,
	DCAM_PATH_MAX,
};

struct statis_path_buf_info {
	enum dcam_path_id path_id;
	size_t buf_size;
	size_t buf_cnt;
	uint32_t buf_type;
};

/*
 * Quick function to check is @id valid.
 */
#define is_path_id(id) ((id) >= DCAM_PATH_FULL && (id) < DCAM_PATH_MAX)

enum dcam_path_cfg_cmd {
	DCAM_PATH_CFG_BASE = 0,
	DCAM_PATH_CFG_OUTPUT_BUF,
	DCAM_PATH_CFG_OUTPUT_ALTER_BUF,
	DCAM_PATH_CLR_OUTPUT_ALTER_BUF,
	DCAM_PATH_CFG_OUTPUT_RESERVED_BUF,
	DCAM_PATH_CFG_SIZE,
	DCAM_PATH_CFG_FULL_SOURCE,/* 4in1 select full path source */
	DCAM_PATH_CFG_SHUTOFF,
	DCAM_PATH_CFG_STATE,
};

/*
 * tile_num = w/64 * h/4;
 * head_bytes = tile_num * 4 / 8;
 * tile_bytes = tile_num * 256;
 * low2_bytes = w * h * 2 / 8;
 */
static inline void
dcam_if_cal_compressed_addr(uint32_t width,
			uint32_t height, unsigned long in,
			struct compressed_addr *out,
			uint32_t compress_4bit_bypass)
{
	uint32_t pixel_count = 0, low2 = 0, header_bytes = 0;
	int32_t tile_col = 0, tile_row = 0, header_size = 0;

	if (unlikely(!out))
		return;

	tile_col = (width + FBC_TILE_WIDTH - 1) / FBC_TILE_WIDTH;
	tile_col = (tile_col + 2 - 1) / 2 * 2;
	tile_row = (height + FBC_TILE_HEIGHT - 1) / FBC_TILE_HEIGHT;
	header_size = (tile_col * tile_row + 1) / 2;
	header_bytes = header_size + FBC_HEADER_REDUNDANT;
	pixel_count = tile_col * tile_row * FBC_TILE_ADDR_ALIGN;
	low2 = pixel_count >> 2;

	out->addr0 = (uint32_t)in;
	out->addr1 = ALIGN(out->addr0 + header_bytes, FBC_TILE_ADDR_ALIGN);//payload addr
	out->addr2 = ALIGN(out->addr1 + pixel_count, FBC_TILE_ADDR_ALIGN);//mid 2 bit_addr
	if (!compress_4bit_bypass)
		out->addr3 = ALIGN(out->addr2 + low2, FBC_TILE_ADDR_ALIGN);//low 4_bit addr
}

/* see @dcam_if_cal_compressed_addr */
static inline uint32_t
dcam_if_cal_compressed_size(uint32_t width,
	uint32_t height, uint32_t compress_4bit_bypass)
{
	uint32_t pixel_count = 0, header = 0, low2 = 0, low4 = 0;
	int32_t tile_col = 0, tile_row = 0, header_size = 0;

	tile_col = (width + FBC_TILE_WIDTH - 1) / FBC_TILE_WIDTH;
	tile_col = (tile_col + 2 - 1) / 2 * 2;
	tile_row = (height + FBC_TILE_HEIGHT - 1) / FBC_TILE_HEIGHT;
	header_size = (tile_col * tile_row + 1) / 2;
	pixel_count = tile_col * tile_row * FBC_TILE_ADDR_ALIGN;
	header = header_size + FBC_HEADER_REDUNDANT
		+ FBC_TILE_ADDR_ALIGN;
	low2 = (pixel_count >> 2) + FBC_TILE_ADDR_ALIGN;
	low4 = 0;
	if (!compress_4bit_bypass)
		low4 = (pixel_count >> 1) + FBC_TILE_ADDR_ALIGN;
	return pixel_count + header + low2 + low4;
}

enum dcam_ioctrl_cmd {
	DCAM_IOCTL_CFG_CAP,
	DCAM_IOCTL_CFG_STATIS_BUF,
	DCAM_IOCTL_PUT_RESERV_STATSBUF,
	DCAM_IOCTL_RECFG_PARAM,
	DCAM_IOCTL_INIT_STATIS_Q,
	DCAM_IOCTL_DEINIT_STATIS_Q,
	DCAM_IOCTL_CFG_EBD,
	DCAM_IOCTL_CFG_SEC,
	DCAM_IOCTL_CFG_FBC,
	DCAM_IOCTL_CFG_RPS, /* raw proc scene */
	DCAM_IOCTL_CFG_REPLACER,
	DCAM_IOCTL_GET_PATH_RECT,
	DCAM_IOCTL_CFG_STATIS_BUF_SKIP,
	DCAM_IOCTL_CFG_GTM_UPDATE,
};

/*
 *DCAM_STOP: normal stop;
 *DCAM_PAUSE_ONLINE: online paused; pause for fdr use same dcam for following offline process
 *DCAM_PAUSE_OFFLINE: offline paused; after fdr use same dcam for offline process and prepare for online resume
 */
enum dcam_stop_cmd {
	DCAM_STOP,
	DCAM_PAUSE_ONLINE,
	DCAM_PAUSE_OFFLINE
};

struct dcam_cap_cfg {
	uint32_t sensor_if;/* MIPI CSI-2 */
	uint32_t format;/* input color format */
	uint32_t mode;/* single or multi mode. */
	uint32_t data_bits;
	uint32_t pattern;/* bayer mode for rgb, yuv pattern for yuv */
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

struct dcam_path_cfg_param {
	uint32_t slowmotion_count;
	uint32_t enable_3dnr;
	uint32_t is_raw;
	uint32_t pack_bits;
	uint32_t bayer_pattern;
	uint32_t is_4in1;
	uint32_t frm_deci;
	uint32_t frm_skip;
	uint32_t force_rds;
	uint32_t raw_cap;
	void *priv_size_data;
	struct img_endian endian;
	struct img_size input_size;
	struct img_trim input_trim;
	struct img_size output_size;
};

/*
 * supported operations for dcam_if device
 *
 * @open:         initialize software and hardware resource for dcam_if
 * @close:        uninitialize resource for dcam_if
 * @start:        configure MIPI parameters and enable capture, parameters
 *                must be updated by ioctl() before this call
 * @stop:         // TODO: fill this
 * @get_path:
 * @put_path:
 * @cfg_path:
 * @ioctl:
 * @proc_frame:
 * @set_callback:
 * @update_clk:
 *
 */
struct dcam_pipe_ops {
	int (*open)(void *handle);
	int (*close)(void *handle);
	int (*reset)(void *handle);
	int (*start)(void *handle, int online);
	int (*stop)(void *handle, enum dcam_stop_cmd pause);
	int (*get_path)(void *handle, int path_id);
	int (*put_path)(void *handle, int path_id);
	int (*cfg_path)(void *dcam_handle,
				enum dcam_path_cfg_cmd cfg_cmd,
				int path_id, void *param);
	int (*ioctl)(void *handle, enum dcam_ioctrl_cmd cmd, void *arg);
	int (*cfg_blk_param)(void *handle, void *param);
	int (*proc_frame)(void *handle, void *param);
	int (*set_callback)(void *handle,
			dcam_dev_callback cb, void *priv_data);
	int (*update_clk)(void *handle, void *arg);
};

/*
 * A nr3_me_data object carries motion vector and settings. Downstream module
 * who peforms noice reduction operation uses these information to calculate
 * correct motion vector for target image size.
 *
 * *****Note: target image should not be cropped*****
 *
 * @valid:         valid bit
 * @sub_me_bypass: sub_me_bypass bit, this has sth to do with mv calculation
 * @project_mode:  project_mode bit, 0 for step, 1 for interlace
 * @mv_x:          motion vector in x direction
 * @mv_y:          motion vector in y direction
 * @src_width:     source image width
 * @src_height:    source image height
 */
struct nr3_me_data {
	uint32_t valid:1;
	uint32_t sub_me_bypass:1;
	uint32_t project_mode:1;
	s8 mv_x;
	s8 mv_y;
	uint32_t src_width;
	uint32_t src_height;
};

/*
 * Use dcam_frame_synchronizer to synchronize frames between different paths.
 * Normally we set WADDR for all paths in CAP_SOF interrupt, which is the best
 * opportunity that we bind buffers together and set frame number for them.
 * Modules such as 3DNR or LTM need to know the precise relationship between
 * bin/full buffer and statis buffer.
 * Some data which can be read directly from register is also included in this
 * object.
 * After data in image or statis buffer consumed, consumer modules must call
 * dcam_core_dcam_if_release_sync() to notify dcam_if.
 *
 * @index:             frame index tracked by dcam_if
 * @valid:             camera_frame valid bit for each path
 * @frames:            pointers to frames from each path
 *
 * @nr3_me:            nr3 data
 */
struct dcam_frame_synchronizer {
	uint32_t index;
	uint32_t valid;
	struct camera_frame *frames[DCAM_PATH_MAX];
	struct nr3_me_data nr3_me;
};

/*
 * Enables/Disables frame sync for path_id. Should be called before streaming.
 */
int dcamcore_dcam_if_sync_enable_set(void *handle, int path_id, int enable);
/*
 * Release frame sync reference for @frame thus dcam_frame_synchronizer data
 * can be recycled for next use.
 */
int dcam_core_dcam_if_release_sync(struct dcam_frame_synchronizer *sync,
			struct camera_frame *frame);
/*
 * Test if frame data is valid for path_id. Data will be valid only after
 * TX_DONE interrupt.
 */
#define dcam_if_is_sync_valid(sync, path_id) (sync->valid & BIT(path_id))

/*
 * Retrieve a dcam_if device for the hardware. A dcam_if device is a wrapper
 * with supported operations defined in dcam_pipe_ops.
 */
void *dcam_core_dcam_if_dev_get(uint32_t idx, struct cam_hw_info *hw);
/*
 * Release a dcam_if device after capture finished.
 */
int dcam_core_dcam_if_dev_put(void *dcam_handle);

/*
 * Retrieve hardware info from dt.
 */
int dcam_drv_hw_init(void *arg);
int dcam_drv_hw_deinit(void *arg);
int dcam_drv_dt_parse(struct platform_device *pdev,
			struct cam_hw_info *hw_info,
			uint32_t *dcam_count);

#endif/* _DCAM_INTERFACE_H_ */
