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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

#include <sprd_mm.h>
#include <isp_hw.h>
#include "sprd_img.h"
#include "cam_trusty.h"
#include "cam_test.h"

#include "cam_debugger.h"
#include "isp_interface.h"
#include "flash_interface.h"

#include "sprd_sensor_drv.h"
#include "dcam_reg.h"
#include "csi_api.h"
#include "dcam_core.h"
#include "isp_core.h"

#ifdef CONFIG_COMPAT
#include "compat_cam_drv.h"
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define IMG_DEVICE_NAME                 "sprd_image"
#define CAMERA_TIMEOUT                  5000
#define CAMERA_LONGEXP_TIMEOUT          50000

#define THREAD_STOP_TIMEOUT             3000

#define CAM_COUNT                       CAM_ID_MAX
/* TODO: extend this for slow motion dev */
#define CAM_SHARED_BUF_NUM              25
#define CAM_FRAME_Q_LEN                 48
#define CAM_IRQ_Q_LEN                   16
#define CAM_STATIS_Q_LEN                16
#define CAM_ZOOM_COEFF_Q_LEN            10
#define CAM_PMBUF_Q_LEN                 (PARAM_BUF_NUM_MAX)
#define CAM_ALLOC_Q_LEN                 48

/* TODO: tuning ratio limit for power/image quality */
#define RATIO_SHIFT                     16

#define ISP_SLICE_OVERLAP_W_MAX         64
#define ALIGN_UP(a, x)                  (((a) + (x) - 1) & (~((x) - 1)))

/* TODO: need to pass the num to driver by hal */
#define CAP_NUM_COMMON                  1

/* limit the w & h to 1080P *2: 1920 *2 && 1080 *2 */
#define CAM_VIDEO_LIMIT_W               3840
#define CAM_VIDEO_LIMIT_H               2160
#define PRE_RDS_OUT                     3264
#define CAM_ZSL_NUM                     16
#define CAM_ZSL_SKIP_NUM                1

enum camera_module_state {
	CAM_INIT = 0,
	CAM_IDLE,
	CAM_CFG_CH,
	CAM_STREAM_ON,
	CAM_STREAM_OFF,
	CAM_RUNNING,
	CAM_ERROR,
};

/* Static Variables Declaration */
static struct camera_format output_img_fmt[] = {
	{ /*ISP_STORE_UYVY = 0 */
		.name = "4:2:2, packed, UYVY",
		.fourcc = IMG_PIX_FMT_UYVY,
		.depth = 16,
	},
	{ /* ISP_STORE_YUV422_3FRAME,*/
		.name = "YUV 4:2:2, planar, (Y-Cb-Cr)",
		.fourcc = IMG_PIX_FMT_YUV422P,
		.depth = 16,
	},
	{ /*ISP_STORE_YUV420_2FRAME*/
		.name = "YUV 4:2:0 planar (Y-CbCr)",
		.fourcc = IMG_PIX_FMT_NV12,
		.depth = 12,
	},
	{ /* ISP_STORE_YVU420_2FRAME,*/
		.name = "YVU 4:2:0 planar (Y-CrCb)",
		.fourcc = IMG_PIX_FMT_NV21,
		.depth = 12,
	},
	{ /*ISP_STORE_YUV420_3FRAME,*/
		.name = "YUV 4:2:0 planar (Y-Cb-Cr)",
		.fourcc = IMG_PIX_FMT_YUV420,
		.depth = 12,
	},
	{
		.name = "RawRGB",
		.fourcc = IMG_PIX_FMT_GREY,
		.depth = 8,
	},
};

struct camera_group;

/* user set information for camera module */
struct camera_uchannel {
	uint32_t sn_fmt;
	uint32_t dst_fmt;

	uint32_t deci_factor;/* for ISP output path */
	uint32_t is_high_fps;/* for DCAM slow motion feature */
	uint32_t high_fps_skip_num;/* for DCAM slow motion feature */
	uint32_t is_compressed;/* for ISP output fbc format */

	struct sprd_img_size src_size;
	struct sprd_img_rect src_crop;
	struct sprd_img_size dst_size;
	uint32_t scene;

	/* for binding small picture */
	uint32_t slave_img_en;
	uint32_t slave_img_fmt;
	struct sprd_img_size slave_img_size;

	struct dcam_regular_desc regular_desc;

	/* for close callback stream frame sync */
	uint32_t frame_sync_close;
};

struct camera_uinfo {
	/* cap info */
	struct sprd_img_sensor_if sensor_if;
	struct sprd_img_size sn_size;
	struct sprd_img_size sn_max_size;
	struct sprd_img_rect sn_rect;
	uint32_t capture_mode;
	uint32_t capture_skip;
	uint32_t is_longexp;
	uint32_t is_4in1;
	uint32_t is_3dnr;
	uint32_t is_rgb_ltm;
	uint32_t is_yuv_ltm;
	uint32_t is_dual;
	uint32_t dcam_slice_mode;/*1: hw,  2:sw*/
	uint32_t slice_num;
	uint32_t slice_count;
	uint32_t is_afbc;
};

struct sprd_img_flash_info {
	uint32_t led0_ctrl;
	uint32_t led1_ctrl;
	uint32_t led0_status;
	uint32_t led1_status;
	uint32_t flash_last_status;
};

struct channel_context {
	enum cam_ch_id ch_id;
	uint32_t enable;
	uint32_t frm_base_id;
	uint32_t frm_cnt;
	uint32_t pack_bits;;
	atomic_t err_status;

	uint32_t compress_input;
	uint32_t compress_4bit_bypass;
	uint32_t compress_3dnr;
	uint32_t compress_output;

	int32_t dcam_path_id;
	uint32_t second_path_id;/* second path */
	uint32_t second_path_enable;

	/* for which need anoter dcam & path offline processing.*/
	int32_t aux_dcam_path_id;

	int32_t isp_ctx_id;
	int32_t isp_path_id;
	int32_t slave_isp_ctx_id;
	int32_t slave_isp_path_id;
	int32_t isp_fdrl_ctx;
	int32_t isp_fdrl_path;
	int32_t isp_fdrh_ctx;
	int32_t isp_fdrh_path;

	uint32_t zsl_buffer_num;
	uint32_t zsl_skip_num;

	struct camera_uchannel ch_uinfo;
	struct img_size swap_size;
	struct img_trim trim_dcam;
	struct img_trim trim_isp;
	struct img_size dst_dcam;
	uint32_t rds_ratio;

	/* to store isp offline param data if frame is discarded. */
	void *isp_updata;

	uint32_t alloc_start;
	struct completion alloc_com;

	uint32_t uinfo_3dnr;/* set by hal, 1:hw 3dnr; */
	uint32_t type_3dnr;/* CAM_3DNR_HW:enable hw,and alloc buffer */
	uint32_t mode_ltm;
	uint32_t ltm_rgb;
	uint32_t ltm_yuv;
	struct camera_frame *fdrl_zoom_buf;
	struct camera_frame *fdrh_zoom_buf;
	struct camera_frame *postproc_buf;
	struct camera_frame *nr3_bufs[ISP_NR3_BUF_NUM];
	struct camera_frame *ltm_bufs[LTM_MAX][ISP_LTM_BUF_NUM];
	struct camera_frame *res_frame;
	int32_t reserved_buf_fd;

	/* dcam/isp shared frame buffer for full path */
	struct camera_queue share_buf_queue;
	struct camera_queue zoom_coeff_queue;/* channel specific coef queue */
};

struct camera_module {
	uint32_t idx;
	atomic_t state;
	atomic_t timeout_flag;
	struct mutex lock;
	struct camera_group *grp;
	uint32_t exit_flag;/*= 1, normal exit, =0, abnormal exit*/

	int attach_sensor_id;
	uint32_t iommu_enable;
	enum camera_cap_status cap_status;
	enum dcam_capture_status dcam_cap_status;

	struct isp_pipe_dev *isp_dev_handle;
	struct dcam_pipe_dev *dcam_dev_handle;
	/* for which need another dcam offline processing raw data*/
	void *aux_dcam_dev;
	struct cam_flash_task_info *flash_core_handle;
	uint32_t dcam_idx;
	uint32_t aux_dcam_id;

	struct mutex fdr_lock;
	uint32_t fdr_init;
	uint32_t fdr_done;
	uint32_t paused;

	uint32_t simu_fid;
	uint32_t simulator;
	uint32_t is_smooth_zoom;
	uint32_t zoom_solution;/* for dynamic zoom type swicth. */
	uint32_t rds_limit;/* raw downsizer limit */
	uint32_t binning_limit;/* binning limit: 1 - 1/2,  2 - 1/4 */
	uint32_t zoom_ratio;/* userspace zoom ratio for aem statis */
	struct camera_uinfo cam_uinfo;

	uint32_t last_channel_id;
	struct channel_context channel[CAM_CH_MAX];
	struct mutex buf_lock[CAM_CH_MAX];

	struct completion frm_com;
	struct camera_queue frm_queue;/* frame message queue for user*/
	struct camera_queue irq_queue;/* IRQ message queue for user*/
	struct camera_queue statis_queue;/* statis data queue or user*/
	struct camera_queue alloc_queue;/* alloc data queue or user*/

	struct cam_thread_info cap_thrd;
	struct cam_thread_info zoom_thrd;
	struct cam_thread_info buf_thrd;

	/*  dump raw  for debug*/
	struct cam_thread_info dump_thrd;
	struct camera_queue dump_queue;
	struct completion dump_com;
	struct timespec cur_dump_ts;
	uint32_t dump_count;
	uint32_t in_dump;

	/* for raw capture post process */
	struct completion streamoff_com;

	struct timer_list cam_timer;

	struct camera_queue zsl_fifo_queue;/* for cmp timestamp */
	struct camera_frame *dual_frame;/* 0: no, to find, -1: no need find */
	atomic_t capture_frames_dcam;/* how many frames to report, -1:always */
	atomic_t cap_skip_frames;
	int64_t capture_times;/* *ns, timestamp get from start_capture */
	uint32_t capture_scene;
	uint32_t lowlux_4in1;/* flag */
	struct camera_queue remosaic_queue;/* 4in1: save camera_frame when remosaic */
	uint32_t auto_3dnr;/* 1: enable hw,and alloc buffer before stream on */
	struct sprd_img_flash_info flash_info;
	uint32_t flash_skip_fid;
	uint32_t path_state;

	struct camera_buf pmbuf_array[PARAM_BUF_NUM_MAX];
	struct camera_queue param_queue;
	int pmq_init;
};

struct camera_group {
	atomic_t camera_opened;
	bool ca_conn;

	spinlock_t module_lock;
	uint32_t module_used;
	struct camera_module *module[CAM_COUNT];

	spinlock_t rawproc_lock;
	uint32_t rawproc_in;

	uint32_t dcam_count;/*dts cfg dcam count*/
	uint32_t isp_count;/*dts cfg isp count*/

	atomic_t runner_nr; /*running camera num*/

	struct miscdevice *md;
	struct platform_device *pdev;
	struct camera_queue empty_frm_q;
	struct camera_queue empty_state_q;
	struct sprd_cam_sec_cfg camsec_cfg;
	struct camera_debugger debugger;
	struct cam_hw_info *hw_info;
};

struct cam_ioctl_cmd {
	unsigned int cmd;
	int (*cmd_proc)(struct camera_module *module, unsigned long arg);
};

struct camera_queue *g_empty_frm_q;
struct camera_queue *g_empty_state_q;
struct cam_global_ctrl g_camctrl = {
	ZOOM_BINNING2,
	DCAM_SCALE_DOWN_MAX * 10,
	0,
	ISP_MAX_LINE_WIDTH
};

static inline uint32_t camcore_ratio16_divide(uint64_t num, uint32_t ratio16)
{
	return (uint32_t)div64_u64(num << 16, ratio16);
}

static inline uint32_t camcore_scale_fix(uint32_t size_in, uint32_t ratio16)
{
	uint64_t size_scaled;

	size_scaled = (uint64_t)size_in;
	size_scaled <<= (2 * RATIO_SHIFT);
	size_scaled = ((div64_u64(size_scaled, ratio16)) >> RATIO_SHIFT);
	return (uint32_t)size_scaled;
}

static inline void camcore_largest_crop_get(
	struct sprd_img_rect *crop_dst, struct sprd_img_rect *crop1)
{
	uint32_t end_x, end_y;
	uint32_t end_x_new, end_y_new;

	if (crop1) {
		end_x = crop_dst->x + crop_dst->w;
		end_y = crop_dst->y + crop_dst->h;
		end_x_new = crop1->x + crop1->w;
		end_y_new = crop1->y + crop1->h;

		crop_dst->x = MIN(crop1->x, crop_dst->x);
		crop_dst->y = MIN(crop1->y, crop_dst->y);
		end_x_new = MAX(end_x, end_x_new);
		end_y_new = MAX(end_y, end_y_new);
		crop_dst->w = end_x_new - crop_dst->x;
		crop_dst->h = end_y_new - crop_dst->y;
	}
}

static void camcore_diff_trim_get(struct sprd_img_rect *orig,
	uint32_t ratio16, struct img_trim *trim0, struct img_trim *trim1)
{
	trim1->start_x = camcore_scale_fix(orig->x - trim0->start_x, ratio16);
	trim1->start_y = camcore_scale_fix(orig->y - trim0->start_y, ratio16);
	trim1->size_x =  camcore_scale_fix(orig->w, ratio16);
	trim1->size_x = ALIGN(trim1->size_x, 2);
	trim1->size_y =  camcore_scale_fix(orig->h, ratio16);
}

static int camcore_cap_info_set(struct camera_module *module)
{
	int ret = 0;
	struct camera_uinfo *info = &module->cam_uinfo;
	struct sprd_img_sensor_if *sensor_if = &info->sensor_if;
	struct dcam_cap_cfg cap_info = { 0 };

	cap_info.mode = info->capture_mode;
	cap_info.frm_skip = info->capture_skip;
	cap_info.is_4in1 = info->is_4in1;
	cap_info.dcam_slice_mode = info->dcam_slice_mode;
	cap_info.sensor_if = sensor_if->if_type;
	cap_info.format = sensor_if->img_fmt;
	cap_info.pattern = sensor_if->img_ptn;
	cap_info.frm_deci = sensor_if->frm_deci;
	cap_info.is_cphy = sensor_if->if_spec.mipi.is_cphy;
	if (cap_info.sensor_if == DCAM_CAP_IF_CSI2) {
		cap_info.href = sensor_if->if_spec.mipi.use_href;
		cap_info.data_bits = sensor_if->if_spec.mipi.bits_per_pxl;
	}
	cap_info.cap_size.start_x = info->sn_rect.x;
	cap_info.cap_size.start_y = info->sn_rect.y;
	cap_info.cap_size.size_x = info->sn_rect.w;
	cap_info.cap_size.size_y = info->sn_rect.h;

	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_CFG_CAP, &cap_info);
	/* for dcam1 mipicap */
	if (info->dcam_slice_mode && module->aux_dcam_dev)
		ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->aux_dcam_dev,
				DCAM_IOCTL_CFG_CAP, &cap_info);
	return ret;
}

static int camcore_slice_num_info_get(struct sprd_img_size *src, struct sprd_img_size *dst)
{
	uint32_t slice_num, slice_w, slice_w_out;
	uint32_t slice_max_w, max_w;
	uint32_t linebuf_len;
	uint32_t input_w = src->w;
	uint32_t output_w = dst->w;

	/* based input */
	linebuf_len = g_camctrl.isp_linebuf_len;
	max_w = input_w;
	slice_num = 1;
	slice_max_w = linebuf_len - ISP_SLICE_OVERLAP_W_MAX;
	if (max_w <= linebuf_len) {
		slice_w = max_w;
	} else {
		do {
			slice_num++;
			slice_w = (max_w + slice_num - 1) / slice_num;
		} while (slice_w >= slice_max_w);
	}
	pr_debug("input_w %d, slice_num %d, slice_w %d\n",
		max_w, slice_num, slice_w);

	/* based output */
	max_w = output_w;
	slice_num = 1;
	slice_max_w = linebuf_len;
	if (max_w > 0) {
		if (max_w > linebuf_len) {
			do {
				slice_num++;
				slice_w_out = (max_w + slice_num - 1) / slice_num;
			} while (slice_w_out >= slice_max_w);
		}
		/* set to equivalent input size, because slice size based on input. */
		slice_w_out = (input_w + slice_num - 1) / slice_num;
	} else
		slice_w_out = slice_w;
	pr_debug("max output w %d, slice_num %d, out limited slice_w %d\n",
		max_w, slice_num, slice_w_out);

	slice_w = MIN(slice_w, slice_w_out);
	slice_w = ALIGN(slice_w, 2);
	slice_num = (input_w + slice_w - 1) / slice_w;
	if (dst->h > DCAM_SW_SLICE_HEIGHT_MAX)
		slice_num *= 2;
	return slice_num;
}

/* 4in1_raw_capture
 * get the second buffer from the same fd for (bin) path
 * input: i: get i group buffer
 */
static struct camera_frame *camcore_secondary_buf_get(
	struct sprd_img_parm *p, struct channel_context *ch, uint32_t i)
{
	struct camera_frame *pframe;
	int ret;
	uint32_t offset = 0;
	uint32_t pack_bits = 0;

	pframe = cam_queue_empty_frame_get();
	pframe->buf.type = CAM_BUF_USER;
	pframe->buf.mfd[0] = p->fd_array[i];
	/* raw capture: 4cell + bin-sum, cal offset */
	if(ch->dcam_path_id == 0)
		pack_bits = 0;
	else
		pack_bits = ch->pack_bits;
	offset = cal_sprd_raw_pitch(ch->ch_uinfo.src_size.w, pack_bits);
	offset *= ch->ch_uinfo.src_size.h;
	offset = ALIGN_UP(offset, 4096);
	/* first buf offset: p->frame_addr_array[i].y */
	offset += p->frame_addr_array[i].y;
	pr_debug("start 0x%x 0x%x 0x%x offset 0x%x\n",
		p->frame_addr_array[i].y,
		p->frame_addr_array[i].u,
		p->frame_addr_array[i].v, offset);
	pframe->buf.offset[0] = offset;
	pframe->channel_id = ch->ch_id;
	pframe->img_fmt = ch->ch_uinfo.dst_fmt;

	ret = cam_buf_ionbuf_get(&pframe->buf);
	if (ret) {
		cam_queue_empty_frame_put(pframe);
		pr_err("fail to get second buffer fail, ret %d\n", ret);
		return NULL;
	}

	return pframe;
}

static int camcore_capture_3dnr_set(struct camera_module *module,
		struct channel_context *ch)
{
	uint32_t mode_3dnr;

	if ((!module) || (!ch))
		return -EFAULT;
	mode_3dnr = MODE_3DNR_OFF;
	if (ch->uinfo_3dnr) {
		if (ch->ch_id == CAM_CH_CAP)
			mode_3dnr = MODE_3DNR_CAP;
		else
			mode_3dnr = MODE_3DNR_PRE;
	}
	pr_debug("mode %d\n", mode_3dnr);
	module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle, ISP_PATH_CFG_3DNR_MODE,
		ch->isp_ctx_id,
		ch->isp_path_id, &mode_3dnr);

	return 0;
}

/* return the number of how many buf in the out_buf_queue */
uint32_t camcore_outbuf_queue_cnt_get(void *dev, int path_id)
{
	struct dcam_path_desc *path;

	path = &(((struct dcam_pipe_dev *)dev)->path[path_id]);

	return cam_queue_cnt_get(&path->out_buf_queue);
}

static void camcore_k_frame_put(void *param)
{
	int ret = 0;
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct camera_frame *)param;
	if (frame->buf.type == CAM_BUF_USER)
		cam_buf_ionbuf_put(&frame->buf);
	else {
		if (frame->buf.mapping_state)
			cam_buf_kunmap(&frame->buf);
		cam_buf_free(&frame->buf);
	}
	ret = cam_queue_empty_frame_put(frame);
}

static void camcore_empty_frame_put(void *param)
{
	int ret = 0;
	struct camera_frame *frame;
	struct camera_module *module;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct camera_frame *)param;
	module = frame->priv_data;
	if (frame->priv_data) {
		if (!frame->irq_type)
			kfree(frame->priv_data);
		else if (module && module->exit_flag == 1)
			cam_buf_ionbuf_put(&frame->buf);
	}
	ret = cam_queue_empty_frame_put(frame);
}

/* No need release buffer, only give back camera_frame
 * for remosaic_queue, it save camera_frame info when
 * buf send to hal for remosaic, use again when 4in1_post
 */
static void camcore_camera_frame_release(void *param)
{
	struct camera_frame *frame;

	if (!param)
		return;
	frame = (struct camera_frame *)param;
	cam_queue_empty_frame_put(frame);
}

static int camcore_resframe_set(struct camera_module *module)
{
	int ret = 0;
	struct channel_context *ch = NULL, *ch_prv = NULL;
	uint32_t i = 0, j = 0, cmd = ISP_PATH_CFG_OUTPUT_RESERVED_BUF;
	uint32_t max_size = 0, out_size = 0, in_size = 0;
	struct camera_frame *pframe = NULL;
	struct camera_frame *pframe1;

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (ch->enable) {
			if (ch->ch_uinfo.dst_fmt != IMG_PIX_FMT_GREY)
				out_size = ch->ch_uinfo.dst_size.w *ch->ch_uinfo.dst_size.h * 3 / 2;
			else
				out_size = cal_sprd_raw_pitch(ch->ch_uinfo.dst_size.w, module->cam_uinfo.sensor_if.if_spec.mipi.is_loose)
					* ch->ch_uinfo.dst_size.h;

			if (ch->ch_uinfo.sn_fmt != IMG_PIX_FMT_GREY)
				in_size = ch->ch_uinfo.src_size.w *ch->ch_uinfo.src_size.h * 3 / 2;
			else
				in_size = cal_sprd_raw_pitch(ch->ch_uinfo.src_size.w, module->cam_uinfo.sensor_if.if_spec.mipi.is_loose)
					* ch->ch_uinfo.src_size.h;

			max_size = max3(max_size, out_size, in_size);
			pr_debug("cam%d, ch %d, max_size = %d, %d, %d\n", module->idx, i, max_size, in_size, out_size);
		}
	}

	ch_prv = &module->channel[CAM_CH_PRE];
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		pframe = ch->res_frame;
		if (!ch->enable || !pframe)
			continue;
		if (ch->isp_path_id >= 0 && ch->ch_uinfo.dst_fmt != IMG_PIX_FMT_GREY) {
			if (((ch->ch_id == CAM_CH_CAP)
				|| (ch->ch_id == CAM_CH_PRE)
				|| (ch->ch_id == CAM_CH_VID && !ch_prv->enable))) {
				cmd = DCAM_PATH_CFG_OUTPUT_RESERVED_BUF;
				pframe1 = cam_queue_empty_frame_get();
				pframe1->is_reserved = 1;
				pframe1->buf.type = CAM_BUF_USER;
				pframe1->buf.mfd[0] = pframe->buf.mfd[0];
				pframe1->buf.offset[0] = pframe->buf.offset[0];
				pframe1->buf.offset[1] = pframe->buf.offset[1];
				pframe1->buf.offset[2] = pframe->buf.offset[2];
				pframe1->channel_id = ch->ch_id;

				ret = cam_buf_ionbuf_get(&pframe1->buf);
				if (ret) {
					pr_err("fail to get ionbuf on cam%d, ch %d\n", module->idx, i);
					cam_queue_empty_frame_put(pframe1);
					ret = -EFAULT;
					break;
				}

				for (j = 0; j < 3; j++)
					pframe1->buf.size[j] = max_size;
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					cmd, ch->dcam_path_id, pframe1);
				if (ret) {
					pr_err("fail to cfg path on cam%d, ch %d\n", module->idx, i);
					cam_buf_ionbuf_put(&pframe1->buf);
					cam_queue_empty_frame_put(pframe1);
				}
			}

			cmd = ISP_PATH_CFG_OUTPUT_RESERVED_BUF;
			for (j = 0; j < 3; j++)
				pframe->buf.size[j] = max_size;

			ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle, cmd,
				ch->isp_ctx_id, ch->isp_path_id, pframe);
		} else {
			cmd = DCAM_PATH_CFG_OUTPUT_RESERVED_BUF;
			for (j = 0; j < 3; j++)
				pframe->buf.size[j] = max_size;

			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				cmd, ch->dcam_path_id, pframe);
			/* 4in1_raw_capture, maybe need two image once */
			if (ch->second_path_enable) {
				pframe1 = cam_queue_empty_frame_get();
				pframe1->is_reserved = 1;
				pframe1->buf.type = CAM_BUF_USER;
				pframe1->buf.mfd[0] = pframe->buf.mfd[0];
				pframe1->buf.offset[0] = pframe->buf.offset[0];
				pframe1->buf.offset[1] = pframe->buf.offset[1];
				pframe1->buf.offset[2] = pframe->buf.offset[2];
				pframe1->channel_id = ch->ch_id;

				ret = cam_buf_ionbuf_get(&pframe1->buf);
				if (ret) {
					pr_err("fail to get ionbuf on cam%d, ch %d\n", module->idx, i);
					cam_queue_empty_frame_put(pframe1);
					ret = -EFAULT;
					break;
				}

				for (j = 0; j < 3; j++)
					pframe1->buf.size[j] = max_size;
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					cmd, ch->second_path_id, pframe1);
			}
		}

		if (ret) {
			pr_err("fail to set output buffer for ch%d.\n", ch->ch_id);
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}
	}

	return ret;
}

static int camcore_param_buffer_cfg(
		struct camera_module *module,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	int j;
	int32_t mfd;
	uint32_t offset;
	enum isp_statis_buf_type stats_type;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;

	stats_type = input->type;
	if (stats_type == STATIS_DBG_INIT)
		goto buf_init;
	if (stats_type == STATIS_PARAM &&
		atomic_read(&module->state) == CAM_RUNNING)
		goto cfg_single;

	return 0;

buf_init:

	pr_info("cam%d, start\n", module->idx);
	memset(&module->pmbuf_array[0], 0, sizeof(module->pmbuf_array));
	cam_queue_init(&module->param_queue,
		CAM_PMBUF_Q_LEN, camcore_camera_frame_release);

	for (j = 0; j < PARAM_BUF_NUM_MAX; j++) {
		mfd = input->mfd_pmdbg[j];
		if (mfd <= 0)
			continue;

		pr_debug("cam%d, param buf %d mfd %d, offset %d\n",
			module->idx, j, mfd, input->offset_pmdbg[j]);

		ion_buf = &module->pmbuf_array[j];
		ion_buf->mfd[0] = mfd;
		ion_buf->offset[0] = input->offset_pmdbg[j];
		ion_buf->type = CAM_BUF_USER;
		ret = cam_buf_ionbuf_get(ion_buf);
		if (ret) {
			memset(ion_buf, 0, sizeof(struct camera_buf));
			continue;
		}
		ret = cam_buf_kmap(ion_buf);
		if (ret) {
			pr_err("fail to kmap statis buf %d\n", mfd);
			cam_buf_ionbuf_put(ion_buf);
			memset(ion_buf, 0, sizeof(struct camera_buf));
			continue;
		}

		pframe = cam_queue_empty_frame_get();
		pframe->irq_property = STATIS_PARAM;
		pframe->buf = *ion_buf;
		ret = cam_queue_enqueue(&module->param_queue, &pframe->list);
		if (ret) {
			pr_warn("cam%d pmbufq overflow\n", module->idx);
			cam_queue_empty_frame_put(pframe);
		}

		pr_debug("cam%d,mfd %d, off %d, kaddr 0x%lx\n",
			module->idx, mfd, ion_buf->offset[0], ion_buf->addr_k[0]);
	}

	module->pmq_init = 1;
	pr_info("cam%d init done\n", module->idx);
	return 0;

cfg_single:
	for (j = 0; j < PARAM_BUF_NUM_MAX; j++) {
		mfd = module->pmbuf_array[j].mfd[0];
		offset = module->pmbuf_array[j].offset[0];
		if ((mfd > 0) && (mfd == input->mfd)
			&& (offset == input->offset)) {
			ion_buf = &module->pmbuf_array[j];
			break;
		}
	}

	if (ion_buf == NULL) {
		pr_err("fail to get pm buf %d\n", input->mfd);
		ret = -EINVAL;
		goto exit;
	}

	pframe = cam_queue_empty_frame_get();
	pframe->irq_property = input->type;
	pframe->buf = *ion_buf;
	ret = cam_queue_enqueue(&module->param_queue, &pframe->list);
	pr_debug("cam%d, pmbuf, mfd %d, off %d,kaddr 0x%lx\n",
		module->idx, mfd, offset, pframe->buf.addr_k[0]);
	if (ret)
		cam_queue_empty_frame_put(pframe);
exit:
	return ret;
}

static int camcore_param_buffer_uncfg(struct camera_module *module)
{
	int j;
	int32_t mfd;
	struct camera_buf *ion_buf = NULL;

	if (module->pmq_init == 0)
		return 0;

	module->pmq_init = 0;
	cam_queue_clear(&module->param_queue, struct camera_frame, list);

	for (j = 0; j < PARAM_BUF_NUM_MAX; j++) {
		ion_buf = &module->pmbuf_array[j];
		mfd = ion_buf->mfd[0];
		if (mfd <= 0)
			continue;

		pr_debug("cam%d, j %d,  mfd %d, offset %d\n",
			module->idx, j, mfd, ion_buf->offset[0]);
		cam_buf_kunmap(ion_buf);
		cam_buf_ionbuf_put(ion_buf);
		memset(ion_buf, 0, sizeof(struct camera_buf));
	}

	pr_info("cam%d done\n", module->idx);
	return 0;
}

static void camcore_compression_cal(struct camera_module *module)
{
	struct channel_context *ch_pre, *ch_cap, *ch_vid;
	struct cam_hw_info *dcam_hw;
	struct compression_override *override;

	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];

	/*
	 * Enable compression for DCAM path by default. Full path is prior to
	 * bin path.
	 */
	ch_cap->compress_input = ch_cap->enable
		&& ch_cap->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY
		&& !ch_cap->ch_uinfo.is_high_fps
		&& !module->cam_uinfo.is_4in1;
	ch_pre->compress_input = ch_pre->enable
		&& ch_pre->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY
		&& !ch_pre->ch_uinfo.is_high_fps
		&& !ch_cap->compress_input
		&& !module->cam_uinfo.is_4in1;
	ch_vid->compress_input = ch_pre->compress_input;

	/* Disable compression for 3DNR by default */
	ch_cap->compress_3dnr = 0;
	ch_pre->compress_3dnr = 0;
	ch_vid->compress_3dnr = ch_pre->compress_3dnr;

	/*
	 * Enable compression for ISP store according to HAL setting. Normally
	 * this only happens in slow motion and only for video path.
	 */
	ch_cap->compress_output =
		ch_cap->enable && ch_cap->ch_uinfo.is_compressed;
	ch_pre->compress_output =
		ch_pre->enable && ch_pre->ch_uinfo.is_compressed;
	ch_vid->compress_output =
		ch_vid->enable && ch_vid->ch_uinfo.is_compressed;

	/* disable all compression on SharkL5 */
	dcam_hw = module->grp->hw_info;
	if (dcam_hw->prj_id == SHARKL5) {
		ch_cap->compress_input = ch_cap->compress_output =
			ch_cap->compress_3dnr = 0;
		ch_pre->compress_input = ch_pre->compress_output =
			ch_pre->compress_3dnr = 0;
		ch_vid->compress_input = ch_vid->compress_output =
			ch_vid->compress_3dnr = 0;
	}

	/* TODO disable all fbc/fbd for bug 1040757 */
	ch_cap->compress_input = ch_cap->compress_3dnr
		= ch_cap->compress_output = 0;
	ch_pre->compress_input = ch_pre->compress_3dnr
		= ch_pre->compress_output = 0;
	ch_vid->compress_input = ch_vid->compress_3dnr
		= ch_vid->compress_output = 0;

	/* Bypass compression low_4bit by default */
	ch_cap->compress_4bit_bypass = 1;
	ch_pre->compress_4bit_bypass = 1;
	ch_vid->compress_4bit_bypass = 1;

	/* open compression on SharkL5 pro */
	if (dcam_hw->prj_id == SHARKL5pro) {
		/* dcam support full path or bin path */
		ch_cap->compress_input = 0;
		ch_pre->compress_input = 0;
		ch_vid->compress_input = 0;
		/* isp support preview path and video path*/
		ch_vid->compress_output = module->cam_uinfo.is_afbc;
		/* 3dnr support preview&capture&video path*/
		ch_cap->compress_3dnr = 0;
		ch_pre->compress_3dnr = 0;
		ch_vid->compress_3dnr = ch_pre->compress_3dnr;
	}

	/* manually control compression policy here */
	override = &module->grp->debugger.compression[module->idx];
	if (override->enable) {
		ch_cap->compress_input = override->override[CH_CAP][FBC_DCAM];
		ch_cap->compress_3dnr = override->override[CH_CAP][FBC_3DNR];
		ch_cap->compress_output = override->override[CH_CAP][FBC_ISP];

		ch_pre->compress_input = override->override[CH_PRE][FBC_DCAM];
		ch_pre->compress_3dnr = override->override[CH_PRE][FBC_3DNR];
		ch_pre->compress_output = override->override[CH_PRE][FBC_ISP];

		ch_vid->compress_input = override->override[CH_VID][FBC_DCAM];
		ch_vid->compress_3dnr = override->override[CH_VID][FBC_3DNR];
		ch_vid->compress_output = override->override[CH_VID][FBC_ISP];
	}

	if (module->dcam_idx > DCAM_ID_1) {
		ch_cap->compress_input = 0;
		ch_pre->compress_input = 0;
	}

	/* dcam not support fbc when dcam need fetch */
	if (module->cam_uinfo.dcam_slice_mode ||
		module->cam_uinfo.is_4in1)
		ch_cap->compress_input = 0;

	/* dcam not support fbc when open slowmotion */
	if (ch_pre->ch_uinfo.is_high_fps)
		ch_pre->compress_input = 0;

	pr_info("cam%d: cap %u %u %u, pre %u %u %u, vid %u %u %u\n",
		module->idx,
		ch_cap->compress_input, ch_cap->compress_3dnr,
		ch_cap->compress_output,
		ch_pre->compress_input, ch_pre->compress_3dnr,
		ch_pre->compress_output,
		ch_vid->compress_input, ch_vid->compress_3dnr,
		ch_vid->compress_output);
}

static void camcore_compression_config(struct camera_module *module)
{
	struct channel_context *ch_pre, *ch_cap, *ch_vid;
	struct isp_ctx_compress_desc ctx_compression_desc;
	struct isp_path_compression_desc path_compression_desc;
	struct cam_hw_info *hw = NULL;
	int fbc_mode = DCAM_FBC_DISABLE;
	struct compression_override *override = NULL;

	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
	hw = module->grp->hw_info;
	override = &module->grp->debugger.compression[module->idx];

	if (ch_cap->compress_input) {
		fbc_mode = hw->ip_dcam[module->dcam_idx]->dcam_full_fbc_mode;
		/* manually control compression policy here */
		if (override->enable)
			fbc_mode = override->override[CH_CAP][FBC_DCAM];
		if (DCAM_FBC_FULL_14_BIT == fbc_mode)
			ch_cap->compress_4bit_bypass = 0;
	}

	if (ch_pre->compress_input) {
		fbc_mode = hw->ip_dcam[module->dcam_idx]->dcam_bin_fbc_mode;
		/* manually control compression policy here */
		if (override->enable)
			fbc_mode = override->override[CH_PRE][FBC_DCAM];
		if (DCAM_FBC_BIN_14_BIT == fbc_mode)
			ch_pre->compress_4bit_bypass = 0;
	}

	pr_debug("fbc = %d\n", fbc_mode);
	ch_vid->compress_input = ch_pre->compress_input;
	ch_vid->compress_4bit_bypass = ch_pre->compress_4bit_bypass;

	module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_FBC, &fbc_mode);

	/* capture context */
	if (ch_cap->enable) {
		ctx_compression_desc.fetch_fbd = ch_cap->compress_input;
		ctx_compression_desc.fetch_fbd_4bit_bypass = ch_cap->compress_4bit_bypass;
		ctx_compression_desc.nr3_fbc_fbd = ch_cap->compress_3dnr;
		module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_CTX_COMPRESSION,
				ch_cap->isp_ctx_id,
				0, &ctx_compression_desc);

		path_compression_desc.store_fbc = ch_cap->compress_output;
		module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_PATH_COMPRESSION,
				ch_cap->isp_ctx_id,
				ch_cap->isp_path_id,
				&path_compression_desc);
	}

	/* preview context */
	if (ch_pre->enable) {
		ctx_compression_desc.fetch_fbd = ch_pre->compress_input;
		ctx_compression_desc.fetch_fbd_4bit_bypass = ch_pre->compress_4bit_bypass;
		ctx_compression_desc.nr3_fbc_fbd = ch_pre->compress_3dnr;
		module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_CTX_COMPRESSION,
				ch_pre->isp_ctx_id,
				0, &ctx_compression_desc);

		path_compression_desc.store_fbc = ch_pre->compress_output;
		module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_PATH_COMPRESSION,
				ch_pre->isp_ctx_id,
				ch_pre->isp_path_id,
				&path_compression_desc);
	}

	/* video context */
	if (ch_vid->enable) {
		ctx_compression_desc.fetch_fbd = ch_vid->compress_input;
		ctx_compression_desc.fetch_fbd_4bit_bypass = ch_vid->compress_4bit_bypass;
		ctx_compression_desc.nr3_fbc_fbd = ch_vid->compress_3dnr;
		module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_CTX_COMPRESSION,
				ch_vid->isp_ctx_id,
				0, &ctx_compression_desc);

		path_compression_desc.store_fbc = ch_vid->compress_output;
		module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_PATH_COMPRESSION,
				ch_vid->isp_ctx_id,
				ch_vid->isp_path_id,
				&path_compression_desc);
	}
}

static void camcore_prepare_frame_from_file(struct camera_queue *queue,
				char *filename,
				uint32_t width, uint32_t height, uint32_t pack_bits)
{
	struct camera_frame *frame, *f;
	struct file *raw;
	uint8_t *buf;
	char fullname[DCAM_IMAGE_REPLACER_FILENAME_MAX + 32] = { 0 };
	const char *folder = "/data/ylog/";
	size_t cur;
	uint32_t total;
	int64_t left;
	const uint32_t per = 4096;
	ktime_t start, stop;
	int result = 0;

	/* prepare 1st buffer */
	frame = cam_queue_dequeue_tail(queue);
	if (!frame)
		return;

	if (frame->is_compressed)
		total = dcam_if_cal_compressed_size(width, height,
			frame->compress_4bit_bypass);
	else
		total = cal_sprd_raw_pitch(width, pack_bits) * height;

	strcpy(fullname, folder);
	/* length of filename is less then DCAM_IMAGE_REPLACER_FILENAME_MAX */
	strcpy(fullname + strlen(folder), filename);

	pr_info("reading %u bytes from %s\n", total, fullname);
	start = ktime_get_boottime();
	raw = filp_open(fullname, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(raw)) {
		pr_err("fail to open data file\n");
		goto enqueue_frame;
	}

	buf = (uint8_t *)frame->buf.addr_k[0];
	left = total;
	do {
		cur = min((uint32_t)left, per);
		result = kernel_read(raw, buf, cur, &raw->f_pos);
		buf += result;
		left -= result;
	} while (result > 0 && left > 0);
	filp_close(raw, 0);
	stop = ktime_get_boottime();
	pr_info("read succeed, costs %lldns\n", ktime_sub(stop, start));

	/* prepare other buffers */
	list_for_each_entry(f, &queue->head, list) {
		start = ktime_get_boottime();
		memcpy((uint8_t *)f->buf.addr_k[0],
			(uint8_t *)frame->buf.addr_k[0], total);
		stop = ktime_get_boottime();
		pr_info("copy succeed, costs %lldns\n", ktime_sub(stop, start));
	}
	pr_info("done\n");

enqueue_frame:
	cam_queue_enqueue(queue, &frame->list);
}

static int camcore_buffer_path_cfg(struct camera_module *module,
	uint32_t index)
{
	int ret = 0;
	uint32_t j, isp_ctx_id, isp_path_id;
	struct channel_context *ch = NULL;
	struct cam_hw_info *hw = NULL;

	if (!module) {
		pr_err("fail to get input ptr\n");
		return -EFAULT;
	}

	ch = &module->channel[index];
	hw = module->grp->hw_info;
	if (!ch->alloc_start)
		return 0;

	if (index == CAM_CH_PRE || index == CAM_CH_VID) {
		ret = wait_for_completion_interruptible(&ch->alloc_com);
		if (ret != 0) {
			pr_err("fail to config channel/path param work %d\n", ret);
			goto exit;
		}
	}

	if (atomic_read(&ch->err_status) != 0) {
		pr_err("fail to get ch %d correct status\n", ch->ch_id);
		ret = -EFAULT;
		goto exit;
	}

	/* set shared frame for dcam output */
	while (1) {
		struct camera_frame *pframe = NULL;

		pframe = cam_queue_dequeue(&ch->share_buf_queue,
			struct camera_frame, list);
		if (pframe == NULL)
			break;
		if (module->cam_uinfo.is_4in1 &&
			index == CAM_CH_CAP)
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
				module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				ch->aux_dcam_path_id,
				pframe);
		else
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
				module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				ch->dcam_path_id,
				pframe);
		if (ret) {
			pr_err("fail to config dcam output buffer\n");
			cam_queue_enqueue(&ch->share_buf_queue, &pframe->list);
			ret = -EINVAL;
			goto exit;
		}
	}
	isp_ctx_id = ch->isp_ctx_id;
	isp_path_id = ch->isp_path_id;

	for (j = 0; j < ISP_NR3_BUF_NUM; j++) {
		if (ch->nr3_bufs[j] == NULL)
			continue;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_3DNR_BUF,
				isp_ctx_id, isp_path_id,
				ch->nr3_bufs[j]);
		if (ret) {
			pr_err("fail to config isp 3DNR buffer\n");
			goto exit;
		}
	}

	if (hw->ip_dcam[module->dcam_idx]->superzoom_support) {
		if (ch->postproc_buf == NULL)
			return 0;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_POSTPROC_BUF,
				isp_ctx_id, isp_path_id,
				ch->postproc_buf);
		if (ret) {
			pr_err("fail to config isp superzoom buffer sw %d,  path id %d\n",
				isp_ctx_id, isp_path_id);
			goto exit;
		}
	}
exit:
	ch->alloc_start = 0;
	return ret;
}

static int camcore_buffer_ltm_cfg(struct camera_module *module,
	uint32_t index)
{
	int ret = 0;
	uint32_t j, isp_ctx_id, isp_path_id;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL;
	struct channel_context *ch_cap = NULL;

	if (!module) {
		pr_err("fail to get input ptr\n");
		return -EFAULT;
	}

	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	/*non-zsl capture, or video path while preview path enable, do nothing*/
	if ((!ch_pre->enable && ch_cap->enable) || (index == CAM_CH_VID && ch_pre->enable))
		return 0;

	ch = &module->channel[index];
	isp_ctx_id = ch->isp_ctx_id;
	isp_path_id = ch->isp_path_id;

	if (module->cam_uinfo.is_rgb_ltm) {
		for (j = 0; j < ISP_LTM_BUF_NUM; j++) {
			ch->ltm_bufs[LTM_RGB][j] =
				ch_pre->ltm_bufs[LTM_RGB][j];
			if (ch->ltm_bufs[LTM_RGB][j] == NULL) {
				pr_err("fail to get rgb_buf ch->ltm_bufs[%d][%d] NULL, index : %x\n",
					LTM_RGB, j, index);
				goto exit;
			}
			ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
					ISP_PATH_CFG_RGB_LTM_BUF,
					isp_ctx_id, isp_path_id,
					ch->ltm_bufs[LTM_RGB][j]);
			if (ret) {
				pr_err("fail to config isp rgb LTM buffer\n");
				goto exit;
			}
		}
	}

	if (module->cam_uinfo.is_yuv_ltm) {
		for (j = 0; j < ISP_LTM_BUF_NUM; j++) {
			ch->ltm_bufs[LTM_YUV][j] =
				ch_pre->ltm_bufs[LTM_YUV][j];
			if (ch->ltm_bufs[LTM_YUV][j] == NULL) {
				pr_err("fail to get yuv_buf ch->ltm_bufs[%d][%d] NULL, index : %x\n",
					LTM_YUV, j, index);
				goto exit;
			}
			ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
					ISP_PATH_CFG_YUV_LTM_BUF,
					isp_ctx_id, isp_path_id,
					ch->ltm_bufs[LTM_YUV][j]);
			if (ret) {
				pr_err("fail to config isp yuv LTM buffer\n");
				goto exit;
			}
		}
	}
exit:
	return ret;
}

static void camcore_buffers_alloc(void *param)
{
	int ret = 0;
	int i, count, total, iommu_enable;
	uint32_t width = 0, height = 0, size = 0, pack_bits = 0;
	uint32_t postproc_w = 0, postproc_h = 0;
	struct camera_module *module;
	struct camera_frame *pframe;
	struct channel_context *channel = NULL;
	struct channel_context *channel_vid = NULL;
	struct camera_debugger *debugger;
	struct cam_hw_info *hw = NULL;
	int path_id = 0;
	struct camera_frame *alloc_buf = NULL;
	uint32_t is_super_size = 0;
	uint32_t zsl_num = 0, zsl_skip_num = 0;

	pr_info("enter.\n");

	module = (struct camera_module *)param;
	alloc_buf = cam_queue_dequeue(&module->alloc_queue,
		struct camera_frame, list);

	if (alloc_buf) {
		channel = (struct channel_context *)alloc_buf->priv_data;
		cam_queue_empty_frame_put(alloc_buf);
	} else {
		pr_err("fail to dequeue alloc_buf\n");
		return;
	}

	hw = module->grp->hw_info;
	iommu_enable = module->iommu_enable;
	channel_vid = &module->channel[CAM_CH_VID];

	width = channel->swap_size.w;
	height = channel->swap_size.h;
	pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;

	if (channel->compress_input) {
		width = ALIGN(width, DCAM_FBC_TILE_WIDTH);
		height = ALIGN(height, DCAM_FBC_TILE_HEIGHT);
		pr_info("ch %d, FBC size (%d %d)\n", channel->ch_id, width, height);
		size = dcam_if_cal_compressed_size(width, height,
			channel->compress_4bit_bypass);
	} else if (channel->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY) {
		size = cal_sprd_raw_pitch(width, pack_bits) * height;
	} else {
		size = width * height * 3;
	}
	size = ALIGN(size, CAM_BUF_ALIGN_SIZE);

	total = 5;
	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.is_dual)
		total = 4;

	/* 4in1 non-zsl capture for single frame */
	if ((module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode)
		&& channel->ch_id == CAM_CH_CAP &&
		module->channel[CAM_CH_PRE].enable == 0 &&
		module->channel[CAM_CH_VID].enable == 0)
		total = 1;

	if (module->dump_thrd.thread_task)
		total += 3;

	/* extend buffer queue for slow motion */
	if (channel->ch_uinfo.is_high_fps)
		total = CAM_SHARED_BUF_NUM;

	if (channel->ch_id == CAM_CH_PRE &&
		module->grp->camsec_cfg.camsec_mode != SEC_UNABLE) {
		total = 4;
	}

	zsl_num = CAM_ZSL_NUM;
	zsl_skip_num = CAM_ZSL_SKIP_NUM;
	channel->zsl_skip_num = zsl_skip_num;
	if (channel->ch_id == CAM_CH_CAP) {
		channel->zsl_buffer_num = zsl_num / (zsl_skip_num + 1);
		total += channel->zsl_buffer_num;
		if (zsl_num % (zsl_skip_num + 1) != 0) {
			channel->zsl_buffer_num += 1;
			total += 1;
		}
	}
	pr_info("idx %d, ch_id %d, camsec=%d, buffer size: %u (%u x %u), num %d\n",
		module->dcam_idx, channel->ch_id, module->grp->camsec_cfg.camsec_mode,
		size, width, height, total);

	for (i = 0, count = 0; i < total; i++) {
		do {
			pframe = cam_queue_empty_frame_get();
			pframe->channel_id = channel->ch_id;
			pframe->is_compressed = channel->compress_input;
			pframe->compress_4bit_bypass =
				channel->compress_4bit_bypass;
			pframe->width = width;
			pframe->height = height;
			pframe->endian = ENDIAN_LITTLE;
			pframe->pattern = module->cam_uinfo.sensor_if.img_ptn;
			if (channel->ch_id == CAM_CH_PRE &&
				module->grp->camsec_cfg.camsec_mode != SEC_UNABLE) {
				pframe->buf.buf_sec = 1;
			} else {
				pframe->buf.buf_sec = 0;
			}

			pr_info("camca: ch_id =%d, buf_sec=%d\n",
				channel->ch_id,
				pframe->buf.buf_sec);

			ret = cam_buf_alloc(&pframe->buf, size, 0, iommu_enable);
			if (ret) {
				pr_err("fail to alloc buf: %d ch %d\n",
						i, channel->ch_id);
				cam_queue_empty_frame_put(pframe);
				atomic_inc(&channel->err_status);
				goto exit;
			}

			ret = cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
			if (ret) {
				pr_err("fail to enqueue shared buf: %d ch %d\n",
					i, channel->ch_id);
				cam_buf_free(&pframe->buf);
				cam_queue_empty_frame_put(pframe);
				break;
			} else {
				count++;
				pr_debug("frame %p,idx %d,cnt %d,phy_addr %p\n",
					pframe, i, count,
					(void *)pframe->buf.addr_vir[0]);
				break;
			}
		} while (1);
	}

	debugger = &module->grp->debugger;
	path_id = channel->dcam_path_id;
	is_super_size = (module->cam_uinfo.dcam_slice_mode == CAM_OFFLINE_SLICE_HW
		&& width >= DCAM_HW_SLICE_WIDTH_MAX) ? 1 : 0;
	if (path_id >= 0 && path_id < DCAM_IMAGE_REPLACER_PATH_MAX) {
		struct dcam_image_replacer *replacer;

		replacer = &debugger->replacer[module->dcam_idx];
		if (replacer->enabled[path_id]) {
			camcore_prepare_frame_from_file(&channel->share_buf_queue,
				replacer->filename[path_id],
				width, height, pack_bits);
			module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
				DCAM_IOCTL_CFG_REPLACER, replacer);
		}
	} else {
		module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_REPLACER, NULL);
	}

	if (hw->ip_dcam[module->dcam_idx]->superzoom_support && !is_super_size) {
		/*more than 8x zoom capture alloc buf*/

		postproc_w = channel->ch_uinfo.dst_size.w / ISP_SCALER_UP_MAX;
		postproc_h = channel->ch_uinfo.dst_size.h / ISP_SCALER_UP_MAX;
		if (channel->ch_id != CAM_CH_CAP && channel_vid->enable) {
			postproc_w = MAX(channel->ch_uinfo.dst_size.w,
				channel_vid->ch_uinfo.dst_size.w) / ISP_SCALER_UP_MAX;
			postproc_h = MAX(channel->ch_uinfo.dst_size.h,
				channel_vid->ch_uinfo.dst_size.h) / ISP_SCALER_UP_MAX;
		}

		size = ((postproc_w + 1) & (~1)) * postproc_h * 3 / 2;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		pframe = cam_queue_empty_frame_get();
		if (!pframe) {
			pr_err("fail to superzoom no empty frame.\n");
			ret = -EINVAL;
			goto exit;
		}

		pframe->channel_id = channel->ch_id;
		if (module->grp->camsec_cfg.camsec_mode != SEC_UNABLE) {
			pframe->buf.buf_sec = 1;
			pr_debug("superzoom camca: ch_id =%d, buf_sec=%d\n",
				channel->ch_id,
				pframe->buf.buf_sec);
		}

		ret = cam_buf_alloc(&pframe->buf, size, 0, 1);
		if (ret) {
			pr_err("fail to alloc superzoom buf\n");
			cam_queue_empty_frame_put(pframe);
			atomic_inc(&channel->err_status);
			goto exit;
		}

		channel->postproc_buf = pframe;
		pr_info("idx %d, superzoom w %d, h %d, buf %p\n",
			module->dcam_idx, postproc_w, postproc_h, pframe);
	}

	pr_debug("channel->ch_id = %d, channel->type_3dnr = %d, channel->uinfo_3dnr = %d\n",
		channel->ch_id, channel->type_3dnr, channel->uinfo_3dnr);
	if ((channel->type_3dnr == CAM_3DNR_HW) &&
		(!((channel->uinfo_3dnr == 0) && (channel->ch_id == CAM_CH_PRE)))) {
		/* YUV420 for 3DNR ref*/
		if (channel->compress_3dnr)
			size = isp_3dnr_cal_compressed_size(width, height);
		else {
			size = ((width + 1) & (~1)) * height * 3 / 2;
			size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		}

		pr_info("ch %d 3dnr buffer size: %u.\n", channel->ch_id, size);
		for (i = 0; i < ISP_NR3_BUF_NUM; i++) {
			pframe = cam_queue_empty_frame_get();

			if (channel->ch_id == CAM_CH_PRE &&
				module->grp->camsec_cfg.camsec_mode != SEC_UNABLE) {
				pframe->buf.buf_sec = 1;
				pr_info("camca:  ch_id =%d, buf_sec=%d\n",
					channel->ch_id,
					pframe->buf.buf_sec);
			}

			ret = cam_buf_alloc(&pframe->buf, size, 0, iommu_enable);
			if (ret) {
				pr_err("fail to alloc 3dnr buf: %d ch %d\n",
					i, channel->ch_id);
				cam_queue_empty_frame_put(pframe);
				atomic_inc(&channel->err_status);
				goto exit;
			}
			channel->nr3_bufs[i] = pframe;
		}
	}

	if (channel->mode_ltm != MODE_LTM_OFF) {
		/* todo: ltm buffer size needs to be refined.*/
		/* size = ((width + 1) & (~1)) * height * 3 / 2; */
		/*
		 * sizeof histo from 1 tile: 128 * 16 bit
		 * MAX tile num: 8 * 8
		 */
		size = 64 * 128 * 2;

		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);

		pr_info("ch %d ltm buffer size: %u.\n", channel->ch_id, size);
		if (channel->ltm_rgb) {
			for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
				if (channel->ch_id == CAM_CH_PRE) {
					pframe = cam_queue_empty_frame_get();

					if (channel->ch_id == CAM_CH_PRE
						&& module->grp->camsec_cfg.camsec_mode == SEC_TIME_PRIORITY) {
						pframe->buf.buf_sec = 1;
						pr_info("camca: ch_id =%d, buf_sec=%d\n",
							channel->ch_id,
							pframe->buf.buf_sec);
					}
					ret = cam_buf_alloc(&pframe->buf, size, 0, iommu_enable);
					if (ret) {
						pr_err("fail to alloc ltm buf: %d ch %d\n",
							i, channel->ch_id);
						cam_queue_empty_frame_put(pframe);
						atomic_inc(&channel->err_status);
						goto exit;
					}
					channel->ltm_bufs[LTM_RGB][i] = pframe;
				}
			}
		}

		if (channel->ltm_yuv) {
			for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
				if (channel->ch_id == CAM_CH_PRE) {
					pframe = cam_queue_empty_frame_get();

					if (channel->ch_id == CAM_CH_PRE
						&& module->grp->camsec_cfg.camsec_mode == SEC_TIME_PRIORITY) {
						pframe->buf.buf_sec = 1;
						pr_info("camca: ch_id =%d, buf_sec=%d\n",
							channel->ch_id,
							pframe->buf.buf_sec);
					}
					ret = cam_buf_alloc(&pframe->buf, size, 0, iommu_enable);
					if (ret) {
						pr_err("fail to alloc ltm buf: %d ch %d\n",
							i, channel->ch_id);
						cam_queue_empty_frame_put(pframe);
						atomic_inc(&channel->err_status);
						goto exit;
					}
					channel->ltm_bufs[LTM_YUV][i] = pframe;
				}
			}
		}
	}

exit:
	if (channel->ch_id != CAM_CH_PRE &&
		channel->ch_id != CAM_CH_VID) {
		ret = camcore_buffer_path_cfg(module, channel->ch_id);
		if (ret)
			pr_err("fail to cfg path buffer\n");
	}
	complete(&channel->alloc_com);
	pr_info("ch %d done. status %d\n",
		channel->ch_id, atomic_read(&channel->err_status));
}

/* frame to fifo queue for dual camera
 * return: NULL: only input, no output
 *         frame: fifo, set to path->out_buf_queue
 */
static struct camera_frame *camcore_dual_fifo_queue(struct camera_module *module,
		struct camera_frame *pframe,
		struct channel_context *channel)
{
	int ret;

	/* zsl, save frames to fifo buffer */
	ret = cam_queue_enqueue(&module->zsl_fifo_queue, &pframe->list);
	if (ret)
		return pframe;

	if (camcore_outbuf_queue_cnt_get(module->dcam_dev_handle,
		channel->dcam_path_id) < 1) {
		/* do fifo */
		pframe = cam_queue_dequeue(&module->zsl_fifo_queue,
			struct camera_frame, list);
		if (pframe)
			return pframe;
	}

	return NULL;
}

static int camcore_dual_same_frame_get(struct camera_module *module)
{
	struct camera_group *grp;
	struct camera_module *pmd[CAM_COUNT];
	struct camera_queue *q[CAM_COUNT];
	struct camera_frame *pframe[CAM_COUNT];
	int i, j;
	int ret = 0;
	int64_t t;

	grp = module->grp;
	if (!grp)
		return -EFAULT;
	/* get the two module */
	for (i = 0, j = 0; i < CAM_COUNT; i++) {
		pmd[j] = grp->module[i];
		if (!pmd[j])
			continue;
		if (pmd[j]->cam_uinfo.is_dual)
			j++;
	}
	if (j != 2) {
		pr_err("fail to get module, dual camera, but have %d module\n", j);
		return -EFAULT;
	}
	q[0] = &(pmd[0]->zsl_fifo_queue);
	q[1] = &(pmd[1]->zsl_fifo_queue);
	t = pmd[0]->capture_times;
	ret = cam_queue_same_frame_get(q[0], q[1], &pframe[0], &pframe[1], t);
	if (ret) {
		pr_err("fail to get same frame\n");
		return ret;
	}
	pmd[0]->dual_frame = pframe[0];
	pmd[1]->dual_frame = pframe[1];

	return 0;
}
/*
 * return: 0: pframe to isp
 *         1: no need more deal
 */
static struct camera_frame *camcore_dual_frame_deal(struct camera_module *module,
		struct camera_frame *pframe,
		struct channel_context *channel)
{
	int ret;
	struct camera_frame *pftmp;

	channel = &module->channel[pframe->channel_id];
	if (atomic_read(&(module->capture_frames_dcam)) == 0) {
		/* no need report to hal, do fifo */
		pframe = camcore_dual_fifo_queue(module, pframe, channel);
		if (pframe) {
			if (pframe->sync_data)
				dcam_core_dcam_if_release_sync(pframe->sync_data,
					pframe);
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);
			if (ret)
				pr_err("fail to set output buffer\n");
		}
		return NULL;
	}
	pftmp = module->dual_frame;
	if (pftmp) {
		module->dual_frame = NULL;
		/* cur frame to out_buf_queue */
		if (pframe->sync_data)
			dcam_core_dcam_if_release_sync(pframe->sync_data,	pframe);
		ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
			DCAM_PATH_CFG_OUTPUT_BUF,
			channel->dcam_path_id, pframe);
		return pftmp;
	}
	/* get the same frame */
	ret = camcore_dual_same_frame_get(module);
	if (!ret) {
		pftmp = module->dual_frame;
		if (pftmp) {
			module->dual_frame = NULL;
			/* cur frame to out_buf_queue */
			if (pframe->sync_data)
				dcam_core_dcam_if_release_sync(pframe->sync_data,
					pframe);
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);
			return pftmp;
		}
	}
	pr_warn("Sync fail, report current frame\n");

	return pframe;
}

static struct camera_frame *camcore_4in1_frame_deal(struct camera_module *module,
		struct camera_frame *pframe,
		struct channel_context *channel)
{
	int ret;
	uint32_t shutoff = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_hw_path_stop patharg;

	/* full path release sync */
	if (pframe->sync_data)
		dcam_core_dcam_if_release_sync(pframe->sync_data,	pframe);
	/* 1: aux dcam bin tx done, set frame to isp
	 * 2: lowlux capture, dcam0 full path done, set frame to isp
	 */
	if (pframe->irq_type != CAMERA_IRQ_4IN1_DONE) {
		/* offline timestamp, check time
		 * recove this time:190415
		 *
		 * if (pframe->sensor_time.tv_sec == 0 &&
		 *	pframe->sensor_time.tv_usec == 0)
		 */
		{
			struct timespec cur_ts;

			memset(&cur_ts, 0, sizeof(struct timespec));
			pframe->boot_sensor_time = ktime_get_boottime();
			ktime_get_ts(&cur_ts);
			pframe->sensor_time.tv_sec = cur_ts.tv_sec;
			pframe->sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
		}

		return pframe;
	}
	/* dcam0 full tx done, frame report to HAL or drop */
	if (atomic_read(&module->capture_frames_dcam) > 0) {
		/* 4in1 send buf to hal for remosaic */
		atomic_dec(&module->capture_frames_dcam);
		pframe->evt = IMG_TX_DONE;
		pframe->channel_id = CAM_CH_RAW;
		ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
		complete(&module->frm_com);
		pr_info("raw frame[%d] fd %d, size[%d %d], 0x%x\n",
			pframe->fid, pframe->buf.mfd[0], pframe->width,
			pframe->height, (uint32_t)pframe->buf.addr_vir[0]);

		/*stop full path & cap eb*/
		shutoff = 1;
		dev = module->dcam_dev_handle;
		patharg.path_id = channel->dcam_path_id;
		patharg.idx = dev->idx;
		dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_PATH_STOP, &patharg);
		dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_STOP_CAP_EB, &dev->idx);
		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dev, DCAM_PATH_CFG_SHUTOFF,
			channel->dcam_path_id, &shutoff);
		return NULL;
	}
	/* set buffer back to dcam0 full path, to out_buf_queue */
	channel = &module->channel[pframe->channel_id];
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
		module->dcam_dev_handle,
		DCAM_PATH_CFG_OUTPUT_BUF,
		channel->dcam_path_id, pframe);
	if (unlikely(ret))
		pr_err("fail to set output buffer, ret %d\n", ret);

	return NULL;
}

/* 4in1_raw_capture
 * full path: sensor raw(4cell), bin path: 4in1 bin sum
 * two image save in one fd(one buffer), full + bin
 */
struct camera_frame *camcore_4in1_raw_capture_deal(struct camera_module *module,
		struct camera_frame *pframe)
{
	static uint32_t flag_path;/* b0:bin tx done, b1:full tx done */

	/* full path tx done */
	if (pframe->irq_type == CAMERA_IRQ_4IN1_DONE) {
		flag_path |= BIT(1);
	} else { /* bin path tx done */
		flag_path |= BIT(0);
	}
	/* check bin, full both tx done */
	if ((flag_path & 0x2) == 0x2) {
		pframe->evt = IMG_TX_DONE;
		pframe->irq_type = CAMERA_IRQ_4IN1_DONE;
		flag_path = 0;
		return pframe;
	}
	/* not report */
	cam_buf_ionbuf_put(&pframe->buf);
	cam_queue_empty_frame_put(pframe);

	return NULL;
}

static int camcore_4in1_slave_init(struct camera_module *module,
		struct channel_context *channel)
{
	int ret = 0;
	uint32_t dcam_path_id;
	struct dcam_path_cfg_param ch_desc;

	/* todo: will update after dcam offline ctx done. */
	dcam_path_id = DCAM_PATH_BIN;
	ret = module->dcam_dev_handle->dcam_pipe_ops->get_path(module->dcam_dev_handle,
		dcam_path_id);
	if (ret < 0) {
		pr_err("fail to get dcam path %d\n", dcam_path_id);
		ret = -EFAULT;
		goto exit;
	} else {
		module->aux_dcam_dev = NULL;
		channel->aux_dcam_path_id = dcam_path_id;
		pr_info("get aux dcam path %d\n", dcam_path_id);
	}

	/* cfg dcam1 bin path */
	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.endian.y_endian = ENDIAN_LITTLE;
	ch_desc.input_size.w = channel->ch_uinfo.src_size.w;
	ch_desc.input_size.h = channel->ch_uinfo.src_size.h;
	/* dcam1 not trim, do it by isp */
	ch_desc.input_trim.size_x = channel->ch_uinfo.src_size.w;
	ch_desc.input_trim.size_y = channel->ch_uinfo.src_size.h;
	ch_desc.output_size.w = ch_desc.input_trim.size_x;
	ch_desc.output_size.h = ch_desc.input_trim.size_y;
	ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_BASE, channel->aux_dcam_path_id, &ch_desc);
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_SIZE, channel->aux_dcam_path_id, &ch_desc);
	/* 4in1 not choose 1 from 3 frames, TODO
	 * channel->frm_cnt = (uint32_t)(-3);
	 */
	pr_info("done\n");
exit:
	return ret;
}

static int camcore_4in1_slave_deinit(struct camera_module *module)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;

	pr_info("E\n");
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	ret = module->dcam_dev_handle->dcam_pipe_ops->put_path(dev, DCAM_PATH_BIN);
	pr_info("Done, ret = %d\n", ret);

	return ret;
}

/* 4in1_raw_capture
 * init second path for bin sum
 */
static int camcore_4in1_secondary_path_init(
	struct camera_module *module, struct channel_context *ch)
{
	int ret = 0;
	uint32_t second_path_id = DCAM_PATH_BIN;
	struct dcam_path_cfg_param ch_desc;

	/* now only raw capture can run to here */
	if (ch->ch_id != CAM_CH_RAW)
		return -EFAULT;

	ch->second_path_enable = 0;
	ret = module->dcam_dev_handle->dcam_pipe_ops->get_path(
			module->dcam_dev_handle, second_path_id);
	if (ret < 0) {
		pr_err("fail to get dcam path %d\n", second_path_id);
		return -EFAULT;
	}
	ch->second_path_id = second_path_id;

	/* todo: cfg param to user setting. */
	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;
	/*
	 * Configure slow motion for BIN path. HAL must set @is_high_fps
	 * and @high_fps_skip_num for both preview channel and video
	 * channel so BIN path can enable slow motion feature correctly.
	 */
	ch_desc.slowmotion_count = ch->ch_uinfo.high_fps_skip_num;
	ch_desc.endian.y_endian = ENDIAN_LITTLE;

	ch_desc.input_size.w = module->cam_uinfo.sn_size.w / 2;
	ch_desc.input_size.h = module->cam_uinfo.sn_size.h / 2;
	ch_desc.input_trim.start_x = 0;
	ch_desc.input_trim.start_y = 0;
	ch_desc.input_trim.size_x = ch_desc.input_size.w;
	ch_desc.input_trim.size_y = ch_desc.input_size.h;
	ch_desc.output_size.w = ch_desc.input_size.w;
	ch_desc.output_size.h = ch_desc.input_size.h;

	if (ch->ch_id == CAM_CH_RAW)
		ch_desc.is_raw = 1;
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_BASE, ch->second_path_id, &ch_desc);
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_SIZE, ch->second_path_id, &ch_desc);
	/* bypass bin path all sub block except 4in1 */

	ch->second_path_enable = 1;
	pr_info("done\n");

	return 0;
}

/* 4in1_raw_capture
 * deinit second path
 */
static void camcore_4in1_secondary_path_deinit(
	struct camera_module *module, struct channel_context *ch)
{
	/* now only raw capture can run to here */
	if (ch->ch_id != CAM_CH_RAW || (!ch->second_path_enable))
		return;
	module->dcam_dev_handle->dcam_pipe_ops->put_path(
		module->dcam_dev_handle, ch->second_path_id);
	ch->second_path_enable = 0;
	pr_info("done\n");
}

static struct camera_frame *camcore_bigsize_frame_deal(struct camera_module *module,
		struct camera_frame *pframe,
		struct channel_context *channel)
{
	int ret;

	/* full path release sync */
	if (pframe->sync_data)
		dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
	/* 1: aux dcam bin tx done, set frame to isp
	 * 2: lowlux capture, dcam0 full path done, set frame to isp
	 */
	if (pframe->irq_type != CAMERA_IRQ_BIGSIZE_DONE) {
		/* offline timestamp, check time
		 * recove this time:190415
		 *
		 * if (pframe->sensor_time.tv_sec == 0 &&
		 * pframe->sensor_time.tv_usec == 0)
		 */
		{
			struct timespec cur_ts;

			memset(&cur_ts, 0, sizeof(struct timespec));
			pframe->boot_sensor_time = ktime_get_boottime();
			ktime_get_ts(&cur_ts);
			pframe->sensor_time.tv_sec = cur_ts.tv_sec;
			pframe->sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
		}
		pr_info("[(PROCESSED)] frame[%d] fd %d, size[%d %d], 0x%x, channel_id %d\n",
			pframe->fid, pframe->buf.mfd[0], pframe->width,
			pframe->height, (uint32_t)pframe->buf.addr_vir[0], pframe->channel_id);
		return pframe;
	}
	/* dcam0 full tx done, frame send to dcam1 or drop */
	pr_info("raw frame[%d] fd %d, size[%d %d], 0x%x, channel_id %d, catpure_cnt = %d, time %lld\n",
			pframe->fid, pframe->buf.mfd[0], pframe->width,
			pframe->height, (uint32_t)pframe->buf.addr_vir[0], pframe->channel_id,
			atomic_read(&module->capture_frames_dcam), pframe->boot_sensor_time);

	if (module->dcam_cap_status == DCAM_CAPTURE_START_FROM_NEXT_SOF
		&& (module->capture_times < pframe->boot_sensor_time)
		&& atomic_read(&module->capture_frames_dcam) > 0) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(module->aux_dcam_dev, pframe);
		if (ret == 0)
			return NULL;
	} else if (module->dcam_cap_status != DCAM_CAPTURE_START_FROM_NEXT_SOF) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(module->aux_dcam_dev, pframe);
		if (ret == 0)
			return NULL;
	}

	/* set buffer back to dcam0 full path, to out_buf_queue */
	pr_debug("drop frame[%d]\n", pframe->fid);
	channel = &module->channel[pframe->channel_id];
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
		module->dcam_dev_handle,
		DCAM_PATH_CFG_OUTPUT_BUF,
		channel->dcam_path_id, pframe);
	if (unlikely(ret))
		pr_err("fail to set output buffer, ret %d\n", ret);

	return NULL;
}

int camcore_isp_callback(enum isp_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	uint32_t ch_id;
	struct camera_frame *pframe;
	struct camera_module *module;
	struct channel_context *channel;

	if (!param || !priv_data) {
		pr_err("fail to get valid param %p %p\n", param, priv_data);
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;

	if (unlikely(type == ISP_CB_GET_PMBUF)) {
		struct camera_frame **pm_frame;
		if (module->pmq_init == 0)
			return 0;
		pm_frame = (struct camera_frame **)param;
		*pm_frame = cam_queue_dequeue(&module->param_queue, struct camera_frame, list);
		return 0;
	}

	if (unlikely(type == ISP_CB_DEV_ERR)) {
		pr_err("fail to get isp state, camera %d\n", module->idx);
		pframe = cam_queue_empty_frame_get();
		if (pframe) {
			pframe->evt = IMG_TX_ERR;
			pframe->irq_type = CAMERA_IRQ_IMG;
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
		}
		complete(&module->frm_com);
		return 0;
	}

	pframe = (struct camera_frame *)param;
	pframe->priv_data = NULL;
	channel = &module->channel[pframe->channel_id];

	if ((pframe->fid & 0x3F) == 0)
		pr_debug("cam %d, module %p, frame %p, ch %d\n",
		module->idx, module, pframe, pframe->channel_id);

	switch (type) {
	case ISP_CB_RET_SRC_BUF:
		if (pframe->irq_property != CAM_FRAME_COMMON) {
			/* FDR frames from user */
			pr_info("fdr %d src buf %d return", pframe->irq_property, pframe->buf.mfd[0]);
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
			break;
		}

		if ((atomic_read(&module->state) != CAM_RUNNING) ||
			module->paused || (channel->dcam_path_id < 0)) {
			/* stream off or test_isp_only */
			pr_info("isp ret src frame %p\n", pframe);
			cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
		} else if (module->cap_status == CAM_CAPTURE_RAWPROC) {
			if (module->cam_uinfo.dcam_slice_mode == CAM_OFFLINE_SLICE_SW) {
				struct channel_context *ch = NULL;

				pr_debug("slice %d %p\n", module->cam_uinfo.slice_count, pframe);
				module->cam_uinfo.slice_count++;
				ch = &module->channel[CAM_CH_CAP];
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
						DCAM_PATH_CFG_OUTPUT_BUF,
						ch->dcam_path_id, pframe);
				if (module->cam_uinfo.slice_count >= module->cam_uinfo.slice_num)
					module->cam_uinfo.slice_count = 0;
				else
					ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(module->dcam_dev_handle, pframe);
				return ret;
			}
			/* for case raw capture post-proccessing
			 * just release it, no need to return
			 */
			if (pframe->buf.type == CAM_BUF_USER)
				cam_buf_ionbuf_put(&pframe->buf);
			else
				cam_buf_free(&pframe->buf);
			pr_info("raw proc return mid frame %p\n", pframe);
			cam_queue_empty_frame_put(pframe);
		} else {
			/* return offline buffer to dcam available queue. */
			pr_debug("isp reset dcam path out %d\n",
				channel->dcam_path_id);

			if (module->dump_thrd.thread_task && module->in_dump) {
				ret = cam_queue_enqueue(&module->dump_queue, &pframe->list);
				if (ret == 0) {
					complete(&module->dump_com);
					return 0;
				}
			}

			if ((module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode) &&
				channel->aux_dcam_path_id == DCAM_PATH_BIN) {
				if (pframe->buf.type == CAM_BUF_USER) {
					/* 4in1, lowlux capture, use dcam0
					 * full path output buffer, from
					 * SPRD_IMG_IO_SET_4IN1_ADDR, user space
					 */
					ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
						DCAM_PATH_CFG_OUTPUT_BUF,
						channel->dcam_path_id,
						pframe);
				} else {
					/* 4in1, dcam1 bin path output buffer
					 * alloced by kernel
					 */
					if (module->cam_uinfo.is_4in1)
						ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->aux_dcam_path_id,
							pframe);
					else
						ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->aux_dcam_dev,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->aux_dcam_path_id,
							pframe);
				}
			} else {
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
					module->dcam_dev_handle,
					DCAM_PATH_CFG_OUTPUT_BUF,
					channel->dcam_path_id, pframe);
			}
		}
		break;

	case ISP_CB_RET_DST_BUF:
		if (atomic_read(&module->state) == CAM_RUNNING) {
			if (module->cap_status == CAM_CAPTURE_RAWPROC) {
				pr_info("raw proc return dst frame %p\n", pframe);
				cam_buf_ionbuf_put(&pframe->buf);
				module->cap_status = CAM_CAPTURE_RAWPROC_DONE;
				pframe->irq_type = CAMERA_IRQ_DONE;
				pframe->irq_property = IRQ_RAW_PROC_DONE;
			} else {
				pframe->irq_type = CAMERA_IRQ_IMG;
				pframe->priv_data = module;

				/* FDR frame done use specific irq_type */
				if (pframe->irq_property != CAM_FRAME_COMMON) {
					pframe->irq_type =
					(pframe->irq_property == CAM_FRAME_FDRL) ?
						CAMERA_IRQ_FDRL : CAMERA_IRQ_FDRH;
					module->fdr_done |= (1 << pframe->irq_type);
					pr_info("fdr %d yuv buf %d done %x\n", pframe->irq_property,
						pframe->buf.mfd[0], module->fdr_done);
					if ((module->fdr_done & (1 << CAMERA_IRQ_FDRL)) &&
						(module->fdr_done & (1 << CAMERA_IRQ_FDRH)))
						complete(&module->cap_thrd.thread_com);
				}
			}
			pframe->evt = IMG_TX_DONE;
			ch_id = pframe->channel_id;

			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				cam_buf_ionbuf_put(&pframe->buf);
				cam_queue_empty_frame_put(pframe);
			} else {
				complete(&module->frm_com);
				pr_debug("ch %d get out frame: %p, evt %d mfd %x\n",
					ch_id, pframe, pframe->evt, pframe->buf.mfd[0]);
			}
		} else {
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
		}
		break;

	case ISP_CB_STATIS_DONE:
		pframe->evt = IMG_TX_DONE;
		pframe->irq_type = CAMERA_IRQ_STATIS;
		pframe->priv_data = module;
		if (atomic_read(&module->state) == CAM_RUNNING) {
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				cam_queue_empty_frame_put(pframe);
			} else {
				complete(&module->frm_com);
				pr_debug("get statis frame: %p, type %d, %d\n",
					pframe, pframe->irq_type, pframe->irq_property);
			}
		} else {
			cam_queue_empty_frame_put(pframe);
		}
		break;
	default:
		pr_err("fail to get cb cmd: %d\n", type);
		break;
	}

	return ret;
}

int camcore_dcam_callback(enum dcam_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	struct camera_frame *pframe;
	struct camera_module *module;
	struct channel_context *channel;
	struct isp_offline_param *cur;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_reg_trace trace;
	int cap_frame = 0, skip_frame = 0;
	uint32_t gtm_param_idx = DCAM_GTM_PARAM_PRE;

	if (!param || !priv_data) {
		pr_err("fail to get valid param %p %p\n", param, priv_data);
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;
	hw = module->grp->hw_info;

	if (unlikely(type == DCAM_CB_GET_PMBUF)) {
		struct camera_frame **pm_frame;
		if (module->pmq_init == 0)
			return 0;
		pm_frame = (struct camera_frame **)param;
		*pm_frame = cam_queue_dequeue(&module->param_queue, struct camera_frame, list);
		return 0;
	}

	if (unlikely(type == DCAM_CB_DEV_ERR)) {
		pr_err("fail to check cb type. camera %d\n", module->idx);
		csi_api_reg_trace();
		trace.type = ABNORMAL_REG_TRACE;
		trace.idx = module->dcam_idx;
		hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);

		module->dcam_dev_handle->dcam_pipe_ops->stop(module->dcam_dev_handle, DCAM_STOP);

		pframe = cam_queue_empty_frame_get();
		if (pframe) {
			pframe->evt = IMG_TX_ERR;
			pframe->irq_type = CAMERA_IRQ_IMG;
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
		}
		complete(&module->frm_com);
		return 0;
	}

	pframe = (struct camera_frame *)param;
	pframe->priv_data = NULL;
	channel = &module->channel[pframe->channel_id];

	pr_debug("module %p, cam%d ch %d.  cb cmd %d, frame %p\n",
		module, module->idx, pframe->channel_id, type, pframe);

	switch (type) {
	case DCAM_CB_DATA_DONE:
		if (pframe->buf.addr_k[0]) {
			uint32_t *ptr = (uint32_t *)pframe->buf.addr_k[0];

			pr_debug("dcam path %d. outdata: %08x %08x %08x %08x\n",
				channel->dcam_path_id, ptr[0], ptr[1], ptr[2], ptr[3]);
		}

		if (channel->ch_id == CAM_CH_CAP && pframe->irq_property != CAM_FRAME_COMMON) {
			/* FDR frames should always be processed by ISP */
			int32_t isp_ctx_id;

			if (pframe->irq_property == CAM_FRAME_FDRL)
				isp_ctx_id = channel->isp_fdrl_ctx;
			else
				isp_ctx_id = channel->isp_fdrh_ctx;
			pr_info("fdr %d mfd %d, ctx_id 0x%x\n", pframe->irq_property,
				pframe->buf.mfd[0], isp_ctx_id);
			ret = module->isp_dev_handle->isp_ops->proc_frame(module->isp_dev_handle, pframe,
				isp_ctx_id);
			return ret;
		}

		if (atomic_read(&module->state) != CAM_RUNNING || module->paused) {
			pr_info("stream off or paused. put frame %p\n", pframe);
			if (pframe->buf.type == CAM_BUF_KERNEL) {
				cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
			} else {
				/* 4in1 or raw buffer is allocate from user */
				cam_buf_ionbuf_put(&pframe->buf);
				cam_queue_empty_frame_put(pframe);
			}
		} else if (channel->ch_id == CAM_CH_RAW) {
			/* RAW capture or test_dcam only */
			if (module->cam_uinfo.is_4in1 == 0) {
				uint32_t capture = 0;
				if (module->cap_status == CAM_CAPTURE_START) {
					if (module->dcam_cap_status != DCAM_CAPTURE_START_FROM_NEXT_SOF) {
						capture = 1;
					} else if (pframe->boot_sensor_time > module->capture_times) {
						/* raw capture with flash */
						capture = 1;
					}
				}
				pr_info("capture %d, fid %d, start %d  type %d\n", capture,
					pframe->fid, module->cap_status, module->dcam_cap_status);
				pr_info("cap time %lld, frame time %lld\n",
					module->capture_times, pframe->boot_sensor_time);
				if (pframe->sync_data)
					dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
				if (channel->ch_uinfo.scene == DCAM_SCENE_MODE_CAPTURE
					&& capture == 0) {
					ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->dcam_path_id, pframe);
					return 0;
				}
				pframe->evt = IMG_TX_DONE;
				pframe->irq_type = CAMERA_IRQ_IMG;
				pframe->priv_data = module;
				ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
				complete(&module->frm_com);
				pr_info("get out raw frame: fd:%d\n", pframe->buf.mfd[0]);
				return 0;
			}

			pframe->evt = IMG_TX_DONE;
			if (pframe->irq_type != CAMERA_IRQ_4IN1_DONE)
				pframe->irq_type = CAMERA_IRQ_IMG;
			if (module->cam_uinfo.is_4in1) {
				pframe = camcore_4in1_raw_capture_deal(module, pframe);
				if (!pframe)
					return 0;
			}
			/* set width,heigth */
			pframe->width = channel->ch_uinfo.dst_size.w;
			pframe->height = channel->ch_uinfo.dst_size.h;
			pframe->priv_data = module;
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			complete(&module->frm_com);
			pr_info("get out raw frame: fd:%d [%d %d]\n",
				pframe->buf.mfd[0], pframe->width, pframe->height);

		} else if (channel->ch_id == CAM_CH_PRE
			|| channel->ch_id == CAM_CH_VID) {

			pr_debug("proc isp path %d, ctx %d\n", channel->isp_path_id, channel->isp_ctx_id);
			/* ISP in_queue maybe overflow.
			 * If previous frame with size updating is dicarded by ISP,
			 * we should set it in current frame for ISP input
			 * If current frame also has new updating param,
			 * here will set it as previous one in a queue for ISP,
			 * ISP can check and free it.
			 */
			if (channel->isp_updata) {
				pr_info("next %p,  prev %p\n",
					pframe->param_data, channel->isp_updata);
				if (pframe->param_data) {
					cur = (struct isp_offline_param *)pframe->param_data;
					cur->prev = channel->isp_updata;
				} else {
					pframe->param_data = channel->isp_updata;
				}
				channel->isp_updata = NULL;
				pr_info("cur %p\n", pframe->param_data);
			}
			if ((module->flash_skip_fid == pframe->fid) && (module->flash_skip_fid != 0)) {
				pr_debug("flash_skip_frame fd = %d\n", pframe->fid);
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					DCAM_PATH_CFG_OUTPUT_BUF,
					channel->dcam_path_id, pframe);
				if (pframe->param_data) {
					cur = (struct isp_offline_param *)pframe->param_data;
					cur->prev = channel->isp_updata;
					channel->isp_updata = (void *)cur;
					pframe->param_data = NULL;
					pr_debug("store:  cur %p   prev %p\n", cur, cur->prev);
				}
				return ret;
			}

			ret = module->isp_dev_handle->isp_ops->proc_frame(module->isp_dev_handle, pframe,
					channel->isp_ctx_id);
			if (ret) {
				pr_warn_ratelimited("warning: isp proc frame failed.\n");
				/* ISP in_queue maybe overflow.
				 * If current frame taking (param_data) for size updating
				 * we should hold it here and set it in next frame for ISP
				 */
				if (pframe->param_data) {
					cur = (struct isp_offline_param *)pframe->param_data;
					cur->prev = channel->isp_updata;
					channel->isp_updata = (void *)cur;
					pframe->param_data = NULL;
					pr_info("store:  cur %p   prev %p\n", cur, cur->prev);
				}
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					DCAM_PATH_CFG_OUTPUT_BUF,
					channel->dcam_path_id, pframe);
				/* release sync if ISP don't need it */
				if (pframe->sync_data)
					dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
			}
		} else if (channel->ch_id == CAM_CH_CAP) {

			if (pframe->irq_property != CAM_FRAME_COMMON) {
				/* FDR frames should always be processed by ISP */
				int32_t isp_ctx_id;

				if (pframe->irq_property == CAM_FRAME_FDRL)
					isp_ctx_id = channel->isp_fdrl_ctx;
				else
					isp_ctx_id = channel->isp_fdrh_ctx;
				pr_info("fdr %d mfd %d, ctx_id 0x%x\n", pframe->irq_property,
					pframe->buf.mfd[0], isp_ctx_id);
				ret = module->isp_dev_handle->isp_ops->proc_frame(module->isp_dev_handle, pframe,
					isp_ctx_id);
				return ret;
			}

			if ((module->cap_status != CAM_CAPTURE_START) &&
				(module->cap_status != CAM_CAPTURE_RAWPROC)) {
				/*
				 * Release sync if we don't deliver this @pframe
				 * to ISP.
				 */
				if (pframe->sync_data)
					dcam_core_dcam_if_release_sync(pframe->sync_data,
							     pframe);

				if (pframe->img_fmt == IMG_PIX_FMT_GREY) {
					pr_info("FDR capture stopped, free buf fd %d\n", pframe->buf.mfd[0]);
					cam_buf_ionbuf_put(&pframe->buf);
					cam_queue_empty_frame_put(pframe);
					return ret;
				}

				if (module->cam_uinfo.is_dual) {
					pframe = camcore_dual_fifo_queue(module,
						pframe, channel);
				} else if (channel->zsl_skip_num == CAM_ZSL_SKIP_NUM) {
					if (channel->zsl_skip_num)
						channel->zsl_skip_num --;
					else
						channel->zsl_skip_num = CAM_ZSL_SKIP_NUM;
					ret = cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
					if (channel->share_buf_queue.cnt > channel->zsl_buffer_num)
						pframe = cam_queue_dequeue(&channel->share_buf_queue,
							struct camera_frame, list);
					else
						return ret;
				} else {
					if (channel->zsl_skip_num)
						channel->zsl_skip_num --;
					else
						channel->zsl_skip_num = CAM_ZSL_SKIP_NUM;
				}

				pr_debug("share buf queue cnt %d skip_cnt %d\n",
					channel->share_buf_queue.cnt, channel->zsl_skip_num);
				if (pframe) {
					if (pframe->dcam_idx == DCAM_ID_1)
						ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
							module->aux_dcam_dev,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->aux_dcam_path_id, pframe);
					else
						ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
							module->dcam_dev_handle,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->dcam_path_id, pframe);
				}
				return ret;
			}

			/* cap scene special process */
			if (module->dcam_cap_status == DCAM_CAPTURE_START_WITH_TIMESTAMP) {

				pframe = camcore_dual_frame_deal(module,
						pframe, channel);

				if (!pframe)
					return 0;

			} else if (module->cam_uinfo.is_4in1) {
				pframe = camcore_4in1_frame_deal(module,
						pframe, channel);
				if (!pframe)
					return 0;

			} else if (module->cam_uinfo.dcam_slice_mode) {
				pframe = camcore_bigsize_frame_deal(module,
						pframe, channel);
				if (!pframe)
					return 0;
			} else if (module->dcam_cap_status == DCAM_CAPTURE_START_FROM_NEXT_SOF) {

				/* FDR catpure should wait for RAW buffer except time condition */
				if (module->capture_scene == CAPTURE_FDR) {
					if (pframe->sync_data)
						dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
					if ((pframe->boot_sensor_time < module->capture_times) ||
						(pframe->img_fmt != IMG_PIX_FMT_GREY) ||
						(atomic_read(&module->capture_frames_dcam) < 1)) {
						uint32_t cmd;
						pr_info("discard fdr frame: fd %d\n", pframe->buf.mfd[0]);
						cmd = (pframe->img_fmt == IMG_PIX_FMT_GREY) ?
							DCAM_PATH_CFG_OUTPUT_ALTER_BUF :
							DCAM_PATH_CFG_OUTPUT_BUF;
						ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
							module->dcam_dev_handle, cmd,
							channel->dcam_path_id, pframe);
						return ret;
					}

					atomic_dec(&module->capture_frames_dcam);
					pframe->evt = IMG_TX_DONE;
					pframe->irq_type = CAMERA_IRQ_IMG;
					pframe->priv_data = module;
					ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
					complete(&module->frm_com);
					pr_info("get fdr raw frame: fd %d\n", pframe->buf.mfd[0]);
					return ret;
				}

				if (pframe->boot_sensor_time < module->capture_times) {

					pr_info("cam%d cap skip frame type[%d] cap_time[%lld] sof_time[%lld]\n",
						module->idx,
						module->dcam_cap_status,
						module->capture_times,
						pframe->boot_sensor_time);
					if (pframe->sync_data)
						dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
					ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
						module->dcam_dev_handle,
						DCAM_PATH_CFG_OUTPUT_BUF,
						channel->dcam_path_id, pframe);

					return ret;
				} else {
					cap_frame = atomic_read(&module->capture_frames_dcam);
					skip_frame = atomic_read(&module->cap_skip_frames);
					if (cap_frame > 0 && skip_frame == 0) {
						module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
							DCAM_IOCTL_CFG_GTM_UPDATE, &gtm_param_idx);
						pr_info("cam%d cap type[%d] num[%d]\n", module->idx,
							module->dcam_cap_status,
							cap_frame);
					} else {
						pr_info("cam%d cap type[%d] num[%d]\n", module->idx,
							module->dcam_cap_status, cap_frame);
						atomic_dec(&module->cap_skip_frames);
						if (pframe->sync_data)
							dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
						ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
							module->dcam_dev_handle,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->dcam_path_id, pframe);

						return ret;
					}
				}
			}

			/* to isp */
			/* skip first frame for online capture (in case of non-zsl) because lsc abnormal */
			if (!module->cam_uinfo.is_4in1 && !module->cam_uinfo.dcam_slice_mode
				&& (module->cap_status != CAM_CAPTURE_RAWPROC) && (pframe->fid < 1))
				ret = 1;
			else
				ret = cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
			if (ret) {
				pr_info("capture queue overflow\n");
				if (pframe->sync_data)
					dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
						module->dcam_dev_handle,
						DCAM_PATH_CFG_OUTPUT_BUF,
						channel->dcam_path_id, pframe);
			} else {
				if ((channel->share_buf_queue.cnt > channel->zsl_buffer_num) && (module->cap_status != CAM_CAPTURE_RAWPROC)) {
					pframe = cam_queue_dequeue(&channel->share_buf_queue,
						struct camera_frame, list);
					if (pframe == NULL) {
						pr_info("fail to get frame from share buf queue\n");
						return 0;
					}
					if (pframe->sync_data)
						dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
					ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
							module->dcam_dev_handle,
							DCAM_PATH_CFG_OUTPUT_BUF,
							channel->dcam_path_id, pframe);
					}
				if (atomic_read(&module->capture_frames_dcam) > 0)
					atomic_dec(&module->capture_frames_dcam);
				pr_debug("capture_frames_dcam = %d\n", atomic_read(&module->capture_frames_dcam));
				complete(&module->cap_thrd.thread_com);
			}
		} else {
			/* should not be here */
			pr_warn("reset dcam path out %d for ch %d\n",
				channel->dcam_path_id, channel->ch_id);
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);
		}
		break;
	case DCAM_CB_STATIS_DONE:
		pframe->evt = IMG_TX_DONE;
		pframe->irq_type = CAMERA_IRQ_STATIS;
		/* temp: statis/irq share same queue with frame data. */
		/* todo: separate statis/irq and frame queue. */

		if ((pframe->irq_property != STATIS_PARAM)
			&& (module->flash_skip_fid == pframe->fid)
			&& (module->flash_skip_fid != 0)) {
			ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
						DCAM_IOCTL_CFG_STATIS_BUF_SKIP,
						pframe);

			return ret;
		}
		if (atomic_read(&module->state) == CAM_RUNNING) {
			pframe->priv_data = module;
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				cam_queue_empty_frame_put(pframe);
			} else {
				complete(&module->frm_com);
				pr_debug("get statis frame: %p, type %d, %d\n",
				pframe, pframe->irq_type, pframe->irq_property);
			}
		} else {
			cam_queue_empty_frame_put(pframe);
		}
		break;

	case DCAM_CB_IRQ_EVENT:
		if (pframe->irq_property == IRQ_DCAM_SN_EOF) {
			cam_queue_empty_frame_put(pframe);
			break;
		}
		if (pframe->irq_property == IRQ_DCAM_SOF) {
			if ((module->flash_info.led0_ctrl && module->flash_info.led0_status < FLASH_STATUS_MAX) ||
				(module->flash_info.led1_ctrl && module->flash_info.led1_status < FLASH_STATUS_MAX)) {
				module->flash_core_handle->flash_core_ops->start_flash(module->flash_core_handle);
				if (module->flash_info.flash_last_status != module->flash_info.led0_status)
					module->flash_skip_fid = pframe->fid;
				else
					pr_info("do not need skip");
				pr_info("skip_fram=%d\n", pframe->fid);
				module->flash_info.flash_last_status = module->flash_info.led0_status;
				module->flash_info.led0_ctrl = 0;
				module->flash_info.led1_ctrl = 0;
				module->flash_info.led0_status = 0;
				module->flash_info.led1_status = 0;
			}

		}
		/* temp: statis/irq share same queue with frame data. */
		/* todo: separate statis/irq and frame queue. */
		if (atomic_read(&module->state) == CAM_RUNNING) {
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				cam_queue_empty_frame_put(pframe);
			} else {
				complete(&module->frm_com);
				pr_debug("get irq frame: %p, type %d, %d\n",
				pframe, pframe->irq_type, pframe->irq_property);
			}
		} else {
			cam_queue_empty_frame_put(pframe);
		}
		break;

	case DCAM_CB_RET_SRC_BUF:

		if (pframe->irq_property != CAM_FRAME_COMMON) {
			pr_info("fdr %d src return, mfd %d\n",
				pframe->irq_property, pframe->buf.mfd[0]);
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
			break;
		}

		pr_info("dcam ret src frame %p. module %p, %d\n", pframe, module, module->idx);

		if (module->cam_uinfo.is_4in1) {
			/* 4in1 capture case: dcam offline src buffer
			 * should be re-used for dcam online output (raw)
			 */
			module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);
		} else if (module->cam_uinfo.dcam_slice_mode) {
			/* 4in1 capture case: dcam offline src buffer
			 * should be re-used for dcam online output (raw)
			 */
			module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);
		} else if ((module->cap_status == CAM_CAPTURE_RAWPROC) ||
			(atomic_read(&module->state) != CAM_RUNNING)) {
			/* for case raw capture post-proccessing
			 * and case 4in1 after stream off
			 * just release it, no need to return
			 */
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
			pr_info("cap status %d, state %d\n",
				module->cap_status,
				atomic_read(&module->state));
		} else {
			pr_err("fail to get cap status\n");
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
		}
		break;

	default:
		break;
	}

	return ret;
}

static int camcore_bigsize_aux_init(struct camera_module *module,
		struct channel_context *channel)
{
	int ret = 0;
	uint32_t dcam_idx = DCAM_ID_1;
	uint32_t dcam_path_id;
	void *dcam = NULL;
	struct camera_group *grp = module->grp;
	struct dcam_path_cfg_param ch_desc;
	struct dcam_pipe_dev *dev = NULL;

	dcam = module->aux_dcam_dev;
	if (dcam == NULL) {
		dcam = dcam_core_dcam_if_dev_get(dcam_idx, grp->hw_info);
		if (IS_ERR_OR_NULL(dcam)) {
			pr_err("fail to get dcam%d\n", dcam_idx);
			return -EFAULT;
		}
		module->aux_dcam_dev = dcam;
		module->aux_dcam_id = dcam_idx;
	}

	dev = (struct dcam_pipe_dev *)module->aux_dcam_dev;
	dev->dcam_slice_mode = module->cam_uinfo.dcam_slice_mode;
	dev->slice_num = module->cam_uinfo.slice_num;
	dev->slice_count = 0;

	ret = module->dcam_dev_handle->dcam_pipe_ops->open(module->aux_dcam_dev);
	if (ret < 0) {
		pr_err("fail to open aux dcam dev\n");
		ret = -EFAULT;
		goto exit_dev;
	}
	ret = module->dcam_dev_handle->dcam_pipe_ops->set_callback(module->aux_dcam_dev,
		camcore_dcam_callback, module);
	if (ret) {
		pr_err("fail to set aux dcam callback\n");
		ret = -EFAULT;
		goto exit_close;
	}
	/* todo: will update after dcam offline ctx done. */
	dcam_path_id = DCAM_PATH_BIN;
	ret = module->dcam_dev_handle->dcam_pipe_ops->get_path(module->aux_dcam_dev,
		dcam_path_id);
	if (ret < 0) {
		pr_err("fail to get dcam path %d\n", dcam_path_id);
		ret = -EFAULT;
		goto exit_close;
	} else {
		channel->aux_dcam_path_id = dcam_path_id;
		pr_info("get aux dcam path %d\n", dcam_path_id);
	}

	/* cfg dcam1 bin path */
	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.endian.y_endian = ENDIAN_LITTLE;
	ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;

	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->aux_dcam_dev,
		DCAM_PATH_CFG_BASE, channel->aux_dcam_path_id, &ch_desc);

	pr_info("done\n");
	return ret;

exit_close:
	module->dcam_dev_handle->dcam_pipe_ops->close(module->aux_dcam_dev);
exit_dev:
	dcam_core_dcam_if_dev_put(module->aux_dcam_dev);
	module->aux_dcam_dev = NULL;
	return ret;
}

static int camcore_bigsize_slave_deinit(struct camera_module *module)
{
	int ret = 0;
	void *dev;

	pr_info("E\n");
	dev = module->aux_dcam_dev;
	ret = module->dcam_dev_handle->dcam_pipe_ops->stop(dev, DCAM_STOP);
	ret = module->dcam_dev_handle->dcam_pipe_ops->put_path(dev, DCAM_PATH_BIN);
	ret += module->dcam_dev_handle->dcam_pipe_ops->close(dev);
	ret += dcam_core_dcam_if_dev_put(dev);
	module->aux_dcam_dev = NULL;
	pr_info("Done, ret = %d\n", ret);

	return ret;
}

static int camcore_channel_swapsize_cal(struct camera_module *module)
{
	uint32_t src_binning = 0;
	uint32_t ratio_min, shift;
	uint32_t ratio_p_w, ratio_p_h;
	uint32_t ratio_v_w, ratio_v_h;
	uint32_t ratio_p_w1, ratio_p_h1;
	uint32_t ratio_v_w1, ratio_v_h1;
	uint32_t isp_linebuf_len = g_camctrl.isp_linebuf_len;
	struct channel_context *ch_prev = NULL;
	struct channel_context *ch_vid = NULL;
	struct channel_context *ch_cap = NULL;
	struct img_size max_bypass, max_bin, max_rds, temp;
	struct img_size src_p, dst_p, dst_v, max;

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];

	if (module->channel[CAM_CH_CAP_THM].enable ||
		module->grp->camsec_cfg.camsec_mode != SEC_UNABLE)
		module->grp->hw_info->ip_dcam[module->dcam_idx]->rds_en = 0;
	if (module->grp->hw_info->ip_dcam[module->dcam_idx]->rds_en) {
		if (ch_vid->enable && (ch_prev->ch_uinfo.src_size.w <= CAM_VIDEO_LIMIT_W
			|| ch_prev->ch_uinfo.src_size.h <= CAM_VIDEO_LIMIT_H))
			module->zoom_solution = ZOOM_DEFAULT;
	}

	if (ch_cap->enable) {
		max.w = ch_cap->ch_uinfo.src_size.w;
		max.h = ch_cap->ch_uinfo.src_size.h;
		ch_cap->swap_size = max;
		pr_info("idx %d , cap swap size %d %d\n", module->idx, max.w, max.h);
	}

	if (ch_prev->enable)
		ch_prev = &module->channel[CAM_CH_PRE];
	else if (!ch_prev->enable && ch_vid->enable)
		ch_prev = &module->channel[CAM_CH_VID];
	else
		return 0;

	if (module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode) {
		ch_prev->ch_uinfo.src_size.w >>= 1;
		ch_prev->ch_uinfo.src_size.h >>= 1;
		ch_vid->ch_uinfo.src_size.w >>= 1;
		ch_vid->ch_uinfo.src_size.h >>= 1;
	}

	src_p.w = ch_prev->ch_uinfo.src_size.w;
	src_p.h = ch_prev->ch_uinfo.src_size.h;
	dst_p.w = ch_prev->ch_uinfo.dst_size.w;
	dst_p.h = ch_prev->ch_uinfo.dst_size.h;
	dst_v.w = dst_v.h = 1;
	if (ch_vid->enable) {
		dst_p.w = ch_vid->ch_uinfo.dst_size.w;
		dst_p.h = ch_vid->ch_uinfo.dst_size.h;
	}

	if ((src_p.w * 2) <= module->cam_uinfo.sn_max_size.w)
		src_binning = 1;

	/* bypass dcam scaler always */
	max_bypass = src_p;

	/* go through binning path */
	max_bin = src_p;
	shift = 0;
	if (src_binning == 1) {
		if ((max_bin.w > isp_linebuf_len) &&
			(dst_p.w <= isp_linebuf_len) &&
			(dst_v.w <= isp_linebuf_len))
			shift = 1;

		module->binning_limit = 0;
		if (module->zoom_solution == ZOOM_BINNING4)
			module->binning_limit = 1;
		else if (shift == 1)
			module->binning_limit = 1;

		pr_info("shift %d for binning, p=%d v=%d  src=%d, %d\n",
			shift, dst_p.w, dst_v.w, max_bin.w, isp_linebuf_len);
	} else {
		if ((max_bin.w >= (dst_p.w * 2)) &&
			(max_bin.w >= (dst_v.w * 2)))
			shift = 1;
		else if ((max_bin.w > isp_linebuf_len) &&
			(dst_p.w <= isp_linebuf_len) &&
			(dst_v.w <= isp_linebuf_len))
			shift = 1;

		module->binning_limit = 1;
		if (module->zoom_solution == ZOOM_BINNING4)
			module->binning_limit = 2;

		pr_info("shift %d for full, p=%d v=%d  src=%d, %d\n",
			shift, dst_p.w, dst_v.w, max_bin.w, isp_linebuf_len);
	}
	if (SEC_UNABLE != module->grp->camsec_cfg.camsec_mode)
		shift = 1;
	max_bin.w >>= shift;
	max_bin.h >>= shift;

	/* go through rds path */
	if ((dst_p.w == 0) || (dst_p.h == 0)) {
		pr_err("fail to get valid w %d h %d\n", dst_p.w, dst_p.h);
		return -EFAULT;
	}
	max = src_p;
	ratio_p_w = (1 << RATIO_SHIFT) * max.w / dst_p.w;
	ratio_p_h = (1 << RATIO_SHIFT) * max.h / dst_p.h;
	ratio_min = MIN(ratio_p_w, ratio_p_h);
	temp.w = ((max.h * dst_p.w) / dst_p.h) & (~3);
	temp.h = ((max.w * dst_p.h) / dst_p.w) & (~3);
	ratio_p_w1 = (1 << RATIO_SHIFT) * temp.w / dst_p.w;
	ratio_p_h1 = (1 << RATIO_SHIFT) * temp.h / dst_p.h;
	ratio_min = MIN(ratio_min, MIN(ratio_p_w1, ratio_p_h1));
	if (ch_vid->enable) {
		ratio_v_w = (1 << RATIO_SHIFT) * max.w / dst_v.w;
		ratio_v_h = (1 << RATIO_SHIFT) * max.h / dst_v.h;
		ratio_min = MIN(ratio_min, MIN(ratio_v_w, ratio_v_h));
		temp.w = ((max.h * dst_v.w) / dst_v.h) & (~3);
		temp.h = ((max.w * dst_v.h) / dst_v.w) & (~3);
		ratio_v_w1 = (1 << RATIO_SHIFT) * temp.w / dst_v.w;
		ratio_v_h1 = (1 << RATIO_SHIFT) * temp.h / dst_v.h;
		ratio_min = MIN(ratio_min, MIN(ratio_v_w1, ratio_v_h1));
	}
	ratio_min = MIN(ratio_min, ((module->rds_limit << RATIO_SHIFT) / 10));
	ratio_min = MAX(ratio_min, (1 << RATIO_SHIFT));
	max.w = camcore_scale_fix(max.w, ratio_min);
	max.h = camcore_scale_fix(max.h, ratio_min);
	if (max.w > DCAM_RDS_OUT_LIMIT) {
		max.w = DCAM_RDS_OUT_LIMIT;
		max.h = src_p.h * max.w / src_p.w;
	}
	max.w = ALIGN(max.w + ALIGN_OFFSET, ALIGN_OFFSET);
	max.h = ALIGN(max.h + ALIGN_OFFSET, ALIGN_OFFSET);
	max_rds = max;

	/* for adaptive solution, select max of rds/bin */
	switch (module->zoom_solution) {
	case ZOOM_DEFAULT:
		ch_prev->swap_size = max_bypass;
		break;
	case ZOOM_BINNING2:
	case ZOOM_BINNING4:
		ch_prev->swap_size = max_bin;
		break;
	case ZOOM_RDS:
		ch_prev->swap_size = max_rds;
		break;
	case ZOOM_ADAPTIVE:
		ch_prev->swap_size.w = MAX(max_bin.w, max_rds.w);
		ch_prev->swap_size.h = MAX(max_bin.h, max_rds.h);
		break;
	default:
		pr_warn("unknown zoom solution %d\n", module->zoom_solution);
		ch_prev->swap_size = max_bypass;
		break;
	}
	pr_info("prev bypass size (%d %d), bin size (%d %d)\n",
		max_bypass.w, max_bypass.h, max_bin.w, max_bin.h);
	pr_info("prev swap size (%d %d), rds size (%d %d)\n",
		ch_prev->swap_size.w, ch_prev->swap_size.h,
		max_rds.w, max_rds.h);

	return 0;
}

static int camcore_channel_size_bininig_cal(
	struct camera_module *module, uint32_t bypass_always)
{
	uint32_t shift = 0, factor = 0, align_size = 0;
	uint32_t src_binning = 0;
	struct channel_context *ch_prev;
	struct channel_context *ch_vid;
	struct channel_context *ch_cap;
	struct channel_context *ch_cap_thm;
	struct sprd_img_rect *crop_p, *crop_v, *crop_c;
	struct sprd_img_rect crop_dst;
	struct img_trim trim_pv = {0};
	struct img_trim trim_c = {0};
	struct img_trim *isp_trim;
	struct img_size src_p, dst_p, dst_v, dcam_out;

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
	ch_cap_thm = &module->channel[CAM_CH_CAP_THM];
	if (!ch_prev->enable && !ch_cap->enable && !ch_vid->enable)
		return 0;

	dcam_out.w = dcam_out.h = 0;
	dst_p.w = dst_p.h = 1;
	dst_v.w = dst_v.h = 1;
	crop_p = crop_v = crop_c = NULL;
	if (ch_prev->enable || (!ch_prev->enable && ch_vid->enable)) {
		src_p.w = ch_prev->ch_uinfo.src_size.w;
		src_p.h = ch_prev->ch_uinfo.src_size.h;
		crop_p = &ch_prev->ch_uinfo.src_crop;
		dst_p.w = ch_prev->ch_uinfo.dst_size.w;
		dst_p.h = ch_prev->ch_uinfo.dst_size.h;
		if ((src_p.w * 2) <= module->cam_uinfo.sn_max_size.w)
			src_binning = 1;
		pr_info("src crop prev %u %u %u %u\n",
			crop_p->x, crop_p->y, crop_p->w, crop_p->h);
	}
	if (ch_vid->enable) {
		crop_v = &ch_vid->ch_uinfo.src_crop;
		dst_v.w = ch_vid->ch_uinfo.dst_size.w;
		dst_v.h = ch_vid->ch_uinfo.dst_size.h;
		pr_info("src crop vid %u %u %u %u\n",
			crop_v->x, crop_v->y, crop_v->w, crop_v->h);
	}
	if (ch_prev->enable || (!ch_prev->enable && ch_vid->enable)) {
		crop_dst = *crop_p;
		camcore_largest_crop_get(&crop_dst, crop_v);
		trim_pv.start_x = crop_dst.x;
		trim_pv.start_y = crop_dst.y;
		trim_pv.size_x = crop_dst.w;
		trim_pv.size_y = crop_dst.h;
	}

	if (ch_cap->enable) {
		trim_c.start_x = ch_cap->ch_uinfo.src_crop.x;
		trim_c.start_y = ch_cap->ch_uinfo.src_crop.y;
		trim_c.size_x = ch_cap->ch_uinfo.src_crop.w;
		trim_c.size_y = ch_cap->ch_uinfo.src_crop.h;
	}
	pr_info("trim_pv: %u %u %u %u\n", trim_pv.start_x,
		trim_pv.start_y, trim_pv.size_x, trim_pv.size_y);
	pr_info("trim_c: %u %u %u %u\n", trim_c.start_x,
		trim_c.start_y, trim_c.size_x, trim_c.size_y);

	if (ch_prev->enable || (!ch_prev->enable && ch_vid->enable)) {
		shift = 0;
		if (bypass_always == 0) {
			factor = (src_binning ? 10 : 9);
			if ((trim_pv.size_x >= (dst_p.w * 4)) &&
				(trim_pv.size_x >= (dst_v.w * 4)) &&
				(trim_pv.size_y >= (dst_p.h * 4)) &&
				(trim_pv.size_y >= (dst_v.h * 4)))
				shift = 2;
			else if ((trim_pv.size_x >= (dst_p.w * 2 * factor / 10)) &&
				(trim_pv.size_x >= (dst_v.w * 2 * factor / 10)) &&
				(trim_pv.size_y >= (dst_p.h * 2 * factor / 10)) &&
				(trim_pv.size_y >= (dst_v.h * 2 * factor / 10)))
					shift = 1;
			if (((trim_pv.size_x >> shift) > ch_prev->swap_size.w) ||
				((trim_pv.size_y >> shift) > ch_prev->swap_size.h))
				shift++;
		}
		if (shift > 2) {
			pr_info("dcam binning should limit to 1/4\n");
			shift = 2;
		}
		if (shift > module->binning_limit) {
			pr_info("bin shift limit to %d\n", module->binning_limit);
			shift = module->binning_limit;
		}

		if (shift == 2) {
			/* make sure output 2 aligned and trim invalid */
			pr_debug("shift 2 trim_pv %d %d %d %d\n",
				trim_pv.start_x, trim_pv.start_y,
				trim_pv.size_x, trim_pv.size_y);
			if ((trim_pv.size_x >> 2) & 1) {
				trim_pv.size_x = (trim_pv.size_x + 4) & ~7;
				if ((trim_pv.start_x + trim_pv.size_x) > src_p.w)
					trim_pv.size_x -= 8;
				trim_pv.start_x = (src_p.w - trim_pv.size_x) >> 1;
			}
			if ((trim_pv.size_y >> 2) & 1) {
				trim_pv.size_y = (trim_pv.size_y + 4) & ~7;
				if ((trim_pv.start_y + trim_pv.size_y) > src_p.h)
					trim_pv.size_y -= 8;
				trim_pv.start_y = (src_p.h - trim_pv.size_y) >> 1;
			}
			pr_debug("shift 2 trim_pv final %d %d %d %d\n",
				trim_pv.start_x, trim_pv.start_y,
				trim_pv.size_x, trim_pv.size_y);
		}

		if (shift == 1)
			align_size = 8;
		else if (shift == 2)
			align_size = 16;
		else
			align_size = 4;

		trim_pv.size_x = ALIGN_DOWN(trim_pv.size_x, align_size);
		trim_pv.size_y = ALIGN_DOWN(trim_pv.size_y, align_size / 2);

		dcam_out.w = (trim_pv.size_x >> shift);
		dcam_out.w = ALIGN_DOWN(dcam_out.w, 2);
		dcam_out.h = (trim_pv.size_y >> shift);
		dcam_out.h = ALIGN_DOWN(dcam_out.h, 2);

		/* avoid isp fetch fbd timeout when isp src width > 1856 */
		if (dcam_out.w > 1856)
			ch_prev->compress_input = 0;

		if (ch_prev->compress_input) {
			dcam_out.h = ALIGN_DOWN(dcam_out.h, DCAM_FBC_TILE_HEIGHT);
			trim_pv.size_y = (dcam_out.h << shift);
		}

		pr_info("shift %d, dst_p %u %u, dst_v %u %u, dcam_out %u %u\n",
			shift, dst_p.w, dst_p.h, dst_v.w, dst_v.h, dcam_out.w, dcam_out.h);

		/* applied latest rect for aem */
		module->zoom_ratio = src_p.w * ZOOM_RATIO_DEFAULT / crop_p->w;
		ch_prev->trim_dcam = trim_pv;
		ch_prev->rds_ratio = ((1 << shift) << RATIO_SHIFT);
		ch_prev->dst_dcam = dcam_out;

		isp_trim = &ch_prev->trim_isp;
		isp_trim->size_x = ((ch_prev->ch_uinfo.src_crop.w >> shift) + 1) & ~1;
		isp_trim->size_y = ((ch_prev->ch_uinfo.src_crop.h >> shift) + 1) & ~1;
		isp_trim->size_x = min(isp_trim->size_x, dcam_out.w);
		isp_trim->size_y = min(isp_trim->size_y, dcam_out.h);
		isp_trim->start_x = ((dcam_out.w - isp_trim->size_x) >> 1) & ~1;
		isp_trim->start_y = ((dcam_out.h - isp_trim->size_y) >> 1) & ~1;
		pr_info("trim isp, prev %u %u %u %u\n",
			isp_trim->start_x, isp_trim->start_y,
			isp_trim->size_x, isp_trim->size_y);
	}

	if (ch_vid->enable) {
		ch_vid->dst_dcam = dcam_out;
		ch_vid->trim_dcam = trim_pv;
		isp_trim = &ch_vid->trim_isp;
		isp_trim->size_x = ((ch_vid->ch_uinfo.src_crop.w >> shift) + 1) & ~1;
		isp_trim->size_y = ((ch_vid->ch_uinfo.src_crop.h >> shift) + 1) & ~1;
		isp_trim->size_x = min(isp_trim->size_x, dcam_out.w);
		isp_trim->size_y = min(isp_trim->size_y, dcam_out.h);
		isp_trim->start_x = ((dcam_out.w - isp_trim->size_x) >> 1) & ~1;
		isp_trim->start_y = ((dcam_out.h - isp_trim->size_y) >> 1) & ~1;
		pr_info("trim isp, vid %u %u %u %u\n",
			isp_trim->start_x, isp_trim->start_y,
			isp_trim->size_x, isp_trim->size_y);
	}

	if (ch_cap->enable) {
		ch_cap->trim_dcam = trim_c;
		camcore_diff_trim_get(&ch_cap->ch_uinfo.src_crop,
			(1 << RATIO_SHIFT), &trim_c, &ch_cap->trim_isp);
		ch_cap->trim_isp.start_x &= ~1;
		ch_cap->trim_isp.start_y &= ~1;
		ch_cap->trim_isp.size_x &= ~1;
		ch_cap->trim_isp.size_y &= ~1;
		if (ch_cap->trim_isp.size_x > trim_c.size_x)
			ch_cap->trim_isp.size_x = trim_c.size_x;
		if (ch_cap->trim_isp.size_y > trim_c.size_y)
			ch_cap->trim_isp.size_y = trim_c.size_y;
		pr_info("trim isp, cap %d %d %d %d\n",
			ch_cap->trim_isp.start_x, ch_cap->trim_isp.start_y,
			ch_cap->trim_isp.size_x, ch_cap->trim_isp.size_y);
		if (ch_cap_thm->enable) {
			ch_cap_thm->trim_dcam = ch_cap->trim_dcam;
			ch_cap_thm->trim_isp = ch_cap->trim_isp;
		}
	}

	pr_info("done\n");
	return 0;
}

static int camcore_channel_size_rds_cal(struct camera_module *module)
{
	uint32_t ratio_min, is_same_fov = 0;
	uint32_t ratio_p_w, ratio_p_h;
	uint32_t ratio_v_w, ratio_v_h;
	uint32_t ratio16_w, ratio16_h;
	uint32_t align_w, align_h;
	struct channel_context *ch_prev;
	struct channel_context *ch_vid;
	struct channel_context *ch_cap;
	struct sprd_img_rect *crop_p, *crop_v, *crop_c;
	struct sprd_img_rect crop_dst;
	struct img_trim trim_pv = {0};
	struct img_trim trim_c = {0};
	struct img_size src_p, dst_p, dst_v, dcam_out;

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
	if (!ch_prev->enable && !ch_cap->enable && !ch_vid->enable)
		return 0;

	dst_p.w = dst_p.h = 1;
	dst_v.w = dst_v.h = 1;
	ratio_v_w = ratio_v_h = 1;
	crop_p = crop_v = crop_c = NULL;
	if (ch_prev->enable) {
		src_p.w = ch_prev->ch_uinfo.src_size.w;
		src_p.h = ch_prev->ch_uinfo.src_size.h;
		crop_p = &ch_prev->ch_uinfo.src_crop;
		dst_p.w = ch_prev->ch_uinfo.dst_size.w;
		dst_p.h = ch_prev->ch_uinfo.dst_size.h;
		pr_info("src crop prev %u %u %u %u\n",
			crop_p->x, crop_p->y, crop_p->w, crop_p->h);
	}

	if (ch_vid->enable) {
		crop_v = &ch_vid->ch_uinfo.src_crop;
		dst_v.w = ch_vid->ch_uinfo.dst_size.w;
		dst_v.h = ch_vid->ch_uinfo.dst_size.h;
		pr_info("src crop vid %u %u %u %u\n",
			crop_v->x, crop_v->y, crop_v->w, crop_v->h);
	}

	if (ch_cap->enable && ch_prev->enable &&
		(ch_cap->mode_ltm == MODE_LTM_PRE)) {
		crop_c = &ch_cap->ch_uinfo.src_crop;
		is_same_fov = 1;
		pr_info("src crop cap %d %d %d %d\n", crop_c->x, crop_c->y,
			crop_c->x + crop_c->w, crop_c->y + crop_c->h);
	}

	if (ch_prev->enable) {
		crop_dst = *crop_p;
		camcore_largest_crop_get(&crop_dst, crop_v);
		camcore_largest_crop_get(&crop_dst, crop_c);
		trim_pv.start_x = crop_dst.x;
		trim_pv.start_y = crop_dst.y;
		trim_pv.size_x = crop_dst.w;
		trim_pv.size_y = crop_dst.h;
	}

	if (is_same_fov)
		trim_c = trim_pv;
	else if (ch_cap->enable) {
		trim_c.start_x = ch_cap->ch_uinfo.src_crop.x & ~1;
		trim_c.start_y = ch_cap->ch_uinfo.src_crop.y & ~1;
		trim_c.size_x = (ch_cap->ch_uinfo.src_crop.w + 1) & ~1;
		trim_c.size_y = (ch_cap->ch_uinfo.src_crop.h + 1) & ~1;

		if (ch_cap->compress_input) {
			uint32_t aligned_y;

			aligned_y = ALIGN_DOWN(trim_c.start_y, 4);
			trim_c.size_y += trim_c.start_y - aligned_y;
			trim_c.start_y = aligned_y;
		}
	}

	pr_info("trim_pv: %u %u %u %u\n", trim_pv.start_x, trim_pv.start_y,
		trim_pv.size_x, trim_pv.size_y);
	pr_info("trim_c: %u %u %u %u\n", trim_c.start_x, trim_c.start_y,
		trim_c.size_x, trim_c.size_y);

	if (ch_prev->enable) {
		ratio_min = 1 << RATIO_SHIFT;
		if (module->zoom_solution >= ZOOM_RDS) {
			ratio_p_w = (1 << RATIO_SHIFT) * crop_p->w / dst_p.w;
			ratio_p_h = (1 << RATIO_SHIFT) * crop_p->h / dst_p.h;
			ratio_min = MIN(ratio_p_w, ratio_p_h);
			if (ch_vid->enable) {
				ratio_v_w = (1 << RATIO_SHIFT) * crop_v->w / dst_v.w;
				ratio_v_h = (1 << RATIO_SHIFT) * crop_v->h / dst_v.h;
				ratio_min = MIN(ratio_min, MIN(ratio_v_w, ratio_v_h));
			}
			ratio_min = MIN(ratio_min, ((module->rds_limit << RATIO_SHIFT) / 10));
			ratio_min = MAX(ratio_min, (1 << RATIO_SHIFT));
			pr_info("ratio_p %d %d, ratio_v %d %d ratio_min %d\n",
				ratio_p_w, ratio_p_h, ratio_v_w, ratio_v_h, ratio_min);
		}

		/* align bin path output size */
		align_w = align_h = DCAM_RDS_OUT_ALIGN;
		align_w = MAX(align_w, DCAM_OUTPUT_DEBUG_ALIGN);
		dcam_out.w = camcore_ratio16_divide(trim_pv.size_x, ratio_min);
		dcam_out.h = camcore_ratio16_divide(trim_pv.size_y, ratio_min);

		/* avoid isp fetch fbd timeout when isp src width > 1856 */
		if (dcam_out.w > 1856)
			ch_prev->compress_input = 0;

		if (ch_prev->compress_input)
			align_h = MAX(align_h, DCAM_FBC_TILE_HEIGHT);

		dcam_out.w = ALIGN(dcam_out.w, align_w);
		dcam_out.h = ALIGN(dcam_out.h, align_h);

		/* keep same ratio between width and height */
		ratio16_w = (uint32_t)div_u64((uint64_t)trim_pv.size_x << RATIO_SHIFT, dcam_out.w);
		ratio16_h = (uint32_t)div_u64((uint64_t)trim_pv.size_y << RATIO_SHIFT, dcam_out.h);
		ratio_min = min(ratio16_w, ratio16_h);

		/* if sensor size is too small */
		if (src_p.w < dcam_out.w || src_p.h < dcam_out.h) {
			dcam_out.w = src_p.w;
			dcam_out.h = src_p.h;
			if (ch_prev->compress_input)
				dcam_out.h = ALIGN_DOWN(dcam_out.h,
							DCAM_FBC_TILE_HEIGHT);
			ratio_min = 1 << RATIO_SHIFT;
		}

		if ((1 << RATIO_SHIFT) >= ratio_min) {
			/* enlarge @trim_pv and crop it in isp */
			uint32_t align = 2;/* TODO set to 4 for zzhdr */

			trim_pv.size_x = max(trim_pv.size_x, dcam_out.w);
			trim_pv.size_y = max(trim_pv.size_y, dcam_out.h);
			trim_pv.size_x = ALIGN(trim_pv.size_x, align >> 1);
			trim_pv.size_y = ALIGN(trim_pv.size_y, align >> 1);
			if (src_p.w >= trim_pv.size_x)
				trim_pv.start_x = (src_p.w - trim_pv.size_x) >> 1;
			else
				trim_pv.start_x = 0;
			if (src_p.h >= trim_pv.size_y)
				trim_pv.start_y = (src_p.h - trim_pv.size_y) >> 1;
			else
				trim_pv.start_y = 0;
			trim_pv.start_x = ALIGN_DOWN(trim_pv.start_x, align);
			trim_pv.start_y = ALIGN_DOWN(trim_pv.start_y, align);

			ratio_min = 1 << RATIO_SHIFT;
			pr_info("trim_pv aligned %u %u %u %u\n",
				trim_pv.start_x, trim_pv.start_y,
				trim_pv.size_x, trim_pv.size_y);
		} else {
			dcam_out.w = camcore_ratio16_divide(trim_pv.size_x, ratio_min);
			dcam_out.h = camcore_ratio16_divide(trim_pv.size_y, ratio_min);
			dcam_out.w = ALIGN(dcam_out.w, align_w);
			dcam_out.h = ALIGN(dcam_out.h, align_h);
		}

		/* check rds out limit if rds used */
		if (dcam_out.w > DCAM_RDS_OUT_LIMIT) {
			dcam_out.w = DCAM_RDS_OUT_LIMIT;
			dcam_out.h = dcam_out.w * trim_pv.size_y / trim_pv.size_x;
			dcam_out.w = ALIGN(dcam_out.w, align_w);
			dcam_out.h = ALIGN(dcam_out.h, align_h);

			/* keep same ratio between width and height */
			ratio16_w = (uint32_t)div_u64((uint64_t)trim_pv.size_x << RATIO_SHIFT, dcam_out.w);
			ratio16_h = (uint32_t)div_u64((uint64_t)trim_pv.size_y << RATIO_SHIFT, dcam_out.h);
			ratio_min = min(ratio16_w, ratio16_h);
		}

		pr_info("dst_p %u %u, dst_v %u %u, dcam_out %u %u, ratio %u\n",
			dst_p.w, dst_p.h, dst_v.w, dst_v.h,
			dcam_out.w, dcam_out.h, ratio_min);

		/* applied latest rect for aem */
		module->zoom_ratio = src_p.w * ZOOM_RATIO_DEFAULT / crop_p->w;
		ch_prev->trim_dcam = trim_pv;
		ch_prev->rds_ratio = ratio_min;/* rds_ratio is not used */
		ch_prev->dst_dcam = dcam_out;

		ch_prev->trim_isp.size_x =
			camcore_ratio16_divide(ch_prev->ch_uinfo.src_crop.w, ratio_min);
		ch_prev->trim_isp.size_y =
			camcore_ratio16_divide(ch_prev->ch_uinfo.src_crop.h, ratio_min);
		ch_prev->trim_isp.size_x = min(ch_prev->trim_isp.size_x, dcam_out.w);
		ch_prev->trim_isp.size_y = min(ch_prev->trim_isp.size_y, dcam_out.h);
		ch_prev->trim_isp.start_x =
			(dcam_out.w - ch_prev->trim_isp.size_x) >> 1;
		ch_prev->trim_isp.start_y =
			(dcam_out.h - ch_prev->trim_isp.size_y) >> 1;

		pr_info("trim isp, prev %u %u %u %u\n",
			ch_prev->trim_isp.start_x, ch_prev->trim_isp.start_y,
			ch_prev->trim_isp.size_x, ch_prev->trim_isp.size_y);

		if (ch_vid->enable) {
			ch_vid->trim_isp.size_x =
				camcore_ratio16_divide(ch_vid->ch_uinfo.src_crop.w, ratio_min);
			ch_vid->trim_isp.size_y =
				camcore_ratio16_divide(ch_vid->ch_uinfo.src_crop.h, ratio_min);
			ch_vid->trim_isp.size_x = min(ch_vid->trim_isp.size_x, dcam_out.w);
			ch_vid->trim_isp.size_y = min(ch_vid->trim_isp.size_y, dcam_out.h);
			ch_vid->trim_isp.start_x =
				(dcam_out.w - ch_vid->trim_isp.size_x) >> 1;
			ch_vid->trim_isp.start_y =
				(dcam_out.h - ch_vid->trim_isp.size_y) >> 1;

			pr_info("trim isp, vid %d %d %d %d\n",
				ch_vid->trim_isp.start_x, ch_vid->trim_isp.start_y,
				ch_vid->trim_isp.size_x, ch_vid->trim_isp.size_y);
		}
	}

	if (ch_cap->enable) {
		ch_cap->trim_dcam = trim_c;
		camcore_diff_trim_get(&ch_cap->ch_uinfo.src_crop,
			(1 << RATIO_SHIFT), &trim_c, &ch_cap->trim_isp);
		ch_cap->trim_isp.start_x &= ~1;
		ch_cap->trim_isp.start_y &= ~1;
		ch_cap->trim_isp.size_x &= ~1;
		ch_cap->trim_isp.size_y &= ~1;
		if (ch_cap->trim_isp.size_x > trim_c.size_x)
			ch_cap->trim_isp.size_x = trim_c.size_x;
		if (ch_cap->trim_isp.size_y > trim_c.size_y)
			ch_cap->trim_isp.size_y = trim_c.size_y;
		pr_info("trim isp, cap %d %d %d %d\n",
			ch_cap->trim_isp.start_x, ch_cap->trim_isp.start_y,
			ch_cap->trim_isp.size_x, ch_cap->trim_isp.size_y);
	}

	pr_info("done.\n");

	return 0;
}

static int camcore_channel_bigsize_config(
	struct camera_module *module,
	struct channel_context *channel)
{
	int ret = 0;
	int i = 0, total = 0, iommu_enable = 0;
	uint32_t width = 0, height = 0, pack_bits = 0, size = 0;
	struct camera_uchannel *ch_uinfo = NULL;
	struct dcam_path_cfg_param ch_desc;
	struct camera_frame *pframe = NULL;

	ch_uinfo = &channel->ch_uinfo;
	iommu_enable = module->iommu_enable;
	width = channel->swap_size.w;
	height = channel->swap_size.h;
	pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;

	if (channel->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY)
		size = cal_sprd_raw_pitch(width, pack_bits) * height;
	size = ALIGN(size, CAM_BUF_ALIGN_SIZE);

	/* dcam1 alloc memory */
	total = 5;

	/* non-zsl capture for single frame */
	if (channel->ch_id == CAM_CH_CAP &&
		module->channel[CAM_CH_PRE].enable == 0 &&
		module->channel[CAM_CH_VID].enable == 0)
		total = 1;

	pr_info("ch %d alloc shared buffer size: %u (w %u h %u), num %d\n",
		channel->ch_id, size, width, height, total);

	for (i = 0; i < total; i++) {
		do {
			pframe = cam_queue_empty_frame_get();
			pframe->channel_id = channel->ch_id;
			pframe->is_compressed = channel->compress_input;
			pframe->compress_4bit_bypass =
					channel->compress_4bit_bypass;
			pframe->width = width;
			pframe->height = height;
			pframe->endian = ENDIAN_LITTLE;
			pframe->pattern = module->cam_uinfo.sensor_if.img_ptn;
			pframe->buf.buf_sec = 0;
			ret = cam_buf_alloc(&pframe->buf, size, 0, iommu_enable);
			if (ret) {
				pr_err("fail to alloc buf: %d ch %d\n",
					i, channel->ch_id);
				cam_queue_empty_frame_put(pframe);
				atomic_inc(&channel->err_status);
				break;
			}
			/* cfg aux_dcam out_buf */
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
				module->aux_dcam_dev,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->aux_dcam_path_id,
				pframe);
		} while (0);
	}

	/* dcam1 cfg path size */
	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.input_size.w = ch_uinfo->src_size.w;
	ch_desc.input_size.h = ch_uinfo->src_size.h;
	ch_desc.output_size = ch_desc.input_size;
	ch_desc.input_trim.start_x = 0;
	ch_desc.input_trim.start_y = 0;
	ch_desc.input_trim.size_x = ch_desc.input_size.w;
	ch_desc.input_trim.size_y = ch_desc.input_size.h;
	ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;

	pr_info("update dcam path %d size for channel %d is_loose %d 4in1 %d\n",
		channel->dcam_path_id, channel->ch_id, ch_desc.pack_bits, ch_desc.is_4in1);

	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->aux_dcam_dev,
		DCAM_PATH_CFG_SIZE, channel->aux_dcam_path_id, &ch_desc);

	pr_info("update channel size done for CAP\n");
	return ret;
}

static int camcore_channel_size_config(
	struct camera_module *module,
	struct channel_context *channel)
{
	int ret = 0;
	int is_zoom, loop_count;
	uint32_t isp_ctx_id, isp_path_id;
	struct isp_offline_param *isp_param = NULL;
	struct channel_context *vid;
	struct camera_uchannel *ch_uinfo = NULL;
	struct cam_hw_info *hw = NULL;
	struct dcam_path_cfg_param ch_desc;
	struct isp_ctx_size_desc ctx_size;
	struct img_trim path_trim;
	struct camera_frame *alloc_buf = NULL;

	if (atomic_read(&module->state) == CAM_RUNNING) {
		is_zoom = 1;
		loop_count = 5;
	} else {
		is_zoom = 0;
		loop_count = 1;
	}

	hw = module->grp->hw_info;
	ch_uinfo = &channel->ch_uinfo;
	/* DCAM full path not updating for zoom. */
	if (is_zoom && channel->ch_id == CAM_CH_CAP)
		goto cfg_isp;

	if (!is_zoom && (channel->swap_size.w > 0)) {
		cam_queue_init(&channel->share_buf_queue,
			CAM_SHARED_BUF_NUM, camcore_k_frame_put);

		/* alloc middle buffer for channel */
		mutex_lock(&module->buf_lock[channel->ch_id]);
		channel->alloc_start = 1;
		mutex_unlock(&module->buf_lock[channel->ch_id]);

		alloc_buf = cam_queue_empty_frame_get();
		alloc_buf->priv_data = (void *)channel;
		cam_queue_enqueue(&module->alloc_queue, &alloc_buf->list);
		complete(&module->buf_thrd.thread_com);
	}

	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.input_size.w = ch_uinfo->src_size.w;
	ch_desc.input_size.h = ch_uinfo->src_size.h;
	if (channel->ch_id == CAM_CH_CAP) {
		/* no trim in dcam full path. */
		ch_desc.output_size = ch_desc.input_size;
		ch_desc.input_trim.start_x = 0;
		ch_desc.input_trim.start_y = 0;
		ch_desc.input_trim.size_x = ch_desc.input_size.w;
		ch_desc.input_trim.size_y = ch_desc.input_size.h;
	} else {
		if (channel->rds_ratio & ((1 << RATIO_SHIFT) - 1))
			ch_desc.force_rds = 1;
		else
			ch_desc.force_rds = 0;
		ch_desc.input_trim = channel->trim_dcam;
		ch_desc.output_size = channel->dst_dcam;
	}

	pr_info("update dcam path %d size for channel %d\n",
		channel->dcam_path_id, channel->ch_id);

	if (channel->ch_id == CAM_CH_PRE || channel->ch_id == CAM_CH_VID) {
		isp_param = kzalloc(sizeof(struct isp_offline_param), GFP_KERNEL);
		if (isp_param == NULL) {
			pr_err("fail to alloc memory.\n");
			return -ENOMEM;
		}
		ch_desc.priv_size_data = (void *)isp_param;
		isp_param->valid |= ISP_SRC_SIZE;
		isp_param->src_info.src_size = ch_desc.input_size;
		isp_param->src_info.src_trim = ch_desc.input_trim;
		isp_param->src_info.dst_size = ch_desc.output_size;
		isp_param->valid |= ISP_PATH0_TRIM;
		isp_param->trim_path[0] = channel->trim_isp;
		vid = &module->channel[CAM_CH_VID];
		if (vid->enable) {
			isp_param->valid |= ISP_PATH1_TRIM;
			isp_param->trim_path[1] = vid->trim_isp;
		}
		pr_debug("isp_param %p\n", isp_param);
	}

	do {
		ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
			DCAM_PATH_CFG_SIZE, channel->dcam_path_id, &ch_desc);
		if (ret) {
			/* todo: if previous updating is not applied yet.
			 * this case will happen.
			 * (zoom ratio changes in short gap)
			 * wait here and retry(how long?)
			 */
			pr_info("wait to update dcam path %d size, zoom %d, lp %d\n",
				channel->dcam_path_id, is_zoom, loop_count);
			msleep(20);
		} else {
			break;
		}
	} while (--loop_count && channel->enable);

	if (ret && ch_desc.priv_size_data) {
		kfree(ch_desc.priv_size_data);
		ch_desc.priv_size_data = NULL;
		isp_param = NULL;
	}
	if (!is_zoom && channel->ch_id == CAM_CH_PRE) {
		isp_ctx_id = channel->isp_ctx_id;
		isp_path_id = channel->isp_path_id;
		ctx_size.src.w = ch_uinfo->src_size.w;
		ctx_size.src.h = ch_uinfo->src_size.h;
		ctx_size.crop = channel->trim_dcam;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_CTX_SIZE, isp_ctx_id, 0, &ctx_size);
		if (ret != 0)
			goto exit;
		path_trim = channel->trim_isp;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_PATH_SIZE,
			isp_ctx_id, isp_path_id, &path_trim);
		if (ret != 0)
			goto exit;
		if (vid->enable) {
			path_trim = vid->trim_isp;
			ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_PATH_SIZE,
				isp_ctx_id, ISP_SPATH_VID, &path_trim);
		}
	}

	/* isp path for prev/video will update from input frame. */
	if (channel->ch_id == CAM_CH_PRE) {
		pr_info("update channel size done for preview\n");
		return ret;
	}

cfg_isp:
	isp_ctx_id = channel->isp_ctx_id;
	isp_path_id = channel->isp_path_id;

	if (channel->ch_id == CAM_CH_CAP) {
		ctx_size.src.w = ch_uinfo->src_size.w;
		ctx_size.src.h = ch_uinfo->src_size.h;
		ctx_size.crop = channel->trim_dcam;
		pr_debug("cfg isp sw %d size src w %d, h %d, crop %d %d %d %d\n",
			isp_ctx_id, ctx_size.src.w, ctx_size.src.h,
			ctx_size.crop.start_x, ctx_size.crop.start_y, ctx_size.crop.size_x, ctx_size.crop.size_y);
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_CTX_SIZE, isp_ctx_id, 0, &ctx_size);
		if (ret != 0)
			goto exit;
	}
	path_trim = channel->trim_isp;

cfg_path:
	pr_info("cfg isp ctx sw %d path %d size, path trim %d %d %d %d\n",
		isp_ctx_id, isp_path_id, path_trim.start_x, path_trim.start_y, path_trim.size_x, path_trim.size_y);
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_PATH_SIZE,
		isp_ctx_id, isp_path_id,  &path_trim);
	if (ret != 0)
		goto exit;
	if (channel->ch_id == CAM_CH_CAP && is_zoom) {
		channel = &module->channel[CAM_CH_CAP_THM];
		if (channel->enable) {
			isp_path_id = channel->isp_path_id;
			goto cfg_path;
		}
	}
	pr_info("update channel size done for CAP\n");

exit:
	if (isp_param != NULL) {
		kfree(isp_param);
		isp_param = NULL;
	}
	return ret;
}

/* set capture channel size for isp fetch and crop, scaler
 * lowlux: 1: size / 2, 0: full size
 */
static int camcore_4in1_channel_size_config(struct camera_module *module,
		uint32_t lowlux_flag)
{
	struct channel_context *ch;
	struct channel_context ch_tmp;
	struct camera_uchannel *p;

	ch  = &module->channel[CAM_CH_CAP];
	/* backup */
	memcpy(&ch_tmp, ch, sizeof(struct channel_context));
	p = &ch_tmp.ch_uinfo;
	if (lowlux_flag) {
		/* bin-sum image, size should /2 */
		p->src_size.w /= 2;
		p->src_size.h /= 2;
		if ((p->src_size.w & 0x1) || (p->src_size.h & 0x1))
			pr_warn("Some problem with sensor size in lowlux\n");
		p->src_crop.x /= 2;
		p->src_crop.y /= 2;
		p->src_crop.w /= 2;
		p->src_crop.h /= 2;
		/* check zoom, low lux not support zoom now 190306 */
		if (p->src_crop.w != p->src_size.w ||
			p->src_crop.h != p->src_size.h) {
			pr_warn("lowlux capture not support zoom now\n");
			p->src_crop.x = 0;
			p->src_crop.y = 0;
			p->src_crop.w = p->src_size.w;
			p->src_crop.h = p->src_size.h;
		}
		p->src_crop.x = ALIGN(p->src_crop.x, 2);
		p->src_crop.y = ALIGN(p->src_crop.y, 2);
		p->src_crop.w = ALIGN(p->src_crop.w, 2);
		p->src_crop.h = ALIGN(p->src_crop.h, 2);
	}
	pr_info("src[%d %d], crop[%d %d %d %d] dst[%d %d]\n",
		p->src_size.w, p->src_size.h,
		p->src_crop.x, p->src_crop.y, p->src_crop.w, p->src_crop.h,
		p->dst_size.w, p->dst_size.h);
	ch_tmp.trim_dcam.start_x = p->src_crop.x;
	ch_tmp.trim_dcam.start_y = p->src_crop.y;
	ch_tmp.trim_dcam.size_x = p->src_crop.w;
	ch_tmp.trim_dcam.size_y = p->src_crop.h;
	ch_tmp.swap_size.w = p->src_size.w;
	ch_tmp.swap_size.h = p->src_size.h;
	camcore_diff_trim_get(&ch_tmp.ch_uinfo.src_crop,
		(1 << RATIO_SHIFT), &ch_tmp.trim_dcam, &ch_tmp.trim_isp);
	pr_info("trim_isp[%d %d %d %d]\n", ch_tmp.trim_isp.start_x,
		ch_tmp.trim_isp.start_y, ch_tmp.trim_isp.size_x,
		ch_tmp.trim_isp.size_y);
	camcore_channel_size_config(module, &ch_tmp);

	return 0;
}

static int camcore_channels_size_init(struct camera_module *module)
{
	uint32_t format = module->cam_uinfo.sensor_if.img_fmt;
	/* bypass RDS if sensor output binning size for image quality */
	module->zoom_solution = g_camctrl.dcam_zoom_mode;
	module->rds_limit = g_camctrl.dcam_rds_limit;

	/* force binning as smaller as possible for security */
	if (module->grp->camsec_cfg.camsec_mode != SEC_UNABLE)
		module->zoom_solution = ZOOM_BINNING4;

	if (format == DCAM_CAP_MODE_YUV)
		module->zoom_solution = ZOOM_DEFAULT;

	camcore_channel_swapsize_cal(module);

	pr_info("zoom_solution %d, limit %d %d\n",
		module->zoom_solution,
		module->rds_limit, module->binning_limit);

	return 0;
}

static int camcore_channel_init(struct camera_module *module,
		struct channel_context *channel)
{
	int ret = 0;
	int isp_ctx_id = 0, isp_path_id = 0, dcam_path_id = 0;
	int slave_path_id = 0;
	int new_isp_ctx, new_isp_path, new_dcam_path;
	struct channel_context *channel_prev = NULL;
	struct channel_context *channel_cap = NULL;
	struct camera_uchannel *ch_uinfo;
	struct isp_path_base_desc path_desc;
	struct isp_init_param init_param;
	struct cam_hw_info *hw = NULL;
	uint32_t format = 0;

	hw = module->grp->hw_info;
	ch_uinfo = &channel->ch_uinfo;
	ch_uinfo->src_size.w = module->cam_uinfo.sn_rect.w;
	ch_uinfo->src_size.h = module->cam_uinfo.sn_rect.h;
	new_isp_ctx = 0;
	new_isp_path = 0;
	new_dcam_path = 0;
	memset(&init_param, 0, sizeof(struct isp_init_param));
	format = module->cam_uinfo.sensor_if.img_fmt;

	switch (channel->ch_id) {
	case CAM_CH_PRE:
		if (format == DCAM_CAP_MODE_YUV)
			dcam_path_id = DCAM_PATH_FULL;
		else
			dcam_path_id = DCAM_PATH_BIN;
		isp_path_id = ISP_SPATH_CP;
		new_isp_ctx = 1;
		new_isp_path = 1;
		new_dcam_path = 1;
		break;

	case CAM_CH_VID:
		/* only consider video/pre share same
		 * dcam path and isp ctx now.
		 */
		channel_prev = &module->channel[CAM_CH_PRE];
		if (channel_prev->enable) {
			channel->dcam_path_id = channel_prev->dcam_path_id;
			isp_ctx_id = channel_prev->isp_ctx_id;
		} else {
			dcam_path_id = DCAM_PATH_BIN;
			new_dcam_path = 1;
			new_isp_ctx = 1;
			pr_info("vid channel enable without preview\n");
		}
		isp_path_id = ISP_SPATH_VID;
		new_isp_path = 1;
		break;

	case CAM_CH_CAP:
		if ( module->grp->hw_info->prj_id == SHARKL5pro && ch_uinfo->src_size.w >= DCAM_HW_SLICE_WIDTH_MAX) {
			dcam_path_id = DCAM_PATH_VCH2;
			module->auto_3dnr = 0;
		} else
			dcam_path_id = DCAM_PATH_FULL;
		if (module->simulator &&
			!module->channel[CAM_CH_PRE].enable &&
			!module->channel[CAM_CH_VID].enable)
			dcam_path_id = DCAM_PATH_BIN;
		isp_path_id = ISP_SPATH_CP;
		new_isp_ctx = 1;
		new_isp_path = 1;
		new_dcam_path = 1;
		break;

	case CAM_CH_PRE_THM:
		channel_prev = &module->channel[CAM_CH_PRE];
		if (channel_prev->enable == 0) {
			pr_err("fail to get preview channel enable status\n");
			return -EINVAL;
		}
		channel->dcam_path_id = channel_prev->dcam_path_id;
		isp_ctx_id = channel_prev->isp_ctx_id;
		isp_path_id = ISP_SPATH_FD;
		new_isp_path = 1;
		break;

	case CAM_CH_CAP_THM:
		channel_cap = &module->channel[CAM_CH_CAP];
		if (channel_cap->enable == 0) {
			pr_err("fail to get capture channel enable status\n");
			return -EINVAL;
		}
		channel->dcam_path_id = channel_cap->dcam_path_id;
		isp_ctx_id = channel_cap->isp_ctx_id;
		isp_path_id = ISP_SPATH_FD;
		new_isp_path = 1;
		break;

	case CAM_CH_RAW:
		if ( module->grp->hw_info->prj_id == SHARKL5pro && ch_uinfo->src_size.w >= 8192)
			dcam_path_id = DCAM_PATH_VCH2;
		else
			dcam_path_id = DCAM_PATH_FULL;
		new_dcam_path = 1;
		break;

	default:
		pr_err("fail to get channel id %d\n", channel->ch_id);
		return -EINVAL;
	}
	if (module->paused) {
		new_isp_ctx = 0;
		new_isp_path = 0;
		isp_ctx_id = channel->isp_ctx_id;
		isp_path_id = channel->isp_path_id;
	}

	pr_info("ch %d, new: (%d %d %d)  path (%d %d %d)\n",
		channel->ch_id, new_isp_ctx, new_isp_path, new_dcam_path,
		isp_ctx_id, isp_path_id, dcam_path_id);

	if (new_dcam_path) {
		struct dcam_path_cfg_param ch_desc;

		ret = module->dcam_dev_handle->dcam_pipe_ops->get_path(
			module->dcam_dev_handle, dcam_path_id);
		if (ret < 0) {
			pr_err("fail to get dcam path %d\n", dcam_path_id);
			return -EFAULT;
		}
		channel->dcam_path_id = dcam_path_id;
		pr_debug("get dcam path : %d\n", channel->dcam_path_id);

		/* todo: cfg param to user setting. */
		memset(&ch_desc, 0, sizeof(ch_desc));
		if (dcam_path_id == 0 && module->cam_uinfo.is_4in1 == 1)
			ch_desc.pack_bits = 0;
		else
			ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
		ch_desc.is_4in1 = module->cam_uinfo.is_4in1;
		/*
		 * Configure slow motion for BIN path. HAL must set @is_high_fps
		 * and @high_fps_skip_num for both preview channel and video
		 * channel so BIN path can enable slow motion feature correctly.
		 */
		ch_desc.slowmotion_count = ch_uinfo->high_fps_skip_num;

		ch_desc.endian.y_endian = ENDIAN_LITTLE;
		ch_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		ch_desc.input_trim.start_x = module->cam_uinfo.sn_rect.x;
		ch_desc.input_trim.start_y = module->cam_uinfo.sn_rect.y;
		ch_desc.input_trim.size_x = module->cam_uinfo.sn_rect.w;
		ch_desc.input_trim.size_y = module->cam_uinfo.sn_rect.h;
		/* auto_3dnr:hw enable, channel->uinfo_3dnr == 1: hw enable */
		ch_desc.enable_3dnr = (module->auto_3dnr | channel->uinfo_3dnr);
		if (channel->ch_id == CAM_CH_RAW)
			ch_desc.is_raw = 1;
		if ((channel->ch_id == CAM_CH_CAP) && module->cam_uinfo.is_4in1)
			ch_desc.is_raw = 1;
		if ((channel->ch_id == CAM_CH_CAP) && module->cam_uinfo.dcam_slice_mode)
			ch_desc.is_raw = 1;
		ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
			DCAM_PATH_CFG_BASE, channel->dcam_path_id, &ch_desc);
	}

	if (new_isp_ctx) {
		struct isp_ctx_base_desc ctx_desc;
		uint32_t format = module->cam_uinfo.sensor_if.img_fmt;

		memset(&ctx_desc, 0, sizeof(struct isp_ctx_base_desc));
		init_param.is_high_fps = ch_uinfo->is_high_fps;
		init_param.cam_id = module->idx;
		ret = module->isp_dev_handle->isp_ops->get_context(module->isp_dev_handle, &init_param);
		if (ret < 0) {
			pr_err("fail to get isp context for cam%d ch %d\n",
				module->idx, channel->ch_id);
			goto exit;
		}
		isp_ctx_id = ret;
		module->isp_dev_handle->isp_ops->set_callback(module->isp_dev_handle,
			isp_ctx_id, camcore_isp_callback, module);

		/* todo: cfg param to user setting. */
		if (format == DCAM_CAP_MODE_YUV)
			ctx_desc.in_fmt = ch_uinfo->dst_fmt;
		else
			ctx_desc.in_fmt = ch_uinfo->sn_fmt;
		ctx_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
		ctx_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		ctx_desc.mode_ltm = MODE_LTM_OFF;
		ctx_desc.mode_3dnr = MODE_3DNR_OFF;
		ctx_desc.enable_slowmotion = ch_uinfo->is_high_fps;
		ctx_desc.slowmotion_count = ch_uinfo->high_fps_skip_num;
		ctx_desc.slw_state = CAM_SLOWMOTION_OFF;
		ctx_desc.ch_id = channel->ch_id;

		/* 20190614: have some change for auto 3dnr, maybe some code
		 * will be refined laster. below show how to use now
		 * 1: ch->type_3dnr, flag for hw 3dnr(dcam) alloc buffer
		 * 2: module->auto_3dnr: 1: alloc buffer for prev,cap,
		 *    later will enable/disable by ch->uinfo_3dnr
		 * scene1: nightshot:module->auto_3dnr==0,prev_ch->type_3dnr==1
		 *         cap_ch->type_3dnr == 0,prev hw, cap sw
		 * scene2: auto_3dnr:module->auto_3dnr==1,ch->type_3dnr==x
		 *         dynamical enable/disable(before start_capture)
		 * scene3: off: module->auto_3dnr == 0, ch->type_3dnr == 0
		 */
		ctx_desc.mode_3dnr = MODE_3DNR_OFF;
		if (module->auto_3dnr) {
			if (channel->uinfo_3dnr) {
				if (channel->ch_id == CAM_CH_CAP)
					ctx_desc.mode_3dnr = MODE_3DNR_CAP;
				else
					ctx_desc.mode_3dnr = MODE_3DNR_PRE;
			}
			channel->type_3dnr = CAM_3DNR_HW;
		} else {
			channel->type_3dnr = CAM_3DNR_OFF;
			if (channel->uinfo_3dnr) {
				channel->type_3dnr = CAM_3DNR_HW;
				if (channel->ch_id == CAM_CH_CAP)
					ctx_desc.mode_3dnr = MODE_3DNR_CAP;
				else
					ctx_desc.mode_3dnr = MODE_3DNR_PRE;
			}
		}

		if (module->cam_uinfo.is_rgb_ltm) {
			channel->ltm_rgb = 1;
			ctx_desc.ltm_rgb = 1;
			if (channel->ch_id == CAM_CH_CAP) {
				channel->mode_ltm = MODE_LTM_CAP;
				ctx_desc.mode_ltm = MODE_LTM_CAP;
			} else if (channel->ch_id == CAM_CH_PRE) {
				channel->mode_ltm = MODE_LTM_PRE;
				ctx_desc.mode_ltm = MODE_LTM_PRE;
			}
		} else {
			channel->ltm_rgb = 0;
			ctx_desc.ltm_rgb = 0;
		}

		if (module->cam_uinfo.is_yuv_ltm) {
			channel->ltm_yuv = 1;
			ctx_desc.ltm_yuv = 1;
			if (channel->ch_id == CAM_CH_CAP) {
				channel->mode_ltm = MODE_LTM_CAP;
				ctx_desc.mode_ltm = MODE_LTM_CAP;
			} else if (channel->ch_id == CAM_CH_PRE) {
				channel->mode_ltm = MODE_LTM_PRE;
				ctx_desc.mode_ltm = MODE_LTM_PRE;
			}
		} else {
			channel->ltm_yuv = 0;
			ctx_desc.ltm_yuv = 0;
		}

		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_CTX_BASE, isp_ctx_id, 0, &ctx_desc);
	}

	if (new_isp_path) {
		ret = module->isp_dev_handle->isp_ops->get_path(
			module->isp_dev_handle, isp_ctx_id, isp_path_id);
		if (ret < 0) {
			pr_err("fail to get isp path %d from context %d\n",
				isp_path_id, isp_ctx_id);
			if (new_isp_ctx)
				module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, isp_ctx_id);
			goto exit;
		}
		channel->isp_ctx_id = (int32_t)(isp_ctx_id);
		channel->isp_path_id = (int32_t)(isp_path_id);
		pr_debug("get isp path : 0x%x\n", channel->isp_path_id);

		memset(&path_desc, 0, sizeof(path_desc));
		if (channel->ch_uinfo.slave_img_en) {
			slave_path_id = ISP_SPATH_VID;
			path_desc.slave_type = ISP_PATH_MASTER;
			path_desc.slave_path_id = slave_path_id;
		}
		path_desc.out_fmt = ch_uinfo->dst_fmt;
		path_desc.endian.y_endian = ENDIAN_LITTLE;
		path_desc.endian.uv_endian = ENDIAN_LITTLE;
		path_desc.output_size.w = ch_uinfo->dst_size.w;
		path_desc.output_size.h = ch_uinfo->dst_size.h;
		path_desc.regular_mode = ch_uinfo->regular_desc.regular_mode;

		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_PATH_BASE, isp_ctx_id, isp_path_id, &path_desc);
	}

	if (new_isp_path && channel->ch_uinfo.slave_img_en) {
		ret = module->isp_dev_handle->isp_ops->get_path(module->isp_dev_handle,
			isp_ctx_id, slave_path_id);
		if (ret < 0) {
			pr_err("fail to get isp path %d from context %d\n",
				slave_path_id, isp_ctx_id);
			module->isp_dev_handle->isp_ops->put_path(
				module->isp_dev_handle, isp_ctx_id, isp_path_id);
			if (new_isp_ctx)
				module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, isp_ctx_id);
			goto exit;
		}
		channel->slave_isp_ctx_id = (int32_t)(isp_ctx_id);
		channel->slave_isp_path_id = (int32_t)(slave_path_id);
		path_desc.slave_type = ISP_PATH_SLAVE;
		path_desc.out_fmt = ch_uinfo->slave_img_fmt;
		path_desc.endian.y_endian = ENDIAN_LITTLE;
		path_desc.endian.uv_endian = ENDIAN_LITTLE;
		path_desc.output_size.w = ch_uinfo->slave_img_size.w;
		path_desc.output_size.h = ch_uinfo->slave_img_size.h;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_PATH_BASE, isp_ctx_id, slave_path_id, &path_desc);
	}

	if (module->paused)
		goto exit;

	/* 4in1 setting */
	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.is_4in1) {
		ret = camcore_4in1_slave_init(module, channel);
		if (ret < 0) {
			pr_err("fail to init dcam for 4in1, ret = %d\n", ret);
			goto exit;
		}
	}
	if (channel->ch_id == CAM_CH_RAW && module->cam_uinfo.is_4in1) {
		ret = camcore_4in1_secondary_path_init(module, channel);
		if (ret)
			pr_err("fail to init 4in1 raw capture for bin sum\n");
	}
	/* bigsize setting */
	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.dcam_slice_mode) {
		ret = camcore_bigsize_aux_init(module, channel);
		if (ret < 0) {
			pr_err("fail to init dcam for 4in1, ret = %d\n", ret);
			goto exit;
		}
	}

exit:
	pr_info("path_id:dcam = %d, aux dcam = %d, isp = 0x%x\n",
		channel->dcam_path_id, channel->aux_dcam_path_id,
		channel->isp_path_id);
	pr_debug("ch %d done. ret = %d\n", channel->ch_id, ret);
	return ret;
}

/* for offline simulator */
static int camcore_channels_set(struct camera_module *module,
		struct sprd_dcam_path_size *in)
{
	int ret = 0;
	struct img_size swap_size, size0 = { 0, 0 };
	struct sprd_dcam_path_size param;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL, *ch_vid = NULL;

	memset(&param, 0, sizeof(struct sprd_dcam_path_size));
	pr_info("cam%d simu %d\n", module->idx, module->simulator);

	ret = camcore_channels_size_init(module);
	if (module->zoom_solution == ZOOM_DEFAULT)
		camcore_channel_size_bininig_cal(module, 1);
	else if (module->zoom_solution == ZOOM_BINNING2 ||
		module->zoom_solution == ZOOM_BINNING4)
		camcore_channel_size_bininig_cal(module, 0);
	else
		camcore_channel_size_rds_cal(module);

	camcore_compression_config(module);

	ch_pre = &module->channel[CAM_CH_PRE];
	if (ch_pre->enable) {
		swap_size = ch_pre->swap_size;
		ch_pre->swap_size = size0;
		cam_queue_init(&ch_pre->share_buf_queue,
			CAM_SHARED_BUF_NUM, camcore_k_frame_put);
		camcore_channel_size_config(module, ch_pre);
		ch_pre->swap_size = swap_size;
	}

	ch_vid = &module->channel[CAM_CH_VID];
	if (ch_vid->enable && !ch_pre->enable)
		camcore_channel_size_config(module, ch_vid);

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable) {
		swap_size = ch->swap_size;
		ch->swap_size = size0;
		cam_queue_init(&ch->share_buf_queue,
			CAM_SHARED_BUF_NUM, camcore_k_frame_put);
		camcore_channel_size_config(module, ch);
		ch->swap_size = swap_size;
	}

	param.dcam_in_w = module->cam_uinfo.sn_size.w;
	param.dcam_in_h = module->cam_uinfo.sn_size.h;
	param.pre_dst_w = ch_pre->swap_size.w;
	param.pre_dst_h = ch_pre->swap_size.h;
	param.vid_dst_w = ch_vid->swap_size.w;
	param.vid_dst_h = ch_vid->swap_size.h;
	param.dcam_out_w = ch->swap_size.w;
	param.dcam_out_h = ch->swap_size.w;
	*in = param;

	return ret;
}

static int camcore_aux_dcam_init(struct camera_module *module,
		struct channel_context *channel)
{
	int ret = 0;
	uint32_t dcam_idx = DCAM_ID_0;
	uint32_t dcam_path_id, opened = 0, newdcam = 0;
	void *dcam = NULL;
	struct camera_group *grp = module->grp;
	struct dcam_path_cfg_param ch_desc;

	dcam = module->aux_dcam_dev;
	if (dcam) {
		pr_info("aux dcam%d already init\n", module->aux_dcam_id);
		return 0;
	}

	if (module->paused) {
		// init current DCAM for sharing;
		dcam = module->dcam_dev_handle;
		dcam_idx = module->dcam_idx;
		module->aux_dcam_dev = dcam;
		module->aux_dcam_id = dcam_idx;
		pr_info("use current dcam%d: %p\n", dcam_idx, dcam);
		goto get_path;
	}

	for ( ; dcam == NULL && dcam_idx <= DCAM_ID_1; dcam_idx++) {
		if (dcam_idx == module->dcam_idx)
			continue;
		dcam = dcam_core_dcam_if_dev_get(dcam_idx, grp->hw_info);
		if (IS_ERR_OR_NULL(dcam)) {
			pr_info("get dcam%d failed\n", dcam_idx);
			continue;
		}
		module->aux_dcam_dev = dcam;
		module->aux_dcam_id = dcam_idx;
		pr_info("get aux dcam %d\n", dcam_idx);
	}

	if (dcam == NULL) {
		pr_err("fail to get aux dcam\n");
		return -EFAULT;
	}
	newdcam = 1;

	ret = module->dcam_dev_handle->dcam_pipe_ops->open(dcam);
	if (ret < 0) {
		pr_err("fail to open aux dcam dev\n");
		ret = -EFAULT;
		goto exit_dev;
	}
	opened = 1;

	ret = module->dcam_dev_handle->dcam_pipe_ops->set_callback(dcam, camcore_dcam_callback, module);
	if (ret) {
		pr_err("fail to set aux dcam callback\n");
		ret = -EFAULT;
		goto exit_close;
	}

get_path:
	dcam_path_id = DCAM_PATH_BIN;
	ret = module->dcam_dev_handle->dcam_pipe_ops->get_path(dcam, dcam_path_id);
	if (ret < 0) {
		pr_err("fail to get dcam path %d\n", dcam_path_id);
		ret = -EFAULT;
		goto exit_close;
	}
	channel->aux_dcam_path_id = dcam_path_id;
	pr_info("get aux dcam path %d\n", dcam_path_id);

	/* cfg dcam_aux bin path */
	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
	ch_desc.endian.y_endian = ENDIAN_LITTLE;
	ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dcam,
		DCAM_PATH_CFG_BASE, channel->aux_dcam_path_id, &ch_desc);

	ch_desc.input_size.w = channel->ch_uinfo.src_size.w;
	ch_desc.input_size.h = channel->ch_uinfo.src_size.h;
	ch_desc.input_trim.size_x = channel->ch_uinfo.src_size.w;
	ch_desc.input_trim.size_y = channel->ch_uinfo.src_size.h;
	ch_desc.output_size.w = ch_desc.input_trim.size_x;
	ch_desc.output_size.h = ch_desc.input_trim.size_y;
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dcam,
		DCAM_PATH_CFG_SIZE, channel->aux_dcam_path_id, &ch_desc);
	pr_info("done\n");
	return ret;

exit_close:
	if (opened)
		module->dcam_dev_handle->dcam_pipe_ops->close(module->aux_dcam_dev);
exit_dev:
	if (newdcam)
		dcam_core_dcam_if_dev_put(module->aux_dcam_dev);
	module->aux_dcam_dev = NULL;
	return ret;
}

static int camcore_aux_dcam_deinit(struct camera_module *module)
{
	int ret = 0;
	int pause = DCAM_STOP;
	int32_t path_id;
	void *dcam;

	pr_info("cam%d, fdr dcam %p, main dcam %p\n", module->idx,
		module->aux_dcam_dev, module->dcam_dev_handle);

	dcam = module->aux_dcam_dev;
	if (dcam == NULL)
		return ret;

	if (dcam == module->dcam_dev_handle)
		pause = DCAM_PAUSE_OFFLINE;
	ret = module->dcam_dev_handle->dcam_pipe_ops->stop(dcam, pause);

	path_id = module->channel[CAM_CH_CAP].aux_dcam_path_id;
	if (module->channel[CAM_CH_CAP].enable && path_id >= 0) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->put_path(dcam, path_id);
		module->channel[CAM_CH_CAP].aux_dcam_path_id = -1;
	}

	if (dcam != module->dcam_dev_handle) {
		pr_info("close and put dcam %d\n", module->aux_dcam_id);
		ret += module->dcam_dev_handle->dcam_pipe_ops->close(dcam);
		ret += dcam_core_dcam_if_dev_put(dcam);
	}

	module->aux_dcam_dev = NULL;
	module->aux_dcam_id = DCAM_ID_MAX;
	pr_info("Done, ret = %d\n", ret);

	return ret;
}

static int camcore_fdr_context_init(struct camera_module *module,
		struct channel_context *ch)
{
	int ret = 0;
	int i = 0, isp_zoom;
	int isp_ctx_id = 0, isp_path_id = 0;
	int32_t *cur_ctx;
	int32_t *cur_path;
	struct camera_uchannel *ch_uinfo;
	struct isp_ctx_base_desc ctx_desc;
	struct isp_ctx_size_desc ctx_size;
	struct isp_path_base_desc path_desc;
	struct img_trim path_trim;
	struct isp_init_param init_param;

	pr_info("cam%d enter\n", module->idx);

	module->fdr_done = 0;
	ret = camcore_aux_dcam_init(module, ch);
	if (ret) {
		pr_err("fail to init aux dcam\n");
		goto init_isp;
	}

	if (ret < 0) {
		pr_err("fail to start dcam cfg, ret %d\n", ret);
		goto exit;
	}

	pr_info("cam%d, dcam %p %p, idx %d %d\n", module->idx,
		module->dcam_dev_handle, module->aux_dcam_dev,
		module->dcam_idx, module->aux_dcam_id);

	ret = module->dcam_dev_handle->dcam_pipe_ops->start(module->aux_dcam_dev, 0);
	if (ret < 0) {
		pr_err("fail to start dcam dev, ret %d\n", ret);
		goto exit;
	}

init_isp:
	isp_zoom = 1;
	ch_uinfo = &ch->ch_uinfo;
	for (i = 0; i < 2; i++) {
		cur_ctx = (i == 0) ? &ch->isp_fdrl_ctx : &ch->isp_fdrh_ctx;
		cur_path = (i == 0) ? &ch->isp_fdrl_path : &ch->isp_fdrh_path;
		if ((*cur_ctx >= 0) && (*cur_path >= 0))
			continue;

		/* get context id and config context */
		memset(&init_param, 0, sizeof(struct isp_init_param));
		init_param.cam_id = module->idx;
		ret = module->isp_dev_handle->isp_ops->get_context(module->isp_dev_handle, &init_param);
		if (ret < 0) {
			pr_err("fail to get isp context for cam%d ch %d\n",
				module->idx, ch->ch_id);
			goto exit;
		}
		isp_ctx_id = ret;
		module->isp_dev_handle->isp_ops->set_callback(module->isp_dev_handle,
			isp_ctx_id, camcore_isp_callback, module);

		memset(&ctx_desc, 0, sizeof(struct isp_ctx_base_desc));
		ctx_desc.in_fmt = ch_uinfo->sn_fmt;
		ctx_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
		ctx_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		ctx_desc.ch_id = ch->ch_id;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_CTX_BASE, isp_ctx_id, 0, &ctx_desc);

		ctx_size.src.w = ch_uinfo->src_size.w;
		ctx_size.src.h = ch_uinfo->src_size.h;

		if (isp_zoom == 0) {
			/* no zoom in ISP */
			ctx_size.crop.start_x = 0;
			ctx_size.crop.start_y = 0;
			ctx_size.crop.size_x = ctx_size.src.w;
			ctx_size.crop.size_y = ctx_size.src.h;
		} else {
			/* zoom in ISP : fetch trim */
			ctx_size.crop.start_x = ch_uinfo->src_crop.x;
			ctx_size.crop.start_y = ch_uinfo->src_crop.y;
			ctx_size.crop.size_x = ch_uinfo->src_crop.w;
			ctx_size.crop.size_y = ch_uinfo->src_crop.h;
		}
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_CTX_SIZE, isp_ctx_id, 0, &ctx_size);

		/* get path id and config path */
		isp_path_id = ISP_SPATH_CP;
		ret = module->isp_dev_handle->isp_ops->get_path(
			module->isp_dev_handle, isp_ctx_id, isp_path_id);

		memset(&path_desc, 0, sizeof(path_desc));
		path_desc.out_fmt = ch_uinfo->dst_fmt;
		path_desc.endian.y_endian = ENDIAN_LITTLE;
		path_desc.endian.uv_endian = ENDIAN_LITTLE;
		if (isp_zoom == 0) {
			path_desc.output_size.w = ch_uinfo->src_size.w;
			path_desc.output_size.h = ch_uinfo->src_size.h;
		} else {
			path_desc.output_size.w = ch_uinfo->dst_size.w;
			path_desc.output_size.h = ch_uinfo->dst_size.h;
		}
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_PATH_BASE,
			isp_ctx_id, isp_path_id, &path_desc);

		if (isp_zoom == 0) {
			/* no zoom in ISP */
			path_trim.start_x = 0;
			path_trim.start_y = 0;
			path_trim.size_x = ch_uinfo->src_size.w;
			path_trim.size_y = ch_uinfo->src_size.h;
		} else {
			/* zoom in ISP : fetch trim, scaler no trim  */
			path_trim.start_x = 0;
			path_trim.start_y = 0;
			path_trim.size_x = ch_uinfo->src_crop.w;
			path_trim.size_y = ch_uinfo->src_crop.h;
		}

		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_PATH_SIZE,
				isp_ctx_id, isp_path_id, &path_trim);

		*cur_ctx = (int32_t)isp_ctx_id;
		*cur_path = (int32_t)isp_path_id;
		pr_info("init fdrl path %x\n", *cur_ctx);
	}
	module->fdr_init = 1;
	module->fdr_done = 0;
	pr_info("fdr init %d\n", module->fdr_init);
	return 0;

exit:
	pr_err("failed %d\n", ret);
	camcore_aux_dcam_deinit(module);
	return ret;
}

static int camcore_fdr_context_deinit(struct camera_module *module, struct channel_context *ch)
{
	int ret = 0;
	int isp_ctx_id = 0, isp_path_id = 0;

	pr_info("enter\n");
	camcore_aux_dcam_deinit(module);

	if (ch->isp_fdrl_path >= 0 && ch->isp_fdrl_ctx >= 0) {
		isp_path_id = ch->isp_fdrl_path;
		isp_ctx_id = ch->isp_fdrl_ctx;
		module->isp_dev_handle->isp_ops->put_path(module->isp_dev_handle,
			isp_ctx_id, isp_path_id);
		module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, isp_ctx_id);
		if (ch->fdrl_zoom_buf) {
			cam_buf_ionbuf_put(&ch->fdrl_zoom_buf->buf);
			cam_queue_empty_frame_put(ch->fdrl_zoom_buf);
			ch->fdrl_zoom_buf = NULL;;
		}
		pr_info("put 0x%x done\n", ch->isp_fdrl_path);
	}

	if (ch->isp_fdrh_path >= 0 && ch->isp_fdrh_ctx >= 0) {
		isp_path_id = ch->isp_fdrh_path;
		isp_ctx_id = ch->isp_fdrh_ctx;
		module->isp_dev_handle->isp_ops->put_path(module->isp_dev_handle,
			isp_ctx_id, isp_path_id);
		module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, isp_ctx_id);
		if (ch->fdrh_zoom_buf) {
			cam_buf_ionbuf_put(&ch->fdrh_zoom_buf->buf);
			cam_queue_empty_frame_put(ch->fdrh_zoom_buf);
			ch->fdrh_zoom_buf = NULL;
		}
		pr_info("put 0x%x done\n", ch->isp_fdrh_path);
	}

	ch->isp_fdrl_path = -1;
	ch->isp_fdrh_path = -1;
	ch->isp_fdrh_ctx = -1;
	ch->isp_fdrl_ctx = -1;
	module->fdr_init = 0;
	module->fdr_done = 0;
	pr_info("done\n");
	return ret;
}

#define CAMERA_DUMP_PATH "/data/ylog/"
/* will create thread in user to read raw buffer*/
#define BYTE_PER_ONCE 4096
static void camcore_write_image_to_file(uint8_t *buffer,
	ssize_t size, const char *file)
{
	ssize_t result = 0, total = 0, writ = 0;
	struct file *wfp;

	wfp = filp_open(file, O_CREAT|O_RDWR, 0666);
	if (IS_ERR_OR_NULL(wfp)) {
		pr_err("fail to open file %s\n", file);
		return;
	}
	pr_debug("write image buf=%p, size=%d\n", buffer, (uint32_t)size);
	do {
		writ = (BYTE_PER_ONCE < size) ? BYTE_PER_ONCE : size;
		result = kernel_write(wfp, buffer, writ, &wfp->f_pos);
		pr_debug("write result: %d, size: %d, pos: %d\n",
		(uint32_t)result,  (uint32_t)size, (uint32_t)wfp->f_pos);

		if (result > 0) {
			size -= result;
			buffer += result;
		}
		total += result;
	} while ((result > 0) && (size > 0));
	filp_close(wfp, NULL);
	pr_debug("write image done, total=%d\n", (uint32_t)total);
}

static int camcore_one_frame_dump(struct camera_module *module,
		struct camera_frame *pframe)
{
	ssize_t size = 0;
	struct channel_context *channel;
	enum cam_ch_id ch_id;
	uint8_t file_name[256] = { '\0' };
	uint8_t tmp_str[20] = { '\0' };
	uint32_t pack_bits = 0;
	uint32_t width = 0;

	ch_id = pframe->channel_id;
	channel = &module->channel[ch_id];

	strcat(file_name, CAMERA_DUMP_PATH);
	if (ch_id == CAM_CH_PRE)
		strcat(file_name, "prevraw_");
	else
		strcat(file_name, "capraw_");

	sprintf(tmp_str, "%d.", (uint32_t)module->cur_dump_ts.tv_sec);
	strcat(file_name, tmp_str);
	sprintf(tmp_str, "%06d", (uint32_t)(module->cur_dump_ts.tv_nsec / NSEC_PER_USEC));
	strcat(file_name, tmp_str);

	if (!pframe->sw_slice_num) {
		sprintf(tmp_str, "_w%d", pframe->width);
		strcat(file_name, tmp_str);
		sprintf(tmp_str, "_h%d", pframe->height);
		strcat(file_name, tmp_str);
		width = pframe->width;
	} else {
		sprintf(tmp_str, "_no%d", pframe->sw_slice_no);
		strcat(file_name, tmp_str);
		sprintf(tmp_str, "_w%d", pframe->slice_trim.size_x);
		strcat(file_name, tmp_str);
		sprintf(tmp_str, "_h%d", pframe->slice_trim.size_y);
		strcat(file_name, tmp_str);
		width = pframe->slice_trim.size_x;
	}

	sprintf(tmp_str, "_No%d", pframe->fid);
	strcat(file_name, tmp_str);

	pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	if (pframe->is_compressed) {
		struct compressed_addr addr;

		dcam_if_cal_compressed_addr(pframe->width, pframe->height,
			pframe->buf.iova[0], &addr,
			pframe->compress_4bit_bypass);
		sprintf(tmp_str, "_tile%08lx",
			addr.addr1 - pframe->buf.iova[0]);
		strcat(file_name, tmp_str);
		sprintf(tmp_str, "_low2tile%08x",
			addr.addr2 - addr.addr1);
		strcat(file_name, tmp_str);
		size = dcam_if_cal_compressed_size(pframe->width,
			pframe->height, pframe->compress_4bit_bypass);
	} else {
		size = cal_sprd_raw_pitch(width, pack_bits) * pframe->height;
	}
	if (pack_bits == CAM_RAW_HALF14 || pack_bits == CAM_RAW_HALF10)
		strcat(file_name, ".raw");
	else
		strcat(file_name, ".mipi_raw");

	if (cam_buf_kmap(&pframe->buf)) {
		pr_err("fail to kmap dump buf\n");
		return -EFAULT;
	}
	camcore_write_image_to_file((char *)pframe->buf.addr_k[0], size, file_name);
	cam_buf_kunmap(&pframe->buf);
	pr_debug("dump for ch %d, size %d, kaddr %p, file %s\n", ch_id,
		(int)size, (void *)pframe->buf.addr_k[0], file_name);

	return 0;
}

static inline int camcore_should_dump(int mode, int path)
{
	return (mode == DUMP_PATH_BOTH)
		|| (mode == DUMP_PATH_BIN && path == DCAM_PATH_BIN)
		|| (mode == DUMP_PATH_FULL && path == DCAM_PATH_FULL);
}

static int camcore_dumpraw_proc(void *param)
{
	uint32_t idx, cnt = 0;
	struct camera_module *module;
	struct channel_context *channel;
	struct camera_frame *pframe = NULL;
	struct cam_dbg_dump *dbg = &g_dbg_dump;

	pr_info("enter. %p\n", param);
	module = (struct camera_module *)param;
	idx = module->dcam_idx;
	if (idx > DCAM_ID_1 || !module->dcam_dev_handle)
		return 0;

	mutex_lock(&dbg->dump_lock);
	dbg->dump_ongoing |= (1 << idx);
	module->dump_count = dbg->dump_count;
	init_completion(&module->dump_com);
	mutex_unlock(&dbg->dump_lock);

	pr_info("start dump count: %d\n", module->dump_count);
	while (module->dump_count) {
		module->in_dump = 1;
		ktime_get_ts(&module->cur_dump_ts);
		if (wait_for_completion_interruptible(
			&module->dump_com) == 0) {
			if ((atomic_read(&module->state) != CAM_RUNNING) ||
				(module->dump_count == 0)) {
				pr_info("dump raw proc exit, %d %u\n",
					atomic_read(&module->state),
					module->dump_count);
				break;
			}
			pframe = cam_queue_dequeue(&module->dump_queue,
				struct camera_frame, list);
			if (!pframe)
				continue;

			channel = &module->channel[pframe->channel_id];
			if (camcore_should_dump(dbg->dump_en, channel->dcam_path_id)) {
				camcore_one_frame_dump(module, pframe);
				module->dump_count--;
				cnt++;
			}

			if (module->cam_uinfo.dcam_slice_mode == CAM_OFFLINE_SLICE_SW) {
				struct channel_context *ch = NULL;

				pr_debug("slice %d %p\n", module->cam_uinfo.slice_count, pframe);
				module->cam_uinfo.slice_count++;
				ch = &module->channel[CAM_CH_CAP];
				module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
						DCAM_PATH_CFG_OUTPUT_BUF, ch->dcam_path_id, pframe);
				if (module->cam_uinfo.slice_count >= module->cam_uinfo.slice_num)
					module->cam_uinfo.slice_count = 0;
				else
					module->dcam_dev_handle->dcam_pipe_ops->proc_frame(module->dcam_dev_handle, pframe);
				continue;
			}

			/* return it to dcam output queue */
			if (module->cam_uinfo.is_4in1 &&
				channel->aux_dcam_path_id == DCAM_PATH_BIN &&
				pframe->buf.type == CAM_BUF_KERNEL)
				module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->aux_dcam_dev,
					DCAM_PATH_CFG_OUTPUT_BUF,
					channel->aux_dcam_path_id, pframe);
			else
				module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					DCAM_PATH_CFG_OUTPUT_BUF,
					channel->dcam_path_id, pframe);

		} else {
			pr_info("dump raw proc exit.");
			break;
		}
	}
	module->dump_count = 0;
	module->in_dump = 0;
	pr_info("end dump, real cnt %d\n", cnt);

	mutex_lock(&dbg->dump_lock);
	dbg->dump_count = 0;
	dbg->dump_ongoing &= ~(1 << idx);
	mutex_unlock(&dbg->dump_lock);
	return 0;
}

static void camcore_timer_callback(unsigned long data)
{
	struct camera_module *module = (struct camera_module *)data;
	struct camera_frame *frame;
	int ret = 0;

	if (!module || atomic_read(&module->state) != CAM_RUNNING) {
		pr_err("fail to get valid module %p or error state\n", module);
		return;
	}

	if (atomic_read(&module->timeout_flag) == 1) {
		pr_err("fail to get frame data, CAM%d timeout.\n", module->idx);
		frame = cam_queue_empty_frame_get();
		if (module->cap_status == CAM_CAPTURE_RAWPROC) {
			module->cap_status = CAM_CAPTURE_RAWPROC_DONE;
			frame->evt = IMG_TX_DONE;
			frame->irq_type = CAMERA_IRQ_DONE;
			frame->irq_property = IRQ_RAW_PROC_TIMEOUT;
		} else {
			frame->evt = IMG_TIMEOUT;
			frame->irq_type = CAMERA_IRQ_IMG;
			frame->irq_property = IRQ_MAX_DONE;
		}
		ret = cam_queue_enqueue(&module->frm_queue, &frame->list);
		complete(&module->frm_com);
		if (ret)
			pr_err("fail to enqueue.\n");
	}
}

static void camcore_timer_init(struct timer_list *cam_timer,
		unsigned long data)
{
	setup_timer(cam_timer, camcore_timer_callback, data);
}

static int camcore_timer_start(struct timer_list *cam_timer,
		uint32_t time_val)
{
	int ret = 0;

	pr_debug("starting timer %ld\n", jiffies);
	ret = mod_timer(cam_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret)
		pr_err("fail to start in mod_timer %d\n", ret);

	return ret;
}

static int camcore_timer_stop(struct timer_list *cam_timer)
{
	pr_debug("stop timer\n");
	del_timer_sync(cam_timer);
	return 0;
}

static int camcore_thread_loop(void *arg)
{
	int idx;
	struct camera_module *module;
	struct cam_thread_info *thrd;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_thread_info *)arg;
	module = (struct camera_module *)thrd->ctx_handle;
	idx = module->idx;
	pr_info("%s loop starts %px\n", thrd->thread_name, thrd);
	while (1) {
		if (!IS_ERR_OR_NULL(thrd) && wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(&thrd->thread_stop, 1, 0) == 1) {
				pr_info("thread %s should stop.\n", thrd->thread_name);
				break;
			}
			pr_info("thread %s trigger\n", thrd->thread_name);
			thrd->proc_func(module);
		} else {
			pr_debug("thread %s exit!", thrd->thread_name);
			break;
		}
	}
	pr_info("%s thread stop.\n", thrd->thread_name);
	complete(&thrd->thread_stop_com);

	return 0;
}

static int camcore_thread_create(struct camera_module *module,
	struct cam_thread_info *thrd, void *func)
{
	thrd->ctx_handle = module;
	thrd->proc_func = func;
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);
	init_completion(&thrd->thread_stop_com);
	thrd->thread_task = kthread_run(camcore_thread_loop,
		thrd, "%s", thrd->thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start thread %s\n", thrd->thread_name);
		thrd->thread_task = NULL;
		return -EFAULT;
	}
	return 0;
}

static void camcore_thread_stop(struct cam_thread_info *thrd)
{
	if (thrd->thread_task) {
		atomic_set(&thrd->thread_stop, 1);
		complete(&thrd->thread_com);
		wait_for_completion(&thrd->thread_stop_com);
		thrd->thread_task = NULL;
	}
}

static int camcore_raw_proc_done(struct camera_module *module)
{
	int ret = 0;
	int isp_ctx_id, isp_path_id;
	unsigned long flag = 0;
	struct camera_group *grp = module->grp;
	struct channel_context *ch;
	struct channel_context *ch_raw;
	struct dcam_pipe_dev *dev = NULL;

	pr_info("cam%d start\n", module->idx);

	module->cap_status = CAM_CAPTURE_STOP;
	module->dcam_cap_status = DCAM_CAPTURE_STOP;
	atomic_set(&module->state, CAM_STREAM_OFF);

	if (atomic_read(&module->timeout_flag) == 1)
		pr_err("fail to raw proc, timeout\n");

	ret = module->dcam_dev_handle->dcam_pipe_ops->stop(module->dcam_dev_handle, DCAM_STOP);
	camcore_timer_stop(&module->cam_timer);

	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_DEINIT_STATIS_Q, NULL);
	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_PUT_RESERV_STATSBUF, NULL);

	ch_raw = &module->channel[CAM_CH_RAW];
	if (ch_raw->enable !=0) {

		module->dcam_dev_handle->dcam_pipe_ops->put_path(module->dcam_dev_handle,
			ch_raw->dcam_path_id);

		isp_ctx_id = ch_raw->isp_ctx_id;
		isp_path_id = ch_raw->isp_path_id;

		module->isp_dev_handle->isp_ops->put_path(module->isp_dev_handle,
			isp_ctx_id, isp_path_id);
		module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, isp_ctx_id);

		ch_raw->enable = 0;
		ch_raw->dcam_path_id = -1;
		ch_raw->isp_ctx_id = -1;
		ch_raw->isp_path_id = -1;
		ch_raw->aux_dcam_path_id = -1;
	}

	ch = &module->channel[CAM_CH_CAP];
	module->dcam_dev_handle->dcam_pipe_ops->put_path(module->dcam_dev_handle,
		ch->dcam_path_id);

	isp_ctx_id = ch->isp_ctx_id;
	isp_path_id = ch->isp_path_id;
	module->isp_dev_handle->isp_ops->put_path(module->isp_dev_handle,
		isp_ctx_id, isp_path_id);
	module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, isp_ctx_id);

	ch->enable = 0;
	ch->dcam_path_id = -1;
	ch->isp_path_id = -1;
	ch->aux_dcam_path_id = -1;
	cam_queue_clear(&ch->share_buf_queue, struct camera_frame, list);
	module->cam_uinfo.dcam_slice_mode = CAM_SLICE_NONE;
	module->cam_uinfo.slice_num = 0;

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	dev->raw_fetch_num = 0;
	dev->raw_fetch_count = 0;
	atomic_set(&module->state, CAM_IDLE);
	pr_info("camera%d rawproc done.\n", module->idx);

	spin_lock_irqsave(&grp->rawproc_lock, flag);
	if (grp->rawproc_in == 0)
		pr_err("fail to raw proc, cam%d rawproc_in should be 1 here.\n", module->idx);
	grp->rawproc_in = 0;
	spin_unlock_irqrestore(&grp->rawproc_lock, flag);

	/* stop raw dump */
	if (module->dump_thrd.thread_task) {
		int i = 0, j = 0;

		if (module->in_dump) {
			module->dump_count = 0;
			cam_queue_clear(&module->dump_queue, struct camera_frame, list);
			complete(&module->dump_com);
		}
		mutex_lock(&g_dbg_dump.dump_lock);
		i = module->dcam_idx;
		if (i < 2) {
			g_dbg_dump.dump_start[i] = NULL;
		}
		mutex_unlock(&g_dbg_dump.dump_lock);
		j = 0;
		while (module->in_dump && (j++ < THREAD_STOP_TIMEOUT)) {
			pr_debug("camera%d in dump, wait...%d\n", module->idx, j);
			msleep(10);
		}

		complete(&module->dump_thrd.thread_com);
		/* default 0, hal set 1 when needed */
	}

	return ret;
}

/* build channel/path in pre-processing */
static int camcore_raw_pre_proc(
		struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	int ctx_id, dcam_path_id, isp_path_id;
	uint32_t loop = 0;
	unsigned long flag = 0;
	struct camera_group *grp = module->grp;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct img_trim path_trim;
	struct dcam_path_cfg_param ch_desc;
	struct isp_ctx_base_desc ctx_desc;
	struct isp_ctx_size_desc ctx_size;
	struct isp_path_base_desc isp_path_desc;
	struct isp_init_param init_param;
	struct cam_hw_lbuf_share camarg;

	pr_info("cam%d in. module:%p,  grp %p, %p\n",
		module->idx, module, grp, &grp->rawproc_in);

	do {
		spin_lock_irqsave(&grp->rawproc_lock, flag);
		if (grp->rawproc_in == 0) {
			grp->rawproc_in = 1;
			spin_unlock_irqrestore(&grp->rawproc_lock, flag);
			pr_info("cam%d get rawproc_in\n", module->idx);
			break;
		} else {
			spin_unlock_irqrestore(&grp->rawproc_lock, flag);
			pr_info("cam%d will wait. loop %d\n", module->idx, loop);
			loop++;
			msleep(10);
		}
	} while (loop < 2000);

	if (loop >= 1000) {
		pr_err("fail to raw proc, wait another camera raw proc\n");
		return -EFAULT;
	}
	/* not care 4in1 */
	ch = &module->channel[CAM_CH_CAP];
	ch->dcam_path_id = -1;
	ch->isp_ctx_id = -1;
	ch->isp_path_id = -1;
	ch->aux_dcam_path_id = -1;

	if ((module->grp->hw_info->prj_id == SHARKL3)
		&& proc_info->src_size.width > ISP_WIDTH_MAX
		&& proc_info->dst_size.width > ISP_WIDTH_MAX) {
		struct dcam_pipe_dev *dev = NULL;

		ch->ch_uinfo.src_size.w = proc_info->src_size.width;
		ch->ch_uinfo.src_size.h = proc_info->src_size.height;
		ch->ch_uinfo.dst_size.w = proc_info->dst_size.width;
		ch->ch_uinfo.dst_size.h = proc_info->dst_size.height;
		module->cam_uinfo.dcam_slice_mode = CAM_OFFLINE_SLICE_SW;
		module->cam_uinfo.slice_num = camcore_slice_num_info_get(&ch->ch_uinfo.src_size,
			&ch->ch_uinfo.dst_size);
		module->cam_uinfo.slice_count = 0;
		module->auto_3dnr = 0;

		dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
		dev->dcam_slice_mode = module->cam_uinfo.dcam_slice_mode;
		dev->slice_num = module->cam_uinfo.slice_num;
		dev->slice_count = 0;
		pr_debug("slice_num %d\n", module->cam_uinfo.slice_num);
	}

	/* specify dcam path */
	dcam_path_id = DCAM_PATH_BIN;
	ret = module->dcam_dev_handle->dcam_pipe_ops->get_path(
		module->dcam_dev_handle, dcam_path_id);
	if (ret < 0) {
		pr_err("fail to get dcam path %d\n", dcam_path_id);
		return -EFAULT;
	}
	ch->dcam_path_id = dcam_path_id;

	/* config dcam path  */
	memset(&ch_desc, 0, sizeof(ch_desc));
	if(ch->dcam_path_id == 0 && module->cam_uinfo.is_4in1 == 1)
		ch_desc.pack_bits = 0;
	else
		ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;
	ch_desc.raw_cap = 1;
	ch_desc.endian.y_endian = ENDIAN_LITTLE;
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_BASE, ch->dcam_path_id, &ch_desc);

	ch_desc.input_size.w = proc_info->src_size.width;
	ch_desc.input_size.h = proc_info->src_size.height;
	ch_desc.input_trim.start_x = 0;
	ch_desc.input_trim.start_y = 0;
	ch_desc.input_trim.size_x = ch_desc.input_size.w;
	ch_desc.input_trim.size_y = ch_desc.input_size.h;
	ch_desc.output_size = ch_desc.input_size;
	ch_desc.priv_size_data = NULL;
	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_SIZE, ch->dcam_path_id, &ch_desc);

	hw = grp->hw_info;
	camarg.idx = module->dcam_idx;
	camarg.width = proc_info->src_size.width;
	camarg.offline_flag = 1;
	if (hw->ip_dcam[module->dcam_idx]->lbuf_share_support)
		hw->dcam_ioctl(hw, DCAM_HW_CFG_LBUF_SHARE_SET, &camarg);

	/* specify isp context & path */
	init_param.is_high_fps = 0;/* raw capture + slow motion ?? */
	init_param.cam_id = module->idx;
	ret = module->isp_dev_handle->isp_ops->get_context(module->isp_dev_handle, &init_param);
	if (ret < 0) {
		pr_err("fail to get isp context\n");
		goto fail_ispctx;
	}
	ctx_id = ret;
	module->isp_dev_handle->isp_ops->set_callback(module->isp_dev_handle,
		ctx_id, camcore_isp_callback, module);

	/* config isp context base */
	memset(&ctx_desc, 0, sizeof(ctx_desc));
	ctx_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	ctx_desc.in_fmt = proc_info->src_format;
	ctx_desc.bayer_pattern = proc_info->src_pattern;
	ctx_desc.mode_ltm = MODE_LTM_OFF;
	ctx_desc.mode_3dnr = MODE_3DNR_OFF;
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_CTX_BASE, ctx_id, 0, &ctx_desc);

	isp_path_id = ISP_SPATH_CP;
	ret = module->isp_dev_handle->isp_ops->get_path(
		module->isp_dev_handle, ctx_id, isp_path_id);
	if (ret < 0) {
		pr_err("fail to get isp path %d from context %d\n",
			isp_path_id, ctx_id);
		goto fail_isppath;
	}
	ch->isp_ctx_id = (int32_t)(ctx_id);
	ch->isp_path_id = (int32_t)(isp_path_id);
	pr_info("get isp path : 0x%x\n", ch->isp_path_id);

	memset(&isp_path_desc, 0, sizeof(isp_path_desc));
	isp_path_desc.out_fmt = IMG_PIX_FMT_NV21;
	isp_path_desc.endian.y_endian = ENDIAN_LITTLE;
	isp_path_desc.endian.uv_endian = ENDIAN_LITTLE;
	isp_path_desc.output_size.w = proc_info->dst_size.width;
	isp_path_desc.output_size.h = proc_info->dst_size.height;
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_PATH_BASE, ctx_id, isp_path_id, &isp_path_desc);

	/* config isp input/path size */
	ctx_size.src.w = proc_info->src_size.width;
	ctx_size.src.h = proc_info->src_size.height;
	ctx_size.crop.start_x = 0;
	ctx_size.crop.start_y = 0;
	ctx_size.crop.size_x = ctx_size.src.w;
	ctx_size.crop.size_y = ctx_size.src.h;
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_CTX_SIZE, ctx_id, 0, &ctx_size);

	path_trim.start_x = 0;
	path_trim.start_y = 0;
	path_trim.size_x = proc_info->src_size.width;
	path_trim.size_y = proc_info->src_size.height;
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_PATH_SIZE, ctx_id, isp_path_id, &path_trim);

	ch->enable = 1;
	ch->ch_uinfo.dst_fmt = isp_path_desc.out_fmt;
	atomic_set(&module->state, CAM_CFG_CH);
	pr_info("done, dcam path %d, isp_path 0x%x\n",
		dcam_path_id, ch->isp_path_id);
	return 0;

fail_isppath:
	module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle, ctx_id);
fail_ispctx:
	module->dcam_dev_handle->dcam_pipe_ops->put_path(module->dcam_dev_handle, ch->dcam_path_id);
	ch->dcam_path_id = -1;
	ch->isp_ctx_id = -1;
	ch->isp_path_id = -1;
	ch->aux_dcam_path_id = -1;

	pr_err("fail to call pre raw proc\n");
	return ret;
}

static int camcore_raw_post_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pack_bits = 0;
	uint32_t size = 0;
	struct channel_context *ch = NULL;
	struct camera_frame *src_frame = NULL;
	struct camera_frame *mid_frame = NULL;
	struct camera_frame *dst_frame = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct timespec cur_ts;

	memset(&cur_ts, 0, sizeof(struct timespec));
	pr_info("start\n");

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_err("fail to get channel enable state\n");
		return -EFAULT;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->start(module->dcam_dev_handle, 0);
	if (ret < 0) {
		pr_err("fail to start dcam dev, ret %d\n", ret);
		return -EFAULT;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_INIT_STATIS_Q, NULL);

	pr_info("src %d 0x%x, mid %d, 0x%x, dst %d, 0x%x\n",
		proc_info->fd_src, proc_info->src_offset,
		proc_info->fd_dst0, proc_info->dst0_offset,
		proc_info->fd_dst1, proc_info->dst1_offset);
	src_frame = cam_queue_empty_frame_get();
	src_frame->buf.type = CAM_BUF_USER;
	src_frame->buf.mfd[0] = proc_info->fd_src;
	src_frame->buf.offset[0] = proc_info->src_offset;
	src_frame->channel_id = ch->ch_id;
	src_frame->width = proc_info->src_size.width;
	src_frame->height = proc_info->src_size.height;
	src_frame->endian = proc_info->src_y_endian;
	src_frame->pattern = proc_info->src_pattern;
	ktime_get_ts(&cur_ts);
	src_frame->sensor_time.tv_sec = cur_ts.tv_sec;
	src_frame->sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	src_frame->time = src_frame->sensor_time;
	src_frame->boot_time = ktime_get_boottime();
	src_frame->boot_sensor_time = src_frame->boot_time;
	ret = cam_buf_ionbuf_get(&src_frame->buf);
	if (ret)
		goto src_fail;

	dst_frame = cam_queue_empty_frame_get();
	dst_frame->buf.type = CAM_BUF_USER;
	dst_frame->buf.mfd[0] = proc_info->fd_dst1;
	dst_frame->buf.offset[0] = proc_info->dst1_offset;
	dst_frame->channel_id = ch->ch_id;
	dst_frame->img_fmt = ch->ch_uinfo.dst_fmt;
	dst_frame->sensor_time = src_frame->sensor_time;
	dst_frame->time = src_frame->time;
	dst_frame->boot_time = src_frame->boot_time;
	dst_frame->boot_sensor_time = src_frame->boot_sensor_time;
	ret = cam_buf_ionbuf_get(&dst_frame->buf);
	if (ret)
		goto dst_fail;

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	if (module->grp->hw_info->prj_id == SHARKL3
		&& module->dcam_idx == DCAM_ID_1)
		dev->raw_fetch_num = 2;
	else
		dev->raw_fetch_num = 1;
	dev->raw_fetch_count = 0;

	mid_frame = cam_queue_empty_frame_get();
	mid_frame->channel_id = ch->ch_id;
	/* if user set this buffer, we use it for dcam output
	 * or else we will allocate one for it.
	 */
	if(ch->dcam_path_id == 0 && module->cam_uinfo.is_4in1 == 1)
		pack_bits = 0;
	else
		pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	pr_info("day raw_proc_post pack_bits %d", pack_bits);
	if (proc_info->fd_dst0 > 0) {
		mid_frame->buf.type = CAM_BUF_USER;
		mid_frame->buf.mfd[0] = proc_info->fd_dst0;
		mid_frame->buf.offset[0] = proc_info->dst0_offset;
		ret = cam_buf_ionbuf_get(&mid_frame->buf);
		if (ret)
			goto mid_fail;
	} else {
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;
		if (module->cam_uinfo.dcam_slice_mode == CAM_OFFLINE_SLICE_SW) {
			width = width / module->cam_uinfo.slice_num;
			if (proc_info->dst_size.height > DCAM_SW_SLICE_HEIGHT_MAX)
				width *= 2;
			width = ALIGN(width, 4);
		}

		if (proc_info->src_format == IMG_PIX_FMT_GREY)
			size = cal_sprd_raw_pitch(width, pack_bits) * height;
		else
			size = width * height * 3;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		ret = cam_buf_alloc(&mid_frame->buf, (size_t)size, 0, module->iommu_enable);
		if (ret)
			goto mid_fail;
	}
	mid_frame->sensor_time = src_frame->sensor_time;
	mid_frame->time = src_frame->time;
	mid_frame->boot_time = src_frame->boot_time;
	mid_frame->boot_sensor_time = src_frame->boot_sensor_time;

	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_OUTPUT_BUF, ch->dcam_path_id, mid_frame);
	if (ret) {
		pr_err("fail to cfg dcam out buffer.\n");
		goto dcam_out_fail;
	}

	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_OUTPUT_BUF, ch->isp_ctx_id, ch->isp_path_id, dst_frame);
	if (ret)
		pr_err("fail to cfg isp out buffer.\n");

	pr_info("raw proc, src %p, mid %p, dst %p\n",
		src_frame, mid_frame, dst_frame);
	cam_queue_init(&ch->share_buf_queue,
			CAM_SHARED_BUF_NUM, camcore_k_frame_put);
	module->cap_status = CAM_CAPTURE_RAWPROC;
	module->dcam_cap_status = DCAM_CAPTURE_START;
	atomic_set(&module->state, CAM_RUNNING);

	if (module->dump_thrd.thread_task) {
		int i = 0;
		cam_queue_init(&module->dump_queue, 10, camcore_k_frame_put);
		init_completion(&module->dump_com);
		mutex_lock(&g_dbg_dump.dump_lock);
		i = module->dcam_idx;
		if (i < 2) {
			g_dbg_dump.dump_start[i] = &module->dump_thrd.thread_com;
		}
		g_dbg_dump.dump_count = 99;
		mutex_unlock(&g_dbg_dump.dump_lock);
		complete(&module->dump_thrd.thread_com);
		pr_debug("cam%d_dumpraw start\n", module->idx);
	}
	ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(module->dcam_dev_handle, src_frame);
	if (ret)
		pr_err("fail to start dcam/isp for raw proc\n");

	atomic_set(&module->timeout_flag, 1);
	ret = camcore_timer_start(&module->cam_timer, CAMERA_TIMEOUT);

	return ret;

dcam_out_fail:
	if (mid_frame->buf.type == CAM_BUF_USER)
		cam_buf_ionbuf_put(&mid_frame->buf);
	else
		cam_buf_free(&mid_frame->buf);
mid_fail:
	cam_queue_empty_frame_put(mid_frame);
	cam_buf_ionbuf_put(&dst_frame->buf);
dst_fail:
	cam_queue_empty_frame_put(dst_frame);
	cam_buf_ionbuf_put(&src_frame->buf);
src_fail:
	cam_queue_empty_frame_put(src_frame);
	ret = module->dcam_dev_handle->dcam_pipe_ops->stop(module->dcam_dev_handle, DCAM_STOP);
	pr_err("fail to call post raw proc\n");
	return ret;
}

static int camcore_raw_post_proc_new(
		struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t pack_bits;
	struct channel_context *ch;
	struct camera_frame *src_frame;
	struct camera_frame *mid_frame;
	struct camera_frame *dst_frame;
	struct timespec cur_ts;

	memset(&cur_ts, 0, sizeof(struct timespec));
	pr_info("cam%d start\n", module->idx);

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_err("fail to get channel enable state\n");
		return -EFAULT;
	}

	pr_info("src %d 0x%x, mid %d, 0x%x, dst %d, 0x%x\n",
		proc_info->fd_src, proc_info->src_offset,
		proc_info->fd_dst0, proc_info->dst0_offset,
		proc_info->fd_dst1, proc_info->dst1_offset);
	src_frame = cam_queue_empty_frame_get();
	src_frame->buf.type = CAM_BUF_USER;
	src_frame->buf.mfd[0] = proc_info->fd_src;
	src_frame->buf.offset[0] = proc_info->src_offset;
	src_frame->channel_id = ch->ch_id;
	src_frame->width = proc_info->src_size.width;
	src_frame->height = proc_info->src_size.height;
	src_frame->endian = proc_info->src_y_endian;
	src_frame->pattern = proc_info->src_pattern;
	ret = cam_buf_ionbuf_get(&src_frame->buf);
	if (ret)
		goto src_fail;

	src_frame->fid = module->simu_fid++;
	ktime_get_ts(&cur_ts);
	src_frame->sensor_time.tv_sec = cur_ts.tv_sec;
	src_frame->sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	src_frame->time = src_frame->sensor_time;
	src_frame->boot_time = ktime_get_boottime();
	src_frame->boot_sensor_time = src_frame->boot_time;

	dst_frame = cam_queue_empty_frame_get();
	dst_frame->buf.type = CAM_BUF_USER;
	dst_frame->buf.mfd[0] = proc_info->fd_dst1;
	dst_frame->buf.offset[0] = proc_info->dst1_offset;
	dst_frame->channel_id = ch->ch_id;
	dst_frame->img_fmt = ch->ch_uinfo.dst_fmt;
	ret = cam_buf_ionbuf_get(&dst_frame->buf);
	if (ret)
		goto dst_fail;

	mid_frame = cam_queue_empty_frame_get();
	mid_frame->channel_id = ch->ch_id;
	pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
	pr_info("day raw_proc_post pack_bits %d", pack_bits);
	if (proc_info->fd_dst0 > 0) {
		mid_frame->buf.type = CAM_BUF_USER;
		mid_frame->buf.mfd[0] = proc_info->fd_dst0;
		mid_frame->buf.offset[0] = proc_info->dst0_offset;
		ret = cam_buf_ionbuf_get(&mid_frame->buf);
		if (ret)
			goto mid_fail;
	} else {
		pr_err("fail to get mid buf fd\n");
		goto mid_fail;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
		DCAM_PATH_CFG_OUTPUT_BUF, ch->dcam_path_id, mid_frame);
	if (ret) {
		pr_err("fail to cfg dcam out buffer.\n");
		goto dcam_out_fail;
	}

	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
		ISP_PATH_CFG_OUTPUT_BUF, ch->isp_ctx_id, ch->isp_path_id, dst_frame);
	if (ret)
		pr_err("fail to cfg isp out buffer.\n");

	pr_info("raw proc, src %p, mid %p, dst %p\n",
		src_frame, mid_frame, dst_frame);

	module->cap_status = CAM_CAPTURE_RAWPROC;
	module->dcam_cap_status = DCAM_CAPTURE_START;

	ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(module->dcam_dev_handle, src_frame);
	if (ret)
		pr_err("fail to start dcam/isp for raw proc\n");

	return ret;

dcam_out_fail:
	cam_buf_ionbuf_put(&mid_frame->buf);
mid_fail:
	cam_queue_empty_frame_put(mid_frame);
	cam_buf_ionbuf_put(&dst_frame->buf);
dst_fail:
	cam_queue_empty_frame_put(dst_frame);
	cam_buf_ionbuf_put(&src_frame->buf);
src_fail:
	cam_queue_empty_frame_put(src_frame);
	pr_err("fail to call post raw proc\n");
	return ret;
}

static int camcore_zoom_proc(void *param)
{
	int update_pv = 0, update_c = 0;
	int update_always = 0;
	struct camera_module *module;
	struct channel_context *ch_prev, *ch_vid, *ch_cap;
	struct camera_frame *pre_zoom_coeff = NULL;
	struct camera_frame *vid_zoom_coeff = NULL;
	struct camera_frame *cap_zoom_coeff = NULL;
	struct sprd_img_rect *crop;

	module = (struct camera_module *)param;
	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
next:
	pre_zoom_coeff = vid_zoom_coeff = cap_zoom_coeff = NULL;
	update_pv = update_c = update_always = 0;
	/* Get node from the preview/video/cap coef queue if exist */
	if (ch_prev->enable)
		pre_zoom_coeff = cam_queue_dequeue(&ch_prev->zoom_coeff_queue,
			struct camera_frame, list);
	if (pre_zoom_coeff) {
		crop = (struct sprd_img_rect *)pre_zoom_coeff->priv_data;
		ch_prev->ch_uinfo.src_crop = *crop;
		kfree(crop);
		cam_queue_empty_frame_put(pre_zoom_coeff);
		update_pv |= 1;
	}

	if (ch_vid->enable)
		vid_zoom_coeff = cam_queue_dequeue(&ch_vid->zoom_coeff_queue,
			struct camera_frame, list);
	if (vid_zoom_coeff) {
		crop = (struct sprd_img_rect *)vid_zoom_coeff->priv_data;
		ch_vid->ch_uinfo.src_crop = *crop;
		kfree(crop);
		cam_queue_empty_frame_put(vid_zoom_coeff);
		update_pv |= 1;
	}

	if (ch_cap->enable)
		cap_zoom_coeff = cam_queue_dequeue(&ch_cap->zoom_coeff_queue,
			struct camera_frame, list);
	if (cap_zoom_coeff) {
		crop = (struct sprd_img_rect *)cap_zoom_coeff->priv_data;
		ch_cap->ch_uinfo.src_crop = *crop;
		kfree(crop);
		cam_queue_empty_frame_put(cap_zoom_coeff);
		update_c |= 1;
	}

	if (update_pv || update_c) {
		if (ch_cap->enable && (ch_cap->mode_ltm == MODE_LTM_CAP))
			update_always = 1;

		if (module->zoom_solution == ZOOM_DEFAULT)
			camcore_channel_size_bininig_cal(module, 1);
		else if (module->zoom_solution == ZOOM_BINNING2 ||
			module->zoom_solution == ZOOM_BINNING4)
			camcore_channel_size_bininig_cal(module, 0);
		else
			camcore_channel_size_rds_cal(module);

		if (ch_cap->enable && (update_c || update_always))
			camcore_channel_size_config(module, ch_cap);
		if (ch_prev->enable && (update_pv || update_always))
			camcore_channel_size_config(module, ch_prev);
		goto next;
	}
	return 0;
}

static int camcore_capture_proc(void *param)
{
	int ret = 0;
	struct camera_module *module;
	struct camera_frame *pframe;
	struct channel_context *channel;

	module = (struct camera_module *)param;

	mutex_lock(&module->fdr_lock);
	if ((module->fdr_done & (1 << CAMERA_IRQ_FDRL)) &&
		(module->fdr_done & (1 << CAMERA_IRQ_FDRH))) {
		pr_info("cam%d fdr done\n", module->idx);
		if (module->fdr_init)
			camcore_fdr_context_deinit(module, &module->channel[CAM_CH_CAP]);
		module->fdr_done = 0;
		mutex_unlock(&module->fdr_lock);
		return 0;
	}
	mutex_unlock(&module->fdr_lock);

	channel = &module->channel[CAM_CH_CAP];
	do {
		pframe = cam_queue_dequeue(&channel->share_buf_queue,
				struct camera_frame, list);
		if (!pframe)
			return 0;
		if (module->dcam_cap_status != DCAM_CAPTURE_START_WITH_TIMESTAMP &&
			pframe->boot_sensor_time < module->capture_times) {
			pr_debug("cam%d cap skip frame type[%d] cap_time[%lld] sof_time[%lld]\n",
				module->idx,
				module->dcam_cap_status,
				module->capture_times,
				pframe->boot_sensor_time);
			if (pframe->sync_data)
				dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
				module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);
		} else {
			break;
		}
	} while (pframe);

	ret = -1;
	if (module->cap_status != CAM_CAPTURE_STOP) {
		pr_info("capture frame cam id %d, fid[%d],  frame w %d, h %d\n",
			module->idx, pframe->fid,
			pframe->width, pframe->height);
		pr_info("cap time %lld, frame time %lld\n",
			module->capture_times, pframe->boot_sensor_time);
		ret = module->isp_dev_handle->isp_ops->proc_frame(module->isp_dev_handle, pframe,
				channel->isp_ctx_id);
	}

	if (ret) {
		pr_info("capture stop or isp queue overflow\n");
		if (module->cam_uinfo.dcam_slice_mode && pframe->dcam_idx == DCAM_ID_1)
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
				module->aux_dcam_dev,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->aux_dcam_path_id, pframe);
		else
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(
				module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, pframe);

		/* Bug 1103913. In Race condition when We have already received stop capture &
		 * stream_off request & then capture thread gets chance to execute. In that case
		 * module->dcam_dev_handle->dcam_pipe_ops->cfg_path will fail & return non-Zero value. This will  cause memory leak.
		 * So we need to free pframe buffer explicitely.
		 */
		if (ret)
			camcore_k_frame_put((void *)pframe);
	}
	return 0;
}

static int camcore_module_init(struct camera_module *module)
{
	int ret = 0;
	int ch;
	struct channel_context *channel;
	struct cam_thread_info *thrd;

	pr_info("sprd_img: camera dev %d init start!\n", module->idx);

	atomic_set(&module->state, CAM_INIT);
	mutex_init(&module->lock);
	mutex_init(&module->fdr_lock);
	init_completion(&module->frm_com);
	init_completion(&module->streamoff_com);
	module->exit_flag = 0;

	module->cap_status = CAM_CAPTURE_STOP;
	module->dcam_cap_status = DCAM_CAPTURE_STOP;

	for (ch = 0; ch < CAM_CH_MAX; ch++) {
		channel = &module->channel[ch];
		channel->ch_id = ch;
		channel->dcam_path_id = -1;
		channel->isp_ctx_id = -1;
		channel->isp_path_id = -1;
		mutex_init(&module->buf_lock[channel->ch_id]);
		init_completion(&channel->alloc_com);
	}

	/* create capture thread */
	thrd = &module->cap_thrd;
	sprintf(thrd->thread_name, "cam%d_capture", module->idx);
	ret = camcore_thread_create(module, thrd, camcore_capture_proc);
	if (ret)
		goto exit;

	/* create zoom thread */
	thrd = &module->zoom_thrd;
	sprintf(thrd->thread_name, "cam%d_zoom", module->idx);
	ret = camcore_thread_create(module, thrd, camcore_zoom_proc);
	if (ret)
		goto exit;

	/* create buf thread */
	thrd = &module->buf_thrd;
	sprintf(thrd->thread_name, "cam%d_alloc_buf", module->idx);
	ret = camcore_thread_create(module, thrd, camcore_buffers_alloc);
	if (ret)
		goto exit;

	if (g_dbg_dump.dump_en) {
		/* create dump thread */
		thrd = &module->dump_thrd;
		sprintf(thrd->thread_name, "cam%d_dumpraw", module->idx);
		ret = camcore_thread_create(module, thrd, camcore_dumpraw_proc);
		if (ret)
			goto exit;
	}

	module->flash_core_handle = get_cam_flash_handle(module->idx);

	camcore_timer_init(&module->cam_timer, (unsigned long)module);
	module->attach_sensor_id = SPRD_SENSOR_ID_MAX + 1;
	module->is_smooth_zoom = 1;
	cam_queue_init(&module->frm_queue,
		CAM_FRAME_Q_LEN, camcore_empty_frame_put);
	cam_queue_init(&module->irq_queue,
		CAM_IRQ_Q_LEN, camcore_empty_frame_put);
	cam_queue_init(&module->statis_queue,
		CAM_STATIS_Q_LEN, camcore_empty_frame_put);
	cam_queue_init(&module->alloc_queue,
		CAM_ALLOC_Q_LEN, camcore_empty_frame_put);
	pr_info("module[%d] init OK %p!\n", module->idx, module);
	return 0;
exit:
	camcore_thread_stop(&module->cap_thrd);
	camcore_thread_stop(&module->zoom_thrd);
	camcore_thread_stop(&module->buf_thrd);
	camcore_thread_stop(&module->dump_thrd);
	return ret;
}

static int camcore_module_deinit(struct camera_module *module)
{
	int ch = 0;
	struct channel_context *channel = NULL;

	put_cam_flash_handle(module->flash_core_handle);
	cam_queue_clear(&module->frm_queue, struct camera_frame, list);
	cam_queue_clear(&module->irq_queue, struct camera_frame, list);
	cam_queue_clear(&module->statis_queue, struct camera_frame, list);
	cam_queue_clear(&module->alloc_queue, struct camera_frame, list);
	camcore_thread_stop(&module->cap_thrd);
	camcore_thread_stop(&module->zoom_thrd);
	camcore_thread_stop(&module->buf_thrd);
	camcore_thread_stop(&module->dump_thrd);
	for (ch = 0; ch < CAM_CH_MAX; ch++) {
		channel = &module->channel[ch];
		mutex_destroy(&module->buf_lock[channel->ch_id]);
	}
	mutex_destroy(&module->fdr_lock);
	mutex_destroy(&module->lock);
	return 0;
}

#define CAM_IOCTL_LAYER
#include "cam_ioctl.c"
#undef CAM_IOCTL_LAYER

static struct cam_ioctl_cmd ioctl_cmds_table[] = {
	[_IOC_NR(SPRD_IMG_IO_SET_MODE)]             = {SPRD_IMG_IO_SET_MODE,             camioctl_mode_set},
	[_IOC_NR(SPRD_IMG_IO_SET_CAP_SKIP_NUM)]     = {SPRD_IMG_IO_SET_CAP_SKIP_NUM,     camioctl_cap_skip_num_set},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_SIZE)]      = {SPRD_IMG_IO_SET_SENSOR_SIZE,      camioctl_sensor_size_set},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_TRIM)]      = {SPRD_IMG_IO_SET_SENSOR_TRIM,      camioctl_sensor_trim_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FRM_ID_BASE)]      = {SPRD_IMG_IO_SET_FRM_ID_BASE,      camioctl_frame_id_base_set},
	[_IOC_NR(SPRD_IMG_IO_SET_CROP)]             = {SPRD_IMG_IO_SET_CROP,             canioctl_crop_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FLASH)]            = {SPRD_IMG_IO_SET_FLASH,            camioctl_flash_set},
	[_IOC_NR(SPRD_IMG_IO_SET_OUTPUT_SIZE)]      = {SPRD_IMG_IO_SET_OUTPUT_SIZE,      camioctl_output_size_set},
	[_IOC_NR(SPRD_IMG_IO_SET_ZOOM_MODE)]        = {SPRD_IMG_IO_SET_ZOOM_MODE,        camioctl_zoom_mode_set},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_IF)]        = {SPRD_IMG_IO_SET_SENSOR_IF,        camioctl_sensor_if_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FRAME_ADDR)]       = {SPRD_IMG_IO_SET_FRAME_ADDR,       camioctl_frame_addr_set},
	[_IOC_NR(SPRD_IMG_IO_PATH_FRM_DECI)]        = {SPRD_IMG_IO_PATH_FRM_DECI,        camioctl_frm_deci_set},
	[_IOC_NR(SPRD_IMG_IO_PATH_PAUSE)]           = {SPRD_IMG_IO_PATH_PAUSE,           camioctl_path_pause},
	[_IOC_NR(SPRD_IMG_IO_PATH_RESUME)]          = {SPRD_IMG_IO_PATH_RESUME,          camioctl_path_resume},
	[_IOC_NR(SPRD_IMG_IO_STREAM_ON)]            = {SPRD_IMG_IO_STREAM_ON,            camioctl_stream_on},
	[_IOC_NR(SPRD_IMG_IO_STREAM_OFF)]           = {SPRD_IMG_IO_STREAM_OFF,           camioctl_stream_off},
	[_IOC_NR(SPRD_IMG_IO_STREAM_PAUSE)]         = {SPRD_IMG_IO_STREAM_PAUSE,         camioctl_stream_pause},
	[_IOC_NR(SPRD_IMG_IO_STREAM_RESUME)]        = {SPRD_IMG_IO_STREAM_RESUME,        camioctl_stream_resume},
	[_IOC_NR(SPRD_IMG_IO_GET_FMT)]              = {SPRD_IMG_IO_GET_FMT,              camioctl_fmt_get},
	[_IOC_NR(SPRD_IMG_IO_GET_CH_ID)]            = {SPRD_IMG_IO_GET_CH_ID,            camioctl_ch_id_get},
	[_IOC_NR(SPRD_IMG_IO_GET_TIME)]             = {SPRD_IMG_IO_GET_TIME,             camioctl_time_get},
	[_IOC_NR(SPRD_IMG_IO_CHECK_FMT)]            = {SPRD_IMG_IO_CHECK_FMT,            camioctl_fmt_check},
	[_IOC_NR(SPRD_IMG_IO_SET_SHRINK)]           = {SPRD_IMG_IO_SET_SHRINK,           camioctl_shrink_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FREQ_FLAG)]        = {SPRD_IMG_IO_SET_FREQ_FLAG,        NULL},
	[_IOC_NR(SPRD_IMG_IO_CFG_FLASH)]            = {SPRD_IMG_IO_CFG_FLASH,            camioctl_flash_cfg},
	[_IOC_NR(SPRD_IMG_IO_PDAF_CONTROL)]         = {SPRD_IMG_IO_PDAF_CONTROL,         NULL},
	[_IOC_NR(SPRD_IMG_IO_GET_IOMMU_STATUS)]     = {SPRD_IMG_IO_GET_IOMMU_STATUS,     camioctl_iommu_status_get},
	[_IOC_NR(SPRD_IMG_IO_DISABLE_MODE)]         = {SPRD_IMG_IO_DISABLE_MODE,         NULL},
	[_IOC_NR(SPRD_IMG_IO_ENABLE_MODE)]          = {SPRD_IMG_IO_ENABLE_MODE,          NULL},
	[_IOC_NR(SPRD_IMG_IO_START_CAPTURE)]        = {SPRD_IMG_IO_START_CAPTURE,        camioctl_capture_start},
	[_IOC_NR(SPRD_IMG_IO_STOP_CAPTURE)]         = {SPRD_IMG_IO_STOP_CAPTURE,         camioctl_capture_stop},
	[_IOC_NR(SPRD_IMG_IO_SET_PATH_SKIP_NUM)]    = {SPRD_IMG_IO_SET_PATH_SKIP_NUM,    NULL},
	[_IOC_NR(SPRD_IMG_IO_SBS_MODE)]             = {SPRD_IMG_IO_SBS_MODE,             NULL},
	[_IOC_NR(SPRD_IMG_IO_DCAM_PATH_SIZE)]       = {SPRD_IMG_IO_DCAM_PATH_SIZE,       camioctl_dcam_path_size},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_MAX_SIZE)]  = {SPRD_IMG_IO_SET_SENSOR_MAX_SIZE,  camioctl_sensor_max_size_set},
	[_IOC_NR(SPRD_ISP_IO_IRQ)]                  = {SPRD_ISP_IO_IRQ,                  NULL},
	[_IOC_NR(SPRD_ISP_IO_READ)]                 = {SPRD_ISP_IO_READ,                 NULL},
	[_IOC_NR(SPRD_ISP_IO_WRITE)]                = {SPRD_ISP_IO_WRITE,                NULL},
	[_IOC_NR(SPRD_ISP_IO_RST)]                  = {SPRD_ISP_IO_RST,                  NULL},
	[_IOC_NR(SPRD_ISP_IO_STOP)]                 = {SPRD_ISP_IO_STOP,                 NULL},
	[_IOC_NR(SPRD_ISP_IO_INT)]                  = {SPRD_ISP_IO_INT,                  NULL},
	[_IOC_NR(SPRD_ISP_IO_SET_STATIS_BUF)]       = {SPRD_ISP_IO_SET_STATIS_BUF,       camioctl_statis_buf_set},
	[_IOC_NR(SPRD_ISP_IO_CFG_PARAM)]            = {SPRD_ISP_IO_CFG_PARAM,            camioctl_param_cfg},
	[_IOC_NR(SPRD_ISP_REG_READ)]                = {SPRD_ISP_REG_READ,                NULL},
	[_IOC_NR(SPRD_ISP_IO_POST_3DNR)]            = {SPRD_ISP_IO_POST_3DNR,            NULL},
	[_IOC_NR(SPRD_STATIS_IO_CFG_PARAM)]         = {SPRD_STATIS_IO_CFG_PARAM,         NULL},
	[_IOC_NR(SPRD_ISP_IO_RAW_CAP)]              = {SPRD_ISP_IO_RAW_CAP,              camioctl_raw_proc},
	[_IOC_NR(SPRD_IMG_IO_GET_DCAM_RES)]         = {SPRD_IMG_IO_GET_DCAM_RES,         camioctl_cam_res_get},
	[_IOC_NR(SPRD_IMG_IO_PUT_DCAM_RES)]         = {SPRD_IMG_IO_PUT_DCAM_RES,         camioctl_cam_res_put},
	[_IOC_NR(SPRD_ISP_IO_SET_PULSE_LINE)]       = {SPRD_ISP_IO_SET_PULSE_LINE,       NULL},
	[_IOC_NR(SPRD_ISP_IO_CFG_START)]            = {SPRD_ISP_IO_CFG_START,            NULL},
	[_IOC_NR(SPRD_ISP_IO_POST_YNR)]             = {SPRD_ISP_IO_POST_YNR,             NULL},
	[_IOC_NR(SPRD_ISP_IO_SET_NEXT_VCM_POS)]     = {SPRD_ISP_IO_SET_NEXT_VCM_POS,     NULL},
	[_IOC_NR(SPRD_ISP_IO_SET_VCM_LOG)]          = {SPRD_ISP_IO_SET_VCM_LOG,          NULL},
	[_IOC_NR(SPRD_IMG_IO_SET_3DNR)]             = {SPRD_IMG_IO_SET_3DNR,             NULL},
	[_IOC_NR(SPRD_IMG_IO_SET_FUNCTION_MODE)]    = {SPRD_IMG_IO_SET_FUNCTION_MODE,    camioctl_function_mode_set},
	[_IOC_NR(SPRD_IMG_IO_GET_FLASH_INFO)]       = {SPRD_IMG_IO_GET_FLASH_INFO,       camioctl_flash_get},
	[_IOC_NR(SPRD_ISP_IO_MASK_3A)]              = {SPRD_ISP_IO_MASK_3A,              NULL},
	[_IOC_NR(SPRD_IMG_IO_EBD_CONTROL)]          = {SPRD_IMG_IO_EBD_CONTROL,          camioctl_ebd_control},
	[_IOC_NR(SPRD_IMG_IO_SET_4IN1_ADDR)]        = {SPRD_IMG_IO_SET_4IN1_ADDR,        camioctl_4in1_raw_addr_set},
	[_IOC_NR(SPRD_IMG_IO_4IN1_POST_PROC)]       = {SPRD_IMG_IO_4IN1_POST_PROC,       camioctl_4in1_post_proc},
	[_IOC_NR(SPRD_IMG_IO_SET_CAM_SECURITY)]     = {SPRD_IMG_IO_SET_CAM_SECURITY,     camioctl_cam_security_set},
	[_IOC_NR(SPRD_IMG_IO_GET_PATH_RECT)]        = {SPRD_IMG_IO_GET_PATH_RECT,        camioctl_path_rect_get},
	[_IOC_NR(SPRD_IMG_IO_SET_3DNR_MODE)]        = {SPRD_IMG_IO_SET_3DNR_MODE,        camioctl_3dnr_mode_set},
	[_IOC_NR(SPRD_IMG_IO_SET_AUTO_3DNR_MODE)]   = {SPRD_IMG_IO_SET_AUTO_3DNR_MODE,   camioctl_auto_3dnr_mode_set},
	[_IOC_NR(SPRD_IMG_IO_CAPABILITY)]           = {SPRD_IMG_IO_CAPABILITY,           camioctl_capability_get},
	[_IOC_NR(SPRD_IMG_IO_POST_FDR)]             = {SPRD_IMG_IO_POST_FDR,             camioctl_fdr_post},
	[_IOC_NR(SPRD_IMG_IO_CAM_TEST)]             = {SPRD_IMG_IO_CAM_TEST,             camioctl_cam_test},
	[_IOC_NR(SPRD_IMG_IO_SET_LONGEXP_CAP)]      = {SPRD_IMG_IO_SET_LONGEXP_CAP,      camioctl_longexp_mode_set},
};

static long camcore_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	int locked = 0;
	struct camera_module *module = NULL;
	struct cam_ioctl_cmd *ioctl_cmd_p = NULL;
	int nr = _IOC_NR(cmd);

	pr_debug("cam ioctl, cmd:0x%x, cmdnum %d\n", cmd, nr);

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (unlikely(!(nr >= 0 && nr < ARRAY_SIZE(ioctl_cmds_table)))) {
		pr_info("invalid cmd: 0x%xn", cmd);
		return -EINVAL;
	}

	ioctl_cmd_p = &ioctl_cmds_table[nr];
	if (unlikely((ioctl_cmd_p->cmd != cmd) ||
			(ioctl_cmd_p->cmd_proc == NULL))) {
		pr_debug("unsupported cmd_k: 0x%x, cmd_u: 0x%x, nr: %d\n",
			ioctl_cmd_p->cmd, cmd, nr);
		return 0;
	}

	/* There is race condition under several cases during stream/off
	 * Take care of lock use
	 */
	if (atomic_read(&module->state) != CAM_RUNNING
		|| cmd == SPRD_IMG_IO_STREAM_OFF
		|| cmd == SPRD_ISP_IO_CFG_PARAM) {
		mutex_lock(&module->lock);
		locked = 1;
	}

	ret = ioctl_cmd_p->cmd_proc(module, arg);
	if (ret) {
		pr_err("fail to ioctl cmd:%x, nr:%d, func %ps\n",
			cmd, nr, ioctl_cmd_p->cmd_proc);
		goto exit;
	}

	pr_debug("cam id:%d, %ps, done!\n",
		module->idx, ioctl_cmd_p->cmd_proc);
exit:
	if (locked)
		mutex_unlock(&module->lock);
	return ret;
}

static ssize_t camcore_read(struct file *file, char __user *u_data,
		size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	int i = 0;
	int superzoom_val = 0;
	struct sprd_img_read_op read_op;
	struct camera_module *module = NULL;
	struct camera_frame *pframe;
	struct channel_context *pchannel;
	struct sprd_img_path_capability *cap;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_reg_trace trace;

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (cnt != sizeof(struct sprd_img_read_op)) {
		pr_err("fail to img read, cnt %zd read_op %d\n", cnt,
			(int32_t)sizeof(struct sprd_img_read_op));
		return -EIO;
	}

	if (copy_from_user(&read_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	pr_debug("cam %d read cmd %d\n", module->idx, read_op.cmd);

	switch (read_op.cmd) {
	case SPRD_IMG_GET_SCALE_CAP:
		hw = module->grp->hw_info;
		for (i = 0; i < DCAM_ID_MAX; i++) {
			if (hw->ip_dcam[i]->superzoom_support) {
				superzoom_val = 1;
				break;
			}
		}

		if (superzoom_val)
			read_op.parm.reserved[1] = 10;
		else
			read_op.parm.reserved[1] = 4;

		read_op.parm.reserved[0] = 4672;
		read_op.parm.reserved[2] = 4672;
		pr_debug("line threshold %d, sc factor %d, scaling %d.\n",
			read_op.parm.reserved[0],
			read_op.parm.reserved[1],
			read_op.parm.reserved[2]);
		break;
	case SPRD_IMG_GET_FRM_BUFFER:
rewait:
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		while (1) {
			ret = wait_for_completion_interruptible(
				&module->frm_com);
			if (ret == 0) {
				break;
			} else if (ret == -ERESTARTSYS) {
				read_op.evt = IMG_SYS_BUSY;
				ret = 0;
				goto read_end;
			} else {
				pr_err("read frame buf, fail to down, %d\n",
					ret);
				return -EPERM;
			}
		}

		pchannel = NULL;
		pframe = cam_queue_dequeue(&module->frm_queue,
			struct camera_frame, list);

		if (!pframe) {
			/* any exception happens or user trigger exit. */
			pr_info("No valid frame buffer. tx stop.\n");
			read_op.evt = IMG_TX_STOP;
		} else if (pframe->evt == IMG_TX_DONE) {
			atomic_set(&module->timeout_flag, 0);
			if ((pframe->irq_type == CAMERA_IRQ_4IN1_DONE) ||
				(pframe->irq_type == CAMERA_IRQ_FDRL) ||
				(pframe->irq_type == CAMERA_IRQ_FDRH) ||
				(pframe->irq_type == CAMERA_IRQ_IMG)) {
				cam_buf_ionbuf_put(&pframe->buf);
				pchannel = &module->channel[pframe->channel_id];
				if (pframe->buf.mfd[0] ==
					pchannel->reserved_buf_fd) {
					pr_info("get output buffer with reserved frame fd %d\n",
						pchannel->reserved_buf_fd);
					cam_queue_empty_frame_put(pframe);
					goto rewait;
				}
				read_op.parm.frame.channel_id = pframe->channel_id;
				read_op.parm.frame.index = pchannel->frm_base_id;
				read_op.parm.frame.frm_base_id = pchannel->frm_base_id;
				read_op.parm.frame.img_fmt = pframe->img_fmt;
			}
			read_op.evt = pframe->evt;
			read_op.parm.frame.irq_type = pframe->irq_type;
			read_op.parm.frame.irq_property = pframe->irq_property;
			read_op.parm.frame.length = pframe->width;
			read_op.parm.frame.height = pframe->height;
			read_op.parm.frame.real_index = pframe->fid;
			read_op.parm.frame.frame_id = pframe->fid;
			/*
			 * read_op.parm.frame.sec = pframe->time.tv_sec;
			 * read_op.parm.frame.usec = pframe->time.tv_usec;
			 * read_op.parm.frame.monoboottime = pframe->boot_time;
			 */
			/* use SOF time instead of ISP time for better accuracy */
			read_op.parm.frame.sec = pframe->sensor_time.tv_sec;
			read_op.parm.frame.usec = pframe->sensor_time.tv_usec;
			read_op.parm.frame.monoboottime = pframe->boot_sensor_time;
			read_op.parm.frame.yaddr_vir = (uint32_t)pframe->buf.addr_vir[0];
			read_op.parm.frame.uaddr_vir = (uint32_t)pframe->buf.addr_vir[1];
			read_op.parm.frame.vaddr_vir = (uint32_t)pframe->buf.addr_vir[2];
			read_op.parm.frame.mfd = pframe->buf.mfd[0];
			read_op.parm.frame.yaddr = pframe->buf.offset[0];
			read_op.parm.frame.uaddr = pframe->buf.offset[1];
			read_op.parm.frame.vaddr = pframe->buf.offset[2];

			if ((pframe->irq_type == CAMERA_IRQ_FDRL) ||
				(pframe->irq_type == CAMERA_IRQ_FDRH)) {
				pr_info("FDR %d ch %d, evt %d, fid %d, buf_fd %d,  time  %06d.%06d\n",
					pframe->irq_type,  read_op.parm.frame.channel_id, read_op.evt,
					read_op.parm.frame.real_index, read_op.parm.frame.mfd,
					read_op.parm.frame.sec, read_op.parm.frame.usec);
			}
			/* for statis buffer address below. */
			read_op.parm.frame.addr_offset = pframe->buf.offset[0];
			if (pframe->irq_type == CAMERA_IRQ_STATIS)
				read_op.parm.frame.zoom_ratio = pframe->zoom_ratio;
			else
				read_op.parm.frame.zoom_ratio = module->zoom_ratio;
		} else {
			struct cam_hw_info *hw = module->grp->hw_info;

			pr_err("fail to get correct event %d\n", pframe->evt);
			if (hw == NULL) {
				pr_err("fail to get hw ops.\n");
				return -EFAULT;
			}
			csi_api_reg_trace();
			trace.type = ABNORMAL_REG_TRACE;
			trace.idx = module->dcam_idx;
			hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
			read_op.evt = pframe->evt;
			read_op.parm.frame.irq_type = pframe->irq_type;
			read_op.parm.frame.irq_property = pframe->irq_property;
		}

		pr_debug("read frame, evt 0x%x irq %d ch 0x%x index 0x%x mfd %d\n",
			read_op.evt,
			read_op.parm.frame.irq_type,
			read_op.parm.frame.channel_id,
			read_op.parm.frame.real_index,
			read_op.parm.frame.mfd);

		if (pframe) {
			if (pframe->irq_type != CAMERA_IRQ_4IN1_DONE) {
				cam_queue_empty_frame_put(pframe);
				break;
			}
			/* 4in1 report frame for remosaic
			 * save frame for 4in1_post IOCTL
			 */
			ret = cam_queue_enqueue(&module->remosaic_queue,
				&pframe->list);
			if (!ret)
				break;
			/* fail, give back */
			cam_queue_empty_frame_put(pframe);
			ret = 0;
		}

		break;

	case SPRD_IMG_GET_PATH_CAP:
		pr_debug("get path capbility\n");
		cap = &read_op.parm.capability;
		memset(cap, 0, sizeof(struct sprd_img_path_capability));
		cap->support_3dnr_mode = 1;
		cap->support_4in1 = 1;
		cap->count = 6;
		cap->path_info[CAM_CH_RAW].support_yuv = 0;
		cap->path_info[CAM_CH_RAW].support_raw = 1;
		cap->path_info[CAM_CH_RAW].support_jpeg = 0;
		cap->path_info[CAM_CH_RAW].support_scaling = 0;
		cap->path_info[CAM_CH_RAW].support_trim = 1;
		cap->path_info[CAM_CH_RAW].is_scaleing_path = 0;
		cap->path_info[CAM_CH_PRE].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_PRE].support_yuv = 1;
		cap->path_info[CAM_CH_PRE].support_raw = 0;
		cap->path_info[CAM_CH_PRE].support_jpeg = 0;
		cap->path_info[CAM_CH_PRE].support_scaling = 1;
		cap->path_info[CAM_CH_PRE].support_trim = 1;
		cap->path_info[CAM_CH_PRE].is_scaleing_path = 0;
		cap->path_info[CAM_CH_CAP].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_CAP].support_yuv = 1;
		cap->path_info[CAM_CH_CAP].support_raw = 0;
		cap->path_info[CAM_CH_CAP].support_jpeg = 0;
		cap->path_info[CAM_CH_CAP].support_scaling = 1;
		cap->path_info[CAM_CH_CAP].support_trim = 1;
		cap->path_info[CAM_CH_CAP].is_scaleing_path = 0;
		cap->path_info[CAM_CH_VID].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_VID].support_yuv = 1;
		cap->path_info[CAM_CH_VID].support_raw = 0;
		cap->path_info[CAM_CH_VID].support_jpeg = 0;
		cap->path_info[CAM_CH_VID].support_scaling = 1;
		cap->path_info[CAM_CH_VID].support_trim = 1;
		cap->path_info[CAM_CH_VID].is_scaleing_path = 0;
		cap->path_info[CAM_CH_PRE_THM].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_PRE_THM].support_yuv = 1;
		cap->path_info[CAM_CH_PRE_THM].support_raw = 0;
		cap->path_info[CAM_CH_PRE_THM].support_jpeg = 0;
		cap->path_info[CAM_CH_PRE_THM].support_scaling = 1;
		cap->path_info[CAM_CH_PRE_THM].support_trim = 1;
		cap->path_info[CAM_CH_PRE_THM].is_scaleing_path = 0;
		cap->path_info[CAM_CH_CAP_THM].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_CAP_THM].support_yuv = 1;
		cap->path_info[CAM_CH_CAP_THM].support_raw = 0;
		cap->path_info[CAM_CH_CAP_THM].support_jpeg = 0;
		cap->path_info[CAM_CH_CAP_THM].support_scaling = 1;
		cap->path_info[CAM_CH_CAP_THM].support_trim = 1;
		cap->path_info[CAM_CH_CAP_THM].is_scaleing_path = 0;
		break;

	default:
		pr_err("fail to get valid cmd\n");
		return -EINVAL;
	}

read_end:
	if (copy_to_user((void __user *)u_data, &read_op, cnt))
		ret = -EFAULT;

	if (ret)
		cnt = ret;

	return cnt;
}

static ssize_t camcore_write(struct file *file, const char __user *u_data,
		size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	struct sprd_img_write_op write_op;
	struct camera_module *module = NULL;

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (cnt != sizeof(struct sprd_img_write_op)) {
		pr_err("fail to write, cnt %zd  write_op %d\n", cnt,
				(uint32_t)sizeof(struct sprd_img_write_op));
		return -EIO;
	}

	if (copy_from_user(&write_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	switch (write_op.cmd) {
	case SPRD_IMG_STOP_DCAM:
		pr_info("user stop camera %d\n", module->idx);
		module->exit_flag = 1;
		complete(&module->frm_com);
		break;

	default:
		pr_err("fail to get write cmd %d\n", write_op.cmd);
		break;
	}

	ret =  copy_to_user((void __user *)u_data, &write_op, cnt);
	if (ret) {
		pr_err("fail to get user info\n");
		cnt = ret;
		return -EFAULT;
	}

	return cnt;
}

static int camcore_open(struct inode *node, struct file *file)
{
	int ret = 0;
	unsigned long flag;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;
	struct miscdevice *md = file->private_data;
	uint32_t i, idx, count = 0;

	grp = md->this_device->platform_data;
	count = grp->dcam_count;

	if (count == 0 || count > CAM_COUNT) {
		pr_err("fail to get valid dts configured dcam count\n");
		return -ENODEV;
	}

	if (atomic_inc_return(&grp->camera_opened) > count) {
		pr_err("fail to open camera, all %d cameras opened already.", count);
		atomic_dec(&grp->camera_opened);
		return -EMFILE;
	}

	pr_info("sprd_img: the camera opened count %d\n",
		atomic_read(&grp->camera_opened));

	pr_info("camca: camsec_mode = %d\n", grp->camsec_cfg.camsec_mode);

	spin_lock_irqsave(&grp->module_lock, flag);
	for (i = 0, idx = count; i < count; i++) {
		if ((grp->module_used & (1 << i)) == 0) {
			if (grp->module[i] != NULL) {
				pr_err("fail to get null module, un-release camera module:  %p, idx %d\n",
					grp->module[i], i);
				spin_unlock_irqrestore(&grp->module_lock, flag);
				ret = -EMFILE;
				goto exit;
			}
			idx = i;
			grp->module_used |= (1 << i);
			break;
		}
	}
	spin_unlock_irqrestore(&grp->module_lock, flag);

	if (idx == count) {
		pr_err("fail to get available camera module.\n");
		ret = -EMFILE;
		goto exit;
	}

	pr_debug("kzalloc. size of module %x, group %x\n",
		(int)sizeof(struct camera_module),
		(int) sizeof(struct camera_group));

	module = vzalloc(sizeof(struct camera_module));
	if (!module) {
		pr_err("fail to alloc camera module %d\n", idx);
		ret = -ENOMEM;
		goto alloc_fail;
	}

	module->idx = idx;
	ret = camcore_module_init(module);
	if (ret) {
		pr_err("fail to init camera module %d\n", idx);
		ret = -ENOMEM;
		goto init_fail;
	}

	if (atomic_read(&grp->camera_opened) == 1) {
		/* should check all needed interface here. */

		if (grp->hw_info && grp->hw_info->soc_dcam->pdev)
			ret = cam_buf_iommudev_reg(
				&grp->hw_info->soc_dcam->pdev->dev,
				CAM_IOMMUDEV_DCAM);
		if (grp->hw_info && grp->hw_info->soc_isp->pdev)
			ret = cam_buf_iommudev_reg(
				&grp->hw_info->soc_isp->pdev->dev,
				CAM_IOMMUDEV_ISP);

		g_empty_frm_q = &grp->empty_frm_q;
		cam_queue_init(g_empty_frm_q, CAM_EMP_Q_LEN_MAX,
			cam_queue_empty_frame_free);
		g_empty_state_q = &grp->empty_state_q;
		cam_queue_init(g_empty_state_q, CAM_EMP_Q_LEN_MAX,
			cam_queue_empty_state_free);

		pr_info("init frm_q%p state_q%p\n", g_empty_frm_q, g_empty_state_q);
	}

	module->idx = idx;
	module->grp = grp;
	grp->module[idx] = module;
	file->private_data = (void *)module;

	pr_info("sprd_img: open end! %d, %p, %p, grp %p\n",
		idx, module, grp->module[idx], grp);

	return 0;

init_fail:
	vfree(module);

alloc_fail:
	spin_lock_irqsave(&grp->module_lock, flag);
	grp->module_used &= ~(1 << idx);
	grp->module[idx] = NULL;
	spin_unlock_irqrestore(&grp->module_lock, flag);

exit:
	atomic_dec(&grp->camera_opened);

	pr_err("fail to open camera %d\n", ret);
	return ret;
}

/* sprd_img_release may be called for all state.
 * if release is called,
 * all other system calls in this file should be returned before.
 * state may be (RUNNING / IDLE / INIT).
 */
static int camcore_release(struct inode *node, struct file *file)
{
	int ret = 0;
	int idx = 0;
	unsigned long flag;
	struct camera_group *group = NULL;
	struct camera_module *module;
	struct dcam_pipe_dev *dcam_dev = NULL;
	struct isp_pipe_dev *isp_dev = NULL;

	pr_info("sprd_img: cam release start.\n");

	module = (struct camera_module *)file->private_data;
	if (!module || !module->grp) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	group = module->grp;
	idx = module->idx;

	if (module->grp->camsec_cfg.camsec_mode  != SEC_UNABLE) {
		bool ret = 0;

		module->grp->camsec_cfg.camsec_mode = SEC_UNABLE;
		ret = cam_trusty_security_set(&module->grp->camsec_cfg,
			CAM_TRUSTY_EXIT);

		pr_info("camca :camsec_mode%d, ret %d\n",
			module->grp->camsec_cfg.camsec_mode, ret);
	}

	pr_info("cam %d, state %d\n", idx,
		atomic_read(&module->state));
	pr_info("used: %d, module %px, %px, grp %px\n",
		group->module_used, module, group->module[idx], group);

	spin_lock_irqsave(&group->module_lock, flag);
	if (((group->module_used & (1 << idx)) == 0) ||
		(group->module[idx] != module)) {
		pr_err("fail to release camera %d. used:%x, module:%px\n",
			idx, group->module_used, module);
		spin_unlock_irqrestore(&group->module_lock, flag);
		return -EFAULT;
	}
	spin_unlock_irqrestore(&group->module_lock, flag);

	ret = camioctl_stream_off(module, 0L);

	camcore_module_deinit(module);

	if (atomic_read(&module->state) == CAM_IDLE) {
		module->attach_sensor_id = -1;

		dcam_dev = module->dcam_dev_handle;
		isp_dev = module->isp_dev_handle;

		if (dcam_dev) {
			pr_info("force close dcam %px\n", dcam_dev);
			module->dcam_dev_handle->dcam_pipe_ops->close(dcam_dev);
			dcam_core_dcam_if_dev_put(dcam_dev);
			module->dcam_dev_handle = NULL;
		}

		if (isp_dev) {
			pr_info("force close isp %px\n", isp_dev);
			module->isp_dev_handle->isp_ops->close(isp_dev);
			isp_core_pipe_dev_put(isp_dev);
			module->isp_dev_handle = NULL;
		}
	}

	spin_lock_irqsave(&group->module_lock, flag);
	group->module_used &= ~(1 << idx);
	group->module[idx] = NULL;
	spin_unlock_irqrestore(&group->module_lock, flag);

	vfree(module);
	file->private_data = NULL;

	if (atomic_dec_return(&group->camera_opened) == 0) {

		cam_buf_iommudev_unreg(CAM_IOMMUDEV_DCAM);
		cam_buf_iommudev_unreg(CAM_IOMMUDEV_ISP);

		pr_info("release %px\n", g_empty_frm_q);

		/* g_leak_debug_cnt should be 0 after clr, or else memory leak.
		 */
		cam_queue_clear(g_empty_frm_q, struct camera_frame, list);
		g_empty_frm_q = NULL;
		cam_queue_clear(g_empty_state_q, struct isp_stream_ctrl, list);
		g_empty_state_q = NULL;

		ret = cam_buf_mdbg_check();
		atomic_set(&group->runner_nr, 0);
	}

	pr_info("sprd_img: cam %d release end.\n", idx);

	return ret;
}

static const struct file_operations image_fops = {
	.open = camcore_open,
	.unlocked_ioctl = camcore_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_sprd_img_ioctl,
#endif
	.release = camcore_release,
	.read = camcore_read,
	.write = camcore_write,
};

static struct miscdevice image_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = IMG_DEVICE_NAME,
	.fops = &image_fops,
};

static int camcore_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("Start camera img probe\n");
	group = kzalloc(sizeof(struct camera_group), GFP_KERNEL);
	if (group == NULL) {
		pr_err("fail to alloc memory\n");
		return -ENOMEM;
	}

	ret = misc_register(&image_dev);
	if (ret) {
		pr_err("fail to register misc devices, ret %d\n", ret);
		kfree(group);
		return -EACCES;
	}

	image_dev.this_device->of_node = pdev->dev.of_node;
	image_dev.this_device->platform_data = (void *)group;
	group->md = &image_dev;
	group->pdev = pdev;
	group->hw_info = (struct cam_hw_info *)
		of_device_get_match_data(&pdev->dev);
	if (!group->hw_info) {
		pr_err("fail to get hw_info\n");
		goto probe_pw_fail;
	}
	atomic_set(&group->camera_opened, 0);
	atomic_set(&group->runner_nr, 0);
	spin_lock_init(&group->module_lock);
	spin_lock_init(&group->rawproc_lock);

	pr_info("sprd img probe pdev name %s\n", pdev->name);
	pr_info("sprd dcam dev name %s\n", pdev->dev.init_name);
	ret = dcam_drv_dt_parse(pdev, group->hw_info, &group->dcam_count);
	if (ret) {
		pr_err("fail to parse dcam dts\n");
		goto probe_pw_fail;
	}

	pr_info("sprd isp dev name %s\n", pdev->dev.init_name);
	ret = isp_drv_dt_parse(pdev->dev.of_node, group->hw_info,
		&group->isp_count);
	if (ret) {
		pr_err("fail to parse isp dts\n");
		goto probe_pw_fail;
	}

	/* for get ta status
	 * group->ca_conn  = cam_trusty_connect();
	 */
	if (group->ca_conn)
		pr_info("cam ca-ta unconnect\n");

	group->debugger.hw = group->hw_info;
	ret = cam_debugger_init(&group->debugger);
	if (ret)
		pr_err("fail to init cam debugfs\n");

	return 0;

probe_pw_fail:
	misc_deregister(&image_dev);
	kfree(group);

	return ret;
}

static int camcore_remove(struct platform_device *pdev)
{
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	group = image_dev.this_device->platform_data;
	if (group) {
		if (group->ca_conn)
			cam_trusty_disconnect();
		kfree(group);
		image_dev.this_device->platform_data = NULL;
	}
	misc_deregister(&image_dev);

	return 0;
}

static const struct of_device_id sprd_cam_of_match[] = {
	#if defined (PROJ_SHARKL3)
	{ .compatible = "sprd,sharkl3-cam", .data = &sharkl3_hw_info},
	#elif defined (PROJ_SHARKL5)
	{ .compatible = "sprd,sharkl5-cam", .data = &sharkl5_hw_info},
	#elif defined (PROJ_SHARKL5PRO)
	{ .compatible = "sprd,sharkl5pro-cam", .data = &sharkl5pro_hw_info},
	#elif defined (PROJ_QOGIRL6)
	{ .compatible = "sprd,qogirl6-cam", .data = &qogirl6_hw_info},
	#endif
	{ },
};

static struct platform_driver sprd_img_driver = {
	.probe = camcore_probe,
	.remove = camcore_remove,
	.driver = {
		.name = IMG_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_cam_of_match),
	},
};

module_platform_driver(sprd_img_driver);

MODULE_DESCRIPTION("SPRD CAM Driver");
MODULE_AUTHOR("Multimedia_Camera@SPRD");
MODULE_LICENSE("GPL");
