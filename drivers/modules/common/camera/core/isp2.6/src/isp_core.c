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

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/sprd_ion.h>
#include <linux/kthread.h>

#include "isp_hw.h"
#include "sprd_img.h"
#include <video/sprd_mmsys_pw_domain.h>
#include <sprd_mm.h>

#include "cam_trusty.h"
#include "cam_queue.h"
#include "cam_debugger.h"
#include "isp_int.h"
#include "isp_reg.h"
#include "isp_core.h"
#include "isp_path.h"
#include "isp_slice.h"
#include "isp_cfg.h"
#include "dcam_core.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

unsigned long *isp_cfg_poll_addr[ISP_CONTEXT_SW_NUM];

static DEFINE_MUTEX(isp_pipe_dev_mutex);

struct isp_pipe_dev *s_isp_dev;
uint32_t s_dbg_linebuf_len = ISP_LINE_BUFFER_W;
extern int s_dbg_work_mode;

struct offline_tmp_param {
	int valid_out_frame;
	int hw_ctx_id;
	uint32_t multi_slice;
	uint32_t target_fid;
	struct isp_stream_ctrl *stream;
};

static void ispcore_offline_pararm_free(void *param)
{
	struct isp_offline_param *cur, *prev;

	cur = (struct isp_offline_param *)param;
	while (cur) {
		prev = (struct isp_offline_param *)cur->prev;
		pr_info("free %p\n", cur);
		kfree(cur);
		cur = prev;
	}
}

static void ispcore_frame_unmap(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}
	frame = (struct camera_frame *)param;
	if (frame->buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&frame->buf);
}

static void ispcore_out_frame_ret(void *param)
{
	struct camera_frame *frame;
	struct isp_sw_context *pctx;
	struct isp_path_desc *path;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	path = (struct isp_path_desc *)frame->priv_data;
	if (!path) {
		pr_err("fail to get out_frame path.\n");
		return;
	}

	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);

	if (frame->is_reserved)
		cam_queue_enqueue(&path->reserved_buf_queue, &frame->list);
	else {
		pctx = path->attach_ctx;
		if (frame->buf.mapping_state & CAM_BUF_MAPPING_DEV)
			cam_buf_iommu_unmap(&frame->buf);
		pctx->isp_cb_func(ISP_CB_RET_DST_BUF, frame, pctx->cb_priv_data);
	}
}

static void ispcore_src_frame_ret(void *param)
{
	struct camera_frame *frame;
	struct isp_sw_context *pctx;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	pctx = (struct isp_sw_context *)frame->priv_data;
	if (!pctx) {
		pr_err("fail to get src_frame pctx.\n");
		return;
	}

	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);
	ispcore_offline_pararm_free(frame->param_data);
	frame->param_data = NULL;
	if (frame->buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&frame->buf);
	pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, frame, pctx->cb_priv_data);
}

static void ispcore_reserved_buf_destroy(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	if (unlikely(frame->is_reserved == 0)) {
		pr_err("fail to get frame reserved buffer.\n");
		return;
	}
	/* is_reserved:
	 *  1:  basic mapping reserved buffer;
	 *  2:  copy of reserved buffer.
	 */
	if (frame->is_reserved == 1) {
		cam_buf_iommu_unmap(&frame->buf);
		cam_buf_ionbuf_put(&frame->buf);
	}
	cam_queue_empty_frame_put(frame);
}

static void ispcore_statis_buf_destroy(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cam_queue_empty_frame_put(frame);
}

static void ispcore_sw_context_clear(void *param)
{
	struct isp_sw_context *ctx;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	ctx = (struct isp_sw_context *)param;
	atomic_dec(&g_mem_dbg->isp_sw_context_cnt);
	vfree(ctx);
}

static int ispcore_blkparam_adapt(struct isp_sw_context *pctx)
{
	uint32_t new_width, old_width;
	uint32_t new_height, old_height;
	uint32_t crop_start_x, crop_start_y;
	uint32_t crop_end_x, crop_end_y;
	struct img_trim *src_trim;
	struct img_size *dst = &pctx->uinfo.original.dst_size;

	if (pctx->uinfo.original.src_size.w > 0) {
		/* for input scaled image */
		src_trim = &pctx->uinfo.original.src_trim;
		new_width = dst->w;
		new_height = dst->h;
		old_width = src_trim->size_x;
		old_height = src_trim->size_y;
	} else {
		src_trim = &pctx->pipe_src.crop;
		old_width = src_trim->size_x;
		old_height = src_trim->size_y;
		new_width = old_width;
		new_height = old_height;
	}
	crop_start_x = src_trim->start_x;
	crop_start_y = src_trim->start_y;
	crop_end_x = src_trim->start_x + src_trim->size_x - 1;
	crop_end_y = src_trim->start_y + src_trim->size_y - 1;

	pr_debug("crop %d %d %d %d, size(%d %d) => (%d %d)\n",
		crop_start_x, crop_start_y, crop_end_x, crop_end_y,
		old_width, old_height, new_width, new_height);

	isp_k_update_nlm(pctx->ctx_id, &pctx->isp_k_param,
		new_width, old_width, new_height, old_height);
	isp_k_update_ynr(pctx->ctx_id, &pctx->isp_k_param,
		new_width, old_width, new_height, old_height);
	isp_k_update_imbalance(pctx->ctx_id, &pctx->isp_k_param,
		new_width, old_width, new_height, old_height);

	if (pctx->uinfo.mode_3dnr != MODE_3DNR_OFF)
		isp_k_update_3dnr(pctx->ctx_id, &pctx->isp_k_param,
			 new_width, old_width, new_height, old_height);

	return 0;
}

static int ispcore_hist_roi_update(struct isp_sw_context *pctx)
{
	int ret = 0;
	struct isp_dev_hist2_info hist2_info;
	struct isp_hw_hist_roi hist_arg;
	struct isp_hw_fetch_info *fetch = &pctx->pipe_info.fetch;

	pr_debug("sw %d, hist_roi w[%d] h[%d]\n",
		pctx->ctx_id, fetch->in_trim.size_x, fetch->in_trim.size_y);

	hist2_info.hist_roi.start_x = 0;
	hist2_info.hist_roi.start_y = 0;

	hist2_info.hist_roi.end_x = fetch->in_trim.size_x - 1;
	hist2_info.hist_roi.end_y = fetch->in_trim.size_y - 1;

	hist_arg.hist_roi = &hist2_info.hist_roi;
	hist_arg.ctx_id = pctx->ctx_id;
	pctx->hw->isp_ioctl(pctx->hw, ISP_HW_CFG_UPDATE_HIST_ROI, &hist_arg);

	return ret;
}

static int ispcore_3dnr_frame_process(struct isp_sw_context *pctx,
		struct camera_frame *pframe)
{
	struct isp_3dnr_ctx_desc *nr3_ctx = NULL;
	struct dcam_frame_synchronizer *fsync = NULL;
	struct isp_uinfo *pipe_src = NULL;

	fsync = (struct dcam_frame_synchronizer *)pframe->sync_data;
	if (fsync) {
		pr_debug("id %u, valid %d, x %d, y %d, w %u, h %u\n",
			 fsync->index, fsync->nr3_me.valid,
			 fsync->nr3_me.mv_x, fsync->nr3_me.mv_y,
			 fsync->nr3_me.src_width, fsync->nr3_me.src_height);
	}
	nr3_ctx = &pctx->nr3_ctx;
	pipe_src = &pctx->pipe_src;

	if (pipe_src->nr3_fbc_fbd) {
		nr3_ctx->nr3_store.st_bypass = 1;
		nr3_ctx->nr3_fbc_store.bypass = 0;
		nr3_ctx->mem_ctrl.nr3_ft_path_sel = 1;
	} else {
		nr3_ctx->nr3_store.st_bypass = 0;
		nr3_ctx->nr3_fbc_store.bypass = 1;
		nr3_ctx->mem_ctrl.nr3_ft_path_sel = 0;
	}

	/*  Check Zoom or not */
	if ((pipe_src->crop.size_x != pctx->nr3_ctx.width) ||
		(pipe_src->crop.size_y != pctx->nr3_ctx.height)) {
		pr_debug("isp %d frame size changed, reset 3dnr blending\n",
			pctx->ctx_id);
		pctx->nr3_ctx.blending_cnt = 0;
	}

	nr3_ctx->width  = pipe_src->crop.size_x;
	nr3_ctx->height = pipe_src->crop.size_y;

	pr_debug("isp %d nr3_type %d input.w[%d], input.h[%d], trim.w[%d], trim.h[%d]\n",
		pctx->ctx_id, pctx->uinfo.mode_3dnr,
		pipe_src->src.w, pipe_src->src.h,
		pipe_src->crop.size_x, pipe_src->crop.size_y);

	switch (pctx->uinfo.mode_3dnr) {
	case MODE_3DNR_PRE:
		nr3_ctx->type = NR3_FUNC_PRE;

		if (fsync && fsync->nr3_me.valid) {
			nr3_ctx->mv.mv_x = fsync->nr3_me.mv_x;
			nr3_ctx->mv.mv_y = fsync->nr3_me.mv_y;
			nr3_ctx->mvinfo = &fsync->nr3_me;

			isp_3dnr_conversion_mv(nr3_ctx);
		} else {
			pr_err("fail to get binning path mv, set default 0\n");
			nr3_ctx->mv.mv_x = 0;
			nr3_ctx->mv.mv_y = 0;
			nr3_ctx->mvinfo = NULL;
		}

		isp_3dnr_config_gen(nr3_ctx);
		isp_3dnr_config_param(nr3_ctx,
			&pctx->isp_k_param,
			pctx->ctx_id,
			NR3_FUNC_PRE);

		pr_debug("MODE_3DNR_PRE frame type: %d from dcam binning path mv[%d, %d]!\n.",
			 pframe->channel_id,
			 nr3_ctx->mv.mv_x,
			 nr3_ctx->mv.mv_y);
		break;

	case MODE_3DNR_CAP:
		nr3_ctx->type = NR3_FUNC_CAP;

		if (fsync && fsync->nr3_me.valid) {
			nr3_ctx->mv.mv_x = fsync->nr3_me.mv_x;
			nr3_ctx->mv.mv_y = fsync->nr3_me.mv_y;
			if (pipe_src->src.w != pipe_src->crop.size_x ||
				pipe_src->src.h != pipe_src->crop.size_y) {
				nr3_ctx->mvinfo = &fsync->nr3_me;
				isp_3dnr_conversion_mv(nr3_ctx);
			}
		} else {
			pr_err("fail to get full path mv, set default 0\n");
			nr3_ctx->mv.mv_x = 0;
			nr3_ctx->mv.mv_y = 0;
		}
		if (nr3_ctx->mode == MODE_3DNR_OFF)
			return 0;

		isp_3dnr_config_gen(nr3_ctx);
		isp_3dnr_config_param(nr3_ctx,
			&pctx->isp_k_param,
			pctx->ctx_id,
			NR3_FUNC_CAP);

		pr_debug("MODE_3DNR_CAP frame type: %d from dcam binning path mv[%d, %d]!\n.",
			pframe->channel_id,
			nr3_ctx->mv.mv_x,
			nr3_ctx->mv.mv_y);
		break;

	case MODE_3DNR_OFF:
		/* default: bypass 3dnr */
		nr3_ctx->type = NR3_FUNC_OFF;
		isp_3dnr_bypass_config(pctx->ctx_id);
		pr_debug("ispcore_offline_frame_start MODE_3DNR_OFF\n");
		break;
	default:
		/* default: bypass 3dnr */
		nr3_ctx->type = NR3_FUNC_OFF;
		isp_3dnr_bypass_config(pctx->ctx_id);
		pr_debug("ispcore_offline_frame_start default\n");
		break;
	}

	if (fsync)
		dcam_core_dcam_if_release_sync(fsync, pframe);

	return 0;
}

static int ispcore_ltm_frame_previous_process(struct isp_sw_context *pctx,
		struct camera_frame *pframe)
{
	if (!pctx || !pframe) {
		pr_err("fail to get valid parameter pctx %p pframe %p\n",
			pctx, pframe);
		return -EINVAL;
	}

	/*
	 * Only preview path care of frame size changed
	 * Because capture path, USING hist from preview path
	 */
	if (pctx->pipe_src.mode_ltm != MODE_LTM_PRE)
		return 0;

	/*  Check Zoom or not */
	if ((pctx->pipe_src.crop.size_x != pctx->ltm_ctx.frame_width) ||
		(pctx->pipe_src.crop.size_y != pctx->ltm_ctx.frame_height) ||
		((pframe->fid - pctx->ltm_ctx.fid) != 1)||
		(pframe->fid == 0)){
		pr_debug("frame size changed or frame id not series , bypass ltm map\n");

		/* 1. hists from preview path always on
		 * 2. map will be off one time in preview case
		 */
		if (pctx->pipe_src.ltm_rgb)
			pctx->ltm_ctx.map[LTM_RGB].bypass = 1;
		if (pctx->pipe_src.ltm_yuv)
			pctx->ltm_ctx.map[LTM_YUV].bypass = 1;
	} else {
		if (pctx->pipe_src.ltm_rgb)
			pctx->ltm_ctx.map[LTM_RGB].bypass = 0;
		if (pctx->pipe_src.ltm_yuv)
			pctx->ltm_ctx.map[LTM_YUV].bypass = 0;
	}

	return 0;
}

static int ispcore_ltm_frame_process(struct isp_sw_context *pctx,
				struct camera_frame *pframe)
{
	int ret = 0;
	struct isp_uinfo *pipe_src = NULL;

	pipe_src = &pctx->pipe_src;

	pctx->ltm_ctx.type = pipe_src->mode_ltm;
	pctx->ltm_ctx.fid = pframe->fid;
	pctx->ltm_ctx.frame_width  = pipe_src->crop.size_x;
	pctx->ltm_ctx.frame_height = pipe_src->crop.size_y;
	pctx->ltm_ctx.isp_pipe_ctx_id = pctx->ctx_id;

	pr_debug("LTM: type %d rgb %d yuv %d ctx id%d\n",
		pctx->ltm_ctx.type, pipe_src->ltm_rgb,
		pipe_src->ltm_yuv, pctx->ctx_id);

	/* pre & cap */
	if (pipe_src->ltm_rgb)
		ret = isp_ltm_frame_config_gen(&pctx->ltm_ctx, LTM_RGB,
			(struct isp_ltm_info *)&pctx->isp_k_param.ltm_rgb_info);
	if (ret == -1) {
		pipe_src->mode_ltm = MODE_LTM_OFF;
		pr_err("fail to rgb LTM cfg frame, DISABLE\n");
	}

	if (pipe_src->ltm_yuv)
		ret = isp_ltm_frame_config_gen(&pctx->ltm_ctx, LTM_YUV,
			(struct isp_ltm_info *)&pctx->isp_k_param.ltm_yuv_info);
	if (ret == -1) {
		pipe_src->mode_ltm = MODE_LTM_OFF;
		pr_err("fail to yuv LTM cfg frame, DISABLE\n");
	}

	pr_debug("type[%d], fid[%d], frame_width[%d], frame_height[%d], isp_pipe_ctx_id[%d]\n",
		pctx->ltm_ctx.type,
		pctx->ltm_ctx.fid,
		pctx->ltm_ctx.frame_width,
		pctx->ltm_ctx.frame_height,
		pctx->ltm_ctx.isp_pipe_ctx_id);

	pr_debug("frame_height_stat[%d], frame_width_stat[%d]",
		pctx->ltm_ctx.frame_height_stat,
		pctx->ltm_ctx.frame_width_stat);

	return ret;
}

static int ispcore_fmcu_slw_queue_set(
		struct isp_fmcu_ctx_desc *fmcu,
		struct isp_sw_context *pctx)
{
	int ret = 0, i;
	uint32_t frame_id;
	struct isp_path_desc *path;
	struct camera_frame *pframe = NULL;
	struct camera_frame *out_frame = NULL;
	struct isp_hw_slw_fmcu_cmds slw;

	if (!fmcu)
		return -EINVAL;

	pframe = cam_queue_dequeue(&pctx->in_queue, struct camera_frame, list);
	if (pframe == NULL) {
		pr_err("fail to get frame from input queue. cxt:%d\n", pctx->ctx_id);
		return -EINVAL;
	}

	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. cxt %d\n", pctx->ctx_id);
		ret = -EINVAL;
	}

	ret = cam_queue_enqueue(&pctx->proc_queue, &pframe->list);
	if (ret) {
		pr_err("fail to input frame queue, timeout.\n");
		ret = -EINVAL;
	}

	pframe->width = pctx->pipe_src.src.w;
	pframe->height = pctx->pipe_src.src.h;
	frame_id = pframe->fid;
	isp_path_fetch_frm_set(pctx, pframe);

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		if (i == ISP_SPATH_VID)
			out_frame = cam_queue_dequeue(&path->out_buf_queue,
				struct camera_frame, list);
		if (out_frame == NULL)
			out_frame = cam_queue_dequeue(&path->reserved_buf_queue,
				struct camera_frame, list);

		if (out_frame == NULL) {
			pr_debug("fail to get available output buffer.\n");
			return -EINVAL;
		}

		if (out_frame->is_reserved == 0) {
			ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
			pr_debug("map output buffer %08x\n", (uint32_t)out_frame->buf.iova[0]);
			if (ret) {
				cam_queue_enqueue(&path->out_buf_queue, &out_frame->list);
				out_frame = NULL;
				pr_err("fail to map isp iommu buf.\n");
				return -EINVAL;
			}
		}

		out_frame->fid = frame_id;
		out_frame->sensor_time = pframe->sensor_time;
		out_frame->boot_sensor_time = pframe->boot_sensor_time;

		pr_debug("isp output buf, iova 0x%x, phy: 0x%x\n",
				(uint32_t)out_frame->buf.iova[0],
				(uint32_t)out_frame->buf.addr_k[0]);
		isp_path_store_frm_set(path, out_frame);
		if ((i < AFBC_PATH_NUM) && pctx->pipe_src.path_info[i].store_fbc)
			isp_path_afbc_store_frm_set(path, out_frame);

		ret = cam_queue_enqueue(&path->result_queue, &out_frame->list);
		if (ret) {
			if (out_frame->is_reserved)
				cam_queue_enqueue(&path->reserved_buf_queue,
						&out_frame->list);
			else {
				cam_buf_iommu_unmap(&out_frame->buf);
				cam_queue_enqueue(&path->out_buf_queue,
						&out_frame->list);
			}
			return -EINVAL;
		}
		atomic_inc(&path->store_cnt);
		slw.store[i] = pctx->pipe_info.store[i].store;
		if ((i < AFBC_PATH_NUM) && pctx->pipe_src.path_info[i].store_fbc)
			slw.afbc_store[i] = pctx->pipe_info.afbc[i].afbc_store;
	}

	slw.fmcu_handle = fmcu;
	slw.ctx_id = pctx->ctx_id;
	slw.fetchaddr = pctx->pipe_info.fetch.addr;
	slw.isp_path = pctx->isp_path;
	ret = path->hw->isp_ioctl(path->hw, ISP_HW_CFG_SLW_FMCU_CMDS, &slw);

	pr_debug("fmcu slw queue done!");
	return ret;
}

static void ispcore_debug_dump_check(
		struct isp_sw_context *pctx,
		struct camera_frame *proc_frame)
{
	int size;
	struct camera_frame *frame = NULL;
	struct debug_base_info *base_info;
	void *pm_data;

	pctx->isp_cb_func(ISP_CB_GET_PMBUF,
		(void *)&frame, pctx->cb_priv_data);
	if (frame == NULL) {
		pr_debug("no pmbuf ctx %d fid %d,  prop %d\n",
			pctx->ctx_id, proc_frame->fid, proc_frame->irq_property);
		return;
	}

	base_info = (struct debug_base_info *)frame->buf.addr_k[0];
	if (base_info == NULL) {
		cam_queue_empty_frame_put(frame);
		return;
	}
	base_info->cam_id = pctx->attach_cam_id;
	base_info->dcam_cid = -1;
	base_info->isp_cid = pctx->ctx_id;

	base_info->scene_id = PM_SCENE_CAP;
	if (proc_frame->channel_id == CAM_CH_PRE)
		base_info->scene_id = PM_SCENE_PRE;
	if (proc_frame->irq_property == CAM_FRAME_FDRL)
		base_info->scene_id = PM_SCENE_FDRL;
	if (proc_frame->irq_property == CAM_FRAME_FDRH)
		base_info->scene_id = PM_SCENE_FDRH;

	base_info->frame_id = proc_frame->fid;
	base_info->sec = proc_frame->sensor_time.tv_sec;
	base_info->usec = proc_frame->sensor_time.tv_usec;
	frame->sensor_time = proc_frame->sensor_time;
	frame->fid = proc_frame->fid;

	pm_data = (void *)(base_info + 1);
	size = isp_k_dump_pm((void *)pm_data,
		(void *)&pctx->isp_k_param);
	if (size >= 0)
		base_info->size = (int32_t)size;
	else
		base_info->size = 0;

	pr_debug("cxt %d scene %d  fid %d  dsize %d\n",
		base_info->isp_cid, base_info->scene_id,
		base_info->frame_id, base_info->size);

	pctx->isp_cb_func(ISP_CB_STATIS_DONE,
		frame, pctx->cb_priv_data);
}

static uint32_t ispcore_fid_across_context_get(struct isp_pipe_dev *dev, enum camera_id cam_id)
{
	struct isp_sw_context *ctx;
	struct isp_path_desc *path;
	struct camera_frame *frame;
	uint32_t target_fid;
	int ctx_id, path_id;

	if (!dev)
		return CAMERA_RESERVE_FRAME_NUM;

	target_fid = CAMERA_RESERVE_FRAME_NUM;
	for (ctx_id = 0; ctx_id < ISP_CONTEXT_SW_NUM; ctx_id++) {
		ctx = dev->sw_ctx[ctx_id];
		if (!ctx || atomic_read(&ctx->user_cnt) < 1 || (ctx->attach_cam_id != cam_id))
			continue;

		for (path_id = 0; path_id < ISP_SPATH_NUM; path_id++) {
			path = &ctx->isp_path[path_id];
			if (!path || atomic_read(&path->user_cnt) < 1
				|| !ctx->uinfo.path_info[path_id].uframe_sync)
				continue;

			frame = cam_queue_dequeue_peek(&path->out_buf_queue,
				struct camera_frame, list);
			if (!frame)
				continue;

			target_fid = min(target_fid, frame->user_fid);
			pr_debug("ISP%d path%d user_fid %u\n",
				 ctx_id, path_id, frame->user_fid);
		}
	}

	pr_debug("target_fid %u, cam_id = %d\n", target_fid, cam_id);

	return target_fid;
}

static bool ispcore_fid_check(struct camera_frame *frame, void *data)
{
	uint32_t target_fid;

	if (!frame || !data)
		return false;

	target_fid = *(uint32_t *)data;

	pr_debug("target_fid = %d frame->user_fid = %d\n", target_fid, frame->user_fid);
	return frame->user_fid == CAMERA_RESERVE_FRAME_NUM
		|| frame->user_fid == target_fid;
}

static int ispcore_sw_context_get(struct isp_pipe_dev *dev)
{
	int cxt_id = -1;
	struct isp_sw_context *ctx = NULL;
	struct isp_sw_context *ctx_tmp = NULL;
	struct camera_queue *q = NULL;

	if (!dev) {
		pr_err("fail to get valid input ptr, %p\n", dev);
		return -EFAULT;
	}

	q = &dev->sw_ctx_q;
	if (q->state == CAM_Q_CLEAR) {
		pr_warn("q is clear\n");
		goto exit;
	}

	if (!list_empty(&q->head) && (q->cnt)) {
		list_for_each_entry(ctx_tmp, &q->head, list) {
			if (ctx_tmp && (atomic_read(&ctx_tmp->user_cnt) == 0)) {
				ctx = ctx_tmp;
				atomic_set(&ctx->user_cnt, 1);
				break;
			}
		}
	}

	if (!ctx) {
		ctx = vzalloc(sizeof(*ctx));
		if (!ctx) {
			pr_err("fail to alloc isp sw context\n");
			goto exit;
		}
		atomic_inc(&g_mem_dbg->isp_sw_context_cnt);
		if (q->cnt >= q->max) {
			pr_warn("q full, cnt %d, max %d\n", q->cnt, q->max);
			goto queue_full;;
		}
		atomic_set(&ctx->user_cnt, 1);
		ctx->ctx_id = q->cnt;
		ctx->dev = dev;
		ctx->hw = dev->isp_hw;
		q->cnt++;
		list_add_tail(&ctx->list, &q->head);
	}
	cxt_id = ctx->ctx_id;
	dev->sw_ctx[cxt_id] = ctx;

	return cxt_id;

queue_full:
	atomic_dec(&g_mem_dbg->isp_sw_context_cnt);
	vfree(ctx);
exit:
	return cxt_id;
}

int isp_core_hw_context_id_get(struct isp_sw_context *pctx)
{
	int i;
	int hw_ctx_id = -1;
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw;

	dev = pctx->dev;

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];

		if ((pctx->ctx_id == pctx_hw->sw_ctx_id)
			&& (pctx == pctx_hw->pctx)) {
			hw_ctx_id = pctx_hw->hw_ctx_id;
			break;
		}
	}
	pr_debug("get hw %d\n", hw_ctx_id);

	return hw_ctx_id;
}

int isp_core_sw_context_id_get(enum isp_context_hw_id hw_ctx_id, struct isp_pipe_dev *dev)
{
	int sw_id = -1;
	struct isp_hw_context *pctx_hw;

	if (hw_ctx_id < ISP_CONTEXT_HW_NUM) {
		pctx_hw = &dev->hw_ctx[hw_ctx_id];
		sw_id = pctx_hw->sw_ctx_id;
		if (sw_id >= ISP_CONTEXT_P0 &&
			sw_id < ISP_CONTEXT_SW_NUM &&
			pctx_hw->pctx == dev->sw_ctx[sw_id]) {
				pr_debug("get sw %d\n", sw_id);
				return sw_id;
			}
	}

	return -1;
}

int isp_core_context_bind(struct isp_sw_context *pctx, int fmcu_need)
{
	int i = 0, m = 0, loop;
	int hw_ctx_id = -1;
	unsigned long flag = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw;

	dev = pctx->dev;
	spin_lock_irqsave(&dev->ctx_lock, flag);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if (pctx->ctx_id == pctx_hw->sw_ctx_id
			&& pctx_hw->pctx == pctx) {
			atomic_inc(&pctx_hw->user_cnt);
			pr_debug("sw %d & hw %d already binding, cnt=%d\n",
				pctx->ctx_id, i, atomic_read(&pctx_hw->user_cnt));
			spin_unlock_irqrestore(&dev->ctx_lock, flag);
			return 0;
		}
	}

	loop = (fmcu_need & FMCU_IS_MUST) ? 1 : 2;
	for (m = 0; m < loop; m++) {
		for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
			pctx_hw = &dev->hw_ctx[i];
			if (m == 0) {
				/* first pass we just select fmcu/no-fmcu context */
				if ((!fmcu_need && pctx_hw->fmcu_handle) ||
					(fmcu_need && !pctx_hw->fmcu_handle))
					continue;
			}

			if (atomic_inc_return(&pctx_hw->user_cnt) == 1) {
				hw_ctx_id = pctx_hw->hw_ctx_id;
				goto exit;
			}
			atomic_dec(&pctx_hw->user_cnt);
		}
	}

	/* force fmcu used, we will retry */
	if (fmcu_need & FMCU_IS_MUST)
		goto exit;

exit:
	spin_unlock_irqrestore(&dev->ctx_lock, flag);

	if (hw_ctx_id == -1)
		return -1;

	pctx_hw->pctx = pctx;
	pctx_hw->sw_ctx_id = pctx->ctx_id;
	pr_debug("sw %d, hw %d %d, fmcu_need %d ptr 0x%lx\n",
		pctx->ctx_id, hw_ctx_id, pctx_hw->hw_ctx_id,
		fmcu_need, (unsigned long)pctx_hw->fmcu_handle);

	return 0;
}

int isp_core_context_unbind(struct isp_sw_context *pctx)
{
	int i, cnt;
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw;
	unsigned long flag = 0;

	if (isp_core_hw_context_id_get(pctx) < 0) {
		pr_err("fail to binding sw ctx %d to any hw ctx\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	spin_lock_irqsave(&dev->ctx_lock, flag);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if ((pctx->ctx_id != pctx_hw->sw_ctx_id) ||
			(pctx != pctx_hw->pctx))
			continue;

		if (atomic_dec_return(&pctx_hw->user_cnt) == 0) {
			pr_debug("sw_id=%d, hw_id=%d unbind success\n",
				pctx->ctx_id, pctx_hw->hw_ctx_id);
			pctx_hw->pctx = NULL;
			pctx_hw->sw_ctx_id = -1;
			goto exit;
		}

		cnt = atomic_read(&pctx_hw->user_cnt);
		if (cnt >= 1) {
			pr_debug("sw id=%d, hw_id=%d, cnt=%d\n",
				pctx->ctx_id, pctx_hw->hw_ctx_id, cnt);
		} else {
			pr_debug("should not be here: sw id=%d, hw id=%d, cnt=%d\n",
				pctx->ctx_id, pctx_hw->hw_ctx_id, cnt);
			spin_unlock_irqrestore(&dev->ctx_lock, flag);
			return -EINVAL;
		}
	}

exit:
	spin_unlock_irqrestore(&dev->ctx_lock, flag);
	return 0;
}

static uint32_t ispcore_slice_needed(struct isp_sw_context *pctx)
{
	int i;
	struct isp_path_uinfo *path_info = NULL;
	struct isp_path_desc *path = NULL;

	if (pctx->uinfo.crop.size_x > g_camctrl.isp_linebuf_len)
		return 1;

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		path_info = &pctx->uinfo.path_info[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		if (path_info->dst.w > g_camctrl.isp_linebuf_len)
			return 1;
	}
	return 0;
}

static int ispcore_slice_ctx_init(struct isp_sw_context *pctx, uint32_t *multi_slice)
{
	int ret = 0;
	int j;
	uint32_t val = 0;
	struct isp_path_desc *path;
	struct slice_cfg_input slc_cfg_in;
	struct isp_hw_nlm_ynr radius_adapt;

	*multi_slice = 0;

	if  ((ispcore_slice_needed(pctx) == 0) && (pctx->uinfo.enable_slowmotion == 0)) {
		pr_debug("sw %d don't need to slice , slowmotion %d\n",
			pctx->ctx_id, pctx->uinfo.enable_slowmotion);
		return 0;
	}

	if (pctx->slice_ctx == NULL) {
		pctx->slice_ctx = isp_slice_ctx_get();
		if (IS_ERR_OR_NULL(pctx->slice_ctx)) {
			pr_err("fail to get memory for slice_ctx.\n");
			pctx->slice_ctx = NULL;
			ret = -ENOMEM;
			goto exit;
		}
	}

	memset(&slc_cfg_in, 0, sizeof(struct slice_cfg_input));
	slc_cfg_in.frame_in_size.w = pctx->pipe_src.crop.size_x;
	slc_cfg_in.frame_in_size.h = pctx->pipe_src.crop.size_y;
	slc_cfg_in.frame_fetch = &pctx->pipe_info.fetch;
	slc_cfg_in.frame_fbd_raw = &pctx->pipe_info.fetch_fbd;
	slc_cfg_in.thumb_scaler = &pctx->pipe_info.thumb_scaler;
	for (j = 0; j < ISP_SPATH_NUM; j++) {
		path = &pctx->isp_path[j];
		if (atomic_read(&path->user_cnt) <= 0)
			continue;
		slc_cfg_in.frame_out_size[j] = &pctx->pipe_info.store[j].store.size;
		slc_cfg_in.frame_store[j] = &pctx->pipe_info.store[j].store;
		slc_cfg_in.frame_scaler[j] = &pctx->pipe_info.scaler[j].scaler;
		slc_cfg_in.frame_deci[j] = &pctx->pipe_info.scaler[j].deci;
		slc_cfg_in.frame_trim0[j] = &pctx->pipe_info.scaler[j].in_trim;
		slc_cfg_in.frame_trim1[j] = &pctx->pipe_info.scaler[j].out_trim;
		if ((j < AFBC_PATH_NUM) && pctx->pipe_src.path_info[j].store_fbc)
			slc_cfg_in.frame_afbc_store[j] = &pctx->pipe_info.afbc[j].afbc_store;
	}

	radius_adapt.val = val;
	radius_adapt.ctx_id = pctx->ctx_id;
	radius_adapt.slc_cfg_in = &slc_cfg_in;
	path->hw->isp_ioctl(path->hw, ISP_HW_CFG_GET_NLM_YNR, &radius_adapt);
	isp_slice_base_cfg(&slc_cfg_in, pctx->slice_ctx, &pctx->valid_slc_num);

	pr_debug("sw %d valid_slc_num %d\n", pctx->ctx_id, pctx->valid_slc_num);
	if (pctx->valid_slc_num > 1)
		*multi_slice = 1;
exit:
	return ret;
}

static int ispcore_slices_proc(struct isp_sw_context *pctx,
		struct camera_frame *pframe)
{
	int ret = 0;
	int hw_ctx_id = -1, first_slice = 1;
	uint32_t slice_id, cnt = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_cfg_ctx_desc *cfg_desc;
	struct cam_hw_info *hw = NULL;
	struct isp_hw_yuv_block_ctrl blk_ctrl;

	pr_debug("enter. need_slice %d\n", pctx->multi_slice);
	hw_ctx_id = isp_core_hw_context_id_get(pctx);
	if (hw_ctx_id < 0) {
		pr_err("fail to get valid hw for ctx %d\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
	pctx->is_last_slice = 0;
	hw = pctx->hw;
	for (slice_id = 0; slice_id < SLICE_NUM_MAX; slice_id++) {
		if (pctx->is_last_slice == 1)
			break;

		ret = isp_slice_update(pctx, pctx->ctx_id, slice_id);
		if (ret < 0)
			continue;

		cnt++;
		if (cnt == pctx->valid_slc_num)
			pctx->is_last_slice = 1;
		pr_debug("slice %d, valid %d, last %d\n", slice_id,
			pctx->valid_slc_num, pctx->is_last_slice);

		pctx->started = 1;
		mutex_lock(&pctx->blkpm_lock);
		blk_ctrl.idx = pctx->ctx_id;
		blk_ctrl.blk_param = &pctx->isp_k_param;
		if (pctx->pipe_src.in_fmt == IMG_PIX_FMT_NV21)
			blk_ctrl.type = ISP_YUV_BLOCK_DISABLE;
		else
			blk_ctrl.type = ISP_YUV_BLOCK_CFG;
		hw->isp_ioctl(hw, ISP_HW_CFG_YUV_BLOCK_CTRL_TYPE, &blk_ctrl);
		if (first_slice) {
			first_slice = 0;
			ispcore_debug_dump_check(pctx, pframe);
			ret = cfg_desc->ops->hw_cfg(
				cfg_desc, pctx->ctx_id, hw_ctx_id, 0);
		} else {
			/* todo - update slice registers only */
			ret = cfg_desc->ops->hw_cfg(
				cfg_desc, pctx->ctx_id, hw_ctx_id, 0);
		}
		mutex_unlock(&pctx->blkpm_lock);

		ret = cfg_desc->hw->isp_ioctl(cfg_desc->hw,
					ISP_HW_CFG_START_ISP, &hw_ctx_id);

		ret = wait_for_completion_interruptible_timeout(
					&pctx->slice_done,
					ISP_CONTEXT_TIMEOUT);
		if (ret == ERESTARTSYS) {
			pr_err("fail to interrupt, when isp wait\n");
			ret = -EFAULT;
			goto exit;
		} else if (ret == 0) {
			pr_err("fail to wait isp context %d, timeout.\n", pctx->ctx_id);
			ret = -EFAULT;
			goto exit;
		}
		pr_debug("slice %d done\n", slice_id);
	}
exit:
	return ret;
}

static void ispcore_sw_slice_prepare(struct isp_sw_context *pctx,
		struct camera_frame *pframe)
{
	struct isp_hw_fetch_info *fetch = &pctx->pipe_info.fetch;
	struct isp_store_info *store = NULL;
	uint32_t mipi_byte_info = 0;
	uint32_t mipi_word_info = 0;
	uint32_t start_col = 0;
	uint32_t end_col = pframe->slice_trim.size_x - 1;
	uint32_t mipi_word_num_start[16] = {
		0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {
		0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};
	struct img_size src;
	uint32_t slice_num = 0;
	uint32_t slice_no = 0;
	uint32_t first_slice = 0;
	struct isp_path_uinfo *path = NULL;

	pctx->sw_slice_num = pframe->sw_slice_num;
	pctx->sw_slice_no = pframe->sw_slice_no;
	slice_num = pctx->sw_slice_num;
	slice_no = pctx->sw_slice_no;
	first_slice = (slice_no != 0) ? 0 : 1;

	mipi_byte_info = start_col & 0xF;
	mipi_word_info = ((end_col + 1) >> 4) * 5
		+ mipi_word_num_end[(end_col + 1) & 0xF]
		- ((start_col + 1) >> 4) * 5
		- mipi_word_num_start[(start_col + 1) & 0xF] + 1;

	fetch->mipi_byte_rel_pos = 0;
	fetch->mipi_word_num = mipi_word_info;
	fetch->pitch.pitch_ch0 = cal_sprd_raw_pitch(pframe->slice_trim.size_x,
		pctx->pipe_src.pack_bits);
	fetch->pitch.pitch_ch1 = 0;
	fetch->pitch.pitch_ch2 = 0;

	path = &pctx->pipe_src.path_info[ISP_SPATH_CP];
	store = &pctx->pipe_info.store[ISP_SPATH_CP].store;

	if (first_slice) {
		fetch->in_trim.size_x = fetch->src.w = pframe->slice_trim.size_x;
		fetch->in_trim.size_y = fetch->src.h = pframe->slice_trim.size_y;

		pctx->pipe_src.crop.start_x = 0;
		pctx->pipe_src.crop.start_y = 0;
		src.w = pctx->pipe_src.crop.size_x = fetch->in_trim.size_x;
		src.h = pctx->pipe_src.crop.size_y = fetch->in_trim.size_y;
		pctx->pipe_src.src = src;

		store->pitch.pitch_ch0 = store->size.w;
		store->pitch.pitch_ch1 = store->size.w;
		if (slice_num > SLICE_NUM_MAX) {
			store->size.w = ALIGN(store->size.w / (slice_num / 2), 2);
			store->size.h = ALIGN(store->size.h / 2, 2);
		} else
			store->size.w = ALIGN(store->size.w / slice_num, 2);
	}

	if (slice_num > SLICE_NUM_MAX) {
		fetch->trim_off.addr_ch0 =
			(slice_no / (slice_num / 2)) * fetch->pitch.pitch_ch0 * fetch->src.h;
		store->slice_offset.addr_ch0 = path->dst.w * (slice_no % (slice_num / 2))
			+ (slice_no / (slice_num / 2)) * store->pitch.pitch_ch0 * store->size.h;
		store->slice_offset.addr_ch1 = path->dst.w * (slice_no % (slice_num / 2))
			+ (slice_no / (slice_num / 2)) * store->pitch.pitch_ch1 * store->size.h / 2;
		store->slice_offset.addr_ch2 = 0;
	} else {
		fetch->trim_off.addr_ch0 = 0;
		store->slice_offset.addr_ch0 = path->dst.w * slice_no;
		store->slice_offset.addr_ch1 = path->dst.w * slice_no;
		store->slice_offset.addr_ch2 = 0;
	}
}

static struct camera_frame *ispcore_path_out_frame_get(
				struct isp_sw_context *pctx,
				struct isp_path_desc *path,
				struct offline_tmp_param *tmp)
{
	int ret = 0, j = 0;
	uint32_t buf_type = 0;
	struct camera_frame *out_frame = NULL;

	if (!pctx || !path || !tmp) {
		pr_err("fail to get valid input pctx %p, path %p\n", pctx, path);
		return NULL;
	}

	if (tmp->stream) {
		buf_type = tmp->stream->buf_type[path->spath_id];
		switch (buf_type) {
		case ISP_STREAM_BUF_OUT:
			goto normal_out_put;
		case ISP_STREAM_BUF_RESERVED:
			out_frame = cam_queue_dequeue(&path->reserved_buf_queue,
				struct camera_frame, list);
			if (out_frame) {
				tmp->valid_out_frame = 1;
				pr_debug("reserved buffer %d %lx\n",
					out_frame->is_reserved, out_frame->buf.iova[0]);
			}
			break;
		case ISP_STREAM_BUF_POSTPROC:
			out_frame = pctx->postproc_buf;
			if (out_frame) {
				ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
				tmp->valid_out_frame = 1;
				if (ret)
					out_frame = NULL;
			}
			break;
		case ISP_STREAM_BUF_RESULT:
			out_frame = cam_queue_dequeue(&path->result_queue,
				struct camera_frame, list);
			if (out_frame)
				tmp->valid_out_frame = 1;
			break;
		default:
			pr_err("fail to support buf_type %d\n", tmp->stream->buf_type);
			break;
		}
		goto exit;
	}

normal_out_put:
	if (pctx->sw_slice_num && pctx->sw_slice_no != 0) {
		out_frame = cam_queue_dequeue(&path->result_queue,
					struct camera_frame, list);
	} else {
		if (pctx->uinfo.path_info[path->spath_id].uframe_sync
			&& tmp->target_fid != CAMERA_RESERVE_FRAME_NUM)
			out_frame = cam_queue_dequeue_if(&path->out_buf_queue,
				ispcore_fid_check, (void *)&tmp->target_fid);
		else
			out_frame = cam_queue_dequeue(&path->out_buf_queue,
				struct camera_frame, list);
	}

	if (out_frame)
		tmp->valid_out_frame = 1;
	else
		out_frame = cam_queue_dequeue(&path->reserved_buf_queue,
			struct camera_frame, list);

	if (out_frame != NULL) {
		if (out_frame->is_reserved == 0 &&
			(out_frame->buf.mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
			if (out_frame->buf.mfd[0] == path->reserved_buf_fd) {
				for (j = 0; j < 3; j++) {
					out_frame->buf.size[j] = path->reserve_buf_size[j];
					pr_debug("out_frame->buf.size[j] = %d, path->reserve_buf_size[j] = %d",
						(int)out_frame->buf.size[j], (int)path->reserve_buf_size[j]);
				}
			}
			ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
			pr_debug("map output buffer %08x\n", (uint32_t)out_frame->buf.iova[0]);
			if (ret) {
				cam_queue_enqueue(&path->out_buf_queue, &out_frame->list);
				out_frame = NULL;
				pr_err("fail to map isp iommu buf.\n");
			}
		}
	}

exit:
	return out_frame;
}

static int ispcore_offline_size_update(
		struct isp_sw_context *pctx,
		struct isp_offline_param *in_param)
{
	int ret = 0;
	int i;
	struct img_size *src_new = NULL;
	struct img_trim path_trim;
	struct isp_path_desc *path = NULL;
	struct isp_ctx_size_desc cfg;
	struct isp_path_uinfo *path_info = NULL;
	uint32_t update[ISP_SPATH_NUM] = {
		ISP_PATH0_TRIM, ISP_PATH1_TRIM, ISP_PATH2_TRIM};

	if(in_param->valid & ISP_SRC_SIZE) {
		memcpy(&pctx->uinfo.original, &in_param->src_info,
			sizeof(pctx->uinfo.original));
		cfg.src = in_param->src_info.dst_size;
		cfg.crop.start_x = 0;
		cfg.crop.start_y = 0;
		cfg.crop.size_x = cfg.src.w;
		cfg.crop.size_y = cfg.src.h;
		pctx->uinfo.src = cfg.src;
		pctx->uinfo.crop = cfg.crop;
		pr_debug("isp sw %d update size: %d %d\n",
			pctx->ctx_id, cfg.src.w, cfg.src.h);
		src_new = &cfg.src;
		pctx->isp_k_param.src_w = pctx->pipe_src.src.w;
		pctx->isp_k_param.src_h = pctx->pipe_src.src.h;
	}

	/* update all path scaler trim0  */
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		path_info = &pctx->uinfo.path_info[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		if (in_param->valid & update[i]) {
			path_trim = in_param->trim_path[i];
		} else if (src_new) {
			path_trim.start_x = path_trim.start_y = 0;
			path_trim.size_x = src_new->w;
			path_trim.size_y = src_new->h;
		} else {
			continue;
		}

		path_info->in_trim = path_trim;
		pr_debug("update isp path%d trim %d %d %d %d\n",
			i, path_trim.start_x, path_trim.start_y,
			path_trim.size_x, path_trim.size_y);
	}
	pctx->updated = 1;
	return ret;
}

static int ispcore_offline_param_cfg(struct isp_sw_context *pctx,
		struct camera_frame *pframe, struct offline_tmp_param *tmp)
{
	int ret = 0;
	int i = 0;
	struct isp_stream_ctrl *stream = NULL;
	struct isp_path_desc *path = NULL;
	struct isp_uinfo *pipe_src = NULL;
	struct isp_path_uinfo *path_info = NULL;
	struct isp_ctx_size_desc cfg;
	struct img_trim path_trim;

	if (!pctx || !pframe || !tmp) {
		pr_err("fail to get input ptr, pctx %p, pframe %p tmp %p\n",
			pctx, pframe, tmp);
		return -EFAULT;
	}

	pipe_src = &pctx->pipe_src;
	memcpy(pipe_src, &pctx->uinfo, sizeof(pctx->uinfo));
	stream = cam_queue_dequeue(&pctx->stream_ctrl_in_q,
		struct isp_stream_ctrl, list);
	tmp->stream = stream;
	if (stream) {
		pipe_src->in_fmt = stream->in_fmt;
		cfg.src = stream->in;
		cfg.crop = stream->in_crop;
		isp_path_fetchsize_update(pctx, &cfg);
		pr_debug("isp %d in_size %d %d crop_szie %d %d %d %d\n",
			pctx->ctx_id, stream->in.w, stream->in.h,
			stream->in_crop.start_x, stream->in_crop.start_y,
			stream->in_crop.size_x, stream->in_crop.size_y);
		for (i = 0; i < ISP_SPATH_NUM; i++) {
			path = &pctx->isp_path[i];
			path_info = &pctx->pipe_src.path_info[i];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			path_info->dst = stream->out[i];
			path_trim = stream->out_crop[i];
			isp_path_storecrop_update(path_info, &path_trim);
			pr_debug("isp %d out_size %d %d crop_szie %d %d %d %d\n",
				pctx->ctx_id, stream->out[i].w, stream->out[i].h,
				stream->out_crop[i].start_x, stream->out_crop[i].start_y,
				stream->out_crop[i].size_x, stream->out_crop[i].size_y);
		}
		if (pctx->uinfo.mode_3dnr == MODE_3DNR_CAP) {
			pctx->nr3_ctx.blending_cnt =
				stream->cur_cnt % NR3_BLEND_CNT;
			if (stream->data_src == ISP_STREAM_SRC_ISP)
				pctx->nr3_ctx.mode = MODE_3DNR_OFF;
			else
				pctx->nr3_ctx.mode = MODE_3DNR_CAP;
		}
		pctx->updated = 1;
	}

	if (pframe->sw_slice_num)
		ispcore_sw_slice_prepare(pctx, pframe);

	isp_drv_pipeinfo_get(pctx, pframe);
	ispcore_hist_roi_update(pctx);
	/*update NR param for crop/scaling image */
	ispcore_blkparam_adapt(pctx);
	/* the context/path maybe init/updated after dev start. */
	if (pctx->updated || pctx->sw_slice_num)
		ret = ispcore_slice_ctx_init(pctx, &tmp->multi_slice);
	if (pctx->pipe_src.uframe_sync)
		tmp->target_fid = ispcore_fid_across_context_get(pctx->dev,
			pctx->attach_cam_id);

	return ret;
}

static int ispcore_offline_param_set(struct isp_sw_context *pctx,
		struct camera_frame *pframe, struct offline_tmp_param *tmp)
{
	int ret = 0;
	int i = 0, loop = 0;
	struct isp_path_desc *path = NULL;
	struct isp_path_uinfo *path_info =NULL;
	struct isp_path_desc *slave_path = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct camera_frame *out_frame = NULL;
	struct cam_hw_info *hw = NULL;
	struct isp_pipe_info *pipe_in = NULL;
	struct isp_uinfo *pipe_src = NULL;

	if (!pctx || !pframe || !tmp) {
		pr_err("fail to get input ptr, pctx %p, pframe %p tmp %p\n",
			pctx, pframe, tmp);
		return -EFAULT;
	}

	dev = pctx->dev;
	hw = dev->isp_hw;
	pipe_src = &pctx->pipe_src;
	pipe_in = &pctx->pipe_info;

	hw->isp_ioctl(hw, ISP_HW_CFG_FETCH_FRAME_ADDR, &pipe_in->fetch);
	if (pipe_src->fetch_path_sel)
		hw->isp_ioctl(hw, ISP_HW_CFG_FBD_ADDR_SET, &pipe_in->fetch_fbd);
	if (pctx->updated || pctx->sw_slice_num) {
		hw->isp_ioctl(hw, ISP_HW_CFG_FETCH_SET, &pipe_in->fetch);
		if (pipe_src->fetch_path_sel)
			hw->isp_ioctl(hw, ISP_HW_CFG_FETCH_FBD_SET, &pipe_in->fetch_fbd);
		if (pipe_src->in_fmt == IMG_PIX_FMT_NV21)
			hw->isp_ioctl(hw, ISP_HW_CFG_ISP_CFG_SUBBLOCK, &pipe_in->fetch);
	}

	/* config all paths output */
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		path_info = &pctx->uinfo.path_info[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		/* slave path output buffer binding to master buffer*/
		if (path_info->bind_type == ISP_PATH_SLAVE)
			continue;
		out_frame = ispcore_path_out_frame_get(pctx, path, tmp);
		if (out_frame) {
			out_frame->fid = pframe->fid;
			out_frame->sensor_time = pframe->sensor_time;
			out_frame->boot_sensor_time = pframe->boot_sensor_time;
		} else {
			ret = -EINVAL;
			goto exit;
		}

		/* config store buffer */
		pr_debug("isp %d is_reserved %d iova 0x%x, user_fid: %x mfd 0x%x\n",
			pctx->ctx_id, out_frame->is_reserved,
			(uint32_t)out_frame->buf.iova[0], out_frame->user_fid,
			out_frame->buf.mfd[0]);
		ret = isp_path_store_frm_set(path, out_frame);
		/* If some error comes then do not start ISP */
		if (ret) {
			cam_buf_iommu_unmap(&out_frame->buf);
			cam_buf_ionbuf_put(&out_frame->buf);
			cam_queue_empty_frame_put(out_frame);
			ret = -EINVAL;
			goto exit;
		}
		if ((i < AFBC_PATH_NUM) && path_info->store_fbc)
			isp_path_afbc_store_frm_set(path, out_frame);

		if (path_info->bind_type == ISP_PATH_MASTER) {
			struct camera_frame temp;
			/* fixed buffer offset here. HAL should use same offset calculation method */
			temp.buf.iova[0] = out_frame->buf.iova[0] +
				pctx->pipe_info.store[i].store.total_size;
			temp.buf.iova[1] = temp.buf.iova[2] = 0;
			slave_path = &pctx->isp_path[path_info->slave_path_id];
			isp_path_store_frm_set(slave_path, &temp);
		}

		hw->isp_ioctl(hw, ISP_HW_CFG_STORE_FRAME_ADDR, &pipe_in->store[i]);
		if (pctx->updated || pctx->sw_slice_num) {
			if (path->spath_id == ISP_SPATH_FD)
				hw->isp_ioctl(hw, ISP_HW_CFG_SET_PATH_THUMBSCALER,
					&pipe_in->thumb_scaler);
			else
				hw->isp_ioctl(hw, ISP_HW_CFG_SET_PATH_SCALER,
					&pipe_in->scaler[i]);
			hw->isp_ioctl(hw, ISP_HW_CFG_SET_PATH_STORE, &pipe_in->store[i]);
			if (i < AFBC_PATH_NUM)
				hw->isp_ioctl(hw, ISP_HW_CFG_AFBC_PATH_SET, &pipe_in->afbc[i]);
		}

		/*
		 * context proc_queue frame number
		 * should be equal to path result queue.
		 * if ctx->proc_queue enqueue OK,
		 * path result_queue enqueue should be OK.
		 */
		loop = 0;
		do {
			ret = cam_queue_enqueue(&path->result_queue, &out_frame->list);
			if (ret == 0)
				break;
			printk_ratelimited(KERN_INFO "wait for output queue. loop %d\n", loop);
			/* wait for previous frame output queue done */
			mdelay(1);
		} while (loop++ < 500);

		if (ret) {
			isp_int_isp_irq_cnt_trace(tmp->hw_ctx_id);
			pr_err("fail to enqueue, hw %d, path %d, store %d\n",
				tmp->hw_ctx_id, path->spath_id,
				atomic_read(&path->store_cnt));
			/* ret frame to original queue */
			if (out_frame->is_reserved) {
				cam_queue_enqueue(
					&path->reserved_buf_queue, &out_frame->list);
				dev->ltm_handle->ops->clear_status(pctx->ltm_ctx.ltm_index);
			} else {
				cam_buf_iommu_unmap(&out_frame->buf);
				cam_queue_enqueue(
					&path->out_buf_queue, &out_frame->list);
			}
			ret = -EINVAL;
			goto exit;
		}
		atomic_inc(&path->store_cnt);
	}

exit:
	return ret;
}

static int ispcore_offline_frame_start(void *ctx)
{
	int ret = 0;
	int i = 0, loop = 0, kick_fmcu = 0, slc_by_ap = 0;
	int hw_ctx_id = -1;
	uint32_t use_fmcu = 0;
	uint32_t frame_id;
	struct isp_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_sw_context *pctx = NULL;
	struct isp_hw_context *pctx_hw = NULL;
	struct isp_path_desc *path;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct cam_hw_info *hw = NULL;
	struct offline_tmp_param tmp = {0};
	struct isp_hw_yuv_block_ctrl blk_ctrl;

	pctx = (struct isp_sw_context *)ctx;
	pr_debug("enter sw id %d, user_cnt=%d, ch_id=%d, cam_id=%d\n",
		pctx->ctx_id, atomic_read(&pctx->user_cnt),
		pctx->ch_id, pctx->attach_cam_id);

	if (atomic_read(&pctx->user_cnt) < 1) {
		pr_err("fail to init isp cxt %d.\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	hw = dev->isp_hw;
	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;

	if (pctx->multi_slice | ispcore_slice_needed(pctx))
		use_fmcu = FMCU_IS_NEED;
	if (use_fmcu && (pctx->uinfo.mode_3dnr != MODE_3DNR_OFF))
		use_fmcu |= FMCU_IS_MUST; /* force fmcu used*/
	if (pctx->uinfo.enable_slowmotion)
		use_fmcu |= FMCU_IS_MUST; // force fmcu used
	if (pctx->sw_slice_num)
		use_fmcu = FMCU_IS_NEED;

	loop = 0;
	do {
		ret = isp_core_context_bind(pctx, use_fmcu);
		if (!ret) {
			hw_ctx_id = isp_core_hw_context_id_get(pctx);
			if (hw_ctx_id >= 0 && hw_ctx_id < ISP_CONTEXT_HW_NUM)
				pctx_hw = &dev->hw_ctx[hw_ctx_id];
			else
				pr_err("fail to get hw_ctx_id\n");
			break;
		}
		pr_info_ratelimited("ctx %d wait for hw. loop %d\n", pctx->ctx_id, loop);
		usleep_range(600, 800);
	} while (loop++ < 5000);

	pframe = cam_queue_dequeue(&pctx->in_queue, struct camera_frame, list);

	if (!pctx_hw || pframe == NULL) {
		pr_err("fail to get hw(%p) or input frame (%p) for ctx %d\n",
			pctx_hw, pframe, pctx->ctx_id);
		ret = 0;
		goto input_err;
	}

	if ((pframe->fid & 0x1f) == 0)
		pr_info(" sw %d, cam%d , fid %d, ch_id %d, buf_fd %d 0x%lx\n",
			pctx->ctx_id, pctx->attach_cam_id,
			pframe->fid, pframe->channel_id, pframe->buf.mfd[0],
			pframe->buf.iova[0]);

	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. cxt %d\n",
			pctx->ctx_id);
		ret = -EINVAL;
		goto map_err;
	}

	frame_id = pframe->fid;
	loop = 0;
	do {
		ret = cam_queue_enqueue(&pctx->proc_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info_ratelimited("wait for proc queue. loop %d\n", loop);
		/* wait for previous frame proccessed done.*/
		usleep_range(600, 2000);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to input frame queue, timeout.\n");
		ret = -EINVAL;
		goto inq_overflow;
	}
	/*
	 * param_mutex to avoid ctx/all paths param
	 * updated when set to register.
	 */
	mutex_lock(&pctx->param_mutex);
	tmp.multi_slice = pctx->multi_slice;
	tmp.valid_out_frame = -1;
	tmp.target_fid = CAMERA_RESERVE_FRAME_NUM;
	tmp.hw_ctx_id = hw_ctx_id;
	tmp.stream = NULL;
	ret = ispcore_offline_param_cfg(pctx, pframe, &tmp);
	if (ret) {
		pr_err("fail to cfg offline param.\n");
		ret = 0;
		goto unlock;
	}

	ret = ispcore_offline_param_set(pctx, pframe, &tmp);
	if (ret) {
		pr_err("fail to set offline param.\n");
		ret = 0;
		goto unlock;
	}

	if (tmp.valid_out_frame == -1) {
		pr_debug(" No available output buffer sw %d, hw %d,discard\n",
			pctx_hw->sw_ctx_id, pctx_hw->hw_ctx_id);
		dev->ltm_handle->ops->clear_status(pctx->ltm_ctx.ltm_index);
		goto unlock;
	}

	ispcore_ltm_frame_previous_process(pctx, pframe);
	ispcore_3dnr_frame_process(pctx, pframe);
	ispcore_ltm_frame_process(pctx, pframe);

	use_fmcu = 0;
	fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
	if (fmcu) {
		use_fmcu = (tmp.multi_slice | pctx->uinfo.enable_slowmotion);
		if (use_fmcu)
			fmcu->ops->ctx_reset(fmcu);
	}

	if (tmp.multi_slice || pctx->uinfo.enable_slowmotion) {
		struct slice_cfg_input slc_cfg;

		memset(&slc_cfg, 0, sizeof(slc_cfg));
		for (i = 0; i < ISP_SPATH_NUM; i++) {
			path = &pctx->isp_path[i];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			slc_cfg.frame_store[i] = &pctx->pipe_info.store[i].store;
			if ((i < AFBC_PATH_NUM) && pctx->pipe_src.path_info[i].store_fbc)
				slc_cfg.frame_afbc_store[i] = &pctx->pipe_info.afbc[i].afbc_store;
		}
		slc_cfg.ltm_rgb_eb = pctx->pipe_src.ltm_rgb;
		slc_cfg.ltm_yuv_eb = pctx->pipe_src.ltm_yuv;
		slc_cfg.frame_fetch = &pctx->pipe_info.fetch;
		slc_cfg.frame_fbd_raw = &pctx->pipe_info.fetch_fbd;
		slc_cfg.frame_in_size.w = pctx->pipe_src.crop.size_x;
		slc_cfg.frame_in_size.h = pctx->pipe_src.crop.size_y;
		slc_cfg.nr3_ctx = &pctx->nr3_ctx;
		slc_cfg.ltm_ctx = &pctx->ltm_ctx;
		slc_cfg.nofilter_ctx = &pctx->isp_k_param;
		isp_slice_info_cfg(&slc_cfg, pctx->slice_ctx);

		if (!use_fmcu) {
			pr_debug("use ap support slices for ctx %d hw %d\n",
				pctx->ctx_id, hw_ctx_id);
			slc_by_ap = 1;
		} else {
			pr_debug("use fmcu support slices for ctx %d hw %d\n",
				pctx->ctx_id, hw_ctx_id);
			ret = isp_slice_fmcu_cmds_set((void *)fmcu, pctx);
			if (ret == 0)
				kick_fmcu = 1;
		}
	}

	pctx->updated = 0;
	mutex_unlock(&pctx->param_mutex);

	if (pctx->uinfo.enable_slowmotion) {
		for (i = 0; i < pctx->uinfo.slowmotion_count - 1; i++) {
			ret = ispcore_fmcu_slw_queue_set(fmcu, pctx);
			if (ret)
				pr_err("fail to set fmcu slw queue\n");
		}
	}

	ret = wait_for_completion_interruptible_timeout(
			&pctx->frm_done, ISP_CONTEXT_TIMEOUT);
	if (ret == ERESTARTSYS) {
		pr_err("fail to interrupt, when isp wait\n");
		ret = -EFAULT;
		goto dequeue;
	} else if (ret == 0) {
		pr_err("fail to wait isp context %d, timeout.\n", pctx->ctx_id);
		ret = -EFAULT;
		goto dequeue;
	}

	if (tmp.stream) {
		ret = cam_queue_enqueue(&pctx->stream_ctrl_proc_q, &tmp.stream->list);
		if (ret) {
			pr_err("fail to stream state overflow\n");
			cam_queue_empty_state_put(tmp.stream);
		}
	}
	pctx->iommu_status = (uint32_t)(-1);
	pctx->started = 1;
	pctx->multi_slice = tmp.multi_slice;
	pctx_hw->fmcu_used = use_fmcu;

	if (slc_by_ap) {
		ret = ispcore_slices_proc(pctx, pframe);
		goto done;
	}

	/* start to prepare/kickoff cfg buffer. */
	if (likely(dev->wmode == ISP_CFG_MODE)) {
		pr_debug("cfg enter.");

		/* blkpm_lock to avoid user config block param across frame */
		mutex_lock(&pctx->blkpm_lock);
		ispcore_debug_dump_check(pctx, pframe);
		blk_ctrl.idx = pctx->ctx_id;
		blk_ctrl.blk_param = &pctx->isp_k_param;
		if (pctx->pipe_src.in_fmt == IMG_PIX_FMT_NV21)
			blk_ctrl.type = ISP_YUV_BLOCK_DISABLE;
		else
			blk_ctrl.type = ISP_YUV_BLOCK_CFG;
		hw->isp_ioctl(hw, ISP_HW_CFG_YUV_BLOCK_CTRL_TYPE, &blk_ctrl);
		ret = cfg_desc->ops->hw_cfg(cfg_desc,
					pctx->ctx_id, hw_ctx_id, kick_fmcu);
		mutex_unlock(&pctx->blkpm_lock);

		if (kick_fmcu) {
			pr_info("isp %d frame %d w %d h %d fmcu start\n", pctx->ctx_id,
				frame_id, pctx->pipe_src.crop.size_x, pctx->pipe_src.crop.size_y);
			if (pctx->pipe_src.slw_state == CAM_SLOWMOTION_ON) {
				ret = fmcu->ops->cmd_ready(fmcu);
			} else {
				ret = fmcu->ops->hw_start(fmcu);
				if (!ret && pctx->uinfo.enable_slowmotion)
					pctx->pipe_src.slw_state = CAM_SLOWMOTION_ON;
			}
		} else {
			pr_debug("cfg start. fid %d\n", frame_id);
			hw->isp_ioctl(hw, ISP_HW_CFG_START_ISP, &hw_ctx_id);
		}
	} else {
		if (kick_fmcu) {
			pr_info("fmcu start.");
			ret = fmcu->ops->hw_start(fmcu);
		} else {
			pr_debug("fetch start.");
			hw->isp_ioctl(hw, ISP_HW_CFG_FETCH_START, NULL);
		}
	}

done:
	pr_debug("done.\n");
	return 0;

unlock:
	mutex_unlock(&pctx->param_mutex);
dequeue:
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		pframe = cam_queue_dequeue_tail(&path->result_queue);
		if (pframe) {
			/* ret frame to original queue */
			if (pframe->is_reserved)
				cam_queue_enqueue(
					&path->reserved_buf_queue, &pframe->list);
			else
				cam_queue_enqueue(
					&path->out_buf_queue, &pframe->list);
		}
		atomic_dec(&path->store_cnt);
	}

	pframe = cam_queue_dequeue_tail(&pctx->proc_queue);
inq_overflow:
	if (pframe)
		cam_buf_iommu_unmap(&pframe->buf);
map_err:
input_err:
	if (pframe) {
		ispcore_offline_pararm_free(pframe->param_data);
		pframe->param_data = NULL;
		/* release sync data as if ISP has consumed */
		if (pframe->sync_data)
			dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
		/* return buffer to cam channel shared buffer queue. */
		if (tmp.stream && tmp.stream->data_src == ISP_STREAM_SRC_ISP)
			pr_debug("isp postproc no need return\n");
		else
			pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);
	}

	if (tmp.stream)
		cam_queue_empty_state_put(tmp.stream);
	if (pctx->uinfo.enable_slowmotion) {
		for (i = 0; i < pctx->uinfo.slowmotion_count - 1; i++) {
			pframe = cam_queue_dequeue(&pctx->in_queue,
					struct camera_frame, list);
			if (pframe) {
				ispcore_offline_pararm_free(pframe->param_data);
				pframe->param_data = NULL;
				/* release sync data as if ISP has consumed */
				if (pframe->sync_data)
					dcam_core_dcam_if_release_sync(pframe->sync_data, pframe);
				/* return buffer to cam channel shared buffer queue. */
				pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);
			}
		}
	}
	if (pctx_hw)
		isp_core_context_unbind(pctx);
	return ret;
}

static int ispcore_offline_thread_loop(void *arg)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_sw_context *pctx;
	struct cam_thread_info *thrd;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_thread_info *)arg;
	pctx = (struct isp_sw_context *)thrd->ctx_handle;
	dev = pctx->dev;

	while (1) {
		if (wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(
					&thrd->thread_stop, 1, 0) == 1) {
				pr_info("isp context %d thread stop.\n",
						pctx->ctx_id);
				break;
			}
			pr_debug("thread com done.\n");

			if (thrd->proc_func(pctx)) {
				pr_err("fail to start isp pipe to proc. exit thread\n");
				pctx->isp_cb_func(
					ISP_CB_DEV_ERR, dev,
					pctx->cb_priv_data);
				break;
			}
		} else {
			pr_debug("offline thread exit!");
			break;
		}
	}

	return 0;
}

static int ispcore_offline_thread_stop(void *param)
{
	int cnt = 0;
	int ret = 0;
	struct cam_thread_info *thrd;
	struct isp_sw_context *pctx;

	thrd = (struct cam_thread_info *)param;
	pctx = (struct isp_sw_context *)thrd->ctx_handle;

	if (thrd->thread_task) {
		atomic_set(&thrd->thread_stop, 1);
		complete(&thrd->thread_com);
		while (cnt < 2500) {
			cnt++;
			if (atomic_read(&thrd->thread_stop) == 0)
				break;
			udelay(1000);
		}
		thrd->thread_task = NULL;
		pr_info("offline thread stopped. wait %d ms\n", cnt);
	}

	/* wait for last frame done */
	ret = wait_for_completion_interruptible_timeout(
		&pctx->frm_done, ISP_CONTEXT_TIMEOUT);
	if (ret == -ERESTARTSYS)
		pr_err("fail to interrupt, when isp wait\n");
	else if (ret == 0)
		pr_err("fail to wait ctx %d, timeout.\n", pctx->ctx_id);
	else
		pr_info("wait time %d\n", ret);
	return 0;
}

static int ispcore_offline_thread_create(void *param)
{
	struct isp_sw_context *pctx;
	struct cam_thread_info *thrd;
	char thread_name[32] = { 0 };

	pctx = (struct isp_sw_context *)param;
	thrd = &pctx->thread;
	thrd->ctx_handle = pctx;

	if (thrd->thread_task) {
		pr_info("isp ctx sw %d offline thread created is exist.\n",
			pctx->ctx_id);
		return 0;
	}

	thrd->proc_func = ispcore_offline_frame_start;
	sprintf(thread_name, "isp_ctx%d_offline", pctx->ctx_id);
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);
	thrd->thread_task = kthread_run(ispcore_offline_thread_loop,
		thrd, "%s", thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start offline thread for isp sw %d err %ld\n",
				pctx->ctx_id, PTR_ERR(thrd->thread_task));
		return -EFAULT;
	}

	pr_info("isp ctx sw %d offline thread created.\n", pctx->ctx_id);
	return 0;
}

static int ispcore_stream_state_get(struct isp_sw_context *pctx)
{
	int ret = 0;
	int i = 0, j = 0;
	uint32_t normal_cnt = 0, postproc_cnt = 0;
	uint32_t maxw = 0, maxh = 0;
	uint32_t scl_x = 0, scl_w = 0, scl_h = 0;
	enum isp_stream_frame_type frame_type = ISP_STREAM_SIGNLE;
	struct isp_stream_ctrl *stream = NULL;
	struct isp_path_desc *path = NULL;
	struct isp_uinfo *uinfo = NULL;
	struct isp_path_uinfo *path_info = NULL;
	struct isp_stream_ctrl tmp_stream[5];

	if (!pctx) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (pctx->stream_ctrl_in_q.cnt != 0) {
		pr_debug("cxt %d stream state not complete\n", pctx->ctx_id);
		return ret;
	}

	uinfo = &pctx->uinfo;
	normal_cnt = 1;
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		path_info = &uinfo->path_info[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		maxw = MAX(maxw, path_info->dst.w);
		maxh = MAX(maxh, path_info->dst.h);
	}

	if (!maxw || !maxh) {
		pr_err("fail to get valid max dst size\n");
		return -EFAULT;
	}

	scl_w = maxw / uinfo->crop.size_x;
	if ((maxw % uinfo->crop.size_x) != 0)
		scl_w ++;
	scl_h = maxh / uinfo->crop.size_y;
	if ((maxh % uinfo->crop.size_y) != 0)
		scl_h ++;
	scl_x = MAX(scl_w, scl_h);
	pr_debug("scl_x %d scl_w %d scl_h %d max_w %d max_h %d\n", scl_x,
		scl_w, scl_h, maxw, maxh);
	do {
		if (scl_x == ISP_SCALER_UP_MAX)
			break;
		scl_x = scl_x / ISP_SCALER_UP_MAX;
		if (scl_x)
			postproc_cnt++;
	} while (scl_x);

	/* If need postproc, first normal proc need to mark as postproc too*/
	if (postproc_cnt) {
		postproc_cnt++;
		normal_cnt--;
	}

	for (i = postproc_cnt - 1; i >= 0; i--) {
		maxw = maxw / ISP_SCALER_UP_MAX;
		maxw = MAX(maxw, uinfo->crop.size_y);
		maxw = ISP_ALIGN_W(maxw);
		maxh = maxh / ISP_SCALER_UP_MAX;
		maxh = MAX(maxh, uinfo->crop.size_y);
		maxh = ISP_ALIGN_H(maxh);

		if (i == 0) {
			tmp_stream[i].in_fmt = uinfo->in_fmt;
			tmp_stream[i].in = uinfo->src;
			tmp_stream[i].in_crop = uinfo->crop;
		} else {
			tmp_stream[i].in_fmt = IMG_PIX_FMT_NV21;
			tmp_stream[i].in.w = maxw;
			tmp_stream[i].in.h = maxh;
			tmp_stream[i].in_crop.start_x = 0;
			tmp_stream[i].in_crop.start_y = 0;
			tmp_stream[i].in_crop.size_x = maxw;
			tmp_stream[i].in_crop.size_y = maxh;
		}
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			path = &pctx->isp_path[j];
			path_info = &uinfo->path_info[j];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			/* This is for ensure the last postproc frame buf is OUT for user
			.* Then the frame before should be POST. Thus, only one post
			.* buffer is enough for all the isp postproc process */
			if ((postproc_cnt + i) % 2 == 1)
				tmp_stream[i].buf_type[j] = ISP_STREAM_BUF_OUT;
			else
				tmp_stream[i].buf_type[j] = ISP_STREAM_BUF_POSTPROC;
			if (j != ISP_SPATH_CP && (i != postproc_cnt - 1))
				tmp_stream[i].buf_type[j] = ISP_STREAM_BUF_RESERVED;
			if (i == (postproc_cnt - 1)) {
				tmp_stream[i].out[j] = path_info->dst;
				tmp_stream[i].out_crop[j].start_x = 0;
				tmp_stream[i].out_crop[j].start_y = 0;
				tmp_stream[i].out_crop[j].size_x = maxw;
				tmp_stream[i].out_crop[j].size_y = maxh;
			} else if (i == 0) {
				tmp_stream[i].out[j].w = tmp_stream[i + 1].in.w;
				tmp_stream[i].out[j].h = tmp_stream[i + 1].in.h;
				tmp_stream[i].out_crop[j] = path_info->in_trim;
			} else {
				tmp_stream[i].out[j].w = tmp_stream[i + 1].in.w;
				tmp_stream[i].out[j].h = tmp_stream[i + 1].in.h;
				tmp_stream[i].out_crop[j].start_x = 0;
				tmp_stream[i].out_crop[j].start_y = 0;
				tmp_stream[i].out_crop[j].size_x = maxw;
				tmp_stream[i].out_crop[j].size_y = maxh;
			}
			pr_debug("isp %d i %d j %d out_size %d %d crop_szie %d %d %d %d\n",
				pctx->ctx_id, i, j, tmp_stream[i].out[j].w, tmp_stream[i].out[j].h,
				tmp_stream[i].out_crop[j].start_x, tmp_stream[i].out_crop[j].start_y,
				tmp_stream[i].out_crop[j].size_x, tmp_stream[i].out_crop[j].size_y);
		}
		pr_debug("isp %d index %d in_size %d %d crop_szie %d %d %d %d\n",
			pctx->ctx_id, i, tmp_stream[i].in.w, tmp_stream[i].in.h,
			tmp_stream[i].in_crop.start_x, tmp_stream[i].in_crop.start_y,
			tmp_stream[i].in_crop.size_x, tmp_stream[i].in_crop.size_y);
	}

	if (pctx->uinfo.mode_3dnr == MODE_3DNR_CAP) {
		normal_cnt = normal_cnt + NR3_BLEND_CNT - 1;
		frame_type = ISP_STREAM_MULTI;
	}
	for (i = 0; i < normal_cnt; i++) {
		stream = cam_queue_empty_state_get();
		stream->state = ISP_STREAM_NORMAL_PROC;
		stream->data_src = ISP_STREAM_SRC_DCAM;
		stream->frame_type = frame_type;
		stream->in = uinfo->src;
		stream->in_crop = uinfo->crop;
		stream->in_fmt = uinfo->in_fmt;
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			path = &pctx->isp_path[j];
			path_info = &uinfo->path_info[j];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			/* Use reserved buffer when not 3dnr last frame */
			if (normal_cnt == 1 || i == (NR3_BLEND_CNT - 1))
				stream->buf_type[j] = ISP_STREAM_BUF_OUT;
			else
				stream->buf_type[j] = ISP_STREAM_BUF_RESERVED;
			stream->out[j] = path_info->dst;
			stream->out_crop[j] = path_info->in_trim;
			if (postproc_cnt) {
				stream->out[j] = tmp_stream[0].out[j];
				stream->out_crop[j] = tmp_stream[0].out_crop[j];
			}
			pr_debug("isp %d out_size %d %d crop_szie %d %d %d %d\n",
				pctx->ctx_id, stream->out[j].w, stream->out[j].h,
				stream->out_crop[j].start_x, stream->out_crop[j].start_y,
				stream->out_crop[j].size_x, stream->out_crop[j].size_y);
		}
		stream->cur_cnt = i;
		stream->max_cnt = normal_cnt + postproc_cnt - 1;
		pr_debug("stream type %d cur_cnt %d max_cnt %d\n",
			stream->buf_type, stream->cur_cnt, stream->max_cnt);
		ret = cam_queue_enqueue(&pctx->stream_ctrl_in_q, &stream->list);
		if (ret) {
			pr_info("stream state overflow\n");
			cam_queue_empty_state_put(stream);
		}
		pr_debug("isp %d in_size %d %d crop_szie %d %d %d %d\n",
			pctx->ctx_id, stream->in.w, stream->in.h,
			stream->in_crop.start_x, stream->in_crop.start_y,
			stream->in_crop.size_x, stream->in_crop.size_y);
	}

	for (i = 0; i < postproc_cnt; i++) {
		stream = cam_queue_empty_state_get();
		stream->state = ISP_STREAM_POST_PROC;
		stream->data_src = ISP_STREAM_SRC_ISP;
		stream->frame_type = frame_type;
		stream->in_fmt = tmp_stream[i].in_fmt;
		stream->in = tmp_stream[i].in;
		stream->in_crop = tmp_stream[i].in_crop;
		pr_debug("isp %d in_size %d %d crop_szie %d %d %d %d\n",
			pctx->ctx_id, stream->in.w, stream->in.h,
			stream->in_crop.start_x, stream->in_crop.start_y,
			stream->in_crop.size_x, stream->in_crop.size_y);
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			path = &pctx->isp_path[j];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			stream->buf_type[j] = tmp_stream[i].buf_type[j];
			stream->out[j] = tmp_stream[i].out[j];
			stream->out_crop[j] = tmp_stream[i].out_crop[j];
			pr_debug("isp %d out_size %d %d crop_szie %d %d %d %d\n",
				pctx->ctx_id, stream->out[j].w, stream->out[j].h,
				stream->out_crop[j].start_x, stream->out_crop[j].start_y,
				stream->out_crop[j].size_x, stream->out_crop[j].size_y);
		}
		if (i == 0)
			stream->data_src = ISP_STREAM_SRC_DCAM;
		stream->cur_cnt = i + normal_cnt;
		stream->max_cnt = normal_cnt + postproc_cnt - 1;
		ret = cam_queue_enqueue(&pctx->stream_ctrl_in_q, &stream->list);
		if (ret) {
			pr_err("fail to stream state overflow\n");
			cam_queue_empty_state_put(stream);
		}
	}

	return ret;
}

static int ispcore_stream_state_put(void *isp_handle, int ctx_id)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_sw_context *pctx = NULL;
	struct isp_stream_ctrl *stream = NULL;

	if (!isp_handle) {
		pr_err("fail to input ptr NULL\n");
		ret = -EFAULT;
		goto exit;
	}

	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_debug("fail to ctx_id is err  %d\n", ctx_id);
		ret = -EFAULT;
		goto exit;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	do {
		stream = cam_queue_dequeue(&pctx->stream_ctrl_in_q,
			struct isp_stream_ctrl, list);
		if (stream)
			cam_queue_empty_state_put(stream);
	} while(stream);

exit:
	return ret;
}

static int ispcore_postproc_irq(void *handle, uint32_t idx,
	enum isp_postproc_type type)
{
	int ret = 0;
	int i, j;
	struct isp_pipe_dev *dev = NULL;
	struct isp_stream_ctrl *stream = NULL;
	struct isp_sw_context *pctx = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_path_desc *path;
	struct timespec cur_ts;
	ktime_t boot_time;

	memset(&cur_ts, 0, sizeof(struct timespec));
	if (!handle || type >= POSTPROC_MAX) {
		pr_err("fail to get valid input handle %p, type %d\n",
			handle, type);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	pctx = dev->sw_ctx[idx];
	if (pctx->uinfo.enable_slowmotion == 0) {
		isp_core_context_unbind(pctx);
		complete(&pctx->frm_done);
	}

	boot_time = ktime_get_boottime();
	ktime_get_ts(&cur_ts);

	stream = cam_queue_dequeue(&pctx->stream_ctrl_proc_q,
		struct isp_stream_ctrl, list);

	pframe = cam_queue_dequeue(&pctx->proc_queue, struct camera_frame, list);

	if (pframe) {
		if (stream && stream->data_src == ISP_STREAM_SRC_ISP) {
			pr_info("isp %d post proc, do not need to return frame\n",
				pctx->ctx_id);
			cam_buf_iommu_unmap(&pframe->buf);
		} else {
			/* return buffer to cam channel shared buffer queue. */
			cam_buf_iommu_unmap(&pframe->buf);
			pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe,
				pctx->cb_priv_data);
			pr_debug("sw %d, ch_id %d, fid:%d, shard buffer cnt:%d\n",
				pctx->ctx_id, pframe->channel_id, pframe->fid,
				pctx->proc_queue.cnt);
		}
	} else {
		pr_err("fail to get src frame  sw_idx=%d  proc_queue.cnt:%d\n",
			pctx->ctx_id, pctx->proc_queue.cnt);
	}

	if (pctx->sw_slice_num) {
		if (pctx->sw_slice_no != (pctx->sw_slice_num - 1)) {
			pr_debug("done cxt_id:%d ch_id[%d] slice %d\n",
				pctx->ctx_id, pctx->ch_id, pctx->sw_slice_no);
			return 0;
		} else {
			pr_debug("done cxt_id:%d ch_id[%d] lastslice %d\n",
				pctx->ctx_id, pctx->ch_id, pctx->sw_slice_no);
			pctx->sw_slice_no = 0;
			pctx->sw_slice_num = 0;
		}
	}

	/* get output buffers for all path */
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) <= 0) {
			pr_debug("path %p not enable\n", path);
			continue;
		}
		if (pctx->uinfo.path_info[i].bind_type == ISP_PATH_SLAVE) {
			pr_debug("slave path %d\n", path->spath_id);
			continue;
		}
		pframe = cam_queue_dequeue(&path->result_queue,
			struct camera_frame, list);
		if (!pframe) {
			pr_err("fail to get frame from queue. cxt:%d, path:%d\n",
				pctx->ctx_id, path->spath_id);
			continue;
		}
		atomic_dec(&path->store_cnt);
		pframe->boot_time = boot_time;
		pframe->time.tv_sec = cur_ts.tv_sec;
		pframe->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;

		pr_debug("ctx %d path %d, ch_id %d, fid %d, mfd 0x%x, storen %d, queue cnt:%d, is_reserved %d\n",
			pctx->ctx_id, path->spath_id, pframe->channel_id, pframe->fid, pframe->buf.mfd[0],
			atomic_read(&path->store_cnt), path->result_queue.cnt, pframe->is_reserved);
		pr_debug("time_sensor %03d.%6d, time_isp %03d.%06d\n",
			(uint32_t)pframe->sensor_time.tv_sec,
			(uint32_t)pframe->sensor_time.tv_usec,
			(uint32_t)pframe->time.tv_sec,
			(uint32_t)pframe->time.tv_usec);

		if (unlikely(pframe->is_reserved)) {
			cam_queue_enqueue(&path->reserved_buf_queue, &pframe->list);
		} else if (stream && stream->cur_cnt != stream->max_cnt &&
			stream->state == ISP_STREAM_POST_PROC) {
			cam_buf_iommu_unmap(&pframe->buf);
			cam_queue_enqueue(&pctx->in_queue, &pframe->list);
			complete(&pctx->thread.thread_com);
		} else {
			if (pframe->buf.mfd[0] == path->reserved_buf_fd) {
				for (j = 0; j < 3; j++) {
					pframe->buf.size[j] = path->reserve_buf_size[j];
					pr_debug("pframe->buf.size[j] = %d, path->reserve_buf_size[j] = %d",
						(int)pframe->buf.size[j], (int)path->reserve_buf_size[j]);
				}
			}
			cam_buf_iommu_unmap(&pframe->buf);
			pctx->isp_cb_func(ISP_CB_RET_DST_BUF,
				pframe, pctx->cb_priv_data);
		}
	}

	if (stream)
		cam_queue_empty_state_put(stream);

	return ret;
}

/* offline process frame */
static int ispcore_frame_proc(void *isp_handle, void *param, int ctx_id)
{
	int ret = 0;
	struct camera_frame *pframe;
	static int slw_frm_cnt;
	struct isp_sw_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_stream_ctrl *stream = NULL;
	struct isp_offline_param *in_param = NULL;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];
	pframe = (struct camera_frame *)param;
	pframe->priv_data = pctx;

	stream = cam_queue_dequeue_peek(&pctx->stream_ctrl_in_q,
		struct isp_stream_ctrl, list);
	if (stream && stream->state == ISP_STREAM_POST_PROC) {
		if (stream->frame_type == ISP_STREAM_SIGNLE ||
			stream->data_src == ISP_STREAM_SRC_ISP) {
			pr_debug("isp postproc running: frame need write back\n");
			return -EFAULT;
		}
	}

	in_param = (struct isp_offline_param *)pframe->param_data;
	if (in_param) {
		/*preview*/
		ispcore_offline_size_update(pctx, in_param);
		ispcore_offline_pararm_free(in_param);
		pframe->param_data = NULL;
	}

	pr_debug("cam%d ctx %d, fid %d, ch_id %d, buf  %d, 3dnr %d , w %d, h %d\n",
		pctx->attach_cam_id, ctx_id, pframe->fid,
		pframe->channel_id, pframe->buf.mfd[0], pctx->uinfo.mode_3dnr,
		pframe->width, pframe->height);

	ret = cam_queue_enqueue(&pctx->in_queue, &pframe->list);
	if (ret == 0) {
		ispcore_stream_state_get(pctx);
		if (pctx->uinfo.enable_slowmotion &&
			++slw_frm_cnt < pctx->uinfo.slowmotion_count)
			return ret;
		complete(&pctx->thread.thread_com);
		slw_frm_cnt = 0;
	}

	return ret;
}

/*
 * Enable path of sepicific path_id for specific context.
 * The context must be enable before get_path.
 */
static int ispcore_path_get(void *isp_handle, int ctx_id, int path_id)
{
	struct isp_sw_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_debug("start.\n");
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM ||
		path_id < 0 || path_id >= ISP_SPATH_NUM) {
		pr_err("fail to get legal ctx_id %d, path_id %d\n", ctx_id, path_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) < 1) {
		mutex_unlock(&dev->path_mutex);
		pr_err("fail to get path from free context.\n");
		return -EFAULT;
	}

	path = &pctx->isp_path[path_id];
	if (atomic_inc_return(&path->user_cnt) > 1) {
		mutex_unlock(&dev->path_mutex);
		atomic_dec(&path->user_cnt);
		pr_err("fail to get used path %d of cxt %d.\n",
			path_id, ctx_id);
		return -EFAULT;
	}

	mutex_unlock(&dev->path_mutex);

	atomic_set(&path->store_cnt, 0);

	if (path->q_init == 0) {
		if (pctx->uinfo.enable_slowmotion)
			cam_queue_init(&path->result_queue,
				ISP_SLW_RESULT_Q_LEN, ispcore_out_frame_ret);
		else
			cam_queue_init(&path->result_queue,
				ISP_RESULT_Q_LEN, ispcore_out_frame_ret);
		cam_queue_init(&path->out_buf_queue, ISP_OUT_BUF_Q_LEN,
			ispcore_out_frame_ret);
		cam_queue_init(&path->reserved_buf_queue,
			ISP_RESERVE_BUF_Q_LEN, ispcore_reserved_buf_destroy);
		path->q_init = 1;
	}

	pr_info("get path %d done.", path_id);
	return 0;
}

/*
 * Disable path of sepicific path_id for specific context.
 * The path and context should be enable before this.
 */
static int ispcore_path_put(void *isp_handle, int ctx_id, int path_id)
{
	int ret = 0;
	struct isp_sw_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_debug("start.\n");
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM ||
		path_id < 0 || path_id >= ISP_SPATH_NUM) {
		pr_err("fail to get legal ctx_id %d, path_id %d\n",
			ctx_id, path_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) < 1) {
		mutex_unlock(&dev->path_mutex);
		pr_err("fail to free path for free context.\n");
		return -EFAULT;
	}

	path = &pctx->isp_path[path_id];

	if (atomic_read(&path->user_cnt) == 0) {
		mutex_unlock(&dev->path_mutex);
		pr_err("fail to use free isp cxt %d, path %d.\n",
			ctx_id, path_id);
		return -EFAULT;
	}

	if (atomic_dec_return(&path->user_cnt) != 0) {
		pr_warn("isp cxt %d, path %d has multi users.\n",
					ctx_id, path_id);
		atomic_set(&path->user_cnt, 0);
	}

	mutex_unlock(&dev->path_mutex);
	pr_info("done, put path_id: %d for ctx %d\n", path_id, ctx_id);
	return ret;
}

static int ispcore_path_cfg(void *isp_handle,
			enum isp_path_cfg_cmd cfg_cmd,
			int ctx_id, int path_id,
			void *param)
{
	int ret = 0;
	int i, j;
	struct isp_sw_context *pctx = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_path_desc *path = NULL;
	struct isp_path_uinfo *slave_path = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_path_uinfo *path_info = NULL;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}

	if (ctx_id >= ISP_CONTEXT_SW_NUM  || ctx_id < ISP_CONTEXT_P0) {
		pr_err("fail to get legal id ctx %d\n", ctx_id);
		return -EFAULT;
	}
	if (path_id >= ISP_SPATH_NUM) {
		pr_err("fail to use legal id path %d\n", path_id);
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];
	path = &pctx->isp_path[path_id];
	path_info = &pctx->uinfo.path_info[path_id];

	if ((cfg_cmd != ISP_PATH_CFG_CTX_BASE) &&
		(cfg_cmd != ISP_PATH_CFG_CTX_SIZE) &&
		(cfg_cmd != ISP_PATH_CFG_CTX_COMPRESSION)) {
		if (atomic_read(&path->user_cnt) == 0) {
			pr_err("fail to use free isp cxt %d, path %d.\n",
					ctx_id, path_id);
			return -EFAULT;
		}
	}

	switch (cfg_cmd) {
	case ISP_PATH_CFG_OUTPUT_RESERVED_BUF:
		pframe = (struct camera_frame *)param;
		ret = cam_buf_iommu_single_page_map(&pframe->buf,
			CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map isp iommu buf.\n");
			ret = -EINVAL;
			goto exit;
		}
		pr_debug("cfg buf path %d, %p\n",
			path->spath_id, pframe);

		/* is_reserved:
		 *  1:  basic mapping reserved buffer;
		 *  2:  copy of reserved buffer.
		 */
		if (unlikely(cfg_cmd == ISP_PATH_CFG_OUTPUT_RESERVED_BUF)) {
			struct camera_frame *newfrm;

			pframe->is_reserved = 1;
			pframe->priv_data = path;
			path->reserved_buf_fd = pframe->buf.mfd[0];
			for (j = 0; j < 3; j++)
				path->reserve_buf_size[j] = pframe->buf.size[j];
			pr_info("reserved buf\n");
			ret = cam_queue_enqueue(&path->reserved_buf_queue,
				&pframe->list);
			i = 1;
			while (i < ISP_RESERVE_BUF_Q_LEN) {
				newfrm = cam_queue_empty_frame_get();
				if (newfrm) {
					newfrm->is_reserved = 2;
					newfrm->priv_data = path;
					newfrm->user_fid = pframe->user_fid;
					newfrm->channel_id = pframe->channel_id;
					memcpy(&newfrm->buf,
						&pframe->buf,
						sizeof(pframe->buf));
					cam_queue_enqueue(
						&path->reserved_buf_queue,
						&newfrm->list);
					i++;
				}
			}
		}
		break;
	case ISP_PATH_CFG_OUTPUT_BUF:
		pframe = (struct camera_frame *)param;
		pr_debug("output buf\n");
		pframe->is_reserved = 0;
		pframe->priv_data = path;
		ret = cam_queue_enqueue(&path->out_buf_queue, &pframe->list);
		if (ret) {
			pr_err("fail to enqueue output buffer, path %d.\n",
				path_id);
			goto exit;
		}
		break;
	case ISP_PATH_CFG_3DNR_BUF:
		pframe = (struct camera_frame *)param;
		ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map isp iommu buf.\n");
			ret = -EINVAL;
			goto exit;
		}
		for (i = 0; i < ISP_NR3_BUF_NUM; i++) {
			if (pctx->nr3_buf[i] == NULL) {
				pctx->nr3_buf[i] = pframe;
				pctx->nr3_ctx.buf_info[i] =
					&pctx->nr3_buf[i]->buf;
				pr_debug("3DNR CFGB[%d][0x%p][0x%p] = 0x%lx, 0x%lx\n",
					i, pframe, pctx->nr3_buf[i],
					pctx->nr3_ctx.buf_info[i]->iova[0],
					pctx->nr3_buf[i]->buf.iova[0]);
				break;
			} else {
				pr_debug("3DNR CFGB[%d][0x%p][0x%p] failed\n",
					i, pframe, pctx->nr3_buf[i]);
			}
		}
		if (i == 2) {
			pr_err("fail to set isp ctx %d nr3 buffers.\n", ctx_id);
			cam_buf_iommu_unmap(&pframe->buf);
			goto exit;
		}
		break;
	case ISP_PATH_CFG_RGB_LTM_BUF:
		pframe = (struct camera_frame *)param;
		if (pctx->uinfo.mode_ltm == MODE_LTM_PRE) {
			ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
			if (ret) {
				pr_err("fail to isp map iommu buf.\n");
				ret = -EINVAL;
				goto exit;
			}
		}
		for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
			if (pctx->ltm_buf[LTM_RGB][i] == NULL) {
				pctx->ltm_buf[LTM_RGB][i] = pframe;
				pctx->ltm_ctx.pbuf[LTM_RGB][i] = &pframe->buf;
				pr_debug("LTM CFGB[%d][0x%p][0x%p] = 0x%lx, 0x%lx\n",
					i, pframe, pctx->ltm_buf[LTM_RGB][i],
					pctx->ltm_ctx.pbuf[LTM_RGB][i]->iova[0],
					pctx->ltm_buf[LTM_RGB][i]->buf.iova[0]);
				break;
			}
		}
		pr_debug("isp ctx [%d], ltm buf idx [%d], buf addr [0x%p]\n",
			pctx->ctx_id, i, pframe);
		break;
	case ISP_PATH_CFG_YUV_LTM_BUF:
		pframe = (struct camera_frame *)param;
		if (pctx->pipe_src.mode_ltm == MODE_LTM_PRE) {
			ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
			if (ret) {
				pr_err("fail to isp map iommu buf.\n");
				ret = -EINVAL;
				goto exit;
			}
		}
		for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
			if (pctx->ltm_buf[LTM_YUV][i] == NULL) {
				pctx->ltm_buf[LTM_YUV][i] = pframe;
				pctx->ltm_ctx.pbuf[LTM_YUV][i] = &pframe->buf;
				pr_debug("LTM CFGB[%d][0x%p][0x%p] = 0x%lx, 0x%lx\n",
					i, pframe, pctx->ltm_buf[LTM_YUV][i],
					pctx->ltm_ctx.pbuf[LTM_YUV][i]->iova[0],
					pctx->ltm_buf[LTM_YUV][i]->buf.iova[0]);
				break;
			}
		}
		pr_debug("isp ctx [%d], ltm buf idx [%d], buf addr [0x%p]\n",
			pctx->ctx_id, i, pframe);
		break;
	case ISP_PATH_CFG_POSTPROC_BUF:
		pframe = (struct camera_frame *)param;
		pctx->postproc_buf = pframe;
		pr_debug("isp %d postproc buffer %d.\n", pctx->ctx_id,
			pframe->buf.mfd[0]);
		break;
	case ISP_PATH_CFG_CTX_BASE:
		ret = isp_path_comn_uinfo_set(pctx, param);
		pctx->updated = 1;
		break;
	case ISP_PATH_CFG_CTX_SIZE:
		mutex_lock(&pctx->param_mutex);
		ret = isp_path_fetch_uinfo_set(pctx, param);
		pctx->updated = 1;
		mutex_unlock(&pctx->param_mutex);
		break;
	case ISP_PATH_CFG_CTX_COMPRESSION:
		ret = isp_path_fetch_compress_uinfo_set(pctx, param);
		break;
	case ISP_PATH_CFG_CTX_UFRAME_SYNC:
		ret = isp_path_fetchsync_uinfo_set(pctx, param);
		break;
	case ISP_PATH_CFG_PATH_BASE:
		ret = isp_path_storecomn_uinfo_set(path_info, param);
		pctx->updated = 1;
		break;
	case ISP_PATH_CFG_PATH_SIZE:
		mutex_lock(&pctx->param_mutex);
		ret = isp_path_storecrop_uinfo_set(path_info, param);
		if (path_info->bind_type == ISP_PATH_MASTER) {
			slave_path = &pctx->uinfo.path_info[path_info->slave_path_id];
			ret = isp_path_storecrop_uinfo_set(slave_path, param);
		}
		pctx->updated = 1;
		mutex_unlock(&pctx->param_mutex);
		break;
	case ISP_PATH_CFG_PATH_COMPRESSION:
		ret = isp_path_store_compress_uinfo_set(path_info, param);
		break;
	case ISP_PATH_CFG_PATH_UFRAME_SYNC:
		ret = isp_path_storeframe_sync_set(path_info, param);
		break;
	case ISP_PATH_CFG_3DNR_MODE:
		mutex_lock(&pctx->param_mutex);
		pctx->uinfo.mode_3dnr = *(uint32_t *)param;
		mutex_unlock(&pctx->param_mutex);
		break;
	default:
		pr_warn("unsupported cmd: %d\n", cfg_cmd);
		break;
	}
exit:
	pr_debug("isp %d cfg path %d cmd %d done.\n",
			pctx->ctx_id, path->spath_id, cfg_cmd);
	return ret;
}

static int ispcore_statis_q_init(void *isp_handle, int ctx_id,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	int j;
	int32_t mfd;
	enum isp_statis_buf_type stats_type;
	struct isp_pipe_dev *dev = NULL;
	struct isp_sw_context *pctx;
	struct camera_buf *ion_buf;
	struct camera_frame *pframe;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	memset(&pctx->statis_buf_array[0], 0, sizeof(pctx->statis_buf_array));
	stats_type = STATIS_HIST2;

	for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
		mfd = input->mfd_array[stats_type][j];
		if (mfd <= 0)
			continue;
		ion_buf = &pctx->statis_buf_array[j];
		ion_buf->mfd[0] = mfd;
		ion_buf->offset[0] = input->offset_array[stats_type][j];
		ion_buf->type = CAM_BUF_USER;
		ret = cam_buf_ionbuf_get(ion_buf);
		if (ret) {
			memset(ion_buf, 0, sizeof(struct camera_buf));
			continue;
		}

		ret = cam_buf_kmap(ion_buf);
		if (ret) {
			cam_buf_ionbuf_put(ion_buf);
			memset(ion_buf, 0, sizeof(struct camera_buf));
			continue;
		}

		pframe = cam_queue_empty_frame_get();
		pframe->channel_id = pctx->ch_id;
		pframe->irq_property = stats_type;
		pframe->buf = *ion_buf;

		ret = cam_queue_enqueue(&pctx->hist2_result_queue, &pframe->list);
		if (ret) {
			pr_info("statis %d overflow\n", stats_type);
			cam_queue_empty_frame_put(pframe);
		}
		pr_debug("buf_num %d, buf %d, off %d, kaddr 0x%lx iova 0x%08x\n",
			j, ion_buf->mfd[0], ion_buf->offset[0],
			ion_buf->addr_k[0], (uint32_t)ion_buf->iova[0]);
	}

	return ret;
}

static int ispcore_statis_buffer_cfg(
		void *isp_handle, int ctx_id,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	int j;
	int32_t mfd;
	uint32_t offset;
	struct isp_pipe_dev *dev = NULL;
	struct isp_sw_context *pctx;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe;

	if (!isp_handle || ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_info("isp_handle=%p, cxt_id=%d\n", isp_handle, ctx_id);
		return 0;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	if (atomic_read(&pctx->user_cnt) == 0) {
		pr_info("isp ctx %d is not enable\n", ctx_id);
		return 0;
	}

	if (input->type == STATIS_INIT) {
		ispcore_statis_q_init(isp_handle, ctx_id, input);
		pr_debug("init done\n");
		return 0;
	}

	for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
		mfd = pctx->statis_buf_array[j].mfd[0];
		offset = pctx->statis_buf_array[j].offset[0];
		if ((mfd > 0) && (mfd == input->mfd)
			&& (offset == input->offset)) {
			ion_buf = &pctx->statis_buf_array[j];
			break;
		}
	}

	if (ion_buf == NULL) {
		pr_err("fail to get statis buf %d, type %d\n",
				input->type, input->mfd);
		ret = -EINVAL;
		return ret;
	}

	pframe = cam_queue_empty_frame_get();
	pframe->channel_id = pctx->ch_id;
	pframe->irq_property = STATIS_HIST2;
	pframe->buf = *ion_buf;
	ret = cam_queue_enqueue(&pctx->hist2_result_queue, &pframe->list);
	if (ret) {
		pr_info("statis %d overflow\n", input->type);
		cam_queue_empty_frame_put(pframe);
	}
	pr_debug("buf %d, off %d, kaddr 0x%lx iova 0x%08x\n",
		ion_buf->mfd[0], ion_buf->offset[0],
		ion_buf->addr_k[0], (uint32_t)ion_buf->iova[0]);
	return 0;
}

static int ispcore_statis_buffer_unmap(void *isp_handle, int ctx_id)
{
	int ret = 0;
	int j;
	struct isp_pipe_dev *dev = NULL;
	struct isp_sw_context *pctx;
	struct camera_buf *ion_buf;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	pr_debug("enter\n");

	for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
		ion_buf = &pctx->statis_buf_array[j];
		if (ion_buf->mfd[0] <= 0) {
			memset(ion_buf, 0, sizeof(struct camera_buf));
			continue;
		}

		pr_info("ctx %d free buf %d, off %d, kaddr 0x%lx iova 0x%08x\n",
			ctx_id, ion_buf->mfd[0], ion_buf->offset[0],
			ion_buf->addr_k[0], (uint32_t)ion_buf->iova[0]);

		cam_buf_kunmap(ion_buf);
		cam_buf_ionbuf_put(ion_buf);
		memset(ion_buf, 0, sizeof(struct camera_buf));
	}

	pr_debug("done.\n");
	return ret;
}

static int ispcore_sec_cfg(struct isp_pipe_dev *dev, void *param)
{

	enum sprd_cam_sec_mode *sec_mode = (enum sprd_cam_sec_mode *)param;

	dev->sec_mode = *sec_mode;

	pr_debug("camca: isp sec_mode=%d\n", dev->sec_mode);

	return 0;
}

static int ispcore_ioctl(void *isp_handle, int ctx_id,
		enum isp_ioctrl_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;

	switch (cmd) {
	case ISP_IOCTL_CFG_STATIS_BUF:
		ret = ispcore_statis_buffer_cfg(isp_handle, ctx_id, param);
		break;
	case ISP_IOCTL_CFG_SEC:
		ret = ispcore_sec_cfg(dev, param);
		break;
	default:
		pr_err("fail to get known cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

static int ispcore_blkparam_cfg(
	void *isp_handle, int ctx_id, void *param)
{
	int ret = 0;
	int i;
	struct isp_sw_context *pctx = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_io_param *io_param = NULL;
	func_isp_cfg_param cfg_fun_ptr = NULL;
	struct cam_hw_info *ops = NULL;
	struct isp_hw_block_func fucarg;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	ops = dev->isp_hw;
	pctx = dev->sw_ctx[ctx_id];
	io_param = (struct isp_io_param *)param;
	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) < 1) {
		pr_err("fail to use unable isp ctx %d.\n", ctx_id);
		mutex_unlock(&dev->path_mutex);
		return -EINVAL;
	}

	/* lock to avoid block param across frame */
	mutex_lock(&pctx->blkpm_lock);
	if (io_param->sub_block == ISP_BLOCK_3DNR) {
		if (pctx->uinfo.mode_3dnr != MODE_3DNR_OFF)
			ret = isp_k_cfg_3dnr(param, &pctx->isp_k_param, ctx_id);
	} else if (io_param->sub_block == ISP_BLOCK_NOISEFILTER) {
		if (io_param->scene_id == PM_SCENE_CAP)
			ret = isp_k_cfg_yuv_noisefilter(param, &pctx->isp_k_param, ctx_id);
	} else {
		i = io_param->sub_block - ISP_BLOCK_BASE;
		fucarg.index = i;
		ops->isp_ioctl(ops, ISP_HW_CFG_BLOCK_FUNC_GET, &fucarg);
		if (fucarg.isp_entry != NULL &&
			fucarg.isp_entry->sub_block == io_param->sub_block)
			cfg_fun_ptr = fucarg.isp_entry->cfg_func;

		if (cfg_fun_ptr == NULL) {
			pr_debug("isp block 0x%x is not supported.\n",
				io_param->sub_block);
			mutex_unlock(&pctx->blkpm_lock);
			mutex_unlock(&dev->path_mutex);
			return 0;
		}

		ret = cfg_fun_ptr(io_param, &pctx->isp_k_param, ctx_id);
	}

	mutex_unlock(&pctx->blkpm_lock);
	mutex_unlock(&dev->path_mutex);

	return ret;
}

static int ispcore_callback_set(void *isp_handle, int ctx_id,
		isp_dev_callback cb, void *priv_data)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_sw_context *pctx;

	if (!isp_handle || !cb || !priv_data) {
		pr_err("fail to get valid input ptr, isp_handle %p, callback %p, priv_data %p\n",
			isp_handle, cb, priv_data);
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];
	if (pctx->isp_cb_func == NULL) {
		pctx->isp_cb_func = cb;
		pctx->cb_priv_data = priv_data;
		pr_info("ctx: %d, cb %p, %p\n", ctx_id, cb, priv_data);
	}

	return ret;
}

/*
 * Get a free context and initialize it.
 * Input param is possible max_size of image.
 * Return valid context id, or -1 for failure.
 */
static int ispcore_context_get(void *isp_handle, void *param)
{
	int ret = 0;
	int i;
	int sel_ctx_id = 0;
	struct isp_sw_context *pctx = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_path_desc *path = NULL;
	struct cam_hw_info *hw = NULL;
	struct isp_cfg_ctx_desc *cfg_desc;
	struct isp_init_param *init_param;
	struct isp_hw_default_param dfult_param;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	pr_debug("start.\n");

	dev = (struct isp_pipe_dev *)isp_handle;
	hw = dev->isp_hw;
	init_param = (struct isp_init_param *)param;

	mutex_lock(&dev->path_mutex);
	sel_ctx_id = ispcore_sw_context_get(dev);
	if (sel_ctx_id == -1) {
		pr_err("fail to get valid ctx\n");
		goto exit;
	} else
		pctx = dev->sw_ctx[sel_ctx_id];

	if (dev->wmode == ISP_CFG_MODE) {
		cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
		cfg_desc->ops->ctx_reset(cfg_desc, sel_ctx_id);
	}

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		path->spath_id = i;
		path->attach_ctx = pctx;
		path->q_init = 0;
		path->hw = hw;
		atomic_set(&path->user_cnt, 0);
	}

	mutex_init(&pctx->param_mutex);
	mutex_init(&pctx->blkpm_lock);
	init_completion(&pctx->frm_done);
	init_completion(&pctx->slice_done);
	/* complete for first frame config */
	complete(&pctx->frm_done);

	init_isp_pm(&pctx->isp_k_param);
	/* bypass fbd_raw by default */
	pctx->pipe_info.fetch_fbd.fetch_fbd_bypass = 1;
	pctx->multi_slice = 0;
	pctx->started = 0;
	pctx->attach_cam_id = init_param->cam_id;
	pctx->ltm_ctx.ltm_index = pctx->attach_cam_id;
	pctx->uinfo.enable_slowmotion = 0;
	pctx->postproc_func = ispcore_postproc_irq;
	if (init_param->is_high_fps)
		pctx->uinfo.enable_slowmotion = hw->ip_isp->slm_cfg_support;
	pr_info("cam%d isp slowmotion eb %d\n",
		pctx->attach_cam_id, pctx->uinfo.enable_slowmotion);

	ret = ispcore_offline_thread_create(pctx);
	if (unlikely(ret != 0)) {
		pr_err("fail to create offline thread for isp cxt:%d\n",
				pctx->ctx_id);
		goto thrd_err;
	}

	if (init_param->is_high_fps == 0) {
		cam_queue_init(&pctx->in_queue,
			ISP_IN_Q_LEN, ispcore_src_frame_ret);
		cam_queue_init(&pctx->proc_queue,
			ISP_PROC_Q_LEN, ispcore_src_frame_ret);
	} else {
		cam_queue_init(&pctx->in_queue,
			ISP_SLW_IN_Q_LEN, ispcore_src_frame_ret);
		cam_queue_init(&pctx->proc_queue,
			ISP_SLW_PROC_Q_LEN, ispcore_src_frame_ret);
	}

	cam_queue_init(&pctx->stream_ctrl_in_q,
		ISP_STREAM_STATE_Q_LEN, cam_queue_empty_state_put);
	cam_queue_init(&pctx->stream_ctrl_proc_q,
		ISP_STREAM_STATE_Q_LEN, cam_queue_empty_state_put);
	cam_queue_init(&pctx->hist2_result_queue, 16,
		ispcore_statis_buf_destroy);
	dfult_param.type = ISP_CFG_PARA;
	dfult_param.index = pctx->ctx_id;
	hw->isp_ioctl(hw, ISP_HW_CFG_DEFAULT_PARA_SET, &dfult_param);
	isp_int_isp_irq_sw_cnt_reset(pctx->ctx_id);

	goto exit;

thrd_err:
	atomic_dec(&pctx->user_cnt); /* free context */
	sel_ctx_id = -1;
exit:
	mutex_unlock(&dev->path_mutex);
	pr_info("success to get context id %d\n", sel_ctx_id);
	return sel_ctx_id;
}

/*
 * Free a context and deinitialize it.
 * All paths of this context should be put before this.
 *
 * TODO:
 * we do not stop or reset ISP here because four contexts share it.
 * How to make sure current context process in ISP is done
 * before we clear buffer Q?
 * If ISP hw doesn't done buffer reading/writting,
 * we free buffer here may cause memory over-writting and perhaps system crash.
 * Delay buffer Q clear is a just solution to reduce this risk
 */
static int ispcore_context_put(void *isp_handle, int ctx_id)
{
	int ret = 0;
	int i;
	uint32_t loop = 0;
	struct isp_pipe_dev *dev;
	struct isp_sw_context *pctx;
	struct isp_path_desc *path;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = dev->sw_ctx[ctx_id];

	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) == 1)
		ispcore_offline_thread_stop(&pctx->thread);

	if (atomic_dec_return(&pctx->user_cnt) == 0) {
		pctx->started = 0;
		pr_info("free context %d without users.\n", pctx->ctx_id);

		/* make sure irq handler exit to avoid crash */
		while (pctx->in_irq_handler && (loop < 1000)) {
			pr_info("cxt % in irq. wait %d", pctx->ctx_id, loop);
			loop++;
			udelay(500);
		};

		if (pctx->slice_ctx)
			isp_slice_ctx_put(&pctx->slice_ctx);

		cam_queue_clear(&pctx->in_queue, struct camera_frame, list);
		cam_queue_clear(&pctx->proc_queue, struct camera_frame, list);
		cam_queue_clear(&pctx->hist2_result_queue,
			struct camera_frame, list);
		cam_queue_clear(&pctx->stream_ctrl_in_q,
			struct isp_stream_ctrl, list);
		cam_queue_clear(&pctx->stream_ctrl_proc_q,
			struct isp_stream_ctrl, list);

		dev->ltm_handle->ops->set_status(0, ctx_id, pctx->pipe_src.mode_ltm,
						pctx->attach_cam_id);
		if (pctx->uinfo.mode_ltm == MODE_LTM_PRE) {
			if (pctx->uinfo.ltm_rgb) {
				dev->ltm_handle->ops->clear_status(pctx->ltm_ctx.ltm_index);
				dev->ltm_handle->ops->complete_completion(LTM_RGB,
					pctx->ltm_ctx.ltm_index);
				for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
					if (pctx->ltm_buf[LTM_RGB][i]) {
						ispcore_frame_unmap(pctx->ltm_buf[LTM_RGB][i]);
						pctx->ltm_buf[LTM_RGB][i] = NULL;
					}
				}
			}

			if (pctx->uinfo.ltm_yuv) {
				dev->ltm_handle->ops->clear_status(pctx->ltm_ctx.ltm_index);
				dev->ltm_handle->ops->complete_completion(LTM_YUV,
					pctx->ltm_ctx.ltm_index);
				for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
					if (pctx->ltm_buf[LTM_YUV][i]) {
						ispcore_frame_unmap(pctx->ltm_buf[LTM_YUV][i]);
						pctx->ltm_buf[LTM_YUV][i] = NULL;
					}
				}
			}
		}

		if (pctx->uinfo.mode_ltm == MODE_LTM_CAP) {
			if (pctx->uinfo.ltm_rgb) {
				dev->ltm_handle->ops->clear_status(pctx->ltm_ctx.ltm_index);
				for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
					if (pctx->ltm_buf[LTM_RGB][i])
						pctx->ltm_buf[LTM_RGB][i] = NULL;
				}
			}

			if (pctx->uinfo.ltm_yuv) {
				dev->ltm_handle->ops->clear_status(pctx->ltm_ctx.ltm_index);
				for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
					if (pctx->ltm_buf[LTM_YUV][i])
						pctx->ltm_buf[LTM_YUV][i] = NULL;
				}
			}
		}

		for (i = 0; i < ISP_NR3_BUF_NUM; i++) {
			if (pctx->nr3_buf[i]) {
				ispcore_frame_unmap(pctx->nr3_buf[i]);
				pctx->nr3_buf[i] = NULL;
			}
		}

		/* clear path queue. */
		for (i = 0; i < ISP_SPATH_NUM; i++) {
			path = &pctx->isp_path[i];
			if (path->q_init == 0)
				continue;

			/* reserved buffer queue should be cleared at last. */
			cam_queue_clear(&path->result_queue,
				struct camera_frame, list);
			cam_queue_clear(&path->out_buf_queue,
				struct camera_frame, list);
			cam_queue_clear(&path->reserved_buf_queue,
				struct camera_frame, list);
		}

		if (pctx->postproc_buf) {
			ispcore_frame_unmap(pctx->postproc_buf);
			pctx->postproc_buf = NULL;
			pr_info("sw %d, postproc out buffer unmap\n", pctx->ctx_id);
		}

		pctx->postproc_func = NULL;
		pctx->isp_cb_func = NULL;
		pctx->cb_priv_data = NULL;
		isp_int_isp_irq_sw_cnt_trace(pctx->ctx_id);
		ispcore_statis_buffer_unmap(isp_handle, ctx_id);
	} else {
		pr_debug("ctx%d.already release.\n", ctx_id);
		atomic_set(&pctx->user_cnt, 0);
	}
	pctx->ctx_id = ctx_id;
	pctx->dev = dev;
	pctx->attach_cam_id = CAM_ID_MAX;
	pctx->hw = dev->isp_hw;

	mutex_unlock(&dev->path_mutex);
	pr_info("done, put ctx_id: %d\n", ctx_id);
	return ret;
}

static int ispcore_context_init(struct isp_pipe_dev *dev)
{
	int ret = 0;
	int i, bind_fmcu;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_hw_context *pctx_hw;
	struct cam_hw_info *hw = NULL;

	pr_info("isp contexts init start!\n");
	cam_queue_init(&dev->sw_ctx_q, ISP_SW_CONTEXT_Q_LEN,
		ispcore_sw_context_clear);

	/* CFG module init */
	if (dev->wmode == ISP_AP_MODE) {
		pr_info("isp ap mode.\n");
		for (i = 0; i < ISP_CONTEXT_SW_NUM; i++)
			isp_cfg_poll_addr[i] = &s_isp_regbase[0];

	} else {
		cfg_desc = isp_cfg_ctx_desc_get();
		if (!cfg_desc || !cfg_desc->ops) {
			pr_err("fail to get isp cfg ctx %p.\n", cfg_desc);
			ret = -EINVAL;
			goto cfg_null;
		}
		cfg_desc->hw = dev->isp_hw;
		pr_debug("cfg_init start.\n");

		ret = cfg_desc->ops->ctx_init(cfg_desc);
		if (ret) {
			pr_err("fail to cfg ctx init.\n");
			goto ctx_fail;
		}
		pr_debug("cfg_init done.\n");

		ret = cfg_desc->ops->hw_init(cfg_desc);
		if (ret)
			goto hw_fail;
	}
	dev->cfg_handle = cfg_desc;

	dev->ltm_handle = isp_ltm_share_ctx_desc_get();

	pr_info("isp hw contexts init start!\n");
	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		pctx_hw->hw_ctx_id = i;
		pctx_hw->sw_ctx_id = 0xffff;
		atomic_set(&pctx_hw->user_cnt, 0);

		hw = dev->isp_hw;
		hw->isp_ioctl(hw, ISP_HW_CFG_ENABLE_IRQ, &i);
		hw->isp_ioctl(hw, ISP_HW_CFG_CLEAR_IRQ, &i);

		bind_fmcu = 0;
		if (unlikely(dev->wmode == ISP_AP_MODE)) {
			/* for AP mode, multi-context is not supported */
			if (i != ISP_CONTEXT_HW_P0) {
				atomic_set(&pctx_hw->user_cnt, 1);
				continue;
			}
			bind_fmcu = 1;
		} else if (*(hw->ip_isp->ctx_fmcu_support + i)) {
			bind_fmcu = 1;
		}

		if (bind_fmcu) {
			fmcu = isp_fmcu_ctx_desc_get(hw, i);
			pr_debug("get fmcu %p\n", fmcu);

			if (fmcu && fmcu->ops) {
				fmcu->hw = dev->isp_hw;
				ret = fmcu->ops->ctx_init(fmcu);
				if (ret) {
					pr_err("fail to init fmcu ctx\n");
					isp_fmcu_ctx_desc_put(fmcu);
				} else {
					pctx_hw->fmcu_handle = fmcu;
				}
			} else {
				pr_info("no more fmcu or ops\n");
			}
		}
		pr_info("isp hw context %d init done. fmcu %p\n",
				i, pctx_hw->fmcu_handle);
	}

	pr_debug("done!\n");
	return 0;

hw_fail:
	cfg_desc->ops->ctx_deinit(cfg_desc);
ctx_fail:
	isp_cfg_ctx_desc_put(cfg_desc);
cfg_null:
	return ret;
}

static int ispcore_context_deinit(struct isp_pipe_dev *dev)
{
	int ret = 0;
	int i, j;
	uint32_t path_id;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_sw_context *pctx;
	struct isp_hw_context *pctx_hw;
	struct isp_path_desc *path;

	pr_debug("enter.\n");

	for (i = 0; i < ISP_SW_CONTEXT_NUM; i++) {
		pctx = dev->sw_ctx[i];
		if (!pctx)
			continue;

		/* free all used path here if user did not call put_path  */
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			path = &pctx->isp_path[j];
			path_id = path->spath_id;
			if (atomic_read(&path->user_cnt) > 0)
				ispcore_path_put(dev, pctx->ctx_id, path_id);
		}
		ispcore_context_put(dev, pctx->ctx_id);

		atomic_set(&pctx->user_cnt, 0);
		mutex_destroy(&pctx->param_mutex);
		mutex_destroy(&pctx->blkpm_lock);
	}
	cam_queue_clear(&dev->sw_ctx_q, struct isp_sw_context, list);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
		if (fmcu) {
			fmcu->ops->ctx_deinit(fmcu);
			isp_fmcu_ctx_desc_put(fmcu);
		}
		pctx_hw->fmcu_handle = NULL;
		pctx_hw->fmcu_used = 0;
		atomic_set(&pctx_hw->user_cnt, 0);
		isp_int_isp_irq_cnt_trace(i);
	}
	pr_info("isp contexts deinit done!\n");

	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
	if (cfg_desc) {
		cfg_desc->ops->ctx_deinit(cfg_desc);
		isp_cfg_ctx_desc_put(cfg_desc);
	}
	dev->cfg_handle = NULL;

	isp_ltm_share_ctx_desc_put(dev->ltm_handle);
	dev->ltm_handle = NULL;

	pr_debug("done.\n");
	return ret;
}

static int ispcore_dev_open(void *isp_handle, void *param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	pr_debug("enter.\n");
	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	hw = (struct cam_hw_info *)param;
	if (hw == NULL) {
		pr_err("fail to get valid hw\n");
		return -EFAULT;
	}

	if (atomic_inc_return(&dev->enable) == 1) {
		pr_info("open_start: sec_mode: %d, work mode: %d,  line_buf_len: %d\n",
			dev->sec_mode, dev->wmode, g_camctrl.isp_linebuf_len);

		/* line_buffer_len for debug */
		if (s_dbg_linebuf_len > 0 &&
			s_dbg_linebuf_len <= ISP_LINE_BUFFER_W)
			g_camctrl.isp_linebuf_len = s_dbg_linebuf_len;
		else
			g_camctrl.isp_linebuf_len = ISP_LINE_BUFFER_W;

		if (dev->sec_mode == SEC_SPACE_PRIORITY)
			dev->wmode = ISP_AP_MODE;
		else
			dev->wmode = s_dbg_work_mode;
		g_camctrl.isp_wmode = dev->wmode;

		dev->isp_hw = param;
		mutex_init(&dev->path_mutex);
		spin_lock_init(&dev->ctx_lock);

		ret = isp_drv_hw_init(dev);
		ret = ispcore_context_init(dev);
		if (ret) {
			pr_err("fail to init isp context.\n");
			ret = -EFAULT;
			goto err_init;
		}
	}

	pr_info("open isp pipe dev done!\n");
	return 0;

err_init:
	hw->isp_ioctl(hw, ISP_HW_CFG_STOP, NULL);
	isp_drv_hw_deinit(dev);
	atomic_dec(&dev->enable);
	pr_err("fail to open isp dev!\n");
	return ret;
}

static int ispcore_dev_close(void *isp_handle)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	hw = dev->isp_hw;
	if (atomic_dec_return(&dev->enable) == 0) {

		ret = hw->isp_ioctl(hw, ISP_HW_CFG_STOP, NULL);
		ret = ispcore_context_deinit(dev);
		mutex_destroy(&dev->path_mutex);
		ret = isp_drv_hw_deinit(dev);
	}

	pr_info("isp dev disable done\n");
	return ret;

}

static int ispcore_dev_reset(void *isp_handle, void *param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	int i = 0;
	struct isp_hw_context *pctx_hw = NULL;
	struct isp_hw_default_param tmp_default;


	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	hw = (struct cam_hw_info *)param;

	hw->isp_ioctl(hw, ISP_HW_CFG_RESET, NULL);
	tmp_default.type = ISP_HW_PARA;
	hw->isp_ioctl(hw, ISP_HW_CFG_DEFAULT_PARA_SET, &tmp_default);

	cfg_desc = dev->cfg_handle;
	if (cfg_desc && cfg_desc->ops)
		ret = cfg_desc->ops->hw_init(cfg_desc);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		pctx_hw->hw_ctx_id = i;
		pctx_hw->sw_ctx_id = 0xffff;
		atomic_set(&pctx_hw->user_cnt, 0);

		hw->isp_ioctl(hw, ISP_HW_CFG_CLEAR_IRQ, &i);
		hw->isp_ioctl(hw, ISP_HW_CFG_ENABLE_IRQ, &i);

		pr_debug("isp hw context %d init done. fmcu %p\n",
				i, pctx_hw->fmcu_handle);

		fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
		if (fmcu) {
			uint32_t val[2];
			unsigned long reg_offset = 0;
			uint32_t reg_bits[4] = { 0x00, 0x02, 0x01, 0x03};

			reg_offset = (fmcu->fid == 0) ?
						ISP_COMMON_FMCU0_PATH_SEL :
						ISP_COMMON_FMCU1_PATH_SEL;
			if (reg_offset == 0) {
				pr_info("no fmcu sel register\n");
				continue;
			}

			ISP_HREG_MWR(reg_offset, BIT_1 | BIT_0, reg_bits[i]);
			pr_debug("FMCU%d reg_bits %d for hw_ctx %d\n", fmcu->fid, reg_bits[i], i);

			val[0] = ISP_HREG_RD(ISP_COMMON_FMCU0_PATH_SEL);
			val[1] = ISP_HREG_RD(ISP_COMMON_FMCU1_PATH_SEL);

			if ((val[0] & 3) == (val[1] & 3)) {
				pr_warn("isp fmcus select same context %d\n", val[0] & 3);
				if (fmcu->fid == 0) {
					/* force to set different value */
					reg_offset = ISP_COMMON_FMCU1_PATH_SEL;
					ISP_HREG_MWR(reg_offset, BIT_1 | BIT_0, reg_bits[(i + 1) & 3]);
					pr_debug("force FMCU1 regbits %d\n", reg_bits[(i + 1) & 3]);
				}
			}
		}
	}
	sprd_iommu_restore(&hw->soc_isp->pdev->dev);

	pr_debug("isp dev reset done\n");
	return ret;
}

static struct isp_pipe_ops isp_ops = {
	.open = ispcore_dev_open,
	.close = ispcore_dev_close,
	.reset = ispcore_dev_reset,
	.get_context = ispcore_context_get,
	.put_context = ispcore_context_put,
	.get_path = ispcore_path_get,
	.put_path = ispcore_path_put,
	.cfg_path = ispcore_path_cfg,
	.ioctl = ispcore_ioctl,
	.cfg_blk_param = ispcore_blkparam_cfg,
	.proc_frame = ispcore_frame_proc,
	.set_callback = ispcore_callback_set,
	.clear_stream_ctrl = ispcore_stream_state_put,
};

void *isp_core_pipe_dev_get(void)
{
	struct isp_pipe_dev *dev = NULL;

	mutex_lock(&isp_pipe_dev_mutex);

	if (s_isp_dev) {
		atomic_inc(&s_isp_dev->user_cnt);
		dev = s_isp_dev;
		pr_info("s_isp_dev is already exist=%p, user_cnt=%d",
			s_isp_dev, atomic_read(&s_isp_dev->user_cnt));
		goto exit;
	}

	dev = vzalloc(sizeof(struct isp_pipe_dev));
	if (!dev)
		goto exit;

	atomic_set(&dev->user_cnt, 1);
	atomic_set(&dev->enable, 0);

	dev->isp_ops = &isp_ops;

	s_isp_dev = dev;
	if (dev)
		pr_info("get isp pipe dev: %p\n", dev);
exit:
	mutex_unlock(&isp_pipe_dev_mutex);

	return dev;
}

int isp_core_pipe_dev_put(void *isp_handle)
{
	int ret = 0;
	int user_cnt, en_cnt;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pr_info("put isp pipe dev:%p, s_isp_dev:%p,  users: %d\n",
		dev, s_isp_dev, atomic_read(&dev->user_cnt));

	mutex_lock(&isp_pipe_dev_mutex);

	if (dev != s_isp_dev) {
		mutex_unlock(&isp_pipe_dev_mutex);
		pr_err("fail to match dev: %p, %p\n",
					dev, s_isp_dev);
		return -EINVAL;
	}

	user_cnt = atomic_read(&dev->user_cnt);
	en_cnt = atomic_read(&dev->enable);
	if (user_cnt != (en_cnt + 1))
		pr_err("fail to match %d %d\n",
				user_cnt, en_cnt);

	if (atomic_dec_return(&dev->user_cnt) == 0) {
		pr_info("free isp pipe dev %p\n", dev);
		vfree(dev);
		dev = NULL;
		s_isp_dev = NULL;
	}
	mutex_unlock(&isp_pipe_dev_mutex);

	if (dev)
		pr_info("put isp pipe dev: %p\n", dev);

	return ret;
}

