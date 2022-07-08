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

#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/sprd_ion.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <isp_hw.h>
#include "sprd_img.h"
#include <video/sprd_mmsys_pw_domain.h>
#include <sprd_mm.h>

#include "cam_debugger.h"
#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_path.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

/* VCH2 maybe used for raw picture output
 * If yes, PDAF should not output data though VCH2
 * todo: avoid conflict between raw/pdaf type3
 */
struct statis_path_buf_info s_statis_path_info_all[] = {
	{DCAM_PATH_PDAF,    0,  0, STATIS_PDAF},
	{DCAM_PATH_VCH2,    0,  0, STATIS_EBD},
	{DCAM_PATH_AEM,     0,  0, STATIS_AEM},
	{DCAM_PATH_AFM,     0,  0, STATIS_AFM},
	{DCAM_PATH_AFL,     0,  0, STATIS_AFL},
	{DCAM_PATH_HIST,    0,  0, STATIS_HIST},
	{DCAM_PATH_3DNR,    0,  0, STATIS_3DNR},
	{DCAM_PATH_LSCM,    0,  0, STATIS_LSCM},
};

atomic_t s_dcam_axi_opened;
atomic_t s_dcam_opened[DCAM_ID_MAX];
static DEFINE_MUTEX(s_dcam_dev_mutex);
static struct dcam_pipe_dev *s_dcam_dev[DCAM_ID_MAX];

/*
 * set MIPI capture related register
 * range: 0x0100 ~ 0x010c
 *
 * TODO: support DCAM2, some registers only exist in DCAM0/1
 */

static int dcamcore_dcamsec_cfg(struct dcam_pipe_dev *dev, void *param)
{

	dev->dcamsec_eb = 0;

	pr_info("camca : dcamsec_mode=%d\n", dev->dcamsec_eb);
	return 0;
}

static int dcamcore_rps_cfg(struct dcam_pipe_dev *dev, void *param)
{
	uint32_t *data = (uint32_t *)param;

	dev->rps = *data;

	pr_info("hwsim : rps[%d]\n", dev->rps);
	return 0;
}

static int dcamcore_hw_slices_set(struct dcam_pipe_dev *dev,
	struct camera_frame *pframe, uint32_t slice_wmax)
{
	int ret = 0, i = 0;
	uint32_t w, offset = 0;
	uint32_t slc_w = 0, f_align;

	w = pframe->width;
	if (w <= slice_wmax) {
		slc_w = w;
		goto slices;
	}

	if (dev->fetch.pack_bits == 2)
		f_align = 8;  /* 8p * 16bits/p = 128 bits fetch aligned */
	else
		f_align = 64;/* 64p * 10bits/p = 128bits * 5 */

	slc_w = slice_wmax / f_align * f_align;

	/* can not get valid slice w aligned  */
	if ((slc_w > slice_wmax) ||
		(slc_w * DCAM_OFFLINE_SLC_MAX) < pframe->width) {
		pr_err("dcam%d failed, pic_w %d, slc_limit %d, algin %d\n",
			dev->idx, pframe->width, slice_wmax, f_align);
		return -EFAULT;
	}

slices:
	while (w > 0) {
		dev->hw_slices[i].start_x = offset;
		dev->hw_slices[i].start_y = 0;
		dev->hw_slices[i].size_x = (w > slc_w) ? slc_w : w;
		dev->hw_slices[i].size_y = pframe->height;
		pr_info("slc%d, (%d %d %d %d), limit %d\n", i,
			dev->hw_slices[i].start_x, dev->hw_slices[i].start_y,
			dev->hw_slices[i].size_x, dev->hw_slices[i].size_y,
			slice_wmax);

		w -= dev->hw_slices[i].size_x;
		offset += dev->hw_slices[i].size_x;
		i++;
	}
	dev->slice_num = i;
	dev->slice_count = i;

	return ret;
}

int dcamcore_slice_trim_get(uint32_t width, uint32_t heigth, uint32_t slice_num,
	uint32_t slice_no, struct img_trim *slice_trim)
{
	int ret = 0;
	uint32_t slice_w = 0, slice_h = 0;
	uint32_t slice_x_num = 0, slice_y_num = 0;
	uint32_t start_x = 0, size_x = 0;
	uint32_t start_y = 0, size_y = 0;

	if (!width || !slice_num || !slice_trim) {
		pr_err("fail to get valid param %d, %d, %p.\n", width, slice_num, slice_trim);
		return -EFAULT;
	}

	if (heigth > DCAM_SW_SLICE_HEIGHT_MAX) {
		slice_x_num = slice_num / 2;
		slice_y_num = 2;
	} else {
		slice_x_num = slice_num;
		slice_y_num = 1;
	}
	slice_w = width / slice_x_num;
	slice_w = ALIGN(slice_w, 2);
	slice_h = heigth / slice_y_num;
	slice_h = ALIGN(slice_h, 2);

	start_x = slice_w * (slice_no % slice_x_num);
	size_x = slice_w;
	if (size_x & 0x03) {
		if (slice_no != 0) {
			start_x -=  ALIGN(size_x, 4) - size_x;
			size_x = ALIGN(size_x, 4);
		} else {
			start_x -=  ALIGN(size_x, 4);
			size_x = ALIGN(size_x, 4);
		}
	}

	start_y = (slice_no / slice_x_num) * slice_h;
	size_y = slice_h;

	slice_trim->start_x = start_x;
	slice_trim->size_x = size_x;
	slice_trim->start_y = start_y;
	slice_trim->size_y = size_y;
	pr_debug("slice %d [%d, %d, %d, %d].\n", slice_no, start_x, size_x, start_y, size_y);
	return ret;
}

void dcamcore_src_frame_ret(void *param)
{
	struct camera_frame *frame;
	struct dcam_pipe_dev *dev;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	dev = (struct dcam_pipe_dev *)frame->priv_data;
	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);

	cam_buf_iommu_unmap(&frame->buf);
	dev->dcam_cb_func(
		DCAM_CB_RET_SRC_BUF,
		frame, dev->cb_priv_data);
}

void dcamcore_out_frame_ret(void *param)
{
	struct camera_frame *frame;
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;

	if (frame->is_reserved) {
		path = (struct dcam_path_desc *)frame->priv_data;
		cam_queue_enqueue(&path->reserved_buf_queue, &frame->list);
	} else {
		cam_buf_iommu_unmap(&frame->buf);
		dev = (struct dcam_pipe_dev *)frame->priv_data;
		dev->dcam_cb_func(DCAM_CB_DATA_DONE, frame, dev->cb_priv_data);
	}
}

void dcamcore_reserved_buf_destroy(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;

	if (unlikely(frame->is_reserved == 0)) {
		pr_err("fail to check frame type if reserved.\n");
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

static struct camera_buf *dcamcore_reserved_buffer_get(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int iommu_enable = 0; /* todo: get from dev dts config value */
	size_t size;
	struct camera_buf *ion_buf = NULL;

	ion_buf = kzalloc(sizeof(*ion_buf), GFP_KERNEL);
	if (!ion_buf) {
		pr_err("fail to alloc buffer.\n");
		goto nomem;
	}

	if (cam_buf_iommu_status_get(CAM_IOMMUDEV_DCAM) == 0)
		iommu_enable = 1;

	size = DCAM_INTERNAL_RES_BUF_SIZE;
	ret = cam_buf_alloc(ion_buf, size, 0, iommu_enable);
	if (ret) {
		pr_err("fail to get dcam reserverd buffer\n");
		goto ion_fail;
	}

	pr_info("dcam%d done. ion %p\n", dev->idx, ion_buf);
	return ion_buf;

ion_fail:
	kfree(ion_buf);
nomem:
	return NULL;
}

static int dcamcore_reserved_buffer_put(struct dcam_pipe_dev *dev)
{
	struct camera_buf *ion_buf = NULL;

	ion_buf = (struct camera_buf *)dev->internal_reserved_buf;
	if (!ion_buf) {
		pr_info("no reserved buffer.\n");
		return 0;
	}
	pr_info("dcam%d, ion %p\n", dev->idx, ion_buf);

	if (ion_buf->type != CAM_BUF_USER)
		cam_buf_free(ion_buf);

	kfree(ion_buf);
	dev->internal_reserved_buf = NULL;

	return 0;
}

static void dcamcore_reserved_statis_bufferq_init(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int i, j;
	enum isp_statis_buf_type stats_type;
	struct camera_frame *newfrm;
	enum dcam_path_id path_id;
	struct camera_buf *ion_buf = NULL;
	struct dcam_path_desc *path;

	pr_info("enter\n");

	if (dev->internal_reserved_buf == NULL) {
		ion_buf = dcamcore_reserved_buffer_get(dev);
		if (IS_ERR_OR_NULL(ion_buf))
			return;

		dev->internal_reserved_buf = ion_buf;
	}
	ion_buf = (struct camera_buf *)dev->internal_reserved_buf;

	if (ion_buf->type == CAM_BUF_USER) {
		ret = cam_buf_ionbuf_get(ion_buf);
		if (ret) {
			pr_err("fail to get buf for %d\n", ion_buf->mfd[0]);
			return;
		}
		pr_debug("reserverd statis buffer get %p\n", ion_buf);
	}

	ret = cam_buf_iommu_map(ion_buf, CAM_IOMMUDEV_DCAM);
	if (ret) {
		pr_err("fail to map dcam reserved buffer to iommu\n");
		return;
	}

	for (i = 0; i < (int)ARRAY_SIZE(s_statis_path_info_all); i++) {
		path_id = s_statis_path_info_all[i].path_id;
		stats_type = s_statis_path_info_all[i].buf_type;
		if (!stats_type)
			continue;

		path = &dev->path[path_id];
		if (path_id == DCAM_PATH_VCH2 && path->src_sel)
			continue;
		j  = 0;
		while (j < DCAM_RESERVE_BUF_Q_LEN) {
			newfrm = cam_queue_empty_frame_get();
			if (newfrm) {
				newfrm->is_reserved = 1;
				memcpy(&newfrm->buf, ion_buf, sizeof(struct camera_buf));
				cam_queue_enqueue(&path->reserved_buf_queue, &newfrm->list);
				j++;
			}
			pr_debug("path%d reserved buffer %d\n", path->path_id, j);
		}
	}
	pr_info("done\n");
}

static enum dcam_path_id dcamcore_statis_type_to_path_id(enum isp_statis_buf_type type)
{
	switch (type) {
	case STATIS_AEM:
		return DCAM_PATH_AEM;
	case STATIS_AFM:
		return DCAM_PATH_AFM;
	case STATIS_AFL:
		return DCAM_PATH_AFL;
	case STATIS_HIST:
		return DCAM_PATH_HIST;
	case STATIS_PDAF:
		return DCAM_PATH_PDAF;
	case STATIS_EBD:
		return DCAM_PATH_VCH2;
	case STATIS_3DNR:
		return DCAM_PATH_3DNR;
	case STATIS_LSCM:
		return DCAM_PATH_LSCM;
	default:
		return DCAM_PATH_MAX;
	}
}

void dcamcore_statis_buf_destroy(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cam_queue_empty_frame_put(frame);
}

static int dcamcore_statis_bufferq_init(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int i, j;
	enum dcam_path_id path_id;
	enum isp_statis_buf_type stats_type;
	struct camera_buf *ion_buf;
	struct camera_frame *pframe;
	struct dcam_path_desc *path;

	pr_debug("enter\n");

	for (i = 0; i < ARRAY_SIZE(s_statis_path_info_all); i++) {
		path_id = s_statis_path_info_all[i].path_id;
		stats_type = s_statis_path_info_all[i].buf_type;
		if (!stats_type)
			continue;

		path = &dev->path[path_id];
		if (path_id == DCAM_PATH_VCH2 && path->src_sel)
			continue;
		cam_queue_init(&path->out_buf_queue,
			DCAM_OUT_BUF_Q_LEN, dcamcore_statis_buf_destroy);
		cam_queue_init(&path->result_queue,
			DCAM_RESULT_Q_LEN, dcamcore_statis_buf_destroy);
		cam_queue_init(&path->reserved_buf_queue,
			DCAM_RESERVE_BUF_Q_LEN,
			dcamcore_statis_buf_destroy);
	}

	dcamcore_reserved_statis_bufferq_init(dev);

	for (i = 0; i < ARRAY_SIZE(s_statis_path_info_all); i++) {

		path_id = s_statis_path_info_all[i].path_id;
		stats_type = s_statis_path_info_all[i].buf_type;
		if (!stats_type)
			continue;

		path = &dev->path[path_id];
		if (path_id == DCAM_PATH_VCH2 && path->src_sel)
			continue;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &dev->statis_buf_array[stats_type][j];
			if (ion_buf->mfd[0] <= 0)
				continue;

			ret = cam_buf_ionbuf_get(ion_buf);
			if (ret) {
				continue;
			}

			ret = cam_buf_iommu_map(ion_buf, CAM_IOMMUDEV_DCAM);
			if (ret) {
				cam_buf_ionbuf_put(ion_buf);
				continue;
			}

			if (stats_type != STATIS_PDAF) {
				ret = cam_buf_kmap(ion_buf);
				if (ret) {
					pr_err("fail to kmap statis buf %d\n", ion_buf->mfd[0]);
					memset(ion_buf->addr_k, 0, sizeof(ion_buf->addr_k));
				}
			}

			pframe = cam_queue_empty_frame_get();
			pframe->channel_id = path_id;
			pframe->irq_property = stats_type;
			pframe->buf = *ion_buf;

			ret = cam_queue_enqueue(&path->out_buf_queue, &pframe->list);
			if (ret) {
				pr_info("dcam%d statis %d overflow\n", dev->idx, stats_type);
				cam_queue_empty_frame_put(pframe);
			}

			pr_debug("dcam%d statis %d buf %d kaddr 0x%lx iova 0x%08x\n",
				dev->idx, stats_type, ion_buf->mfd[0],
				ion_buf->addr_k[0], (uint32_t)ion_buf->iova[0]);
		}
	}

	pr_info("done.\n");
	return ret;
}

static int dcamcore_statis_bufferq_deinit(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int i;
	enum dcam_path_id path_id;
	struct dcam_path_desc *path;

	pr_info("enter\n");

	for (i = 0; i < ARRAY_SIZE(s_statis_path_info_all); i++) {
		path_id = s_statis_path_info_all[i].path_id;
		path = &dev->path[path_id];

		if (path_id == DCAM_PATH_VCH2 && path->src_sel)
			continue;

		pr_debug("path_id[%d] i[%d]\n", path_id, i);
		if (path_id == DCAM_PATH_MAX)
			continue;

		atomic_set(&path->user_cnt, 0);
		cam_queue_clear(&path->out_buf_queue, struct camera_frame, list);
		cam_queue_clear(&path->result_queue, struct camera_frame, list);
		cam_queue_clear(&path->reserved_buf_queue,
			struct camera_frame, list);
	}

	pr_info("done.\n");
	return ret;
}

static int dcamcore_statis_buffer_unmap(struct dcam_pipe_dev *dev)
{
	int i, j;
	int32_t mfd;
	enum dcam_path_id path_id;
	enum isp_statis_buf_type stats_type;
	struct camera_buf *ion_buf = NULL;
	struct dcam_path_desc *path;

	for (i = 0; i < ARRAY_SIZE(s_statis_path_info_all); i++) {
		path_id = s_statis_path_info_all[i].path_id;
		stats_type = s_statis_path_info_all[i].buf_type;
		path = &dev->path[path_id];
		if (!stats_type)
			continue;
		if (path_id == DCAM_PATH_VCH2 && path->src_sel)
			continue;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &dev->statis_buf_array[stats_type][j];
			mfd = ion_buf->mfd[0];
			if (mfd <= 0)
				continue;

			pr_debug("stats %d,  j %d,  mfd %d, offset %d\n",
				stats_type, j, mfd, ion_buf->offset[0]);
			if (ion_buf->mapping_state & CAM_BUF_MAPPING_KERNEL)
				cam_buf_kunmap(ion_buf);
			cam_buf_iommu_unmap(ion_buf);
			cam_buf_ionbuf_put(ion_buf);
			memset(ion_buf->iova, 0, sizeof(ion_buf->iova));
			memset(ion_buf->addr_k, 0, sizeof(ion_buf->addr_k));
		}
	}

	if (dev->internal_reserved_buf) {
		ion_buf = dev->internal_reserved_buf;
		pr_debug("reserved statis buffer unmap %p\n", ion_buf);

		cam_buf_iommu_unmap(ion_buf);
		if (ion_buf->type == CAM_BUF_USER)
			cam_buf_ionbuf_put(ion_buf);
	}

	pr_info("done\n");
	return 0;
}

static int dcamcore_statis_buffer_cfg(
		struct dcam_pipe_dev *dev,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	int i, j;
	int32_t mfd;
	uint32_t offset;
	enum dcam_path_id path_id;
	enum isp_statis_buf_type stats_type;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_path_desc *path = NULL;

	if (input->type == STATIS_INIT) {
		memset(&dev->statis_buf_array[0][0], 0, sizeof(dev->statis_buf_array));
		for (i = 0; i < ARRAY_SIZE(s_statis_path_info_all); i++) {
			path_id = s_statis_path_info_all[i].path_id;
			stats_type = s_statis_path_info_all[i].buf_type;
			if (!stats_type)
				continue;
			path = &dev->path[path_id];
			if (path_id == DCAM_PATH_VCH2 && path->src_sel)
				continue;

			for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
				mfd = input->mfd_array[stats_type][j];

				pr_debug("i %d, type %d, mfd %d, offset %d\n",
					i, stats_type, mfd, input->offset_array[stats_type][j]);

				if (mfd <= 0)
					continue;

				ion_buf = &dev->statis_buf_array[stats_type][j];
				ion_buf->mfd[0] = mfd;
				ion_buf->offset[0] = input->offset_array[stats_type][j];
				ion_buf->type = CAM_BUF_USER;

				pr_debug("stats %d, mfd %d, off %d\n",
					stats_type, mfd, ion_buf->offset[0]);
			}
		}
		pr_info("done\n");

	} else if (atomic_read(&dev->state) == STATE_RUNNING) {

		path_id = dcamcore_statis_type_to_path_id(input->type);
		if (path_id == DCAM_PATH_MAX) {
			pr_err("fail to get a valid statis type: %d\n", input->type);
			ret = -EINVAL;
			goto exit;
		}

		path = &dev->path[path_id];
		if (path_id == DCAM_PATH_VCH2 && path->src_sel)
			goto exit;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			mfd = dev->statis_buf_array[input->type][j].mfd[0];
			offset = dev->statis_buf_array[input->type][j].offset[0];
			if ((mfd > 0) && (mfd == input->mfd)
				&& (offset == input->offset)) {
				ion_buf = &dev->statis_buf_array[input->type][j];
				break;
			}
		}

		if (ion_buf == NULL) {
			pr_err("fail to get statis buf %d, type %d\n",
					input->type, input->mfd);
			ret = -EINVAL;
			goto exit;
		}

		pframe = cam_queue_empty_frame_get();
		pframe->irq_property = input->type;
		pframe->buf = *ion_buf;
		path = &dev->path[path_id];
		ret = cam_queue_enqueue(&path->out_buf_queue, &pframe->list);
		pr_debug("statis %d, mfd %d, off %d, iova 0x%08x,  kaddr 0x%lx\n",
			input->type, mfd, offset,
			(uint32_t)pframe->buf.iova[0], pframe->buf.addr_k[0]);

		if (ret)
			cam_queue_empty_frame_put(pframe);
	}
exit:
	return ret;
}

static int dcamcore_statis_buffer_skip_cfg(struct dcam_pipe_dev *dev, struct camera_frame *pframe)
{
	int ret = 0;
	enum dcam_path_id path_id;

	path_id = dcamcore_statis_type_to_path_id(pframe->irq_property);
	if (path_id == DCAM_PATH_MAX) {
		pr_err("invalid statis type: %d\n", pframe->irq_property);
		ret = -EINVAL;
		goto exit;
	}

	ret = cam_queue_enqueue(&dev->path[path_id].out_buf_queue, &pframe->list);
exit:
	return ret;
}

static int dcamcore_context_init(
		struct dcam_pipe_dev *dev,
		struct dcam_pipe_context *pctx)
{
	int ret = 0;
	int iommu_enable = 0;
	struct dcam_dev_param *blk_pm_ctx = &pctx->blk_pm;

	memset(blk_pm_ctx, 0, sizeof(struct dcam_dev_param));
	if (cam_buf_iommu_status_get(CAM_IOMMUDEV_DCAM) == 0)
		iommu_enable = 1;

	ret = cam_buf_alloc(&blk_pm_ctx->lsc.buf, DCAM_LSC_BUF_SIZE,
		0, iommu_enable);
	if (ret)
		goto alloc_fail;

	ret = cam_buf_kmap(&blk_pm_ctx->lsc.buf);
	if (ret)
		goto map_fail;

	blk_pm_ctx->offline = (pctx->ctx_id == 0) ? 0 : 1;
	blk_pm_ctx->idx = dev->idx;
	blk_pm_ctx->dev = (void *)dev;

	atomic_set(&pctx->user_cnt, 1);
	mutex_init(&blk_pm_ctx->lsc.lsc_lock);
	mutex_init(&blk_pm_ctx->param_lock);

	init_dcam_pm(blk_pm_ctx);

	pr_info("dcam%d ctx %d done\n", dev->idx, pctx->ctx_id);
	return 0;

map_fail:
	cam_buf_free(&blk_pm_ctx->lsc.buf);
alloc_fail:
	pr_err("failed %d\n", ret);
	return ret;
}

static int dcamcore_context_deinit(struct dcam_pipe_context *pctx)
{
	struct dcam_dev_param *blk_pm_ctx = &pctx->blk_pm;
	struct dcam_pipe_dev *dev = NULL;

	dev = (struct dcam_pipe_dev *)blk_pm_ctx->dev;

	mutex_destroy(&blk_pm_ctx->param_lock);
	mutex_destroy(&blk_pm_ctx->lsc.lsc_lock);

	if (blk_pm_ctx->lsc.buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&blk_pm_ctx->lsc.buf);
	cam_buf_kunmap(&blk_pm_ctx->lsc.buf);
	cam_buf_free(&blk_pm_ctx->lsc.buf);
	atomic_set(&pctx->user_cnt, 0);

	pr_info("dcam%d ctx %d done\n", dev->idx, pctx->ctx_id);
	return 0;
}

static void dcamcore_offline_debug_dump(
	struct dcam_pipe_dev *dev,
	struct dcam_dev_param *pm,
	struct camera_frame *proc_frame)
{
	int size;
	struct camera_frame *frame = NULL;
	struct debug_base_info *base_info;
	void *pm_data;

	dev->dcam_cb_func(DCAM_CB_GET_PMBUF,
		(void *)&frame, dev->cb_priv_data);
	if (frame == NULL) {
		pr_debug("no pmbuf for dcam%d\n", dev->idx);
		return;
	}

	base_info = (struct debug_base_info *)frame->buf.addr_k[0];
	if (base_info == NULL) {
		cam_queue_empty_frame_put(frame);
		return;
	}
	base_info->cam_id = -1;
	base_info->dcam_cid = dev->idx | (proc_frame->irq_property << 8);
	base_info->isp_cid = -1;
	base_info->scene_id = PM_SCENE_CAP;
	if (proc_frame->irq_property == CAM_FRAME_FDRL)
		base_info->scene_id = PM_SCENE_FDRL;
	if (proc_frame->irq_property == CAM_FRAME_FDRH)
		base_info->scene_id = PM_SCENE_FDRH;

	base_info->frame_id = proc_frame->fid;
	base_info->sec = proc_frame->sensor_time.tv_sec;
	base_info->usec = proc_frame->sensor_time.tv_usec;
	frame->fid = proc_frame->fid;
	frame->sensor_time = proc_frame->sensor_time;

	pm_data = (void *)(base_info + 1);
	size = dcam_k_dump_pm(pm_data, (void *)pm);
	if (size >= 0)
		base_info->size = (int32_t)size;
	else
		base_info->size = 0;

	pr_debug("dcam_cxt %d, scene %d  fid %d  dsize %d\n",
		base_info->dcam_cid, base_info->scene_id,
		base_info->frame_id, base_info->size);

	dev->dcam_cb_func(DCAM_CB_STATIS_DONE,
		frame, dev->cb_priv_data);
}

static int dcamcore_offline_slices_sw_start(void *param)
{
	int ret = 0;
	int i, loop;
	uint32_t force_ids = DCAM_CTRL_ALL;
	uint32_t dev_fid;
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_path_desc *path;
	struct dcam_fetch_info *fetch;
	struct dcam_hw_force_copy copyarg;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_path_start patharg;
	struct dcam_hw_slice_fetch slicearg;
	uint32_t slice_no;

	dev = (struct dcam_pipe_dev *)param;
	dev->offline = 1;
	fetch = &dev->fetch;
	hw = dev->hw;

	if (dev->slice_count) {
		pframe = cam_queue_dequeue(&dev->proc_queue,
				struct camera_frame, list);
		if (!pframe) {
			pr_err("fail to map buf to dcam%d iommu.\n", dev->idx);
			goto map_err;
		}
	} else {
		pframe = cam_queue_dequeue(&dev->in_queue, struct camera_frame, list);
		if (pframe == NULL) {
			pr_warn("no frame from in_q. dcam%d\n", dev->idx);
			return 0;
		}

		pr_debug("frame %p, dcam%d  ch_id %d.  buf_fd %d\n", pframe,
			dev->idx, pframe->channel_id, pframe->buf.mfd[0]);
		pr_debug("size %d %d,  endian %d, pattern %d\n",
			pframe->width, pframe->height, pframe->endian, pframe->pattern);

		ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_DCAM);
		if (ret) {
			pr_err("fail to map buf to dcam%d iommu.\n", dev->idx);
			goto map_err;
		}

		dev->slice_count = dev->slice_num;
	}
	slice_no = dev->slice_num - dev->slice_count;

	ret = wait_for_completion_interruptible_timeout(
		&dev->slice_done, DCAM_OFFLINE_TIMEOUT);
	if (ret == ERESTARTSYS) {
		pr_err("interrupt when dcam wait\n");
		ret = -EFAULT;
		goto wait_err;
	} else if (ret == 0) {
		pr_err("error: dcam%d offline timeout.\n", dev->idx);
		ret = -EFAULT;
		goto wait_err;
	}
	ret = 0;

	loop = 0;
	do {
		ret = cam_queue_enqueue(&dev->proc_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info("wait for proc queue. loop %d\n", loop);

		/* wait for previous frame proccessed done. */
		mdelay(1);
	} while (loop++ < 500);
	if (ret) {
		pr_err("error: input frame queue tmeout.\n");
		ret = 0;
		goto inq_overflow;
	}

	/* prepare frame info for tx done
	 * ASSERT: this dev has no cap_sof
	 */
	dev->index_to_set = pframe->fid - dev->base_fid;
	dev_fid = dev->index_to_set;
	if (pframe->sensor_time.tv_sec || pframe->sensor_time.tv_usec) {
		dev->frame_ts[tsid(dev_fid)].tv_sec =
			pframe->sensor_time.tv_sec;
		dev->frame_ts[tsid(dev_fid)].tv_nsec =
			pframe->sensor_time.tv_usec * NSEC_PER_USEC;
		dev->frame_ts_boot[tsid(dev_fid)] = pframe->boot_sensor_time;
		pr_info("frame[%d]\n", pframe->fid);
	}
	/* cfg path output and path */
	for (i  = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		if (atomic_read(&path->user_cnt) < 1 || atomic_read(&path->is_shutoff) > 0)
			continue;
		ret = dcam_path_store_frm_set(dev, path, NULL); /* TODO: */
		if (ret == 0) {
			/* interrupt need > 1 */
			atomic_set(&path->set_frm_cnt, 1);
			atomic_inc(&path->set_frm_cnt);
			patharg.path_id = i;
			patharg.idx = dev->idx;
			patharg.slowmotion_count = dev->slowmotion_count;
			patharg.pdaf_path_eb = 0;
			patharg.cap_info = dev->cap_info;
			patharg.pack_bits = dev->path[i].pack_bits;
			patharg.src_sel = dev->path[i].src_sel;
			patharg.bayer_pattern = dev->path[i].bayer_pattern;
			patharg.in_trim = dev->path[i].in_trim;
			patharg.endian = dev->path[i].endian;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &patharg);
		} else {
			pr_err("fail to set dcam%d path%d store frm\n",
				dev->idx, path->path_id);
			ret = 0;
			goto dequeue;
		}
	}

	dcamcore_slice_trim_get(pframe->width, pframe->height, dev->slice_num,
		slice_no, &dev->slice_trim);
	dev->cur_slice = &dev->slice_trim;

	/* cfg fetch */
	fetch->pack_bits = 0;
	fetch->endian = pframe->endian;
	fetch->pattern = pframe->pattern;
	fetch->size.w = pframe->width;
	fetch->size.h = pframe->height;
	fetch->trim.start_x = 0;
	fetch->trim.start_y = 0;
	fetch->trim.size_x = pframe->width;
	fetch->trim.size_y = pframe->height;
	fetch->addr.addr_ch0 = (uint32_t)pframe->buf.iova[0];
	slicearg.idx = dev->idx;
	slicearg.fetch = &dev->fetch;
	slicearg.cur_slice = dev->cur_slice;
	slicearg.slice_trim = dev->slice_trim;
	slicearg.dcam_slice_mode = dev->dcam_slice_mode;
	slicearg.slice_count = dev->slice_count;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_SLICE_FETCH_SET, &slicearg);

	dcam_init_lsc_slice(&dev->ctx[dev->cur_ctx_id].blk_pm, 0);
	copyarg.id = force_ids;
	copyarg.idx = dev->idx;
	copyarg.glb_reg_lock = dev->glb_reg_lock;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_FORCE_COPY, &copyarg);
	udelay(500);
	dev->iommu_status = (uint32_t)(-1);
	atomic_set(&dev->state, STATE_RUNNING);
	dev->err_count = 1;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_START, hw);
	pr_debug("done slice %d.\n", slice_no);

	return 0;

dequeue:
	pframe = cam_queue_dequeue_tail(&dev->proc_queue);
inq_overflow:
wait_err:
	cam_buf_iommu_unmap(&pframe->buf);
	complete(&dev->slice_done);
	complete(&dev->frm_done);
map_err:
	dev->slice_num = 0;
	dev->slice_count = 0;
	dev->dcam_cb_func(DCAM_CB_RET_SRC_BUF, pframe, dev->cb_priv_data);
	return ret;
}

static int dcamcore_offline_frame_start(void *param)
{
	int ret = 0;
	int i, loop;
	uint32_t force_ids = DCAM_CTRL_ALL;
	uint32_t dev_fid;
	uint32_t loose_val = 0;
	uint32_t val_4in1 = 0;
	uint32_t dev_lbuf, multi_cxt;
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_path_desc *path = NULL;
	struct dcam_fetch_info *fetch = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_reg_trace trace;
	struct dcam_hw_force_copy copyarg;
	struct dcam_hw_fetch_set fetcharg;
	struct dcam_hw_path_start patharg;
	struct dcam_hw_fetch_block blockarg;
	struct cam_hw_lbuf_share lbufarg;
	struct dcam_hw_slice_fetch slicearg;
	struct dcam_pipe_context *pctx = NULL;
	struct dcam_dev_param *pm;

	dev = (struct dcam_pipe_dev *)param;
	dev->offline = 1;
	hw = dev->hw;
	fetch = &dev->fetch;
	pr_debug("dcam%d enter\n", dev->idx);

	ret = wait_for_completion_interruptible_timeout(
		&dev->frm_done,
		DCAM_OFFLINE_TIMEOUT);
	if (ret == ERESTARTSYS) {
		pr_err("fail to wait as interrupted.\n");
		ret = -EFAULT;
		pframe = cam_queue_dequeue(&dev->in_queue, struct camera_frame, list);
		if (pframe == NULL) {
			pr_warn("no frame from in_q. dcam%d\n", dev->idx);
			return 0;
		}
		goto wait_err;
	} else if (ret == 0) {
		pr_err("fail to wait as dcam%d offline timeout.\n", dev->idx);
		ret = -EFAULT;
		pframe = cam_queue_dequeue(&dev->in_queue, struct camera_frame, list);
		if (pframe == NULL) {
			pr_warn("no frame from in_q. dcam%d\n", dev->idx);
			return 0;
		}
		goto wait_err;
	}

	if (dev->dcam_slice_mode == CAM_OFFLINE_SLICE_SW) {
		ret = dcamcore_offline_slices_sw_start(param);
		return ret;
	}
	dev->dcam_slice_mode = CAM_SLICE_NONE;

	pframe = cam_queue_dequeue(&dev->in_queue, struct camera_frame, list);
	if (pframe == NULL) {
		pr_warn("no frame from in_q. dcam%d\n", dev->idx);
		return 0;
	}

	if (DCAM_FETCH_TWICE(dev)) {
		dev->raw_fetch_count++;
		blockarg.idx = dev->idx;
		blockarg.raw_fetch_count = dev->raw_fetch_count;
		ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_BLOCK_SET, &blockarg);
		if (!DCAM_FIRST_FETCH(dev)) {
			struct camera_frame *frame = NULL;

			frame = cam_queue_dequeue(&dev->proc_queue,
					struct camera_frame, list);
			if (!frame) {
				goto wait_err;
			} else {
				path = &dev->path[DCAM_PATH_BIN];
				ret = cam_queue_enqueue(&path->out_buf_queue, &frame->list);
				pframe->endian = frame->endian;
				pframe->pattern = frame->pattern;
				pframe->width = frame->width;
				pframe->height = frame->height;
			}
		}
	}

	lbufarg.idx = dev->idx;
	lbufarg.width = 0;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_LBUF_SHARE_GET, &lbufarg);
	dev_lbuf = lbufarg.width;
	if (!dev_lbuf)
		dev_lbuf = DCAM_PATH_WMAX * 2;
	if (pframe->width > dev_lbuf) {
		if (hw->ip_dcam[dev->idx]->offline_slice_support == 0) {
			pr_err("dcam%d failed, width %d > lbuf %d\n",
				dev->idx, pframe->width, dev_lbuf);
			ret = -EFAULT;
			goto map_err;
		}

		pr_info("dcam%d, frame w %d over line buf len %d",
			dev->idx, pframe->width, dev_lbuf);
		dev->dcam_slice_mode = CAM_OFFLINE_SLICE_HW;
	}

	/* FDR solution: select context for frame type */
	dev->cur_ctx_id = DCAM_CXT_0;
	if (pframe->irq_property != CAM_FRAME_COMMON) {
		dev->cur_ctx_id = DCAM_CXT_1;
		udelay(800);
	}
	pctx = &dev->ctx[dev->cur_ctx_id];
	pm = &pctx->blk_pm;
	if ((pm->lsc.buf.mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
		ret = cam_buf_iommu_map(&pm->lsc.buf, CAM_IOMMUDEV_DCAM);
		if (ret)
			pm->lsc.buf.iova[0] = 0L;
	}

	pr_info("dcam%d frm %p, ch_id %d.  type %d, buf_fd %d\n",
		dev->idx, pframe, pframe->channel_id,
		pframe->irq_property, pframe->buf.mfd[0]);
	pr_info("slice %d, size %d %d,  endian %d, pattern %d\n",
		dev->dcam_slice_mode, pframe->width, pframe->height,
		pframe->endian, pframe->pattern);

	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_DCAM);
	if (ret) {
		pr_err("fail to map buf to dcam%d iommu.\n", dev->idx);
		goto map_err;
	}

	loop = 0;
	do {
		ret = cam_queue_enqueue(&dev->proc_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info("wait for proc queue. loop %d\n", loop);

		/* wait for previous frame proccessed done. */
		mdelay(1);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to enqueue frame as timeout.\n");
		ret = 0;
		goto inq_overflow;
	}

	/* todo: enable statis path from user config */
	pr_debug("hwsim:rps[%d]\n", dev->rps);

	if (dev->rps == 1) {
		pr_debug("hwsim:offline enable aem\n");
		atomic_set(&dev->path[DCAM_PATH_AEM].user_cnt, 1);/* hwsim first loop need aem statis */
	} else {
		atomic_set(&dev->path[DCAM_PATH_AEM].user_cnt, 0);
	}

	atomic_set(&dev->path[DCAM_PATH_PDAF].user_cnt, 0);
	atomic_set(&dev->path[DCAM_PATH_AFM].user_cnt, 0);
	atomic_set(&dev->path[DCAM_PATH_AFL].user_cnt, 0);
	atomic_set(&dev->path[DCAM_PATH_HIST].user_cnt, 0);
	atomic_set(&dev->path[DCAM_PATH_LSCM].user_cnt, 0);

	/* prepare frame info for tx done
	 * ASSERT: this dev has no cap_sof
	 */
	dev->index_to_set = pframe->fid - dev->base_fid;
	dev_fid = dev->index_to_set;
	if (pframe->sensor_time.tv_sec || pframe->sensor_time.tv_usec) {
		dev->frame_ts[tsid(dev_fid)].tv_sec =
			pframe->sensor_time.tv_sec;
		dev->frame_ts[tsid(dev_fid)].tv_nsec =
			pframe->sensor_time.tv_usec * NSEC_PER_USEC;
		dev->frame_ts_boot[tsid(dev_fid)] = pframe->boot_sensor_time;
		pr_info("frame[%d]\n", pframe->fid);
	}

	for (i = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		if (atomic_read(&path->user_cnt) < 1 || atomic_read(&path->is_shutoff) > 0)
			continue;
		ret = dcam_path_store_frm_set(dev, path, NULL); /* TODO: */
		if (ret == 0) {
			/* interrupt need > 1 */
			if (i == DCAM_PATH_FULL || i == DCAM_PATH_BIN) {
				if(dev->rps == 1)
					loose_val = (loose_val | (dev->pack_bits) | (path->pack_bits));
				else
					loose_val = ((dev->pack_bits) | (path->pack_bits));
			}
			path->pack_bits = loose_val;
			val_4in1 = ((dev->is_4in1) | (path->is_4in1));
			atomic_set(&path->set_frm_cnt, 1);
			atomic_inc(&path->set_frm_cnt);
			patharg.path_id = i;
			patharg.idx = dev->idx;
			patharg.slowmotion_count = dev->slowmotion_count;
			patharg.pdaf_path_eb = 0;
			patharg.cap_info = dev->cap_info;
			patharg.pack_bits = dev->path[i].pack_bits;
			patharg.src_sel = dev->path[i].src_sel;
			patharg.bayer_pattern = dev->path[i].bayer_pattern;
			patharg.in_trim = dev->path[i].in_trim;
			patharg.endian = dev->path[i].endian;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &patharg);
		} else {
			pr_err("fail to set dcam%d path%d store frm\n",
				dev->idx, path->path_id);
			ret = 0;
			goto dequeue;
		}
	}

	/* todo - need to cfg fetch param from input or frame. */
	fetch->pack_bits = loose_val;
	if(val_4in1 == 1) {
		if(dev->rps == 1)
			fetch->pack_bits = loose_val;
		else
			fetch->pack_bits = 0;
	}
	pr_info("pack_bits =%d",fetch->pack_bits);
	fetch->endian = pframe->endian;
	fetch->pattern = pframe->pattern;
	fetch->size.w = pframe->width;
	fetch->size.h = pframe->height;
	fetch->trim.start_x = 0;
	fetch->trim.start_y = 0;
	fetch->trim.size_x = pframe->width;
	fetch->trim.size_y = pframe->height;
	fetch->addr.addr_ch0 = (uint32_t)pframe->buf.iova[0];
	ret = dcamcore_hw_slices_set(dev, pframe, dev_lbuf);
	if (ret)
		goto dequeue;

	multi_cxt = 0;
	for (i = 0; i < DCAM_CXT_NUM; i++) {
		if (atomic_read(&dev->ctx[i].user_cnt) > 0)
			multi_cxt++;
	}
	multi_cxt = (multi_cxt < 2) ? 0 : 1;

	/* for multi-contexts: */
	/* bypass all blks and then set all blks to current pm */
	if (multi_cxt) {
		pr_info("dcam%d multi context, re-config all blks\n", dev->idx);
		hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETALL, pm);
	}

	for (i = 0; i < dev->slice_num; i++) {
		ret = wait_for_completion_interruptible_timeout(
			&dev->slice_done, DCAM_OFFLINE_TIMEOUT);
		if (ret == ERESTARTSYS) {
			pr_err("fail to wait as interrupted\n");
			ret = -EFAULT;
			goto dequeue;
		} else if (ret == 0) {
			pr_err("fail to wait as dcam%d offline timeout.\n", dev->idx);
			ret = -EFAULT;
			goto dequeue;
		}
		ret = 0;
		dev->cur_slice = &dev->hw_slices[i];
		pr_info("slice%d proc start\n", i);

		if (dev->slice_num <= 1) {
			fetcharg.idx = dev->idx;
			fetcharg.fetch_info = fetch;
			ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_SET, &fetcharg);
		} else if (hw->ip_dcam[dev->idx]->offline_slice_support) {
			slicearg.idx = dev->idx;
			slicearg.fetch = fetch;
			slicearg.cur_slice = dev->cur_slice;
			slicearg.slice_trim = dev->slice_trim;
			slicearg.dcam_slice_mode = dev->dcam_slice_mode;
			slicearg.slice_count = dev->slice_count;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_SLICE_FETCH_SET, &slicearg);
		}

		if (pframe->irq_property != CAM_FRAME_FDRH) {
			mutex_lock(&pm->lsc.lsc_lock);
			if (i == 0)
				dcam_init_lsc(pm, 0);
			else
				dcam_init_lsc_slice(pm, 0);
			mutex_unlock(&pm->lsc.lsc_lock);
		}

		/* DCAM_CTRL_COEF will always set in dcam_init_lsc() */
		//force_ids &= ~DCAM_CTRL_COEF;
		copyarg.id = force_ids;
		copyarg.idx = dev->idx;
		copyarg.glb_reg_lock = dev->glb_reg_lock;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_FORCE_COPY, &copyarg);
		udelay(500);
		dev->iommu_status = (uint32_t)(-1);
		dev->err_count = 1;
		atomic_set(&dev->state, STATE_RUNNING);
		pr_info("slice%d fetch start\n", i);

		if (i == 0) {
			trace.type = NORMAL_REG_TRACE;
			trace.idx = dev->idx;
			hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
			dcamcore_offline_debug_dump(dev, pm, pframe);
		}

		/* start fetch */
		if (dev->dcamsec_eb)
			pr_warn("camca : dcam%d sec_eb= %d, fetch disable\n",
				dev->idx, dev->dcamsec_eb);
		else
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_START, hw);
	}

	return 0;

dequeue:
	pframe = cam_queue_dequeue_tail(&dev->proc_queue);
inq_overflow:
	cam_buf_iommu_unmap(&pframe->buf);
map_err:
	complete(&dev->slice_done);
	complete(&dev->frm_done);
wait_err:
	dev->slice_num = 0;
	dev->slice_count = 0;
	/* return buffer to cam channel shared buffer queue. */
	dev->dcam_cb_func(DCAM_CB_RET_SRC_BUF, pframe, dev->cb_priv_data);
	return 0;
}

static int dcamcore_offline_thread_loop(void *arg)
{
	struct dcam_pipe_dev *dev = NULL;
	struct cam_thread_info *thrd;
	uint32_t idx = 0;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_thread_info *)arg;
	dev = (struct dcam_pipe_dev *)thrd->ctx_handle;
	idx = dev->idx;

	init_completion(&dev->frm_done);
	complete(&dev->frm_done);
	pr_info("dcam%d\n", dev->idx);
	init_completion(&dev->slice_done);
	complete(&dev->slice_done);
	while (1) {
		if (wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(
				&thrd->thread_stop, 1, 0) == 1) {
				pr_info("dcam%d offline thread stop.\n", idx);
				break;
			}
			pr_debug("thread com done.\n");

			if (thrd->proc_func(dev)) {
				pr_err("fail to start dcam pipe to proc. exit thread\n");
				dev->dcam_cb_func(DCAM_CB_DEV_ERR, dev, dev->cb_priv_data);
				break;
			}
		} else {
			pr_debug("offline thread exit!");
			break;
		}
	}

	return 0;
}

static int dcamcore_offline_thread_stop(void *param)
{
	int ret = 0;
	int cnt = 0;
	struct cam_thread_info *thrd;
	struct dcam_pipe_dev *dev;

	thrd = (struct cam_thread_info *)param;
	dev = (struct dcam_pipe_dev *)thrd->ctx_handle;

	if (thrd->thread_task) {
		atomic_set(&thrd->thread_stop, 1);
		complete(&thrd->thread_com);
		while (cnt < 1000) {
			cnt++;
			if (atomic_read(&thrd->thread_stop) == 0)
				break;
			udelay(1000);
		}
		thrd->thread_task = NULL;
		pr_info("offline thread stopped. wait %d ms\n", cnt);
	} else {
		pr_info("dcam%d no offline thread\n", dev->idx);
		return 0;
	}

	/* wait for last frame done */
	ret = wait_for_completion_interruptible_timeout(
		&dev->frm_done, DCAM_OFFLINE_TIMEOUT);
	if (ret == -ERESTARTSYS)
		pr_err("interrupt when isp wait\n");
	else if (ret == 0)
		pr_err("dcam%d timeout.\n", dev->idx);
	else
		pr_info("wait time %d\n", ret);

	return 0;
}

static int dcamcore_offline_thread_create(void *param)
{
	struct dcam_pipe_dev *dev;
	struct cam_thread_info *thrd;
	char thread_name[32] = { 0 };

	dev = (struct dcam_pipe_dev *)param;
	thrd = &dev->thread;
	thrd->ctx_handle = dev;
	thrd->proc_func = dcamcore_offline_frame_start;
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);

	sprintf(thread_name, "dcam%d_offline", dev->idx);
	thrd->thread_task = kthread_run(dcamcore_offline_thread_loop,
						thrd, "%s", thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start offline thread for dcam%d\n",
			dev->idx);
		return -EFAULT;
	}

	pr_info("dcam%d offline thread created.\n", dev->idx);
	return 0;
}

/*
 * Helper function to initialize dcam_sync_helper.
 */
static inline void dcamcore_sync_helper_locked_init(struct dcam_pipe_dev *dev,
			struct dcam_sync_helper *helper)
{
	memset(&helper->sync, 0, sizeof(struct dcam_frame_synchronizer));
	helper->enabled = 0;
	helper->dev = dev;
}

/*
 * Initialize frame synchronizer helper.
 */
static inline int dcamcore_sync_helper_init(struct dcam_pipe_dev *dev)
{
	unsigned long flags = 0;
	int i = 0;

	INIT_LIST_HEAD(&dev->helper_list);

	spin_lock_irqsave(&dev->helper_lock, flags);

	for (i = 0; i < DCAM_SYNC_HELPER_COUNT; i++) {
		dcamcore_sync_helper_locked_init(dev, &dev->helpers[i]);
		list_add_tail(&dev->helpers[i].list, &dev->helper_list);
	}
	spin_unlock_irqrestore(&dev->helper_lock, flags);

	return 0;
}

/*
 * Helper function to put dcam_sync_helper.
 */
static inline void dcamcore_sync_helper_locked_put(struct dcam_pipe_dev *dev,
			struct dcam_sync_helper *helper)
{
	dcamcore_sync_helper_locked_init(dev, helper);
	list_add_tail(&helper->list, &dev->helper_list);
}

/*
 * Enables/Disables frame sync for path_id. Should be called before streaming.
 */
int dcamcore_dcam_if_sync_enable_set(void *handle, int path_id, int enable)
{
	struct dcam_pipe_dev *dev = NULL;
	int ret = 0;

	if (unlikely(!handle)) {
		pr_err("fail to get valid param.\n");
		return -EINVAL;
	}
	dev = (struct dcam_pipe_dev *)handle;

	if (unlikely(!is_path_id(path_id))) {
		pr_err("fail to get valid param path_id: %d\n", path_id);
		return -EINVAL;
	}

	ret = atomic_read(&dev->state);
	if (unlikely(ret != STATE_INIT && ret != STATE_IDLE)) {
		pr_err("fail to get a valid ret, DCAM%u frame sync in %d state\n",
			dev->idx, ret);
		return -EINVAL;
	}

	if (enable)
		dev->helper_enabled |= BIT(path_id);
	else
		dev->helper_enabled &= ~BIT(path_id);

	pr_info("DCAM%u %s %s frame sync\n", dev->idx,
		dcam_path_name_get(path_id), enable ? "enable" : "disable");

	return 0;
}

/*
 * Release frame sync reference for @frame thus dcam_frame_synchronizer data
 * can be recycled for next use.
 */
int dcam_core_dcam_if_release_sync(struct dcam_frame_synchronizer *sync,
		struct camera_frame *frame)
{
	struct dcam_sync_helper *helper = NULL;
	struct dcam_pipe_dev *dev = NULL;
	unsigned long flags = 0;
	int ret = 0, path_id = 0;
	bool ignore = false;

	if (unlikely(!sync || !frame)) {
		pr_err("fail to get valid param. sync=%p, frame=%p\n", sync, frame);
		return -EINVAL;
	}

	helper = container_of(sync, struct dcam_sync_helper, sync);
	dev = helper->dev;

	for (path_id = 0; path_id < DCAM_PATH_MAX; path_id++) {
		if (frame == sync->frames[path_id])
			break;
	}

	if (unlikely(!is_path_id(path_id))) {
		pr_err("fail to get a valid path_id, DCAM%u can't find path id, fid %u, sync %u\n",
			dev->idx, frame->fid, sync->index);
		return -EINVAL;
	}

	/* unbind each other */
	frame->sync_data = NULL;
	sync->frames[path_id] = NULL;

	pr_debug("DCAM%u %s release sync, id %u, data 0x%p\n",
		dev->idx, dcam_path_name_get(path_id), sync->index, sync);

	spin_lock_irqsave(&dev->helper_lock, flags);
	if (unlikely(!helper->enabled)) {
		ignore = true;
		goto exit;
	}
	helper->enabled &= ~BIT(path_id);
	if (!helper->enabled)
		dcamcore_sync_helper_locked_put(dev, helper);

exit:
	spin_unlock_irqrestore(&dev->helper_lock, flags);

	if (ignore)
		pr_warn("ignore not enabled sync helper\n");

	return ret;
}

/*
 * Get an empty dcam_sync_helper. Returns NULL if no empty helper remains.
 */
struct dcam_sync_helper *dcam_core_sync_helper_get(struct dcam_pipe_dev *dev)
{
	struct dcam_sync_helper *helper = NULL;
	unsigned long flags = 0;
	bool running_low = false;

	if (unlikely(!dev)) {
		pr_err("fail to get valid param.\n");
		return NULL;
	}

	spin_lock_irqsave(&dev->helper_lock, flags);
	if (unlikely(list_empty(&dev->helper_list))) {
		running_low = true;
		goto exit;
	}

	helper = list_first_entry(&dev->helper_list,
			struct dcam_sync_helper, list);
	list_del_init(&helper->list);

exit:
	spin_unlock_irqrestore(&dev->helper_lock, flags);

	if (running_low)
		pr_warn_ratelimited("DCAM%u helper is running low...\n", dev->idx);

	return helper;
}

/*
 * Put an empty dcam_sync_helper.
 *
 * In CAP_SOF, when the requested empty dcam_sync_helper finally not being used,
 * it should be returned to FIFO. This only happens when no buffer is
 * available and all paths are using reserved memory.
 *
 * This is also an code defect in CAP_SOF that should be changed later...
 */
void dcam_core_sync_helper_put(struct dcam_pipe_dev *dev,
		struct dcam_sync_helper *helper)
{
	unsigned long flags = 0;

	if (unlikely(!dev)) {
		pr_err("fail to get valid param.\n");
		return;
	}

	spin_lock_irqsave(&dev->helper_lock, flags);
	dcamcore_sync_helper_locked_put(dev, helper);
	spin_unlock_irqrestore(&dev->helper_lock, flags);
}

static int dcamcore_param_cfg(void *dcam_handle, void *param)
{
	int ret = 0;
	uint32_t i = 0;
	func_dcam_cfg_param cfg_fun_ptr = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct isp_io_param *io_param;
	struct dcam_dev_param *pm = NULL;
	struct cam_hw_info *ops = NULL;
	struct dcam_hw_block_func_get get;
	struct dcam_pipe_context *pctx;

	if (!dcam_handle || !param) {
		pr_err("fail to get valid param, dev %p, param %p\n",
			dcam_handle, param);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	ops = dev->hw;
	io_param = (struct isp_io_param *)param;

	pctx = &dev->ctx[DCAM_CXT_0];
	if (io_param->scene_id == PM_SCENE_FDRL)
		pctx = &dev->ctx[DCAM_CXT_1];
	if (io_param->scene_id == PM_SCENE_FDRH)
		pctx = &dev->ctx[DCAM_CXT_2];
	pm = &pctx->blk_pm;

	if (atomic_read(&pctx->user_cnt) == 0) {
		pr_info("dcam%d ctx %d not init\n", dev->idx, pctx->ctx_id);
		ret = dcamcore_context_init(dev, pctx);
		if (ret)
			goto exit;
	}

	i = io_param->sub_block - DCAM_BLOCK_BASE;
	get.index = i;
	ops->dcam_ioctl(ops, DCAM_HW_CFG_BLOCK_FUNC_GET, &get);
	if (get.dcam_entry != NULL &&
		get.dcam_entry->sub_block == io_param->sub_block) {
		cfg_fun_ptr = get.dcam_entry->cfg_func;
	} else { /* if not, some error */
		pr_err("fail to check param, io_param->sub_block = %d, error\n", io_param->sub_block);
	}
	if (cfg_fun_ptr == NULL) {
		pr_debug("block %d not supported.\n", io_param->sub_block);
		goto exit;
	}

	if (dev->dcam_slice_mode && dev->slice_count > 0
		&& (io_param->sub_block != DCAM_BLOCK_LSC))
		return 0;

	if (io_param->sub_block == DCAM_BLOCK_LSC)
		mutex_lock(&pm->lsc.lsc_lock);

	pm->dcam_slice_mode = dev->dcam_slice_mode;
	ret = cfg_fun_ptr(io_param, pm);

	if ((io_param->sub_block == DCAM_BLOCK_LSC) &&
		(dev->offline == 0) &&
		(atomic_read(&dev->state) == STATE_RUNNING)) {
		dcam_update_lsc(pm);
	}
    
	if (io_param->sub_block == DCAM_BLOCK_LSC)
		mutex_unlock(&pm->lsc.lsc_lock);
exit:
	return ret;
}

static int dcamcore_param_reconfig(
	struct dcam_pipe_dev *dev, uint32_t cxt_id)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_pipe_context *pctx = NULL;
	struct dcam_dev_param *pm;

	pr_info("dcam%d,  cxt_id %d\n", dev->idx, cxt_id);
	if (cxt_id > DCAM_CXT_3) {
		pr_err("invalid context id\n");
		return -1;
	}

	dev->cur_ctx_id = cxt_id;
	pctx = &dev->ctx[dev->cur_ctx_id];
	pm = &pctx->blk_pm;
	hw = dev->hw;

	if (atomic_read(&pctx->user_cnt) <= 0) {
		pr_err("context%d is not in use\n", cxt_id);
		return -1;
	}

	hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETSTATIS, pm);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETALL, pm);

	pr_info("dcam%d done\n", dev->idx);
	return ret;
}

static inline void dcamcore_frame_info_show(struct dcam_pipe_dev *dev,
			struct dcam_path_desc *path,
			struct camera_frame *frame)
{
	uint32_t size = 0, pack_bits = 0;

	pack_bits = path->pack_bits;
	if (frame->is_compressed)
		size = dcam_if_cal_compressed_size(frame->width,
			frame->height, frame->compress_4bit_bypass);
	else
		size = cal_sprd_raw_pitch(frame->width, pack_bits) * frame->height;

	pr_debug("DCAM%u %s frame %u %u size %u %u buf %08lx %08x\n",
		dev->idx, dcam_path_name_get(path->path_id),
		frame->is_reserved, frame->is_compressed,
		frame->width, frame->height,
		frame->buf.iova[0], size);
}

/* offline process frame */
static int dcamcore_frame_proc(
		void *dcam_handle, void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *pframe;

	if (!dcam_handle || !param) {
		pr_err("fail to get a valid param, dcam_handle=%p, param=%p\n", dcam_handle, param);
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;

	pr_debug("dcam%d offline proc frame!\n", dev->idx);
	/* if enable, 4in1 capture once more then dcam1 can't run
	 * if (atomic_read(&dev->state) == STATE_RUNNING) {
	 *	pr_err("DCAM%u started for online\n", dev->idx);
	 *	return -EFAULT;
	 * }
	 */
	pframe = (struct camera_frame *)param;
	pframe->priv_data = dev;

	if (dev->slice_count != 0 &&
		dev->dcam_slice_mode == CAM_OFFLINE_SLICE_SW) {
		complete(&dev->thread.thread_com);
		return ret;
	}

	ret = cam_queue_enqueue(&dev->in_queue, &pframe->list);
	if (ret == 0)
		complete(&dev->thread.thread_com);
	else
		pr_err("fail to enqueue frame to dev->in_queue, ret = %d\n", ret);

	return ret;
}

static int dcamcore_path_get(
	void *dcam_handle, int path_id)
{
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path = NULL;

	if (!dcam_handle) {
		pr_err("fail to get valid param, dcam_handle=%p\n", dcam_handle);
		return -EFAULT;
	}
	if (path_id < DCAM_PATH_FULL || path_id >= DCAM_PATH_MAX) {
		pr_err("fail to get a valid path_id, path id %d\n", path_id);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	path = &dev->path[path_id];
	if (atomic_inc_return(&path->user_cnt) > 1) {
		atomic_dec(&path->user_cnt);
		pr_err("fail to get a valid param, dcam%d path %d in use.\n", dev->idx, path_id);
		return -EFAULT;
	}

	cam_queue_init(&path->result_queue, DCAM_RESULT_Q_LEN,
		dcamcore_out_frame_ret);
	cam_queue_init(&path->out_buf_queue, DCAM_OUT_BUF_Q_LEN,
		dcamcore_out_frame_ret);
	cam_queue_init(&path->alter_out_queue, DCAM_OUT_BUF_Q_LEN,
		dcamcore_out_frame_ret);
	cam_queue_init(&path->reserved_buf_queue, DCAM_RESERVE_BUF_Q_LEN,
		dcamcore_reserved_buf_destroy);

	return 0;
}

static int dcamcore_path_put(void *dcam_handle, int path_id)
{
	int ret = 0;
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path = NULL;

	if (!dcam_handle) {
		pr_err("fail to get a valid param,  dcam_handle=%p\n",
			dcam_handle);
		return -EFAULT;
	}
	if (path_id < DCAM_PATH_FULL || path_id >= DCAM_PATH_MAX) {
		pr_err("fail to get a valid param, path id %d\n", path_id);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	path = &dev->path[path_id];

	if (atomic_read(&path->user_cnt) == 0) {
		pr_debug("fail to get a valid user_cnt, dcam%d path %d is not in use.\n",
			dev->idx, path_id);
		return -EFAULT;
	}

	if (atomic_dec_return(&path->user_cnt) != 0) {
		pr_warn("dcam%d path %d has multi users.\n",
			dev->idx, path_id);
		atomic_set(&path->user_cnt, 0);
	}

	cam_queue_clear(&path->result_queue, struct camera_frame, list);
	cam_queue_clear(&path->out_buf_queue, struct camera_frame, list);
	cam_queue_clear(&path->alter_out_queue, struct camera_frame, list);
	cam_queue_clear(&path->reserved_buf_queue, struct camera_frame, list);

	pr_info("put dcam%d path %d done\n", dev->idx, path_id);
	return ret;
}

static int dcamcore_path_cfg(void *dcam_handle, enum dcam_path_cfg_cmd cfg_cmd,
	int path_id, void *param)
{
	int ret = 0;
	int i = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_desc *path = NULL;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_path_src_sel patharg;
	struct camera_frame *pframe = NULL, *newfrm = NULL;
	uint32_t lowlux_4in1 = 0;
	uint32_t shutoff = 0;
	unsigned long flag = 0;
	enum dcam_path_state state = DCAM_PATH_IDLE;

	static const char *tb_src[] = {"(4c)raw", "bin-sum"};/* for log */

	if (!dcam_handle || !param) {
		pr_err("fail to get a valid param, input param: %p, %p\n",
			dcam_handle, param);
		return -EFAULT;
	}
	if (path_id < DCAM_PATH_FULL || path_id >= DCAM_PATH_MAX) {
		pr_err("fail to get a valid param, dcam path id %d\n", path_id);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	hw = dev->hw;
	path = &dev->path[path_id];

	if (atomic_read(&path->user_cnt) == 0) {
		pr_err("fail to get a valid user_cnt, dcam%d, path %d is not in use.\n",
			dev->idx, path_id);
		return -EFAULT;
	}

	switch (cfg_cmd) {
	case DCAM_PATH_CFG_OUTPUT_BUF:
		pframe = (struct camera_frame *)param;
		if (path_id != DCAM_PATH_FULL) {
			ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_DCAM);
			if (ret)
				goto exit;
		}

		if (atomic_read(&dev->state) != STATE_RUNNING)
			dcamcore_frame_info_show(dev, path, pframe);

		pframe->is_reserved = 0;
		pframe->priv_data = dev;
		ret = cam_queue_enqueue(&path->out_buf_queue, &pframe->list);
		if (ret) {
			pr_err("fail to enqueue frame of dcam path %d\n",
				path_id);
			cam_buf_iommu_unmap(&pframe->buf);
			goto exit;
		}
		pr_debug("config dcam output buffer.\n");
		break;

	case DCAM_PATH_CFG_OUTPUT_ALTER_BUF:
		pframe = (struct camera_frame *)param;
		pframe->is_reserved = 0;
		pframe->priv_data = dev;
		ret = cam_queue_enqueue(&path->alter_out_queue, &pframe->list);
		if (ret) {
			pr_err("fail to enqueue frame of dcam path %d\n",
				path_id);
			goto exit;
		}
		pr_info("config dcam alter output buffer %d\n", pframe->buf.mfd[0]);
		break;

	case DCAM_PATH_CLR_OUTPUT_ALTER_BUF:
		do {
			pframe = cam_queue_dequeue(&path->alter_out_queue,
				struct camera_frame, list);
			if (pframe == NULL)
				break;
			pr_info("clr fdr raw buf fd %d, type %d, mapping %x\n",
				pframe->buf.mfd[0], pframe->buf.type, pframe->buf.mapping_state);
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);

		} while (1);

		break;

	case DCAM_PATH_CFG_OUTPUT_RESERVED_BUF:
		pframe = (struct camera_frame *)param;
		ret = cam_buf_iommu_single_page_map(&pframe->buf,
			CAM_IOMMUDEV_DCAM);
		if (ret) {
			pr_err("fail to map dcam path %d reserve buffer.\n", path_id);
			goto exit;
		}

		pframe->is_reserved = 1;
		pframe->priv_data = path;
		ret = cam_queue_enqueue(&path->reserved_buf_queue, &pframe->list);
		if (ret) {
			pr_err("fail to enqueue frame of dcam path %d reserve buffer.\n",
				path_id);
			cam_buf_iommu_unmap(&pframe->buf);
			goto exit;
		}

		pr_info("config dcam output reserverd buffer.\n");

		i = 1;
		while (i < DCAM_RESERVE_BUF_Q_LEN) {
			newfrm = cam_queue_empty_frame_get();
			if (newfrm) {
				newfrm->is_reserved = 2;
				newfrm->priv_data = path;
				memcpy(&newfrm->buf, &pframe->buf,
					sizeof(pframe->buf));
				ret = cam_queue_enqueue(
					&path->reserved_buf_queue,
					&newfrm->list);
				i++;
			}
		}
		break;
	case DCAM_PATH_CFG_SIZE:
		ret = dcam_path_size_cfg(dev, path, param);
		break;
	case DCAM_PATH_CFG_BASE:
		ret = dcam_path_base_cfg(dev, path, param);
		break;
	case DCAM_PATH_CFG_FULL_SOURCE:
		lowlux_4in1 = *(uint32_t *)param;

		if (lowlux_4in1) {
			dev->lowlux_4in1 = 1;
			patharg.src_sel = PROCESS_RAW_SRC_SEL;
			patharg.idx = dev->idx;
			ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SRC_SEL, &patharg);
			dev->skip_4in1 = 1; /* auto copy, so need skip 1 frame */
		} else {
			dev->lowlux_4in1 = 0;
			patharg.src_sel = PROCESS_RAW_SRC_SEL;
			patharg.idx = dev->idx;
			ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SRC_SEL, &patharg);
			dev->skip_4in1 = 1;
		}
		pr_info("dev%d lowlux %d, skip_4in1 %d, full src: %s\n", dev->idx,
			dev->lowlux_4in1, dev->skip_4in1, tb_src[lowlux_4in1]);
		break;
	case DCAM_PATH_CFG_SHUTOFF:
		shutoff = *(uint32_t *)param;
		atomic_set(&path->is_shutoff, shutoff);
		pr_debug("set path %d shutoff %d\n", path_id, shutoff);
		break;
	case DCAM_PATH_CFG_STATE:
		state = *(uint32_t *)param;
		spin_lock_irqsave(&path->state_lock, flag);
		path->state = state;
		path->state_update = 1;
		spin_unlock_irqrestore(&path->state_lock, flag);
		break;
	default:
		pr_warn("unsupported command: %d\n", cfg_cmd);
		break;
	}
exit:
	return ret;
}

/* get path rect from register
 */
static int dcamcore_path_rect_get(struct dcam_pipe_dev *dev, void *param)
{
	struct sprd_img_path_rect *p = (struct sprd_img_path_rect *)param;
	struct dcam_path_desc *path;
	struct dcam_dev_aem_win *aem_win;
	struct isp_img_rect *afm_crop;
	struct dcam_dev_param *pm;

	if ((!dev) || (!param)) {
		pr_err("fail to get valid param, dev=%p, param=%p\n", dev, param);
		return -EINVAL;
	}
	path = &dev->path[DCAM_PATH_BIN];
	p->trim_valid_rect.x = path->in_trim.start_x;
	p->trim_valid_rect.y = path->in_trim.start_y;
	p->trim_valid_rect.w = path->in_trim.size_x;
	p->trim_valid_rect.h = path->in_trim.size_y;

	pm = &dev->ctx[dev->cur_ctx_id].blk_pm;
	aem_win = &(pm->aem.win_info);
	afm_crop = &(pm->afm.crop_size);
	p->ae_valid_rect.x = aem_win->offset_x;
	p->ae_valid_rect.y = aem_win->offset_y;
	p->ae_valid_rect.w = aem_win->blk_width * aem_win->blk_num_x;
	p->ae_valid_rect.h = aem_win->blk_height * aem_win->blk_num_y;

	p->af_valid_rect.x = afm_crop->x;
	p->af_valid_rect.y = afm_crop->y;
	p->af_valid_rect.w = afm_crop->w;
	p->af_valid_rect.h = afm_crop->h;

	return 0;
}

static int dcamcore_ioctrl(void *dcam_handle, enum dcam_ioctrl_cmd cmd, void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_mipi_info *cap = NULL;
	struct dcam_path_desc *path = NULL;
	struct camera_frame *frame = NULL;
	struct dcam_hw_fbc_ctrl arg;
	struct dcam_hw_ebd_set set;
	struct cam_hw_gtm_update gtmarg;
	struct dcam_dev_param *pm = NULL;
	int *fbc_mode = NULL;
	uint32_t gtm_param_idx = 0;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;

	if (unlikely(atomic_read(&dev->state) == STATE_INIT)) {
		pr_err("fail to get valid dev state of DCAM%d\n", dev->idx);
		return -EINVAL;
	}

	switch (cmd) {
	case DCAM_IOCTL_CFG_CAP:
		cap = &dev->cap_info;
		memcpy(cap, param, sizeof(struct dcam_mipi_info));
		dev->is_4in1 = cap->is_4in1;
		dev->dcam_slice_mode = cap->dcam_slice_mode;
		dev->slice_count = 0;
		break;
	case DCAM_IOCTL_CFG_STATIS_BUF:
		ret = dcamcore_statis_buffer_cfg(dev, param);
		break;
	case DCAM_IOCTL_PUT_RESERV_STATSBUF:
		ret = dcamcore_reserved_buffer_put(dev);
		break;
	case DCAM_IOCTL_INIT_STATIS_Q:
		ret = dcamcore_statis_bufferq_init(dev);
		break;
	case DCAM_IOCTL_DEINIT_STATIS_Q:
		ret = dcamcore_statis_bufferq_deinit(dev);
		dcamcore_statis_buffer_unmap(dev);
		break;
	case DCAM_IOCTL_RECFG_PARAM:
		/* online context id is always 0 */
		ret = dcamcore_param_reconfig(dev, DCAM_CXT_0);
		break;
	case DCAM_IOCTL_CFG_EBD:
		dev->is_ebd = 1;
		set.idx = dev->idx;
		set.p = param;
		ret = dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_EBD_SET, &set);
		break;
	case DCAM_IOCTL_CFG_SEC:
		ret = dcamcore_dcamsec_cfg(dev, param);
		break;
	case DCAM_IOCTL_CFG_FBC:
		fbc_mode = (int *)param;
		/* update compressed flag for reserved buffer */
		if (*fbc_mode == DCAM_FBC_FULL_14_BIT ||
			*fbc_mode == DCAM_FBC_FULL_10_BIT)
			path = &dev->path[DCAM_PATH_FULL];
		else if (*fbc_mode == DCAM_FBC_BIN_14_BIT ||
			*fbc_mode == DCAM_FBC_BIN_10_BIT)
			path = &dev->path[DCAM_PATH_BIN];
		if (!path) {
			pr_info("Unsupport fbc mode %d\n", *fbc_mode);
			return 0;
		}
		arg.idx = dev->idx;
		arg.fbc_mode = *fbc_mode;
		dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_FBC_CTRL, &arg);

		list_for_each_entry(frame, &path->reserved_buf_queue.head, list) {
			if (!frame)
				break;
			else {
				frame->is_compressed = 1;
				if (*fbc_mode == DCAM_FBC_FULL_14_BIT ||
					*fbc_mode == DCAM_FBC_BIN_14_BIT)
					frame->compress_4bit_bypass = 0;
			}
		}
		break;
	case DCAM_IOCTL_CFG_RPS:
		ret = dcamcore_rps_cfg(dev, param);
		break;
	case DCAM_IOCTL_CFG_REPLACER:
		dev->replacer = (struct dcam_image_replacer *)param;
		break;
	case DCAM_IOCTL_GET_PATH_RECT:
		ret = dcamcore_path_rect_get(dev, param);
		break;
	case DCAM_IOCTL_CFG_STATIS_BUF_SKIP:
		ret = dcamcore_statis_buffer_skip_cfg(dev, param);
		break;
	case DCAM_IOCTL_CFG_GTM_UPDATE:
		pm = &dev->ctx[DCAM_CXT_0].blk_pm;
		gtm_param_idx = *(uint32_t *)param;
		if (gtm_param_idx == DCAM_GTM_PARAM_CAP)
			pm->gtm[DCAM_GTM_PARAM_PRE].update_en = 0;
		else
			pm->gtm[DCAM_GTM_PARAM_PRE].update_en = 1;
		//pm->gtm[DCAM_GTM_PARAM_PRE].update |= BIT(0);
		gtmarg.gtm_idx = gtm_param_idx;
		gtmarg.idx = dev->idx;
		gtmarg.hw = dev->hw;
		gtmarg.blk_dcam_pm = pm;
		gtmarg.glb_reg_lock = dev->glb_reg_lock;
		dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_GTM_UPDATE, &gtmarg);
		break;
	default:
		pr_err("fail to get a known cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

static int dcamcore_cb_set(void *dcam_handle,
		dcam_dev_callback cb, void *priv_data)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;

	if (!dcam_handle || !cb || !priv_data) {
		pr_err("fail to get valid param, dcam_handle=%p, cb=%p, priv_data=%p\n",
			dcam_handle, cb, priv_data);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	dev->dcam_cb_func = cb;
	dev->cb_priv_data = priv_data;

	return ret;
}

static int dcamcore_dev_start(void *dcam_handle, int online)
{
	int ret = 0;
	int i;
	uint32_t force_ids = DCAM_CTRL_ALL;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_dev_param *pm;
	struct dcam_sync_helper *helper = NULL;
	struct dcam_path_desc *path = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_reg_trace trace;
	struct dcam_hw_path_ctrl pause;
	struct dcam_hw_start parm;
	struct dcam_hw_force_copy copyarg;
	struct dcam_hw_mipi_cap caparg;
	struct dcam_hw_path_start patharg;
	struct dcam_hw_sram_ctrl sramarg;
	unsigned long flag;

	if (!dcam_handle) {
		pr_err("fail to get valid dcam_pipe_dev\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	dev->offline = !online;
	dev->cur_ctx_id = DCAM_CXT_0;
	hw = dev->hw;
	pm = &dev->ctx[dev->cur_ctx_id].blk_pm;

	if ((pm->lsc.buf.mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
		ret = cam_buf_iommu_map(&pm->lsc.buf, CAM_IOMMUDEV_DCAM);
		if (ret)
			pm->lsc.buf.iova[0] = 0L;
	}

	if (!online) {
		ret = dcamcore_offline_thread_create(dev);
		if (ret) {
			pr_err("fail to creat offline thread\n");
			return ret;
		}
		atomic_set(&dev->state, STATE_RUNNING);
		return ret;
	}

	ret = atomic_read(&dev->state);
	if (unlikely(ret != STATE_IDLE)) {
		pr_err("fail to get a valid state, starting DCAM%u in state %d\n", dev->idx, ret);
		return -EINVAL;
	}

	pr_info("DCAM%u start: %p, state = %d\n", dev->idx, dev, atomic_read(&dev->state));

	ret = dcamcore_sync_helper_init(dev);
	if (ret < 0) {
		pr_err("fail to init DCAM%u sync helper, ret: %d\n",
			dev->idx, ret);
		return ret;
	}

	/* enable statistic paths  */
	if (pm->aem.bypass == 0)
		atomic_set(&dev->path[DCAM_PATH_AEM].user_cnt, 1);
	if (pm->lscm.bypass == 0)
		atomic_set(&dev->path[DCAM_PATH_LSCM].user_cnt, 1);
	if (pm->afm.bypass == 0)
		atomic_set(&dev->path[DCAM_PATH_AFM].user_cnt, 1);
	if (pm->afl.afl_info.bypass == 0)
		atomic_set(&dev->path[DCAM_PATH_AFL].user_cnt, 1);
	if (pm->hist.bayerHist_info.hist_bypass == 0)
		atomic_set(&dev->path[DCAM_PATH_HIST].user_cnt, 1);

	if (pm->pdaf.bypass == 0)
		atomic_set(&dev->path[DCAM_PATH_PDAF].user_cnt, 1);
	if (dev->is_3dnr)
		atomic_set(&dev->path[DCAM_PATH_3DNR].user_cnt, 1);

	if (dev->is_ebd)
		atomic_set(&dev->path[DCAM_PATH_VCH2].user_cnt, 1);

	dev->base_fid = pm->frm_idx;
	dev->frame_index = 0;
	dev->index_to_set = 0;
	pr_info("dcam%d start  frame idx %d\n", dev->idx, pm->frm_idx);
	dev->iommu_status = 0;
	memset(dev->frame_ts, 0,
		sizeof(dev->frame_ts[0]) * DCAM_FRAME_TIMESTAMP_COUNT);
	memset(dev->frame_ts_boot, 0,
		sizeof(dev->frame_ts_boot[0]) * DCAM_FRAME_TIMESTAMP_COUNT);

	dev->helper_enabled = 0;
	if (!dev->slowmotion_count) {
		/* enable frame sync for 3DNR in normal mode */
		dcamcore_dcam_if_sync_enable_set(dev, DCAM_PATH_FULL, 1);
		dcamcore_dcam_if_sync_enable_set(dev, DCAM_PATH_BIN, 1);
		dcamcore_dcam_if_sync_enable_set(dev, DCAM_PATH_3DNR, 1);

		helper = dcam_core_sync_helper_get(dev);
	}

	caparg.idx = dev->idx;
	caparg.cap_info = dev->cap_info;
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_MIPI_CAP_SET, &caparg);

	if (ret < 0) {
		pr_err("fail to set DCAM%u mipi cap\n", dev->idx);
		return ret;
	}

	for (i = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		patharg.path_id = i;
		patharg.idx = dev->idx;
		patharg.slowmotion_count = dev->slowmotion_count;
		patharg.pdaf_path_eb = (pm->pdaf.bypass == 0) ? 1 : 0;
		patharg.cap_info = dev->cap_info;
		patharg.pack_bits = dev->path[i].pack_bits;
		patharg.src_sel = dev->path[i].src_sel;
		patharg.bayer_pattern = dev->path[i].bayer_pattern;
		patharg.in_trim = dev->path[i].in_trim;
		patharg.endian = dev->path[i].endian;
		atomic_set(&path->set_frm_cnt, 0);

		if (atomic_read(&path->user_cnt) < 1 || atomic_read(&path->is_shutoff) > 0)
			continue;

		if (path->path_id == DCAM_PATH_FULL) {
			spin_lock_irqsave(&path->state_lock, flag);
			if (path->state == DCAM_PATH_PAUSE) {
				hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &patharg);
				pause.idx = dev->idx;
				pause.path_id = path->path_id;
				pause.type = HW_DCAM_PATH_PAUSE;
				hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_CTRL, &pause);
				spin_unlock_irqrestore(&path->state_lock, flag);
				continue;
			}
			spin_unlock_irqrestore(&path->state_lock, flag);
		}

		ret = dcam_path_store_frm_set(dev, path, helper);
		if (ret < 0) {
			pr_err("fail to set frame for DCAM%u %s , ret %d\n",
				dev->idx, dcam_path_name_get(path->path_id), ret);
			return ret;
		}

		if (atomic_read(&path->set_frm_cnt) > 0)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &patharg);
	}

	if (dev->is_4in1 == 0)
		dcam_init_lsc(pm, 1);
	/* DCAM_CTRL_COEF will always set in dcam_init_lsc() */
	//force_ids &= ~DCAM_CTRL_COEF;
	copyarg.id = force_ids;
	copyarg.idx = dev->idx;
	copyarg.glb_reg_lock = dev->glb_reg_lock;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_FORCE_COPY, &copyarg);

	if (helper) {
		if (helper->enabled)
			helper->sync.index = dev->base_fid + dev->index_to_set;
		else
			dcam_core_sync_helper_put(dev, helper);
	}

	/* TODO: change AFL trigger */
	atomic_set(&dev->path[DCAM_PATH_AFL].user_cnt, 0);

	dcam_int_tracker_reset(dev->idx);
	parm.idx = dev->idx;
	parm.format = dev->cap_info.format;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_START, &parm);

	if (dev->is_4in1 == 0) {
		sramarg.sram_ctrl_en = 1;
		sramarg.idx = dev->idx;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_SRAM_CTRL_SET, &sramarg);
	}

	atomic_set(&dev->state, STATE_RUNNING);
	trace.type = NORMAL_REG_TRACE;
	trace.idx = dev->idx;
	hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
	dev->auto_cpy_id = 0;
	dev->err_count = 1;
	pr_info("dcam%d done state = %d\n", dev->idx, atomic_read(&dev->state));
	return ret;
}

static int dcamcore_dev_stop(void *dcam_handle, enum dcam_stop_cmd pause)
{
	int ret = 0, state = 0;
	int i = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_dev_param *pm;
	struct dcam_path_desc *path = NULL;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;

	if (dev->offline) {
		ret = dcamcore_offline_thread_stop(&dev->thread);
		if (ret) {
			pr_err("fail to stop offline thread\n");
		}
	}

	state = atomic_read(&dev->state);
	if (unlikely(state == STATE_INIT) || unlikely(state == STATE_IDLE)) {
		pr_warn("DCAM%d not started yet\n", dev->idx);
		return -EINVAL;
	}

	dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_STOP, &dev->idx);
	dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_RESET, &dev->idx);

	dcam_int_tracker_dump(dev->idx);
	dcam_int_tracker_reset(dev->idx);
	atomic_set(&dev->state, STATE_IDLE);

	for (i = DCAM_CXT_1; i < DCAM_CXT_NUM; i++) {
		dev->ctx[i].ctx_id = i;
		if (atomic_read(&dev->ctx[i].user_cnt) > 0)
			dcamcore_context_deinit(&dev->ctx[i]);
	}
	pm = &dev->ctx[DCAM_CXT_0].blk_pm;
	if (pause == DCAM_STOP) {
		pm->aem.bypass = 1;
		pm->afm.bypass = 1;
		pm->afl.afl_info.bypass = 1;
		pm->hist.bayerHist_info.hist_bypass = 1;
		pm->lscm.bypass = 1;
		pm->pdaf.bypass = 1;
		pm->gtm[DCAM_GTM_PARAM_PRE].update_en = 1;
		pm->gtm[DCAM_GTM_PARAM_CAP].update_en = 1;
		pm->frm_idx = 0;
		pr_info("stop all\n");

		dev->is_3dnr = dev->is_4in1 = 0;
	} else if (pause == DCAM_PAUSE_ONLINE) {
		pm->frm_idx = dev->base_fid + dev->frame_index;
		pr_info("dcam%d online pause fram id %d %d, base_fid %d, new %d\n", dev->idx,
			dev->frame_index, dev->index_to_set, dev->base_fid, pm->frm_idx);
	} else {
		pr_info("offline stopped %d\n", pause);
	}

	if (pm->lsc.buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&pm->lsc.buf);

	dev->err_count = 0;
	dev->offline = 0;

	for (i = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		atomic_set(&path->is_shutoff, 0);
	}
	pr_info("stop dcam pipe dev[%d] state = %d!\n", dev->idx, atomic_read(&dev->state));
	return ret;
}

/*
 * Open dcam_pipe_dev and hardware dcam_if IP.
 */
static int dcamcore_dev_open(void *dcam_handle)
{
	int ret = 0;
	int i = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_desc *path = NULL;
	struct cam_hw_info *hw = NULL;
	struct dcam_pipe_context *pctx;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	mutex_lock(&s_dcam_dev_mutex);
	dev = (struct dcam_pipe_dev *)dcam_handle;

	ret = atomic_read(&dev->state);
	if (unlikely(ret != STATE_INIT)) {
		pr_err("fail to get a valid dev state, DCAM%u, state=%d\n",
			dev->idx, ret);
		mutex_unlock(&s_dcam_dev_mutex);
		return -EINVAL;
	}

	hw = dev->hw;
	memset(&dev->path[0], 0, sizeof(dev->path));
	for (i = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		path->path_id = i;
		atomic_set(&path->user_cnt, 0);
		atomic_set(&path->set_frm_cnt, 0);
		atomic_set(&path->is_shutoff, 0);
		spin_lock_init(&path->size_lock);
		spin_lock_init(&path->state_lock);

		if (path->path_id == DCAM_PATH_BIN) {
			path->rds_coeff_size = RDS_COEF_TABLE_SIZE;
			path->rds_coeff_buf = kzalloc(path->rds_coeff_size, GFP_KERNEL);
			if (path->rds_coeff_buf == NULL) {
				path->rds_coeff_size = 0;
				pr_err("fail to alloc rds coeff buffer.\n");
				ret = -ENOMEM;
				goto exit;
			}
		}
	}

	for (i = 0; i < DCAM_CXT_NUM; i++) {
		dev->ctx[i].ctx_id = i;
		atomic_set(&dev->ctx[i].user_cnt, 0);
	}
	/* use context[0] by default */
	pctx = &dev->ctx[DCAM_CXT_0];
	ret = dcamcore_context_init(dev, pctx);
	if (ret) {
		pr_err("fail to init dcam context[0]\n");
		goto exit;
	}
	dev->cur_ctx_id = DCAM_CXT_0;

	ret = dcam_drv_hw_init(dev);
	if (ret) {
		pr_err("fail to open DCAM%u, ret: %d\n",
			dev->idx, ret);
		goto hw_fail;
	}

	if (atomic_inc_return(&s_dcam_axi_opened) == 1)
		hw->dcam_ioctl(hw, DCAM_HW_CFG_INIT_AXI, &dev->idx);
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_RESET, &dev->idx);
	if (ret)
		goto reset_fail;

	cam_queue_init(&dev->in_queue, DCAM_IN_Q_LEN,
		dcamcore_src_frame_ret);
	cam_queue_init(&dev->proc_queue, DCAM_PROC_Q_LEN,
		dcamcore_src_frame_ret);

	atomic_set(&dev->state, STATE_IDLE);
	spin_lock_init(&dev->glb_reg_lock);

	/* for debugfs */
	atomic_inc(&s_dcam_opened[dev->idx]);

	mutex_unlock(&s_dcam_dev_mutex);
	pr_info("open dcam pipe dev[%d]!\n", dev->idx);

	return 0;

reset_fail:
	atomic_dec(&s_dcam_axi_opened);
	ret = dcam_drv_hw_deinit(dev);
hw_fail:
	dcamcore_context_deinit(pctx);
exit:
	if (dev->path[DCAM_PATH_BIN].rds_coeff_buf) {
		kfree(dev->path[DCAM_PATH_BIN].rds_coeff_buf);
		dev->path[DCAM_PATH_BIN].rds_coeff_buf = NULL;
		dev->path[DCAM_PATH_BIN].rds_coeff_size = 0;
	}
	mutex_unlock(&s_dcam_dev_mutex);
	pr_info("fail to open dcam pipe dev[%d]!\n", dev->idx);

	return ret;
}

/*
 * Close dcam_pipe_dev and hardware dcam_if IP.
 */
int dcamcore_dev_close(void *dcam_handle)
{
	int ret = 0;
	int i;
	struct dcam_pipe_dev *dev = NULL;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;

	if (unlikely(atomic_read(&dev->state) == STATE_INIT)) {
		pr_err("fail to get dev state, DCAM%u already closed\n", dev->idx);
		return -EINVAL;
	}

	cam_queue_clear(&dev->in_queue, struct camera_frame, list);
	cam_queue_clear(&dev->proc_queue, struct camera_frame, list);

	for (i = 0; i < DCAM_CXT_NUM; i++) {
		dev->ctx[i].ctx_id = i;
		if (atomic_read(&dev->ctx[i].user_cnt)  > 0)
			dcamcore_context_deinit(&dev->ctx[i]);
	}

	if (dev->path[DCAM_PATH_BIN].rds_coeff_buf) {
		kfree(dev->path[DCAM_PATH_BIN].rds_coeff_buf);
		dev->path[DCAM_PATH_BIN].rds_coeff_buf = NULL;
		dev->path[DCAM_PATH_BIN].rds_coeff_size = 0;
	}

	ret = dcam_drv_hw_deinit(dev);

	atomic_set(&dev->state, STATE_INIT);
	/* for debugfs */
	atomic_dec(&s_dcam_opened[dev->idx]);
	atomic_dec(&s_dcam_axi_opened);

	pr_info("close dcam pipe dev[%d]!\n", dev->idx);

	return ret;
}

/*
 * Operations for this dcam_pipe_dev.
 */
static struct dcam_pipe_ops s_dcam_pipe_ops = {
	.open = dcamcore_dev_open,
	.close = dcamcore_dev_close,
	.start = dcamcore_dev_start,
	.stop = dcamcore_dev_stop,
	.get_path = dcamcore_path_get,
	.put_path = dcamcore_path_put,
	.cfg_path = dcamcore_path_cfg,
	.ioctl = dcamcore_ioctrl,
	.cfg_blk_param = dcamcore_param_cfg,
	.proc_frame = dcamcore_frame_proc,
	.set_callback = dcamcore_cb_set,
};

/*
 * Create a dcam_pipe_dev for designated cam_hw_info.
 */
void *dcam_core_dcam_if_dev_get(uint32_t idx, struct cam_hw_info *hw)
{
	struct dcam_pipe_dev *dev = NULL;

	if (idx >= DCAM_ID_MAX) {
		pr_err("fail to get valid DCAM index: %u\n", idx);
		return NULL;
	}

	if (unlikely(!hw)) {
		pr_err("fail to get valid param hw\n");
		return NULL;
	}

	mutex_lock(&s_dcam_dev_mutex);
	if (s_dcam_dev[idx]) {
		pr_err("fail to get valid dcam dev, dcam %d already in use. pipe dev: %p\n",
			idx, s_dcam_dev[idx]);
		goto exit;
	}

	dev = vzalloc(sizeof(struct dcam_pipe_dev));
	if (!dev) {
		pr_err("fail to alloc memory for DCAM%u\n", idx);
		goto exit;
	}

	dev->idx = idx;
	dev->hw = hw;
	/*
	 * Operations for this dcam_pipe_dev.
	 */
	dev->dcam_pipe_ops = &s_dcam_pipe_ops;

	/* frame sync helper */
	spin_lock_init(&dev->helper_lock);

	atomic_set(&dev->state, STATE_INIT);

	s_dcam_dev[idx] = dev;

exit:
	mutex_unlock(&s_dcam_dev_mutex);

	if (dev == NULL)
		pr_err("fail to get DCAM%u pipe dev\n", idx);
	else
		pr_info("get DCAM%u pipe dev: %p\n", idx, dev);

	return dev;
}

/*
 * Release a dcam_pipe_dev.
 */
int dcam_core_dcam_if_dev_put(void *dcam_handle)
{
	uint32_t idx = 0;
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	if (idx >= DCAM_ID_MAX) {
		pr_err("fail to get valid dcam idx, index: %u\n", idx);
		return -EINVAL;
	}

	mutex_lock(&s_dcam_dev_mutex);
	if (dev != s_dcam_dev[idx]) {
		pr_err("fail to get matched dev: %p, %p\n",
			dev, s_dcam_dev[idx]);
		mutex_unlock(&s_dcam_dev_mutex);
		return -EFAULT;
	}

	ret = atomic_read(&dev->state);
	if (unlikely(ret != STATE_INIT)) {
		pr_warn("releasing DCAM%u in state %d may cause leak\n",
			dev->idx, ret);
	}
	pr_info("put DCAM%u pipe dev: %p\n", idx, dev);
	vfree(dev);
	s_dcam_dev[idx] = NULL;

	mutex_unlock(&s_dcam_dev_mutex);

	return ret;
}

