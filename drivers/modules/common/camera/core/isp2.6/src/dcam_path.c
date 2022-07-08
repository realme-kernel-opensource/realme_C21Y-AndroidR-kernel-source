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

#include <linux/err.h>
#include <sprd_mm.h>

#include "cam_scaler.h"
#include "cam_queue.h"
#include "cam_debugger.h"
#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_path.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_PATH: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

/*
 * path name for debug output
 */
static const char *_DCAM_PATH_NAMES[DCAM_PATH_MAX] = {
	[DCAM_PATH_FULL] = "FULL",
	[DCAM_PATH_BIN] = "BIN",
	[DCAM_PATH_PDAF] = "PDAF",
	[DCAM_PATH_VCH2] = "VCH2",
	[DCAM_PATH_VCH3] = "VCH3",
	[DCAM_PATH_AEM] = "AEM",
	[DCAM_PATH_AFM] = "AFM",
	[DCAM_PATH_AFL] = "AFL",
	[DCAM_PATH_HIST] = "HIST",
	[DCAM_PATH_3DNR] = "3DNR",
	[DCAM_PATH_BPC] = "BPC",
	[DCAM_PATH_LSCM] = "LSCM",
};

/*
 * convert @path_id to path name
 */
const char *dcam_path_name_get(enum dcam_path_id path_id)
{
	return is_path_id(path_id) ? _DCAM_PATH_NAMES[path_id] : "(null)";
}

int dcam_path_base_cfg(void *dcam_handle,
		struct dcam_path_desc *path, void *param)
{
	int ret = 0;
	uint32_t idx;
	unsigned long flags = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_cfg_param *ch_desc;

	if (!dcam_handle || !path || !param) {
		pr_err("fail to get valid param, dcam_handle=%p, path=%p, param=%p.\n",
			dcam_handle, path, param);
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;
	ch_desc = (struct dcam_path_cfg_param *)param;
	idx = dev->idx;
	dev->cap_info.cap_size = ch_desc->input_trim;

	switch (path->path_id) {
	case DCAM_PATH_FULL:
		spin_lock_irqsave(&path->size_lock, flags);
		path->src_sel = ch_desc->is_raw ? 0 : 1;
		path->frm_deci = ch_desc->frm_deci;
		path->frm_skip = ch_desc->frm_skip;
		path->pack_bits = ch_desc->pack_bits;
		path->endian = ch_desc->endian;
		path->is_4in1 = ch_desc->is_4in1;
		path->bayer_pattern = ch_desc->bayer_pattern;
		path->base_update = 1;
		spin_unlock_irqrestore(&path->size_lock, flags);
		break;
	case DCAM_PATH_BIN:
		path->frm_deci = ch_desc->frm_deci;
		path->frm_skip = ch_desc->frm_skip;

		path->pack_bits = ch_desc->pack_bits;
		path->endian = ch_desc->endian;
		path->is_4in1 = ch_desc->is_4in1;
		path->bayer_pattern = ch_desc->bayer_pattern;
		/*
		 * TODO:
		 * Better not binding dcam_if feature to BIN path, which is a
		 * architecture defect and not going to be fixed now.
		 */
		dev->slowmotion_count = ch_desc->slowmotion_count;
		dev->is_3dnr |= ch_desc->enable_3dnr;
		dev->raw_cap = ch_desc->raw_cap;
		break;
	case DCAM_PATH_VCH2:
		path->endian = ch_desc->endian;
		path->src_sel = ch_desc->is_raw ? 1 : 0;
		break;
	default:
		pr_err("fail to get known path %d\n", path->path_id);
		ret = -EFAULT;
		break;
	}
	return ret;
}

int dcam_path_size_cfg(void *dcam_handle,
		struct dcam_path_desc *path,
		void *param)
{
	int ret = 0;
	uint32_t idx;
	uint32_t invalid;
	struct img_size crop_size, dst_size;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_cfg_param *ch_desc;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_calc_rds_phase arg;
	uint32_t dcam_max_w = 0, dcam_max_h = 0;
	unsigned long flag;

	if (!dcam_handle || !path || !param) {
		pr_err("fail to get valid param, dcam_handle=%p, path=%p, param=%p.\n",
			dcam_handle, path, param);
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;
	hw = dev->hw;
	if (!hw) {
		pr_err("fail to get a valid hw, hw ptr is NULL.\n");
		return -EFAULT;
	}

	ch_desc = (struct dcam_path_cfg_param *)param;
	idx = dev->idx;
	dcam_max_w = hw->ip_dcam[idx]->max_width;
	dcam_max_h = hw->ip_dcam[idx]->max_height;

	switch (path->path_id) {
	case DCAM_PATH_FULL:
		spin_lock_irqsave(&path->size_lock, flag);
		if (path->size_update) {
			spin_unlock_irqrestore(&path->size_lock, flag);
			return -EFAULT;
		}
		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;

		invalid = 0;
		invalid |= ((path->in_size.w == 0) || (path->in_size.h == 0));
		invalid |= (path->in_size.w > dcam_max_w);
		invalid |= (path->in_size.h > dcam_max_h);
		invalid |= ((path->in_trim.start_x +
				path->in_trim.size_x) > path->in_size.w);
		invalid |= ((path->in_trim.start_y +
				path->in_trim.size_y) > path->in_size.h);
		if (invalid) {
			spin_unlock_irqrestore(&path->size_lock, flag);
			pr_err("fail to get valid size, size:%d %d, trim %d %d %d %d\n",
				path->in_size.w, path->in_size.h,
				path->in_trim.start_x, path->in_trim.start_y,
				path->in_trim.size_x,
				path->in_trim.size_y);
			return -EINVAL;
		}
		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {
			path->out_size.w = path->in_trim.size_x;
			path->out_size.h = path->in_trim.size_y;
		} else {
			path->out_size.w = path->in_size.w;
			path->out_size.h = path->in_size.h;
		}

		path->priv_size_data = ch_desc->priv_size_data;
		path->size_update = 1;
		spin_unlock_irqrestore(&path->size_lock, flag);

		pr_info("cfg full path done. size %d %d %d %d\n",
			path->in_size.w, path->in_size.h,
			path->out_size.w, path->out_size.h);
		pr_info("sel %d. trim %d %d %d %d\n", path->src_sel,
			path->in_trim.start_x, path->in_trim.start_y,
			path->in_trim.size_x, path->in_trim.size_y);
		break;

	case DCAM_PATH_BIN:
		/* lock here to keep all size parameters updating is atomic.
		 * because the rds coeff caculation may be time-consuming,
		 * we should not disable irq here, or else may cause irq missed.
		 * Just trylock set_next_frame in irq handling to avoid deadlock
		 * If last updating has not been applied yet, will return error.
		 * error may happen if too frequent zoom ratio updateing,
		 * but should not happen for first time cfg before stream on,
		 * if error return, caller can discard updating
		 * or try cfg_size again after while.
		 */
		spin_lock_irqsave(&path->size_lock, flag);
		if (path->size_update) {
			if (atomic_read(&dev->state) != STATE_RUNNING)
				pr_info("Overwrite dcam path size before dcam start if any\n");
			else {
				spin_unlock_irqrestore(&path->size_lock, flag);
				pr_info("Previous path updating pending\n");
				return -EFAULT;
			}
		}

		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;
		path->out_size = ch_desc->output_size;

		invalid = 0;
		invalid |= ((path->in_size.w == 0) || (path->in_size.h == 0));
		if (!dev->dcam_slice_mode) {
			invalid |= (path->in_size.w > dcam_max_w);
			invalid |= (path->in_size.h > dcam_max_h);
		}
		/* trim should not be out range of source */
		invalid |= ((path->in_trim.start_x +
				path->in_trim.size_x) > path->in_size.w);
		invalid |= ((path->in_trim.start_y +
				path->in_trim.size_y) > path->in_size.h);

		/* output size should not be larger than trim ROI */
		invalid |= path->in_trim.size_x < path->out_size.w;
		invalid |= path->in_trim.size_y < path->out_size.h;

		/* Down scaling should not be smaller then 1/4*/
		invalid |= path->in_trim.size_x >
				(path->out_size.w * DCAM_SCALE_DOWN_MAX);
		invalid |= path->in_trim.size_y >
				(path->out_size.h * DCAM_SCALE_DOWN_MAX);

		if (invalid) {
			spin_unlock_irqrestore(&path->size_lock, flag);
			pr_err("fail to get valid size, size:%d %d, trim %d %d %d %d, dst %d %d\n",
				path->in_size.w, path->in_size.h,
				path->in_trim.start_x, path->in_trim.start_y,
				path->in_trim.size_x, path->in_trim.size_y,
				path->out_size.w, path->out_size.h);
			return -EINVAL;
		}
		crop_size.w = path->in_trim.size_x;
		crop_size.h = path->in_trim.size_y;
		dst_size = path->out_size;

		if ((crop_size.w == dst_size.w) &&
			(crop_size.h == dst_size.h))
			path->scaler_sel = DCAM_SCALER_BYPASS;
		else if ((dst_size.w * 2 == crop_size.w) &&
			(dst_size.h * 2 == crop_size.h)) {
			pr_debug("1/2 binning used. src %d %d, dst %d %d\n",
				crop_size.w, crop_size.h, dst_size.w,
				dst_size.h);
			path->scaler_sel = DCAM_SCALER_BINNING;
			path->bin_ratio = 0;
		} else if ((dst_size.w * 4 == crop_size.w) &&
			(dst_size.h * 4 == crop_size.h)) {
			pr_debug("1/4 binning used. src %d %d, dst %d %d\n",
				crop_size.w, crop_size.h, dst_size.w,
				dst_size.h);
			path->scaler_sel = DCAM_SCALER_BINNING;
			path->bin_ratio = 1;
		} else {
			pr_debug("RDS used. in %d %d, out %d %d\n",
				crop_size.w, crop_size.h, dst_size.w,
				dst_size.h);
			path->scaler_sel = DCAM_SCALER_RAW_DOWNSISER;
			path->gphase.rds_input_h_global = path->in_trim.size_y;
			path->gphase.rds_input_w_global = path->in_trim.size_x;
			path->gphase.rds_output_w_global = path->out_size.w;
			path->gphase.rds_output_h_global = path->out_size.h;
			arg.gphase = &path->gphase;
			arg.slice_id = 0;
			arg.slice_end0 = 0;
			arg.slice_end1 = 0;
			ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_CALC_RDS_PHASE_INFO, &arg);
			if (ret)
				pr_err("fail to calc rds phase info\n");
			cam_scaler_dcam_rds_coeff_gen((uint16_t)crop_size.w,
				(uint16_t)crop_size.h,
				(uint16_t)dst_size.w,
				(uint16_t)dst_size.h,
				(uint32_t *)path->rds_coeff_buf);
		}
		path->priv_size_data = ch_desc->priv_size_data;
		path->size_update = 1;
		/* if 3dnr path enable, need update when zoom */
		{
			struct dcam_path_desc *path_3dnr;

			path_3dnr = &dev->path[DCAM_PATH_3DNR];
			if (atomic_read(&path_3dnr->user_cnt) > 0)
				path_3dnr->size_update = 1;
			path_3dnr->in_trim = path->in_trim;
		}
		spin_unlock_irqrestore(&path->size_lock, flag);

		pr_info("cfg bin path done. size %d %d  dst %d %d\n",
			path->in_size.w, path->in_size.h,
			path->out_size.w, path->out_size.h);
		pr_info("scaler %d. trim %d %d %d %d\n", path->scaler_sel,
			path->in_trim.start_x, path->in_trim.start_y,
			path->in_trim.size_x, path->in_trim.size_y);
		break;

	default:
		if (path->path_id == DCAM_PATH_VCH2 && path->src_sel)
			return ret;

		pr_err("fail to get known path %d\n", path->path_id);
		ret = -EFAULT;
		break;
	}
	return ret;
}

/*
 * Set skip num for path @path_id so that we can update address accordingly in
 * CAP SOF interrupt.
 *
 * For slow motion mode.
 * On previous project, there's no slow motion support for AEM on DCAM. Instead,
 * we use @skip_num to generate AEM TX DONE every @skip_num frame. If @skip_num
 * is set by algorithm, we should configure AEM output address accordingly in
 * CAP SOF every @skip_num frame.
 */
int dcam_path_skip_num_set(struct dcam_pipe_dev *dev,
		int path_id, uint32_t skip_num)
{
	struct dcam_path_desc *path = NULL;

	if (unlikely(!dev || !is_path_id(path_id)))
		return -EINVAL;

	if (atomic_read(&dev->state) == STATE_RUNNING) {
		pr_warn("DCAM%u %s set skip_num while running is forbidden\n",
			dev->idx, dcam_path_name_get(path_id));
		return -EINVAL;
	}

	path = &dev->path[path_id];
	path->frm_deci = skip_num;
	path->frm_deci_cnt = 0;

	pr_info("DCAM%u %s set skip_num %u\n",
		dev->idx, dcam_path_name_get(path_id), skip_num);

	return 0;
}

static inline struct camera_frame *
dcam_path_frame_cycle(struct dcam_pipe_dev *dev, struct dcam_path_desc *path)
{
	uint32_t src;
	struct camera_frame *frame = NULL;

	if (path->path_id == DCAM_PATH_FULL && path->src_sel == 0) {
		/* get raw buffer */
		frame = cam_queue_dequeue(&path->alter_out_queue, struct camera_frame, list);
		src = 0;
	}
	if (frame == NULL) {
		frame = cam_queue_dequeue(&path->out_buf_queue, struct camera_frame, list);
		src = 1;
		if (frame != NULL && path->path_id == DCAM_PATH_FULL) {
			if (cam_buf_iommu_map(&frame->buf, CAM_IOMMUDEV_DCAM)) {
				pr_err("mapping failed\n");
				cam_queue_enqueue(&path->out_buf_queue, &frame->list);
				frame = NULL;
			}
		}
	}

	if (frame && (src == 0)) {
		/* usr buffer for raw, mapping delay to here*/
		if (cam_buf_iommu_map(&frame->buf, CAM_IOMMUDEV_DCAM)) {
			pr_err("mapping failed\n");
			cam_queue_enqueue(&path->alter_out_queue, &frame->list);
			pr_debug("mapping raw buffer for ch %d, mfd %d\n",
				frame->channel_id, frame->buf.mfd[0]);
			frame = NULL;
		}
	}

	if (frame == NULL)
		frame = cam_queue_dequeue(&path->reserved_buf_queue, struct camera_frame, list);

	if (frame == NULL) {
		pr_debug("DCAM%u %s buffer unavailable\n",
			dev->idx, dcam_path_name_get(path->path_id));
		return ERR_PTR(-ENOMEM);
	}

	if (cam_queue_enqueue(&path->result_queue, &frame->list) < 0) {
		if (frame->is_reserved)
			cam_queue_enqueue(&path->reserved_buf_queue, &frame->list);
		else if (src == 1)
			cam_queue_enqueue(&path->out_buf_queue, &frame->list);
		else {
			cam_buf_iommu_unmap(&frame->buf);
			cam_queue_enqueue(&path->alter_out_queue, &frame->list);
		}

		pr_err("fail to enqueue frame to result_queue, DCAM%u %s output queue overflow\n",
			dev->idx, dcam_path_name_get(path->path_id));
		return ERR_PTR(-EPERM);
	}

	frame->fid = dev->base_fid + dev->index_to_set;
	frame->sync_data = NULL;

	return frame;
}

static inline void dcam_path_frame_pointer_swap(struct camera_frame **frame1,
			struct camera_frame **frame2)
{
	struct camera_frame *frame;

	frame = *frame1;
	*frame1 = *frame2;
	*frame2 = frame;
}

int dcam_path_store_frm_set(void *dcam_handle,
		struct dcam_path_desc *path,
		struct dcam_sync_helper *helper)
{
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_dev_param *blk_dcam_pm;
	struct cam_hw_info *hw = NULL;
	struct camera_frame *frame = NULL, *saved = NULL;
	struct dcam_hw_fbc_addr fbcadr;
	struct dcam_hw_path_size path_size;
	uint32_t idx = 0, path_id = 0;
	unsigned long flags = 0, addr = 0;
	unsigned long u_addr = 0;
	const int _bin = 0, _aem = 1, _hist = 2;
	int i = 0, ret = 0;
	uint32_t slm_path = 0;

	if (unlikely(!dcam_handle || !path))
		return -EINVAL;

	dev = (struct dcam_pipe_dev *)dcam_handle;
	blk_dcam_pm = &dev->ctx[dev->cur_ctx_id].blk_pm;
	hw = dev->hw;
	idx = dev->idx;
	path_id = path->path_id;
	dev->auto_cpy_id |= *(hw->ip_dcam[idx]->path_ctrl_id_tab + path_id);

	if (dev->cap_info.format == DCAM_CAP_MODE_YUV)
		dev->auto_cpy_id |= *(hw->ip_dcam[idx]->path_ctrl_id_tab + DCAM_PATH_BIN);

	pr_debug("DCAM%u %s enter\n", idx, dcam_path_name_get(path_id));

	frame = dcam_path_frame_cycle(dev, path);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	/* assign last buffer for AEM and HIST in slow motion */
	i = dev->slowmotion_count - 1;

	if (dev->slowmotion_count && path_id == DCAM_PATH_AEM) {
		/* slow motion AEM */
		addr = slowmotion_store_addr[_aem][i];
		frame->fid += i;
	} else if (dev->slowmotion_count && path_id == DCAM_PATH_HIST) {
		/* slow motion HIST */
		addr = slowmotion_store_addr[_hist][i];
		frame->fid += i;
	} else {
		/* normal scene */
		addr = *(hw->ip_dcam[idx]->store_addr_tab + path_id);
	}

	if (addr == 0L) {
		pr_info("DCAM%d invalid path id %d, path name %s\n",
			idx, path_id, dcam_path_name_get(path_id));
		return 0;
	}

	/* replace image data for debug */
	if (dev->replacer) {
		struct dcam_image_replacer *replacer = dev->replacer;

		if ((path_id < DCAM_IMAGE_REPLACER_PATH_MAX)
			&& replacer->enabled[path_id])
			saved = cam_queue_dequeue(&path->reserved_buf_queue,
				struct camera_frame, list);
	}

	if (saved)
		dcam_path_frame_pointer_swap(&frame, &saved);
	if (frame->is_compressed) {
		struct compressed_addr fbc_addr;
		struct img_size *size = &path->out_size;

		dcam_if_cal_compressed_addr(size->w, size->h,
			frame->buf.iova[0],
			&fbc_addr,
			frame->compress_4bit_bypass);
		fbcadr.idx = idx;
		fbcadr.addr = addr;
		fbcadr.fbc_addr = &fbc_addr;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_ADDR_SET, &fbcadr);
	} else if (path_id == DCAM_PATH_AEM) {
		DCAM_REG_WR(idx, addr,
			frame->buf.iova[0] + STATIS_AEM_HEADER_SIZE);
	} else if (path_id == DCAM_PATH_HIST) {
		DCAM_REG_WR(idx, addr,
			frame->buf.iova[0] + STATIS_HIST_HEADER_SIZE);
	} else {
		DCAM_REG_WR(idx, addr, frame->buf.iova[0]);
		if (dev->cap_info.format == DCAM_CAP_MODE_YUV) {
			if (path_id == DCAM_PATH_FULL) {
				u_addr = *(hw->ip_dcam[idx]->store_addr_tab + DCAM_PATH_BIN);
				DCAM_REG_WR(idx, u_addr, frame->buf.iova[0]+frame->width * frame->height);
				pr_debug("w %d,  h %d\n", frame->width, frame->height);
			}
		}
	}
	if (saved)
		dcam_path_frame_pointer_swap(&frame, &saved);

	atomic_inc(&path->set_frm_cnt);
	if (path_id == DCAM_PATH_AFL)
		DCAM_REG_WR(idx, ISP_AFL_REGION_WADDR,
			frame->buf.iova[0] + hw->ip_dcam[idx]->afl_gbuf_size);

	if (!blk_dcam_pm->pdaf.bypass &&
		blk_dcam_pm->pdaf.pdaf_type == 3 && path_id == DCAM_PATH_PDAF) {
		/* PDAF type3, half buffer for right PD, TBD */
		addr = hw->ip_dcam[idx]->pdaf_type3_reg_addr;
		DCAM_REG_WR(idx, addr,
			frame->buf.iova[0] + frame->buf.size[0] / 2);
		pr_debug("PDAF iova %08x,  offset %x\n", (uint32_t)frame->buf.iova[0],
			(uint32_t)frame->buf.size[0] / 2);
	}

	pr_debug("DCAM%u %s set frame: fid %u, count %d\n",
		idx, dcam_path_name_get(path_id), frame->fid,
		atomic_read(&path->set_frm_cnt));

	pr_debug("DCAM%u %s reg %08x, addr %08x\n", idx, dcam_path_name_get(path_id),
		(uint32_t)addr, (uint32_t)frame->buf.iova[0]);

	/* bind frame sync data if it is not reserved buffer and not raw */
	if (helper && !frame->is_reserved && is_sync_enabled(dev, path_id)
		&& !(path_id == DCAM_PATH_FULL && path->src_sel == 0)) {
		helper->enabled |= BIT(path_id);
		helper->sync.frames[path_id] = frame;
		frame->sync_data = &helper->sync;
	}

	if ((path_id == DCAM_PATH_FULL) || (path_id == DCAM_PATH_BIN) ||
		(path_id == DCAM_PATH_3DNR)) {
		/* use trylock here to avoid waiting if cfg_path_size
		 * is already lock
		 * because this function maybe called from irq handling.
		 * disable irq to make sure less time gap between size updating
		 * to register and frame buffer address updating to register
		 * make sure that corresponding param
		 * will applied for same frame.
		 * or else size may mismatch with frame.
		 */
		if (spin_trylock_irqsave(&path->size_lock, flags)) {
			if (path->size_update && !frame->is_reserved) {
				path_size.idx = dev->idx;
				path_size.auto_cpy_id = dev->auto_cpy_id;
				path_size.size_x = dev->cap_info.cap_size.size_x;
				path_size.size_y = dev->cap_info.cap_size.size_y;
				path_size.path_id = path->path_id;
				path_size.src_sel = path->src_sel;
				path_size.bin_ratio = path->bin_ratio;
				path_size.scaler_sel = path->scaler_sel;
				path_size.rds_coeff_size = path->rds_coeff_size;
				path_size.rds_coeff_buf = path->rds_coeff_buf;
				path_size.in_size = path->in_size;
				path_size.in_trim = path->in_trim;
				path_size.out_size = path->out_size;
				path_size.rds_init_phase_int0 = path->gphase.rds_init_phase_int0;
				path_size.rds_init_phase_int1 = path->gphase.rds_init_phase_int1;
				path_size.rds_init_phase_rdm0 = path->gphase.rds_init_phase_rdm0;
				path_size.rds_init_phase_rdm1 = path->gphase.rds_init_phase_rdm1;
				hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SIZE_UPDATE, &path_size);
				frame->param_data = path->priv_size_data;
				path->size_update = 0;
				path->priv_size_data = NULL;

				if (path_id == DCAM_PATH_BIN) {
					dev->next_roi = path->in_trim;
					dev->zoom_ratio = ZOOM_RATIO_DEFAULT *
						path->in_size.w / path->in_trim.size_x;
				}
			}
			spin_unlock_irqrestore(&path->size_lock, flags);
		}
		spin_lock_irqsave(&path->size_lock, flags);
		if (path_id == DCAM_PATH_FULL && path->base_update) {
			struct dcam_hw_path_src_sel patharg;
			patharg.idx = dev->idx;
			patharg.src_sel = path->src_sel;
			ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SRC_SEL, &patharg);
		}
		path->base_update = 0;
		spin_unlock_irqrestore(&path->size_lock, flags);
	}
	frame->zoom_ratio = dev->zoom_ratio;

	/* Re-config aem win if it is updated */
	if (path_id == DCAM_PATH_AEM && (!dev->offline || dev->rps)) {
		struct dcam_dev_aem_win *win;
		struct sprd_img_rect *zoom_rect;

		spin_lock_irqsave(&path->size_lock, flags);
		dcam_k_aem_win(blk_dcam_pm);

		if (frame->buf.addr_k[0]) {
			win = (struct dcam_dev_aem_win *)(frame->buf.addr_k[0]);
			pr_debug("kaddr %lx\n", frame->buf.addr_k[0]);
			memcpy(win,
				&blk_dcam_pm->aem.win_info,
				sizeof(struct dcam_dev_aem_win));
			win++;
			zoom_rect = (struct sprd_img_rect *)win;
			zoom_rect->x = dev->next_roi.start_x;
			zoom_rect->y = dev->next_roi.start_y;
			zoom_rect->w = dev->next_roi.size_x;
			zoom_rect->h = dev->next_roi.size_y;
		}
		spin_unlock_irqrestore(&path->size_lock, flags);
	}

	/* Re-config hist win if it is updated */
	if (path_id == DCAM_PATH_HIST) {
		spin_lock_irqsave(&path->size_lock, flags);
		hw->dcam_ioctl(hw, DCAM_HW_CFG_HIST_ROI_UPDATE, blk_dcam_pm);
		if (frame->buf.addr_k[0]) {
			struct dcam_dev_hist_info *info = NULL;
			info = (struct dcam_dev_hist_info *)frame->buf.addr_k[0];
			memcpy(info, &blk_dcam_pm->hist.bayerHist_info,
				sizeof(struct dcam_dev_hist_info));
		}
		spin_unlock_irqrestore(&path->size_lock, flags);
	}

	slm_path = hw->ip_dcam[idx]->slm_path;
	if (dev->slowmotion_count && !dev->index_to_set &&
		(path_id == DCAM_PATH_AEM || path_id == DCAM_PATH_HIST)
		&& (slm_path & BIT(path_id))) {
		/* configure reserved buffer for AEM and hist */
		frame = cam_queue_dequeue(&path->reserved_buf_queue,
			struct camera_frame, list);
		if (!frame) {
			pr_debug("DCAM%u %s buffer unavailable\n",
				idx, dcam_path_name_get(path_id));
			return -ENOMEM;
		}

		i = 0;
		while (i < dev->slowmotion_count - 1) {
			if (path_id == DCAM_PATH_AEM)
				addr = slowmotion_store_addr[_aem][i];
			else
				addr = slowmotion_store_addr[_hist][i];
			DCAM_REG_WR(idx, addr, frame->buf.iova[0]);

			pr_debug("DCAM%u %s set reserved frame\n", dev->idx,
				 dcam_path_name_get(path_id));
			i++;
		}


		/* put it back */
		cam_queue_enqueue(&path->reserved_buf_queue, &frame->list);
	} else if (dev->slowmotion_count && path_id == DCAM_PATH_BIN) {
		i = 1;
		while (i < dev->slowmotion_count) {
			frame = dcam_path_frame_cycle(dev, path);
			/* in init phase, return failure if error happens */
			if (IS_ERR(frame) && !dev->index_to_set) {
				ret = PTR_ERR(frame);
				goto enqueue_reserved;
			}

			/* in normal running, just stop configure */
			if (IS_ERR(frame))
				break;

			addr = slowmotion_store_addr[_bin][i];
			if (saved)
				dcam_path_frame_pointer_swap(&frame, &saved);
			DCAM_REG_WR(idx, addr, frame->buf.iova[0]);
			if (saved)
				dcam_path_frame_pointer_swap(&frame, &saved);
			atomic_inc(&path->set_frm_cnt);

			frame->fid = dev->base_fid + dev->index_to_set + i;

			pr_debug("DCAM%u BIN set frame: fid %u, count %d\n",
				idx, frame->fid,
				atomic_read(&path->set_frm_cnt));
			i++;
		}

		if (unlikely(i != dev->slowmotion_count))
			pr_warn("DCAM%u BIN %d frame missed\n",
				idx, dev->slowmotion_count - i);
	}

enqueue_reserved:
	if (saved)
		cam_queue_enqueue(&path->reserved_buf_queue, &saved->list);

	return ret;
}
