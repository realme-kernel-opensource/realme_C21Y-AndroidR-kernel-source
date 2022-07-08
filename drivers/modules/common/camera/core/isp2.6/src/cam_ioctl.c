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

#ifdef CAM_IOCTL_LAYER

static int camioctl_time_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_time utime;
	struct timespec ts;

	memset(&ts, 0, sizeof(struct timespec));
	ktime_get_ts(&ts);
	utime.sec = (uint32_t)ts.tv_sec;
	utime.usec = (uint32_t)(ts.tv_nsec / NSEC_PER_USEC);
	pr_debug("get_time %d.%06d\n", utime.sec, utime.usec);

	ret = copy_to_user((void __user *)arg, &utime,
		sizeof(struct sprd_img_time));
	if (unlikely(ret)) {
		pr_err("fail to put user info, ret %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int camioctl_flash_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_set_flash set_param;

	ret = copy_from_user((void *)&set_param,
		(void __user *)arg,
		sizeof(struct sprd_img_set_flash));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		goto exit;
	}

	module->flash_info.led0_ctrl = set_param.led0_ctrl;
	module->flash_info.led1_ctrl = set_param.led1_ctrl;
	module->flash_info.led0_status = set_param.led0_status;
	module->flash_info.led1_status = set_param.led1_status;
	pr_info("led0_ctrl=%d,led1_ctrl=%d\n", set_param.led0_ctrl, set_param.led1_ctrl);

	ret = module->flash_core_handle->flash_core_ops->set_flash(module->flash_core_handle,
		(void *)&set_param);
exit:
	return ret;
}

static int camioctl_flash_cfg(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_flash_cfg_param cfg_parm;

	ret = copy_from_user((void *) &cfg_parm,
		(void __user *)arg,
		sizeof(struct sprd_flash_cfg_param));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		goto exit;
	}
	ret = module->flash_core_handle->flash_core_ops->cfg_flash(module->flash_core_handle,
		(void *)&cfg_parm);

exit:
	return ret;
}

static int camioctl_flash_get(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_flash_capacity flash_info = {0};

	ret = module->flash_core_handle->flash_core_ops->get_flash(module->flash_core_handle,
		(void *)&flash_info);

	ret = copy_to_user((void __user *)arg, (void *)&flash_info,
		sizeof(struct sprd_flash_capacity));

	return ret;
}

static int camioctl_iommu_status_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	unsigned int iommu_enable;
	struct device	*dcam_dev;

	dcam_dev = &module->grp->pdev->dev;
	if (cam_buf_iommu_status_get(CAM_IOMMUDEV_DCAM) == 0)
		iommu_enable = 1;
	else
		iommu_enable = 0;
	module->iommu_enable = iommu_enable;

	ret = copy_to_user((void __user *)arg, &iommu_enable,
		sizeof(unsigned char));

	if (unlikely(ret)) {
		pr_err("fail to copy to user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	pr_info("iommu_enable:%d\n", iommu_enable);
exit:
	return ret;
}

static int camioctl_statis_buf_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct channel_context *ch = NULL;
	struct isp_statis_buf_input statis_buf;

	ret = copy_from_user((void *)&statis_buf,
		(void *)arg, sizeof(struct isp_statis_buf_input));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	if ((statis_buf.type == STATIS_INIT) &&
		(atomic_read(&module->state) != CAM_IDLE) &&
		(atomic_read(&module->state) != CAM_CFG_CH)) {
		pr_err("fail to get init statis buf state: %d\n",
			atomic_read(&module->state));
		ret = -EFAULT;
		goto exit;
	}

	if ((statis_buf.type != STATIS_INIT) &&
		(statis_buf.type < STATIS_TYPE_MAX) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("should not configure statis buf for state %d\n",
			atomic_read(&module->state));
		goto exit;
	}

	if (module->paused && statis_buf.type >= STATIS_AEM && statis_buf.type <= STATIS_LSCM) {
		pr_info("cam%d paused, type %d ignored\n", module->idx, statis_buf.type);
		goto exit;
	}

	if (statis_buf.type < STATIS_HIST2) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_STATIS_BUF,
			&statis_buf);
	}

	if ((statis_buf.type == STATIS_INIT) ||
		(statis_buf.type == STATIS_HIST2)) {
		ch = &module->channel[CAM_CH_PRE];
		if (!ch->enable && module->simulator)
			ch = &module->channel[CAM_CH_CAP];
		if (ch->enable) {
			ret = module->isp_dev_handle->isp_ops->ioctl(module->isp_dev_handle,
			ch->isp_ctx_id,
			ISP_IOCTL_CFG_STATIS_BUF,
			&statis_buf);
		}
	}

	if ((statis_buf.type == STATIS_DBG_INIT) ||
		(statis_buf.type == STATIS_PARAM)) {
		ret = camcore_param_buffer_cfg(module, &statis_buf);
	}

exit:
	return ret;
}

static int camioctl_param_cfg(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	int for_capture = 0, for_fdr = 0;
	int32_t isp_ctx_id;
	struct channel_context *channel;
	struct isp_io_param param;
	struct dcam_pipe_dev *dev = NULL;

	ret = copy_from_user((void *)&param,
		(void *)arg, sizeof(struct isp_io_param));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	if (param.property_param == NULL) {
		pr_err("fail to get user param ptr.\n");
		ret = -EFAULT;
		goto exit;
	}

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	if (dev && dev->raw_fetch_count > 0) {
		pr_warn("raw fetch in progress!\n");
		return 0;
	}

	if ((param.scene_id == PM_SCENE_FDRL) ||
		(param.scene_id == PM_SCENE_FDRH))
		for_fdr = 1;
	for_capture = (param.scene_id == PM_SCENE_CAP ? 1 : 0) | for_fdr;

	if (for_capture &&
		(module->channel[CAM_CH_CAP].enable == 0)) {
		pr_warn("ch scene_id[%d] ch_cap en[%d] ch_pre en[%d]\n",
		param.scene_id,
		module->channel[CAM_CH_CAP].enable,
		module->channel[CAM_CH_PRE].enable);

		return 0;
	}

	if (atomic_read(&module->state) == CAM_STREAM_OFF)
		return 0;

	mutex_lock(&module->fdr_lock);
	if (for_fdr && (module->fdr_init == 0)) {
		ret = camcore_fdr_context_init(module, &module->channel[CAM_CH_CAP]);
		if (unlikely(ret)) {
			pr_err("fail to init fdr\n");
			mutex_unlock(&module->fdr_lock);
			return 0;
		}
	}
	mutex_unlock(&module->fdr_lock);

	if ((param.scene_id == PM_SCENE_FDRL) ||
		(param.scene_id == PM_SCENE_FDRH)) {
		pr_debug("cam%d, cfg FDR scene %d,  blk 0x%x, aux_dcam %pm  dcam_id %d\n",
			module->idx, param.scene_id, param.sub_block,
			module->aux_dcam_dev, module->aux_dcam_id);
	}

	if ((param.sub_block & DCAM_ISP_BLOCK_MASK) == DCAM_BLOCK_BASE) {
		if (for_capture && (module->aux_dcam_dev == NULL)) {
			pr_debug("Config DCAM param for capture. Maybe raw proc\n");
		}

		if (for_capture && (module->aux_dcam_dev != NULL))
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_blk_param(
				module->aux_dcam_dev, &param);
		else {
			if (module->paused) {
				pr_info("cam%d paused, block %x\n", module->idx, param.sub_block);
				return 0;
			}
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_blk_param(
				module->dcam_dev_handle, &param);
		}
	} else {
		if (param.scene_id == PM_SCENE_PRE)
			channel = &module->channel[CAM_CH_PRE];
		else
			channel = &module->channel[CAM_CH_CAP];

		if (channel->enable && channel->isp_ctx_id >= 0) {
			isp_ctx_id = channel->isp_ctx_id;
			if (param.scene_id == PM_SCENE_FDRL)
				isp_ctx_id = channel->isp_fdrl_ctx;
			else if (param.scene_id == PM_SCENE_FDRH)
				isp_ctx_id = channel->isp_fdrh_ctx;
			ret = module->isp_dev_handle->isp_ops->cfg_blk_param(module->isp_dev_handle,
				isp_ctx_id, &param);
		}
	}

exit:
	return ret;
}

static int camioctl_function_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	/*struct sprd_img_sensor_if *dst;*/
	struct sprd_img_function_mode __user *uparam;

	uparam = (struct sprd_img_function_mode __user *)arg;
	ret |= get_user(module->cam_uinfo.is_4in1, &uparam->need_4in1);
	ret |= get_user(module->cam_uinfo.is_3dnr, &uparam->need_3dnr);
	ret |= get_user(module->cam_uinfo.is_dual, &uparam->dual_cam);
	ret |= get_user(module->cam_uinfo.is_afbc, &uparam->need_afbc);
	module->cam_uinfo.is_rgb_ltm = 0;
	module->cam_uinfo.is_yuv_ltm = 0;

	if (module->grp->hw_info->prj_id == SHARKL5pro) {
		module->cam_uinfo.is_rgb_ltm = 1;
		module->cam_uinfo.is_yuv_ltm = 0;
	}

	/* no use */
	module->cam_uinfo.is_3dnr = 0;

	pr_info("4in1:[%d], 3dnr[%d], rgb_ltm[%d], yuv_ltm[%d], dual[%d], afbc[%d]\n",
		module->cam_uinfo.is_4in1,
		module->cam_uinfo.is_3dnr,
		module->cam_uinfo.is_rgb_ltm,
		module->cam_uinfo.is_yuv_ltm,
		module->cam_uinfo.is_dual,
		module->cam_uinfo.is_afbc);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;

	ret = copy_from_user(&module->cam_uinfo.capture_mode,
		(void __user *)arg,
		sizeof(uint32_t));

	pr_info("mode %d\n", module->cam_uinfo.capture_mode);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_cam_security_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	bool sec_ret = 0;
	struct sprd_cam_sec_cfg uparam;

	ret = copy_from_user(&uparam,
		(void __user *)arg,
		sizeof(struct sprd_cam_sec_cfg));

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	pr_info("camca : conn = %d, security mode %d,  u camsec_mode=%d, u work_mode=%d\n",
		module->grp->ca_conn,
		module->grp->camsec_cfg.camsec_mode,
		uparam.camsec_mode, uparam.work_mode);

	if (uparam.camsec_mode != SEC_UNABLE) {
		if (!module->grp->ca_conn)
			module->grp->ca_conn = cam_trusty_connect();

		if (!module->grp->ca_conn) {
			pr_err("fail to init cam_trusty_connect\n");
			ret = -EFAULT;
			goto exit;
		}

		sec_ret = cam_trusty_security_set(&uparam, CAM_TRUSTY_ENTER);

		if (!sec_ret) {
			ret = -EFAULT;
			pr_err("fail to init cam security set\n");
			goto exit;
		}

		module->grp->camsec_cfg.work_mode = uparam.work_mode;
		module->grp->camsec_cfg.camsec_mode = uparam.camsec_mode;

		ret = sprd_iommu_set_cam_bypass(true);

	}  else {
		module->grp->camsec_cfg.work_mode = uparam.work_mode;
		module->grp->camsec_cfg.camsec_mode = uparam.camsec_mode;

		ret = sprd_iommu_set_cam_bypass(false);
	}

exit:
	return ret;
}

static int camioctl_sensor_if_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_sensor_if *dst;

	dst = &module->cam_uinfo.sensor_if;

	ret = copy_from_user(dst,
		(void __user *)arg,
		sizeof(struct sprd_img_sensor_if));
	pr_info("sensor_if %d %x %x, %d.....mipi %d %d %d %d\n",
		dst->if_type, dst->img_fmt, dst->img_ptn, dst->frm_deci,
		dst->if_spec.mipi.use_href, dst->if_spec.mipi.bits_per_pxl,
		dst->if_spec.mipi.is_loose, dst->if_spec.mipi.lane_num);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_sensor_size_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size *dst;

	dst = &module->cam_uinfo.sn_size;

	ret = copy_from_user(dst,
		(void __user *)arg,
		sizeof(struct sprd_img_size));

	pr_info("sensor_size %d %d\n", dst->w, dst->h);
	module->cam_uinfo.dcam_slice_mode = dst->w > DCAM_24M_WIDTH ? CAM_OFFLINE_SLICE_HW : 0;
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_sensor_trim_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_rect *dst;

	dst = &module->cam_uinfo.sn_rect;

	ret = copy_from_user(dst,
		(void __user *)arg,
		sizeof(struct sprd_img_rect));
	pr_info("sensor_trim %d %d %d %d\n", dst->x, dst->y, dst->w, dst->h);
	/* make sure MIPI CAP size is 4 pixel aligned */
	if (unlikely(ret || (dst->w | dst->h) & (DCAM_MIPI_CAP_ALIGN - 1))) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_sensor_max_size_set(
		struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size *dst;

	dst = &module->cam_uinfo.sn_max_size;

	ret = copy_from_user(dst,
		(void __user *)arg,
		sizeof(struct sprd_img_size));
	pr_info("sensor_max_size %d %d\n", dst->w, dst->h);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_cap_skip_num_set(	struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t *dst;

	dst = &module->cam_uinfo.capture_skip;

	ret = copy_from_user(dst,
		(void __user *)arg,
		sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	pr_debug("set cap skip frame %d\n", *dst);
	return 0;
}

static int camioctl_output_size_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t scene_mode;
	uint32_t cap_type;
	uint32_t sn_fmt, dst_fmt;
	struct channel_context *channel = NULL;
	struct camera_uchannel *dst;
	struct sprd_img_parm __user *uparam;

	module->last_channel_id = CAM_CH_MAX;
	uparam = (struct sprd_img_parm __user *)arg;

	ret |= get_user(scene_mode, &uparam->scene_mode);
	ret |= get_user(cap_type, &uparam->need_isp_tool);
	ret |= get_user(sn_fmt, &uparam->sn_fmt);
	ret |= get_user(dst_fmt, &uparam->pixel_fmt);

	pr_info("cam%d, scene %d  cap_type %d, fmt %x %x\n",
		module->idx, scene_mode, cap_type, sn_fmt, dst_fmt);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		goto exit;
	}

	if (((cap_type == CAM_CAP_RAW_FULL) || (dst_fmt == sn_fmt)) &&
		(module->channel[CAM_CH_RAW].enable == 0)) {
		channel = &module->channel[CAM_CH_RAW];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_PREVIEW) &&
		(module->channel[CAM_CH_PRE].enable == 0)) {
		channel = &module->channel[CAM_CH_PRE];
		channel->enable = 1;
	} else if (((scene_mode == DCAM_SCENE_MODE_RECORDING) ||
			(scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK)) &&
			(module->channel[CAM_CH_VID].enable == 0)) {
		channel = &module->channel[CAM_CH_VID];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_CAPTURE) &&
		(module->channel[CAM_CH_CAP].enable == 0)) {
		channel = &module->channel[CAM_CH_CAP];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_CAPTURE_THUMB) &&
		(module->channel[CAM_CH_CAP_THM].enable == 0)) {
		channel = &module->channel[CAM_CH_CAP_THM];
		channel->enable = 1;
	} else if (module->channel[CAM_CH_PRE].enable == 0) {
		channel = &module->channel[CAM_CH_PRE];
		channel->enable = 1;
	} else if (module->channel[CAM_CH_CAP].enable == 0) {
		channel = &module->channel[CAM_CH_CAP];
		channel->enable = 1;
	}

	if (channel == NULL) {
		pr_err("fail to get valid channel\n");
		ret = -EINVAL;
		goto exit;
	}

	module->last_channel_id = channel->ch_id;
	channel->dcam_path_id = -1;
	channel->aux_dcam_path_id = -1;
	channel->isp_ctx_id = -1;
	channel->isp_path_id = -1;
	channel->isp_fdrh_path = -1;
	channel->isp_fdrl_path = -1;
	channel->isp_fdrh_ctx = -1;
	channel->isp_fdrl_ctx = -1;
	channel->slave_isp_ctx_id = -1;
	channel->slave_isp_path_id = -1;

	dst = &channel->ch_uinfo;
	dst->sn_fmt = sn_fmt;
	dst->dst_fmt = dst_fmt;
	ret |= get_user(dst->is_high_fps, &uparam->is_high_fps);
	ret |= get_user(dst->high_fps_skip_num, &uparam->high_fps_skip_num);
	if (dst->high_fps_skip_num == 1) {
		pr_err("fail to get valid high fps %u\n", dst->high_fps_skip_num);
		ret = -EINVAL;
	}
	ret |= copy_from_user(&dst->src_crop,
			&uparam->crop_rect, sizeof(struct sprd_img_rect));
	ret |= copy_from_user(&dst->dst_size,
			&uparam->dst_size, sizeof(struct sprd_img_size));
	ret |= get_user(dst->slave_img_en, &uparam->aux_img.enable);
	ret |= get_user(dst->slave_img_fmt, &uparam->aux_img.pixel_fmt);
	ret |= copy_from_user(&dst->slave_img_size,
		&uparam->aux_img.dst_size, sizeof(struct sprd_img_size));
	ret |= get_user(dst->frame_sync_close, &uparam->reserved[3]);

	// TODO get this from HAL
	dst->is_compressed = 0;
	dst->scene = scene_mode;
	if (cap_type == CAM_CAP_RAW_FULL && dst->is_high_fps)
		dst->is_high_fps = 0;

	pr_info("cam_channel: ch_id %d high fps %u %u. aux %d %d %d %d\n",
		channel->ch_id, dst->is_high_fps, dst->high_fps_skip_num,
		dst->slave_img_en, dst->slave_img_fmt,
		dst->slave_img_size.w, dst->slave_img_size.h);
	pr_info("cam_channel: crop %d %d %d %d dst %d %d\n",
		dst->src_crop.x, dst->src_crop.y,
		dst->src_crop.w, dst->src_crop.h,
		dst->dst_size.w, dst->dst_size.h);
	pr_info("frame sync close %d\n", dst->frame_sync_close);

exit:
	return ret;
}

static int camioctl_ch_id_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;

	if ((atomic_read(&module->state) != CAM_IDLE) &&
		(atomic_read(&module->state) != CAM_CFG_CH)) {
		pr_err("fail to get correct state, state %d\n",
			atomic_read(&module->state));
		return -EFAULT;
	}

	if (module->last_channel_id >= CAM_CH_MAX) {
		ret = -EINVAL;
		goto exit;
	}
	pr_info("cam_channel: get ch id: %d\n", module->last_channel_id);

	ret = copy_to_user((void __user *)arg, &module->last_channel_id,
		sizeof(uint32_t));
	if (unlikely(ret))
		pr_err("fail to copy to user. ret %d\n", ret);

exit:
	/* todo: error handling. */
	atomic_set(&module->state, CAM_CFG_CH);
	return ret;
}

static int camioctl_dcam_path_size(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_dcam_path_size param;

	ret = copy_from_user(
		&param, (void __user *)arg,
		sizeof(struct sprd_dcam_path_size));
	param.dcam_out_w = param.dcam_in_w;
	param.dcam_out_h = param.dcam_in_h;

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	if (module->simulator)
		camcore_channels_set(module, &param);

	if (atomic_read(&module->timeout_flag) == 1)
		pr_info("cam%d, in %d  %d. pre %d %d, vid %d, %d, out %d %d\n",
			module->idx, param.dcam_in_w, param.dcam_in_h,
			param.pre_dst_w, param.pre_dst_h,
			param.vid_dst_w, param.vid_dst_h,
			param.dcam_out_w, param.dcam_out_h);

	ret = copy_to_user((void __user *)arg, &param,
		sizeof(struct sprd_dcam_path_size));

exit:
	return ret;
}

static int camioctl_shrink_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id;
	struct sprd_img_parm __user *uparam;

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_debug("skip\n");
		return ret;
	}

	uparam = (struct sprd_img_parm __user *)arg;

	ret = get_user(channel_id, &uparam->channel_id);
	if (ret == 0 && channel_id < CAM_CH_MAX) {
		ret = copy_from_user(
			&module->channel[channel_id].ch_uinfo.regular_desc,
			(void __user *)&uparam->regular_desc,
			sizeof(struct dcam_regular_desc));
	}

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

 static int camioctl_ebd_control(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_ebd_control ebd_tmp;
	uint32_t channel_id;
	struct sprd_img_parm __user *uparam;

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_debug("skip\n");
		return ret;
	}

	uparam = (struct sprd_img_parm __user *)arg;
	ret = get_user(channel_id, &uparam->channel_id);
	if (ret || (channel_id != CAM_CH_PRE))
		return 0;

	ret = copy_from_user(&ebd_tmp, &uparam->ebd_ctrl,
		sizeof(struct sprd_ebd_control));
	if (unlikely(ret)) {
		pr_err("fail to copy pdaf param from user, ret %d\n", ret);
		return -EFAULT;
	}

	pr_info("MODE: %d, VC:%d, DT:%d\n", ebd_tmp.mode,
		ebd_tmp.image_vc, ebd_tmp.image_dt);

	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_CFG_EBD, &ebd_tmp);

	return ret;
}

static int camioctl_zoom_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;

	ret = copy_from_user(&module->is_smooth_zoom,
		(void __user *)arg,
		sizeof(uint32_t));

	pr_info("is_smooth_zoom %d\n", module->is_smooth_zoom);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int canioctl_crop_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0, zoom = 0;
	uint32_t channel_id;
	struct img_size max;
	struct channel_context *ch, *ch_vid;
	struct sprd_img_rect *crop;
	struct camera_frame *first = NULL;
	struct camera_frame *zoom_param = NULL;
	struct sprd_img_parm __user *uparam;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("module state: %d\n", atomic_read(&module->state));
		return 0;
	}

	uparam = (struct sprd_img_parm __user *)arg;

	ret = get_user(channel_id, &uparam->channel_id);
	ch = &module->channel[channel_id];
	ch_vid = &module->channel[CAM_CH_VID];
	if (ret || (channel_id >= CAM_CH_MAX) || !ch->enable) {
		pr_err("fail to set crop, ret %d, ch %d\n", ret, channel_id);
		ret = -EINVAL;
		goto exit;
	}

	/* CAM_RUNNING: for zoom update
	 * only crop rect size can be re-configured during zoom
	 * and it is forbidden during capture.
	 */
	if (atomic_read(&module->state) == CAM_RUNNING) {
		if ((module->cap_status == CAM_CAPTURE_START) &&
			module->channel[CAM_CH_CAP].enable &&
			(module->channel[CAM_CH_CAP].type_3dnr != CAM_3DNR_OFF)) {
			pr_err("fail to zoom during 3DNR capture\n");
			goto exit;
		}
		crop = kzalloc(sizeof(struct sprd_img_rect), GFP_KERNEL);
		if (crop == NULL) {
			ret = -ENOMEM;
			goto exit;
		}
		zoom_param = cam_queue_empty_frame_get();
		zoom_param->priv_data = (void *)crop;
		zoom = 1;
	} else {
		crop = &ch->ch_uinfo.src_crop;
	}

	ret = copy_from_user(crop,
		(void __user *)&uparam->crop_rect,
		sizeof(struct sprd_img_rect));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		goto exit;
	}

	max.w = module->cam_uinfo.sn_rect.w;
	max.h = module->cam_uinfo.sn_rect.h;

	pr_info("4in1[%d],set ch%d crop %d %d %d %d.\n",
		module->cam_uinfo.is_4in1, channel_id,
		crop->x, crop->y, crop->w, crop->h);
	/* 4in1 prev, enable 4in1 binning, size/2 */
	if (module->cam_uinfo.is_4in1 &&
		((channel_id == CAM_CH_PRE) || (channel_id == CAM_CH_VID))) {
		crop->x >>= 1;
		crop->y >>= 1;
		crop->w >>= 1;
		crop->h >>= 1;
		max.w >>= 1;
		max.h >>= 1;
	}
	/* > 24M, size/2 */
	if (module->cam_uinfo.dcam_slice_mode &&
		((channel_id == CAM_CH_PRE) || (channel_id == CAM_CH_VID))) {
		crop->x >>= 1;
		crop->y >>= 1;
		crop->w >>= 1;
		crop->h >>= 1;
		max.w >>= 1;
		max.h >>= 1;
	}
	crop->x &= ~1;
	crop->y &= ~1;

	/* Sharkl5pro crop align need to do research*/
	crop->w = ((crop->w + DCAM_PATH_CROP_ALIGN - 1)
		& ~(DCAM_PATH_CROP_ALIGN - 1));
	crop->h = ((crop->h + DCAM_PATH_CROP_ALIGN - 1)
		& ~(DCAM_PATH_CROP_ALIGN - 1));
	if ((crop->x + crop->w) > max.w)
		crop->w -= DCAM_PATH_CROP_ALIGN;
	if ((crop->y + crop->h) > max.h)
		crop->h -= DCAM_PATH_CROP_ALIGN;
	pr_info("aligned crop %d %d %d %d.  max %d %d\n",
		crop->x, crop->y, crop->w, crop->h, max.w, max.h);

	if (zoom) {
		if (cam_queue_enqueue(&ch->zoom_coeff_queue, &zoom_param->list)) {
			/* if zoom queue overflow, discard first one node in queue*/
			pr_warn("ch %d zoom q overflow\n", channel_id);
			first = cam_queue_dequeue(&ch->zoom_coeff_queue,
				struct camera_frame, list);
			if (first) {
				kfree(first->priv_data);
				cam_queue_empty_frame_put(first);
			}
			cam_queue_enqueue(&ch->zoom_coeff_queue, &zoom_param->list);
		}
		zoom_param = NULL;
		if (ch_vid->enable && channel_id == CAM_CH_PRE)
			goto exit;
		complete(&module->zoom_thrd.thread_com);
	}
exit:
	if (zoom_param) {
		kfree(zoom_param->priv_data);
		cam_queue_empty_frame_put(zoom_param);
	}
	return ret;
}

static int camioctl_fmt_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_get_fmt fmt_desc;
	struct camera_format *fmt = NULL;

	ret = copy_from_user(&fmt_desc, (void __user *)arg,
		sizeof(struct sprd_img_get_fmt));
	if (unlikely(ret)) {
		pr_err("fail to copy from user ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	if (unlikely(fmt_desc.index >= ARRAY_SIZE(output_img_fmt))) {
		pr_err("fail to get valid index > arrar size\n");
		ret = -EINVAL;
		goto exit;
	}

	fmt = &output_img_fmt[fmt_desc.index];
	fmt_desc.fmt = fmt->fourcc;

	ret = copy_to_user((void __user *)arg,
		&fmt_desc,
		sizeof(struct sprd_img_get_fmt));
	if (unlikely(ret)) {
		pr_err("fail to put user info, GET_FMT, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
exit:
	return ret;
}

static int camioctl_fmt_check(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	enum cam_ch_id channel_id;
	struct sprd_img_format img_format;
	struct channel_context *channel;

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_debug("skip\n");
		return ret;
	}

	pr_debug("check fmt\n");
	ret = copy_from_user(&img_format,
		(void __user *)arg,
		sizeof(struct sprd_img_format));
	if (ret) {
		pr_err("fail to get img_format\n");
		return -EFAULT;
	}

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_err("fail to get module state: %d\n", atomic_read(&module->state));
		return -EFAULT;
	}

	channel_id = (enum cam_ch_id)img_format.channel_id;
	channel = &module->channel[channel_id];

	if (atomic_read(&module->state) == CAM_CFG_CH) {
		/* to do: check & set channel format / param before stream on */
		pr_info("chk_fmt ch %d\n", channel_id);

		ret = camcore_channel_init(module, channel);
		if (ret) {
			/* todo: error handling. */
			pr_err("fail to init channel %d\n", channel->ch_id);
			goto exit;
		}
	}

	img_format.need_binning = 0;
	ret = copy_to_user((void __user *)arg,
		&img_format,
		sizeof(struct sprd_img_format));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		goto exit;
	}
exit:
	return ret;
}

/*
 * SPRD_IMG_IO_PATH_FRM_DECI
 *
 * Set frame deci factor for each channel, which controls the number of dropped
 * frames in ISP output path. It's typically used in slow motion scene. There're
 * two situations in slow motion: preview and recording. HAL will set related
 * parameters according to the table below:
 *
 * ===================================================================================
 * |     scene     |  channel  |  is_high_fps  |  high_fps_skip_num  |  deci_factor  |
 * |---------------|-----------|---------------|---------------------|---------------|
 * |    normal     |  preview  |       0       |          0          |       0       |
 * |    preview    |           |               |                     |               |
 * |---------------|-----------|---------------|---------------------|---------------|
 * |  slow motion  |  preview  |       1       |          4          |       3       |
 * |    preview    |           |               |                     |               |
 * |---------------|-----------|---------------|---------------------|---------------|
 * |               |  preview  |       1       |          4          |       3       |
 * |  slow motion  |           |               |                     |               |
 * |   recording   |-----------|---------------|---------------------|---------------|
 * |               |   video   |       1       |          4          |       0       |
 * |               |           |               |                     |               |
 * ===================================================================================
 *
 * Here, is_high_fps means sensor is running at a high frame rate, thus DCAM
 * slow motion function should be enabled. And deci_factor controls how many
 * frames will be dropped by ISP path before DONE interrupt generated. The
 * high_fps_skip_num is responsible for keeping SOF interrupt running at 30
 * frame rate.
 */
static int camioctl_frm_deci_set(struct camera_module *module,
		unsigned long arg)
{
	struct sprd_img_parm __user *uparam = NULL;
	struct channel_context *ch = NULL;
	uint32_t deci_factor = 0, channel_id = 0;
	int ret = 0;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warn: only for state CFG_CH or RUNNING\n");
		return 0;
	}

	uparam = (struct sprd_img_parm __user *)arg;
	ret |= get_user(channel_id, &uparam->channel_id);
	ret |= get_user(deci_factor, &uparam->deci);
	if (ret) {
		pr_err("fail to get from user. ret %d\n", ret);
		return -EFAULT;
	}

	if ((channel_id >= CAM_CH_MAX) ||
		(module->channel[channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d\n", channel_id);
		return -EPERM;
	}

	ch = &module->channel[channel_id];
	ch->ch_uinfo.deci_factor = deci_factor;

	return ret;
}

static int camioctl_frame_addr_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t i = 0, cmd = ISP_PATH_CFG_OUTPUT_BUF;
	struct sprd_img_parm param;
	struct channel_context *ch = NULL;
	struct channel_context *ch_prv = NULL;
	struct camera_frame *pframe = NULL;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warn: only for state CFG_CH or RUNNING\n");
		return 0;
	}

	ret = copy_from_user(&param, (void __user *)arg,
		sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy from user. ret %d\n", ret);
		return -EFAULT;
	}

	if ((param.channel_id >= CAM_CH_MAX) ||
		(param.buffer_count == 0) ||
		(module->channel[param.channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d. buf cnt %d\n",
			param.channel_id,  param.buffer_count);
		return -EFAULT;
	}

	pr_debug("ch %d, buffer_count %d\n", param.channel_id,
		param.buffer_count);

	if (param.channel_id == CAM_CH_CAP) {
		pr_info("ch %d, buffer_count %d\n", param.channel_id,
		param.buffer_count);
	}

	ch_prv = &module->channel[CAM_CH_PRE];
	ch = &module->channel[param.channel_id];
	for (i = 0; i < param.buffer_count; i++) {
		pframe = cam_queue_empty_frame_get();
		pframe->buf.type = CAM_BUF_USER;
		pframe->buf.mfd[0] = param.fd_array[i];
		pframe->buf.offset[0] = param.frame_addr_array[i].y;
		pframe->buf.offset[1] = param.frame_addr_array[i].u;
		pframe->buf.offset[2] = param.frame_addr_array[i].v;
		pframe->channel_id = ch->ch_id;
		pframe->img_fmt = ch->ch_uinfo.dst_fmt;
		pframe->user_fid = param.user_fid;
		pframe->buf.addr_vir[0] = param.frame_addr_vir_array[i].y;
		pframe->buf.addr_vir[1] = param.frame_addr_vir_array[i].u;
		pframe->buf.addr_vir[2] = param.frame_addr_vir_array[i].v;

		pr_debug("ch %d, mfd 0x%x, off 0x%x 0x%x 0x%x, reserved %d user_fid[%d]\n",
			pframe->channel_id, pframe->buf.mfd[0],
			pframe->buf.offset[0], pframe->buf.offset[1],
			pframe->buf.offset[2], param.is_reserved_buf,
			pframe->user_fid);

		ret = cam_buf_ionbuf_get(&pframe->buf);
		if (ret) {
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}

		if (param.channel_id == CAM_CH_CAP || param.is_reserved_buf) {
			pr_debug("ch %d, mfd 0x%x, off 0x%x 0x%x 0x%x, size 0x%x, reserved %d\n",
				pframe->channel_id, pframe->buf.mfd[0],
				pframe->buf.offset[0], pframe->buf.offset[1],
				pframe->buf.offset[2], (uint32_t)pframe->buf.size[0],
				param.is_reserved_buf);
		}

		if (ch->isp_path_id >= 0 && param.pixel_fmt != IMG_PIX_FMT_GREY) {

			cmd = ISP_PATH_CFG_OUTPUT_BUF;
			if (param.is_reserved_buf) {
				ch->reserved_buf_fd = pframe->buf.mfd[0];
				pframe->is_reserved = 1;
				ch->res_frame = pframe;
				pr_debug("ch %d, mfd 0x%x, off 0x%x 0x%x 0x%x, size %d user_fid[%d]\n",
					pframe->channel_id, pframe->buf.mfd[0],
					pframe->buf.offset[0], pframe->buf.offset[1],
					pframe->buf.offset[2], (uint32_t)pframe->buf.size[0],
					pframe->user_fid);
				continue;
			}
			ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle, cmd,
				ch->isp_ctx_id, ch->isp_path_id, pframe);
		} else {
			cmd = DCAM_PATH_CFG_OUTPUT_BUF;
			if (param.is_reserved_buf) {
				ch->reserved_buf_fd = pframe->buf.mfd[0];
				pframe->is_reserved = 1;
				ch->res_frame = pframe;
				pr_debug("ch %d, mfd 0x%x, off 0x%x 0x%x 0x%x, size %d user_fid[%d]\n",
					pframe->channel_id, pframe->buf.mfd[0],
					pframe->buf.offset[0], pframe->buf.offset[1],
					pframe->buf.offset[2], (uint32_t)pframe->buf.size[0],
					pframe->user_fid);
				continue;
			} else if ((ch->ch_id == CAM_CH_CAP) &&
					param.pixel_fmt == IMG_PIX_FMT_GREY) {
				pframe->img_fmt = param.pixel_fmt;
				cmd = DCAM_PATH_CFG_OUTPUT_ALTER_BUF;
			}

			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				cmd, ch->dcam_path_id, pframe);
			/* 4in1_raw_capture, maybe need two image once */
			if (ch->second_path_enable) {
				ch->pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
				pframe = camcore_secondary_buf_get(&param, ch, i);
				if (!pframe) {
					ret = -EFAULT;
					break;
				}
				ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					cmd, ch->second_path_id, pframe);
			}
		}

		if (ret) {
			pr_err("fail to set output buffer for ch%d.\n",
				ch->ch_id);
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}
	}

	return ret;
}

static int camioctl_frame_id_base_set(	struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id, frame_id_base;
	struct channel_context *ch;
	struct sprd_img_parm __user *uparam;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warn: only for state CFG_CH or RUNNING\n");
		return 0;
	}

	uparam = (struct sprd_img_parm __user *)arg;
	ret = get_user(channel_id, &uparam->channel_id);
	ret |= get_user(frame_id_base, &uparam->frame_base_id);
	if (ret) {
		pr_err("fail to get from user. ret %d\n", ret);
		return -EFAULT;
	}
	if ((channel_id >= CAM_CH_MAX) ||
		(module->channel[channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d\n", channel_id);
		return -EFAULT;
	}

	ch = &module->channel[channel_id];
	ch->frm_base_id = frame_id_base;

	return ret;
}

static int camioctl_stream_off(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t i, j;
	uint32_t raw_cap = 0, running = 0;
	int32_t dcam_path_id;
	struct channel_context *ch = NULL;
	struct channel_context *ch_prv = NULL;
	int isp_ctx_id[CAM_CH_MAX] = { -1 };
	struct cam_hw_info *hw = NULL;

	if ((atomic_read(&module->state) != CAM_RUNNING) &&
		(atomic_read(&module->state) != CAM_CFG_CH)) {
		pr_info("cam%d state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	if (atomic_read(&module->state) == CAM_RUNNING)
		running = 1;

	pr_info("cam %d stream off. state: %d\n",
		module->idx, atomic_read(&module->state));

	ch = &module->channel[CAM_CH_CAP];
	if (ch) {
		mutex_lock(&module->buf_lock[ch->ch_id]);
		if (ch->enable && ch->alloc_start) {
			wait_for_completion(&ch->alloc_com);
			pr_debug("alloc buffer done.\n");
			ch->alloc_start = 0;
		}
		mutex_unlock(&module->buf_lock[ch->ch_id]);
	}

	atomic_set(&module->state, CAM_STREAM_OFF);
	module->cap_status = CAM_CAPTURE_STOP;
	module->dcam_cap_status = DCAM_CAPTURE_STOP;

	hw = module->grp->hw_info;
	/* stop raw dump */
	if (module->dump_thrd.thread_task) {
		if (module->in_dump)
			complete(&module->dump_com);
		mutex_lock(&g_dbg_dump.dump_lock);
		i = module->dcam_idx;
		if (i < 2) {
			g_dbg_dump.dump_start[i] = NULL;
			g_dbg_dump.dump_count = 0;
		}
		mutex_unlock(&g_dbg_dump.dump_lock);
		j = 0;
		while (module->in_dump && (j++ < THREAD_STOP_TIMEOUT)) {
			pr_info("camera%d in dump, wait...%d\n", module->idx, j);
			msleep(10);
		}
	}

	if (running) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->stop(module->dcam_dev_handle, DCAM_STOP);
		if (ret != 0)
			pr_err("fail to stop dcam %d\n", ret);
		camcore_timer_stop(&module->cam_timer);
	}

	if (module->cam_uinfo.is_4in1)
		camcore_4in1_slave_deinit(module);
	if (module->cam_uinfo.dcam_slice_mode && module->aux_dcam_dev)
		camcore_bigsize_slave_deinit(module);

	ch_prv = &module->channel[CAM_CH_PRE];
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		isp_ctx_id[i] = -1;
		if (!ch->enable)
			continue;
		if (ch->ch_id == CAM_CH_RAW) {
			raw_cap = 1;
			if (module->cam_uinfo.is_4in1)
				camcore_4in1_secondary_path_deinit(module, ch);
		}
		if (ch->ch_id == CAM_CH_CAP) {
			mutex_lock(&module->fdr_lock);
			if (module->fdr_init)
				camcore_fdr_context_deinit(module, ch);
			mutex_unlock(&module->fdr_lock);
		}

		pr_info("clear ch %d, dcam path %d, isp path 0x%x\n",
			ch->ch_id,
			ch->dcam_path_id,
			ch->isp_path_id);
		/* prv & vid use same dcam bin path, no need to put it twice */
		if (ch->ch_id == CAM_CH_VID && ch_prv->enable)
			dcam_path_id = -1;
		else
			dcam_path_id = ch->dcam_path_id;
		if (dcam_path_id >= 0) {
			module->dcam_dev_handle->dcam_pipe_ops->put_path(module->dcam_dev_handle,
					ch->dcam_path_id);
		}
		if (ch->isp_path_id >= 0) {
			isp_ctx_id[i] = ch->isp_ctx_id;
			module->isp_dev_handle->isp_ops->put_path(module->isp_dev_handle,
				isp_ctx_id[i],
				ch->isp_path_id);
		}
		if (ch->slave_isp_path_id >= 0) {
			module->isp_dev_handle->isp_ops->put_path(module->isp_dev_handle,
				isp_ctx_id[i],
				ch->slave_isp_path_id);
		}
	}

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable || (ch->ch_id == CAM_CH_RAW))
			continue;
		cam_queue_clear(&ch->zoom_coeff_queue,
			struct camera_frame, list);

		if ((ch->ch_id == CAM_CH_PRE) || (ch->ch_id == CAM_CH_CAP)) {
			if (isp_ctx_id[i] != -1)
				module->isp_dev_handle->isp_ops->put_context(module->isp_dev_handle,
					isp_ctx_id[i]);
			mutex_lock(&module->buf_lock[ch->ch_id]);
			if (ch->alloc_start) {
				wait_for_completion(&ch->alloc_com);
				pr_debug("alloc buffer done.\n");
				ch->alloc_start = 0;
			}
			mutex_unlock(&module->buf_lock[ch->ch_id]);
			if (ch->isp_updata) {
				struct isp_offline_param *cur, *prev;

				cur = (struct isp_offline_param *)ch->isp_updata;
				ch->isp_updata = NULL;
				while (cur) {
					prev = (struct isp_offline_param *)cur->prev;
					pr_info("free %p\n", cur);
					kfree(cur);
					cur = prev;
				}
			}
			cam_queue_clear(&ch->share_buf_queue,
				struct camera_frame, list);

			for (j = 0; j < ISP_NR3_BUF_NUM; j++) {
				if (ch->nr3_bufs[j]) {
					camcore_k_frame_put(ch->nr3_bufs[j]);
					ch->nr3_bufs[j] = NULL;
				}
			}

			if (module->cam_uinfo.is_rgb_ltm) {
				for (j = 0; j < ISP_LTM_BUF_NUM; j++) {
					if (ch->ltm_bufs[LTM_RGB][j]) {
						if (ch->ch_id == CAM_CH_PRE)
							camcore_k_frame_put(ch->ltm_bufs[LTM_RGB][j]);
						ch->ltm_bufs[LTM_RGB][j] = NULL;
					}
				}
			}

			if (module->cam_uinfo.is_yuv_ltm) {
				for (j = 0; j < ISP_LTM_BUF_NUM; j++) {
					if (ch->ltm_bufs[LTM_YUV][j]) {
						if (ch->ch_id == CAM_CH_PRE)
							camcore_k_frame_put(ch->ltm_bufs[LTM_YUV][j]);
						ch->ltm_bufs[LTM_YUV][j] = NULL;
					}
				}
			}

			if (hw->ip_dcam[module->dcam_idx]->superzoom_support) {
				if (ch->postproc_buf) {
					camcore_k_frame_put(ch->postproc_buf);
					ch->postproc_buf = NULL;
					pr_info("postproc put frame\n");
				}
			}
		}
	}

	if (module->dcam_dev_handle) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
				DCAM_IOCTL_DEINIT_STATIS_Q, NULL);
		if (ret != 0)
			pr_err("fail to deinit statis q %d\n", ret);
		ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
				DCAM_IOCTL_PUT_RESERV_STATSBUF, NULL);
	}

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		memset(ch, 0, sizeof(struct channel_context));
		ch->ch_id = i;
		ch->dcam_path_id = -1;
		ch->isp_path_id = -1;
		ch->isp_ctx_id = -1;
		init_completion(&ch->alloc_com);
	}

	if (running) {
		if (atomic_dec_return(&module->grp->runner_nr) == 0)
			module->isp_dev_handle->isp_ops->reset(module->isp_dev_handle, hw);

		/* wait for read thread take all events in frm_queue,
		 * frm_queue max len is CAM_FRAME_Q_LEN
		 * then we loop for this counter.
		 * if read thread has exited unexpectedly,
		 * queue_clear() will clear frm_queue as well
		 */
		j = CAM_FRAME_Q_LEN;
		while (j--) {
			i = cam_queue_cnt_get(&module->frm_queue);
			if (i == 0)
				break;
			pr_info("camera%d wait for read %d %d\n", module->idx, i, j);
			msleep(20);
		}

		if (module->dual_frame) {
			cam_queue_enqueue(&module->zsl_fifo_queue,
				&module->dual_frame->list);
			module->dual_frame = NULL;
		}
		cam_queue_clear(&module->zsl_fifo_queue,
			struct camera_frame, list);
		cam_queue_clear(&module->remosaic_queue,
			struct camera_frame, list);
		if (module->dump_thrd.thread_task)
			cam_queue_clear(&module->dump_queue,
				struct camera_frame, list);
		/* default 0, hal set 1 when needed */
		module->auto_3dnr = 0;
	}

	camcore_param_buffer_uncfg(module);

	atomic_set(&module->state, CAM_IDLE);
	if (raw_cap)
		complete(&module->streamoff_com);

	ret = cam_buf_mdbg_check();
	pr_info("cam %d stream off done.\n", module->idx);

	return ret;
}

static int camioctl_stream_on(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t online = 1;
	uint32_t i, line_w, isp_ctx_id, isp_path_id;
	uint32_t uframe_sync, live_ch_count = 0, shutoff = 0;
	uint32_t timer = 0;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL, *ch_vid = NULL;
	struct cam_hw_info *hw = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_hw_lbuf_share camarg;

	ret = copy_from_user(&online, (void __user *)arg, sizeof(uint32_t));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	online = !!online;
	pr_info("online %d\n", online);

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_info("cam%d error state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	module->flash_skip_fid = 0;
	atomic_set(&module->state, CAM_STREAM_ON);
	pr_info("cam%d stream on starts %d\n", module->idx, online);

	module->simu_fid = 0;
	if (!online)
		goto cfg_ch_done;

	/* settle down compression policy here */
	camcore_compression_cal(module);

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
	if (ch_pre->enable)
		camcore_channel_size_config(module, ch_pre);

	ch_vid = &module->channel[CAM_CH_VID];
	if (ch_vid->enable && !ch_pre->enable)
		camcore_channel_size_config(module, ch_vid);

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable) {
		camcore_channel_size_config(module, ch);
		/* alloc dcam1 memory and cfg out buf */
		if (module->cam_uinfo.dcam_slice_mode)
			camcore_channel_bigsize_config(module, ch);
	}

	ch = &module->channel[CAM_CH_CAP_THM];
	if (ch->enable)
		camcore_channel_size_config(module, ch);
cfg_ch_done:

	/* line buffer share mode setting
	 * Precondition: dcam0, dcam1 size not conflict
	 */
	hw = module->grp->hw_info;
	line_w = module->cam_uinfo.sn_rect.w;
	if (module->cam_uinfo.is_4in1)
		line_w /= 2;
	camarg.idx = module->dcam_idx;
	camarg.width = line_w;
	camarg.offline_flag = 0;
	if (hw->ip_dcam[module->dcam_idx]->lbuf_share_support)
		ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_LBUF_SHARE_SET, &camarg);
	if (ret) {
		pr_err("fail to set line buf share\n");
		ret = -EFAULT;
		goto exit;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_INIT_STATIS_Q, NULL);

	camcore_resframe_set(module);
	for (i = 0;  i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable || (ch->ch_id == CAM_CH_RAW))
			continue;

		live_ch_count++;

		uframe_sync = ch->ch_id != CAM_CH_CAP;
		if (ch->ch_uinfo.frame_sync_close)
			uframe_sync = 0;
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_PATH_UFRAME_SYNC,
			ch->isp_ctx_id,
			ch->isp_path_id,
			&uframe_sync);
		ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_CTX_UFRAME_SYNC,
			ch->isp_ctx_id,
			ch->isp_path_id,
			&uframe_sync);

		cam_queue_init(&ch->zoom_coeff_queue,
			(module->is_smooth_zoom ? CAM_ZOOM_COEFF_Q_LEN : 1),
			camcore_empty_frame_put);

		if (i == CAM_CH_PRE || i == CAM_CH_VID) {
			ret = camcore_buffer_path_cfg(module, i);
			if (ret) {
				pr_err("fail to cfg path buffer\n");
				goto exit;
			}
		}
		ret = camcore_buffer_ltm_cfg(module, i);
		if (ret) {
			pr_err("fail to cfg ltm buffer\n");
			goto exit;
		}
	}

	/* TODO: WORKAROUND for BBAT/factory_test/mini_camera, remove later */
	if (live_ch_count == 1) {
		pr_info("disable all uframe_sync feature\n");

		uframe_sync = 0;
		for (i = 0; i < CAM_CH_MAX; i++) {
			ch = &module->channel[i];
			if (!ch->enable)
				continue;

			isp_ctx_id = ch->isp_ctx_id;
			isp_path_id = ch->isp_path_id;

			module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_PATH_UFRAME_SYNC,
				isp_ctx_id, isp_path_id,
				&uframe_sync);
		}
	}

	cam_queue_init(&module->zsl_fifo_queue,
		CAM_SHARED_BUF_NUM, camcore_k_frame_put);
	/* no need release buffer, only release camera_frame */
	cam_queue_init(&module->remosaic_queue,
		CAM_IRQ_Q_LEN, camcore_camera_frame_release);

	camcore_cap_info_set(module);

	module->dual_frame = NULL;
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	if (module->cam_uinfo.is_4in1) {
		shutoff = 1;
		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dev, DCAM_PATH_CFG_SHUTOFF, DCAM_PATH_BIN, &shutoff);
	}

	if (module->channel[CAM_CH_CAP].enable)
		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dev, DCAM_PATH_CFG_STATE,
			DCAM_PATH_FULL, &module->path_state);

	ret = module->dcam_dev_handle->dcam_pipe_ops->start(module->dcam_dev_handle, online);
	if (ret < 0) {
		pr_err("fail to start dcam dev, ret %d\n", ret);
		goto exit;
	}
	if (module->aux_dcam_dev) {
		ret = module->dcam_dev_handle->dcam_pipe_ops->start(module->aux_dcam_dev, 0);
		if (ret < 0) {
			pr_err("fail to start aux_dcam dev, ret %d\n", ret);
			goto exit;
		}
	}
	atomic_set(&module->state, CAM_RUNNING);
	atomic_set(&module->timeout_flag, 1);

	if (module->cam_uinfo.is_longexp)
		timer = CAMERA_LONGEXP_TIMEOUT;
	else
		timer = CAMERA_TIMEOUT;
	ret = camcore_timer_start(&module->cam_timer, timer);

	if (module->dump_thrd.thread_task) {
		cam_queue_init(&module->dump_queue, 10, camcore_k_frame_put);
		init_completion(&module->dump_com);
		mutex_lock(&g_dbg_dump.dump_lock);
		i = module->dcam_idx;
		if (i < 2) {
			g_dbg_dump.dump_start[i] = &module->dump_thrd.thread_com;
			g_dbg_dump.dump_count = 0;
		}
		mutex_unlock(&g_dbg_dump.dump_lock);
	}
	atomic_inc(&module->grp->runner_nr);

	pr_info("stream on done.\n");
	return 0;

exit:
	atomic_set(&module->state, CAM_CFG_CH);
	pr_info("stream on failed\n");

	/* call stream_off to clear buffers/path */
	camioctl_stream_off(module, 0L);

	return ret;
}

static int camioctl_stream_pause(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t i;
	int32_t dcam_path_id;
	struct channel_context *ch = NULL;
	struct channel_context *ch_prv = NULL;
	struct dcam_pipe_dev *dev;

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_info("cam%d state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	if (module->dcam_dev_handle == NULL) {
		pr_err("fail to cam%d dcam_dev_handle is NULL\n", module->idx);
		return -EFAULT;
	}

	pr_info("cam%d stream pause\n", module->idx);

	module->paused = 1;
	dev = module->dcam_dev_handle;
	dev->dcam_pipe_ops->stop(dev, DCAM_PAUSE_ONLINE);

	ch_prv = &module->channel[CAM_CH_PRE];
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable)
			continue;

		pr_info("clear ch %d, dcam path %d, isp path 0x%x\n",
				ch->ch_id,
				ch->dcam_path_id,
				ch->isp_path_id);
		/* prv & vid use same dcam bin path, no need to put it twice */
		if (ch->ch_id == CAM_CH_VID && ch_prv->enable)
			dcam_path_id = -1;
		else
			dcam_path_id = ch->dcam_path_id;
		if (dcam_path_id >= 0) {
			pr_info("put dcam path %d\n", ch->dcam_path_id);
			dev->dcam_pipe_ops->put_path(dev, ch->dcam_path_id);
		}
	}

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable || (ch->ch_id == CAM_CH_RAW))
			continue;

		if ((ch->ch_id == CAM_CH_PRE) && (ch->isp_updata)) {
			struct isp_offline_param *cur, *prev;

			cur = (struct isp_offline_param *)ch->isp_updata;
			ch->isp_updata = NULL;
			while (cur) {
				prev = (struct isp_offline_param *)cur->prev;
				pr_info("free %p\n", cur);
				kfree(cur);
				cur = prev;
			}
		}
	}

	ret = dev->dcam_pipe_ops->ioctl(dev, DCAM_IOCTL_DEINIT_STATIS_Q, NULL);
	if (ret != 0)
		pr_err("fail to deinit statis q %d\n", ret);

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		ch->dcam_path_id = -1;
	}

	pr_info("cam %d stream pause done.\n", module->idx);

	return ret;
}

static int camioctl_stream_resume(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t i, online = 1;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL, *ch_vid = NULL;
	struct dcam_pipe_dev *dev;

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_info("cam%d error state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	pr_info("cam%d stream resume enter\n", module->idx);

	dev = module->dcam_dev_handle;

	mutex_lock(&module->fdr_lock);
	if (module->fdr_init || module->fdr_done) {
		pr_err("cam%d fdr not deinit, %d %d\n", module->idx,
			module->fdr_init, module->fdr_done);
	}
	mutex_unlock(&module->fdr_lock);

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable)
			continue;
		camcore_channel_init(module, ch);
	}

	if (module->zoom_solution == ZOOM_DEFAULT)
		camcore_channel_size_bininig_cal(module, 1);
	else if (module->zoom_solution == ZOOM_BINNING2 ||
		module->zoom_solution == ZOOM_BINNING4)
		camcore_channel_size_bininig_cal(module, 0);
	else
		camcore_channel_size_rds_cal(module);

	camcore_compression_config(module);

	ch_pre = &module->channel[CAM_CH_PRE];
	if (ch_pre->enable)
		camcore_channel_size_config(module, ch_pre);

	ch_vid = &module->channel[CAM_CH_VID];
	if (ch_vid->enable && !ch_pre->enable)
		camcore_channel_size_config(module, ch_vid);

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable)
		camcore_channel_size_config(module, ch);

	ret = dev->dcam_pipe_ops->ioctl(dev, DCAM_IOCTL_RECFG_PARAM, NULL);

	ret = dev->dcam_pipe_ops->ioctl(dev, DCAM_IOCTL_INIT_STATIS_Q, NULL);

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable)
			continue;

		if (ch->ch_id == CAM_CH_PRE || ch->ch_id == CAM_CH_CAP) {
			/* set shared frame for dcam output */
			while (1) {
				struct camera_frame *pframe = NULL;

				pframe = cam_queue_dequeue(&ch->share_buf_queue,
					struct camera_frame, list);
				if (pframe == NULL)
					break;
				ret = dev->dcam_pipe_ops->cfg_path(dev,
						DCAM_PATH_CFG_OUTPUT_BUF,
						ch->dcam_path_id, pframe);
				if (ret) {
					pr_err("fail to config dcam output buffer\n");
					cam_queue_enqueue(&ch->share_buf_queue, &pframe->list);
					ret = -EINVAL;
					goto exit;
				}
			}
		}
	}

	module->paused = 0;
	camcore_cap_info_set(module);

	ret = dev->dcam_pipe_ops->start(dev, online);
	if (ret < 0) {
		pr_err("fail to start dcam dev, ret %d\n", ret);
		goto exit;
	}

	pr_info("cam%d stream resume done\n", module->idx);
	return 0;

exit:
	pr_info("cam%d stream resume failed\n", module->idx);
	return ret;
}

static int camioctl_cam_res_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	int dcam_idx, retry = 1;
	uint32_t rps_info = 0;
	struct sprd_img_res res = {0};
	struct camera_group *grp = module->grp;
	void *dcam = NULL;
	void *isp = NULL;

	ret = copy_from_user(&res, (void __user *)arg,
		sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	if (atomic_read(&module->state) != CAM_INIT) {
		pr_err("fail to get cam%d state: %d\n",
			module->idx, atomic_read(&module->state));
		return -EFAULT;
	}
	pr_info("cam%d get res, flag %d, sensor %d\n",
		module->idx, res.flag, res.sensor_id);

	dcam_idx = -1;
	rps_info =  !!res.flag;
	module->simulator = rps_info;
	if (unlikely(rps_info)) {
		if (res.sensor_id >= DCAM_ID_MAX) {
			/* invalid sensor id means no senor and any DCAM is OK */
			/* camera offline simulator should set to it */
			dcam_idx = DCAM_ID_0;
			retry = 1;
		} else {
			dcam_idx = res.sensor_id;
		}
	} else if (res.sensor_id < SPRD_SENSOR_ID_MAX) {
		/* get a preferred dcam dev */
		dcam_idx = sprd_sensor_find_dcam_id(res.sensor_id);
	}

check:
	if (!is_dcam_id(dcam_idx) || (dcam_idx < 0)) {
		pr_err("fail to get dcam id for sensor: %d\n", res.sensor_id);
		return -EFAULT;
	}

	dcam = module->dcam_dev_handle;
	if (dcam == NULL) {
		dcam = dcam_core_dcam_if_dev_get(dcam_idx, grp->hw_info);
		if (IS_ERR_OR_NULL(dcam)) {
			if (retry) {
				dcam_idx++;
				goto check;
			}
			pr_err("fail to get dcam%d\n", dcam_idx);
			ret = -EINVAL;
			goto no_dcam;
		}
		module->dcam_dev_handle = dcam;
		module->dcam_idx = dcam_idx;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->open(dcam);
	if (ret) {
		ret = -EINVAL;
		goto dcam_fail;
	}

	pr_info("camca get camera res camsec mode %d.\n",
		module->grp->camsec_cfg.camsec_mode);

	if (module->grp->camsec_cfg.camsec_mode != SEC_UNABLE) {
		bool sec_eb = true;

		ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_SEC, &sec_eb);
	}

	if (ret) {
		pr_err("fail to set cam sec %d.\n", module->grp->camsec_cfg.camsec_mode);
		goto dcam_cb_fail;
	}

	if (rps_info)
		module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_RPS, &rps_info);

	ret = module->dcam_dev_handle->dcam_pipe_ops->set_callback(dcam, camcore_dcam_callback, module);
	if (ret) {
		pr_err("fail to set cam%d callback for dcam.\n", dcam_idx);
		goto dcam_cb_fail;
	}

	isp = module->isp_dev_handle;
	if (isp == NULL) {
		isp = isp_core_pipe_dev_get();
		if (IS_ERR_OR_NULL(isp)) {
			pr_err("fail to get isp\n");
			module->isp_dev_handle = NULL;
			ret = -EINVAL;
			goto no_isp;
		}
		module->isp_dev_handle = isp;
	}

	ret = module->isp_dev_handle->isp_ops->ioctl(module->isp_dev_handle, 0,
		ISP_IOCTL_CFG_SEC, &module->grp->camsec_cfg.camsec_mode);

	if (ret) {
		pr_err("fail to set isp sec %d.\n", module->grp->camsec_cfg.camsec_mode);
		goto wq_fail;
	}

	ret = module->isp_dev_handle->isp_ops->open(isp, grp->hw_info);
	if (ret) {
		pr_err("fail to enable isp module.\n");
		ret = -EINVAL;
		goto isp_fail;
	}


	module->attach_sensor_id = res.sensor_id;

	if (dcam_idx == DCAM_ID_0)
		res.flag = DCAM_RES_DCAM0_CAP | DCAM_RES_DCAM0_PATH;
	else if (dcam_idx == DCAM_ID_1)
		res.flag = DCAM_RES_DCAM1_CAP | DCAM_RES_DCAM1_PATH;
	else
		res.flag = DCAM_RES_DCAM2_CAP | DCAM_RES_DCAM2_PATH;


	pr_debug("sensor %d w %u h %u, cam [%d], dcam %d\n",
		res.sensor_id, res.width, res.height, module->idx, module->dcam_idx);

	pr_info("get camera res for sensor %d res %x done.\n",
		res.sensor_id, res.flag);

	ret = copy_to_user((void __user *)arg, &res,
		sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_to_user\n");
		ret = -EFAULT;
		goto copy_fail;
	}
	atomic_set(&module->state, CAM_IDLE);
	pr_info("cam%d get res done\n", module->idx);
	return 0;

copy_fail:
wq_fail:
	module->isp_dev_handle->isp_ops->close(isp);

isp_fail:
	isp_core_pipe_dev_put(isp);
	module->isp_dev_handle = NULL;

no_isp:
dcam_cb_fail:
	module->dcam_dev_handle->dcam_pipe_ops->close(dcam);

dcam_fail:
	dcam_core_dcam_if_dev_put(dcam);
	module->dcam_dev_handle = NULL;
no_dcam:
	pr_err("fail to get camera res for sensor: %d\n", res.sensor_id);

	return ret;
}

static int camioctl_cam_res_put(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t idx;
	struct sprd_img_res res = {0};

	ret = copy_from_user(&res, (void __user *)arg,
		sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	pr_info("cam%d put res state: %d\n", module->idx,
		atomic_read(&module->state));

	ret = camioctl_stream_off(module, arg);

	if (atomic_read(&module->state) != CAM_IDLE) {
		pr_info("cam%d error state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	idx = module->idx;

	if (module->attach_sensor_id != res.sensor_id) {
		pr_warn("warn: mismatch sensor id: %d, %d for cam %d\n",
			module->attach_sensor_id, res.sensor_id,
			module->idx);
	}

	module->attach_sensor_id = -1;

	if (module->dcam_dev_handle) {
		module->dcam_dev_handle->dcam_pipe_ops->close(module->dcam_dev_handle);
		dcam_core_dcam_if_dev_put(module->dcam_dev_handle);
		module->dcam_dev_handle = NULL;
	}
	if (module->isp_dev_handle) {
		module->isp_dev_handle->isp_ops->close(module->isp_dev_handle);
		isp_core_pipe_dev_put(module->isp_dev_handle);
		module->isp_dev_handle = NULL;
	}

	atomic_set(&module->state, CAM_INIT);

	pr_debug("sensor %d w %u h %u, cam [%d]\n",
		res.sensor_id, res.width, res.height, module->idx);

	pr_info("put camera res for sensor %d res %x done.",
			res.sensor_id, res.flag);
	ret = copy_to_user((void __user *)arg, &res,
			sizeof(struct sprd_img_res));
	if (ret)
		pr_err("fail to copy_to_user!\n");

	return ret;
}

static int camioctl_capture_start(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_capture_param param;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_gtm_ltm_dis dis;
	ktime_t start_time = 0;
	uint32_t idx;
	uint32_t isp_idx = 0;
	uint32_t cap_skip_num = 0;
	uint32_t gtm_param_idx = DCAM_GTM_PARAM_CAP;
	struct channel_context *ch = NULL;

	ret = copy_from_user(&param, (void __user *)arg,
			sizeof(struct sprd_img_capture_param));
	if (ret) {
		pr_err("fail to copy user info\n");
		return -EFAULT;
	}

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_info("cam%d state: %d\n", module->idx,
			atomic_read(&module->state));
		return ret;
	}
	hw = module->grp->hw_info;
	start_time = ktime_get_boottime();
	module->capture_times = start_time;
	ch = &module->channel[CAM_CH_CAP];
	if(ch) {
		mutex_lock(&module->buf_lock[ch->ch_id]);
		if (ch->alloc_start) {
			ret = wait_for_completion_interruptible(&ch->alloc_com);
			if (ret != 0)
				pr_err("fail to config channel/path param work %d\n", ret);
			pr_debug("alloc buffer done.\n");
			ch->alloc_start = 0;
		}
		mutex_unlock(&module->buf_lock[ch->ch_id]);
	}

	module->capture_scene = param.cap_scene;
	isp_idx = module->channel[CAM_CH_CAP].isp_ctx_id;
	if (module->capture_scene == CAPTURE_HDR
		|| module->capture_scene == CAPTURE_FDR
		|| module->capture_scene == CAPTURE_SW3DNR
		|| module->capture_scene == CAPTURE_HW3DNR
		|| module->capture_scene == CAPTURE_FLASH) {
		dis.dcam_idx = module->dcam_idx;
		dis.isp_idx = isp_idx;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_GTM_LTM_DIS, &dis);
	} else if (param.type == DCAM_CAPTURE_START_FROM_NEXT_SOF) {
		module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_GTM_UPDATE, &gtm_param_idx);
	}
	module->isp_dev_handle->isp_ops->clear_stream_ctrl(module->isp_dev_handle, isp_idx);
	if (module->capture_scene == CAPTURE_FDR) {
		struct dcam_path_cfg_param ch_desc;

		/* reconfig full path to raw */
		memset(&ch_desc, 0, sizeof(ch_desc));
		ch_desc.pack_bits = 2;
		ch_desc.is_raw = 1;
		ch_desc.endian.y_endian = ENDIAN_LITTLE;
		ch_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
			DCAM_PATH_CFG_BASE,
			module->channel[CAM_CH_CAP].dcam_path_id, &ch_desc);
	}

	atomic_set(&module->cap_skip_frames, -1);

	/* recognize the capture scene */
	if (param.type == DCAM_CAPTURE_START_FROM_NEXT_SOF) {
		module->dcam_cap_status = DCAM_CAPTURE_START_FROM_NEXT_SOF;
		idx = module->dcam_idx;
		cap_skip_num = hw->dcam_ioctl(hw, DCAM_HW_CFG_GTM_STATUS_GET, &idx);
		atomic_set(&module->cap_skip_frames, cap_skip_num);
		atomic_set(&module->capture_frames_dcam, param.cap_cnt);
		if (module->capture_scene == CAPTURE_SW3DNR && (param.timestamp != 0))
			module->capture_times = param.timestamp;
		else
			module->capture_times = start_time;
	} else if (param.type == DCAM_CAPTURE_START_WITH_TIMESTAMP) {
		module->dcam_cap_status = DCAM_CAPTURE_START_WITH_TIMESTAMP;
		/* dual camera need 1 frame */
		atomic_set(&module->capture_frames_dcam, CAP_NUM_COMMON);
		module->capture_times = param.timestamp;

	} else if (module->cam_uinfo.is_4in1) {
		/* not report when setting */
		atomic_set(&module->capture_frames_dcam, 0);
		if (param.type == DCAM_CAPTURE_START_4IN1_LOWLUX) {
			/* 4in1: low lux mode, report until stop capture
			 * raw capture then not change source of full
			 * maybe need 1 frame
			 */
			if (module->last_channel_id == CAM_CH_RAW) {
				atomic_set(&module->capture_frames_dcam, -1);
				module->lowlux_4in1 = 0;
			} else {
				module->lowlux_4in1 = 1;
				module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
					DCAM_PATH_CFG_FULL_SOURCE,
					DCAM_PATH_FULL,
					&module->lowlux_4in1);
			}
		} else {
			/* 4in1: report 1 frame for remosaic */
			module->lowlux_4in1 = 0;
			atomic_set(&module->capture_frames_dcam, CAP_NUM_COMMON);
		}
		camcore_4in1_channel_size_config(module, module->lowlux_4in1);
	} else if (param.type == DCAM_CAPTURE_START) {
		module->capture_times = param.timestamp;
		module->dcam_cap_status = DCAM_CAPTURE_START;
		atomic_set(&module->capture_frames_dcam, CAP_NUM_COMMON);
	} else {
		atomic_set(&module->capture_frames_dcam, -1);
	}

	if (param.type != DCAM_CAPTURE_STOP)
		module->cap_status = CAM_CAPTURE_START;

	/* alway trigger dump for capture */
	if (module->dump_thrd.thread_task) {
		uint32_t idx = module->dcam_idx;
		struct cam_dbg_dump *dbg = &g_dbg_dump;

		if (idx < 2 && module->dcam_dev_handle) {
			mutex_lock(&dbg->dump_lock);
			if (!(dbg->dump_ongoing & (1 << idx))) {
				complete(&module->dump_thrd.thread_com);
				dbg->dump_count = 99;
				pr_debug("trigger sdump capture raw\n");
			}
			mutex_unlock(&dbg->dump_lock);
		}
	}

	pr_info("cam %d start capture type %d, cnt %d, time %lld\n",
		module->idx, param.type, param.cap_cnt, module->capture_times);
	return ret;
}

static int camioctl_capture_stop(struct camera_module *module,
		unsigned long arg)
{
	struct channel_context *channel = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_gtm_ltm_eb eb;
	uint32_t isp_idx = 0;

	if (module->cap_status == CAM_CAPTURE_STOP) {
		pr_info("cam%d alreay capture stopped\n", module->idx);
		return 0;
	}

	module->cap_status = CAM_CAPTURE_STOP;
	module->dcam_cap_status = DCAM_CAPTURE_STOP;

	pr_info("cam %d stop capture.\n", module->idx);
	hw = module->grp->hw_info;
	isp_idx = module->channel[CAM_CH_CAP].isp_ctx_id;
	if (module->capture_scene == CAPTURE_HDR
		|| module->capture_scene == CAPTURE_FDR
		|| module->capture_scene == CAPTURE_SW3DNR
		|| module->capture_scene == CAPTURE_HW3DNR
		|| module->capture_scene == CAPTURE_FLASH) {
		eb.dcam_idx = module->dcam_idx;
		eb.isp_idx = isp_idx;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_GTM_LTM_EB, &eb);
	}

	if (module->capture_scene == CAPTURE_FDR) {
		struct dcam_path_cfg_param ch_desc;

		pr_info("STOP FDR capture\n");
		memset(&ch_desc, 0, sizeof(ch_desc));
		ch_desc.pack_bits = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
		ch_desc.is_raw = 0;
		ch_desc.endian.y_endian = ENDIAN_LITTLE;
		ch_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_BASE,
				module->channel[CAM_CH_CAP].dcam_path_id, &ch_desc);

		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CLR_OUTPUT_ALTER_BUF,
				module->channel[CAM_CH_CAP].dcam_path_id, &ch_desc);
	}

	/* Handling special case in which stop_capture comes before start_capture.
	 * In this case before assigning NULL to  module->dual_frame, we should check if it points
	 * to some valid frame, if yes then add that frame to dcam path queue to avoid Memroy leak.
	 */
	if (module->cam_uinfo.is_dual && module->dual_frame) {
		channel = &module->channel[module->dual_frame->channel_id];
		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				channel->dcam_path_id, module->dual_frame);
		pr_info("stop capture comes before start capture, dcam_path_Id = %d\n", channel->dcam_path_id);
		module->dual_frame = NULL;
	}
	/* stop dump for capture */
	if (module->dump_thrd.thread_task && module->in_dump) {
		module->dump_count = 0;
		complete(&module->dump_com);
	}
	/* 4in1 lowlux deinit */
	if (module->cam_uinfo.is_4in1 && module->lowlux_4in1) {
		module->lowlux_4in1 = 0;
		module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_FULL_SOURCE,
				DCAM_PATH_FULL,
				&module->lowlux_4in1);
	}
	module->capture_scene = CAPTURE_COMMON;

	return 0;
}

static int camioctl_raw_proc(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	int error_state = 0;
	struct isp_raw_proc_info proc_info;
	uint32_t rps_info;

	ret = copy_from_user(&proc_info, (void __user *)arg,
				sizeof(struct isp_raw_proc_info));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	if (!module->dcam_dev_handle || !module->isp_dev_handle) {
		pr_err("fail to init hw resource.\n");
		return -EFAULT;
	}
	if (proc_info.scene  == RAW_PROC_SCENE_HWSIM_NEW) {
		ret = camcore_raw_post_proc_new(module, &proc_info);
		return ret;
	}

	if (proc_info.scene == RAW_PROC_SCENE_HWSIM) {
		rps_info = 1;
		pr_info("hwsim\n");
		ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
			DCAM_IOCTL_CFG_RPS, &rps_info);
		if (ret != 0) {
			pr_err("fail to config rps %d\n", ret);
			return -EFAULT;
		}
	}

	if ((proc_info.cmd == RAW_PROC_PRE) && (proc_info.scene == RAW_PROC_SCENE_RAWCAP)) {
		mutex_unlock(&module->lock);
		/* raw proc must wait for camera stream off and hw is idle */
		ret = wait_for_completion_interruptible(
				&module->streamoff_com);
		mutex_lock(&module->lock);
		if (ret != 0) {
			pr_err("fail to wait streamoff com %d\n", ret);
			return -EFAULT;
		}
	}

	if (proc_info.scene == RAW_PROC_SCENE_RAWCAP) {

		error_state = ((proc_info.cmd == RAW_PROC_PRE) &&
			(atomic_read(&module->state) != CAM_IDLE));
		error_state |= ((proc_info.cmd == RAW_PROC_POST) &&
			(atomic_read(&module->state) != CAM_CFG_CH));
		if (error_state) {
			pr_info("cam%d rawproc %d error state: %d\n",
					module->idx, proc_info.cmd,
					atomic_read(&module->state));
			return -EFAULT;
		}
	} else {
		pr_info("state[%d]\n", atomic_read(&module->state));
	}

	if (proc_info.cmd == RAW_PROC_PRE) {
		ret = camcore_raw_pre_proc(module, &proc_info);
	} else if (proc_info.cmd == RAW_PROC_POST) {
		ret = camcore_raw_post_proc(module, &proc_info);
	} else if (proc_info.cmd == RAW_PROC_DONE) {
		ret = camcore_raw_proc_done(module);
	} else {
		pr_err("fail to get correct cmd %d\n", proc_info.cmd);
		ret = -EINVAL;
	}
	return ret;
}

static int camioctl_fdr_post(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	int i;
	int32_t isp_path_id, isp_ctx_id;
	void *dcam;
	struct channel_context *ch;
	struct camera_frame *pframe = NULL, *pfrm[3] = { NULL, NULL, NULL };
	struct sprd_img_parm param;

	ret = copy_from_user(&param, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	mutex_lock(&module->fdr_lock);
	if (module->fdr_init == 0) {
		mutex_unlock(&module->fdr_lock);
		pr_debug("deinit_fdr_context may be called for stream off\n");
		return 0;
	}

	ch = &module->channel[CAM_CH_CAP];
	for (i = 0; i < 3; i++) {
		pframe = cam_queue_empty_frame_get();
		pframe->endian = ENDIAN_LITTLE;
		pframe->pattern = module->cam_uinfo.sensor_if.img_ptn;
		pframe->width = ch->ch_uinfo.src_size.w;
		pframe->height = ch->ch_uinfo.src_size.h;
		pframe->user_fid = param.user_fid;
		pframe->fid = param.index;
		pframe->sensor_time.tv_sec = param.reserved[0];
		pframe->sensor_time.tv_usec = param.reserved[1];
		pframe->boot_sensor_time = param.reserved[3];
		pframe->boot_sensor_time = (pframe->boot_sensor_time << 32) | param.reserved[2];
		pframe->buf.type = CAM_BUF_USER;
		pframe->buf.mfd[0] = param.fd_array[i];
		pframe->buf.offset[0] = param.frame_addr_array[i].y;
		pframe->buf.offset[1] = param.frame_addr_array[i].u;
		pframe->buf.offset[2] = param.frame_addr_array[i].v;
		pframe->buf.addr_vir[0] = param.frame_addr_vir_array[i].y;
		pframe->buf.addr_vir[1] = param.frame_addr_vir_array[i].u;
		pframe->buf.addr_vir[2] = param.frame_addr_vir_array[i].v;
		pframe->channel_id = ch->ch_id;
		if (param.scene_mode == FDR_POST_LOW) {
			pframe->irq_property = CAM_FRAME_FDRL;
			isp_path_id = ch->isp_fdrl_path;
			isp_ctx_id = ch->isp_fdrl_ctx;
		} else {
			pframe->irq_property = CAM_FRAME_FDRH;
			isp_path_id = ch->isp_fdrh_path;
			isp_ctx_id = ch->isp_fdrh_ctx;
		}
		ret = cam_buf_ionbuf_get(&pframe->buf);
		if (ret) {
			cam_queue_empty_frame_put(pframe);
			goto exit;
		}
		pfrm[i] = pframe;
	}

	dcam = module->aux_dcam_dev;
	if (dcam == NULL) {
		cam_buf_ionbuf_put(&pfrm[0]->buf);
		cam_queue_empty_frame_put(pfrm[0]);
		goto isp_proc;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dcam,
			DCAM_PATH_CFG_OUTPUT_BUF,
			ch->aux_dcam_path_id, pfrm[1]);
	if (ret) {
		pr_err("fail to cfg dcam out buffer.\n");
		goto exit;
	}

	pframe = pfrm[2];
	pframe->width = ch->ch_uinfo.dst_size.w;
	pframe->height = ch->ch_uinfo.dst_size.h;
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_OUTPUT_BUF,
			isp_ctx_id, isp_path_id, pfrm[2]);
	if (ret) {
		pr_err("fail to cfg isp out buffer.\n");
		goto exit;
	}

	pframe = cam_queue_empty_frame_get();
	memcpy(pframe, pfrm[0], sizeof(struct camera_frame));
	pframe->buf.dmabuf_p[0] = NULL;
	ret = cam_buf_ionbuf_get(&pframe->buf);
	if (ret) {
		cam_queue_empty_frame_put(pframe);
		goto exit;
	}

	if (param.scene_mode == FDR_POST_LOW)
		ch->fdrl_zoom_buf = pframe;
	else
		ch->fdrh_zoom_buf = pframe;

	module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_POSTPROC_BUF,
			isp_ctx_id, isp_path_id, pframe);

	ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(dcam, pfrm[0]);

	pr_info("scene %d, frm fd (%d 0x%x), (%d 0x%x), (%d 0x%x)\n",
		param.scene_mode,
		pfrm[0]->buf.mfd[0], pfrm[0]->buf.offset[0],
		pfrm[1]->buf.mfd[0], pfrm[1]->buf.offset[0],
		pfrm[2]->buf.mfd[0], pfrm[2]->buf.offset[0]);

	mutex_unlock(&module->fdr_lock);
	return 0;


isp_proc:
	ret = module->isp_dev_handle->isp_ops->cfg_path(module->isp_dev_handle,
			ISP_PATH_CFG_OUTPUT_BUF,
			isp_ctx_id,
			isp_path_id, pfrm[2]);
	if (ret) {
		pr_err("fail to cfg isp out buffer.\n");
		goto exit;
	}

	pframe = pfrm[1];
	pr_info("fdr %d , isp path 0x%x ctx_id 0x%x\n", pframe->irq_property, isp_path_id, isp_ctx_id);
	ret = module->isp_dev_handle->isp_ops->proc_frame(module->isp_dev_handle, pframe,
		isp_ctx_id);

	pr_info("scene %d, frm fd (%d 0x%x), (%d 0x%x), (%d 0x%x)\n",
		param.scene_mode,
		pfrm[0]->buf.mfd[0], pfrm[0]->buf.offset[0],
		pfrm[1]->buf.mfd[0], pfrm[1]->buf.offset[0],
		pfrm[2]->buf.mfd[0], pfrm[2]->buf.offset[0]);

	mutex_unlock(&module->fdr_lock);
	return 0;

exit:
	for (i = 0; i < 3; i++) {
		if (pfrm[i]) {
			cam_buf_ionbuf_put(&pfrm[i]->buf);
			cam_queue_empty_frame_put(pfrm[i]);
		}
	}
	mutex_unlock(&module->fdr_lock);
	return ret;
}

/* set addr for 4in1 raw which need remosaic
 */
static int camioctl_4in1_raw_addr_set(struct camera_module *module,
		unsigned long arg)
{
	int ret;
	enum dcam_id idx;
	struct sprd_img_parm param;
	struct channel_context *ch;
	struct camera_frame *pframe;
	int i, j;

	ret = copy_from_user(&param, (void __user *)arg,
		sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}
	idx = module->idx;
	pr_debug("idx %d, channel_id %d, enable %d\n", idx,
		param.channel_id, module->channel[param.channel_id].enable);
	if ((param.channel_id >= CAM_CH_MAX) ||
		(param.buffer_count == 0) ||
		(module->channel[param.channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d. buf cnt %d\n",
			param.channel_id,  param.buffer_count);
		return -EFAULT;
	}
	pr_info("ch %d, buffer_count %d\n", param.channel_id,
		param.buffer_count);

	ch = &module->channel[param.channel_id];
	for (i = 0; i < param.buffer_count; i++) {
		pframe = cam_queue_empty_frame_get();
		pframe->buf.type = CAM_BUF_USER;
		pframe->buf.mfd[0] = param.fd_array[i];
		pframe->buf.addr_vir[0] = param.frame_addr_vir_array[i].y;
		pframe->buf.addr_vir[1] = param.frame_addr_vir_array[i].u;
		pframe->buf.addr_vir[2] = param.frame_addr_vir_array[i].v;
		pframe->channel_id = ch->ch_id;
		pframe->width = ch->ch_uinfo.src_size.w;
		pframe->height = ch->ch_uinfo.src_size.h;
		pr_debug("mfd %d, reserved %d\n", pframe->buf.mfd[0],
			param.is_reserved_buf);
		ret = cam_buf_ionbuf_get(&pframe->buf);
		if (ret) {
			pr_err("fail to get ion buffer\n");
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}

		if (param.is_reserved_buf) {
			ch->reserved_buf_fd = pframe->buf.mfd[0];
			pframe->is_reserved = 1;
			ch->res_frame = pframe;
			for (j = 0; j < 3; j++)
				pframe->buf.size[j] = cal_sprd_raw_pitch(ch->ch_uinfo.src_size.w, module->cam_uinfo.sensor_if.if_spec.mipi.is_loose)
					* ch->ch_uinfo.src_size.h;
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_RESERVED_BUF,
				ch->dcam_path_id, pframe);
		} else {
			ret = module->dcam_dev_handle->dcam_pipe_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				ch->dcam_path_id, pframe);
		}

		if (ret) {
			pr_err("fail to set output buffer for ch%d.\n",
				ch->ch_id);
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}
	}
	pr_info("exit, ret = %d\n", ret);

	return ret;
}

/* buffer set to kernel after remosaic
 */
static int camioctl_4in1_post_proc(struct camera_module *module,
		unsigned long arg)
{
	struct sprd_img_parm param;
	int ret = 0;
	int iommu_enable;
	struct channel_context *channel;
	struct camera_frame *pframe = NULL;
	int i;
	ktime_t sensor_time;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_hw_lbuf_share lbuf;
	struct dcam_hw_binning_4in1 binning;
	uint32_t shutoff = 0;

	ret = copy_from_user(&param, (void __user *)arg,
		sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	pr_info("E\n");
	/* about image data address
	 * fetch: from HAL, in param
	 * dcam out: alloc by kernel, share buf
	 */
	channel = &module->channel[CAM_CH_CAP];
	iommu_enable = module->iommu_enable;
	/* timestamp, reserved	([2]<<32)|[1]
	 * Attention: Only send back one time, maybe need some
	 * change when hal use another time
	 */
	sensor_time = param.reserved[2];
	sensor_time <<= 32;
	sensor_time |= param.reserved[1];
	/* get frame: 1:check id;2:check time;3:get first,4:get new */
	i = cam_queue_cnt_get(&module->remosaic_queue);
	while (i--) {
		pframe = cam_queue_dequeue(&module->remosaic_queue,
			struct camera_frame, list);
		/* check frame id */
		if (pframe == NULL)
			break;
		if (pframe->fid == param.index)
			break;
		if (pframe->boot_sensor_time == sensor_time)
			break;
		cam_queue_enqueue(&module->remosaic_queue, &pframe->list);
		pframe = NULL;
	}
	if (pframe == NULL) {
		pr_info("Can't find frame in the queue, get new one\n");
		pframe = cam_queue_empty_frame_get();
		pframe->boot_sensor_time = sensor_time;
		pframe->sensor_time = ktime_to_timeval(ktime_sub(
			pframe->boot_sensor_time, ktime_sub(
			ktime_get_boottime(), ktime_get())));
		pframe->fid = param.index;
	}
	pframe->endian = ENDIAN_LITTLE;
	pframe->pattern = module->cam_uinfo.sensor_if.img_ptn;
	pframe->width = channel->ch_uinfo.src_size.w;
	pframe->height = channel->ch_uinfo.src_size.h;
	pframe->buf.type = CAM_BUF_USER;
	pframe->buf.mfd[0] = param.fd_array[0];
	pframe->buf.addr_vir[0] = param.frame_addr_vir_array[0].y;
	pframe->buf.addr_vir[1] = param.frame_addr_vir_array[0].u;
	pframe->buf.addr_vir[2] = param.frame_addr_vir_array[0].v;
	pframe->channel_id = channel->ch_id;

	ret = cam_buf_ionbuf_get(&pframe->buf);
	if (ret) {
		cam_queue_empty_frame_put(pframe);
		return -EFAULT;
	}
	pr_info("frame[%d] fd %d addr_vir[0]=0x%lx iova[0]=0x%lx\n",
		pframe->fid, pframe->buf.mfd[0], pframe->buf.addr_vir[0],
		pframe->buf.iova[0]);

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	/*stop full path & start bin path*/
	binning.binning_4in1_en = 0;
	binning.idx = dev->idx;
	dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_BINNING_4IN1_SET, &binning);
	shutoff = 0;
	module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dev, DCAM_PATH_CFG_SHUTOFF, DCAM_PATH_BIN, &shutoff);
	ret = module->dcam_dev_handle->dcam_pipe_ops->start(dev, 0);
	if (ret < 0) {
		pr_err("fail to start dcam dev, ret %d\n", ret);
		goto exit;
	}
	lbuf.idx = module->dcam_idx;
	lbuf.width = pframe->width;
	lbuf.offline_flag = 1;
	if (dev->hw->ip_dcam[module->dcam_idx]->lbuf_share_support)
		dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_LBUF_SHARE_SET, &lbuf);

	ret = module->dcam_dev_handle->dcam_pipe_ops->proc_frame(dev, pframe);

exit:
	return ret;
}

/* set which channel use hw 3dnr
 * if auto 3dnr, will be set when previewing
 */
static int camioctl_3dnr_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_3dnr_mode parm;
	struct channel_context *ch;
	uint32_t ch_id;

	if (!module) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}

	memset((void *)&parm, 0, sizeof(parm));
	ret = copy_from_user(&parm, (void __user *)arg,
		sizeof(parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		return -EFAULT;
	}
	ch_id = parm.channel_id;
	if (ch_id >= CAM_CH_MAX) {
		pr_err("fail to get channel id %d\n", ch_id);
		return -EFAULT;
	}
	ch = &module->channel[ch_id];
	pr_info("ch_id %d, need_3dnr %d\n", ch_id, parm.need_3dnr);

	/* dynamic set when auto 3dnr */
	if (module->auto_3dnr) {
		if (ch->uinfo_3dnr == parm.need_3dnr)
			return ret;

		ch->uinfo_3dnr = parm.need_3dnr;
		camcore_capture_3dnr_set(module, ch);

	} else {
		ch->uinfo_3dnr = parm.need_3dnr;
	}

	return ret;
}

/* set auto_3dnr enable bit to drv
 * 190614: hal set 1 if need, but not set 0(default 0)
 */
static int camioctl_auto_3dnr_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_auto_3dnr_mode parm;

	if (!module) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}

	memset((void *)&parm, 0, sizeof(parm));
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		return -EFAULT;
	}
	module->auto_3dnr = parm.auto_3dnr_enable;
	pr_info("auto_3dnr %d\n", module->auto_3dnr);

	return ret;
}

static int camioctl_capability_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size size = {0};

	if (!module) {
		pr_err("fail to get valid param\n");
		ret = -EINVAL;
		goto exit;
	}

	size.w = ISP_WIDTH_MAX;
	size.h = ISP_HEIGHT_MAX;
	ret = copy_to_user((void __user *)arg, &size,
		sizeof(struct sprd_img_size));
	if (unlikely(ret)) {
		pr_err("fail to get capability, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_path_rect_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_path_rect parm;

	if (!module) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}

	memset((void *)&parm, 0, sizeof(parm));
	ret = copy_from_user(&parm, (void __user *)arg,
		sizeof(struct sprd_img_path_rect));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		return -EFAULT;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->ioctl(module->dcam_dev_handle,
		DCAM_IOCTL_GET_PATH_RECT, &parm);
	pr_debug("TRIM rect info x %d y %d w %d h %d\n",
		parm.trim_valid_rect.x, parm.trim_valid_rect.y,
		parm.trim_valid_rect.w, parm.trim_valid_rect.h);
	pr_debug("AE rect info x %d y %d w %d h %d\n",
		parm.ae_valid_rect.x, parm.ae_valid_rect.y,
		parm.ae_valid_rect.w, parm.ae_valid_rect.h);
	pr_debug("AF rect info x %d y %d w %d h %d\n",
		parm.af_valid_rect.x, parm.af_valid_rect.y,
		parm.af_valid_rect.w, parm.af_valid_rect.h);

	ret = copy_to_user((void __user *)arg, &parm,
		sizeof(struct sprd_img_path_rect));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		return ret;
	}

	return ret;
}

static int camioctl_path_pause(struct camera_module *module,
	unsigned long arg)
{
	int ret = 0;
	uint32_t dcam_path_state = DCAM_PATH_PAUSE;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("pause %d with dcam %d not running\n",
		module->idx, module->dcam_idx);
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	module->path_state = dcam_path_state;
	if (atomic_read(&module->state) == CAM_RUNNING) {
		mutex_lock(&module->lock);
		if (module->channel[CAM_CH_CAP].enable)
			module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dev, DCAM_PATH_CFG_STATE,
				DCAM_PATH_FULL, &dcam_path_state);
		mutex_unlock(&module->lock);
	}

	return ret;
}

static int camioctl_path_resume(struct camera_module *module,
	unsigned long arg)
{
	int ret = 0;
	uint32_t dcam_path_state = DCAM_PATH_RESUME;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("resume %d with dcam %d not running\n",
			module->idx, module->dcam_idx);
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	module->path_state = dcam_path_state;
	if (atomic_read(&module->state) == CAM_RUNNING) {
		mutex_lock(&module->lock);
		if (module->channel[CAM_CH_CAP].enable)
			module->dcam_dev_handle->dcam_pipe_ops->cfg_path(dev, DCAM_PATH_CFG_STATE,
				DCAM_PATH_FULL, &dcam_path_state);
		mutex_unlock(&module->lock);
	}

	return ret;
}

static int camioctl_cam_test(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;

	hw = module->grp->hw_info;
	ret = camt_test(hw, arg);

	return ret;
}

static int camioctl_longexp_mode_set(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_longexp_mode param = {0};

	if (!module) {
		pr_err("fail to get input ptr %p\n", module);
		return -EINVAL;
	}

	ret = copy_from_user(&param, (void __user *)arg, sizeof(param));
	if (ret) {
		pr_err("fail to get longexp mode %d\n", ret);
		return -EFAULT;
	}

	module->cam_uinfo.is_longexp = param.need_longexp;
	pr_debug("is_longexp %d\n", param.need_longexp);

	return ret;
}

#endif
