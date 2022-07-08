/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>

#include "cpp_common.h"
#include "cpp_reg.h"
#include "sprd_cpp.h"
#include "cpp_core.h"
#include "scale_drv.h"

/* Macro Definition */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "SCALE_DRV: %d %d %s: " \
	fmt, current->pid, __LINE__, __func__

#define PATH0_ADDR_ALIGN 0x07

#define SCALE_LOWEST_ADDR		0x800
#define SCALE_ADDR_INVALID(addr) \
	((unsigned long)(addr) < SCALE_LOWEST_ADDR)
#define SCALE_YUV_ADDR_INVALID(y, u, v) \
	(SCALE_ADDR_INVALID(y) && \
	SCALE_ADDR_INVALID(u) && \
	SCALE_ADDR_INVALID(v))

#define SCALE_FRAME_WIDTH_MAX		8192
#define SCALE_FRAME_HEIGHT_MAX		8192
#define SCALE_FRAME_OUT_WIDTH_MAX	768
#define SCALE_SC_COEFF_MAX		4
#define SCALE_DECI_FAC_MAX		3

#define SCALE_PIXEL_ALIGNED		4
#define ALIGNED_DOWN_2(w) ((w) & ~(2 - 1))
#define ALIGNED_DOWN_4(w) ((w) & ~(4 - 1))
#define ALIGNED_DOWN_8(w) ((w) & ~(8 - 1))

/* Internal Function Implementation */

static inline void scale_dev_stop(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return;
	}

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_START, (~CPP_SCALE_START_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static inline void scale_dev_start(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return;
	}

	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_START, CPP_SCALE_START_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}


static int scale_k_set_src_pitch(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_SRC_PITCH_MASK,
		cfg_parm->input_size.w & CPP_SCALE_SRC_PITCH_MASK);
	return 0;
}

static int scale_k_check_sc_limit(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	/* check for width */
	if (p->sc_input_size.w > (8 * cfg_parm->output_size.w)
	 || p->sc_input_size.w < cfg_parm->output_size.w) {
		pr_err("fail to cfg src_w %d, hor_deci %d, des_w %d\n",
			p->sc_input_size.w,
			p->sc_deci_val,
			cfg_parm->output_size.w);
		ret = -EINVAL;
	}

	/* check for height */
	if (cfg_parm->input_format == SCALE_YUV422
	 && cfg_parm->output_format == SCALE_YUV420) {
		if (p->sc_input_size.h > (4 * cfg_parm->output_size.h)) {
			pr_err("fail to cfg src_h %d >> ver_dec %d > 4 * dst_h %d\n",
				p->sc_input_size.h,
				p->sc_deci_val,
				cfg_parm->output_size.h);
			ret = -EINVAL;
		}
	} else {
		if (p->sc_input_size.h > (8 * cfg_parm->output_size.h)) {
			pr_err("fail to cfg src_h %d >> ver_dec %d > 8 * dst_h %d\n",
				p->sc_input_size.h,
				p->sc_deci_val,
				cfg_parm->output_size.h);
			ret = -EINVAL;
		}
	}

	if (cfg_parm->input_format == SCALE_YUV420
	 && cfg_parm->output_format == SCALE_YUV422) {
		if (p->sc_input_size.h < (2 * cfg_parm->output_size.h)) {
			pr_err("fail to cfg src_h %d >> ver_dec %d < 2 * dst_h %d\n",
				p->sc_input_size.h,
				p->sc_deci_val,
				cfg_parm->output_size.h);
			ret = -EINVAL;
		}
	} else {
		if (p->sc_input_size.h < cfg_parm->output_size.h) {
			pr_err("fail to cfg src_h %d >> ver_dec %d < des_h %d\n",
				p->sc_input_size.h,
				p->sc_deci_val,
				cfg_parm->output_size.h);
			ret = -EINVAL;
		}
	}

	return ret;
}

static int scale_k_set_input_rect(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (scale_k_check_sc_limit(p)) {
		pr_err("fail to get valid src-des size\n");
		return -EINVAL;
	}

	if (cfg_parm->input_rect.x > SCALE_FRAME_WIDTH_MAX
	 || cfg_parm->input_rect.y > SCALE_FRAME_HEIGHT_MAX
	 || cfg_parm->input_rect.w > SCALE_FRAME_WIDTH_MAX
	 || cfg_parm->input_rect.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("fail to get valid input rect %d %d %d %d\n",
			cfg_parm->input_rect.x, cfg_parm->input_rect.y,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);
		return -EINVAL;
	}

	reg_mwr(p, CPP_PATH0_CFG1, CPP_SCALE_SRC_HEIGHT_MASK,
		cfg_parm->input_rect.h << 16);
	reg_mwr(p, CPP_PATH0_CFG1, CPP_SCALE_SRC_WIDTH_MASK,
		cfg_parm->input_rect.w);
	reg_mwr(p, CPP_PATH0_CFG4, CPP_SCALE_SRC_OFFSET_X_MASK,
		cfg_parm->input_rect.x << 16);
	reg_mwr(p, CPP_PATH0_CFG4, CPP_SCALE_SRC_OFFSET_Y_MASK,
		cfg_parm->input_rect.y);

	return 0;
}

static int scale_k_calc_sc_size(struct scale_drv_private *p)
{
	int i = 0;
	unsigned int div_factor = 1;
	unsigned int deci_val = 0;
	unsigned int pixel_aligned_num = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_H_MASK, 0 << 4);
	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_V_MASK, 0 << 6);

	if (unlikely(cfg_parm->input_rect.w >
		(cfg_parm->output_size.w * SCALE_SC_COEFF_MAX *
		(1 << SCALE_DECI_FAC_MAX))
		|| cfg_parm->input_rect.h >
		(cfg_parm->output_size.h * SCALE_SC_COEFF_MAX *
		(1 << SCALE_DECI_FAC_MAX)))) {
		pr_err("fail to get input rect %d %d, output size %d %d\n",
			cfg_parm->input_rect.w, cfg_parm->input_rect.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
		return -EINVAL;
	}

	p->sc_input_size.w = cfg_parm->input_rect.w;
	p->sc_input_size.h = cfg_parm->input_rect.h;

	/* check for decimation */
	if (cfg_parm->input_rect.w
		> (cfg_parm->output_size.w * SCALE_SC_COEFF_MAX)
		|| cfg_parm->input_rect.h
		> (cfg_parm->output_size.h * SCALE_SC_COEFF_MAX)) {

		for (i = 1; i < SCALE_DECI_FAC_MAX; i++) {
			div_factor =
			(unsigned int)(SCALE_SC_COEFF_MAX * (1 << i));
			if ((cfg_parm->input_rect.w
			<= (cfg_parm->output_size.w * div_factor))
			&& (cfg_parm->input_rect.h
			<= (cfg_parm->output_size.h * div_factor))) {
				break;
			}
		}
		deci_val = (1 << i);
		pixel_aligned_num =
			(deci_val >= SCALE_PIXEL_ALIGNED)
			? deci_val : SCALE_PIXEL_ALIGNED;

		p->sc_input_size.w = cfg_parm->input_rect.w >> i;
		p->sc_input_size.h = cfg_parm->input_rect.h >> i;

		if ((p->sc_input_size.w % pixel_aligned_num)
		 || (p->sc_input_size.h % pixel_aligned_num)) {
			p->sc_input_size.w =
				p->sc_input_size.w
			/ pixel_aligned_num
				* pixel_aligned_num;
			p->sc_input_size.h =
			p->sc_input_size.h
			/ pixel_aligned_num
			* pixel_aligned_num;

			cfg_parm->input_rect.w =
				p->sc_input_size.w << i;
			cfg_parm->input_rect.h =
				p->sc_input_size.h << i;
		}
		p->sc_deci_val = i;

		reg_mwr(p, CPP_PATH0_CFG0,
			CPP_SCALE_DEC_H_MASK, i << 4);
		reg_mwr(p, CPP_PATH0_CFG0,
			CPP_SCALE_DEC_V_MASK, i << 6);
	}

	pr_debug("sc_input_size %d %d, deci %d input_rect %d %d\n",
		p->sc_input_size.w,
		p->sc_input_size.h,
		i,
		cfg_parm->input_rect.w,
		cfg_parm->input_rect.h);

	return 0;
}

static int scale_k_check_param(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;
	int ret = 0;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_size.w > SCALE_FRAME_WIDTH_MAX ||
		cfg_parm->input_size.h > SCALE_FRAME_HEIGHT_MAX ||
		cfg_parm->output_size.w > SCALE_FRAME_OUT_WIDTH_MAX ||
		cfg_parm->output_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("fail to get valid input or output size:%d %d %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
		ret = -1;
		goto exit;
	} else if (cfg_parm->input_size.w < cfg_parm->input_rect.w +
		cfg_parm->input_rect.x ||
		cfg_parm->input_size.h < cfg_parm->input_rect.h +
		cfg_parm->input_rect.y) {
		pr_err("fail to get valid input size %d %d %d %d %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h,
			cfg_parm->input_rect.x, cfg_parm->input_rect.y,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);
		ret = -1;
		goto exit;
	} else {
		if (cfg_parm->output_size.w % 8 != 0) {
			pr_err("fail to get dst width 8 byte align: %d\n",
					cfg_parm->input_rect.x);
			ret = -1;
			goto exit;
		}
		if (cfg_parm->output_format == SCALE_YUV420)
			if (cfg_parm->output_size.h % 2 != 0) {
			pr_err("fail to get dst height 2 byte align: %d\n",
					cfg_parm->input_rect.x);
			ret = -1;
			goto exit;
			}
		if (cfg_parm->input_size.w % 8 != 0) {
			pr_err("fail to get src scale pitch size %d\n",
				cfg_parm->input_size.w);
			ret = -1;
			goto exit;
		}
		if (cfg_parm->input_format == SCALE_YUV420) {
			if (cfg_parm->input_rect.h % 2 != 0) {
				cfg_parm->input_rect.h =
					ALIGNED_DOWN_2(cfg_parm->input_rect.h);
				pr_info("adjust src height align 2: %d\n",
					cfg_parm->input_rect.y);
			}
			if (cfg_parm->input_rect.y % 2 != 0) {
				cfg_parm->input_rect.y =
					ALIGNED_DOWN_2(cfg_parm->input_rect.y);
				pr_info("adjust src offset y align 2: %d\n",
					cfg_parm->input_rect.y);
			}
		}
		if (cfg_parm->input_rect.w % 4 != 0) {
			cfg_parm->input_rect.w =
				ALIGNED_DOWN_4(cfg_parm->input_rect.w);
			pr_info("adjust src width align 4: %d\n",
					cfg_parm->input_rect.y);
		}
		if (cfg_parm->input_rect.x % 2 != 0) {
			cfg_parm->input_rect.x =
				ALIGNED_DOWN_2(cfg_parm->input_rect.x);
			pr_info("adjust src offset x align 2: %d\n",
					cfg_parm->input_rect.x);
		}
	}
exit:
	return ret;
}
static int scale_k_set_input_size(struct scale_drv_private *p)
{
	int ret = 0;

	ret = scale_k_calc_sc_size(p);
	if (ret) {
		pr_err("fail to configure scaler\n");
		return ret;
	}

	ret = scale_k_set_input_rect(p);
	if (ret) {
		pr_err("fail to set input rect size\n");
		return ret;
	}

	return ret;
}

static int scale_k_set_input_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (!(cfg_parm->input_format == SCALE_YUV422
	 || cfg_parm->input_format == SCALE_YUV420)) {
		pr_err("fail to get valid input format %d\n",
			cfg_parm->input_format);
		return -EINVAL;
	}

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_INPUT_FORMAT,
		(cfg_parm->input_format << 2));

	return 0;
}


static int scale_k_set_addr(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_addr.mfd[0] == 0 ||
		cfg_parm->input_addr.mfd[1] == 0 ||
		cfg_parm->output_addr.mfd[0] == 0 ||
		cfg_parm->output_addr.mfd[1] == 0) {
		pr_err("fail to get valid input mfd %d %d %d\n",
			cfg_parm->input_addr.mfd[0],
			cfg_parm->input_addr.mfd[1],
			cfg_parm->input_addr.mfd[2]);
		pr_err("fail to get valid output mfd %d %d %d\n",
			cfg_parm->output_addr.mfd[0],
			cfg_parm->output_addr.mfd[1],
			cfg_parm->output_addr.mfd[2]);
		ret = -1;
		goto exit;
	} else {
		memcpy(p->iommu_src.mfd, cfg_parm->input_addr.mfd,
			3 * sizeof(unsigned int));
		memcpy(p->iommu_dst.mfd, cfg_parm->output_addr.mfd,
			3 * sizeof(unsigned int));

		ret = cpp_get_sg_table(&p->iommu_src);
		if (ret) {
			pr_err("fail to get cpp src sg table\n");
			return -1;
		}
		p->iommu_src.offset[0] = cfg_parm->input_addr.y;
		p->iommu_src.offset[1] = cfg_parm->input_addr.u;
		p->iommu_src.offset[2] = cfg_parm->input_addr.v;
		ret = cpp_get_addr(&p->iommu_src);
		if (ret) {
			pr_err("fail to get cpp src addr\n");
			return -1;
		}

		ret = cpp_get_sg_table(&p->iommu_dst);
		if (ret) {
			pr_err("fail to get cpp dst sg table\n");
			cpp_free_addr(&p->iommu_src);
			return ret;
		}
		p->iommu_dst.offset[0] = cfg_parm->output_addr.y;
		p->iommu_dst.offset[1] = cfg_parm->output_addr.u;
		p->iommu_dst.offset[2] = cfg_parm->output_addr.v;
		ret = cpp_get_addr(&p->iommu_dst);
		if (ret) {
			pr_err("fail to get cpp dst addr\n");
			cpp_free_addr(&p->iommu_src);
			return ret;
		}

		reg_wr(p, CPP_PATH0_SRC_ADDR_Y,
			p->iommu_src.iova[0]);
		reg_wr(p, CPP_PATH0_SRC_ADDR_UV,
			p->iommu_src.iova[1]);
		reg_wr(p, CPP_PATH0_DES_ADDR_Y,
			p->iommu_dst.iova[0]);
		reg_wr(p, CPP_PATH0_DES_ADDR_UV,
			p->iommu_dst.iova[1]);

		pr_debug("iommu_src y:0x%lx, uv:0x%lx\n",
			p->iommu_src.iova[0], p->iommu_src.iova[1]);
		pr_debug("cpp iommu_dst y:0x%lx, uv:0x%lx\n",
			p->iommu_dst.iova[0], p->iommu_dst.iova[1]);
	}
exit:
	return ret;
}

static int scale_k_set_input_endian(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;
	int ret = 0;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_endian.y_endian >= SCALE_ENDIAN_MAX
	 || cfg_parm->input_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_err("fail to get valid input endian %d %d\n",
			cfg_parm->input_endian.y_endian,
			cfg_parm->input_endian.uv_endian);
		ret = -1;
		goto exit;
	} else {
		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE
		 && cfg_parm->input_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_INPUT_Y_ENDIAN),
			y_endian);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_INPUT_UV_ENDIAN),
			uv_endian << 3);

		pr_debug("input endian y:%d, uv:%d\n", y_endian, uv_endian);
	}

exit:
	return ret;
}

static int scale_k_set_des_pitch(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_DES_PITCH_MASK,
		cfg_parm->output_size.w << 16);

	return 0;
}

static int scale_k_set_output_size(struct scale_drv_private *p)
{

	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG2, CPP_SCALE_DES_WIDTH_MASK,
			cfg_parm->output_size.w);



	reg_mwr(p, CPP_PATH0_CFG2, CPP_SCALE_DES_HEIGHT_MASK,
			cfg_parm->output_size.h << 16);

	reg_wr(p, CPP_PATH0_CFG5, 0);

	return 0;
}

static int scale_k_set_output_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (!(cfg_parm->output_format == SCALE_YUV422
	 || cfg_parm->output_format == SCALE_YUV420)) {
		pr_err("fail to get valid output format %d\n",
			cfg_parm->output_format);
		return -EINVAL;
	}

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_OUTPUT_FORMAT,
		(cfg_parm->output_format << 4));

	return 0;
}


static int scale_k_set_output_endian(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;
	int ret = 0;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_endian.y_endian >= SCALE_ENDIAN_MAX
	 || cfg_parm->output_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_err("fail to get valid output endian %d %d\n",
			cfg_parm->output_endian.y_endian,
			cfg_parm->output_endian.uv_endian);
		ret = -1;
		goto exit;
	} else {
		if (cfg_parm->output_endian.y_endian == SCALE_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->output_endian.y_endian == SCALE_ENDIAN_LITTLE
		 && cfg_parm->output_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_OUTPUT_Y_ENDIAN),
			y_endian << 4);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_OUTPUT_UV_ENDIAN),
			uv_endian << 7);

		pr_debug("output endian y:%d, uv:%d\n",
			y_endian, uv_endian);
	}
exit:
	return ret;
}

static inline void scale_dev_enable(struct scale_drv_private *p)
{
	unsigned long flags;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return;
	}
	cfg_parm = &p->cfg_parm;
	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_EB, CPP_SCALE_PATH_EB_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static inline void scale_dev_disable(struct scale_drv_private *p)
{
	unsigned long flags;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_EB, (~CPP_SCALE_PATH_EB_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void cpp_reg_trace(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return;
	}
	cfg_parm = &p->cfg_parm;
#ifdef SCALE_DRV_DEBUG
	unsigned long addr = 0;

	pr_info("CPP: Register list\n");
	for (addr = CPP_BASE; addr <= CPP_END; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			reg_rd(p, addr), reg_rd(p, addr + 4),
			reg_rd(p, addr + 8), reg_rd(p, addr + 12));
	}
#endif
}

static void scale_k_set_qos(struct scale_drv_private *p)
{
	reg_mwr(p, CPP_AXIM_CHN_SET, CPP_AXIM_CHN_SET_QOS_MASK, (0x1 << 16));
	reg_mwr(p, CPP_MMU_PT_UPDATE_QOS, CPP_MMU_PT_UPDATE_QOS_MASK, 0x1);
}

void get_cpp_max_size(unsigned int *max_width, unsigned int *max_height)
{
	*max_width = SCALE_FRAME_OUT_WIDTH_MAX;
	*max_height = SCALE_FRAME_HEIGHT_MAX;
}

int cpp_scale_start(struct sprd_cpp_scale_cfg_parm *parm,
			struct scale_drv_private *p)
{
	int ret = 0;

	if (!parm || !p) {
		pr_err("fail to get Input ptr\n");
		return -EINVAL;
	}
	memset(&p->sc_input_size, 0, sizeof(struct sprd_cpp_size));
	p->sc_deci_val = 0;

	memcpy((void *)&p->cfg_parm, (void *)parm,
		sizeof(struct sprd_cpp_scale_cfg_parm));

	scale_dev_stop(p);
	scale_dev_enable(p);

	ret = scale_k_check_param(p);
	if (ret) {
		pr_err("fail to check size param\n");
		return ret;
	}

	ret = scale_k_set_src_pitch(p);
	if (ret)
		goto exit;

	ret = scale_k_set_des_pitch(p);
	if (ret)
		goto exit;

	ret = scale_k_set_output_size(p);
	if (ret)
		goto exit;

	ret = scale_k_set_input_size(p);
	if (ret)
		goto exit;

	ret = scale_k_set_input_format(p);
	if (ret)
		goto exit;

	ret = scale_k_set_output_format(p);
	if (ret)
		goto exit;


	ret = scale_k_set_input_endian(p);
	if (ret)
		goto exit;

	ret = scale_k_set_output_endian(p);
	if (ret)
		goto exit;

	ret = scale_k_set_addr(p);
	if (ret) {
		pr_err("fail to set addr\n");
		return ret;
	}

	pr_debug("in_size %d %d in_rect %d %d %d %d out_size %d %d dc_val %d\n",
		p->cfg_parm.input_size.w, p->cfg_parm.input_size.h,
		p->cfg_parm.input_rect.x, p->cfg_parm.input_rect.y,
		p->cfg_parm.input_rect.w, p->cfg_parm.input_rect.h,
		p->cfg_parm.output_size.w, p->cfg_parm.output_size.h,
		p->sc_deci_val);
	pr_debug("in_addr 0x%x 0x%x out_addr 0x%x 0x%x\n",
		p->cfg_parm.input_addr.y, p->cfg_parm.input_addr.u,
		p->cfg_parm.output_addr.y, p->cfg_parm.output_addr.u);

	scale_k_set_qos(p);
	scale_dev_start(p);

exit:
	cpp_reg_trace(p);

	return ret;
}

void cpp_scale_stop(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get Input ptr\n");
		return;
	}
	cfg_parm = &p->cfg_parm;
	scale_dev_stop(p);
	scale_dev_disable(p);
	cpp_free_addr(&p->iommu_src);
	cpp_free_addr(&p->iommu_dst);
}

int cpp_scale_capability(struct sprd_cpp_scale_capability *scale_param)
{
	int ret = 0;

	if (scale_param->dst_size.w > scale_param->src_size.w ||
		scale_param->dst_size.h > scale_param->src_size.h) {
		pr_err("fail to upscale src.w:%d h:%d dst w:%d h:%d\n",
			scale_param->src_size.w, scale_param->src_size.h,
			scale_param->dst_size.w, scale_param->dst_size.h);
		return -1;
	}
	if (scale_param->src_size.w > SCALE_FRAME_WIDTH_MAX ||
		scale_param->src_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("fail to get valid src size width:%d height:%d\n",
			scale_param->src_size.w, scale_param->src_size.h);
		return -1;
	}
	if (scale_param->src_size.w % 8 != 0) {
		pr_err("fail to align scale src width 8 byte: %d\n",
				scale_param->src_size.w);
		return -1;
	}

	if (scale_param->dst_size.w > SCALE_FRAME_OUT_WIDTH_MAX
	 || scale_param->dst_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("fail to get valid dst size width:%d height:%d\n",
			scale_param->dst_size.w, scale_param->dst_size.h);
		return -1;
	}
	if (scale_param->dst_size.w % 8 != 0) {
		pr_err("fail to align scale dst width 8 byte: %d\n",
			scale_param->dst_size.w);
		return -1;
	}

	if ((scale_param->dst_format == SCALE_YUV420)
		&& (scale_param->dst_size.h % 2 != 0)) {
		pr_err("fail to get align scale dst height 2 byte:%d\n",
			scale_param->dst_size.h);
		return -1;
	}
	return ret;
}
