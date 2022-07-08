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
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "cpp_common.h"
#include "cpp_reg.h"
#include "sprd_cpp.h"
#include "dma_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DMA_DRV: %d %d %s: " \
	fmt, current->pid, __LINE__, __func__

#define DMA_ADDR_ALIGN 0x07

static inline void cpp_dma_dev_enable(struct dma_drv_private *p)
{
	unsigned long flags;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_EB, CPP_DMA_EB_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static inline void cpp_dma_dev_disable(struct dma_drv_private *p)
{
	unsigned long flags;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_EB, ~CPP_DMA_EB_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static inline void cpp_dma_dev_start(struct dma_drv_private *p)
{
	unsigned long flags;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_START, CPP_DMA_START_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static inline void cpp_dma_dev_stop(struct dma_drv_private *p)
{
	unsigned long flags;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_START, ~CPP_DMA_START_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static int cpp_dma_check_parm(struct sprd_cpp_dma_cfg_parm *parm,
					struct dma_drv_private *p)
{
	if (!parm || !p)
		return -EINVAL;

	pr_info("DMA:\n");
	pr_info("w %d h %d\n", parm->input_size.w, parm->input_size.h);
	pr_info("src y:u:v 0x%x 0x%x 0x%x\n", parm->input_addr.y,
		 parm->input_addr.u, parm->input_addr.v);
	pr_info("dst y:u:v 0x%x 0x%x 0x%x\n", parm->output_addr.y,
		 parm->output_addr.u, parm->output_addr.v);

	if ((parm->input_addr.y & DMA_ADDR_ALIGN) ||
		(parm->input_addr.u & DMA_ADDR_ALIGN) ||
		(parm->input_addr.v & DMA_ADDR_ALIGN) ||
		(parm->output_addr.y & DMA_ADDR_ALIGN) ||
		(parm->output_addr.u & DMA_ADDR_ALIGN) ||
		(parm->output_addr.v & DMA_ADDR_ALIGN)) {
		pr_err("fail to get aligned addr\n");
		return -EINVAL;
	}

	if ((parm->input_size.w % 4 != 0) || (parm->input_size.h % 1 != 0)) {
		pr_err("fail to get aligned width and height:%u, %u\n",
			parm->input_size.w, parm->input_size.h);
		return -EINVAL;
	}

	if ((p->src_offset_x % 2 != 0) || (p->dst_offset_x % 8 != 0)) {
		pr_err("fail to get aligned src and dest offset x:%u, %u\n",
			p->src_offset_x, p->dst_offset_x);
		return -EINVAL;
	}

	if ((p->src_pitch % 8 != 0)
		|| (p->src_pitch < (parm->input_size.w + p->src_offset_x))) {
		pr_err("fail to get aligned src pitch:%u\n", p->src_pitch);
		return -EINVAL;
	}

	if ((p->dst_pitch % 8 != 0)
		|| (p->dst_pitch < (parm->input_size.w + p->dst_offset_x))) {
		pr_err("fail to get aligned dest pitch:%u\n", p->src_pitch);
		return -EINVAL;
	}

	if (parm->input_endian.y_endian >= PATH2_ENDIAN_MAX
		|| parm->input_endian.uv_endian >= PATH2_ENDIAN_MAX) {
		pr_err("fail to get valid input endian %d %d\n",
		parm->input_endian.y_endian,
		parm->input_endian.uv_endian);
		return -EINVAL;
	}

	if (parm->output_endian.y_endian >= PATH2_ENDIAN_MAX
		|| parm->output_endian.uv_endian >= PATH2_ENDIAN_MAX) {
		pr_err("fail to get valid input endian %d %d\n",
		parm->output_endian.y_endian,
		parm->output_endian.uv_endian);
		return -EINVAL;
	}
	return 0;
}

static int cpp_dma_cfg_parm(struct sprd_cpp_dma_cfg_parm *parm,
			struct dma_drv_private *p)
{
	int ret = 0;
	if (!parm || !p)
		return -EINVAL;

	memcpy((void *)&p->cfg_parm, (void *)parm,
		sizeof(struct sprd_cpp_dma_cfg_parm));

	memcpy(p->iommu_src.mfd, parm->input_addr.mfd,
		3 * sizeof(unsigned int));
	memcpy(p->iommu_dst.mfd, parm->output_addr.mfd,
		3 * sizeof(unsigned int));

	ret = cpp_get_sg_table(&p->iommu_src);
	if (ret) {
		pr_err("fail to get cpp sg table\n");
		return -1;
	}
	p->iommu_src.offset[0] = p->cfg_parm.input_addr.y;
	p->iommu_src.offset[1] = p->cfg_parm.input_addr.u;
	ret = cpp_get_addr(&p->iommu_src);
	if (ret) {
		pr_err("fail to get src addr\n");
		return -EFAULT;
	}
	p->dma_src_addr = p->iommu_src.iova[0];

	ret = cpp_get_sg_table(&p->iommu_dst);
	if (ret) {
		pr_err("fail to get cpp sg table\n");
		return -1;
	}
	p->iommu_dst.offset[0] = p->cfg_parm.output_addr.y;
	p->iommu_dst.offset[1] = p->cfg_parm.output_addr.u;
	ret = cpp_get_addr(&p->iommu_dst);
	if (ret) {
		pr_err("fail to get src addr\n");
		return -EFAULT;
	}
	p->dma_dst_addr = p->iommu_dst.iova[0];

	p->dma_size.w = parm->input_size.w;
	p->dma_size.h = parm->input_size.h;


	p->input_endian.y_endian = parm->input_endian.y_endian;
	p->input_endian.uv_endian = parm->input_endian.uv_endian;

	p->output_endian.y_endian = parm->output_endian.y_endian;
	p->output_endian.uv_endian = parm->output_endian.uv_endian;

	return 0;
}

static int cpp_dma_set_parm(struct dma_drv_private *p)
{
	if (!p)
		return -EINVAL;

	/* set src addr */
	if ((p->dma_src_addr & DMA_ADDR_ALIGN) ||
		(p->dma_dst_addr & DMA_ADDR_ALIGN)) {
		pr_err("fail to get aligned src and dst addr\n");
		return -EINVAL;
	}

	reg_wr(p, CPP_DMA_SRC_ADDR, p->dma_src_addr);

	/* set dst addr */
	reg_wr(p, CPP_DMA_DES_ADDR, p->dma_dst_addr);

	/* set image offset/size/pitch */
	reg_wr(p, CPP_DMA_CFG1, ((p->dma_size.w & 0x1FFF)
			 | ((p->dma_size.h & 0x1FFF) << 16)));

	/* set src and dest pitch */
	reg_mwr(p, CPP_DMA_CFG2, CPP_DMA_SRC_PITCH_MASK,
		p->src_pitch);
	reg_mwr(p, CPP_DMA_CFG2, CPP_DMA_DES_PITCH_MASK,
		(p->dst_pitch << 16));

	/* set src offset*/
	reg_mwr(p, CPP_DMA_CFG3, CPP_DMA_SRC_OFFSET_X_MASK,
			p->src_offset_x<<16);
	reg_mwr(p, CPP_DMA_CFG3, CPP_DMA_SRC_OFFSET_Y_MASK,
			p->src_offset_y);

	/* set dest offset */
	reg_mwr(p, CPP_DMA_CFG4, CPP_DMA_DES_OFFSET_X_MASK,
			p->dst_offset_x<<16);
	reg_mwr(p, CPP_DMA_CFG4, CPP_DMA_DES_OFFSET_Y_MASK,
			p->dst_offset_y);

	/* set endian */
	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_INPUT_Y_ENDIAN),
			p->input_endian.y_endian);
	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_INPUT_UV_ENDIAN),
			p->input_endian.uv_endian << 3);

	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_OUTPUT_Y_ENDIAN),
			p->output_endian.y_endian << 4);
	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_OUTPUT_UV_ENDIAN),
			p->output_endian.uv_endian << 7);

	return 0;
}

int cpp_dma_start(struct sprd_cpp_dma_cfg_parm *parm,
			struct dma_drv_private *p)
{
	int ret = 0;

	cpp_dma_dev_stop(p);
	cpp_dma_dev_enable(p);

	ret = cpp_dma_check_parm(parm, p);
	if (ret) {
		pr_err("fail to check dma parm\n");
		return -EINVAL;
	}

	ret = cpp_dma_cfg_parm(parm, p);
	if (ret) {
		pr_err("fail to cfg dma parm\n");
		return -EINVAL;
	}

	ret = cpp_dma_set_parm(p);
	if (ret) {
		pr_err("fail to set dma parm\n");
		return -EINVAL;
	}

	cpp_dma_dev_start(p);
	return ret;
}

void cpp_dma_stop(struct dma_drv_private *p)
{
	/* disable dma */
	cpp_dma_dev_stop(p);
	udelay(1);
	cpp_dma_dev_disable(p);

	cpp_free_addr(&p->iommu_src);
	cpp_free_addr(&p->iommu_dst);
}
