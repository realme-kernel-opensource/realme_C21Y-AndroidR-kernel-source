/*
 * Copyright (C) 2019 Spreadtrum Communications Inc.
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
#ifndef VDSP_SMEM_H
#define VDSP_SMEM_H

#include <linux/types.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include "ion.h"

#define CB_MSG	0
#define CB_MAX	3

typedef phys_addr_t (*mem_cb_t)(void *);

enum {
	IOMMU_MSTI = 0,
	IOMMU_MSTD,
	IOMMU_IDMA,
	IOMMU_VDMA,
	IOMMU_ALL,
};

struct ion_buf {
	struct device *dev;
	int mfd[3];
	unsigned int num; /*valid buf num,max 3*/
	struct sg_table *table[3];
	void *buf[3];
	size_t size[3];
	unsigned long addr_k[3];
	unsigned long iova[3];
	struct dma_buf *dmabuf_p[3];
	unsigned int offset[3];
	unsigned long addr_p[3];
};

struct vdsp_mem_ops;
struct vdsp_mem_desc {
	/*
	 * temporary solution:
	 * dev to record iommu attach device,
	 * other modules can use iommu map / unmap function
	 */
	struct device *dev;

	mem_cb_t  cb_func[CB_MAX];
	void *    cb_args[CB_MAX];
	struct mutex iommu_lock;
	struct vdsp_mem_ops *ops;
};


struct vdsp_mem_ops {
	int (*ctx_init)(struct vdsp_mem_desc *ctx);
	int (*ctx_deinit)(struct vdsp_mem_desc *ctx);
	int (*mem_alloc)(struct vdsp_mem_desc *ctx,
			 struct ion_buf *ion_buf, int heap_type, size_t size);
	int (*mem_free)(struct vdsp_mem_desc *ctx,
			struct ion_buf *buf_info);
	int (*mem_kmap)(struct vdsp_mem_desc *ctx,
			struct ion_buf *buf_info);
	int (*mem_kunmap)(struct vdsp_mem_desc *ctx,
			  struct ion_buf *buf_info);
	int (*mem_kmap_userbuf)(struct ion_buf *buf_info);
	int (*mem_kunmap_userbuf)(struct ion_buf *buf_info);
	int (*mem_iommu_map)(struct vdsp_mem_desc *ctx,
			     struct ion_buf *pfinfo, int idx);
	int (*mem_iommu_unmap)(struct vdsp_mem_desc *ctx,
			       struct ion_buf *pfinfo, int idx);
	int (*mem_get_ionbuf)(struct vdsp_mem_desc *ctx,
			      struct ion_buf *pfinfo);

	int (*mem_register_callback)(struct vdsp_mem_desc *ctx,
				     unsigned int idx, mem_cb_t cb, void *arg);
	int (*mem_unregister_callback)(struct vdsp_mem_desc *ctx,
				       unsigned int idx);
};

struct vdsp_mem_desc *get_vdsp_mem_ctx_desc(struct device *dev);

#endif /* VDSP_SMEM_H */
