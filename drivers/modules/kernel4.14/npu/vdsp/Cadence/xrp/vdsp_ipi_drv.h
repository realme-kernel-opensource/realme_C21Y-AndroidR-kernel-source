/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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
#ifndef _VDSP_IPI_DRV_H
#define _VDSP_IPI_DRV_H

#include <linux/interrupt.h>
#include <sprd_vdsp_mm.h>


#define XRP_IRQ_REG_OFFSET    0x00800000


enum xrp_irq_mode {
	XRP_IRQ_NONE,
	XRP_IRQ_LEVEL,
	XRP_IRQ_EDGE,
	XRP_IRQ_EDGE_SW,
	XRP_IRQ_MAX,
};


#define IPI_IDX_0	0
#define IPI_IDX_1	1
#define IPI_IDX_2	2
#define IPI_IDX_3	3
#define IPI_IDX_MAX	4


struct vdsp_ipi_ops;
struct vdsp_ipi_ctx_desc {
	phys_addr_t  base_addr;
	void __iomem *vir_addr;
	void __iomem *ipi_addr;
	/* how IRQ is used to notify the device of incoming data */
	enum xrp_irq_mode irq_mode;
	struct vdsp_ipi_ops *ops;
	spinlock_t ipi_spinlock;
	uint32_t ipi_active;
};

struct vdsp_ipi_ops {
	int (*ctx_init)(struct vdsp_ipi_ctx_desc *ctx);
	int (*ctx_deinit)(struct vdsp_ipi_ctx_desc *ctx);
	irqreturn_t (*irq_handler)(int irq, void *arg);
	int (*irq_register)(int idx, irq_handler_t handler, void *arg);
	int (*irq_unregister)(int idx);
	int (*irq_send)(int idx);
	int (*irq_clear)(int idx);
};


struct vdsp_ipi_ctx_desc *get_vdsp_ipi_ctx_desc(void);


#define IPI_HREG_WR(reg, val) \
		(REG_WR((reg), (val)))

#define IPI_HREG_RD(reg) \
		(REG_RD((reg)))

#define IPI_HREG_MWR(reg, msk, val) \
		(REG_WR((reg), \
		((val) & (msk)) | \
		(REG_RD((reg)) & \
		(~(msk)))))

#define IPI_HREG_OWR(reg, val) \
		(REG_WR((reg), \
		(REG_RD(reg) | (val))))
#endif

