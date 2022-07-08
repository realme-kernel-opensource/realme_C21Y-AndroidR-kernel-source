/*
 * Internal XRP structures definition.
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Alternatively you can use and distribute this file under the terms of
 * the GNU General Public License version 2 or later.
 */

#ifndef XRP_INTERNAL_H
#define XRP_INTERNAL_H

#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include "vdsp_smem.h"
#include "xrp_library_loader.h"
#include "vdsp_dvfs.h"
#include "vdsp_log.h"

struct device;
struct firmware;
struct xrp_hw_ops;
struct xrp_allocation_pool;

struct firmware_origin {
 size_t size;
 u8 *data;
};

struct xrp_comm {
	struct mutex lock;
	void __iomem *comm;
	struct completion completion;
	u32 priority;
};

struct faceid_mem_addr {
	struct ion_buf ion_fd_weights_p;
	struct ion_buf ion_fd_weights_r;
	struct ion_buf ion_fd_weights_o;
	struct ion_buf ion_fp_weights;
	struct ion_buf ion_flv_weights;
	struct ion_buf ion_fo_weights;

	struct ion_buf ion_fd_mem_pool;
};

struct xvp {
	struct device *dev;
	const char *firmware_name;
	const struct firmware *firmware;
	const struct firmware *firmware2_sign;/*faceid sign fw*/
	struct miscdevice miscdev;
	const struct xrp_hw_ops *hw_ops;
	void *hw_arg;
	unsigned n_queues;

	u32 *queue_priority;
	struct xrp_comm *queue;
	struct xrp_comm **queue_ordered;
	void __iomem *comm;
	phys_addr_t pmem;
	phys_addr_t comm_phys;
	phys_addr_t dsp_comm_addr;
	/*ion buff for firmware, comm, dram backup memory*/
	struct ion_buf ion_firmware;
	struct ion_buf ion_comm;
	/*firmware addr infos*/
	void *firmware_viraddr;
	phys_addr_t firmware_phys;
	phys_addr_t dsp_firmware_addr;

	phys_addr_t shared_size;
	atomic_t reboot_cycle;
	atomic_t reboot_cycle_complete;

	bool host_irq_mode;

	struct xrp_allocation_pool *pool;
	bool off;
	int nodeid;
	bool secmode;/*used for faceID*/
	bool tee_con;/*the status of connect TEE*/
	int irq_status;
	struct ion_buf ion_faceid_fw;/*faceid fw*/
	struct ion_buf ion_faceid_fw_sign;/*faceid fw ion which used to sign*/
	struct ion_buf ion_faceid_comm;
	struct faceid_mem_addr faceid_pool;
	const struct firmware *faceid_fw;
	struct vdsp_log_state *log_state;
	struct ion_buf ion_vdsp_log;
	void *fd_weights_p_viraddr;
	struct xrp_load_lib_info load_lib;
	uint32_t open_count;
	struct mutex xvp_lock;
	uint32_t cur_opentype;
	struct vdsp_dvfs_info dvfs_info;
	struct vdsp_mem_desc *vdsp_mem_desc;
	struct hlist_head xrp_known_files[1<<10];
//	spinlock_t xrp_known_files_lock;
	struct mutex xrp_known_files_lock;
	uint32_t sysdump_num;
	uint32_t soc_type;
};

#endif
