/*
 * XRP: Linux device driver for Xtensa Remote Processing
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

#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)
#include <linux/dma-mapping.h>
#else
#include <linux/dma-direct.h>
#endif
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/soc/sprd/sprd_systimer.h>
#include <linux/soc/sprd/hwfeature.h>
#include <linux/timer.h>
#include <asm/mman.h>
#include <asm/uaccess.h>
#include "xrp_firmware.h"
#include "vdsp_hw.h"
#include "xrp_internal.h"
#include "xrp_kernel_defs.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_private_alloc.h"
#include "xvp_main.h"
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include "ion.h"
#include "vdsp_smem.h"
#include "vdsp_ipi_drv.h"
#include "xrp_faceid.h"
#include "vdsp_log.h"
#include "vdsp_dvfs.h"
#include "vdsp_dump.h"

#ifndef __io_virt
#define __io_virt(a) ((void __force *)(a))
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "sprd-vdsp: xvp_main %d %d %s : "\
        fmt, current->pid, __LINE__, __func__

#define XRP_DEFAULT_TIMEOUT 20

enum {
	/* normal work mode */
	LOOPBACK_NORMAL,
	/* don't communicate with FW, but still load it and control DSP */
	LOOPBACK_NOIO,
	/* don't comminicate with FW or use DSP MMIO, but still load the FW */
	LOOPBACK_NOMMIO,
	/* don't communicate with FW or use DSP MMIO, don't load the FW */
	LOOPBACK_NOFIRMWARE,
};

static int log_counter = 0;
static struct mutex xvp_global_lock;
static struct semaphore log_start;
static unsigned long firmware_command_timeout = XRP_DEFAULT_TIMEOUT;
static unsigned long firmware_boot_timeout = 5;
module_param(firmware_command_timeout, ulong, 0644);
MODULE_PARM_DESC(
	firmware_command_timeout,
	"Firmware command timeout in seconds.");
static int firmware_reboot = 1;
module_param(firmware_reboot, int, 0644);
MODULE_PARM_DESC(
	firmware_reboot,
	"Reboot firmware on command timeout.");
static int loopback = 0;
module_param(loopback, int, 0644);
MODULE_PARM_DESC(loopback,
	"Don't use actual DSP, perform everything locally.");
//static DEFINE_HASHTABLE(xrp_known_files, 10);
//static DEFINE_SPINLOCK(xrp_known_files_lock);
static DEFINE_IDA(xvp_nodeid);
static int xrp_boot_firmware(struct xvp *xvp);
static inline void xvp_dsp_setdvfs(struct xvp *xvp, uint32_t index);
static inline void xvp_dsp_enable_dvfs(struct xvp *xvp);
static inline void xvp_dsp_disable_dvfs(struct xvp *xvp);
static long xrp_ioctl_submit_sync(struct file *filp,
                                struct xrp_ioctl_queue __user *p ,
				struct xrp_request *pkernel);

static void init_files_know_info(struct xvp* xvp)
{
	int i;
	for(i = 0; i < (1<<10); i ++)
		xvp->xrp_known_files[i].first = NULL;
//        spin_lock_init(&xvp->xrp_known_files_lock);
	mutex_init(&xvp->xrp_known_files_lock);
}
static int compare_queue_priority(const void *a, const void *b)
{
	const void * const *ppa = a;
	const void * const *ppb = b;
	const struct xrp_comm *pa = *ppa, *pb = *ppb;

	if (pa->priority == pb->priority)
		return 0;
	else
		return pa->priority < pb->priority ? -1 : 1;
}

void sprd_log_sem_init(void)
{
	sema_init(&log_start, 0);
}
EXPORT_SYMBOL_GPL(sprd_log_sem_init);

void sprd_log_sem_up(void)
{
	up(&log_start);
}
EXPORT_SYMBOL_GPL(sprd_log_sem_up);

void sprd_log_sem_down(void)
{
	down(&log_start);
}
EXPORT_SYMBOL_GPL(sprd_log_sem_down);

static inline void xrp_comm_write32(
	volatile void __iomem *addr, u32 v)
{
	__raw_writel(v, addr);
}

static inline u32 xrp_comm_read32(
	volatile void __iomem *addr)
{
	return __raw_readl(addr);
}

static inline void __iomem *xrp_comm_put_tlv(
	void __iomem **addr,
	uint32_t type,
	uint32_t length)
{
	struct xrp_dsp_tlv __iomem *tlv = *addr;

	xrp_comm_write32(&tlv->type, type);
	xrp_comm_write32(&tlv->length, length);
	*addr = tlv->value + ((length + 3) / 4);

	return tlv->value;
}

static inline void __iomem *xrp_comm_get_tlv(
	void __iomem **addr,
	uint32_t *type,
	uint32_t *length)
{
	struct xrp_dsp_tlv __iomem *tlv = *addr;

	*type = xrp_comm_read32(&tlv->type);
	*length = xrp_comm_read32(&tlv->length);
	*addr = tlv->value + ((*length + 3) / 4);

	return tlv->value;
}

//auto.efuse,1:T610,2:T700,0:T618,:-1
static uint32_t xvp_get_type(void)
{
	char chip_type[HWFEATURE_STR_SIZE_LIMIT];
	sprd_kproperty_get("auto/efuse" ,chip_type, "-1");
	if(!strcmp(chip_type , "T618")) {
		pr_debug("func:%s , this is T618\n" , __func__);
		return 0;
	} else {
		pr_debug("func:%s , this is T610\n" , __func__);
		return 1;
	}
}

static inline void xrp_comm_write(
	volatile void __iomem *addr,
	const void *p,
	size_t sz)
{
	size_t sz32 = sz & ~3;
	u32 v = 0;

	while (sz32) {
		memcpy(&v, p, sizeof(v));
		__raw_writel(v, addr);
		p += 4;
		addr += 4;
		sz32 -= 4;
	}

	sz &= 3;
	if (sz) {
		v = 0;
		memcpy(&v, p, sz);
		__raw_writel(v, addr);
	}
}

static inline void xrp_comm_read(
	volatile void __iomem *addr,
	void *p,
	size_t sz)
{
	size_t sz32 = sz & ~3;
	u32 v = 0;

	while (sz32) {
		v = __raw_readl(addr);
		memcpy(p, &v, sizeof(v));
		p += 4;
		addr += 4;
		sz32 -= 4;
	}

	sz &= 3;
	if (sz) {
		v = __raw_readl(addr);
		memcpy(p, &v, sz);
	}
}

static inline void xrp_send_device_irq(struct xvp *xvp)
{
	if (likely(xvp->hw_ops->send_irq))
		xvp->hw_ops->send_irq(xvp->hw_arg);
}

static inline bool xrp_panic_check(struct xvp *xvp)
{
	if (unlikely(xvp->hw_ops->panic_check))
		return xvp->hw_ops->panic_check(xvp->hw_arg);
	else
		return false;
}

static void xrp_add_known_file(struct file *filp)
{
	struct xrp_known_file *p =
		kmalloc(sizeof(*p), GFP_KERNEL);
	struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;

	if (unlikely(!p))
		return;

	p->filp = filp;
	mutex_lock(&xvp->xrp_known_files_lock);
	hash_add(xvp->xrp_known_files, &p->node, (unsigned long)filp);
	pr_debug("func:%s enter\n" , __func__);
	mutex_unlock(&xvp->xrp_known_files_lock);
}

static void xrp_remove_known_file(struct file *filp)
{
	struct xrp_known_file *p;
	struct xrp_known_file *pf = NULL;
	struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;

	mutex_lock(&xvp->xrp_known_files_lock);
	pr_debug("func:%s enter\n" , __func__);
	hash_for_each_possible(
			xvp->xrp_known_files, p, node,
			(unsigned long)filp) {
		if (p->filp == filp) {
			hash_del(&p->node);
			pf = p;
			break;
		}
	}
	mutex_unlock(&xvp->xrp_known_files_lock);
	if (pf)
		kfree(pf);
}

static void xrp_sync_v2(struct xvp *xvp,
	void *hw_sync_data, size_t sz)
{
	unsigned i;
	struct xrp_dsp_sync_v2 __iomem *queue_sync;
	struct xrp_dsp_sync_v2 __iomem *shared_sync = xvp->comm;
	void __iomem *addr = shared_sync->hw_sync_data;

	xrp_comm_write(xrp_comm_put_tlv(&addr,
		XRP_DSP_SYNC_TYPE_HW_SPEC_DATA, sz),
		hw_sync_data, sz);
	if (xvp->n_queues > 1) {
		xrp_comm_write(xrp_comm_put_tlv(&addr,
			XRP_DSP_SYNC_TYPE_HW_QUEUES,
			xvp->n_queues * sizeof(u32)),
			xvp->queue_priority,
			xvp->n_queues * sizeof(u32));
		for (i = 1; i < xvp->n_queues; ++i) {
			queue_sync = xvp->queue[i].comm;
			xrp_comm_write32(&queue_sync->sync,
				XRP_DSP_SYNC_IDLE);
		}
	}
	xrp_comm_put_tlv(&addr, XRP_DSP_SYNC_TYPE_LAST, 0);
}

static int xrp_sync_complete_v2(struct xvp *xvp, size_t sz)
{
	struct xrp_dsp_sync_v2 __iomem *shared_sync = xvp->comm;
	void __iomem *addr = shared_sync->hw_sync_data;
	u32 type, len;

	xrp_comm_get_tlv(&addr, &type, &len);
	if (len != sz) {
		pr_err("HW spec data size modified by the DSP\n");
		return -EINVAL;
	}
	if (!(type & XRP_DSP_SYNC_TYPE_ACCEPT))
		pr_err("HW spec data not recognized by the DSP\n");

	if (xvp->n_queues > 1) {
		void __iomem *p = xrp_comm_get_tlv(&addr, &type, &len);

		if (len != xvp->n_queues * sizeof(u32)) {
			pr_err("Queue priority size modified by the DSP\n");
			return -EINVAL;
		}
		if (type & XRP_DSP_SYNC_TYPE_ACCEPT) {
			xrp_comm_read(p, xvp->queue_priority,
				xvp->n_queues * sizeof(u32));
		}else {
			pr_err("Queue priority data not recognized by the DSP\n");
			xvp->n_queues = 1;
		}
	}
	return 0;
}

static int xrp_synchronize(struct xvp *xvp)
{
	size_t sz = 0;
	void *hw_sync_data;
	unsigned long deadline = jiffies + firmware_boot_timeout * HZ;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	int ret;
	u32 v, v1;

	/*
	 * TODO
	 * BAD METHOD
	 * Just Using sz temp for transfer share memory address
	 */

	if (xvp->vdsp_mem_desc->cb_func[CB_MSG])
		sz = (size_t)xvp->vdsp_mem_desc->cb_func[CB_MSG](
		xvp->vdsp_mem_desc->cb_args[CB_MSG]);
	else
		pr_err("get smsg share memory address failed\n");

	pr_debug("get smsg share memory address: 0x%lx\n", (unsigned long)sz);
	hw_sync_data = xvp->hw_ops->get_hw_sync_data(xvp->hw_arg, &sz,
		xvp->ion_vdsp_log.iova[0]);
	if (unlikely(!hw_sync_data)) {
		ret = -ENOMEM;
		goto err;
	}
	ret = -ENODEV;
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_START);
	pr_info("start sync:%d\n", XRP_DSP_SYNC_START);

	mb();
	do {
		v = xrp_comm_read32(&shared_sync->sync);
		if (v != XRP_DSP_SYNC_START)
			break;
		if (xrp_panic_check(xvp))
			goto err;
		schedule();
	} while (time_before(jiffies, deadline));
	pr_debug("sync:%d\n", v);

	switch (v) {
	case XRP_DSP_SYNC_DSP_READY_V1:
		if (xvp->n_queues > 1) {
			pr_err("[ERROR]Queue priority data not recognized\n");
			xvp->n_queues = 1;
		}
		xrp_comm_write(&shared_sync->hw_sync_data, hw_sync_data, sz);
		break;
	case XRP_DSP_SYNC_DSP_READY_V2:
		xrp_sync_v2(xvp, hw_sync_data, sz);
		break;
	case XRP_DSP_SYNC_START:
		pr_err("[ERROR]DSP is not ready for synchronization\n");
		goto err;
	default:
		pr_err("[ERROR]DSP response to XRP_DSP_SYNC_START is not recognized\n");
		goto err;
	}

	mb();
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_HOST_TO_DSP);

	do {
		mb();
		v1 = xrp_comm_read32(&shared_sync->sync);
		if (v1 == XRP_DSP_SYNC_DSP_TO_HOST)
			break;
		if (xrp_panic_check(xvp))
			goto err;
		schedule();
	} while (time_before(jiffies, deadline));
	if (v1 != XRP_DSP_SYNC_DSP_TO_HOST) {
		pr_err("[ERROR]DSP haven't confirmed initialization data reception\n");
		goto err;
	}

	if (v == XRP_DSP_SYNC_DSP_READY_V2) {
		ret = xrp_sync_complete_v2(xvp, sz);
		if (ret < 0)
			goto err;
	}
	xrp_send_device_irq(xvp);
	pr_debug("completev2 end, send devie irq-32k timer[%lld]\n", sprd_sysfrt_read());

	if (xvp->host_irq_mode) {
		int res = wait_for_completion_timeout(&xvp->queue[0].completion,
			firmware_boot_timeout * HZ);

		ret = -ENODEV;
		if (xrp_panic_check(xvp))
			goto err;
		if (res == 0) {
			pr_err("wait dsp send irq to host timeout\n");
			goto err;
		}
	}
	ret = 0;
err:
	if (hw_sync_data)
		kfree(hw_sync_data);
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
	pr_info("sync end ret:%d\n", ret);

	return ret;
}

static bool xrp_cmd_complete(struct xrp_comm *xvp)
{
	struct xrp_dsp_cmd __iomem *cmd = xvp->comm;
	u32 flags = xrp_comm_read32(&cmd->flags);

	rmb();
	return (flags & (XRP_DSP_CMD_FLAG_REQUEST_VALID |
		XRP_DSP_CMD_FLAG_RESPONSE_VALID)) ==
		(XRP_DSP_CMD_FLAG_REQUEST_VALID |
		XRP_DSP_CMD_FLAG_RESPONSE_VALID);
}

irqreturn_t xrp_irq_handler(int irq, struct xvp *xvp)
{
	unsigned i, n = 0;
	if (unlikely(!xvp->comm))
		return IRQ_NONE;

	for (i = 0; i < xvp->n_queues; ++i) {
		if (xrp_cmd_complete(xvp->queue + i)) {
			complete(&xvp->queue[i].completion);
			++n;
		}
	}
	return n ? IRQ_HANDLED : IRQ_NONE;
}
EXPORT_SYMBOL(xrp_irq_handler);
static irqreturn_t xrp_hw_irq_handler_ex(int irq, void * data)
{
	struct xvp *xvp = data;

	return xrp_irq_handler(irq, xvp);
}

static inline void xvp_file_lock(struct xvp_file *xvp_file)
{
	spin_lock(&xvp_file->busy_list_lock);
}

static inline void xvp_file_unlock(struct xvp_file *xvp_file)
{
	spin_unlock(&xvp_file->busy_list_lock);
}

static long xvp_complete_cmd_irq(
	struct xvp *xvp,
	struct xrp_comm *comm,
	bool (*cmd_complete)(struct xrp_comm *p) , uint32_t krqflag)
{
	long timeout = firmware_command_timeout * HZ;

	if (cmd_complete(comm))
		return 0;
	if (xrp_panic_check(xvp)) {
		pr_err("[error]xrp panic\n");
		return -EBUSY;
	}
	do {
		timeout =
				wait_for_completion_timeout(
                                &comm->completion,
                                timeout);

		pr_info("xvp_complete_cmd_irq wait_for_completion_timeout %ld\n", timeout);
		if (cmd_complete(comm))
			return 0;
		if (xrp_panic_check(xvp)) {
			pr_err("[error]xrp panic\n");
			return -EBUSY;
		}
	} while (timeout > 0);

	if (timeout == 0) {
		pr_err("[error]vdsp timeout\n");
		return -EBUSY;
	}

	pr_debug("xvp_complete_cmd_irq %ld\n", timeout);
	return timeout;
}

static long xvp_complete_cmd_poll(struct xvp *xvp, struct xrp_comm *comm,
	bool(*cmd_complete)(struct xrp_comm *p))
{
	unsigned long deadline = jiffies + firmware_command_timeout * HZ;

	do {
		if (cmd_complete(comm))
			return 0;
		if (xrp_panic_check(xvp))
			return -EBUSY;
		schedule();
	} while (time_before(jiffies, deadline));

	return -EBUSY;
}

static inline int xvp_enable_dsp(struct xvp *xvp)
{
	if(xvp->secmode){
		if (loopback < LOOPBACK_NOMMIO)
			return sprd_faceid_enable_vdsp(xvp);
		else
			return 0;
	}
	else{
		if (loopback < LOOPBACK_NOMMIO &&
			xvp->hw_ops->enable)
			return xvp->hw_ops->enable(xvp->hw_arg);
		else
			return 0;
	}
}

static inline void xvp_disable_dsp(struct xvp *xvp)
{
	if(xvp->secmode){
		if (loopback < LOOPBACK_NOMMIO)
			sprd_faceid_disable_vdsp(xvp);
	}
	else{
		if (loopback < LOOPBACK_NOMMIO &&
			xvp->hw_ops->disable)
			xvp->hw_ops->disable(xvp->hw_arg);
	}
}

static inline void xrp_reset_dsp(struct xvp *xvp)
{
	if(xvp->secmode){
		if (loopback < LOOPBACK_NOMMIO)
			sprd_faceid_reset_vdsp(xvp);
	}
	else{
		if (loopback < LOOPBACK_NOMMIO &&
			xvp->hw_ops->reset)
			xvp->hw_ops->reset(xvp->hw_arg);
	}
}

static inline void xrp_halt_dsp(struct xvp *xvp)
{
	if(xvp->secmode){
		if (loopback < LOOPBACK_NOMMIO)
			sprd_faceid_halt_vdsp(xvp);
	}
	else{
		if (loopback < LOOPBACK_NOMMIO &&
			xvp->hw_ops->halt)
			xvp->hw_ops->halt(xvp->hw_arg);
	}
}

static inline void xrp_release_dsp(struct xvp *xvp)
{
	if(xvp->secmode){
		if (loopback < LOOPBACK_NOMMIO)
			sprd_faceid_release_vdsp(xvp);
	}
	else{
		if (loopback < LOOPBACK_NOMMIO &&
			xvp->hw_ops->release)
			xvp->hw_ops->release(xvp->hw_arg);
	}

}

static inline void xvp_dsp_enable_dvfs(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->enable_dvfs)
		xvp->hw_ops->enable_dvfs(xvp->hw_arg);
}
static inline void xvp_dsp_disable_dvfs(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->disable_dvfs)
		xvp->hw_ops->disable_dvfs(xvp->hw_arg);
}
static inline void xvp_dsp_setdvfs(struct xvp *xvp, uint32_t index)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->setdvfs)
		xvp->hw_ops->setdvfs(xvp->hw_arg, index);
}

static inline void xvp_set_qos(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->set_qos)
		xvp->hw_ops->set_qos(xvp->hw_arg);
}

static int xvp_file_release_list(struct file *filp)//xvp *xvp , struct xvp_file* xvp_file)
{
	struct loadlib_info *libinfo,*libinfo1 , *temp , *temp1;
	struct xvp_file * xvpfile_temp , *xvp_file;
	unsigned long bkt;
	uint32_t find = 0;
	long ret;
	int32_t result = 0;
	struct xrp_known_file *p;
	struct xvp *xvp;
	struct xrp_unload_cmdinfo unloadinfo;
	char libname[XRP_NAMESPACE_ID_SIZE];

	xvp_file = (struct xvp_file*)filp->private_data;
	xvp = xvp_file->xvp;
	libinfo = libinfo1 = temp = temp1 = NULL;
	pr_debug("func:%s step 0\n" , __func__);
	mutex_lock(&(xvp->load_lib.libload_mutex));
	/*release lib load list*/
	list_for_each_entry_safe(libinfo , temp , &xvp_file->load_lib_list , node_libinfo) {
		find = 0;
		/*check whether other xvp_file in system has loaded this lib*/
		mutex_lock(&xvp->xrp_known_files_lock);
		hash_for_each(xvp->xrp_known_files , bkt , p , node) {
			if (((struct file*)(p->filp))->private_data != xvp_file) {
					xvpfile_temp = (struct xvp_file *)(((struct file*)(p->filp))->private_data);
					find = 0;
					list_for_each_entry_safe(libinfo1 , temp1 , &xvpfile_temp->load_lib_list , node_libinfo) {
						if(0 == strcmp(libinfo1->libname , libinfo->libname)) {
							/*find the same lib loaded by other file*/
							pr_debug("func:%s find :%s %s\n" , __func__ , libinfo1->libname , libinfo->libname);
							find = 1;
							break;
						}
					}
					if(1 == find) {
						pr_debug("func:%s find 2 :%s %s\n" , __func__ , libinfo1->libname , libinfo->libname);
						break;
					}
			}
		}
		mutex_unlock(&xvp->xrp_known_files_lock);
		if(1 != find) {
			/*if not find in other files need unload*/
			/*do unload process--------------- later*/
			pr_debug("func:%s step 1\n" , __func__);
			ret = xrp_create_unload_cmd(filp , libinfo , &unloadinfo);
			if(ret != LIB_RESULT_OK) {
				pr_err("func:%s xrp_create_unload_cmd failed , maybe library leak\n");
				result = -EINVAL;
				continue;
			}
			pr_debug("func:%s step 2\n" , __func__);
			mutex_unlock(&(xvp->load_lib.libload_mutex));
			libinfo->load_count = 1; /*force set 1 here*/
			snprintf(libname , XRP_NAMESPACE_ID_SIZE , "%s" , libinfo->libname);
			ret = xrp_ioctl_submit_sync(filp, NULL , unloadinfo.rq);
			pr_debug("func:%s after xrp_ioctl_submit_sync ret:%d step 3\n" , __func__ , ret);
			mutex_lock(&(xvp->load_lib.libload_mutex));
			if(ret == -ENODEV) {
				/*if went off , release here*/
				xrp_library_decrelease(filp , libname);
			}
			xrp_free_unload_cmd(filp , &unloadinfo);
			pr_debug("func:%s step 4\n" , __func__);
		} else {
			list_del(&libinfo->node_libinfo);
			vfree(libinfo);
		}
	}
	mutex_unlock(&(xvp->load_lib.libload_mutex));
	pr_debug("func:%s step 5\n" , __func__);
	/*release power hint later*/
	vdsp_release_powerhint(filp);
	return result;
}
static int xrp_boot_firmware(struct xvp *xvp)
{
	int ret;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	s64 tv0, tv1, tv2;

	tv0 = ktime_to_us(ktime_get());
	xrp_halt_dsp(xvp);
	xrp_reset_dsp(xvp);

	pr_debug("firmware name:%s, loopback:%d\n", xvp->firmware_name, loopback);
	if (likely(xvp->firmware_name)) {
		if (loopback < LOOPBACK_NOFIRMWARE) {
			ret = xrp_request_firmware(xvp);
			if (ret < 0) {
				/*may be halt vdsp here, and set went off true?*/
				xrp_halt_dsp(xvp);
				xvp->off = true;
				pr_err("xrp_request_firmware failed\n");
				return ret;
			}
		}

		if (loopback < LOOPBACK_NOIO) {
			xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
			mb();
		}
	}

	xrp_release_dsp(xvp);
	tv1 = ktime_to_us(ktime_get());

	if (loopback < LOOPBACK_NOIO) {
		ret = xrp_synchronize(xvp);
		if (unlikely(ret < 0)) {
			xrp_halt_dsp(xvp);
			pr_err("couldn't synchronize with the DSP core\n");
			xvp->off = true;
			return ret;
		}
	}
	tv2 = ktime_to_us(ktime_get());
	/*request firmware - sync*/
	pr_info("[TIME]request firmware(%s):%lld (us), sync:%lld (us)\n",
		xvp->firmware_name, tv1 - tv0, tv2 - tv1);
	return 0;
}

static int xrp_boot_faceid_firmware(struct xvp *xvp)
{
	int ret;
	s64 tv0, tv1, tv2;

	tv0 = ktime_to_us(ktime_get());
	ret = sprd_faceid_secboot_entry(xvp);
	if (ret < 0)
		return ret;

	xrp_halt_dsp(xvp);
	xrp_reset_dsp(xvp);

	pr_debug("loopback:%d\n", loopback);

	if (loopback < LOOPBACK_NOFIRMWARE) {
		ret = sprd_faceid_sec_sign(xvp);
		if (ret < 0) {
			return ret;
		}
		ret = sprd_faceid_load_firmware(xvp);
		if (ret < 0) {
			return ret;
		}
	}

	xrp_release_dsp(xvp);
	tv1 = ktime_to_us(ktime_get());

	if (loopback < LOOPBACK_NOIO) {
		ret = sprd_faceid_sync_vdsp(xvp);
		if (ret < 0) {
			xrp_halt_dsp(xvp);
			pr_err("couldn't synchronize with the DSP core\n");
			xvp->off = true;
			return ret;
		}
	}
	tv2 = ktime_to_us(ktime_get());
	/*request firmware - sync*/
	pr_info("[TIME]request firmware:%lld(us), sync:%lld(us)\n",
		tv1 - tv0, tv2 - tv1);
	return 0;
}

static int sprd_vdsp_boot_firmware(struct xvp *xvp)
{
        if (xvp->secmode)
                return xrp_boot_faceid_firmware(xvp);
        else
                return xrp_boot_firmware(xvp);
}

static int sprd_unmap_request(
	struct file *filp,
	struct xrp_request *rq , uint32_t krqflag)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	size_t n_buffers = rq->n_buffers;
	size_t i;
	long ret = 0;

	pr_debug("[UNMAP][IN]\n");
	if(krqflag == 1) {
                pr_debug("[UNMAP] krqflag is 1 [OUT]\n");
                return 0;
        }
	if (rq->ioctl_queue.in_data_size >
		XRP_DSP_CMD_INLINE_DATA_SIZE) {
		ret |= xvp->vdsp_mem_desc->ops->mem_iommu_unmap(
			xvp->vdsp_mem_desc,
			&rq->ion_in_buf,
			IOMMU_ALL);
	}
	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		ret |= xvp->vdsp_mem_desc->ops->mem_iommu_unmap(
			xvp->vdsp_mem_desc,
			&rq->ion_out_buf,
			IOMMU_ALL);
	}else {
		if (copy_to_user(
			(void __user *)(unsigned long)rq->ioctl_queue.out_data_addr,
			rq->out_data,
			rq->ioctl_queue.out_data_size)) {
			pr_err("out_data could not be copied\n");
			ret |= -EFAULT;
		}
	}

	if (n_buffers) {
		ret |= xvp->vdsp_mem_desc->ops->mem_kunmap(
			xvp->vdsp_mem_desc,
			rq->dsp_buf);
		ret |= xvp->vdsp_mem_desc->ops->mem_iommu_unmap(
			xvp->vdsp_mem_desc, rq->dsp_buf,
			IOMMU_ALL);
		ret |= xvp->vdsp_mem_desc->ops->mem_free(
			xvp->vdsp_mem_desc,
				rq->dsp_buf);
		kfree(rq->dsp_buf);
		for (i = 0; i < n_buffers; ++i) {
			ret |= xvp->vdsp_mem_desc->ops->mem_iommu_unmap(
				xvp->vdsp_mem_desc,
				&rq->ion_dsp_pool[i],
				IOMMU_ALL);
			if (ret < 0)
				pr_err("[ERROR]buffer %d could not be unshared\n", i);
		}
		kfree(rq->ion_dsp_pool);

		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
			kfree(rq->dsp_buffer);

		rq->n_buffers = 0;
	}
	if(ret != 0)
		pr_err("[ERROR] sprd_unmap_request failed ret:%x\n" , ret);
	return ret;
}

static int sprd_map_request(
	struct file *filp,
	struct xrp_request *rq , uint32_t krqflag)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_ioctl_buffer __user *buffer;
	int n_buffers = rq->ioctl_queue.buffer_size /
		sizeof(struct xrp_ioctl_buffer);
	struct ion_buf *p_in_buf = &rq->ion_in_buf;
	struct ion_buf *p_out_buf = &rq->ion_out_buf;
	struct ion_buf *p_dsp_buf;
	int i , nbufferflag;
	long ret = 0;
	nbufferflag = 0;
	pr_debug("[MAP][IN]\n");
	if(1 == krqflag) {
		pr_debug("[MAP] krqflag is 1 [OUT]");
		return 0;
	}
	memset((void *)&rq->ion_in_buf, 0x00, sizeof(struct ion_buf));
	memset((void *)&rq->ion_out_buf, 0x00, sizeof(struct ion_buf));

	if ((rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID) &&
		copy_from_user(rq->nsid,
		(void __user *)(unsigned long)rq->ioctl_queue.nsid_addr,
		sizeof(rq->nsid))) {
		pr_err("[ERROR]nsid could not be copied\n");
		return -EINVAL;
	}

	rq->n_buffers = n_buffers;
	if (n_buffers) {
		nbufferflag = 1;
		rq->dsp_buf = kmalloc(sizeof(*rq->dsp_buf), GFP_KERNEL);
		if (!rq->dsp_buf) {
			pr_err("[ERROR]fail to kmalloc buffer.\n");
			return -ENOMEM;
		}
		ret = xvp->vdsp_mem_desc->ops->mem_alloc(
			xvp->vdsp_mem_desc,
			rq->dsp_buf,
			ION_HEAP_ID_MASK_SYSTEM,
			n_buffers * sizeof(*rq->dsp_buffer));
		if (ret != 0) {
			pr_err("[ERROR]fail to mem alloc buffer.\n");
			kfree(rq->dsp_buf);
			return -ENOMEM;
		}
		ret = xvp->vdsp_mem_desc->ops->mem_kmap(
			xvp->vdsp_mem_desc,
			rq->dsp_buf);
		if(ret != 0) {
			pr_err("[ERROR]fail to mem kmap dsp_buf\n");
			xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc , rq->dsp_buf);
			kfree(rq->dsp_buf);
			return -EINVAL;
		}
		rq->dsp_buf->dev = xvp->dev;
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
			xvp->vdsp_mem_desc,
			rq->dsp_buf,
			IOMMU_ALL);
		if(ret != 0) {
			pr_err("[ERROR]fail to mem_iommu_map dsp_buf\n");
			xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc , rq->dsp_buf);
			xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc , rq->dsp_buf);
			kfree(rq->dsp_buf);
			return -EINVAL;
		}
		rq->dsp_buffer_phys = (phys_addr_t)rq->dsp_buf->iova[0];

		rq->ion_dsp_pool =
			kmalloc(n_buffers * sizeof(*rq->ion_dsp_pool),
			GFP_KERNEL);
		if (!rq->ion_dsp_pool) {
			pr_err("[ERROR]fail to kmalloc ion_dsp_pool\n");
			xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc , rq->dsp_buf , IOMMU_ALL);
			xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc , rq->dsp_buf);
			xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc , rq->dsp_buf);
			kfree(rq->dsp_buf);
			return -ENOMEM;
		}
		memset((void *)rq->ion_dsp_pool,
			0x00, n_buffers * sizeof(*rq->ion_dsp_pool));
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			rq->dsp_buffer =
				kmalloc(n_buffers * sizeof(*rq->dsp_buffer),
				GFP_KERNEL);
			if (!rq->dsp_buffer) {
				xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc , rq->dsp_buf , IOMMU_ALL);
				xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc , rq->dsp_buf);
				xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc , rq->dsp_buf);
				kfree(rq->ion_dsp_pool);
				kfree(rq->dsp_buf);
				return -ENOMEM;
			}
		}else {
			rq->dsp_buffer = rq->buffer_data;
		}
	}

	//in_data addr
	if ((rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)) {
		if (rq->ioctl_queue.in_data_fd < 0) {
			pr_err("[ERROR]in data fd is:%d\n", rq->ioctl_queue.in_data_fd);
			ret = -EFAULT;
			goto free_indata_err;
		}
		p_in_buf->mfd[0] = rq->ioctl_queue.in_data_fd;
		p_in_buf->dev = xvp->dev;
		ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(
			xvp->vdsp_mem_desc,
			p_in_buf);
		if (ret) {
			pr_err("[ERROR]fail to get in_ion_buf\n");
			ret = -EFAULT;
			goto free_indata_err;
		}
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
			xvp->vdsp_mem_desc,
			p_in_buf,
			IOMMU_ALL);
		if (ret) {
			pr_err("[ERROR]fail to get in_data addr!!!!\n");
			ret = -EFAULT;
			goto free_indata_err;
		}
		rq->in_data_phys = (uint32_t)p_in_buf->iova[0];

		pr_debug("in_data_fd=%d, in_data_phys=0x%x size=%d\n",
			(int)rq->ioctl_queue.in_data_fd, (uint32_t)rq->in_data_phys,
			(uint32_t)p_in_buf->size[0]);
	}else {
		if (copy_from_user(
			rq->in_data,
			(void __user *)(unsigned long)rq->ioctl_queue.in_data_addr,
			rq->ioctl_queue.in_data_size)) {
			pr_err("[ERROR]in_data could not be copied\n");
			ret = -EFAULT;
			goto free_indata_err;
		}
	}

	//out_data addr
	if ((rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		&& (rq->ioctl_queue.out_data_fd >= 0)) {
		p_out_buf->mfd[0] = rq->ioctl_queue.out_data_fd;
		p_out_buf->dev = xvp->dev;
		ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(
			xvp->vdsp_mem_desc,
			p_out_buf);
		if (ret) {
			pr_err("fail to get ion_out_buf\n");
			ret = -EFAULT;
			goto share_err;
		}
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
			xvp->vdsp_mem_desc,
			p_out_buf,
			IOMMU_ALL);
		if (ret) {
			pr_err("[ERROR]fail to get out_data addr!!!!\n");
			ret = -EFAULT;
			goto share_err;
		}
		rq->out_data_phys = (uint32_t)p_out_buf->iova[0];

		pr_debug("out_data_fd=%d, out_data_phys=0x%x, size=%d\n",
			(int)rq->ioctl_queue.out_data_fd, (uint32_t)rq->out_data_phys,
			(uint32_t)p_out_buf->size[0]);
	}

	//bufer addr
	pr_debug("n_buffers [%d]\n", n_buffers);
	if (n_buffers) {
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			buffer = (void __user *)(unsigned long)rq->ioctl_queue.buffer_addr;

			for (i = 0; i < n_buffers; ++i) {
				struct xrp_ioctl_buffer ioctl_buffer;

				if (copy_from_user(&ioctl_buffer, buffer + i,
					sizeof(ioctl_buffer))) {
					ret = -EFAULT;
					pr_err("[ERROR]copy from user failed\n");
					goto share_err;
				}
				if (ioctl_buffer.fd >= 0) {
					p_dsp_buf = &rq->ion_dsp_pool[i];
					p_dsp_buf->mfd[0] = ioctl_buffer.fd;
					p_dsp_buf->dev = xvp->dev;
					ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(
						xvp->vdsp_mem_desc,
						p_dsp_buf);
					if (ret) {
						pr_err("[ERROR]fail to get ion_des_buf\n");
						ret = -EFAULT;
						goto share_err;
					}
					ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
						xvp->vdsp_mem_desc,
						p_dsp_buf,
						IOMMU_ALL);
					if (ret) {
						pr_err("[ERROR]fail to get dsp addr[%d]!!!!\n", i);
						ret = -EFAULT;
						goto share_err;
					}
					rq->dsp_buffer[i] = (struct xrp_dsp_buffer){
						.flags = ioctl_buffer.flags,
						.size = ioctl_buffer.size,
						.addr = (uint32_t)p_dsp_buf->iova[0],
						.fd = ioctl_buffer.fd,
					};
					pr_debug("dsp_buffer[%d] addr:0x%x, size:%d, fd:%d, flags:%d\n",
						i, rq->dsp_buffer[i].addr, rq->dsp_buffer[i].size,
						rq->dsp_buffer[i].fd, rq->dsp_buffer[i].flags);
				}
			}
		}else {
			struct xrp_ioctl_buffer ioctl_buffer;

			buffer = (void __user *)(unsigned long)rq->ioctl_queue.buffer_addr;
			if (copy_from_user(&ioctl_buffer, buffer,
				sizeof(ioctl_buffer))) {
				ret = -EFAULT;
				pr_err("[ERROR]copy from user failed\n");
				goto share_err;
			}

			if (ioctl_buffer.fd >= 0) {
				p_dsp_buf = rq->ion_dsp_pool;
				p_dsp_buf->mfd[0] = ioctl_buffer.fd;
				p_dsp_buf->dev = xvp->dev;
				ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(
					xvp->vdsp_mem_desc,
					p_dsp_buf);
				if (ret) {
					pr_err("[ERROR]fail to get ion_des_buf\n");
					ret = -EFAULT;
					goto share_err;
				}
				ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
					xvp->vdsp_mem_desc,
					p_dsp_buf,
					IOMMU_ALL);
				if (ret) {
					pr_err("[ERROR]fail to get dsp addr!!!!\n");
					ret = -EFAULT;
					goto share_err;
				}

				rq->dsp_buffer->addr = (uint32_t)p_dsp_buf->iova[0];
				rq->dsp_buffer->fd = ioctl_buffer.fd;
				rq->dsp_buffer->size = ioctl_buffer.size;
				rq->dsp_buffer->flags = ioctl_buffer.flags;
				pr_debug("Only one dsp_buffer addr:0x%x, size:%d, fd:%d, flags:%d\n",
					rq->dsp_buffer->addr, rq->dsp_buffer->size,
					rq->dsp_buffer->fd, rq->dsp_buffer->flags);
			}
		}
		memcpy((void *)rq->dsp_buf->addr_k[0],
			(void *)rq->dsp_buffer, n_buffers * sizeof(*rq->dsp_buffer));
	}
share_err:
	if (ret < 0)
		sprd_unmap_request(filp, rq , krqflag);
	pr_debug("[MAP][OUT]\n");
	return ret;
free_indata_err:
	if(nbufferflag == 1) {
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc , rq->dsp_buf , IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc , rq->dsp_buf);
		kfree(rq->ion_dsp_pool);
		if(n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
			kfree(rq->dsp_buffer);
		kfree(rq->dsp_buf);
        }
	pr_debug("[MAP][OUT] free_indata_err\n");
	return ret;
}

static void sprd_fill_hw_request(struct xrp_dsp_cmd __iomem *cmd,
	struct xrp_request *rq)
{
	xrp_comm_write32(&cmd->in_data_size, rq->ioctl_queue.in_data_size);
	xrp_comm_write32(&cmd->out_data_size, rq->ioctl_queue.out_data_size);
	xrp_comm_write32(&cmd->buffer_size, rq->n_buffers * sizeof(struct xrp_dsp_buffer));

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->in_data_addr, rq->in_data_phys);
	else
		xrp_comm_write(&cmd->in_data, rq->in_data, rq->ioctl_queue.in_data_size);

	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->out_data_addr, rq->out_data_phys);

	if (rq->n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_write32(&cmd->buffer_addr, rq->dsp_buffer_phys);
	else
		xrp_comm_write(&cmd->buffer_data, rq->dsp_buffer,
		rq->n_buffers * sizeof(struct xrp_dsp_buffer));

	if (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID)
		xrp_comm_write(&cmd->nsid, rq->nsid, sizeof(rq->nsid));

	wmb();
	/* update flags */
	xrp_comm_write32(&cmd->flags,
		(rq->ioctl_queue.flags & ~XRP_DSP_CMD_FLAG_RESPONSE_VALID) |
		XRP_DSP_CMD_FLAG_REQUEST_VALID);
}

static long sprd_complete_hw_request(struct xrp_dsp_cmd __iomem *cmd,
	struct xrp_request *rq)
{
	u32 flags = xrp_comm_read32(&cmd->flags);

	if (rq->ioctl_queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_read(&cmd->out_data, rq->out_data,
		rq->ioctl_queue.out_data_size);
	if (rq->n_buffers <= XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_read(&cmd->buffer_data, rq->dsp_buffer,
		rq->n_buffers * sizeof(struct xrp_dsp_buffer));
	xrp_comm_write32(&cmd->flags, 0);

	return (flags & XRP_DSP_CMD_FLAG_RESPONSE_DELIVERY_FAIL) ? -ENXIO : 0;
}

static long xrp_ioctl_submit_sync(struct file *filp,
				struct xrp_ioctl_queue __user *p , struct xrp_request *pk_rq)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_comm *queue = xvp->queue;
	struct xrp_request xrp_rq, *rq = &xrp_rq;
	long ret = 0;
	bool went_off = false;
	enum load_unload_flag load_flag;
	char libname[XRP_NAMESPACE_ID_SIZE];
	bool rebootflag = 0;
	int32_t lib_result = 0;
	uint32_t krqflag = 0;
	s64 tv0, tv1, tv2, tv3, tv4, tv5;

	tv2 = tv3 = tv0 = tv1 = tv4 = tv5 = 0;
	tv0 = ktime_to_us(ktime_get());
	if(p != NULL) {
		if (unlikely(copy_from_user(&rq->ioctl_queue, p, sizeof(*p))))
			return -EFAULT;
	} else if(pk_rq != NULL) {
		/*null for kernel*/
		krqflag = 1;
		memcpy(rq , pk_rq , sizeof(*pk_rq));
	}
	if (unlikely(rq->ioctl_queue.flags & ~XRP_QUEUE_VALID_FLAGS)) {
		pr_err("invalid flags 0x%08x\n", rq->ioctl_queue.flags);
		return -EINVAL;
	}

	if (xvp->n_queues > 1) {
		unsigned n = (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_PRIO) >>
			XRP_QUEUE_FLAG_PRIO_SHIFT;
		if (n >= xvp->n_queues)
			n = xvp->n_queues - 1;
		queue = xvp->queue_ordered[n];
		pr_debug("queue index:%d, priority: %d\n", n, queue->priority);
	}
	tv1 = ktime_to_us(ktime_get());
	ret = sprd_map_request(filp, rq , krqflag);
	tv1 = ktime_to_us(ktime_get()) - tv1;

	if (unlikely(ret < 0)){
		pr_err("[ERROR]map request fail\n");
		return ret;
	}

	if (likely(loopback < LOOPBACK_NOIO)) {
		int reboot_cycle;

	retry:
		mutex_lock(&queue->lock);
		reboot_cycle = atomic_read(&xvp->reboot_cycle);
		if (reboot_cycle != atomic_read(&xvp->reboot_cycle_complete)) {
			mutex_unlock(&queue->lock);
			goto retry;
		}

		if (unlikely(xvp->off)) {
			ret = -ENODEV;
		}else {
			/*check whether libload command and if it is, do load*/
			tv2 = ktime_to_us(ktime_get());
			load_flag = xrp_check_load_unload(xvp , rq , krqflag);
			pr_info("cmd nsid:%s,(cmd:0/load:1/unload:2)flag:%d , filp:%lx\n",rq->nsid, load_flag , (unsigned long)filp);
			mutex_lock(&(xvp->load_lib.libload_mutex));
			lib_result = xrp_pre_process_request(
				filp , rq , load_flag , libname , krqflag);
			tv2 = ktime_to_us(ktime_get()) - tv2;//lib load/unload
			if (lib_result != 0) {
				mutex_unlock(&queue->lock);
				mutex_unlock(&(xvp->load_lib.libload_mutex));
				ret = sprd_unmap_request(filp , rq , krqflag);
				if(lib_result == -EEXIST) {
					return 0;
				}else {
					pr_err("xrp pre process failed lib_result:%d\n" , lib_result);
					return lib_result;
				}

			}else if ((load_flag != XRP_UNLOAD_LIB_FLAG)
				&& (load_flag != XRP_LOAD_LIB_FLAG)) {
				mutex_unlock(&(xvp->load_lib.libload_mutex));
			}

			sprd_fill_hw_request(queue->comm, rq);
			tv3 = ktime_to_us(ktime_get());
			xrp_send_device_irq(xvp);
			pr_debug("send vdsp cmd-32k time[%lld]\n", sprd_sysfrt_read());

			if (xvp->host_irq_mode) {
				ret = xvp_complete_cmd_irq(xvp, queue,
						xrp_cmd_complete , krqflag);
			} else {
				ret = xvp_complete_cmd_poll(xvp, queue,
						xrp_cmd_complete);
			}
			tv3 = ktime_to_us(ktime_get()) - tv3;//send irq->reci

			pr_debug("func:%s xvp_complete_cmd_irq ret:%d\n" , __func__ , ret);
			/* copy back inline data */
			if (likely(ret == 0)) {
				ret = sprd_complete_hw_request(queue->comm, rq);
			}else if (ret == -EBUSY && firmware_reboot &&
				atomic_inc_return(&xvp->reboot_cycle) ==
				reboot_cycle + 1) {
				int rc;
				unsigned i;
				if ((load_flag == XRP_LOAD_LIB_FLAG)
					|| (load_flag == XRP_UNLOAD_LIB_FLAG)) {
					mutex_unlock(&(xvp->load_lib.libload_mutex));
				}
				//dump vdsp
				vdsp_log_coredump(xvp);

				pr_info("###enter reboot flow!###\n");
				for (i = 0; i < xvp->n_queues; ++i)
					if (xvp->queue + i != queue)
						mutex_lock(&xvp->queue[i].lock);
				rc = sprd_vdsp_boot_firmware(xvp);

				/*release library loaded here because vdsp is reseting ok
				so release all library resource here, but if boot failed
				the libraries loaded may be leaked because we don't know whether
				vdsp processing these resource or not*/
				if(rc == 0)
				{
					mutex_lock(&(xvp->load_lib.libload_mutex));
					xrp_library_release_all(xvp);
					mutex_unlock(&(xvp->load_lib.libload_mutex));
				}
				atomic_set(&xvp->reboot_cycle_complete,
					atomic_read(&xvp->reboot_cycle));
				for (i = 0; i < xvp->n_queues; ++i)
					if (xvp->queue + i != queue)
						mutex_unlock(&xvp->queue[i].lock);
				if (unlikely(rc < 0)) {
					ret = rc;
					went_off = xvp->off;
					pr_err("vdsp reboot failed may be encounter fatal error!!!\n");
				}
				pr_info("###reboot flow end!###\n");
				rebootflag = 1;
			}
			if (likely(0 == rebootflag)) {
				if ((load_flag != XRP_LOAD_LIB_FLAG)
					&& (load_flag != XRP_UNLOAD_LIB_FLAG))
					mutex_lock(&(xvp->load_lib.libload_mutex));

				lib_result = post_process_request(
					filp , rq , libname , load_flag , ret);
				if((load_flag == XRP_LOAD_LIB_FLAG)
					|| (load_flag == XRP_UNLOAD_LIB_FLAG)) {
					mutex_unlock(&(xvp->load_lib.libload_mutex));
				}else {
					if (unlikely(lib_result != 0)) {
						mutex_unlock(&queue->lock);
						mutex_unlock(&(xvp->load_lib.libload_mutex));
						sprd_unmap_request(filp , rq , krqflag);
						pr_err("[ERROR]lib result error\n");
						return -EFAULT;
					}else {
						mutex_unlock(&(xvp->load_lib.libload_mutex));
					}
				}
			}
		}
		mutex_unlock(&queue->lock);
	}

	tv4 = ktime_to_us(ktime_get());
	if(likely(ret == 0))
		ret = sprd_unmap_request(filp, rq , krqflag);
	else if(!went_off)
		sprd_unmap_request(filp, rq , krqflag);
	/*
	 * Otherwise (if the DSP went off) all mapped buffers are leaked here.
	 * There seems to be no way to recover them as we don't know what's
	 * going on with the DSP; the DSP may still be reading and writing
	 * this memory.
	 */
	tv5 = ktime_to_us(ktime_get());
	pr_info("[TIME](cmd->nsid:%s)total:%lld(us),map:%lld(us),load/unload:%lld(us),"
		"vdsp:%lld(us),unmap:%lld(us),ret:%d\n",
		rq->nsid, tv5 - tv0, tv1, tv2, tv3, tv5 - tv4, ret);

	return ret;
}
static long xrp_ioctl_faceid_cmd(struct file *filp,
	struct xrp_faceid_ctrl __user *arg)
{
	struct xrp_faceid_ctrl faceid;
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;

	if(xvp->soc_type != 0) {
		pr_err("this platform does not support faceid\n");
		return -EFAULT;
	}

	if (unlikely(copy_from_user(&faceid, arg, sizeof(struct xrp_faceid_ctrl)))){
		pr_err("[ERROR]copy from user failed\n");
		return -EFAULT;
	}
	pr_debug("faceid:in %d, out %d\n", faceid.in_fd, faceid.out_fd);
	sprd_faceid_run_vdsp(xvp, faceid.in_fd, faceid.out_fd);

	return 0;
}

static long xrp_ioctl_set_dvfs(struct file *filp,
	struct xrp_dvfs_ctrl __user *arg)
{
	struct xrp_dvfs_ctrl dvfs;
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;

	if (unlikely(copy_from_user(&dvfs, arg, sizeof(struct xrp_dvfs_ctrl)))){
		pr_err("[ERROR]copy from user failed\n");
		return -EFAULT;
	}
	if (0 == dvfs.en_ctl_flag) {
		xvp_dsp_setdvfs(xvp, dvfs.index);
	}else {
		if (dvfs.enable)
			xvp_dsp_enable_dvfs(xvp);
		else
			xvp_dsp_disable_dvfs(xvp);
	}
	return 0;
}

static long xrp_ioctl_set_powerhint(struct file *filp,
	struct xrp_powerhint_ctrl __user *arg)
{
	struct xrp_powerhint_ctrl powerhint;

	if (unlikely(copy_from_user(&powerhint, arg, sizeof(struct xrp_powerhint_ctrl)))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}
	set_powerhint_flag(filp, powerhint.level, powerhint.acquire_release);
	return 0;
}

static long xvp_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	long retval = -EINVAL;
	struct xvp_file *xvp_file = NULL;
	pr_debug("cmd:%x\n", cmd);
	mutex_lock(&xvp_global_lock);
	if(unlikely(filp->private_data == NULL)) {
		mutex_unlock(&xvp_global_lock);
		pr_warn("func:%s filp private is NULL\n" , __func__);
		return retval;
	}
	xvp_file = filp->private_data;
	mutex_lock(&xvp_file->lock);
	xvp_file->working = 1;
	mutex_unlock(&xvp_file->lock);
	mutex_unlock(&xvp_global_lock);
	switch (cmd){
	case XRP_IOCTL_ALLOC:
	case XRP_IOCTL_FREE:
		break;
	case XRP_IOCTL_QUEUE:
	case XRP_IOCTL_QUEUE_NS:
		preprocess_work_piece(((struct xvp_file *)(filp->private_data))->xvp);
		retval = xrp_ioctl_submit_sync(filp,
				(struct xrp_ioctl_queue __user *)arg , NULL);
		postprocess_work_piece(((struct xvp_file *)(filp->private_data))->xvp);
		break;
	case XRP_IOCTL_SET_DVFS:
		retval = xrp_ioctl_set_dvfs(filp,
			(struct xrp_dvfs_ctrl __user *)arg);
		break;
	case XRP_IOCTL_SET_POWERHINT:
		retval = xrp_ioctl_set_powerhint(filp,
			(struct xrp_powerhint_ctrl __user *)arg);
		break;
	case XRP_IOCTL_FACEID_CMD:
		retval = xrp_ioctl_faceid_cmd(filp,
			(struct xrp_faceid_ctrl __user *)arg);
		break;
	default:
		retval = -EINVAL;
		break;
	}
	mutex_lock(&xvp_file->lock);
	xvp_file->working = 0;
	mutex_unlock(&xvp_file->lock);
	return retval;
}

static int32_t sprd_alloc_commbuffer(struct xvp *xvp)
{
	int ret;

	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
		&xvp->ion_comm,
		ION_HEAP_ID_MASK_SYSTEM,
		PAGE_SIZE);
	if (unlikely(0 != ret)) {
		pr_err("[ERROR]alloc comm buffer failed\n");
		return -ENOMEM;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(
		xvp->vdsp_mem_desc, &xvp->ion_comm);
	if (unlikely(0 != ret)) {
		xvp->vdsp_mem_desc->ops->mem_free(
			xvp->vdsp_mem_desc, &xvp->ion_comm);
		return -EFAULT;
	}
	xvp->comm = (void*)xvp->ion_comm.addr_k[0];
	xvp->ion_comm.dev = xvp->dev;
	pr_debug("xvp alloc comm vaddr:%p\n", xvp->comm);

	return 0;
}

static int32_t sprd_free_commbuffer(struct xvp *xvp)
{
	pr_debug("xvp free comm vaddr:%p\n", xvp->comm);
	if (likely(xvp->comm)) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(
			xvp->vdsp_mem_desc, &xvp->ion_comm);
		xvp->vdsp_mem_desc->ops->mem_free(
			xvp->vdsp_mem_desc, &xvp->ion_comm);
		xvp->comm = NULL;
	}

	return 0;
}

static int32_t sprd_alloc_extrabuffer(struct xvp *xvp)
{
	int ret;

	ret = xvp->vdsp_mem_desc->ops->mem_alloc(
		xvp->vdsp_mem_desc,
		&xvp->ion_firmware,
		ION_HEAP_ID_MASK_SYSTEM,
		VDSP_FIRMWIRE_SIZE);

	if (unlikely(ret != 0)){
		pr_err("[ERROR]alloc firmware fail\n");
		return -ENOMEM;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(
		xvp->vdsp_mem_desc, &xvp->ion_firmware);
	if (unlikely(ret)){
		xvp->vdsp_mem_desc->ops->mem_kunmap(
			xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_free(
			xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->firmware_viraddr = NULL;
		pr_err("[ERROR]map fail\n");
		return -EFAULT;
	}
	xvp->firmware_viraddr = (void*)xvp->ion_firmware.addr_k[0];
	xvp->ion_firmware.dev = xvp->dev;

	pr_debug("alloc firmware virvaddr:0x%p, size:%d\n",
		xvp->firmware_viraddr, VDSP_FIRMWIRE_SIZE);

	return ret;
}

static int sprd_free_extrabuffer(struct xvp *xvp)
{
	pr_debug("free firmwareaddr:%p\n", xvp->firmware_viraddr);
	if (likely(xvp->firmware_viraddr)) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(
			xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_free(
			xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->firmware_viraddr = NULL;
	}

	return 0;
}

static int32_t sprd_iommu_map_commbuffer(struct xvp *xvp)
{
	int ret = -EFAULT;

	if (unlikely(xvp->comm == NULL)) {
		pr_err("[ERROR]null comm addr\n");
		return ret;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
		xvp->vdsp_mem_desc, &xvp->ion_comm, IOMMU_ALL);
	if (unlikely(ret)) {
		pr_err("[ERROR]map ion_comm failed\n");
		return ret;
	}
	xvp->dsp_comm_addr = xvp->ion_comm.iova[0];

	return ret;
}

static int32_t sprd_iommu_unmap_commbuffer(struct xvp *xvp)
{
	int ret;

	if (likely(xvp->comm)) {
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(
			xvp->vdsp_mem_desc,
			&xvp->ion_comm,
			IOMMU_ALL);
		if (unlikely(ret)) {
			pr_err("[ERROR]unmap ion_comm failed\n");
			return -EFAULT;
		}
	}else {
		pr_err("[ERROR]xvp->comm is NULL\n");
		return -EINVAL;
	}
	pr_debug("xvp->comm:%p\n", xvp->comm);

	return 0;
}

static int sprd_iommu_map_extrabuffer(struct xvp *xvp)
{
	int ret = -EFAULT;

	if (unlikely(xvp->firmware_viraddr == NULL)) {
		pr_err("[ERROR]NULL firmware viraddr\n");
		return ret;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
		xvp->vdsp_mem_desc,
		&xvp->ion_firmware,
		IOMMU_ALL);
	if (unlikely(ret)){
		pr_err("[ERROR]map firmware failed\n");
		return ret;
	}
	xvp->dsp_firmware_addr = xvp->ion_firmware.iova[0];
	pr_debug("dsp fw addr:%lx\n", (unsigned long)xvp->dsp_firmware_addr);

	return ret;
}

static int sprd_iommu_unmap_extrabuffer(struct xvp *xvp)
{
	int ret = -EFAULT;

	if (unlikely(xvp->firmware_viraddr == NULL)) {
		pr_err("[ERROR]NULL firmware viraddr\n");
		return ret;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(
		xvp->vdsp_mem_desc,
		&xvp->ion_firmware,
		IOMMU_ALL);
	if (ret)
		pr_err("[ERROR]iommu unmap failed\n");

	pr_debug("unmap fw vddr:%p, ret:%d\n", xvp->firmware_viraddr, ret);

	return ret;
}

static int sprd_vdsp_alloc_map(struct xvp *xvp)
{
	int ret = 0;

	if (xvp->secmode) {
		ret = sprd_iommu_map_faceid_fwbuffer(xvp);
		if (ret != 0) {
			pr_err("[ERROR]map faceid fw failed\n");
			return  ret;
		}
	}else {
		ret = sprd_alloc_extrabuffer(xvp);
		if (unlikely(ret != 0)) {
			pr_err("[ERROR]sprd_alloc_extrabuffer failed\n");
			return ret;
		}
		ret = sprd_iommu_map_extrabuffer(xvp);
		if (unlikely(ret != 0)) {
			pr_err("[ERROR]sprd_iommu_map_extrabuffer failed\n");
			sprd_free_extrabuffer(xvp);
			return ret;
		}
	}
	ret = sprd_iommu_map_commbuffer(xvp);
	if (unlikely(ret != 0)) {
		if (xvp->secmode) {
			sprd_iommu_unmap_faceid_fwbuffer(xvp);
		}else {
			sprd_iommu_unmap_extrabuffer(xvp);
			sprd_free_extrabuffer(xvp);
		}
		pr_err("[ERROR]sprd_iommu_map_commbuffer failed\n");
		return ret;
	}

	return ret;
}

static int sprd_vdsp_clear_buff(struct xvp *xvp)
{
	int ret = 0;

	if (xvp->secmode) {
		ret = sprd_iommu_unmap_faceid_fwbuffer(xvp);
		if (ret != 0) {
			pr_err("[ERROR]unmap faceid fw failed\n");
		}
	}else {
		ret = sprd_iommu_unmap_extrabuffer(xvp);
		if (unlikely(ret != 0))
			pr_err("[ERROR]sprd_iommu_unmap_extrabuffer failed\n");

		ret = sprd_free_extrabuffer(xvp);
		if (unlikely(ret != 0))
			pr_err("[ERROR]sprd_free_extrabuffer failed \n");
	}

	ret = sprd_iommu_unmap_commbuffer(xvp);
	if (unlikely(ret != 0))
		pr_err("[ERROR]sprd_iommu_unmap_commbuffer failed\n");

	return ret;
}


static int xvp_open(struct inode *inode, struct file *filp)
{
	struct xvp *xvp = container_of(filp->private_data,
		struct xvp, miscdev);
	struct xvp_file *xvp_file;
	int ret;
	uint32_t opentype = 0xffffffff;
	s64 tv0, tv1, tv2, tv3;
	struct vdsp_ipi_ctx_desc *vdsp_ipi_desc = get_vdsp_ipi_ctx_desc();

	tv0 = ktime_to_us(ktime_get());

	pr_debug("xvp is:%p, filp:%lx , flags:%x, fmode:%x\n", xvp, (unsigned long)filp , filp->f_flags, filp->f_mode);
	mutex_lock(&xvp_global_lock/*xvp->xvp_lock*/);
	if (filp->f_flags & O_RDWR){
		/*check cur open type*/
		if(xvp->soc_type != 0) {
			pr_err("this platform does not support faceid\n");
			ret = -EFAULT;
			goto err_unlock;
		}
		if (likely((xvp->cur_opentype == 0xffffffff) || (xvp->cur_opentype == 1))) {
			pr_debug("open faceid mode!!!!!\n");

			ret = sprd_faceid_secboot_init(xvp);
			if (unlikely(ret < 0)) {
				goto err_unlock;
			}

			opentype = 1;
		}else {
			pr_err("open faceid mode but refused by curr opentype:%u\n",
				xvp->cur_opentype);
			ret = -EINVAL;
			goto err_unlock;
		}
	}else {
		if (unlikely((xvp->cur_opentype != 0xffffffff) && (xvp->cur_opentype != 0))) {
			pr_err("open failed refused by curr opentype:%u\n",
				xvp->cur_opentype);
			ret = -EINVAL;
			goto err_unlock;
		}
		opentype = 0;
		vdsp_ipi_desc->ops->irq_register(0,
				xrp_hw_irq_handler_ex, xvp);
	}
	xvp_file = devm_kzalloc(xvp->dev, sizeof(*xvp_file), GFP_KERNEL);
	if (unlikely(!xvp_file)) {
		pr_err("devm_kzalloc failed\n");
		ret = -ENOMEM;
		goto err_unlock;
	} else {
		/*init lists in xvp_file*/
		INIT_LIST_HEAD(&xvp_file->load_lib_list);
		mutex_init(&xvp_file->lock);
	}
	tv1 = ktime_to_us(ktime_get());
	if (!xvp->open_count) {
		ret = xvp_enable_dsp(xvp);
		//TBD enable function can not return failed status
		if (unlikely(ret < 0)) {
			pr_err("[ERROR]couldn't enable DSP\n");
			goto devmalloc_fault;
		}
		ret = sprd_vdsp_alloc_map(xvp);
		if (unlikely(ret < 0)) {
			pr_err("[ERROR]open alloc or map failed\n");
			goto enable_fault;
		}
		ret = vdsp_log_mapbuffer(xvp);
		if (unlikely(ret != 0)) {
			pr_err("vdsp log init failed\n");
			goto map_fault;
		}
		ret = vdsp_dvfs_init(xvp);
		if (unlikely(ret < 0)) {
			pr_err("vdsp_dvfs_init failed\n");
			goto log_map_fault;
		}
	}
	tv2 = ktime_to_us(ktime_get());
	ret = pm_runtime_get_sync(xvp->dev);
	if (unlikely(ret < 0)) {
		pr_err("[ERROR]pm_runtime_get_sync failed\n");
		goto dvfs_fault;
	}else {
		ret = 0;
	}
	pr_debug("func:%s step 1\n" , __func__);
	if (!xvp->open_count) {
		ret = sprd_vdsp_boot_firmware(xvp);
		if (unlikely(ret < 0)) {
			pr_err("[ERROR]vdsp boot firmware failed\n");
			goto err_pm_runtime;
		}
		if (!ret && (log_counter == 0)) {
			log_counter++;
			sprd_log_sem_up();
		}
		xvp->off = false;
	}

	xvp_file->xvp = xvp;
	spin_lock_init(&xvp_file->busy_list_lock);
	filp->private_data = xvp_file;
	pr_debug("func:%s step 2\n" , __func__);
	xrp_add_known_file(filp);
	tv3 = ktime_to_us(ktime_get());
	/*total - map*/
	pr_info("[TIME]VDSP Boot total(xvp->sync done):%lld(us), map firmware:%lld(us)\n",
		tv3 - tv0, tv2 - tv1);
	xvp->open_count++;
	xvp->cur_opentype = opentype;
	mutex_unlock(&xvp_global_lock/*&xvp->xvp_lock*/);
	return ret;

err_pm_runtime:
	pm_runtime_put_sync(xvp->dev);
dvfs_fault:
	vdsp_dvfs_deinit(xvp);
log_map_fault:
	vdsp_log_unmapbuffer(xvp);
map_fault:
	sprd_vdsp_clear_buff(xvp);
enable_fault:
	xvp_disable_dsp(xvp);
devmalloc_fault:
	devm_kfree(xvp->dev, xvp_file);
err_unlock:
	pr_err("[ERROR]ret = %ld\n", ret);
	if(opentype == 1) {
		sprd_faceid_secboot_deinit(xvp);
	}
	mutex_unlock(&xvp_global_lock/*&xvp->xvp_lock*/);
	return ret;
}

static int xvp_close(struct inode *inode, struct file *filp)
{
	struct xvp_file *xvp_file = filp->private_data;
	int ret = 0;
	struct xvp *xvp = (struct xvp*)(xvp_file->xvp);

//	mutex_lock(&xvp->xvp_lock);
	mutex_lock(&xvp_global_lock);
	mutex_lock(&xvp_file->lock);
	while(xvp_file->working) {
		mutex_unlock(&xvp_file->lock);
		pr_warn("func:%s xvpfile is working\n" , __func__);
		schedule();
		mutex_lock(&xvp_file->lock);
	}
	/*wait for xvp_file idle*/
	pm_runtime_put_sync(xvp_file->xvp->dev);
	xvp_file->xvp->open_count--;

	pr_debug("xvp_close open_count is:%d , filp:%lx\n", xvp_file->xvp->open_count , (unsigned long)filp);
	if (0 == xvp_file->xvp->open_count) {
		xvp_disable_dsp(xvp_file->xvp);
		vdsp_log_unmapbuffer(xvp);
		/*release xvp load_lib info*/
		mutex_lock(&(xvp->load_lib.libload_mutex));
		pr_debug("func:%s before xrp_library_release_all\n" , __func__);
		ret = xrp_library_release_all(xvp_file->xvp);
		pr_debug("func:%s after xrp_library_release_all\n" , __func__);
		mutex_unlock(&(xvp->load_lib.libload_mutex));
		sprd_vdsp_clear_buff(xvp_file->xvp);
		xvp_file->xvp->cur_opentype = 0xffffffff;
		vdsp_dvfs_deinit(xvp_file->xvp);
	} else {
		xvp_file_release_list(filp);
	}
	if(xvp->soc_type == 0) {
		ret = sprd_faceid_secboot_deinit(xvp_file->xvp);
	}
	/*release lists in xvp_file*/
	xrp_remove_known_file(filp);
	pr_debug("[OUT]ret:%d\n", ret);
	filp->private_data = NULL;
	mutex_unlock(&xvp_file->lock);
	mutex_unlock(&xvp_global_lock);
//	mutex_unlock(&xvp->xvp_lock);
	devm_kfree(xvp_file->xvp->dev, xvp_file);
	return ret;
}

static const struct file_operations xvp_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = xvp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = xvp_ioctl,
#endif
	.open = xvp_open,
	.release = xvp_close,
};

int vdsp_runtime_resume(struct device *dev)
{
	struct vdsp_ipi_ctx_desc *ipidesc = NULL;
	struct xvp *xvp = dev_get_drvdata(dev);

	if (unlikely(xvp->off)) {
		//TBD CHECK
	}

	ipidesc = get_vdsp_ipi_ctx_desc();
	if (ipidesc) {
		pr_debug("ipi init called\n");
		ipidesc->ops->ctx_init(ipidesc);
	}

	/*set qos*/
	xvp_set_qos(xvp);

	return 0;
}
EXPORT_SYMBOL(vdsp_runtime_resume);

int vdsp_runtime_suspend(struct device *dev)
{
	struct vdsp_ipi_ctx_desc *ipidesc = NULL;
	struct xvp *xvp = dev_get_drvdata(dev);

	xrp_halt_dsp(xvp);
	ipidesc = get_vdsp_ipi_ctx_desc();
	if (ipidesc) {
		pr_debug("ipi deinit called\n");
		ipidesc->ops->ctx_deinit(ipidesc);
	}

	return 0;
}
EXPORT_SYMBOL(vdsp_runtime_suspend);

static int sprd_vdsp_init_buffer(
struct platform_device *pdev, struct xvp *xvp)
{
	if (unlikely(sprd_alloc_commbuffer(xvp) != 0)){
		pr_err("sprd_alloc_commbuffer failed\n");
		return -EFAULT;
	}

	if(xvp->soc_type == 0) {
		if (unlikely(sprd_faceid_init(xvp) != 0)){
			pr_err("sprd_alloc_faceid buffer failed\n");
			return -EFAULT;
		}
	}
	return 0;
}

static int sprd_vdsp_parse_soft_dt(struct xvp *xvp,
struct platform_device *pdev)
{
	int i, ret = 0;

	ret = device_property_read_u32_array(xvp->dev,
		"queue-priority", NULL, 0);
	if (ret > 0) {
		xvp->n_queues = ret;
		xvp->queue_priority = devm_kmalloc(&pdev->dev,
			ret * sizeof(u32),
			GFP_KERNEL);
		if (xvp->queue_priority == NULL) {
			pr_err("failed to kmalloc queue priority \n");
			ret = -EFAULT;
			return ret;
		}

		ret = device_property_read_u32_array(xvp->dev,
			"queue-priority",
			xvp->queue_priority,
			xvp->n_queues);
		if (ret < 0){
			pr_err("failed to read queue priority \n");
			ret = -EFAULT;
			return ret;
		}

		pr_debug("multiqueue (%d) configuration, queue priorities:\n", xvp->n_queues);
		for (i = 0; i < xvp->n_queues; ++i)
			pr_debug("  %d\n", xvp->queue_priority[i]);
	}else {
		xvp->n_queues = 1;
	}

	xvp->queue = devm_kmalloc(&pdev->dev,
		xvp->n_queues * sizeof(*xvp->queue),
		GFP_KERNEL);
	xvp->queue_ordered = devm_kmalloc(&pdev->dev,
		xvp->n_queues * sizeof(*xvp->queue_ordered),
		GFP_KERNEL);
	if (xvp->queue == NULL ||
		xvp->queue_ordered == NULL){
		pr_err("failed to kmalloc queue ordered \n");
		ret = -EFAULT;
		return ret;
	}

	for (i = 0; i < xvp->n_queues; ++i) {
		mutex_init(&xvp->queue[i].lock);
		xvp->queue[i].comm = xvp->comm + XRP_DSP_CMD_STRIDE * i;
		init_completion(&xvp->queue[i].completion);
		if (xvp->queue_priority)
			xvp->queue[i].priority = xvp->queue_priority[i];
		xvp->queue_ordered[i] = xvp->queue + i;
		pr_debug("queue i:%d, comm:0x%x\n", i, xvp->queue[i].comm);
	}
	sort(xvp->queue_ordered, xvp->n_queues,
		sizeof(*xvp->queue_ordered),
		compare_queue_priority, NULL);
	ret = device_property_read_string(xvp->dev, "firmware-name", &xvp->firmware_name);
	if (unlikely(ret == -EINVAL || ret == -ENODATA))
		pr_err("no firmware-name property, not loading firmware");
	else if (unlikely(ret < 0))
		pr_err("invalid firmware name (%ld)", ret);

	return ret;
}

static long vdsp_init_common(struct platform_device *pdev,
	enum vdsp_init_flags init_flags,
	const struct xrp_hw_ops *hw_ops, void *hw_arg,
	int(*vdsp_init_bufs)(struct platform_device *pdev,
	struct xvp *xvp))
{
	long ret;
	char nodename[sizeof("vdsp") + 3 * sizeof(int)];
	struct xvp *xvp;
	int nodeid;

	xvp = devm_kzalloc(&pdev->dev, sizeof(*xvp), GFP_KERNEL);
	if (unlikely(!xvp)) {
		ret = -ENOMEM;
		goto err;
	}
	sprd_log_sem_init();
	mutex_init(&(xvp->load_lib.libload_mutex));
	mutex_init(&(xvp->dvfs_info.dvfs_lock));
	mutex_init(&xvp_global_lock);
	mutex_init(&xvp->xvp_lock);
	init_files_know_info(xvp);
	xvp->open_count = 0;
	xvp->dvfs_info.dvfs_init = 0;
	xvp->cur_opentype = 0xffffffff; /*0 is normal type , 1 is faceid , 0xffffffff is initialized value*/
	xvp->dev = &pdev->dev;
	xvp->hw_ops = hw_ops;
	xvp->hw_arg = hw_arg;
	xvp->secmode = false;
	xvp->tee_con = false;
	xvp->irq_status = IRQ_STATUS_REQUESTED;
	xvp->vdsp_mem_desc = get_vdsp_mem_ctx_desc(xvp->dev);
	xvp->soc_type = xvp_get_type();
	if (unlikely(!xvp->vdsp_mem_desc)) {
		pr_err("fail to get mem desc\n");
		ret = -1;
		goto err;
	}
	mutex_init(&(xvp->vdsp_mem_desc->iommu_lock));

	if (init_flags & XRP_INIT_USE_HOST_IRQ)
		xvp->host_irq_mode = true;
	platform_set_drvdata(pdev, xvp);

	if (unlikely(vdsp_init_bufs(pdev, xvp))) {
		ret = -1;
		goto err;
	}
	if (unlikely(sprd_vdsp_parse_soft_dt(xvp, pdev))) {
		ret = -1;
		goto err;
	}

	if (unlikely(vdsp_log_init(xvp) != 0))
		pr_err("vdsp log init fail. \n");

	pm_runtime_enable(xvp->dev);
	if (!pm_runtime_enabled(xvp->dev)) {
		ret = vdsp_runtime_resume(xvp->dev);
		if (ret)
			goto err_pm_disable;
	}

	nodeid = ida_simple_get(&xvp_nodeid, 0, 0, GFP_KERNEL);
	if (unlikely(nodeid < 0)) {
		ret = nodeid;
		goto err_pm_disable;
	}
	xvp->nodeid = nodeid;
	sprintf(nodename, "vdsp%u", nodeid);


	xvp->miscdev = (struct miscdevice){
		.minor = MISC_DYNAMIC_MINOR,
			.name = devm_kstrdup(&pdev->dev, nodename, GFP_KERNEL),
			.nodename = devm_kstrdup(&pdev->dev, nodename, GFP_KERNEL),
			.fops = &xvp_fops,
	};

	ret = misc_register(&xvp->miscdev);
	if (unlikely(ret < 0))
		goto err_free_id;
	return PTR_ERR(xvp);
err_free_id:
	ida_simple_remove(&xvp_nodeid, nodeid);
err_pm_disable:
	pm_runtime_disable(xvp->dev);
err:
	pr_err("error, ret = %ld\n", ret);
	return ret;
}

long sprd_vdsp_init(struct platform_device *pdev,
	enum vdsp_init_flags flags,
	const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return vdsp_init_common(
		pdev, flags, hw_ops,
		hw_arg,
		sprd_vdsp_init_buffer);
}
EXPORT_SYMBOL(sprd_vdsp_init);

int sprd_vdsp_deinit(struct platform_device *pdev)
{
	struct xvp *xvp = platform_get_drvdata(pdev);

	pm_runtime_disable(xvp->dev);
	if (!pm_runtime_status_suspended(xvp->dev))
		vdsp_runtime_suspend(xvp->dev);

	misc_deregister(&xvp->miscdev);
	release_firmware(xvp->firmware);
	sprd_free_commbuffer(xvp);
	if(xvp->soc_type == 0) {
		sprd_faceid_deinit(xvp);
	}
	vdsp_log_deinit(xvp);
	ida_simple_remove(&xvp_nodeid, xvp->nodeid);

	return 0;
}
EXPORT_SYMBOL(sprd_vdsp_deinit);

MODULE_AUTHOR("Takayuki Sugawara");
MODULE_AUTHOR("Max Filippov");
MODULE_DESCRIPTION("XRP: Linux device driver for Xtensa Remote Processing");
MODULE_LICENSE("Dual MIT/GPL");
