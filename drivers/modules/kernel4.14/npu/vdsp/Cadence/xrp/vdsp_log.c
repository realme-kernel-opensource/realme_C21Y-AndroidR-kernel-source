
#include <linux/notifier.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include "xrp_internal.h"
#include "vdsp_log.h"
#include "vdsp_dump.h"

#define VDSP_LOG_BUFFER_SZIE (1024*128)
#define BANK_BUSY 0
#define BANK_READY 1

#define COREDUMP_NONE		(0)
#define COREDUMP_START		(1)
#define COREDUMP_STACK_DONE	(2)
#define COREDUMP_FINISH		(3)

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "sprd-vdsp: log %d %d : "\
        fmt, current->pid, __LINE__

struct log_header_stru
{
	volatile int32_t mode;	// 0:not wait,1:wait
	volatile int32_t bank_size;
	volatile int32_t flag[2];	//flag, write disable/enable
	volatile uint32_t addr[2];	//log bank address
	volatile uint32_t log_size[2];
	volatile uint32_t log_level;
	volatile uint32_t coredump_flag;
	volatile uint32_t reverse[6];
}log_header;


static int log_read_line(struct vdsp_log_state *s, int put, int get, char *addr)
{
	int i;
	char c = '\0';
	size_t max_to_read = min((size_t)(put - get),
				 sizeof(s->line_buffer) - 1);

	for (i = 0; i < max_to_read && c != '\n';)
		s->line_buffer[i++] = c = addr[get++];
	s->line_buffer[i] = '\0';
	return i;
}

static void vdsp_dump_logs(struct vdsp_log_state *s)
{
	uint32_t put, get, bank;
	char *addr;
	int read_chars;

	struct log_header_stru *log = (struct log_header_stru*)s->log_vaddr;

	if(BANK_READY == log->flag[0]) {
		bank = 0;
		addr = (char*)s->log_vaddr + sizeof(struct log_header_stru);
	} else if(BANK_READY == log->flag[1]) {
		bank = 1;
		addr = (char*)s->log_vaddr + sizeof(struct log_header_stru) + log->bank_size;
	} else {
		pr_err("bank is busy\n");
		return;
	}
	get = s->get;
	put = log->log_size[bank];
	pr_debug("addr %x, log size %d\n", addr, put);
	while (put != get) {
			/* Make sure that the read of put occurs before the read of log data */
			rmb();

			/* Read a line from the log */
			read_chars = log_read_line(s,put,get,addr);

			/* Force the loads from log_read_line to complete. */
			rmb();

		pr_info("%s", s->line_buffer);
		get += read_chars;
	}
	log->flag[bank] = BANK_BUSY;
	return;
}

irqreturn_t vdsp_log_irq_handler(int irq, void *private)
{
	struct xvp * xvp= (struct xvp *)private;
	struct vdsp_log_state *s;
	struct vdsp_log_work *vlw;

	s = xvp->log_state;
	vlw = this_cpu_ptr(s->nop_works);
	queue_work(s->nop_wq, &vlw->work);

	return IRQ_HANDLED;
}
static void vdsp_log_nop_work_func(struct work_struct *work)
{
	struct vdsp_log_state *s;
	struct vdsp_log_work *vlw = container_of(work, struct vdsp_log_work, work);
	unsigned long flags;

	s = vlw->vls;
	spin_lock_irqsave(&s->lock, flags);
	vdsp_dump_logs(s);
	spin_unlock_irqrestore(&s->lock, flags);
}

int vdsp_log_alloc_buffer(struct xvp *xvp)
{
	int ret;
	struct ion_buf *ion_buf = &xvp->ion_vdsp_log;
	struct vdsp_log_state *s = xvp->log_state;

	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
					ion_buf,
					ION_HEAP_ID_MASK_SYSTEM,
					VDSP_LOG_BUFFER_SZIE);
	if(0 != ret) {
		pr_err("alloc log buffer failed\n");
		return -ENOMEM;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, ion_buf);
	if(0 != ret) {
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, ion_buf);
		return -EFAULT;
	}
	ion_buf->dev = xvp->dev;
	s->log_vaddr = (void*)ion_buf->addr_k[0];

	pr_debug("log buffer vdsp va:%lx\n", ion_buf->addr_k[0]);

	return 0;
}
int vdsp_log_free_buffer(struct xvp *xvp)
{
	struct ion_buf *ion_buf = &xvp->ion_vdsp_log;
	unsigned long dst_viraddr = ion_buf->addr_k[0];

	if(dst_viraddr) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, ion_buf);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, ion_buf);
	}
	return 0;
}

int vdsp_log_mapbuffer(struct xvp* xvp)
{
	int ret;
	struct ion_buf *ion_buf = &xvp->ion_vdsp_log;
	struct log_header_stru *log;

	log = (struct log_header_stru*)xvp->log_state->log_vaddr;
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(
			xvp->vdsp_mem_desc,
			ion_buf,
			IOMMU_ALL);
	if(ret != 0) {
		return -EFAULT;
	}
	log->addr[0] = xvp->ion_vdsp_log.iova[0] + sizeof(struct log_header_stru);
	log->addr[1] = xvp->ion_vdsp_log.iova[0] + sizeof(struct log_header_stru) + log->bank_size;
	log->flag[0] = 0;					//init log flag, BANK_BUSY
	log->flag[1] = 0;
	log->log_size[0] = 0;				//vdsp write log size
	log->log_size[1] = 0;
	pr_debug("log addr map, 0x%x-0x%x", log->addr[0], log->addr[1]);
	return 0;
}

int vdsp_log_unmapbuffer(struct xvp* xvp)
{
	int ret;
	struct ion_buf *ion_buf = &xvp->ion_vdsp_log;

	ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, ion_buf , IOMMU_ALL);

	return ret;
}

int vdsp_log_init(struct xvp *xvp)
{
	struct vdsp_log_state *s;
	int result;
	struct log_header_stru *log;
	unsigned int cpu;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		result = -ENOMEM;
		goto error_alloc_state;
	}
	xvp->log_state = s;

	spin_lock_init(&s->lock);
	s->get = 0;
	s->nop_wq = alloc_workqueue("vdsplog-nop-wq", WQ_CPU_INTENSIVE, 0);
	if (!s->nop_wq) {
		result = -ENODEV;
		pr_err("Failed create vdsplog-nop-wq\n");
		goto err_create_nop_wq;
	}

	s->nop_works = alloc_percpu(struct vdsp_log_work);
	if (!s->nop_works) {
		result = -ENOMEM;
		pr_err("Failed to allocate works\n");
		goto err_alloc_works;
	}

	for_each_possible_cpu(cpu) {
		struct vdsp_log_work *vlw = per_cpu_ptr(s->nop_works, cpu);

		vlw->vls = s;
		INIT_WORK(&vlw->work, vdsp_log_nop_work_func);
	}

	s->vdsp_ipi_desc = get_vdsp_ipi_ctx_desc();
	if (!s->vdsp_ipi_desc) {
		result = -ENOMEM;
		pr_err("get ipi desc error\n");
		goto error_get_desc;
	}
	/*register IPI callback*/
	s->vdsp_ipi_desc->ops->irq_register(2,
						     vdsp_log_irq_handler,
						     xvp);

	if(vdsp_log_alloc_buffer(xvp) != 0) {
		result = -ENOMEM;
		goto error_alloc_log;
	}

	log = (struct log_header_stru*)s->log_vaddr;
	log->bank_size = (VDSP_LOG_BUFFER_SZIE - sizeof(struct log_header_stru)) / 2;

	log->mode = LOG_OVERFLOW_MODE;		//wait or ignore when log full
	log->flag[0] = 0;					//init log flag, BANK_BUSY
	log->flag[1] = 0;
	log->addr[0] = 0;
	log->addr[1] = 0;
	log->log_size[0] = 0;				//vdsp write log size
	log->log_size[1] = 0;
	log->log_level = 5;					//vdsp log level
	log->coredump_flag = COREDUMP_NONE;

	pr_info("log header addr:%lx, bank addr:%x,%x\n", xvp->ion_vdsp_log.iova[0], log->addr[0], log->addr[1]);

	return 0;

error_alloc_log:
error_get_desc:
	for_each_possible_cpu(cpu) {
		struct vdsp_log_work *vlw  = per_cpu_ptr(s->nop_works, cpu);

		flush_work(&vlw->work);
	}
	free_percpu(s->nop_works);
err_alloc_works:
	destroy_workqueue(s->nop_wq);
err_create_nop_wq:
	kfree(s);
error_alloc_state:
	return result;
}
int vdsp_log_deinit(struct xvp *xvp)
{
	struct vdsp_log_state *s;
	unsigned int cpu;

	s = xvp->log_state;
	if(s) {
		for_each_possible_cpu(cpu) {
			struct vdsp_log_work *vlw  = per_cpu_ptr(s->nop_works, cpu);

			flush_work(&vlw->work);
		}
		free_percpu(s->nop_works);
		destroy_workqueue(s->nop_wq);
		vdsp_log_free_buffer(xvp);
		kfree(s);
	}

	return 0;
}

int vdsp_log_coredump(struct xvp *xvp)
{
	int res = -1;
	int dump_res = -1;
	struct log_header_stru *log = (struct log_header_stru*)xvp->log_state->log_vaddr;
	unsigned long deadline = jiffies + 5 * HZ;	//5s

	pr_info("dump start,flag:%d \n", log->coredump_flag);
	if(log->coredump_flag == COREDUMP_START) {
		dump_res = xrp_dump_libraries(xvp);
		log->coredump_flag = COREDUMP_STACK_DONE;
		do{
			if(log->coredump_flag == COREDUMP_FINISH){
				res = 0;
				break;
			}
			schedule();
		}while(time_before(jiffies, deadline));
	}
	log->coredump_flag = COREDUMP_NONE;
	pr_info("dump end, process(-1/0):%d, data(-1/0):%d \n", res, dump_res);
	return res;
}



