#ifndef _VDSP_COMMON_H
#define _VDSP_COMMON_H
#include "sprd_vdsp_cmd.h"
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include "ion.h"
#include <linux/semaphore.h>
#include <linux/spinlock_types.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include "vdsp_lib.h"


typedef enum {
	SHARKL3 = 1,
	ROC1 = 2,
	MAX_VERSIONS,
} VDSP_VERSION_E;


struct sprd_vdsp_cfg_data {
	unsigned int version;
	unsigned int max_freq_level;
	unsigned int softreset_reg_offset;
	unsigned int reset_mask;
};
struct vdsp_context {
	unsigned long xm6_base;
	unsigned long icu_base;
	unsigned int  base_offset[2];
	int irq;
	bool is_inited;
	bool is_stopped;
};

struct vdsp_img_size {
        uint32_t width;
        uint32_t height;
};

struct vdsp_msg_info {
        uint32_t id;
        uint32_t irq_type;
        uint32_t irq_flag;
        uint32_t format;
        uint32_t channel_id;
        uint32_t base_id;
        uint32_t img_id;
        uint32_t irq_id;
        uint32_t sensor_id;
        uint32_t statis_cnt;
        unsigned long yaddr;
        unsigned long uaddr;
        unsigned long vaddr;
        unsigned long yaddr_vir;
        unsigned long uaddr_vir;
        unsigned long vaddr_vir;
        uint32_t img_y_fd;
        uint32_t img_u_fd;
        uint32_t img_v_fd;
        unsigned long length;
        struct vdsp_img_size buf_size;
        //struct sprd_sp_time time_stamp;
        uint32_t frm_index;
};



struct msg_pending_post {
	struct list_head head;
	struct sprd_dsp_cmd msg;
	void *state;
};
struct vdsp_buf {
	struct device *dev;
	unsigned int mfd[3];
	struct sg_table *table[3];
	void *buf[3];
	size_t size[3];
	unsigned long iova[3];
	struct dma_buf *dmabuf_p[3];
	unsigned int offset[3];
};

struct camera_buf {
	/* user buffer info */
	int32_t mfd[3];
	struct dma_buf *dmabuf_p[3];

	/* kernel buffer info */
	struct ion_client *client[3];
	struct ion_handle *handle[3];

	void *ionbuf[3]; /* for iommu map */
	uint32_t offset[3];
	size_t size[3];
	unsigned long addr_vir[3];
	unsigned long addr_k[3];
	unsigned long iova[3];
	struct device *dev; /* mapped device */
	uint32_t mapping_state;
};


extern struct list_head vdsp_core_head;
extern struct list_head vdsp_clk_head;
extern struct list_head vdsp_glb_head;

#define vdsp_core_ops_register(entry) \
	vdsp_ops_register(entry, &vdsp_core_head)
#define vdsp_clk_ops_register(entry) \
	vdsp_ops_register(entry, &vdsp_clk_head)
#define vdsp_glb_ops_register(entry) \
	vdsp_ops_register(entry, &vdsp_glb_head)

#define vdsp_core_ops_attach(str) \
	vdsp_ops_attach(str, &vdsp_core_head)
#define vdsp_clk_ops_attach(str) \
	vdsp_ops_attach(str, &vdsp_clk_head)
#define vdsp_glb_ops_attach(str) \
	vdsp_ops_attach(str, &vdsp_glb_head)
struct vdsp_core_ops {
	int (*parse_dt)(struct vdsp_context *ctx,
			struct device_node *np);
	int (*do_pdma)(struct vdsp_context *ctx,
			u32 code_addr, u32 size);
	int (*do_ddma)(struct vdsp_context *ctx,
			u32 data_addr, u32 size);
	int (*set_pcache)(struct vdsp_context *ctx);
	int (*set_edp_aximo_range)(struct vdsp_context *ctx);
	int (*isr_triggle_int0)(struct vdsp_context *ctx, u32 index);
	int (*isr_triggle_int1)(struct vdsp_context *ctx, u32 index);
	int (*isr)(struct vdsp_context *ctx);
	void (*dump)(struct vdsp_context *ctx);
};

struct vdsp_clk_ops {
	int (*parse_dt)(struct vdsp_context *ctx,
			struct device_node *np);
	int (*init)(struct vdsp_context *ctx);
	int (*uinit)(struct vdsp_context *ctx);
	int (*enable)(struct vdsp_context *ctx);
	int (*disable)(struct vdsp_context *ctx);
	int (*update)(struct vdsp_context *ctx, int clk_id, int val);
};

struct vdsp_glb_ops {
	int (*parse_dt)(struct vdsp_context *ctx,
			struct device_node *np);
	void (*init)(struct vdsp_context *ctx);
	void (*enable)(struct vdsp_context *ctx);
	void (*disable)(struct vdsp_context *ctx);
	void (*reset)(struct vdsp_context *ctx);
	void (*core_reset)(struct vdsp_context *ctx);
	void (*acu_acc)(struct vdsp_context *ctx, int enbale);
	void (*boot_signal_set)(struct vdsp_context *ctx, int enbale);
	void (*vector_addr_set)(struct vdsp_context *ctx, u32 ext_code_addr);
	void (*power)(struct vdsp_context *ctx, int enable);
};


struct vdsp_dev_t {
	unsigned int irq;
	unsigned int version;
	struct sprd_vdsp_cfg_data *vdsp_cfg_data;
	struct device_node *dev_np;
	struct device dev;
	struct kthread_worker post_worker;
	struct task_struct *post_thread;
	struct kthread_work post_work;
	struct mutex post_lock;
	struct list_head post_list;
	wait_queue_head_t wait_queue;
	struct task_struct *log_task;
	u32 fw_code_size;
	u32 fw_ext_code_size;
	u32 fw_ext_data_size;
	u32 fw_data_size;
	bool is_opened;
	struct vdsp_context ctx;
	struct vdsp_core_ops *core;
	struct vdsp_clk_ops *clk;
	struct vdsp_glb_ops *glb;
	struct vdsp_buf ion_in_buf;
	struct vdsp_buf ion_out_buf;
	struct vdsp_buf ion_dsp_buf[8];
};

#endif
