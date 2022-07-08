#ifndef __FD_DRV_H__
#define __FD_DRV_H__

#include "fd_core.h"

#define FD_DRV_OP_READ 1
#define FD_DRV_OP_WRITE 2

enum FD_BUF_INDEX {
	FD_BUF_CMD_INDEX,
	FD_BUF_IMG_INDEX,
	FD_BUF_DIM_INDEX,
	FD_BUF_OUT_INDEX,
	FD_BUF_INDEX_MAX
};

enum sprd_fd_state {
	SPRD_FD_STATE_CLOSED,
	SPRD_FD_STATE_IDLE,
	SPRD_FD_STATE_RUNNING,
	SPRD_FD_STATE_ERROR
};

enum fd_syscon {
	FD_SYSCON_ENABLE,
	FD_SYSCON_RESET,
	FD_SYSCON_SAVE_RESET,
	FD_SYSCON_LPC,
	FD_SYSCON_DVFS_ENABLE,
	FD_SYSCON_MAX
};

struct fd_buf {
	unsigned int mfd;
	struct camera_buf buf_info;
};

struct fd_syscon_info {
	struct regmap *reg_map;
	unsigned int reg;
	unsigned int mask;
};

struct fd_drv {
	atomic_t pw_users;
	int irq_no;
	unsigned long io_base;
	unsigned long io_dvfs_base;
	uint32_t awqos;
	uint32_t arqos;
	enum sprd_fd_state state;
	struct clk *fd_eb;
	struct clk *clk;
	struct clk *clk_parent;
	struct clk *axi_eb;
	struct clk *clk_default;
	struct platform_device *pdev;
	struct regmap *cam_dvfs_gpr;
	struct fd_syscon_info syscon[FD_SYSCON_MAX];
	struct completion fd_wait_com;
	struct fd_buf fd_buf_info[FD_BUF_INDEX_MAX];
};

struct fd_reg_info {
	unsigned int reg_addr;
	int (*reg_write)(struct fd_drv *hw_handle,
	    unsigned int reg_addr,
	    unsigned int val);

};

struct sprd_fd_dvfs {
	unsigned int reg_addr;
	enum sprd_fd_clk_freq feq_sel;
	int mtx_index;
	int volte;
};

int sprd_fd_drv_init(void **fd_drv_handle,
	struct device_node *dn);

int sprd_fd_drv_deinit(void *fd_handle);
int fd_drv_reg_write_handler(void *handle,
	struct sprd_fd_cfg_param  *cfg_param);

int fd_drv_reg_multi_write_handler(void *handle,
	    struct sprd_fd_multi_reg_cfg_param  *param);

int fd_drv_reg_read_handler(void *handle, unsigned long arg);

int fd_drv_reg_multi_read_handler(void *handle,
	    struct sprd_fd_multi_reg_cfg_param  *param);
int fd_drv_dvfs_work_clk_cfg(void *handle,
	unsigned int index);

int fd_drv_dvfs_idle_clk_cfg(void *handle,
	unsigned int index);
int sprd_fd_drv_open(void *drv_handle);
int sprd_fd_drv_close(void *drv_handle);
#endif
