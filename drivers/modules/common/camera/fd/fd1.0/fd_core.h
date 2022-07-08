#ifndef __FD_CORE_H__
#define __FD_CORE_H__

#define FD_IDLE_CLK 1
#define FD_WORK_CLK 2


struct fd_module {
	atomic_t pw_users;
	struct miscdevice *md;
	struct platform_device *pdev;
	void *drv_handle;
	struct mutex mod_lock;
};

struct fd_ioctl_cmd {
	unsigned int cmd;
	int (*cmd_proc)(struct fd_module *module,
		       unsigned long arg);
};

#endif
