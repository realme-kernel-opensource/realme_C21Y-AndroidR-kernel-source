#ifndef _SPRD_VDSP_DRIVER_DVFS_H_
#define _SPRD_VDSP_DRIVER_DVFS_H_
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/ktime.h>

enum sprd_vdsp_kernel_power_level
{
        SPRD_VDSP_KERNEL_POWERHINT_RESTORE_DVFS = -1,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_0,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_1,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_2,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_3,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_4,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_5,
        SPRD_VDSP_KERNEL_POWERHINT_LEVEL_MAX,
};

enum sprd_vdsp_kernel_powerhint_acquire_release
{
        SPRD_VDSP_KERNEL_POWERHINT_ACQUIRE = 0,
        SPRD_VDSP_KERNEL_POWERHINT_RELEASE,
        SPRD_VDSP_KERNEL_POWERHINT_MAX,
};

struct vdsp_dvfs_info {
	struct mutex dvfs_lock;
	struct mutex timepiece_lock;
	struct mutex powerhint_lock;
	uint32_t workingcount;
	ktime_t  starttime;
	ktime_t cycle_totaltime;
	ktime_t piece_starttime;
	uint32_t last_powerhint_level;
	uint32_t last_dvfs_index;
	uint32_t dvfs_init;
	uint32_t max_freq;
	wait_queue_head_t wait_q;
	struct task_struct *dvfs_thread;
};

struct vdsp_dvfs_filpowerhint {
	uint32_t powerhint_count_level[SPRD_VDSP_KERNEL_POWERHINT_LEVEL_MAX];
};

int vdsp_dvfs_init(void *data);
void vdsp_dvfs_deinit(void *data);
int vdsp_dvfs_thread(void* data);
int32_t set_powerhint_flag(void *data , int32_t level , uint32_t acquire_release);
void preprocess_work_piece(void *data);
void postprocess_work_piece(void *data);
void vdsp_release_powerhint(void *filp);
#endif
