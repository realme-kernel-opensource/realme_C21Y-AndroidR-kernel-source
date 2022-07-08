#ifndef __XVP_MAIN_H__
#define __XVP_MAIN_H__

#include <linux/types.h>
#include "xrp_kernel_defs.h"
#include "xrp_alloc.h"
#include "xrp_kernel_dsp_interface.h"
#include "vdsp_dvfs.h"

struct xrp_alien_mapping {
        unsigned long vaddr;
        unsigned long size;
        phys_addr_t paddr;
        void *allocation;
        enum {
                ALIEN_GUP,
                ALIEN_PFN_MAP,
                ALIEN_COPY,
        } type;
};

struct xrp_mapping {
        enum {
                XRP_MAPPING_NONE,
                XRP_MAPPING_NATIVE,
                XRP_MAPPING_ALIEN,
                XRP_MAPPING_KERNEL = 0x4,
        } type;
        union {
                struct {
                        struct xrp_allocation *xrp_allocation;
                        unsigned long vaddr;
                } native;
                struct xrp_alien_mapping alien_mapping;
        };
};

struct xvp_file {
        struct xvp *xvp;
	struct list_head load_lib_list;
	struct vdsp_dvfs_filpowerhint powerhint_info;
        spinlock_t busy_list_lock;
        struct xrp_allocation *busy_list;
	struct mutex lock;
	uint32_t working;
};

struct xrp_known_file {
        void *filp;
        struct hlist_node node;
};

struct xrp_request {
        struct xrp_ioctl_queue ioctl_queue;
        size_t n_buffers;
        struct xrp_mapping *buffer_mapping;
        struct xrp_dsp_buffer *dsp_buffer;
        phys_addr_t in_data_phys;
        phys_addr_t out_data_phys;
        phys_addr_t dsp_buffer_phys;
        struct ion_buf ion_in_buf;
        struct ion_buf ion_out_buf;
        struct ion_buf *ion_dsp_pool;
        struct ion_buf *dsp_buf;
        union {
                struct xrp_mapping in_data_mapping;
                u8 in_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
        };
        union {
                struct xrp_mapping out_data_mapping;
                u8 out_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
        };
        union {
                struct xrp_mapping dsp_buffer_mapping;
                struct xrp_dsp_buffer buffer_data[XRP_DSP_CMD_INLINE_BUFFER_COUNT];
        };
        u8 nsid[XRP_DSP_CMD_NAMESPACE_ID_SIZE];
};

#endif
