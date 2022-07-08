
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/highmem.h>
#include <linux/hashtable.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "xrp_internal.h"
#include "xvp_main.h"
#include "xrp_library_loader.h"
#include "xt_library_loader.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_kernel_defs.h"

#define LIBRARY_CMD_PIL_INFO_OFFSET   40
#define LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE 44

#define XRP_EXAMPLE_V3_NSID_INITIALIZER \
{0x73, 0x79, 0x73, 0x74, 0x65, 0x6d, 0x20, 0x63, \
        0x6d, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define LIBRARY_LOAD_UNLOAD_NSID (unsigned char [])XRP_EXAMPLE_V3_NSID_INITIALIZER

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "sprd-vdsp: library_loader %d %d %s : "\
        fmt, current->pid, __LINE__, __func__


static void* libinfo_alloc_element()
{
	struct loadlib_info *pnew = NULL;

	pnew = vmalloc(sizeof(struct loadlib_info));
	if (unlikely(pnew != NULL)) {
		memset(pnew, 0, sizeof(struct loadlib_info));
		mutex_init(&pnew->mutex);
	}

	return pnew;
}


static xt_ptr xt_lib_memcpy(xt_ptr dest, const void * src, unsigned int n, void *user)
{
	memcpy(user, src, n);
	return LIB_RESULT_OK;
}
static xt_ptr xt_lib_memset(xt_ptr s, int c, unsigned int n, void *user)
{
	memset(user, c, n);
	return LIB_RESULT_OK;
}

static uint32_t xrp_check_whether_loaded(struct file *filp , const char* libname , struct loadlib_info **outlibinfo) {
	struct loadlib_info *libinfo , *libinfo1 , *temp, *temp1;
	struct xvp_file * xvpfile_temp;
	unsigned long bkt;
	uint32_t find = 0;
	struct xrp_known_file *p;
	struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;
	struct xvp_file *xvp_file = (struct xvp_file*)filp->private_data;
	libinfo = libinfo1 = temp = temp1 = NULL;
	*outlibinfo = NULL;

	/*check whether loaded myself*/
	list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
		if(0 == strcmp(libinfo->libname , libname)) {
			find = 1;
			break;
		}
	}
	if(1 == find) {
		pr_debug("func:%s this filp has loaded libname:%s\n" , __func__ , libname);
		return 1; /*loaded return 1*/
	}
	pr_debug("func:%s enter0\n" , __func__);
	mutex_lock(&xvp->xrp_known_files_lock);
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
		pr_debug("func:%s enter1\n" , __func__);
		if(((struct file*)(p->filp))->private_data != xvp_file) {
			xvpfile_temp = (struct xvp_file*)(((struct file*)(p->filp))->private_data);
			find = 0;
			pr_debug("func:%s enter1\n" , __func__);
			list_for_each_entry(libinfo1 , &xvpfile_temp->load_lib_list , node_libinfo) {
				pr_debug("func:%s enter3 libname:%s, %s\n" , __func__ , libinfo1->libname , libname);
				if(0 == strcmp(libinfo1->libname , libname)) {
					find = 1;
					*outlibinfo = libinfo1;
					break;
				}
			}
			if(1 == find) {
				break;
			}
		}
	}
	mutex_unlock(&xvp->xrp_known_files_lock);
	return find;
}

static int32_t xrp_library_checkprocessing(struct file *filp , const char *libname)
{
	unsigned long bkt;
	struct xrp_known_file *p;
	struct xvp_file *xvp_file;
	struct xvp *xvp = (struct xvp*) (((struct xvp_file*)(filp->private_data))->xvp);
	struct loadlib_info *libinfo = NULL;
	mutex_lock(&xvp->xrp_known_files_lock);
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
                        xvp_file = (struct xvp_file*)(((struct file*)(p->filp))->private_data);
                        list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
                                if((0 == strcmp(libinfo->libname , libname)) && (libinfo->lib_state == XRP_LIBRARY_PROCESSING_CMD)) {
					pr_info("xrp_library_checkprocessing processing\n");
					mutex_unlock(&xvp->xrp_known_files_lock);
                                        return 1;
                                }
                        }
        }
	mutex_unlock(&xvp->xrp_known_files_lock);
	return 0;
}

static struct loadlib_info *xrp_library_getlibinfo(struct file *filp , const char *libname)
{
	struct loadlib_info *libinfo = NULL;
	struct xvp_file *xvp_file = (struct xvp_file*)filp->private_data;

	list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
		if(0 == strcmp(libinfo->libname , libname)) {
			return libinfo;
		}
	}
	return NULL;
}

static int32_t xrp_library_load_internal(struct file *filp , const char* buffer , const char *libname)
{
	unsigned int size;
	int32_t ret = 0;
	struct loadlib_info *new_element;
	unsigned int result;
	struct ion_buf *lib_ion_mem = NULL;
	struct ion_buf *libinfo_ion_mem = NULL;
	void *kvaddr = NULL;
	void *kvaddr_libinfo = NULL;
	struct xvp_file *xvp_file = (struct xvp_file*)(filp->private_data);
	struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;
	phys_addr_t kpaddr , kpaddr_libinfo;

	/*load library to ddr*/
	size = xtlib_pi_library_size((xtlib_packaged_library *)buffer);
	/*alloc ion buffer later*/
	lib_ion_mem = vmalloc(sizeof(struct ion_buf));
	if(unlikely(lib_ion_mem == NULL)) {
		pr_err("[ERROR]vmalloc fail,lib:%p,libinfo:%p\n", lib_ion_mem);
		return -ENOMEM;
	}
	libinfo_ion_mem = vmalloc(sizeof(struct ion_buf));
	if(unlikely(libinfo_ion_mem == NULL)) {
		pr_err("[ERROR]vmalloc fail,lib:%p,libinfo:%p\n", libinfo_ion_mem);
		ret = -ENOMEM;
		goto __load_internal_err0;
	}
	/*alloc lib ion buffer*/
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
		lib_ion_mem,
		ION_HEAP_ID_MASK_SYSTEM,
		size);
	if (unlikely(ret != 0)) {
		ret = -ENOMEM;
		pr_err("[ERROR]alloc lib_ion_mem failed\n");
		goto __load_internal_err1;
	}
	lib_ion_mem->dev = xvp->dev;
	/*alloc libinfo ion buffer*/
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
		libinfo_ion_mem,
		ION_HEAP_ID_MASK_SYSTEM,
		sizeof(xtlib_pil_info));
	if (unlikely(ret != 0)) {
		ret = -ENOMEM;
		pr_err("[ERROR]alloc libinfo_ion_mem failed\n");
		goto __load_internal_err2;
	}
	libinfo_ion_mem->dev = xvp->dev;
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, lib_ion_mem);
	if(unlikely(ret != 0)) {
		pr_err("[ERROR]mem_kmap lib_ion_mem failed\n");
		ret = -EFAULT;
		goto __load_internal_err3;
	}
	kvaddr = (void*)lib_ion_mem->addr_k[0];
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, lib_ion_mem, IOMMU_ALL);
	if(unlikely(ret != 0)) {
		pr_err("[ERROR] mem_iommu_map lib_ion_mem failed\n");
		ret = -EFAULT;
		goto __load_internal_err4;
	}
	kpaddr = lib_ion_mem->iova[0];

	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, libinfo_ion_mem);
	if(unlikely(ret != 0)) {
		pr_err("[ERROR]mem_kmap libinfo_ion_mem failed\n");
		ret = -EFAULT;
		goto __load_internal_err5;
	}
	kvaddr_libinfo = (void*)libinfo_ion_mem->addr_k[0];
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, libinfo_ion_mem, IOMMU_ALL);
	if(unlikely(ret != 0)) {
		pr_err("[ERROR]mem_iommu_map libinfo_ion_mem failed\n");
		ret = -EFAULT;
		goto __load_internal_err6;
	}
	kpaddr_libinfo = libinfo_ion_mem->iova[0];
	pr_debug("buffer:%p, kpaddr:%lx, kvaddr:%p, (libinfo)kpaddr:%lx, kvaddr:%p\n",
		buffer, (unsigned long)kpaddr, kvaddr,
		(unsigned long)kpaddr_libinfo, kvaddr_libinfo);

	result = xtlib_host_load_pi_library((xtlib_packaged_library*)buffer, kpaddr,
		(xtlib_pil_info*)kvaddr_libinfo, xt_lib_memcpy, xt_lib_memset, kvaddr);
	if (unlikely(result == 0)){
		/*free ion buffer*/
		pr_err("[ERROR]xtlib_host_load_pi_library failed\n");
		ret = -EFAULT;
		goto __load_internal_err7;
	}

	new_element = (struct loadlib_info*)libinfo_alloc_element();
	if (unlikely(new_element == NULL)) {
		/*free ion buffer*/
		pr_err("[ERROR]libinfo_alloc_element failed\n");
		ret = -ENOMEM;
		goto __load_internal_err7;
	}else {
		snprintf(new_element->libname, XRP_NAMESPACE_ID_SIZE , "%s", libname);
		/*may be change later*/
		new_element->length = size;
		new_element->load_count = 0;
		new_element->ionhandle = lib_ion_mem;
		new_element->ion_phy = (phys_addr_t)kpaddr;
		new_element->ion_kaddr = kvaddr;
		new_element->pil_ionhandle = libinfo_ion_mem;
		new_element->pil_info = kpaddr_libinfo;
		new_element->lib_state = XRP_LIBRARY_LOADING;
		new_element->lib_processing_count = 0;
		new_element->original_flag = 1;
	}
	pr_debug("func:%s add new libinfo xvpfile:%x , libname:%s\n" , __func__ , xvp_file , libname);
	list_add_tail(&new_element->node_libinfo,  &xvp_file->load_lib_list);

	return LIB_RESULT_OK;

__load_internal_err7:
	xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc , libinfo_ion_mem , IOMMU_ALL);
__load_internal_err6:
	xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo_ion_mem);
__load_internal_err5:
	xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, lib_ion_mem, IOMMU_ALL);
__load_internal_err4:
	xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, lib_ion_mem);
__load_internal_err3:
	xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo_ion_mem);
__load_internal_err2:
	xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, lib_ion_mem);
__load_internal_err1:
	vfree(libinfo_ion_mem);
__load_internal_err0:
	vfree(lib_ion_mem);
	return ret;
}

enum load_unload_flag xrp_check_load_unload(struct xvp *xvp , struct xrp_request *rq , uint32_t krqflag)
{
	__u32 indata_size;
	enum load_unload_flag load_flag = XRP_NOT_LOAD_UNLOAD;
	__u8 *tempbuffer = NULL;
	void *tempsrc = NULL;

	indata_size = rq->ioctl_queue.in_data_size;
	if (0 == strcmp(rq->nsid, LIBRARY_LOAD_UNLOAD_NSID)) {
		if (indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
			tempsrc = (void*)(rq->ioctl_queue.in_data_addr);
		else
			tempsrc = rq->in_data;

		if(krqflag == 1) {
			load_flag = XRP_UNLOAD_LIB_FLAG;
		} else {
			tempbuffer = vmalloc(indata_size);
			if(unlikely(copy_from_user(tempbuffer , tempsrc , indata_size))) {
				vfree(tempbuffer);
				return -EFAULT;
			}
			load_flag = *tempbuffer;
			vfree(tempbuffer);
		}
		pr_info("load flag:%d\n" , load_flag);
		return load_flag;
	} else
		return XRP_NOT_LOAD_UNLOAD;
}

int32_t xrp_library_setall_missstate(struct xvp *xvp)
{
	unsigned long bkt;
	struct xrp_known_file *p;
	struct loadlib_info *libinfo;
	struct xvp_file *xvp_file;

	mutex_lock(&xvp->xrp_known_files_lock);
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
		xvp_file = (struct xvp_file *)(((struct file*)(p->filp))->private_data);
		list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
			libinfo->lib_state = XRP_LIBRARY_MISSED_STATE;
		}
        }
	mutex_unlock(&xvp->xrp_known_files_lock);
	return LIB_RESULT_OK;
}

static uint32_t xrp_library_get_loadcount(struct file *filp , const char* libname , uint32_t* curfilecnt, uint32_t *totalcount ) {
	unsigned long bkt;
	struct xrp_known_file *p;
	struct loadlib_info *libinfo;
	struct xvp_file *xvp_file = (struct xvp_file*)(filp->private_data);
        struct xvp *xvp = xvp_file->xvp;
	*curfilecnt = 0;
	*totalcount = 0;

	pr_debug("func:%s enter\n" , __func__);
	mutex_lock(&xvp->xrp_known_files_lock);
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
		pr_debug("func:%s check in new file\n" , __func__);
		xvp_file = (struct xvp_file *)(((struct file*)(p->filp))->private_data);
		list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
			pr_debug("func:%s libname:%s, checked libname:%s , loadcount:%d\n" , __func__ , libinfo->libname , libname , libinfo->load_count);
			if(0 == strcmp(libinfo->libname , libname)) {
				(*totalcount) += libinfo->load_count;
				if(p->filp == filp) {
					pr_debug("func:%s find libname:%s, checked lib:%s\n" , __func__ , libinfo->libname , libname);
					(*curfilecnt) += libinfo->load_count;
				}
			}
		}
	}
	mutex_unlock(&xvp->xrp_known_files_lock);
	pr_debug("func:%s curfilecnt:%d , totalcount:%d , libname:%s\n" , __func__ , *curfilecnt , *totalcount , libname);
	return LIB_RESULT_OK;
}


static int32_t xrp_library_increase(struct file *filp , const char *libname , struct loadlib_info *inlibinfo)
{
	struct loadlib_info *libinfo = NULL;
	struct loadlib_info *newlibinfo = NULL;
	struct xvp_file *xvptemp_file;
	struct xvp_file *xvp_file = (struct xvp_file*)(filp->private_data);
	uint32_t find = 0;
	unsigned long bkt;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_known_file *p;

	list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
		if(0 == strcmp(libname , libinfo->libname)) {
			libinfo->load_count ++;
			find = 1;
			pr_debug("func:%s libname:%s find is 1 ,and only add load_count\n" , __func__ , libinfo->libname);
			break;
		}
	}
	if(1 == find) {
		pr_info("func:%s , loadcount:%d , libname:%s\n" , __func__ , libinfo->load_count , libname);
		return libinfo->load_count;
	}
	mutex_lock(&xvp->xrp_known_files_lock);
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
		find = 0;
                xvptemp_file = (struct xvp_file*)(((struct file*)(p->filp))->private_data);
		if(xvp_file == xvptemp_file) {
	                list_for_each_entry(libinfo , &xvp_file->load_lib_list , node_libinfo) {
				if(0 == strcmp(libname , libinfo->libname)) {
					libinfo->load_count ++;
					find = 1;
					break;
				}
			}
			if((0 == find) && (inlibinfo != NULL)) {
				newlibinfo = libinfo_alloc_element();
				pr_debug("func:%s alloc new element xvpfile:%x , libname:%s\n" , __func__ , xvp_file , inlibinfo->libname);
				if(newlibinfo) {
					find = 1;
					memcpy(newlibinfo , inlibinfo , sizeof(struct loadlib_info));
					newlibinfo->load_count = 1;
					newlibinfo->original_flag = 0;
					newlibinfo->lib_state = XRP_LIBRARY_LOADED;
					newlibinfo->lib_processing_count = 0;
					mutex_init(&newlibinfo->mutex);
					list_add_tail(&newlibinfo->node_libinfo,  &xvp_file->load_lib_list);
				}
			} else {
				pr_err("%s , find is:%d , inlibinfo:%x\n" , __func__ , find , inlibinfo);
			}
		}
        }
	mutex_unlock(&xvp->xrp_known_files_lock);
	if(0 == find) {
		return -EFAULT;
	}
//	pr_info("func:%s , loadcount:%d , libname:%s\n" , __func__ , libinfo->load_count , libname);
	return 0;
}

static uint32_t library_check_otherfile_count(struct file *filp , const char *libname)
{
	struct loadlib_info *libinfo = NULL;
	struct loadlib_info *temp;
	unsigned long bkt;
	uint32_t count = 0;
	struct xrp_known_file *p;
	struct xvp_file *xvp_file;
	struct xvp *xvp = NULL;
	struct xvp_file *xvp_file_curr = (struct xvp_file*)(filp->private_data);
	xvp = xvp_file_curr->xvp;
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
                xvp_file = (struct xvp_file*)(((struct file*)(p->filp))->private_data);
		if(xvp_file_curr != xvp_file) {
			list_for_each_entry_safe(libinfo , temp , &xvp_file->load_lib_list , node_libinfo) {
				if(strcmp(libinfo->libname , libname) == 0) {
					count ++;
				}
                	}
		}
        }
	pr_debug("func:%s count is:%d\n" , __func__ , count);
	return count;
}

int32_t xrp_library_decrelease(struct file* filp , const char *libname)
{
	int32_t find = 0;
	int32_t release = 0;
	struct loadlib_info *libinfo = NULL;
	struct loadlib_info *temp = NULL;
	struct xvp_file *xvp_file = (struct xvp_file*)(filp->private_data);
	struct xvp *xvp = xvp_file->xvp;

	/*decrease load_count*/
	list_for_each_entry_safe(libinfo , temp , &xvp_file->load_lib_list , node_libinfo) {
                if(0 == strcmp(libinfo->libname , libname)) {
                        find = 1;
			if(libinfo->load_count > 0)
                        	libinfo->load_count--;
			if(libinfo->load_count == 0) {
				mutex_lock(&xvp->xrp_known_files_lock);
				if(library_check_otherfile_count(filp , libname) == 0) {
					xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->ionhandle);
					xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->ionhandle , IOMMU_ALL);
					xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->ionhandle);
					vfree(libinfo->ionhandle);
					xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
					xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle, /*IOMMU_MSTD*/IOMMU_ALL);
					xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
					vfree(libinfo->pil_ionhandle);
				}
				mutex_unlock(&xvp->xrp_known_files_lock);
				pr_debug("xrp_library_decrelease libname:%s ok original_flag:%d\n" , libname, libinfo->original_flag);
				/*remove this lib element*/
				list_del(&libinfo->node_libinfo);
				vfree(libinfo);
				release = 1;
			} else {
				pr_debug("xrp_library_decrelease warning libname:%s loadcount:%d\n" , libname , libinfo->load_count);
			}
                        break;
		}
	}
	if(1 == find && 1 == release) {
		return LIB_RESULT_OK;
	} else {
		pr_err("[ERROR]not find lib [%s] or not release, may be some error find:%d, release:%d\n", libname , find , release);
		return LIB_RESULT_ERROR;
	}
}

static int32_t xrp_library_getloadunload_libname(struct xvp *xvp , struct xrp_request *rq , char *outlibname , uint32_t krqflag)
{
	__u32 indata_size;
	int32_t ret = LIB_RESULT_OK;
	void *tempsrc = NULL;
	__u8 *tempbuffer = NULL;
	indata_size = rq->ioctl_queue.in_data_size;

	if(likely(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID))) {
		/*check libname*/
		if (indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
			tempsrc = (void*)(rq->ioctl_queue.in_data_addr);
		else
			tempsrc = (void*)(rq->in_data);
		pr_info("before copy_from_user src:%p , dst:%p\n" , tempsrc , tempbuffer);
		if(krqflag == 1) {
			snprintf(outlibname , XRP_NAMESPACE_ID_SIZE , "%s" , ((char*)(rq->ioctl_queue.in_data_addr) +1));
		} else {
			tempbuffer = vmalloc(indata_size);
			if(unlikely(copy_from_user(tempbuffer , tempsrc , indata_size))) {
				pr_err("[ERROR]copy from user failed\n");
				ret = -EINVAL;
			} else {
				snprintf(outlibname , XRP_NAMESPACE_ID_SIZE , "%s" , tempbuffer+1);
			}
			vfree(tempbuffer);
		}
	} else {
		ret = -EINVAL;
	}
	pr_debug("outlibname:%s, ret:%d\n", outlibname, ret);

	return ret;
}

static int32_t xrp_library_unload_prepare(struct file *filp , struct xrp_request *rq , char *libname , uint32_t krqflag)
{
	int ret = LIB_RESULT_OK;
	__u32 indata_size;
	__u8 *inputbuffer = NULL;
	struct loadlib_info *libinfo = NULL;
	struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;

	pr_info("[UNLOAD][IN]lib unload, nsid[%s]\n" , rq->nsid);
	if(krqflag == 1) {
		pr_info("[UNLOAD][IN]lib unload krqflag is 1, nsid[%s]\n" , rq->nsid);
		return ret;
	}
	indata_size = rq->ioctl_queue.in_data_size;
	if(likely(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID) && (indata_size >= LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE))) {
		libinfo = xrp_library_getlibinfo(filp , libname);
		if(likely(libinfo != NULL)) {
			ret = xvp->vdsp_mem_desc->ops->mem_kmap_userbuf(&rq->ion_in_buf);
			if (unlikely(ret != 0)) {
				pr_err("[ERROR]xrp_library_unload_prepare kmap failed ret:%d\n", ret);
				return -EFAULT;
			}
			inputbuffer = (__u8*)((struct ion_buf*)(&rq->ion_in_buf))->addr_k[0];
			*((unsigned int*)((__u8 *)inputbuffer + LIBRARY_CMD_PIL_INFO_OFFSET)) = libinfo->pil_info;
			xvp->vdsp_mem_desc->ops->mem_kunmap_userbuf(&rq->ion_in_buf);
			wmb();
		}else {
			pr_err("[ERROR]libinfo null\n");
			ret = -EINVAL;
		}
	}else {
		pr_err("[ERROR]nsid is not unload\n");
		ret = -EINVAL;
	}
	pr_debug("unload nsid[%s], ret[%d]\n", rq->nsid, ret);

	return ret;
}

/* return value 0 is need load, 1 is loaded already*/
static int32_t xrp_library_load_prepare(struct file *filp , struct xrp_request *rq , char *outlibname , struct loadlib_info **outlibinfo)
{
	__u32 indata_size;
	__u8 load_flag = 0;
	int32_t ret = LIB_RESULT_OK;
	struct xvp *xvp = NULL;
	uint32_t loaded = 0;
	void *tempsrc = NULL;
	__u8 *tempbuffer = NULL;
	struct loadlib_info *libinfo = NULL;
	__u8 *input_ptr = NULL;
	__u8 *libbuffer = NULL;
	__u8 *inputbuffer = NULL;
	char libname[XRP_NAMESPACE_ID_SIZE];
	*outlibinfo = NULL;
	xvp = ((struct xvp_file*)(filp->private_data))->xvp;
	indata_size = rq->ioctl_queue.in_data_size;
	/*check whether load cmd*/
	if (0 == strcmp(rq->nsid, LIBRARY_LOAD_UNLOAD_NSID)) {
		/*check libname*/
		if (indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
			tempsrc = (void*)(rq->ioctl_queue.in_data_addr);
		else
			tempsrc = (void*)(rq->in_data);

		tempbuffer = vmalloc(indata_size);
		if (unlikely(copy_from_user(tempbuffer, tempsrc, indata_size))) {
			pr_err("[ERROR]copy from user failed\n");
			vfree(tempbuffer);
			return -EFAULT;
		}
		input_ptr = tempbuffer;
		/*input_vir first byte is load or unload*/
		load_flag = *input_ptr;

		if (XRP_LOAD_LIB_FLAG == load_flag) {
			/*load*/
			snprintf(libname, XRP_NAMESPACE_ID_SIZE ,"%s", input_ptr + 1);
			/*check whether loaded*/
			snprintf(outlibname, XRP_NAMESPACE_ID_SIZE , "%s", libname);
			loaded = xrp_check_whether_loaded(filp , libname , outlibinfo);
			if(loaded) {
				pr_info("lib[%s] already loaded, not need reload\n", libname);
				//xrp_library_increase(filp , libname , *outlibinfo);
				ret = 1;/*loaded*/
			}else {
				/*not loaded alloc libinfo node ,load internal*/
				ret = xvp->vdsp_mem_desc->ops->mem_kmap_userbuf(rq->ion_dsp_pool);
				if (ret != 0) {
					pr_err("[ERROR]kmap ionbuf failed\n");
					vfree(tempbuffer);
					return -EFAULT;
				}
				libbuffer = (__u8*)(rq->ion_dsp_pool->addr_k[0]);
				pr_debug("func:%s before xrp_library_load_internal libname:%s\n" , __func__ , libname);
				ret = xrp_library_load_internal(filp , libbuffer , libname);
				if(unlikely(ret != 0)) {
					pr_err("[ERROR]xrp_library_load_internal ret:%d\n",ret);
					//xrp_library_kunmap_ionbuf(dmabuf);
					xvp->vdsp_mem_desc->ops->mem_kunmap_userbuf(rq->ion_dsp_pool);
					vfree(tempbuffer);
					ret = -ENOMEM;
					return ret;
				}
				//xrp_library_kunmap_ionbuf(dmabuf);
				xvp->vdsp_mem_desc->ops->mem_kunmap_userbuf(rq->ion_dsp_pool);
				/*re edit rq for register libname , input data: input[0] load unload flag
				input[1] ~input[32] --- libname , input[LIBRARY_CMD_PIL_INFO_OFFSET]~input[43] ---- libinfo addr*/
				libinfo = xrp_library_getlibinfo(filp , libname);
				if(unlikely(libinfo == NULL)) {
					pr_err("[ERROR]xrp_library_getlibinfo NULL\n");
					xrp_library_decrelease(filp , libname);
					vfree(tempbuffer);
					ret = -ENOMEM;
					return ret;
				}else {
					*((uint32_t*)(input_ptr + LIBRARY_CMD_PIL_INFO_OFFSET)) =
						libinfo->pil_info;
					pr_debug("nsid:%s, loadflag:%d, libname:%s,"
						"pil_info:%x,indata_size:%d\n",
						rq->nsid, load_flag, libname,
						libinfo->pil_info, indata_size);
				}
				ret = xvp->vdsp_mem_desc->ops->mem_kmap_userbuf(&rq->ion_in_buf);
				if (unlikely(ret != 0)) {
					pr_err("[ERROR]kmap ionbuf failed\n");
					vfree(tempbuffer);
					xrp_library_decrelease(filp, libname);
					return -EFAULT;
				}
				inputbuffer = (__u8*) ((struct ion_buf*)(&rq->ion_in_buf))->addr_k[0];
				memcpy(inputbuffer, tempbuffer, indata_size);
				//xrp_library_kunmap_ionbuf(dmabuf);
				xvp->vdsp_mem_desc->ops->mem_kunmap_userbuf(&rq->ion_in_buf);
				wmb();
			}
		}else {
			pr_err("[ERROR]not load flag\n");
			ret = -EINVAL;
		}
		vfree(tempbuffer);
		return ret;
	}else{
		return 0;
	}
}

int32_t xrp_library_release_all(struct xvp *xvp)
{
	struct loadlib_info *libinfo = NULL;
	struct loadlib_info *temp;
	unsigned long bkt;
	struct xrp_known_file *p;
	struct xvp_file *xvp_file;

	mutex_lock(&xvp->xrp_known_files_lock);
	hash_for_each(xvp->xrp_known_files , bkt , p , node) {
		pr_debug("%s p:%lx\n" , __func__ , (unsigned long)p);
		xvp_file = (struct xvp_file*)(((struct file*)(p->filp))->private_data);
		list_for_each_entry_safe(libinfo , temp , &xvp_file->load_lib_list , node_libinfo) {
			pr_info("%s list_for_each_entry libinfo:%lx , libname:%s\n" , __func__ ,(unsigned long)libinfo , libinfo->libname);
			if(likely(NULL != libinfo)) {
				if(library_check_otherfile_count(p->filp , libinfo->libname) == 0) {
					xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->ionhandle);
					xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->ionhandle , IOMMU_ALL);
					xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->ionhandle);
					vfree(libinfo->ionhandle);
					xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
					xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle , IOMMU_ALL);
					xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
					pr_info("%s list_for_each_entry libinfo:%lx before pil_ionhandle:%lx\n" , __func__ ,(unsigned long)libinfo,
						(unsigned long)libinfo->pil_ionhandle);
					vfree(libinfo->pil_ionhandle);
				}
				pr_debug("%s , release ion handle, pil handle current:%p, original_flag:%d\n" , __func__ , get_current() , libinfo->original_flag );
				list_del(&libinfo->node_libinfo);
				vfree(libinfo);
			}
		}
        }
	mutex_unlock(&xvp->xrp_known_files_lock);

	return LIB_RESULT_OK;
}

/*return value 0 is ok, other value is fail or no need process continue*/
int32_t xrp_pre_process_request(struct file *filp , struct xrp_request *rq , enum load_unload_flag loadflag, char *libname , uint32_t krqflag)
{
	int32_t lib_result;
	struct loadlib_info *libinfo = NULL;
	struct xvp* xvp = ((struct xvp_file*)(filp->private_data))->xvp;

	if(loadflag == XRP_LOAD_LIB_FLAG) {
		lib_result = xrp_library_load_prepare(filp , rq , libname , &libinfo);
		if(unlikely(0 != lib_result)) {
			/*has loaded needn't reload*/
			if (unlikely(lib_result != 1)) {
				pr_err("[ERROR]result:%d\n", lib_result);
				return -EFAULT;
			}else {
				pr_warn("[WARN]already loaded needn't reload\n");
				/*increase*/
				xrp_library_increase(filp , libname , libinfo);
				return -EEXIST;
			}
		}else {
			/*re-edit the rq for register*/
			pr_debug("Load libname:%s\n", libname);
			return LIB_RESULT_OK;
		}
	} else if(loadflag == XRP_UNLOAD_LIB_FLAG) {
		lib_result = xrp_library_getloadunload_libname(xvp , rq , libname , krqflag);
		if(likely(lib_result == 0)) {
			uint32_t curfilecnt, totalcount;

			xrp_library_get_loadcount(filp ,libname , &curfilecnt, &totalcount);
			if(curfilecnt == 1) {
				if(totalcount == 1) {
					if (0 != xrp_library_checkprocessing(filp , libname)) {
						pr_err("[ERROR]the same lib is processing invalid operation\n");
						return -EINVAL;
					}
					/*if need unload may be modify libinfo addr, only follow the default send cmd*/
					lib_result = xrp_library_unload_prepare(filp , rq , libname , krqflag);
					if(unlikely(lib_result != 0)) {
						pr_err("[ERROR]xrp_library_unload failed:%d\n", lib_result);
						return -EINVAL;
					}
					libinfo = xrp_library_getlibinfo(filp, libname);
					if (likely(libinfo != NULL))
						 libinfo->lib_state = XRP_LIBRARY_UNLOADING;
					pr_debug("Unload libname:%s\n", libname);
					return LIB_RESULT_OK;
				} else {
					pr_debug("curfile cnt is:%d total cnt:%d needn't unload\n" , curfilecnt , totalcount);
					xrp_library_decrelease(filp , libname);
					return -EEXIST;
				}
			} else if(curfilecnt > 1) {
				pr_debug("curfile cnt is:%d needn't unload\n" , curfilecnt);
				xrp_library_decrelease(filp , libname);
				return -EEXIST;
			} else {
				pr_err("cur file load count is:%d, total count:%d ,libname:%s curfilecnt is abnormal\n" , curfilecnt , totalcount , libname);
				return -ENXIO;
			}
		}else {
			pr_err("[ERROR]get libname error, libname:%s\n", libname);
			return -EINVAL;
		}
	}else {
		libinfo = xrp_library_getlibinfo(filp , rq->nsid);
		if(libinfo != NULL) {
			mutex_lock(&libinfo->mutex);
			if((libinfo->lib_state != XRP_LIBRARY_LOADED) && (libinfo->lib_state != XRP_LIBRARY_PROCESSING_CMD)) {
				pr_err("[ERROR]lib:%s , libstate is:%d not XRP_LIBRARY_LOADED , so return inval\n", rq->nsid , libinfo->lib_state);
				return -EINVAL;
			}
			pr_debug("func:%s , lib_processing_count:%d\n" , __func__, libinfo->lib_processing_count +1);
			/*set processing libname*/
			libinfo->lib_processing_count++;
			libinfo->lib_state = XRP_LIBRARY_PROCESSING_CMD;
			mutex_unlock(&libinfo->mutex);
		}else {
			pr_err("libinfo null\n");
		}
		/*check whether libname unloading state, if unloading return*/
		pr_debug("Command libname:%s\n", rq->nsid);
		return LIB_RESULT_OK;
	}
}

int post_process_request(struct file *filp , struct xrp_request *rq , const char* libname , enum load_unload_flag load_flag , int32_t resultflag)
{
	struct loadlib_info *libinfo = NULL;
	int32_t ret = 0;

	pr_info("[IN]load_flag[%d], resultflag[%d]\n", load_flag, resultflag);
	if(load_flag == XRP_LOAD_LIB_FLAG) {
		if(likely(resultflag == 0)){
			xrp_library_increase(filp , libname , NULL);
			libinfo = xrp_library_getlibinfo(filp , libname);
			if(likely(libinfo != NULL)) {
				libinfo->lib_state = XRP_LIBRARY_LOADED;
				pr_info("libname:%s, libstate XRP_LIBRARY_LOADED\n" , libname);
			}
		} else {
			/*load failedd release here*/
			xrp_library_decrelease(filp , libname);
			pr_err("[ERROR]libname:%s, load failed xrp_library_decrelease\n", libname);
			ret = -EFAULT;
		}
	} else if(load_flag == XRP_UNLOAD_LIB_FLAG) {
		if(likely(resultflag == 0)) {
			libinfo = xrp_library_getlibinfo(filp , libname);
			if(likely(libinfo != NULL)) {
				libinfo->lib_state = XRP_LIBRARY_IDLE;
			}
			pr_info("libname:%s, libstate XRP_LIBRARY_IDLE libinfo:%p\n", libname , libinfo);
			xrp_library_decrelease(filp , libname);
		} else {
//			xrp_library_increase(filp , libname , NULL);
			pr_err("[ERROR]libname:%s, unload failed\n", libname);
			ret = -EFAULT;
		}
	}else {
		/*remove processing lib*/
		libinfo = xrp_library_getlibinfo(filp , rq->nsid);
		if(libinfo != NULL) {
			mutex_lock(&libinfo->mutex);
			libinfo->lib_processing_count --;
			pr_debug("--------func:%s post processing count:%d\n" , __func__ , libinfo->lib_processing_count);
			if(libinfo->lib_state != XRP_LIBRARY_PROCESSING_CMD) {
				pr_err("[ERROR]lib:%s processing cmd , but not XRP_LIBRARY_PROCESSING_CMD state\n", rq->nsid);
				ret = -EINVAL;
			}
			/*set processing libname*/
			if(libinfo->lib_processing_count == 0)
				libinfo->lib_state = XRP_LIBRARY_LOADED;
			mutex_unlock(&libinfo->mutex);
		}
		/*set processing lib state*/
		pr_debug("lib:%s, process cmd over\n", rq->nsid);
	}

	return ret;
}

int32_t xrp_create_unload_cmd(struct file *filp , struct loadlib_info *libinfo , struct xrp_unload_cmdinfo *info) {
        struct xrp_request *rq;
        struct ion_buf *lib_input_ionmem;
        int ret;
        char *kvaddr;
        const char *libname = libinfo->libname;
        struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;

        rq = vmalloc(sizeof(*rq));
	if(unlikely(rq == NULL)) {
		goto rq_failed;
	}
        lib_input_ionmem = vmalloc(sizeof(struct ion_buf));
        if(unlikely(lib_input_ionmem == NULL)) {
                goto ion_buf_failed;
        }
        ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
                                                lib_input_ionmem,
                                                ION_HEAP_ID_MASK_SYSTEM,
                                                LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE);
        if(unlikely(ret != 0)) {
                goto alloc_lib_input_ionmem_failed;
        }
        lib_input_ionmem->dev = xvp->dev;
        /*alloc libinfo ion buffer*/
        ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, lib_input_ionmem);
	if(unlikely(ret != 0)) {
		goto kmap_failed;
	}
        kvaddr = (void*)lib_input_ionmem->addr_k[0];
        ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, lib_input_ionmem , IOMMU_ALL);
	if(unlikely(ret != 0)) {
		goto iommu_map_failed;
	}
	memset(rq , 0 , sizeof(*rq));
        /*nsid to load/unload nsid*/
        sprintf(rq->nsid , "%s" , LIBRARY_LOAD_UNLOAD_NSID);
        kvaddr[0] = XRP_UNLOAD_LIB_FLAG; /*unload*/
        strncpy(kvaddr+1 , libname , 33); /*libname*/
        *((uint32_t*)(kvaddr+40)) = libinfo->pil_info;
	info->input_kaddr = kvaddr;
	rq->ioctl_queue.flags = (1 ? XRP_QUEUE_FLAG_NSID : 0) |
                                ((2 << XRP_QUEUE_FLAG_PRIO_SHIFT) &
                                 XRP_QUEUE_FLAG_PRIO);
        rq->ioctl_queue.in_data_size = LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE;
        rq->ioctl_queue.in_data_addr = (unsigned long)kvaddr;
	rq->in_data_phys = lib_input_ionmem->iova[0];
        info->rq = rq;
        info->input_ion_handle = lib_input_ionmem;
	pr_debug("func:%s create unload cmd\n" , __func__);
        return LIB_RESULT_OK;
iommu_map_failed:
	xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, lib_input_ionmem , IOMMU_ALL);
kmap_failed:
	xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc , lib_input_ionmem);
alloc_lib_input_ionmem_failed:
	vfree(lib_input_ionmem);
ion_buf_failed:
	vfree(rq);
rq_failed:
	return -EFAULT;
}

int32_t xrp_free_unload_cmd(struct file *filp , struct xrp_unload_cmdinfo *info)
{
        struct ion_buf *lib_input_ionmem;
        int ret;
        struct xvp *xvp = ((struct xvp_file*)(filp->private_data))->xvp;

        lib_input_ionmem = info->input_ion_handle;
	xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, lib_input_ionmem);
	xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, lib_input_ionmem , IOMMU_ALL);
        ret = xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc,
                                                lib_input_ionmem);
        vfree(info->rq);
        vfree(info->input_ion_handle);
	pr_debug("func:%s free unload cmd\n" , __func__);
        return LIB_RESULT_OK;
}
