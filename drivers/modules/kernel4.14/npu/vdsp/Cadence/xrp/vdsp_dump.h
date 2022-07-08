#ifndef __VDSP_DUMP_FILE__
#define __VDSP_DUMP_FILE__

int32_t xrp_save_file(const char* filename, const char* buffer , uint32_t size);
int32_t xrp_dump_libraries(struct xvp *xvp);
#endif
