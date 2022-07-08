/*
 * xrp_firmware: firmware manipulation for the XRP
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

#ifndef XRP_FACEID_H
#define XRP_FACEID_H

#define VDSP_FACEID_FIRMWIRE_SIZE    (1024*1024*10)

#define FACEID_FD_MEM_SIZE (1024*1024*16)
#define FACEID_FIRMWARE "faceid_fw.bin"
#define FACEID_COEFF_NUM (6)

struct xvp;
/*sync with vdsp side*/
struct faceid_hw_sync_data
{
	__u32 fd_p_coffe_addr;
	__u32 fd_r_coffe_addr;
	__u32 fd_o_coffe_addr;
	__u32 fp_coffe_addr;
	__u32 flv_coffe_addr;
	__u32 fv_coffe_addr;
	__u32 mem_pool_addr;
	__u32 in_addr;
	__u32 out_addr;
	__u32 transfer_addr;/*face landmark point*/
};

typedef struct
{
	uint32_t width, height;
	uint32_t phyaddr;		/*image phyaddr*/
	uint32_t liveness;		/*0:off 1:faceid_single 2:faceid_3D 3:pay_3D*/
	uint32_t l_ir_phyaddr;	/*Left IR phyaddr*/
	uint32_t r_ir_phyaddr;	/*Right IR phyaddr*/
	uint32_t bgr_phyaddr;	/*bgr phyaddr*/
	uint32_t otp_phyaddr;	/*otp phyaddr*/
}FACEID_IN;

typedef struct
{
	int x, y;
}FV_POINT;

typedef struct
{
	FV_POINT landmarks[7];
}FV_FAECINFO;

enum IRQ_STATUS
{
	IRQ_STATUS_REQUESTED = 0,
	IRQ_STATUS_FREED ,
};

int sprd_faceid_init(struct xvp *xvp);
int sprd_faceid_deinit(struct xvp *xvp);

int sprd_faceid_secboot_init(struct xvp *xvp);
int sprd_faceid_secboot_deinit(struct xvp *xvp);
int sprd_faceid_secboot_entry(struct xvp *xvp);
int sprd_faceid_secboot_exit(struct xvp *xvp);

int sprd_faceid_sec_sign(struct xvp *xvp);
int sprd_faceid_load_firmware(struct xvp *xvp);

int sprd_faceid_sync_vdsp(struct xvp *xvp);
int sprd_faceid_halt_vdsp(struct xvp *xvp);
int sprd_faceid_reset_vdsp(struct xvp *xvp);
int sprd_faceid_release_vdsp(struct xvp *xvp);
int sprd_faceid_enable_vdsp(struct xvp *xvp);
int sprd_faceid_disable_vdsp(struct xvp *xvp);
int sprd_faceid_run_vdsp(struct xvp *xvp, uint32_t in_fd, uint32_t out_fd);

int sprd_iommu_map_faceid_fwbuffer(struct xvp *xvp);
int sprd_iommu_unmap_faceid_fwbuffer(struct xvp *xvp);


#endif
