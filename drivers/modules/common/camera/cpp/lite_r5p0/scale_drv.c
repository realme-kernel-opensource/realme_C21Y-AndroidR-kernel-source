/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>
#include "sprd_cpp.h"

#include "cpp_common.h"
#include "cpp_reg.h"
#include "cpp_core.h"
#include "scale_drv.h"

#define PATH0_ADDR_ALIGN						0x07
#define SCALE_LOWEST_ADDR						0x800
#define SCALE_ADDR_INVALID(addr) \
	((unsigned long)(addr) < SCALE_LOWEST_ADDR)
#define SCALE_YUV_ADDR_INVALID(y, u, v) \
	(SCALE_ADDR_INVALID(y) && \
	SCALE_ADDR_INVALID(u) && \
	SCALE_ADDR_INVALID(v))

#define SCALE_FRAME_WIDTH_MAX					8192
#define SCALE_FRAME_HEIGHT_MAX					8192
#define SCALE_FRAME_WIDTH_MIN					64
#define SCALE_FRAME_HEIGHT_MIN					32
#define BP_TRIM_SIZE_MIN						32
#define BP_TRIM_SIZE_MAX						8192
#define SCALE_WIDTH_MIN							64
#define SCALE_HEIGHT_MIN						32
#define SCALE_SLICE_OUT_WIDTH_MAX				2048
#define SCALE_FRAME_OUT_WIDTH_MAX				768
#define SCALE_SC_COEFF_MAX						4
#define SCALE_SC_COEFF_MID						4
#define SCALE_DECI_FAC_MAX						8
#define SCALE_PIXEL_ALIGNED						4
#define NORMAL_CLK_DOMAIN						0
#define COEFF_CLK_DOMAIN						1
#define ALIGNED_DOWN_2(w)		((w) & ~(2 - 1))
#define ALIGNED_DOWN_4(w)		((w) & ~(4 - 1))
#define ALIGNED_DOWN_8(w)		((w) & ~(8 - 1))
#define ALIGNED_DOWN_16(w)		((w) & ~(16 - 1))
#define SCALE_WR_COEFF(addr, value)	\
	CPP_REG_WR(addr, value & 0x3ffff)

#define MOD(x, a) (x % a)
#define MOD2(x)	(x % 2)
#define OSIDE(x, a, b) ((x < a) || (x > b))
#define CMP(x, a, b) (x < (a + b))

#ifdef CPP_DEBUG
#undef CPP_DEBUG
#define CPP_DEBUG
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "SCALE_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

void sprd_scaledrv_stop(
	struct scale_drv_private *p)
{
	unsigned long flags = 0;

	spin_lock_irqsave(p->hw_lock, flags);
	CPP_REG_AWR(CPP_PATH_START, (~CPP_SCALE_START_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

void sprd_scaledrv_start(
	struct scale_drv_private *p)
{
	unsigned long flags = 0;

	spin_lock_irqsave(p->hw_lock, flags);
	CPP_REG_OWR(CPP_PATH_START, CPP_SCALE_START_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

int sprd_scaledrv_slice_param_check(
	struct scale_drv_private *p)
{
	/* pitch check */
	pr_debug("srcpitch%d,rect%d,%d,%d,%d,infmt%d,inend%d,%d,scdespi%d\n",
	p->src_pitch, p->src_rect.x, p->src_rect.y,
	p->src_rect.w, p->src_rect.h, p->input_fmt,
	p->input_endian, p->input_uv_endian, p->sc_des_pitch);
	pr_debug("scdesrect%d,%d,%d,%d,bpp%d,rec%d,%d,%d,%d,hor%dver%d,ofmt%d\n",
	p->sc_des_rect.x, p->sc_des_rect.y, p->sc_des_rect.w,
	p->sc_des_rect.h, p->bp_des_pitch, p->bp_des_rect.x,
	p->bp_des_rect.y, p->bp_des_rect.w, p->bp_des_rect.h,
	p->hor_deci, p->ver_deci, p->sc_output_fmt);
	if (p->src_pitch > SCALE_FRAME_WIDTH_MAX ||
		MOD(p->src_pitch, 8) != 0 ||
		CMP(p->src_pitch, p->src_rect.w, p->src_rect.x) ||
		OSIDE(p->src_pitch, SCALE_FRAME_WIDTH_MIN,
		SCALE_FRAME_WIDTH_MAX)) {
		pr_err("fail to get valid src pitch %d\n",
			p->src_pitch);
		return -1;
	}
	if (MOD(p->sc_des_pitch, 8) != 0 ||
		OSIDE(p->sc_des_pitch, SCALE_FRAME_WIDTH_MIN,
		SCALE_FRAME_HEIGHT_MAX) ||
		CMP(p->sc_des_pitch, p->sc_des_rect.w, p->sc_des_rect.x)) {
		pr_err("fail to get valid sc des pitch %d\n",
			p->sc_des_pitch);
		return -1;
	}
	if ((p->bp_en == 1) && (MOD(p->bp_des_pitch, 8) != 0 ||
		OSIDE(p->bp_des_pitch, BP_TRIM_SIZE_MIN, BP_TRIM_SIZE_MAX) ||
		CMP(p->bp_des_pitch, p->bp_des_rect.w, p->bp_des_rect.x))) {
		pr_err("fail to get valid bp des pitch %d\n",
			p->bp_des_pitch);
		return -1;
	}
	/* input rect check */
	if (OSIDE(p->src_rect.w, SCALE_FRAME_WIDTH_MIN,
		SCALE_FRAME_WIDTH_MAX) ||
		OSIDE(p->src_rect.h, SCALE_FRAME_HEIGHT_MIN,
		SCALE_FRAME_HEIGHT_MAX)) {
		pr_err("input rect.wh check failed\n");
		return -1;
	}
	if (p->src_rect.w % (2 << p->hor_deci) != 0) {
		pr_err("fail to get valid src_rect.w %d\n",
			p->src_rect.w);
		return -1;
	}
	if ((p->input_fmt == SCALE_YUV420) &&
		(p->src_rect.h % (2 << p->ver_deci) != 0)) {
		pr_err("fail to get valid src_rect.h %d\n",
			p->src_rect.h);
		return -1;
	}
	if ((p->input_fmt == SCALE_YUV422) &&
		(p->src_rect.h % (1 << p->ver_deci) != 0)) {
		pr_err("fail to get valid src_rect.h %d\n",
			p->src_rect.h);
		return -1;
	}
	if (MOD2(p->src_rect.x) != 0) {
		pr_err("src_rect.x not align failed\n");
		return -1;
	}
	if (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->src_rect.y) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->src_rect.y, 1) != 0)) {
		pr_err("src_rect.y align failed\n");
		return -1;
	}

	/* output rect */
	if ((MOD2(p->sc_des_rect.w) != 0) ||
		OSIDE(p->sc_des_rect.w, SCALE_WIDTH_MIN,
		SCALE_SLICE_OUT_WIDTH_MAX) ||
		OSIDE(p->sc_des_rect.h, SCALE_HEIGHT_MIN,
		SCALE_FRAME_WIDTH_MAX)) {
		pr_err("sc_des_rect.w out size failed\n");
		return -1;
	}
	if (((p->sc_output_fmt == SCALE_YUV420) &&
		MOD2(p->sc_des_rect.h) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->sc_des_rect.h, 1) != 0)) {
		pr_err("sc_des_rect.h align failed\n");
		return -1;
	}

	if (MOD2(p->sc_des_rect.x) != 0) {
		pr_err("sc_des_rect align failed\n");
		return -1;
	}
	if (((p->sc_output_fmt == SCALE_YUV420) &&
		MOD2(p->sc_des_rect.y) != 0) ||
		((p->sc_output_fmt == SCALE_YUV422) &&
		MOD(p->sc_des_rect.y, 1) != 0)) {
		pr_err("sc_des_rect.y align failed\n");
		return -1;
	}
	if ((p->bp_en == 1) && (MOD2(p->bp_des_rect.w) != 0 ||
		OSIDE(p->bp_des_rect.w, BP_TRIM_SIZE_MIN,
		SCALE_FRAME_HEIGHT_MAX) ||
		OSIDE(p->bp_des_rect.h, BP_TRIM_SIZE_MIN,
		SCALE_FRAME_HEIGHT_MAX))) {
		pr_err("bp_des_rect.wh outsize failed\n");
		return -1;
	}
	if ((p->bp_en == 1) &&
			(((p->input_fmt == SCALE_YUV420) &&
			  MOD2(p->bp_des_rect.h) != 0) ||
			 ((p->input_fmt == SCALE_YUV422) &&
			  MOD(p->bp_des_rect.h, 1) != 0))) {
		pr_err("bp_des_rect.h align failed\n");
		return -1;
	}
	if ((p->bp_en == 1) && (MOD2(p->bp_des_rect.x) != 0 ||
				OSIDE(p->bp_des_rect.x, 0,
					SCALE_FRAME_HEIGHT_MAX) ||
				OSIDE(p->bp_des_rect.y, 0,
					SCALE_FRAME_HEIGHT_MAX))) {
		pr_err("bp_des_rect.xy outsize failed\n");
		return -1;
	}
	if ((p->bp_en == 1) && (((p->input_fmt == SCALE_YUV420) &&
					MOD2(p->bp_des_rect.y) != 0) ||
				((p->input_fmt == SCALE_YUV422) &&
				 MOD(p->bp_des_rect.y, 1) != 0))) {
		pr_err("bp_des_rect.y align failed\n");
		return -1;
	}
	/* in trim */
	if (MOD2(p->sc_intrim_rect.w) != 0 ||
		p->sc_intrim_rect.w < SCALE_WIDTH_MIN ||
		p->sc_intrim_rect.h < SCALE_HEIGHT_MIN) {
		pr_err("sc_intrim_rect.wh outsize failed\n");
		return -1;
	}
	if (CMP((p->src_rect.w >> p->hor_deci), p->sc_intrim_rect.w,
		p->sc_intrim_rect.x) ||
		CMP((p->src_rect.h >> p->ver_deci), p->sc_intrim_rect.h,
		p->sc_intrim_rect.y)) {
		pr_err("sc_intrim_rect.wh outsize with deci failed\n");
		return -1;
	}
	if (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->sc_intrim_rect.h) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->sc_intrim_rect.h, 1) != 0)) {
		pr_err("sc_intrim_rect.h align failed\n");
		return -1;
	}
	if (MOD2(p->sc_intrim_rect.x) != 0) {
		pr_err("sc_intrim_rect.x align failed\n");
		return -1;
	}
	if (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->sc_intrim_rect.y) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->sc_intrim_rect.y, 1) != 0)) {
		pr_err("sc_intrim_rect.y align failed\n");
		return -1;
	}
	/* slice in */
	if (MOD2(p->sc_slice_in_size.w) != 0) {
		pr_err("sc_slice_in_size.w align failed\n");
		return -1;
	}
	if (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->sc_slice_in_size.h) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->sc_slice_in_size.h, 1) != 0)) {
		pr_err("sc_slice_in_size.h align failed\n");
		return -1;
	}
	/* slice out */
	if (MOD2(p->sc_slice_out_size.w) != 0) {
		pr_err("sc_slice_out_size.w align failed\n");
		return -1;
	}
	if (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->sc_slice_out_size.h) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->sc_slice_out_size.h, 1) != 0)) {
		pr_err("sc_slice_out_size.h align failed\n");
		return -1;
	}
	/* sc out trim */
	if (MOD2(p->sc_outtrim_rect.w) != 0) {
		pr_err("sc_outtrim_rect.w align failed\n");
		return -1;
	}
	if (((p->sc_output_fmt == SCALE_YUV420) &&
		MOD2(p->sc_outtrim_rect.h) != 0) ||
		((p->sc_output_fmt == SCALE_YUV422) &&
		MOD(p->sc_outtrim_rect.h, 1) != 0)) {
		pr_err("sc_outtrim_rect.h align failed\n");
		return -1;
	}
	if (MOD2(p->sc_intrim_rect.x) != 0) {
		pr_err("sc_intrim_rect.x align failed\n");
		return -1;
	}
	if (((p->sc_output_fmt == SCALE_YUV420) &&
		MOD2(p->sc_outtrim_rect.y) != 0) ||
		((p->sc_output_fmt == SCALE_YUV422) &&
		MOD(p->sc_outtrim_rect.y, 1) != 0)) {
		pr_err("sc_outtrim_rect.y align failed\n");
		return -1;
	}
	/* bp trim */
	if ((p->bp_en == 1) && (MOD2(p->bp_trim_rect.w) != 0)) {
		pr_err("bp_trim_rect.w align failed\n");
		return -1;
	}
	if ((p->bp_en == 1) && (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->bp_trim_rect.h) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->bp_trim_rect.h, 1) != 0))) {
		pr_err("bp_trim_rect.h align failed\n");
		return -1;
	}
	if ((p->bp_en == 1) && (MOD2(p->bp_trim_rect.x) != 0)) {
		pr_err("bp_trim_rect.x align failed\n");
		return -1;
	}
	if ((p->bp_en == 1) && (((p->input_fmt == SCALE_YUV420) &&
		MOD2(p->bp_trim_rect.y) != 0) ||
		((p->input_fmt == SCALE_YUV422) &&
		MOD(p->bp_trim_rect.y, 1) != 0))) {
		pr_err("bp_trim_rect.y align failed\n");
		return -1;
	}

	return 0;
}
static void scale_scaledrv_coeff_set(struct scale_drv_private *p,
		struct sprd_cpp_scale_cfg_parm *sc_cfg)
{
	slice_drv_scaler_coef_t *drv_scaler_coef = NULL;
	int i = 0;
	int j = 0;
	unsigned long reg_addr = CPP_BASE;
	unsigned long reg_luma_h_coeff = CPP_BASE;
	unsigned long reg_chrima_h_coeff = CPP_BASE;
	unsigned long reg_v_coeff = CPP_BASE;
	unsigned int sc_slice_in_height_y = 0;
	unsigned int sc_slice_out_height_y = 0;
	unsigned int sc_slice_out_height_uv = 0;
	unsigned int sc_slice_in_height_uv = 0;
	int vcoef_reorder[144];
	int tmp = 0;
	int luma_hcoeff[32];
	int chroma_hcoeff[16];
	int vcoeff[144];
	int luma_hor[64] __aligned(16);
	int chroma_hor[32] __aligned(16);

	drv_scaler_coef = &sc_cfg->slice_param.output.scaler_path_coef;

	pr_debug("%s entry\n", __func__);
	 reg_luma_h_coeff	+= PATH0_LUMA_H_COEFF_BASE_ADDR;
	 reg_chrima_h_coeff += PATH0_CHROMA_H_COEFF_BASE_ADDR;
	 reg_v_coeff		+= PATH0_V_COEFF_BASE_ADDR;

	memset(&luma_hcoeff, 0, sizeof(luma_hcoeff));
	memset(&chroma_hcoeff, 0, sizeof(chroma_hcoeff));
	memset(&vcoeff, 0, sizeof(vcoeff));
	memset(&vcoef_reorder, 0, sizeof(vcoef_reorder));

	CPP_REG_MWR(CPP_PATH0_CFG0, CPP_PATH0_CLK_SWITCH,
			CPP_PATH0_CLK_SWITCH);

	/*  handle luma her coeff */
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 8; j++)
			luma_hor[i * 8 + j] =
				drv_scaler_coef->y_hor_coef[i][j];
	}
	for (i = 0; i < 32; i++)
		luma_hcoeff[i] =
			((luma_hor[2 * i] & 0x000001ff) << 9) |
			(luma_hor[2*i+1]&0x000001ff);
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4; j++) {
			reg_addr = reg_luma_h_coeff + 4 * (4 * i + j);
			SCALE_WR_COEFF(reg_addr, luma_hcoeff[4 * i + 3 - j]);
		}
	}
	/* handle chroma her coeff */
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4; j++)
			chroma_hor[i * 4 + j] =
				drv_scaler_coef->c_hor_coef[i][j];
	}
	for (i = 0; i < 16; i++) {
		chroma_hcoeff[i] = ((chroma_hor[2*i]&0x000001ff)<<9) |
			(chroma_hor[2*i+1]&0x000001ff);
	}
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 2; j++) {
			reg_addr = reg_chrima_h_coeff + 4 * (2 * i + j);
			SCALE_WR_COEFF(reg_addr, chroma_hcoeff[2 * i + 1 - j]);
		}
	}

	/* handle ver coeff */
	sc_slice_in_height_y = p->sc_slice_in_size.h;
	sc_slice_out_height_y = p->sc_slice_out_size.h;

	if (p->input_fmt == 0)
		sc_slice_in_height_uv = p->sc_slice_in_size.h >> 1;
	else if (p->input_fmt == 2)
		sc_slice_in_height_uv = p->sc_slice_in_size.h;

	if (p->sc_output_fmt == 0)
		sc_slice_out_height_uv = p->sc_slice_out_size.h >> 1;
	else if (p->sc_output_fmt == 2)
		sc_slice_out_height_uv = p->sc_slice_out_size.h;

	for (i = 0; i < 9; i++) {
		for (j = 0; j < 16; j++) {
			tmp = drv_scaler_coef->c_ver_coef[i][j];
			vcoeff[i * 16 + j] = ((tmp&0x000001ff)<<9) |
				(drv_scaler_coef->y_ver_coef[i][j]&0x000001ff);
		}
	}

	for (i = 0; i < 8; i++) {
		if (sc_slice_out_height_y*2 > sc_slice_in_height_y) {
			for (j = 0; j < 4; j++)
				vcoef_reorder[i*4+j] =
					vcoeff[i*16+j]&0x000001ff;
		} else {
			for (j = 0; j < 16; j++)
				vcoef_reorder[i*16+j] =
					vcoeff[i*16+j]&0x000001ff;
		}
	}

	for (i = 0; i < 8; i++) {
		if (sc_slice_out_height_uv * 2 > sc_slice_in_height_uv) {
			for (j = 0; j < 4; j++) {
				vcoef_reorder[i * 4 + j] |=
					(vcoeff[i*16+j]&0x0003fe00);
			}
		} else {
			for (j = 0; j < 16; j++)
				vcoef_reorder[i * 16 + j] |=
					(vcoeff[i*16+j]&0x0003fe00);
		}
	}

	for (i = 0; i < 16; i++)
		vcoef_reorder[128 + i] = vcoeff[128+i];

	for (i = 0; i < 132; i++) {
		reg_addr = reg_v_coeff + 4 * i;
		SCALE_WR_COEFF(reg_addr, vcoef_reorder[i]);
	}

	CPP_REG_AWR(CPP_PATH0_CFG0, ~CPP_PATH0_CLK_SWITCH);
	pr_debug("set scale coeff over\n");
}


static int sprd_scaledrv_src_pitch_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	CPP_REG_MWR(CPP_PATH0_CFG4, CPP_PATH0_SRC_PITCH,
		p->src_pitch);

	return 0;
}

static int sprd_scaledrv_des_pitch_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	CPP_REG_MWR(CPP_PATH0_CFG5, CPP_PATH0_BP_DES_PITCH,
		p->bp_des_pitch << 16);
	CPP_REG_MWR(CPP_PATH0_CFG5, CPP_PATH0_SC_DES_PITCH,
		p->sc_des_pitch);

	return 0;
}

static int sprd_scaledrv_input_rect_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	CPP_REG_MWR(CPP_PATH0_CFG1, CPP_PATH0_SRC_WIDTH_MASK,
		p->src_rect.w);
	CPP_REG_MWR(CPP_PATH0_CFG1, CPP_PATH0_SRC_HEIGHT_MASK,
		p->src_rect.h << 16);
	CPP_REG_MWR(CPP_PATH0_CFG6, CPP_PATH0_SRC_OFFSET_X_MASK,
		p->src_rect.x << 16);
	CPP_REG_MWR(CPP_PATH0_CFG6, CPP_PATH0_SRC_OFFSET_Y_MASK,
		p->src_rect.y);

	return 0;
}

static int sprd_scaledrv_output_rect_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	/* SC_DES_SIZE */
	CPP_REG_MWR(CPP_PATH0_CFG2, CPP_PATH0_SC_DES_WIDTH_MASK,
			p->sc_des_rect.w);
	CPP_REG_MWR(CPP_PATH0_CFG2, CPP_PATH0_SC_DES_HEIGHT_MASK,
			p->sc_des_rect.h << 16);
	/*SC_DES_OFFSET */
	CPP_REG_MWR(CPP_PATH0_CFG7, CPP_PATH0_SC_DES_OFFSET_X_MASK,
			p->sc_des_rect.x << 16);
	CPP_REG_MWR(CPP_PATH0_CFG7, CPP_PATH0_SC_DES_OFFSET_Y_MASK,
			p->sc_des_rect.y);
	/* BP_DES_SIZE */
	CPP_REG_MWR(CPP_PATH0_CFG3, CPP_PATH0_BP_DES_WIDTH_MASK,
			p->bp_des_rect.w);
	CPP_REG_MWR(CPP_PATH0_CFG3, CPP_PATH0_BP_DES_HEIGHT_MASK,
			p->bp_des_rect.h << 16);
	/* BP_DES_OFFSET */
	CPP_REG_MWR(CPP_PATH0_CFG8, CPP_PATH0_BP_DES_OFFSET_X_MASK,
			p->bp_des_rect.x << 16);
	CPP_REG_MWR(CPP_PATH0_CFG8, CPP_PATH0_BP_DES_OFFSET_Y_MASK,
			p->bp_des_rect.y);

	return 0;
}
static int sprd_scaledrv_input_format_set(
	struct scale_drv_private *p)
{

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_PATH0_CFG0, CPP_PATH0_INPUT_FORMAT,
		p->input_fmt);

	return 0;
}

static int sprd_scaledrv_output_format_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid output format ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_PATH0_CFG0, CPP_PATH0_OUTPUT_FORMAT,
		(p->sc_output_fmt << 2));

	return 0;
}

static int sprd_scaledrv_deci_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_PATH0_CFG0, CPP_PATH0_HOR_DECI,
		(p->hor_deci << 4));
	CPP_REG_MWR(CPP_PATH0_CFG0, CPP_PATH0_VER_DECI,
		(p->ver_deci << 6));

	return 0;
}

static int sprd_scaledrv_input_endian_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_AXIM_CHN_SET, CPP_PATH0_IN_ENDIAN,
		p->input_endian);
	CPP_REG_MWR(CPP_AXIM_CHN_SET, CPP_PATH0_IN_UV_ENDIAN,
		p->input_uv_endian << 3);

	return 0;
}

static int sprd_scaledrv_output_endian_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_AXIM_CHN_SET, CPP_PATH0_OUT_ENDIAN,
		p->output_endian << 4);
	CPP_REG_MWR(CPP_AXIM_CHN_SET, CPP_PATH0_OUT_UV_ENDIAN,
		p->output_uv_endian << 7);

	return 0;
}

static int sprd_scaledrv_burst_gap_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_AXIM_BURST_GAP_PATH0, CPP_PATH0_RCH_BURST_GAP,
		p->rch_burst_gap << 16);
	CPP_REG_MWR(CPP_AXIM_BURST_GAP_PATH0, CPP_PATH0_WCH_BURST_GAP,
		p->wch_burst_gap);

	return 0;
}

static int sprd_scaledrv_bpen_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_PATH0_CFG0, CPP_PATH0_BP_EB,
		p->bp_en << 8);

	return 0;
}

static int sprd_scaledrv_ini_phase_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_PATH0_SC_Y_HOR_INI_PHASE,
		CPP_PATH0_SC_Y_HOR_INI_PHASE_INT,
		p->y_hor_ini_phase_int << 16);
	CPP_REG_MWR(CPP_PATH0_SC_Y_HOR_INI_PHASE,
		CPP_PATH0_SC_Y_HOR_INI_PHASE_FRAC,
		p->y_hor_ini_phase_frac);
	CPP_REG_MWR(CPP_PATH0_SC_UV_HOR_INI_PHASE,
		CPP_PATH0_SC_UV_HOR_INI_PHASE_INT,
		p->uv_hor_ini_phase_int << 16);
	CPP_REG_MWR(CPP_PATH0_SC_UV_HOR_INI_PHASE,
		CPP_PATH0_SC_UV_HOR_INI_PHASE_FRAC,
		p->uv_hor_ini_phase_frac);
	CPP_REG_MWR(CPP_PATH0_SC_Y_VER_INI_PHASE,
		CPP_PATH0_SC_Y_VER_INI_PHASE_INT,
		p->y_ver_ini_phase_int << 16);
	CPP_REG_MWR(CPP_PATH0_SC_Y_VER_INI_PHASE,
		CPP_PATH0_SC_Y_VER_INI_PHASE_FRAC,
		p->y_ver_ini_phase_frac);
	CPP_REG_MWR(CPP_PATH0_SC_UV_VER_INI_PHASE,
		CPP_PATH0_SC_UV_VER_INI_PHASE_INT,
		p->uv_ver_ini_phase_int << 16);
	CPP_REG_MWR(CPP_PATH0_SC_UV_VER_INI_PHASE,
		CPP_PATH0_SC_UV_VER_INI_PHASE_FRAC,
		p->uv_ver_ini_phase_frac);

	return 0;
}

static int sprd_scaledrv_tap_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CPP_REG_MWR(CPP_PATH0_SC_TAP, CPP_PATH0_Y_VER_TAP,
		p->y_ver_tap << 5);
	CPP_REG_MWR(CPP_PATH0_SC_TAP, CPP_PATH0_UV_VER_TAP,
		p->uv_ver_tap);

	return 0;
}

static int sprd_scaledrv_offset_size_set(
	struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	/* sc_in_trim */
	CPP_REG_MWR(CPP_PATH0_SC_IN_TRIM_OFFSET,
		CPP_PATH0_SC_IN_TRIM_OFFSET_X_MASK,
			p->sc_intrim_rect.x << 16);
	CPP_REG_MWR(CPP_PATH0_SC_IN_TRIM_OFFSET,
		CPP_PATH0_SC_IN_TRIM_OFFSET_Y_MASK,
			p->sc_intrim_rect.y);
	CPP_REG_MWR(CPP_PATH0_SC_IN_TRIM_SIZE,
		CPP_PATH0_SC_IN_TRIM_HEIGHT_MASK,
			p->sc_intrim_rect.h << 16);
	CPP_REG_MWR(CPP_PATH0_SC_IN_TRIM_SIZE,
		CPP_PATH0_SC_IN_TRIM_WIDTH_MASK,
			p->sc_intrim_rect.w);
	/* sc_out_trim */
	CPP_REG_MWR(CPP_PATH0_SC_OUT_TRIM_OFFSET,
		CPP_PATH0_SC_OUT_TRIM_OFFSET_X_MASK,
			p->sc_outtrim_rect.x << 16);
	CPP_REG_MWR(CPP_PATH0_SC_OUT_TRIM_OFFSET,
		CPP_PATH0_SC_OUT_TRIM_OFFSET_Y_MASK,
			p->sc_outtrim_rect.y);
	CPP_REG_MWR(CPP_PATH0_SC_OUT_TRIM_SIZE,
		CPP_PATH0_SC_OUT_TRIM_HEIGHT_MASK,
			p->sc_outtrim_rect.h << 16);
	CPP_REG_MWR(CPP_PATH0_SC_OUT_TRIM_SIZE,
		CPP_PATH0_SC_OUT_TRIM_WIDTH_MASK,
			p->sc_outtrim_rect.w);
	/* bp_trim */
	CPP_REG_MWR(CPP_PATH0_BP_TRIM_OFFSET,
		CPP_PATH0_BP_TRIM_OFFSET_X_MASK,
			p->bp_trim_rect.x << 16);
	CPP_REG_MWR(CPP_PATH0_BP_TRIM_OFFSET,
		CPP_PATH0_BP_TRIM_OFFSET_Y_MASK,
			p->bp_trim_rect.y);
	CPP_REG_MWR(CPP_PATH0_BP_TRIM_SIZE,
		CPP_PATH0_BP_TRIM_HEIGHT_MASK,
			p->bp_trim_rect.h << 16);
	CPP_REG_MWR(CPP_PATH0_BP_TRIM_SIZE,
		CPP_PATH0_BP_TRIM_WIDTH_MASK,
			p->bp_trim_rect.w);
	/* sc_full_in */
	CPP_REG_MWR(CPP_PATH0_SC_FULL_IN_SIZE,
		CPP_PATH0_SC_FULL_IN_HEIGHT_MASK,
			p->sc_full_in_size.h << 16);
	CPP_REG_MWR(CPP_PATH0_SC_FULL_IN_SIZE,
		CPP_PATH0_SC_FULL_IN_WIDTH_MASK,
			p->sc_full_in_size.w);
	/* sc_full_out */
	CPP_REG_MWR(CPP_PATH0_SC_FULL_OUT_SIZE,
		CPP_PATH0_SC_FULL_OUT_HEIGHT_MASK,
			p->sc_full_out_size.h << 16);
	CPP_REG_MWR(CPP_PATH0_SC_FULL_OUT_SIZE,
		CPP_PATH0_SC_FULL_OUT_WIDTH_MASK,
			p->sc_full_out_size.w);
	/* sc_slice_in */
	CPP_REG_MWR(CPP_PATH0_SC_SLICE_IN_SIZE,
		CPP_PATH0_SC_SLICE_IN_HEIGHT_MASK,
			p->sc_slice_in_size.h << 16);
	CPP_REG_MWR(CPP_PATH0_SC_SLICE_IN_SIZE,
		CPP_PATH0_SC_SLICE_IN_WIDTH_MASK,
			p->sc_slice_in_size.w);
	/* sc_slice_out */
	CPP_REG_MWR(CPP_PATH0_SC_SLICE_OUT_SIZE,
		CPP_PATH0_SC_SLICE_OUT_HEIGHT_MASK,
			p->sc_slice_out_size.h << 16);
	CPP_REG_MWR(CPP_PATH0_SC_SLICE_OUT_SIZE,
		CPP_PATH0_SC_SLICE_OUT_WIDTH_MASK,
			p->sc_slice_out_size.w);

	return 0;
}

static int sprd_scaledrv_addr_set(
	struct scale_drv_private *p,
	struct sprd_cpp_scale_cfg_parm *cfg_parm)
{
	int ret = 0;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
#ifndef HAPS_TEST
	if (cfg_parm->input_addr.mfd[0] == 0 ||
		cfg_parm->input_addr.mfd[1] == 0 ||
		cfg_parm->output_addr.mfd[0] == 0 ||
		cfg_parm->output_addr.mfd[1] == 0) {
		pr_err("fail to get valid in or out mfd\n");
		return -1;
	}
	if ((p->bp_en == 1) &&
			(cfg_parm->bp_output_addr.mfd[0] == 0 ||
			 cfg_parm->bp_output_addr.mfd[1] == 0)) {
		pr_err("fail to get valid bp mfd\n");
		return -1;
	}
	memcpy(p->iommu_src.mfd, cfg_parm->input_addr.mfd,
		sizeof(cfg_parm->input_addr.mfd));
	memcpy(p->iommu_dst.mfd, cfg_parm->output_addr.mfd,
		sizeof(cfg_parm->output_addr.mfd));
	if (p->bp_en == 1)
		memcpy(p->iommu_dst_bp.mfd, cfg_parm->bp_output_addr.mfd,
				sizeof(cfg_parm->bp_output_addr.mfd));

	ret = sprd_cpp_core_get_sg_table(&p->iommu_src);
	if (ret) {
		pr_err("fail to get cpp src sg table\n");
		return -1;
	}
	p->iommu_src.offset[0] = cfg_parm->input_addr.y;
	p->iommu_src.offset[1] = cfg_parm->input_addr.u;
	p->iommu_src.offset[2] = cfg_parm->input_addr.v;
	ret = sprd_cpp_core_get_addr(&p->iommu_src);
	if (ret) {
		pr_err("fail to get cpp src addr\n");
		return -1;
	}

	ret = sprd_cpp_core_get_sg_table(&p->iommu_dst);
	if (ret) {
		pr_err("fail to get cpp dst sg table\n");
		sprd_cpp_core_free_addr(&p->iommu_src);
		return ret;
	}
	p->iommu_dst.offset[0] = cfg_parm->output_addr.y;
	p->iommu_dst.offset[1] = cfg_parm->output_addr.u;
	p->iommu_dst.offset[2] = cfg_parm->output_addr.v;
	ret = sprd_cpp_core_get_addr(&p->iommu_dst);
	if (ret) {
		pr_err("fail to get cpp dst addr\n");
		sprd_cpp_core_free_addr(&p->iommu_src);
		return ret;
	}

	if (p->bp_en == 1) {
		ret = sprd_cpp_core_get_sg_table(&p->iommu_dst_bp);
		if (ret) {
			pr_err("fail to get cpp dst sg table\n");
			sprd_cpp_core_free_addr(&p->iommu_src);
			sprd_cpp_core_free_addr(&p->iommu_dst);
			return ret;
		}

		p->iommu_dst_bp.offset[0] = cfg_parm->bp_output_addr.y;
		p->iommu_dst_bp.offset[1] = cfg_parm->bp_output_addr.u;
		p->iommu_dst_bp.offset[2] = cfg_parm->bp_output_addr.v;
		ret = sprd_cpp_core_get_addr(&p->iommu_dst_bp);
		if (ret) {
			pr_err("fail to get cpp dst addr\n");
			sprd_cpp_core_free_addr(&p->iommu_src);
			sprd_cpp_core_free_addr(&p->iommu_dst);
			return ret;
		}
	}

	CPP_REG_WR(CPP_PATH0_SRC_ADDR_Y,
		p->iommu_src.iova[0]);
	CPP_REG_WR(CPP_PATH0_SRC_ADDR_UV,
		p->iommu_src.iova[1]);
	CPP_REG_WR(CPP_PATH0_SC_DES_ADDR_Y,
		p->iommu_dst.iova[0]);
	CPP_REG_WR(CPP_PATH0_SC_DES_ADDR_UV,
		p->iommu_dst.iova[1]);
	CPP_REG_WR(CPP_PATH0_BP_DES_ADDR_Y,
		p->iommu_dst_bp.iova[0]);
	CPP_REG_WR(CPP_PATH0_BP_DES_ADDR_UV,
		p->iommu_dst_bp.iova[1]);
	pr_debug("iommu_src y:0x%lx, uv:0x%lx\n",
		p->iommu_src.iova[0], p->iommu_src.iova[1]);
	pr_debug("iommu_dst y:0x%lx, uv:0x%lx\n",
		p->iommu_dst.iova[0], p->iommu_dst.iova[1]);
	pr_debug("iommu_dst_bp y:0x%lx, uv:0x%lx\n",
		p->iommu_dst_bp.iova[0], p->iommu_dst_bp.iova[1]);
#else
	CPP_REG_WR(CPP_PATH0_SRC_ADDR_Y,
		cfg_parm->input_addr.y);
	CPP_REG_WR(CPP_PATH0_SRC_ADDR_UV,
		cfg_parm->input_addr.u);
	CPP_REG_WR(CPP_PATH0_SC_DES_ADDR_Y,
		cfg_parm->output_addr.y);
	CPP_REG_WR(CPP_PATH0_SC_DES_ADDR_UV,
		cfg_parm->output_addr.u);
	CPP_REG_WR(CPP_PATH0_BP_DES_ADDR_Y,
		cfg_parm->bp_output_addr.y);
	CPP_REG_WR(CPP_PATH0_BP_DES_ADDR_UV,
		cfg_parm->bp_output_addr.u);
#endif
	return ret;
}

void sprd_scaledrv_dev_enable(
	struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	spin_lock_irqsave(p->hw_lock, flags);
	CPP_REG_OWR(CPP_PATH_EB, CPP_SCALE_PATH_EB_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void sprd_scaledrv_dev_disable(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	spin_lock_irqsave(p->hw_lock, flags);
	CPP_REG_AWR(CPP_PATH_EB, (~CPP_SCALE_PATH_EB_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

void sprd_scale_drv_max_size_get(unsigned int *max_width,
		unsigned int *max_height)
{
	*max_width = SCALE_FRAME_WIDTH_MAX;
	*max_height = SCALE_FRAME_HEIGHT_MAX;
}

void convert_param_to_drv(struct scale_drv_private *p,
		struct sprd_cpp_scale_cfg_parm *sc_cfg,
		slice_drv_slice_param_t *convert_param)
{
	p->input_endian = sc_cfg->input_endian.y_endian;
	p->input_uv_endian = sc_cfg->input_endian.uv_endian;
	p->output_endian = sc_cfg->output_endian.y_endian;
	p->output_uv_endian = sc_cfg->output_endian.uv_endian;
	p->rch_burst_gap = 0;
	p->wch_burst_gap = 0;
	p->src_pitch = convert_param->Path0_src_pitch;
	p->src_rect.x = convert_param->Path0_src_offset_x;
	p->src_rect.y = convert_param->Path0_src_offset_y;
	p->src_rect.w = convert_param->Path0_src_width;
	p->src_rect.h = convert_param->Path0_src_height;
	p->ver_deci = convert_param->ver_deci;
	p->hor_deci = convert_param->hor_deci;
	p->input_fmt = convert_param->Input_format;
	p->sc_intrim_src_size.w = convert_param->Sc_in_trim_src_width;
	p->sc_intrim_src_size.h = convert_param->Sc_in_trim_src_height;
	p->sc_intrim_rect.x = convert_param->Sc_in_trim_offset_x;
	p->sc_intrim_rect.y = convert_param->Sc_in_trim_offset_y;
	p->sc_intrim_rect.w = convert_param->Sc_in_trim_width;
	p->sc_intrim_rect.h = convert_param->Sc_in_trim_height;
	p->sc_slice_in_size.w = convert_param->Sc_slice_in_width;
	p->sc_slice_in_size.h = convert_param->Sc_slice_in_height;
	p->sc_slice_out_size.w = convert_param->Sc_slice_out_width;
	p->sc_slice_out_size.h = convert_param->Sc_slice_out_height;
	p->sc_full_in_size.w = convert_param->Sc_full_in_width;
	p->sc_full_in_size.h = convert_param->Sc_full_in_height;
	p->sc_full_out_size.w = convert_param->Sc_full_out_width;
	p->sc_full_out_size.h = convert_param->Sc_full_out_height;
	p->y_hor_ini_phase_int = convert_param->y_hor_ini_phase_int;
	p->y_hor_ini_phase_frac = convert_param->y_hor_ini_phase_frac;
	p->y_ver_ini_phase_int = convert_param->y_ver_ini_phase_int;
	p->y_ver_ini_phase_frac = convert_param->y_ver_ini_phase_frac;
	p->uv_hor_ini_phase_int = convert_param->uv_hor_ini_phase_int;
	p->uv_hor_ini_phase_frac = convert_param->uv_hor_ini_phase_frac;
	p->uv_ver_ini_phase_int = convert_param->uv_ver_ini_phase_int;
	p->uv_ver_ini_phase_frac = convert_param->uv_ver_ini_phase_frac;
	p->y_ver_tap = convert_param->y_ver_tap;
	p->uv_ver_tap = convert_param->uv_ver_tap;
	p->Sc_out_trim_src_size.w = convert_param->Sc_out_trim_src_width;
	p->Sc_out_trim_src_size.h = convert_param->Sc_out_trim_src_height;
	p->sc_outtrim_rect.x = convert_param->Sc_out_trim_offset_x;
	p->sc_outtrim_rect.y = convert_param->Sc_out_trim_offset_y;
	p->sc_outtrim_rect.w = convert_param->Sc_out_trim_width;
	p->sc_outtrim_rect.h = convert_param->Sc_out_trim_height;
	p->sc_des_pitch = convert_param->Path0_sc_des_pitch;
	p->sc_des_rect.w = convert_param->Path0_sc_des_width;
	p->sc_des_rect.h = convert_param->Path0_sc_des_height;
	p->sc_des_rect.x = convert_param->Path0_sc_des_offset_x;
	p->sc_des_rect.y = convert_param->Path0_sc_des_offset_y;
	p->sc_output_fmt = convert_param->Path0_sc_output_format;
	p->bp_en = convert_param->path0_bypass_path_en;
	p->bp_trim_src_size.w = convert_param->bypass_trim_src_width;
	p->bp_trim_src_size.h = convert_param->bypass_trim_src_height;
	p->bp_trim_rect.x = convert_param->bypass_trim_offset_x;
	p->bp_trim_rect.y = convert_param->bypass_trim_offset_y;
	p->bp_trim_rect.w = convert_param->bypass_trim_width;
	p->bp_trim_rect.h = convert_param->bypass_trim_height;
	p->bp_des_rect.x = convert_param->Path0_bypass_des_offset_x;
	p->bp_des_rect.y = convert_param->Path0_bypass_des_offset_y;
	p->bp_des_rect.w = convert_param->Path0_bypass_des_width;
	p->bp_des_rect.h = convert_param->Path0_bypass_des_height;
	p->bp_des_pitch = convert_param->Path0_bypass_des_pitch;
}

int sprd_scale_slice_parm_set(struct scale_drv_private *p,
		struct sprd_cpp_scale_cfg_parm *sc_cfg)
{
	int ret = 0;
	/* common */
	ret = sprd_scaledrv_input_format_set(p);
	if (ret) {
		pr_err("fail to get valid input format\n");
		goto exit;
	}
	ret = sprd_scaledrv_output_format_set(p);
	if (ret) {
		pr_err("fail to get valid output format\n");
		goto exit;
	}
	ret = sprd_scaledrv_input_endian_set(p);
	if (ret) {
		pr_err("fail to get valid input endian\n");
		goto exit;
	}
	ret = sprd_scaledrv_output_endian_set(p);
	if (ret) {
		pr_err("fail to get valid output endian\n");
		goto exit;
	}
	ret = sprd_scaledrv_burst_gap_set(p);
	if (ret) {
		pr_err("fail to set burst gap\n");
		goto exit;
	}
	ret = sprd_scaledrv_deci_set(p);
	if (ret) {
		pr_err("fail to set valid deci\n");
		goto exit;
	}

	ret = sprd_scaledrv_bpen_set(p);
	if (ret) {
		pr_err("fail to set bp en\n");
		goto exit;
	}
	/* pitch */
	ret = sprd_scaledrv_src_pitch_set(p);
	if (ret) {
		pr_err("fail to set valid src pitch\n");
		goto exit;
	}
	ret = sprd_scaledrv_des_pitch_set(p);
	if (ret) {
		pr_err("fail to set valid des pitch\n");
		goto exit;
	}
	/* rect offset size */
	ret = sprd_scaledrv_input_rect_set(p);
	if (ret) {
		pr_err("fail to set valid input rect\n");
		goto exit;
	}
	ret = sprd_scaledrv_output_rect_set(p);
	if (ret) {
		pr_err("fail to set ouput rect\n");
		goto exit;
	}
	ret = sprd_scaledrv_offset_size_set(p);
	if (ret) {
		pr_err("fail to set offset size\n");
		goto exit;
	}
	/* addr */
	ret = sprd_scaledrv_addr_set(p, sc_cfg);
	if (ret) {
		pr_err("fail to get valid output addr\n");
		goto exit;
	}
	scale_scaledrv_coeff_set(p, sc_cfg);
	ret = sprd_scaledrv_ini_phase_set(p);
	if (ret) {
		pr_err("fail to get valid ini phase\n");
		goto exit;
	}
	ret = sprd_scaledrv_tap_set(p);
	if (ret) {
		pr_err("fail to get valid tap value\n");
		goto exit;
	}

exit:
	return ret;
}

void sprd_scale_drv_free_mem(struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
#ifndef HAPS_TEST
	sprd_cpp_core_free_addr(&p->iommu_src);
	sprd_cpp_core_free_addr(&p->iommu_dst);
	if (p->bp_en == 1)
		sprd_cpp_core_free_addr(&p->iommu_dst_bp);
#endif
}

void sprd_scale_drv_start(struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	sprd_scaledrv_start(p);
}

void sprd_scale_drv_stop(struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	sprd_scaledrv_stop(p);
	sprd_scaledrv_dev_disable(p);
}

/*this function used to check parament from user space whether suitable*/
int sprd_scale_drv_capability_get(
	struct sprd_cpp_scale_capability *scale_param)
{
	int ret = 0;
	/* input */
	if (scale_param->src_size.w > SCALE_FRAME_WIDTH_MAX ||
		scale_param->src_size.h > SCALE_FRAME_HEIGHT_MAX ||
		scale_param->src_size.w < SCALE_FRAME_WIDTH_MIN ||
		scale_param->src_size.h < SCALE_FRAME_HEIGHT_MIN ||
		scale_param->dst_size.w < SCALE_FRAME_WIDTH_MIN ||
		scale_param->dst_size.h < SCALE_FRAME_HEIGHT_MIN) {
		pr_err("invalid input size %d %d\n",
				scale_param->src_size.w,
				scale_param->src_size.h);
		return -1;
	}

	if (MOD(scale_param->src_size.w, 8) != 0) {
		pr_err("scale pitch size is error\n");
		return -1;
	}
	/* sc des */
	if (MOD2(scale_param->dst_size.w) != 0) {
		pr_err("scale pitch size is error\n");
		return -1;
	}

	if ((scale_param->dst_format == SCALE_YUV420)
		&& (MOD(scale_param->dst_size.h, 4) != 0)) {
		pr_err("scale output height is invalid:%u\n",
			scale_param->dst_size.h);
		return -1;
	}
	if ((scale_param->dst_format == SCALE_YUV422)
		&& (MOD(scale_param->dst_size.h, 2) != 0)) {
		pr_err("scale output height is invalid:%u\n",
			scale_param->dst_size.h);
		return -1;
	}
	/* bp des */
#if 0
	if (MOD2(scale_param->dst_bp_size.w) != 0) {
		pr_err("scale pitch size is error\n");
		return -1;
	}

	if ((scale_param->dst_format == SCALE_YUV420)
		&& (MOD(scale_param->dst_bp_size.h, 4) != 0)) {
		pr_err("scale output height is invalid:%u\n",
			scale_param->dst_size.h);
		return -1;
	}
	if ((scale_param->dst_format == SCALE_YUV422)
		&& (MOD(scale_param->dst_bp_size.h, 2) != 0)) {
		pr_err("scale output height is invalid:%u\n",
			scale_param->dst_size.h);
		return -1;
	}
#endif
	return ret;
}
