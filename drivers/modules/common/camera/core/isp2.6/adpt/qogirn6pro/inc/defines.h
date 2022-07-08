/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ADPT_DEFINES_H_
#define _ADPT_DEFINES_H_

/* configure fmcu with isp register offset, range is 0x620-0x630 */
#define ISP_OFFSET_RANGE           0x6300620

#define ISP_WIDTH_MAX              8048
#define ISP_HEIGHT_MAX             6036
#define DCAM_24M_WIDTH             5664
#define DCAM_16M_WIDTH             4672
#define DCAM_13M_WIDTH             4160
#define DCAM_8M_WIDTH              3264
#define DCAM_RDS_OUT_LIMIT         2160

#define DCAM_PATH_WMAX             8048
#define DCAM_PATH_HMAX             6036
#define RAW_OVERLAP_UP             58
#define RAW_OVERLAP_DOWN           78
#define RAW_OVERLAP_LEFT           118
#define RAW_OVERLAP_RIGHT          138
#define ISP_SCALER_UP_MAX          4
#define DCAM_SW_SLICE_HEIGHT_MAX   8192
#define DCAM_HW_SLICE_WIDTH_MAX    8192

#define ISP_SC_COEFF_COEF_SIZE     (1 << 10)
#define ISP_SC_COEFF_TMP_SIZE      (21 << 10)

#define ISP_SC_H_COEF_SIZE         0xC0
#define ISP_SC_V_COEF_SIZE         0x210
#define ISP_SC_V_CHROM_COEF_SIZE   0x210

#define ISP_SC_COEFF_H_NUM         (ISP_SC_H_COEF_SIZE / 6)
#define ISP_SC_COEFF_H_CHROMA_NUM  (ISP_SC_H_COEF_SIZE / 12)
#define ISP_SC_COEFF_V_NUM         (ISP_SC_V_COEF_SIZE / 4)
#define ISP_SC_COEFF_V_CHROMA_NUM  (ISP_SC_V_CHROM_COEF_SIZE / 4)

/*
 *DCAM_CONTROL register bit map id
 * for force_cpy/auto_cpy control
 */
enum dcam_ctrl_id {
	DCAM_CTRL_CAP = (1 << 0),
	DCAM_CTRL_COEF = (1 << 1),
	DCAM_CTRL_RDS = (1 << 2),
	DCAM_CTRL_FULL = (1 << 3),
	DCAM_CTRL_BIN = (1 << 4),
	DCAM_CTRL_PDAF = (1 << 5),
	DCAM_CTRL_VCH2 = (1 << 6),
	DCAM_CTRL_VCH3 = (1 << 7),
};
#define DCAM_CTRL_ALL  0xff

enum raw_pitch_format{
	RAW_PACK10 = 0x00,
	RAW_HALF10 = 0x01,
	RAW_HALF14 = 0x02,
	RAW_FORMAT_MAX
};

static inline uint32_t cal_sprd_raw_pitch(uint32_t w, uint32_t is_loose)
{
	if(is_loose == RAW_PACK10)
		w = (w * 10 + 127) / 128 * 128 / 8;
	else
		w = (w * 16 + 127) / 128 * 128 / 8;

	return w;
}
#endif
