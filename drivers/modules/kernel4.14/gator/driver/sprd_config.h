/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#ifndef __GATOR_SPRD_CONFIG__
#define __GATOR_SPRD_CONFIG__
#include <linux/sizes.h>

#define DONOT_CARE (0xFFFFFFFF)
#define PBM (1 << 0)
#define PTM (1 << 1)

/* 0xFFFFFFFF don't care */
struct sprd_chipid_map {
	unsigned int chipid1; //0x00FC
	unsigned int chipid0; //0x00F8
	unsigned int chipid3; //0x00E4
	unsigned int chipid2; //0x00E0
	unsigned int type;
	unsigned int gp_id;
	char name[8];
};


struct sprd_busmon_ext_config {
	unsigned int reg_base;
	unsigned int en_bit;
	unsigned int glb_reg_off;
	unsigned int glb_en_bit;
	unsigned int ext_reg_off;
	unsigned int ext_en_bit;
	unsigned int ext1_reg_off;
	unsigned int ext1_en_bit;
};

struct sprd_busmon_config {
	unsigned int bm_glb_reg;
	unsigned int bm_glb_en_bit_off;
	unsigned int bm_cnt;
	struct sprd_busmon_ext_config * bm_tmr;
	struct sprd_busmon_ext_config * bm_pmu;
};

struct sprd_busmon_gp_config {
	unsigned int busmon_gp_cnt;
	struct sprd_busmon_config * bm_cfg;
};

static struct sprd_busmon_ext_config Whale2_bm_tmr_cfg = {
	0x404D0000,
	0,
	0x000001D0,
	1,
	0x00000138,
	19,
	DONOT_CARE,
};

static struct sprd_busmon_config Whale2_bm_cfg[2] = {
	{
		0x30010004,
		16,
		11,
		&Whale2_bm_tmr_cfg,
	},
	{
		0x30810004,
		16,
		11,
		&Whale2_bm_tmr_cfg,
	},
};

static struct sprd_busmon_ext_config iWhale2_bm_tmr_cfg = {
	0xE44D0000,
	0,
	0x000001D0,
	1,
	0x00000138,
	19,
	DONOT_CARE,
};

static struct sprd_busmon_ext_config iWhale2_bm_pmu_cfg = {
	0x00000204,
	0,
	0x00000204,
	1,
	DONOT_CARE,
	0,
	DONOT_CARE,
};

static struct sprd_busmon_config iWhale2_bm_cfg[2] = {
	{
		0xC0010004,
		21,
		6,
		&iWhale2_bm_tmr_cfg,
		&iWhale2_bm_pmu_cfg,
	},
	{
		0xC0810004,
		21,
		4,
		&iWhale2_bm_tmr_cfg,
		&iWhale2_bm_pmu_cfg,
	},
};


static struct sprd_busmon_ext_config SharkL2_bm_tmr_cfg = {
	0x40270000,
	0,
	0x000000B0,
	14,
	0x00000134,
	11,
	DONOT_CARE,
};

static struct sprd_busmon_config SharkL2_bm_cfg[1] = {
	{
		0x300E0004,
		16,
		8,
		&SharkL2_bm_tmr_cfg,
	}
};

static struct sprd_busmon_config SharkLE_bm_cfg[1] = {
	{
		0x300E0004,
		23,
		7,
	}
};

static struct sprd_busmon_config SharkL3_bm_cfg[1] = {
	{
		0x300F0054,
		0,
		8,
	}
};

static struct sprd_chipid_map busmon_map[] = {
	// Whale2
	{0x5768616C, 0x65320000, DONOT_CARE, DONOT_CARE, PBM, 0, "Wh2"},
	// SharkL2
	{0x96330000, DONOT_CARE, 0x53686172, 0x6B4C3200, PBM, 1, "SL2"},
	// SharkLE
	{0x96360000, DONOT_CARE, 0x53686172, 0x6B4C4500, PTM, 2, "SLE"},
	// SharkLJ1
	{0x96350000, DONOT_CARE, 0x53686172, 0x6B4C4A31, PBM, 3, "SJ1"},
	// iWhale2
	{0x69576861, 0x6C653200, DONOT_CARE, DONOT_CARE, PBM, 4, "iWh2"},
	// SharkL3
	{0x98550000, DONOT_CARE, 0x53686172, 0x6B4C3300, PTM, 5, "SL3"},
};

static struct sprd_busmon_gp_config bm_gp_cfg[] = {
	// Whale2
	{2, &Whale2_bm_cfg[0]},
	// SharkL2
	{1, &SharkL2_bm_cfg[0]},
	// SharkLE
	{1, &SharkLE_bm_cfg[0]},
	// SharkLJ1
	{1, &SharkL2_bm_cfg[0]},
	// iWhale2
	{2, &iWhale2_bm_cfg[0]},
	// SharkL3
	{1, &SharkL3_bm_cfg[0]},
};

#endif
