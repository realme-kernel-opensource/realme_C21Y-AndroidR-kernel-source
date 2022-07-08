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

#ifndef __CAM_DEBUGGER_H__
#define __CAM_DEBUGGER_H__

#include "dcam_interface.h"

/* compression override commands */
enum {
	CH_PRE = 0,
	CH_CAP = 1,
	CH_VID = 2,
	CH_MAX = 3,

	FBC_DCAM = 0,
	FBC_3DNR = 1,
	FBC_ISP = 2,
	FBC_MAX = 3,
};

/* compression override setting */
struct compression_override {
	uint32_t enable;
	uint32_t override[CH_MAX][FBC_MAX];
};

#define DCAM_IMAGE_REPLACER_FILENAME_MAX 128
#define DCAM_IMAGE_REPLACER_PATH_MAX (DCAM_PATH_BIN + 1)
/*
 * replace input/output image by using reserved buffer...
 * currently only for FULL and BIN path
 */
struct dcam_image_replacer {
	uint32_t enabled[DCAM_IMAGE_REPLACER_PATH_MAX];
	char filename[DCAM_IMAGE_REPLACER_PATH_MAX][DCAM_IMAGE_REPLACER_FILENAME_MAX];
};

struct cam_debug_bypass {
	uint32_t idx;
	struct cam_hw_info *hw;
};

struct camera_debugger {
	struct compression_override compression[CAM_ID_MAX];
	struct dcam_image_replacer replacer[DCAM_ID_MAX];
	struct cam_hw_info *hw;
};

int cam_debugger_init(struct camera_debugger *debugger);
int cam_debugger_deinit(void);

#endif
