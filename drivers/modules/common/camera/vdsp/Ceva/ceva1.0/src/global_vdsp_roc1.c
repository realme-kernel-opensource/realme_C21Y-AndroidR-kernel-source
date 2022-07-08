/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/io.h>
#include "sprd_vdsp.h"
#include "vdsp_lib.h"

struct glb_ctrl {
	struct regmap *regmap;
	u32 reg;
	u32 mask;
};

struct glb_ctrl power;
struct glb_ctrl pd_sel;
struct glb_ctrl dslp_ena;
struct glb_ctrl pw_dbg;
static struct glb_ctrl enable;
struct glb_ctrl rst;
struct glb_ctrl rst_core;
struct glb_ctrl vdsp_ahb;  //TODO

struct glb_ctrl frc_sleep;
struct glb_ctrl stop_en;
//struct glb_ctrl vdma_eb;
struct glb_ctrl int_mask;
struct glb_ctrl int_s_en_l;
struct glb_ctrl int_s_en_h;
struct glb_ctrl vdsp_misc;
struct glb_ctrl vdsp_vector;


void vdsp_power_on(void)
{

	u32 status;

	regmap_update_bits(power.regmap, power.reg,
                           power.mask,(u32)~ power.mask);
	regmap_read(frc_sleep.regmap, frc_sleep.reg, &status);
	regmap_write(frc_sleep.regmap, frc_sleep.reg,
                           (u32)~frc_sleep.mask & status);
	regmap_read(frc_sleep.regmap, frc_sleep.reg, &status);
	regmap_write(frc_sleep.regmap, frc_sleep.reg,
                           (u32)~stop_en.mask & status);
}

void vdsp_clk_en(void)
{
}

void vdsp_reg_init(struct vdsp_context *ctx)
{
	int status = 0;

	regmap_update_bits(enable.regmap, enable.reg,
			enable.mask,(u32)enable.mask);
	VDSP_DEBUG("int_mask reg =0x%x, mask=0x%x, value =0x%x\n",int_mask.reg,int_mask.mask,(u32)~int_mask.mask);
	regmap_read(int_mask.regmap, int_mask.reg, &status);
	regmap_write(int_mask.regmap, int_mask.reg,
			(u32)~int_mask.mask & status);
	regmap_write(int_s_en_l.regmap, int_s_en_l.reg,
			(u32)int_s_en_l.mask);
	regmap_write(int_s_en_h.regmap, int_s_en_h.reg,
			(u32)int_s_en_h.mask);
	regmap_update_bits(rst.regmap, rst.reg,
			rst.mask,(u32)rst.mask);
	regmap_update_bits(rst_core.regmap, rst_core.reg,
			rst_core.mask,(u32)rst_core.mask);
}



void vdsp_power_off_no_boot(void)
{
	u32 status;

	regmap_write(dslp_ena.regmap, dslp_ena.reg,
			(u32)dslp_ena.mask);
	regmap_update_bits(power.regmap, power.reg,
			power.mask,(u32)power.mask);
	regmap_update_bits(pd_sel.regmap, pd_sel.reg,
			pd_sel.mask,(u32)~pd_sel.mask);

	regmap_update_bits(enable.regmap, enable.reg,
			enable.mask,(u32)~enable.mask);
	/* should cmd enter stand-by mode */
	regmap_read(frc_sleep.regmap, frc_sleep.reg, &status);
	regmap_write(frc_sleep.regmap, frc_sleep.reg,
			(u32)frc_sleep.mask | status);
	regmap_read(frc_sleep.regmap, frc_sleep.reg, &status);
	regmap_write(frc_sleep.regmap, frc_sleep.reg,
			(u32)stop_en.mask | status);
	regmap_read(pw_dbg.regmap, pw_dbg.reg, &status);
	regmap_update_bits(rst.regmap, rst.reg,
			rst.mask,(u32)rst.mask);
	regmap_update_bits(rst_core.regmap, rst_core.reg,
			rst_core.mask,(u32)rst_core.mask);
/*
	//waiting pwr_status1_dbg{15:8} = 0x7
	while ((status & pw_dbg.mask) != 0x700){
		msleep(1);
		regmap_read(pw_dbg.regmap, pw_dbg.reg, &status);
	}
*/
}




static int vdsp_clk_parse_dt(struct vdsp_context *ctx, struct device_node *np)
{
	int status = 0;

	return status;
}


static int vdsp_clk_init(struct vdsp_context *ctx)
{
	int ret = 0;

	return ret;
}


static int vdsp_clk_enable(struct vdsp_context *ctx)
{
	int ret = -1;

	return ret;
}

static int vdsp_clk_disable(struct vdsp_context *ctx)
{

	return 0;
}


static int vdsp_clk_update(struct vdsp_context *ctx, int clk_id, int val)
{
	int ret = -1;

	return ret;
}

static int vdsp_glb_parse_dt(struct vdsp_context *ctx, struct device_node *np)
{
	int ret;
	u32 args[2];

	VDSP_DEBUG("vdsp_glb_parse_dt called\n");
	power.regmap = syscon_regmap_lookup_by_name(np, "power");
	if (IS_ERR(power.regmap))
		VDSP_ERROR("failed to get syscon-name: power\n");

	ret = syscon_get_args_by_name(np, "power", 2, args);
	if (ret == 2) {
		power.reg = args[0];
		power.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name power\n");

	pd_sel.regmap = syscon_regmap_lookup_by_name(np, "pd_sel");
	if (IS_ERR(pd_sel.regmap))
		VDSP_ERROR("failed to get syscon-name: pd_sel\n");

	ret = syscon_get_args_by_name(np, "pd_sel", 2, args);
	if (ret == 2) {
		pd_sel.reg = args[0];
		pd_sel.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name pd_sel\n");

	dslp_ena.regmap = syscon_regmap_lookup_by_name(np, "dslp_ena");
	if (IS_ERR(dslp_ena.regmap))
		VDSP_ERROR("failed to get syscon-name: dslp_ena\n");

	ret = syscon_get_args_by_name(np, "dslp_ena", 2, args);
	if (ret == 2) {
		dslp_ena.reg = args[0];
		dslp_ena.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name dslp_ena\n");

	pw_dbg.regmap = syscon_regmap_lookup_by_name(np, "pw_dbg");
	if (IS_ERR(pw_dbg.regmap))
		VDSP_ERROR("failed to get syscon-name: pw_dbg\n");

	ret = syscon_get_args_by_name(np, "pw_dbg", 2, args);
	if (ret == 2) {
		pw_dbg.reg = args[0];
		pw_dbg.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name pw_dbg\n");

	frc_sleep.regmap = syscon_regmap_lookup_by_name(np, "frc_sleep");
	if (IS_ERR(frc_sleep.regmap))
		VDSP_ERROR("failed to get syscon-name: frc_sleep\n");

	ret = syscon_get_args_by_name(np, "frc_sleep", 2, args);
	if (ret == 2) {
		frc_sleep.reg = args[0];
		frc_sleep.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name frc_sleep\n");

	stop_en.regmap = syscon_regmap_lookup_by_name(np, "stop_en");
	if (IS_ERR(stop_en.regmap))
		VDSP_ERROR("failed to get syscon-name: stop_en\n");

	ret = syscon_get_args_by_name(np, "stop_en", 2, args);
	if (ret == 2) {
		stop_en.reg = args[0];
		stop_en.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name stop_en\n");


	enable.regmap = syscon_regmap_lookup_by_name(np, "enable");
	if (IS_ERR(enable.regmap))
		VDSP_ERROR("failed to get syscon-name: enable\n");

	ret = syscon_get_args_by_name(np, "enable", 2, args);
	if (ret == 2) {
		enable.reg = args[0];
		enable.mask = args[1];
		VDSP_INFO("enable reg =0x%x, mask=0x%x\n",enable.reg,enable.mask);
	} else
		VDSP_ERROR("failed to get args for syscon-name enable\n");
#if 0
	vdma_eb.regmap = syscon_regmap_lookup_by_name(np, "vdma_eb");
	if (IS_ERR(vdma_eb.regmap))
		pr_emerg("failed to get syscon-name: vdma_eb\n");

	ret = syscon_get_args_by_name(np, "vdma_eb", 2, args);
	if (ret == 2) {
		vdma_eb.reg = args[0];
		vdma_eb.mask = args[1];
	} else
		pr_emerg("failed to get args for syscon-name vdma_eb\n");
#endif

	int_mask.regmap = syscon_regmap_lookup_by_name(np, "int_mask");
	if (IS_ERR(int_mask.regmap))
		VDSP_ERROR("failed to get syscon-name: int_mask\n");

	ret = syscon_get_args_by_name(np, "int_mask", 2, args);
	if (ret == 2) {
		int_mask.reg = args[0];
		int_mask.mask = args[1];
		VDSP_INFO("int_mask reg =0x%x, mask=0x%x\n",int_mask.reg,int_mask.mask);
	} else
		VDSP_ERROR("failed to get args for syscon-name int_maks\n");

	int_s_en_l.regmap = syscon_regmap_lookup_by_name(np, "int_s_en_l");
	if (IS_ERR(int_s_en_l.regmap))
		VDSP_ERROR("failed to get syscon-name: int_s_en_l\n");

	ret = syscon_get_args_by_name(np, "int_s_en_l", 2, args);
	if (ret == 2) {
		int_s_en_l.reg = args[0];
		int_s_en_l.mask = args[1];
		VDSP_INFO("int_s_en_l reg =0x%x, mask=0x%x\n",int_s_en_l.reg,int_s_en_l.mask);
	} else
		VDSP_ERROR("failed to get args for syscon-name int_s_en_l\n");

	int_s_en_h.regmap = syscon_regmap_lookup_by_name(np, "int_s_en_h");
	if (IS_ERR(int_s_en_h.regmap))
		VDSP_ERROR("failed to get syscon-name: int_s_en_h\n");

	ret = syscon_get_args_by_name(np, "int_s_en_h", 2, args);
	if (ret == 2) {
		int_s_en_h.reg = args[0];
		int_s_en_h.mask = args[1];
		VDSP_INFO("int_s_en_h reg =0x%x, mask=0x%x\n",int_s_en_h.reg,int_s_en_h.mask);
	} else
		VDSP_ERROR("failed to get args for syscon-name int_s_en_h\n");

	rst.regmap = syscon_regmap_lookup_by_name(np, "rst");
	if (IS_ERR(rst.regmap))
		VDSP_ERROR("failed to get syscon-name: rst\n");

	ret = syscon_get_args_by_name(np, "rst", 2, args);
	if (ret == 2) {
		rst.reg = args[0];
		rst.mask = args[1];
		VDSP_INFO("rst reg =0x%x, mask=0x%x\n",rst.reg,rst.mask);
	} else
		VDSP_ERROR("failed to get args for syscon-name rst\n");

	rst_core.regmap = syscon_regmap_lookup_by_name(np, "rst_core");
	if (IS_ERR(rst_core.regmap))
		VDSP_ERROR("failed to get syscon-name: rst_core\n");

	ret = syscon_get_args_by_name(np, "rst_core", 2, args);
	if (ret == 2) {
		rst_core.reg = args[0];
		rst_core.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name rst_core\n");


	vdsp_misc.regmap = syscon_regmap_lookup_by_name(np, "misc");
	if (IS_ERR(vdsp_misc.regmap))
		VDSP_ERROR("failed to get syscon-name: misc\n");

	ret = syscon_get_args_by_name(np, "misc", 2, args);
	if (ret == 2) {
		vdsp_misc.reg = args[0];
		vdsp_misc.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name misc\n");

	vdsp_vector.regmap = syscon_regmap_lookup_by_name(np, "vector");
	if (IS_ERR(vdsp_vector.regmap))
		VDSP_ERROR("failed to get syscon-name: vector\n");

	ret = syscon_get_args_by_name(np, "vector", 2, args);
	if (ret == 2) {
		vdsp_vector.reg = args[0];
		vdsp_vector.mask = args[1];
	} else
		VDSP_ERROR("failed to get args for syscon-name vector\n");
#if 0
	rst_sys.regmap = syscon_regmap_lookup_by_name(np, "rst_sys");
	if (IS_ERR(rst_sys.regmap))
		pr_emerg("failed to get syscon-name: rst_sys\n");
	ret = syscon_get_args_by_name(np, "rst_sys", 2, args);
	if (ret == 2) {
		rst_sys.reg = args[0];
		rst_sys.mask = args[1];
	} else
		pr_emerg("failed to get args for syscon-name rst_sys\n");

	rst_ocem.regmap = syscon_regmap_lookup_by_name(np, "rst_ocem");
	if (IS_ERR(rst_ocem.regmap))
		pr_emerg("failed to get syscon-name: rst_ocem\n");

	ret = syscon_get_args_by_name(np, "rst_ocem", 2, args);
	if (ret == 2) {
		rst_ocem.reg = args[0];
		rst_ocem.mask = args[1];
	} else
		pr_emerg("failed to get args for syscon-name rst_ocem\n");

	rst_core.regmap = syscon_regmap_lookup_by_name(np, "rst_core");
	if (IS_ERR(rst_core.regmap))
		pr_emerg("failed to get syscon-name: rst_core\n");

	ret = syscon_get_args_by_name(np, "rst_core", 2, args);
	if (ret == 2) {
		rst_core.reg = args[0];
		rst_core.mask = args[1];
	} else
		pr_emerg("failed to get args for syscon-name rst_core\n");
#endif
	return ret;
}

static void vdsp_glb_enable(struct vdsp_context *ctx)
{

}

static void vdsp_glb_disable(struct vdsp_context *ctx)
{

}

static void vdsp_reset(struct vdsp_context *ctx)
{
	VDSP_INFO("roc1 global_vdsp reset enter\n");
	regmap_update_bits(rst.regmap, rst.reg,
                           rst.mask,(u32)~ rst.mask);
}

static void vdsp_core_reset(struct vdsp_context *ctx) //en: 0,1
{
	VDSP_INFO("roc1 global_vdsp core reset enter\n");
	regmap_update_bits(rst_core.regmap, rst_core.reg,
                           rst_core.mask,(u32)~ rst_core.mask);
}
static void vdsp_acu_acc(struct vdsp_context *ctx, int enable)
{
	int status = 0;

	regmap_read(vdsp_misc.regmap, vdsp_misc.reg, &status);
	if (enable) {
		regmap_write(vdsp_misc.regmap, vdsp_misc.reg,
				(u32)status | 0x400);
		regmap_read(vdsp_misc.regmap, vdsp_misc.reg, &status);
		VDSP_INFO("vdsp_acu_acc value after set=0x%x\n",(u32)status);
	}
	else {
		regmap_write(vdsp_misc.regmap, vdsp_misc.reg,
				(u32)status & 0xfffffbff);
		regmap_read(vdsp_misc.regmap, vdsp_misc.reg, &status);
		VDSP_INFO("vdsp_acu_acc value after set  =0x%x\n",(u32)status);
	}

}

static void vdsp_boot_signal_set(struct vdsp_context *ctx, int enable)
{
	int status = 0;

	regmap_read(vdsp_misc.regmap, vdsp_misc.reg, &status);
	if (enable) {
		regmap_write(vdsp_misc.regmap, vdsp_misc.reg,
				(u32)status | 0x200);
		regmap_read(vdsp_misc.regmap, vdsp_misc.reg, &status);
		VDSP_INFO("vdsp_acu_acc value after set=0x%x\n",(u32)status);
	}
	else {
		regmap_write(vdsp_misc.regmap, vdsp_misc.reg,
				(u32)status & 0xfffffbdff);
		regmap_read(vdsp_misc.regmap, vdsp_misc.reg, &status);
		VDSP_INFO("vdsp_acu_acc value after set  =0x%x\n",(u32)status);
	}

}

static void vdsp_vector_addr_set(struct vdsp_context *ctx, u32 ext_code_addr)
{
	VDSP_INFO("vdsp_vector_addr_set enter\n");
	regmap_write(vdsp_vector.regmap, vdsp_vector.reg,
                           ext_code_addr);
}

static void vdsp_power_domain(struct vdsp_context *ctx, int enable)
{
	if (enable)
		vdsp_power_on();
	else
		vdsp_power_off_no_boot();
}

static struct vdsp_clk_ops vdsp_clk_ops = {
	.parse_dt = vdsp_clk_parse_dt,
	.init = vdsp_clk_init,
	.enable = vdsp_clk_enable,
	.disable = vdsp_clk_disable,
	.update = vdsp_clk_update,
};

static struct vdsp_glb_ops vdsp_glb_ops = {
	.parse_dt = vdsp_glb_parse_dt,
	.init = vdsp_reg_init,
	.reset = vdsp_reset,
	.core_reset = vdsp_core_reset,
	.acu_acc = vdsp_acu_acc,
	.boot_signal_set = vdsp_boot_signal_set,
	.vector_addr_set = vdsp_vector_addr_set,
	.enable = vdsp_glb_enable,
	.disable = vdsp_glb_disable,
	.power = vdsp_power_domain,
};

static struct ops_entry clk_entry = {
	.ver = "roc1",
	.ops = &vdsp_clk_ops,
};

static struct ops_entry glb_entry = {
	.ver = "roc1",
	.ops = &vdsp_glb_ops,
};

int vdsp_glb_register(void)
{
	vdsp_clk_ops_register(&clk_entry);
	vdsp_glb_ops_register(&glb_entry);
	return 0;
}

//subsys_initcall(vdsp_glb_register);

