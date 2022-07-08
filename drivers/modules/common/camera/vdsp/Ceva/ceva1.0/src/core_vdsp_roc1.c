#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/io.h>
#include "sprd_vdsp.h"
#include "vdsp_lib.h"
struct p_add {
	u32 p_addx_start;
	u32 p_addx_att0;
	u32 reserved_0x41c_0x420[2];
};

struct xm6_reg {
	u32 reserved_0x400;
	u32 mss_pcr;
	u32 mss_pdea1;
	u32 mss_pdia1;

	u32 mss_pdtc1;
	struct p_add add[16]; /*414-510*/

	u32 p_ccosar; /*514*/
	u32 p_ccocr;
	u32 p_ecadd;

	u32 p_mapar;
	u32 p_mapsr;
	u32 p_axi_ou_c;
	u32 reserved_0x52c;

	u32 p_mecccor;
	u32 reserved_0x534_0x53c[3];

	u32 p_mssacs;
	u32 p_shw_mapsr;
	u32 p_shw_ecadd;
	u32 p_shw_mecccor;

	u32 reserved_0x550_0x5fc[44];

	u32 mss_dmba;
	u32 mss_dmbe;
	u32 reserved_0x608_0x618[5];
	u32 mss_hdcfg;

	u32 mss_gpin;
	u32 mss_gpout;
	u32 mss_dacc;
	u32 mss_sdcfg;
	u32 reserved_0x630_0x634[2];
	u32 mss_barrier;
	u32 mss_dmab;
	u32 mss_ddsec_id;
	u32 dbg_wrc0;
	u32 dbg_wrc1;
	u32 dbg_wrc2;
	u32 reserved_0x650_0x654[2];
	u32 mss_ddtc;
	u32 reserved_0x65c;
	u32 mss_2dcfg1;
	u32 mss_2dcfg2;
	u32 mss_2dcfg3;
	u32 mss_2dcfg4;
	u32 mss_2dcfg5;
	u32 mss_2dcfg6;
	u32 mss_ddqs;
	u32 reserved_0x67c;
	u32 mss_ddea;
	u32 mss_ddia;
	u32 reserved_0x688;
	u32 mss_ddcl;
	u32 reserved_0x690_0x91c[164];
	u32 add0_start;
	u32 add0_att0;
	u32 add0_att1;
	u32 reserved_0x92c;
	u32 addx_start;
	u32 addx_att0;
	u32 addx_att1;
};
/* 0x78400400 - 0x78401600  */


struct icu_reg {
	u32 isr_l;
	u32 isr_h;
	u32 icr_l;
	u32 icr_h;
	u32 ipr_l;
	u32 ipr_h;
	u32 imr0_l;
	u32 imr0_h;
	u32 imr1_l;
	u32 imr1_h;
	u32 imr2_l;
	u32 imr2_h;
	u32 vimr_l;
	u32 vimr_h;
	u32 spc_cfg;
	u32 igr_l;
	u32 igr_h;
	u32 nmi_cfg;
	u32 vec46;
	u32 vec47;
	u32 vibase0;
	u32 vibase1;
	u32 vibase2;
	u32 vics_boot_l;
	u32 vics_boot_h;
	u32 cxr_cvr;
};
/* 0x20800000 - 0x20800064 */

static int vdsp_do_pdma(struct vdsp_context *ctx,
			u32 code_addr, u32 size)
{
	int status = 0;
	struct xm6_reg *reg = (struct xm6_reg *)ctx->xm6_base;

	VDSP_INFO("vdsp do pdma(ctx->xm6_base=%lx)\n", ctx->xm6_base);
	VDSP_INFO("vdsp do pdma(reg->mss_pdea1=%p)\n", &(reg->mss_pdea1));
	reg->mss_pdea1 = code_addr;
	reg->mss_pdia1 = 0x0;
	reg->mss_pdtc1 = size;
	VDSP_INFO("vdsp do pdma end\n");
	return status;
}

static int vdsp_do_ddma(struct vdsp_context *ctx,
			u32 data_addr, u32 size)
{
	int status = 0;
	struct xm6_reg *reg = (struct xm6_reg *)ctx->xm6_base;

	reg->mss_ddea = data_addr;
	reg->mss_ddia = 0x0;
	reg->mss_ddtc = size;
	reg->mss_ddcl = 0x0;
	VDSP_INFO("vdsp do pddma end\n");
	return status;
}

static int vdsp_core_parse_dt(struct vdsp_context *ctx, struct device_node *np)
{
	int status = 0;

	return status;
}

static int vdsp_set_pcache(struct vdsp_context *ctx)
{
	int status = 0;
	struct xm6_reg *reg = (struct xm6_reg *)ctx->xm6_base;

	reg->add[0].p_addx_att0 |= BIT(0);
	VDSP_INFO("vdsp_set_pcache end(reg->add[0].p_addx_att0=%p)\n",&(reg->add[0].p_addx_att0));
	return status;
}

static int vdsp_set_edp_aximo_range(struct vdsp_context *ctx)
{
	int status = 0;
	struct xm6_reg *reg = (struct xm6_reg *)ctx->xm6_base;

	reg->add0_start = 0x200000;
	reg->addx_start = 0x20160000;

	reg->mss_ddcl |= 0x60;
	reg->mss_dacc |= 0x3c0;

	VDSP_INFO("vdsp_set_edp_aximo_range end\n");
	return status;
}

static int icu_triggle_int0(struct vdsp_context *ctx, u32 index) //set int src index to int0
{
	int status = 0;
	struct icu_reg *reg = (struct icu_reg *)ctx->icu_base;

	reg->cxr_cvr &= 0xfffffff0;
	reg->imr0_l |= BIT(index);
	reg->igr_l  |= BIT(index);
	VDSP_INFO("icu_triggle_int0(reg->igr_l=%p)\n", &reg->igr_l);

	return status;
}

static int icu_triggle_int1(struct vdsp_context *ctx, u32 index) //set int src index to int1
{
	int status = 0;
	struct icu_reg *reg = (struct icu_reg *)ctx->icu_base;

	reg->imr1_l |= BIT(index);
	reg->igr_l  |= BIT(index);
	VDSP_INFO("icu_triggle_int0(reg->igr_l=%p)\n", &reg->igr_l);

	return status;
}

static int icu_isr(struct vdsp_context *ctx)
{
	struct icu_reg *reg = (struct icu_reg *)ctx->icu_base;
	u32 reg_val;
	reg_val = reg->isr_l; // src 0,1 is for vdsp
	reg_val &= 0xfffffffc;
	reg->icr_l = reg_val;
	VDSP_INFO("icu_triggle_int1(reg->icr_l=%p)\n", &reg->icr_l);
	return reg_val;
}

static void vdsp_dump(struct vdsp_context *ctx)
{
        u32 *reg = (u32 *)ctx->icu_base;
	struct xm6_reg *reg_xm6 = (struct xm6_reg *)ctx->xm6_base;
        int i;

        VDSP_INFO("dump vdsp icu reg list\n");
        VDSP_INFO("      0          4          8          C\n");
        for (i = 0; i < 64; i += 4) {
                VDSP_INFO("%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
                        i * 4, reg[i], reg[i + 1], reg[i + 2], reg[i + 3]);
        }
        VDSP_INFO("dump vdsp ceva reg\n");
	VDSP_INFO("reg->mss_pdea1=0x%x\n",reg_xm6->mss_pdea1);
	VDSP_INFO("reg->mss_pdia1=0x%x\n",reg_xm6->mss_pdia1);
	VDSP_INFO("reg->mss_pdtc1=0x%x\n",reg_xm6->mss_pdtc1);
	VDSP_INFO("reg->mss_ddea=0x%x\n",reg_xm6->mss_ddea);
	VDSP_INFO("reg->mss_ddia=0x%x\n",reg_xm6->mss_ddia);
	VDSP_INFO("reg->mss_ddtc=0x%x\n",reg_xm6->mss_ddtc);
	VDSP_INFO("reg->mss_ddcl=0x%x\n",reg_xm6->mss_ddcl);
	VDSP_INFO("reg->add[0].p_addx_att0=-x%x\n",reg_xm6->add[0].p_addx_att0);
	VDSP_INFO("reg->add0_start=0x%x\n",reg_xm6->add0_start);
	VDSP_INFO("reg->addx_start=0x%x\n",reg_xm6->addx_start);
	VDSP_INFO("reg->mss_ddcl=0x%x\n",reg_xm6->mss_ddcl);
	VDSP_INFO("reg->mss_dacc=0x%x\n",reg_xm6->mss_dacc);

}



static struct vdsp_core_ops vdsp_core_ops = {
	.parse_dt = vdsp_core_parse_dt,
	.do_pdma = vdsp_do_pdma,
	.do_ddma = vdsp_do_ddma,
	.set_pcache = vdsp_set_pcache,
	.set_edp_aximo_range = vdsp_set_edp_aximo_range,
	.dump = vdsp_dump,
	.isr_triggle_int0 = icu_triggle_int0,
	.isr_triggle_int1 = icu_triggle_int1,
	.isr = icu_isr,
};

static struct ops_entry entry = {
	.ver = "roc1",
	.ops = &vdsp_core_ops,
};

int vdsp_core_register(void)
{
	return vdsp_core_ops_register(&entry);
}

//subsys_initcall(vdsp_core_register);

