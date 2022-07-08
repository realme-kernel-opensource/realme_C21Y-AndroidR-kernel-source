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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <video/sprd_mmsys_pw_domain.h>

#include "dcam_int.h"
#include "dcam_path.h"
#include "dcam_reg.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

unsigned long g_dcam_regbase[DCAM_ID_MAX];
unsigned long g_dcam_aximbase;
unsigned long g_dcam_mmubase;

/*
 * Initialize dcam_if hardware, power/clk/int should be prepared after this call
 * returns. It also brings the dcam_pipe_dev from INIT state to IDLE state.
 */
int dcam_drv_hw_init(void *arg)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	if (unlikely(!arg)) {
		pr_err("fail to get invalid arg\n");
		return -EINVAL;
	}

	dev = (struct dcam_pipe_dev *)arg;
	hw = dev->hw;

	ret = sprd_cam_pw_on();
	ret = sprd_cam_domain_eb();
	/* prepare clk */
	hw->dcam_ioctl(hw, DCAM_HW_CFG_ENABLE_CLK, NULL);
        sprd_iommu_restore(&hw->soc_dcam->pdev->dev);
	ret = dcam_int_irq_request(&hw->pdev->dev,
		hw->ip_dcam[dev->idx]->irq_no, arg);

	return ret;
}

/*
 * De-initialize dcam_if hardware thus power/clk/int resource can be released.
 * Registers will be inaccessible and dcam_pipe_dev will enter INIT state from
 * IDLE state.
 */
int dcam_drv_hw_deinit(void *arg)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	if (unlikely(!arg)) {
		pr_err("fail to get invalid arg\n");
		return -EINVAL;
	}

	dev = (struct dcam_pipe_dev *)arg;
	hw = dev->hw;

	dcam_int_irq_free(&hw->pdev->dev, arg);
	/* unprepare clk and other resource */
	hw->dcam_ioctl(hw, DCAM_HW_CFG_DISABLE_CLK, NULL);
	ret = sprd_cam_domain_disable();
	ret = sprd_cam_pw_off();

	return ret;
}

int dcam_drv_dt_parse(struct platform_device *pdev,
			struct cam_hw_info *hw_info,
			uint32_t *dcam_count)
{
	struct cam_hw_soc_info *soc_dcam = NULL;
	struct cam_hw_ip_info *ip_dcam = NULL;
	struct device_node *dn = NULL;
	struct device_node *qos_node = NULL;
	struct device_node *iommu_node = NULL;
	struct regmap *ahb_map = NULL;
	void __iomem *reg_base = NULL;
	struct resource reg_res = {0}, irq_res = {0};
	uint32_t count = 0, prj_id = 0;
	uint32_t dcam_max_w = 0, dcam_max_h = 0;
	int i = 0, irq = 0;
	int args_count = 0;
	uint32_t args[2];
	char dcam_name[20];

	pr_info("start dcam dts parse\n");

	if (!pdev || !hw_info) {
		pr_err("fail to get pdev %p hw info %p\n", pdev, hw_info);
		return -EINVAL;
	}

	dn = pdev->dev.of_node;
	if (unlikely(!dn)) {
		pr_err("fail to get a valid device node\n");
		return -EINVAL;
	}

	ahb_map = syscon_regmap_lookup_by_phandle(dn, "sprd,cam-ahb-syscon");
	if (IS_ERR_OR_NULL(ahb_map)) {
		pr_err("fail to get sprd,cam-ahb-syscon\n");
		return PTR_ERR(ahb_map);
	}

	if (of_property_read_u32(dn, "sprd,dcam-count", &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dn, "sprd,project-id", &prj_id))
		pr_info("fail to parse the property of sprd,projectj-id\n");

	/* bounded kernel device node */
	hw_info->pdev = pdev;
	hw_info->prj_id = (enum cam_prj_id) prj_id;

	dcam_max_w = DCAM_PATH_WMAX;
	dcam_max_h = DCAM_PATH_HMAX;
	if (prj_id == ROC1) {
		dcam_max_w = DCAM_PATH_WMAX_ROC1;
		dcam_max_h = DCAM_PATH_HMAX_ROC1;
	}

	if (count > DCAM_ID_MAX) {
		pr_err("fail to get a valid dcam count, count: %u\n", count);
		return -EINVAL;
	}

	pr_info("dev: %s, full name: %s, cam_ahb_gpr: %p, count: %u\n",
		pdev->name, dn->full_name, ahb_map, count);

	pr_info("DCAM dcam_max_w = %u dcam_max_h = %u\n", dcam_max_w, dcam_max_h);

	iommu_node = of_parse_phandle(dn, "iommus", 0);
	if (iommu_node) {
		if (of_address_to_resource(iommu_node, 0, &reg_res))
			pr_err("fail to get DCAM IOMMU  addr\n");
		else {
			reg_base = ioremap(reg_res.start,
				reg_res.end - reg_res.start + 1);
			if (!reg_base)
				pr_err("fail to map DCAM IOMMU base\n");
			else
				g_dcam_mmubase = (unsigned long)reg_base;
		}
	}
	pr_info("DCAM IOMMU Base  0x%lx\n", g_dcam_mmubase);

	/* Start dcam soc related dt parse */
	soc_dcam = hw_info->soc_dcam;
	soc_dcam->pdev = pdev;
	/* AHB bus register mapping */
	soc_dcam->cam_ahb_gpr = ahb_map;
	/* qos dt parse */
	qos_node = of_parse_phandle(dn, "dcam_qos", 0);
	if (qos_node) {
		uint8_t val;

		if (of_property_read_u8(qos_node, "awqos-high", &val)) {
			pr_warn("isp awqos-high reading fail.\n");
			val = 0xD;
		}
		soc_dcam->awqos_high = (uint32_t)val;

		if (of_property_read_u8(qos_node, "awqos-low", &val)) {
			pr_warn("isp awqos-low reading fail.\n");
			val = 0xA;
		}
		soc_dcam->awqos_low = (uint32_t)val;

		if (of_property_read_u8(qos_node, "arqos", &val)) {
			pr_warn("isp arqos-high reading fail.\n");
			val = 0xA;
		}
		soc_dcam->arqos_high = val;
		soc_dcam->arqos_low = val;

		pr_info("get dcam qos node. r: %d %d w: %d %d\n",
			soc_dcam->arqos_high, soc_dcam->arqos_low,
			soc_dcam->awqos_high, soc_dcam->awqos_low);
	} else {
		soc_dcam->awqos_high = 0xD;
		soc_dcam->awqos_low = 0xA;
		soc_dcam->arqos_high = 0xA;
		soc_dcam->arqos_low = 0xA;
	}

	/* read dcam clk */
	soc_dcam->core_eb = of_clk_get_by_name(dn, "dcam_eb");
	if (IS_ERR_OR_NULL(soc_dcam->core_eb)) {
		pr_err("fail to read clk, dcam_eb\n");
		goto err_iounmap;
	}
	soc_dcam->axi_eb = of_clk_get_by_name(dn, "dcam_axi_eb");
	if (IS_ERR_OR_NULL(soc_dcam->axi_eb)) {
		pr_err("fail to read clk, dcam_axi_eb\n");
		goto err_iounmap;
	}
	soc_dcam->clk = of_clk_get_by_name(dn, "dcam_clk");
	if (IS_ERR_OR_NULL(soc_dcam->clk)) {
		pr_err("fail to read clk, dcam_clk\n");
		goto err_iounmap;
	}
	soc_dcam->clk_parent = of_clk_get_by_name(dn, "dcam_clk_parent");
	if (IS_ERR_OR_NULL(soc_dcam->clk_parent)) {
		pr_err("fail to read clk, dcam_clk_parent\n");
		goto err_iounmap;
	}
	soc_dcam->clk_default = clk_get_parent(soc_dcam->clk);
	if (hw_info->prj_id == SHARKL3) {
		soc_dcam->bpc_clk = of_clk_get_by_name(dn, "dcam_bpc_clk");
		if (IS_ERR_OR_NULL(soc_dcam->bpc_clk)) {
			pr_err("fail to get dcam_bpc_clk\n");
			goto err_iounmap;
		}
		soc_dcam->bpc_clk_parent =
			of_clk_get_by_name(dn, "dcam_bpc_clk_parent");
		if (IS_ERR_OR_NULL(soc_dcam->bpc_clk_parent)) {
			pr_err("fail to get dcam_bpc_clk_parent\n");
			goto err_iounmap;
		}
		soc_dcam->bpc_clk_default = clk_get_parent(soc_dcam->bpc_clk);
	}

	if (hw_info->prj_id != SHARKL3) {
		soc_dcam->axi_clk = of_clk_get_by_name(dn, "dcam_axi_clk");
		if (IS_ERR_OR_NULL(soc_dcam->clk)) {
			pr_err("fail to read clk, axi_clk\n");
			goto err_iounmap;
		}
		soc_dcam->axi_clk_parent =
			of_clk_get_by_name(dn, "dcam_axi_clk_parent");
		if (IS_ERR_OR_NULL(soc_dcam->clk_parent)) {
			pr_err("fail to read clk, axi_clk_parent\n");
			goto err_iounmap;
		}
		soc_dcam->axi_clk_default = clk_get_parent(soc_dcam->axi_clk);
	}

	args_count = syscon_get_args_by_name(dn, "dcam_all_reset", sizeof(args), args);
	if (args_count != ARRAY_SIZE(args)) {
		pr_err("fail to get dcam all reset syscon\n");
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		ip_dcam = hw_info->ip_dcam[i];
		/* DCAM index */
		ip_dcam->idx = i;
		/* Assign project ID, DCAM Max Height & Width Info */
		ip_dcam->max_width = dcam_max_w;
		ip_dcam->max_height = dcam_max_h;

		/* irq */
		irq = of_irq_to_resource(dn, i, &irq_res);
		if (irq <= 0) {
			pr_err("fail to get DCAM%d irq, error: %d\n", i, irq);
			goto err_iounmap;
		}
		ip_dcam->irq_no = (uint32_t) irq;

		/* DCAM register mapping */
		if (of_address_to_resource(dn, i, &reg_res)) {
			pr_err("fail to get DCAM%d phy addr\n", i);
			goto err_iounmap;
		}
		ip_dcam->phy_base = (unsigned long) reg_res.start;

		reg_base = ioremap(reg_res.start,
					reg_res.end - reg_res.start + 1);
		if (!reg_base) {
			pr_err("fail to map DCAM%d reg base\n", i);
			goto err_iounmap;
		}
		ip_dcam->reg_base = (unsigned long) reg_base;
		g_dcam_regbase[i] = (unsigned long)reg_base; /* TODO */

		pr_info("DCAM%d reg: %s 0x%lx %lx, irq: %s %u\n", i,
			reg_res.name, ip_dcam->phy_base, ip_dcam->reg_base,
			irq_res.name, ip_dcam->irq_no);

		ip_dcam->syscon.all_rst = args[0];
		ip_dcam->syscon.all_rst_mask = args[1];
		sprintf(dcam_name, "dcam%d_reset", i);
		args_count = syscon_get_args_by_name(dn, dcam_name,
			sizeof(args), args);
		if (args_count == ARRAY_SIZE(args)) {
			ip_dcam->syscon.rst = args[0];
			ip_dcam->syscon.rst_mask = args[1];
		} else {
			pr_err("fail to get dcam%d reset syscon\n", i);
			goto err_iounmap;
		}
	}

	if (of_address_to_resource(dn, i, &reg_res)) {
		pr_err("fail to get AXIM phy addr\n");
		goto err_iounmap;
	}

	reg_base = ioremap(reg_res.start, reg_res.end - reg_res.start + 1);
	if (!reg_base) {
		pr_err("fail to map AXIM reg base\n");
		goto err_iounmap;
	}
	g_dcam_aximbase = (unsigned long)reg_base; /* TODO */
	soc_dcam->axi_reg_base = (unsigned long)reg_base;

	pr_info("DCAM AXIM reg: %s %lx\n", reg_res.name, g_dcam_aximbase);

	*dcam_count = count;

	return 0;

err_iounmap:
	for (i = i - 1; i >= 0; i--)
		iounmap((void __iomem *)(hw_info->ip_dcam[0]->reg_base));
	iounmap((void __iomem *)g_dcam_mmubase);
	g_dcam_mmubase = 0;

	return -ENXIO;
}
