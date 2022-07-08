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
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "sprd_img.h"
#include <video/sprd_mmsys_pw_domain.h>
#include <sprd_mm.h>
#include <sprd_fd.h>

#include "cam_types.h"
#include "cam_buf.h"
#include "fd_core.h"
#include "fd_drv.h"
#include "fd_reg.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "fd_drv: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define FD_IRQ_MASK ((FD_MASK_INT_ERR) | (FD_MASK_INT_RAW))
#define FD_AXI_STOP_TIMEOUT			2000
#define FD_INT_TIMEOUT			2000

#define FD_DVFS_ENABLED 0

#if FD_DVFS_ENABLED
static struct sprd_fd_dvfs fd_dvfs_map[SPRD_FD_DVFS_INDEX_MAX] = {
	[SPRD_FD_DVFS_INDEX0] = {FD_REG_DVFS_INDEX0_MAP,
				    SPRD_FD_CLK_FREQ_76_8M, 0, 0},
	[SPRD_FD_DVFS_INDEX1] = {FD_REG_DVFS_INDEX1_MAP,
				    SPRD_FD_CLK_FREQ_128M, 1, 0},
	[SPRD_FD_DVFS_INDEX2] = {FD_REG_DVFS_INDEX2_MAP,
				    SPRD_FD_CLK_FREQ_256M, 2, 0},
	[SPRD_FD_DVFS_INDEX3] = {FD_REG_DVFS_INDEX3_MAP,
				    SPRD_FD_CLK_FREQ_384M, 4, 1},
	[SPRD_FD_DVFS_INDEX4] = {FD_REG_DVFS_INDEX4_MAP,
				    SPRD_FD_CLK_FREQ_384M, 5, 1},
	[SPRD_FD_DVFS_INDEX5] = {FD_REG_DVFS_INDEX5_MAP,
				    SPRD_FD_CLK_FREQ_384M, 6, 2},
	[SPRD_FD_DVFS_INDEX6] = {FD_REG_DVFS_INDEX6_MAP,
				    SPRD_FD_CLK_FREQ_384M, 6, 2},
	[SPRD_FD_DVFS_INDEX7] = {FD_REG_DVFS_INDEX7_MAP,
				    SPRD_FD_CLK_FREQ_384M, 6, 2},
};
#endif
static void fd_regmap_on(struct fd_drv *handle,
			unsigned int index)
{
	regmap_update_bits(handle->syscon[index].reg_map,
		handle->syscon[index].reg, handle->syscon[index].mask,
		handle->syscon[index].mask);
}

static void fd_regmap_off(struct fd_drv *handle,
			unsigned int index)
{
	regmap_update_bits(handle->syscon[index].reg_map,
		handle->syscon[index].reg, handle->syscon[index].mask,
		~handle->syscon[index].mask);
}

#if FD_DVFS_ENABLED

static void fd_dvfs_map_cfg(struct fd_drv *drv_handle)
{
	int index = 0;
	unsigned int mask = 0;
	unsigned int val = 0;
	struct sprd_fd_dvfs *fd_dvfs = NULL;

	mask = (FD_MASK_DVFS_CGM_SEL_INDEX) |
	    (FD_MASK_DVFS_VOL_SEL_INDEX) |
	    (FD_MASK_DVFS_VOTE_MTX_SEL_INDEX);

	for (index = 0; index < SPRD_FD_DVFS_INDEX_MAX; index++) {

		fd_dvfs = &fd_dvfs_map[index];
		val = (fd_dvfs->feq_sel <<
			FD_MASK_DVFS_CGM_SEL_START_BIT) |
		    (fd_dvfs->mtx_index <<
		     FD_MASK_DVFS_VOTE_MTX_SEL_START_BIT) |
		    (fd_dvfs->volte << FD_MASK_DVFS_VOL_SEL_START_BIT);
		/*TODO sync with dvfs implementation*/
		FD_REG_MWR(drv_handle->io_dvfs_base,
			fd_dvfs->reg_addr,
			mask, val);
	}

}
#else
static void fd_dvfs_map_cfg(struct fd_drv *drv_handle)
{
	pr_debug("FD_DRV, %s disabled DVFS\n", __func__);
}
#endif
char *fd_syscons[FD_SYSCON_MAX] = {
	"fd_eb",
	"fd_rst",
	"fd_save_rst",
	"fd_lpc",
	"fd_dvfs_eb",
};

static int sprd_fd_config_syscon(struct fd_drv *handle,
	struct device_node *dn)
{
	unsigned int args[2];
	int i = 0;
	int ret = 0;

	for (i = 0; i < FD_SYSCON_MAX; i++) {

		handle->syscon[i].reg_map = syscon_regmap_lookup_by_name(dn,
						fd_syscons[i]);
		if (handle->syscon[i].reg_map == NULL) {
			pr_err("FD_ERR, failed to get syscon %d\n", i);
			return -1;
		}
		ret = syscon_get_args_by_name(dn, fd_syscons[i], 2, args);
		if (ret < 0) {
			pr_err("FD_ERR, fail  get syscon args ret %d i %d\n",
					ret, i);
			return ret;
		}
		handle->syscon[i].reg = args[0];
		handle->syscon[i].mask = args[1];
		pr_info("syscon info  %s  reg 0x%x  mask 0x%x",
			fd_syscons[i], args[0], args[1]);
	}
	return ret;
}

static int sprd_fd_parse_dt(struct fd_drv *handle,
	struct device_node *dn)
{
	struct device_node *qos_node;
	void __iomem *io_base = NULL;
	struct resource res = {0};
	struct platform_device *dev = NULL;

	/*TODO sync with DVFS implementation */
#if FD_DVFS_ENABLED
	void __iomem *io_dvfs_base = NULL;
	unsigned long dvfs_start = 0x62600000;
	unsigned long dvfs_size = 0x10000;
#endif
	int ret = 0;

	pr_info("start fd dts parse\n");

	dev = of_find_device_by_node(dn);
	if (!dev) {
		pr_err("fail to find corresponding device\n");
		return -EINVAL;
	}
	handle->pdev = dev;
	ret = sprd_fd_config_syscon(handle, dn);
	if (ret < 0) {
		pr_err("FD_ERR fail config syscon %d ret\n", ret);
		return ret;
	}
#if 0
	/* TODO to be updated as dvfs is ON */
	handle->cam_dvfs_gpr = syscon_regmap_lookup_by_phandle(dn,
		"sprd,cam-ahb-syscon");
#endif
	pr_info("dev: %s, node: %s, full name: %s\n",
		dev->name, dn->name, dn->full_name);


#ifdef TEST_ON_HAPS
	pr_info("skip parse clock tree on haps.\n");
#else
	handle->fd_eb = of_clk_get_by_name(dn, "fd_eb");
	if (IS_ERR_OR_NULL(handle->fd_eb)) {
		pr_err("read dts fd eb fail\n");
		return -EFAULT;
	}
	handle->clk = of_clk_get_by_name(dn, "fd_clk");
	if (IS_ERR_OR_NULL(handle->clk)) {
		pr_err("read dts fd clk fail\n");
		return -EFAULT;
	}
	handle->clk_parent = of_clk_get_by_name(dn, "fd_clk_parent");
	if (IS_ERR_OR_NULL(handle->clk_parent)) {
		pr_err("read dts fd clk parent fail\n");
		return -EFAULT;
	}
	handle->axi_eb = of_clk_get_by_name(dn, "fd_axi_eb");
	if (IS_ERR_OR_NULL(handle->axi_eb)) {
		pr_err("read dts fd axi eb fail\n");
		return -EFAULT;
	}

	handle->clk_default = clk_get_parent(handle->clk);
#endif

	if (of_address_to_resource(dn, 0, &res)) {
		pr_err("fail to get dcam phys addr\n");
		return -ENXIO;
	}

	io_base = ioremap(res.start, res.end - res.start + 1);
	if (!io_base) {
		pr_err("fail to map fd reg base\n");
		return -ENXIO;
	}
	handle->io_base = (unsigned long)io_base;
	handle->irq_no = irq_of_parse_and_map(dn, 0);
	pr_info("fd_drv iobase 0x%lx  irq_no %d\n",
		handle->io_base, handle->irq_no);
	/*TODO temp implementation for DVFS*/

#if FD_DVFS_ENABLED
	io_dvfs_base = ioremap(dvfs_start, dvfs_size);
	handle->io_dvfs_base = (unsigned long)io_dvfs_base;
	pr_info("fd_drv iodvfs base 0x%lx\n",
		handle->io_dvfs_base);
#endif
	if (handle->irq_no <= 0) {
		pr_err("fail to get fd irq\n");
		return -EFAULT;
	}

	qos_node = of_parse_phandle(dn, "fd_qos", 0);
	if (qos_node) {
		uint8_t val;

		if (of_property_read_u8(qos_node, "arqos", &val)) {
			pr_warn("fd arqos reading fail.\n");
			val = 1;
		}
		handle->arqos = (uint32_t)val;
		if (of_property_read_u8(qos_node, "awqos", &val)) {
			pr_warn("fd awqos reading fail.\n");
			val = 1;
		}
		handle->awqos = (uint32_t)val;
		pr_info("get fd qos node. r: %d w: %d\n",
				handle->arqos, handle->awqos);
	} else {
		handle->arqos = 1;
		handle->awqos = 1;
	}
	return 0;
}


static int sprd_fd_drv_reset(struct fd_drv *handle)
{

	unsigned int time_out = 0;
	unsigned int val = 0;

	/*TODO  code for axi busy wait*/
	while (++time_out < FD_AXI_STOP_TIMEOUT) {
		if (0 == (FD_REG_RD(handle->io_base, FD_AXI_DEBUG) &
			(0xf << 16)))
			break;
	}
	if (time_out >= FD_AXI_STOP_TIMEOUT)
		pr_info("ISP reset timeout axi val 0x%x\n", val);
	fd_regmap_on(handle, FD_SYSCON_RESET);
	udelay(1);
	fd_regmap_off(handle, FD_SYSCON_RESET);

	udelay(1);
	fd_regmap_on(handle, FD_SYSCON_SAVE_RESET);
	udelay(1);
	fd_regmap_off(handle, FD_SYSCON_SAVE_RESET);

	return 0;
}

static int fd_unmap_buf(struct fd_drv *hw_handle)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i <  FD_BUF_INDEX_MAX; i++) {
		if(hw_handle->fd_buf_info[i].mfd != 0 && FD_BUF_CFG_INDEX != i) {
			ret = cam_buf_iommu_unmap(&hw_handle->fd_buf_info[i].buf_info);
			if (ret)
				pr_err("FD_DRV unmap err %d index\n", i);
			cam_buf_ionbuf_put(&hw_handle->fd_buf_info[i].buf_info);
			hw_handle->fd_buf_info[i].mfd = 0;
		}
	}
	return ret;
}

static void sprd_fd_irq_enable(struct fd_drv *hw_handle)
{
	uint32_t mask = FD_IRQ_MASK;

	FD_REG_MWR(hw_handle->io_base, FD_INT_EN, mask,
			mask);
}

static void sprd_fd_irq_disable(struct fd_drv *hw_handle)
{
	uint32_t mask = FD_IRQ_MASK;

	FD_REG_MWR(hw_handle->io_base, FD_INT_EN, mask,
			0);
}

static void  fd_reg_dump(struct fd_drv *handle)
{
	int i = 0;


	for (i = 0; i < FD_SPARE_GATE; i += 4)
		pr_info("0x%x   0x%x\n", i, FD_REG_RD(handle->io_base, i));

	for (i = FD_MODEL_CFG_EN; i < FD_MODEL67_FINE_NUM; i += 4)
		pr_info("0x%x   0x%x\n", i, FD_REG_RD(handle->io_base, i));

	for (i = FD_INT_EN; i < FD_MMU_READ_PAGE_CMD_CNT; i += 4)
		pr_info("0x%x   0x%x\n", i, FD_REG_RD(handle->io_base, i));

	for (i = FD_MMU_INT_EN; i < FD_MMU_INT_RAW; i += 4)
		pr_info("0x%x   0x%x\n", i, FD_REG_RD(handle->io_base, i));

	for (i = FD_MMU_VERSION; i <= FD_MMU_INT_RAW; i += 4)
		pr_info("0x%x   0x%x\n", i, FD_REG_RD(handle->io_base, i));
}

static irqreturn_t fd_isr_root(int irq, void *priv)
{
	uint32_t status = 0;
	struct fd_drv *hw_handle = (struct fd_drv *)priv;
	uint32_t err_mask = FD_MASK_INT_ERR;
	uint32_t irq_mask = FD_IRQ_MASK;

	status = FD_REG_RD(hw_handle->io_base, FD_INT_RAW);

	pr_debug("FD_INT int 0x%x\n", status);
	/*clear the interrupt*/
	FD_REG_MWR(hw_handle->io_base, FD_INT_CLR, irq_mask, status);

	if ((status & err_mask) != 0) {
		pr_err("FD_ERR int error 0x%x\n", status);
		hw_handle->state = SPRD_FD_STATE_ERROR;
		fd_reg_dump(hw_handle);

		/*print_reg_trace*/
	}
	if (hw_handle->state != SPRD_FD_STATE_ERROR &&
			hw_handle->state == SPRD_FD_STATE_RUNNING)
		hw_handle->state = SPRD_FD_STATE_IDLE;
	fd_unmap_buf(hw_handle);
	complete(&hw_handle->fd_wait_com);
	return IRQ_HANDLED;
}

static int fd_disable_clk(struct fd_drv *handle)
{
	int ret = 0;

	pr_debug(",fd disable clk enter\n");
	if (!handle) {
		pr_err("param erro\n");
		return -EINVAL;
	}
#ifndef TEST_ON_HAPS
	clk_set_parent(handle->clk, handle->clk_default);
	clk_disable_unprepare(handle->clk);
	clk_disable_unprepare(handle->axi_eb);
	clk_disable_unprepare(handle->fd_eb);
#endif

	return ret;
}

static int fd_clk_enable(struct fd_drv *handle)
{
	int ret = 0;

	pr_debug(",fd_clk enable enter\n");
	if (!handle) {
		pr_err("param erro\n");
		return -EINVAL;
	}
#ifndef TEST_ON_HAPS
	ret = clk_set_parent(handle->clk, handle->clk_parent);
	if (ret) {
		pr_err("set fd parent fail, ret = %d\n", ret);
		clk_set_parent(handle->clk, handle->clk_default);
		return ret;
	}
	ret = clk_prepare_enable(handle->clk);
	if (ret) {
		pr_err("enable fd clk fail, ret = %d\n", ret);
		clk_set_parent(handle->clk, handle->clk_default);
		return ret;
	}
	ret = clk_prepare_enable(handle->fd_eb);
	if (ret) {
		pr_err("set fd eb fail, ret = %d\n", ret);
		clk_disable_unprepare(handle->clk);
		return ret;
	}
	ret = clk_prepare_enable(handle->axi_eb);
	if (ret) {
		pr_err("set fd axi eb fail, ret = %d\n", ret);
		clk_disable_unprepare(handle->clk);
		clk_disable_unprepare(handle->fd_eb);
		return ret;
	}
#endif

	return ret;
}
int sprd_fd_drv_open(void *drv_handle)
{
	struct fd_drv *hw_handle = NULL;
	int ret = 0;

	hw_handle  = (struct fd_drv *)drv_handle;
	ret = devm_request_irq(&hw_handle->pdev->dev, hw_handle->irq_no,
		fd_isr_root, IRQF_SHARED, "FD", (void *)hw_handle);
	if (ret) {
		pr_err("FD_ERR: irq register failed %d\n", ret);
		return ret;
	}

	if (atomic_inc_return(&hw_handle->pw_users) == 1) {
		ret = sprd_cam_pw_on();
		if (ret != 0) {
			pr_err("FD_ERR: sprd cam_sys power on failed %d\n",
							ret);
			return ret;
		}
		ret = sprd_cam_domain_eb();
		fd_clk_enable(hw_handle);
		fd_regmap_on(hw_handle, FD_SYSCON_ENABLE);
#if FD_DVFS_ENABLED
		fd_regmap_on(hw_handle, FD_SYSCON_DVFS_ENABLE);
#endif
		ret = sprd_fd_drv_reset(hw_handle);
		if (ret) {
			pr_err("FD_DRV err, drv open reset failed ret %d",
								ret);
			return ret;
		}
		sprd_iommu_restore(&hw_handle->pdev->dev);
		FD_REG_MWR(hw_handle->io_base, FD_INT_CLR, FD_IRQ_MASK,
				FD_IRQ_MASK);
		fd_dvfs_map_cfg(hw_handle);
#if 0
		/*TODO sync with low power*/
		fd_regmap_on(hw_handle, FD_SYSCON_LPC);
#endif

		/*TODO sync with dvfs implementation*/
#if FD_DVFS_ENABLED
		FD_REG_MWR(hw_handle->io_dvfs_base,
			FD_REG_DVFS_DFS_EN_CTRL,
			FD_MASK_DVFS_DFS_EN, 1);
		FD_REG_MWR(hw_handle->io_dvfs_base,
			FD_REG_DVFS_DFS_SW_TRIG_CFG,
			FD_MASK_DVFS_DFS_SW_TRIG, 1);
		FD_REG_MWR(hw_handle->io_dvfs_base,
			FD_REG_DVFS_DFS_FREQ_UPDATE_BYPASS,
			FD_MASK_DVFS_DFS_FREQ_UPDATE, 1);
#endif
		/* set qos parsed from dts */
		FD_REG_MWR(hw_handle->io_base, FD_MOD_CFG,
			FD_MASK_MOD_CFG_ARQOS | FD_MASK_MOD_CFG_AWQOS,
			((hw_handle->arqos & 0xf) << 20) |
			((hw_handle->awqos & 0xf) << 16));
	}
	init_completion(&hw_handle->fd_wait_com);
	sprd_fd_irq_enable(hw_handle);

	if (hw_handle->state == SPRD_FD_STATE_CLOSED)
		hw_handle->state = SPRD_FD_STATE_IDLE;
	return ret;
}

int sprd_fd_drv_close(void *drv_handle)
{
	struct fd_drv *hw_handle = NULL;
	int ret = 0;
	int i = 0;

	hw_handle  = (struct fd_drv *)drv_handle;

	for (i = 0; i <  FD_BUF_INDEX_MAX; i++) {
		if(hw_handle->fd_buf_info[i].mfd != 0 && (hw_handle->fd_buf_info[i].buf_info.mapping_state & CAM_BUF_MAPPING_DEV)) {
			ret = cam_buf_iommu_unmap(&hw_handle->fd_buf_info[i].buf_info);
			if (ret)
				pr_err("fail to unmap FD_BUF_CFG_INDEX %d\n", i);

			cam_buf_ionbuf_put(&hw_handle->fd_buf_info[i].buf_info);
			hw_handle->fd_buf_info[i].mfd = 0;
		}
	}

	devm_free_irq(&hw_handle->pdev->dev, hw_handle->irq_no,
			(void *)hw_handle);
	if (atomic_dec_return(&hw_handle->pw_users) == 0) {

		sprd_fd_irq_disable(hw_handle);
		/*wait if fd is busy*/
		ret = sprd_fd_drv_reset(hw_handle);
		if (ret)
			pr_err("FD_ERR: close reset failed\n");

		/*TODO sync with dvfs implementation*/
#if FD_DVFS_ENABLED
		FD_REG_MWR(hw_handle->io_dvfs_base,
			FD_REG_DVFS_DFS_EN_CTRL,
			FD_MASK_DVFS_DFS_EN, 0);
		FD_REG_MWR(hw_handle->io_dvfs_base,
			FD_REG_DVFS_DFS_SW_TRIG_CFG,
			FD_MASK_DVFS_DFS_SW_TRIG, 0);
		FD_REG_MWR(hw_handle->io_dvfs_base,
			FD_REG_DVFS_DFS_FREQ_UPDATE_BYPASS,
			FD_MASK_DVFS_DFS_FREQ_UPDATE, 0);
		fd_regmap_off(hw_handle, FD_SYSCON_DVFS_ENABLE);
#endif
		fd_regmap_off(hw_handle, FD_SYSCON_ENABLE);
		fd_disable_clk(hw_handle);
		sprd_cam_domain_disable();
		ret = sprd_cam_pw_off();
		if (ret != 0) {
			pr_err("FD_ERR: sprd cam_sys power off failed\n");
			return ret;
		}
	}
	if (hw_handle->state == SPRD_FD_STATE_IDLE)
		hw_handle->state = SPRD_FD_STATE_CLOSED;
	complete(&hw_handle->fd_wait_com);
	return ret;
}


int sprd_fd_drv_init(void **fd_handle,
	struct device_node *dn)
{
	struct fd_drv *handle = NULL;
	int ret = 0;

	handle = kzalloc(sizeof(struct fd_drv), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;
	ret = sprd_fd_parse_dt(handle, dn);
	cam_buf_iommudev_reg(&handle->pdev->dev, CAM_IOMMUDEV_FD);
	if (ret) {
		pr_err("FD:ERR parse dt failed");
		kfree(handle);
		return ret;
	}
	*fd_handle = handle;

	return ret;
}

int sprd_fd_drv_deinit(void *fd_handle)
{
	struct fd_drv *hw_handle = NULL;

	hw_handle = (struct fd_drv *)fd_handle;
	if (hw_handle == NULL)
		return -ENOMEM;
	cam_buf_iommudev_unreg(CAM_IOMMUDEV_FD);

	kfree(hw_handle);
	return 0;
}

static int fd_write_run(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	val &= FD_MASK_RUN;
	/*fd_reg_dump(hw_handle);*/
	FD_REG_WR(hw_handle->io_base, reg_addr, val);
	/*fd_reg_dump(hw_handle);*/

	if (hw_handle->state == SPRD_FD_STATE_IDLE)
		hw_handle->state = SPRD_FD_STATE_RUNNING;
	return 0;
}

static int fd_int_eb(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	val &= FD_MASK_EB;

	FD_REG_WR(hw_handle->io_base, reg_addr, val);

	if (hw_handle->state == SPRD_FD_STATE_IDLE)
		hw_handle->state = SPRD_FD_STATE_RUNNING;
	return 0;
}

static int fd_int_clr(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	val &= FD_MASK_CLR;

	FD_REG_WR(hw_handle->io_base, reg_addr, val);

	if (hw_handle->state == SPRD_FD_STATE_IDLE)
		hw_handle->state = SPRD_FD_STATE_RUNNING;
	return 0;
}

static int fd_get_buf(unsigned int val,
		struct camera_buf *buf_info)
{
	int ret = 0;

	buf_info->mfd[0] = val;
	buf_info->mfd[1] = 0;
	buf_info->mfd[2] = 0;
	buf_info->type = CAM_BUF_USER;
	ret = cam_buf_ionbuf_get(buf_info);
	return ret;
}

static int fd_write_mod_cfg(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	unsigned int mask = 0;

	/* todo: qos should be set by user, should be parsed from dts */
	val &= ~(FD_MASK_MOD_CFG_ARQOS | FD_MASK_MOD_CFG_AWQOS);
	val |= ((hw_handle->arqos & 0xf) << 20);
	val |= ((hw_handle->awqos & 0xf) << 16);

	mask = FD_MASK_MOD_CFG_ARQOS | FD_MASK_MOD_CFG_AWQOS |
		FD_MASK_MOD_CFG_OUT_END | FD_MASK_MOD_CFG_IN_END |
		FD_MASK_MOD_CFG_CMD_END;

	/*FD_REG_MWR(hw_handle->io_base, reg_addr, mask, val);*/
	FD_REG_WR(hw_handle->io_base, reg_addr, val);

	return 0;
}
static int fd_write_baddr(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	/* get the phys addr using the mfd*/
	unsigned long phys_addr = 0;
	enum FD_BUF_INDEX index = FD_BUF_INDEX_MAX;
	int ret = 0;

	switch (reg_addr) {

	case FD_CMD_BADDR:
		index =  FD_BUF_CMD_INDEX;
		break;
	case FD_IMAGE_BADDR:
		index =  FD_BUF_IMG_INDEX;
		break;
	case FD_OUT_BUF_ADDR:
		index =  FD_BUF_OUT_INDEX;
		break;
	case FD_DIM_BUF_ADDR:
		index =  FD_BUF_DIM_INDEX;
		break;
	case FD_MODEL_CFG_BADDR:
		index = FD_BUF_CFG_INDEX;
		break;
	default:
		pr_err("fd_drv write address error 0x%x\n", reg_addr);
		return -1;
	}

	hw_handle->fd_buf_info[index].mfd = val;
	ret = fd_get_buf(val, &hw_handle->fd_buf_info[index].buf_info);
	if (ret) {
		pr_err("FD_DRV iommu get buf err index %d\n", index);
		return ret;
	}
	ret = cam_buf_iommu_map(&hw_handle->fd_buf_info[index].buf_info,
		CAM_IOMMUDEV_FD);
	if (ret) {
		pr_err("FD_DRV iommu map err index %d\n", index);
		return ret;
	}
	phys_addr = hw_handle->fd_buf_info[index].buf_info.iova[0];
	phys_addr &= FD_MASK_BADDR;
	pr_debug("fd phys addr mask 0x%lx  index %d\n", phys_addr, index);
	FD_REG_WR(hw_handle->io_base, reg_addr, phys_addr);
	return 0;
}

static int fd_write_cmd_num(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	unsigned int mask = 0;

	mask = FD_MASK_CMD_TOT_NUM;
	FD_REG_MWR(hw_handle->io_base, reg_addr, mask, val);
	return 0;
}

static int fd_write_image_linenum(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	unsigned int mask = 0;

	mask  = FD_MASK_IMG_LINESTEP;
	FD_REG_MWR(hw_handle->io_base, reg_addr, mask, val);
	return 0;
}

static int fd_write_face_num(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{

	unsigned int mask = 0;

	mask  = FD_MASK_FACE_MAX_NUM;
	FD_REG_MWR(hw_handle->io_base, reg_addr, mask, val);
	return 0;
}

static int fd_write_model_cfg_en(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	unsigned int mask = 0;
	mask = FD_MASK_MODEL_CFG_EN;
	FD_REG_MWR(hw_handle->io_base, reg_addr, mask, val);
	return 0;
}
static int fd_write_model_fine(struct fd_drv *hw_handle,
			unsigned int reg_addr,
			unsigned int val)
{
	unsigned int mask = 0;
	mask = FD_MASK_MODEL_0_FINE | FD_MASK_MODEL_1_FINE;
	FD_REG_MWR(hw_handle->io_base, reg_addr, mask, val);
	return 0;
}

static struct fd_reg_info fd_reg_info_table[SPRD_FD_REG_PARAM_MAX] = {
	[SPRD_FD_REG_PARAM_RUN] = {FD_RUN, fd_write_run},
	[SPRD_FD_REG_PARAM_INFO] = {FD_INFO, NULL},
	[SPRD_FD_REG_PARAM_MOD_CFG] = {FD_MOD_CFG, fd_write_mod_cfg},
	[SPRD_FD_REG_PARAM_CMD_BADDR] = {FD_CMD_BADDR, fd_write_baddr},
	[SPRD_FD_REG_PARAM_CMD_NUM] = {FD_CMD_NUM, fd_write_cmd_num},
	[SPRD_FD_REG_PARAM_IMAGE_BADDR] = {FD_IMAGE_BADDR, fd_write_baddr},
	[SPRD_FD_REG_PARAM_IMAGE_LINENUM] = {FD_IMAGE_LINENUM,
					fd_write_image_linenum},
	[SPRD_FD_REG_PARAM_FACE_NUM] = {FD_FACE_NUM, fd_write_face_num},
	[SPRD_FD_REG_PARAM_OUT_BUF_ADDR] = {FD_OUT_BUF_ADDR, fd_write_baddr},
	[SPRD_FD_REG_PARAM_DIM_BUF_ADDR] = {FD_DIM_BUF_ADDR, fd_write_baddr},
	[SPRD_FD_REG_PARAM_CROP_START_STS] = {FD_CROP_START_STS, NULL},
	[SPRD_FD_REG_PARAM_CROP_SIZE_STS] = {FD_CROP_SIZE_STS, NULL},
	[SPRD_FD_REG_PARAM_DIM_STS] = {FD_DIM_STS, NULL},
	[SPRD_FD_REG_PARAM_WKC_STS] = {FD_WKC_STS, NULL},
	[SPRD_FD_REG_PARAM_PAD_STS] = {FD_PAD_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL_CUT0_STS] = {FD_MODEL_CUT0_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL_CUT1_STS] = {FD_MODEL_CUT1_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL0_STS] = {FD_MODEL0_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL1_STS] = {FD_MODEL1_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL2_STS] = {FD_MODEL2_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL3_STS] = {FD_MODEL3_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL4_STS] = {FD_MODEL4_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL5_STS] = {FD_MODEL5_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL6_STS] = {FD_MODEL6_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL7_STS] = {FD_MODEL7_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL8_STS] = {FD_MODEL8_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL9_STS] = {FD_MODEL9_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL10_STS] = {FD_MODEL10_STS, NULL},
	[SPRD_FD_REG_PARAM_MODEL11_STS] = {FD_MODEL11_STS, NULL},
	[SPRD_FD_REG_PARAM_IP_REV] = {FD_IP_REV, NULL},
	[SPRD_FD_REG_PARAM_AXI_DEBUG] = {FD_AXI_DEBUG, NULL},
	[SPRD_FD_REG_PARAM_BUSY_DEBUG] = {FD_BUSY_DEBUG, NULL},
	[SPRD_FD_REG_PARAM_DET_DEBUG0] = {FD_DET_DEBUG0, NULL},
	[SPRD_FD_REG_PARAM_DET_DEBUG1] = {FD_DET_DEBUG1, NULL},
	[SPRD_FD_REG_PARAM_SCL_DEBUG] = {FD_SCL_DEBUG, NULL},
	[SPRD_FD_REG_PARAM_FEAT_DEBUG0] = {FD_FEAT_DEBUG0, NULL},
	[SPRD_FD_REG_PARAM_FEAT_DEBUG1] = {FD_FEAT_DEBUG1, NULL},
	[SPRD_FD_REG_PARAM_SPARE_GATE] = {FD_SPARE_GATE, NULL},
	[SPRD_FD_REG_PARAM_CFG_EN] = {FD_MODEL_CFG_EN, fd_write_model_cfg_en},
	[SPRD_FD_REG_PARAM_CFG_BADDR] = {FD_MODEL_CFG_BADDR, fd_write_baddr},
	[SPRD_FD_REG_PARAM_MODEL_01_FINE] = {FD_MODEL01_FINE_NUM,
					fd_write_model_fine},
	[SPRD_FD_REG_PARAM_MODEL_23_FINE] = {FD_MODEL23_FINE_NUM,
					fd_write_model_fine},
	[SPRD_FD_REG_PARAM_MODEL_45_FINE] = {FD_MODEL45_FINE_NUM,
					fd_write_model_fine},
	[SPRD_FD_REG_PARAM_MODEL_67_FINE] = {FD_MODEL67_FINE_NUM,
					fd_write_model_fine},
	[SPRD_FD_REG_PARAM_INT_EN] = {FD_INT_EN, fd_int_eb},
	[SPRD_FD_REG_PARAM_INT_CLR] = {FD_INT_CLR, fd_int_clr},
	[SPRD_FD_REG_PARAM_INT_RAW] = {FD_INT_RAW, NULL},
	[SPRD_FD_REG_PARAM_INT_MASK] = {FD_INT_MASK, NULL},
};

static int fd_post_write_proc(struct fd_drv *hw_handle, unsigned int reg_param)
{
	int ret = 0;
	int left_time = 0;

	switch (reg_param) {
	case SPRD_FD_REG_PARAM_RUN:
		left_time = wait_for_completion_timeout(
				&hw_handle->fd_wait_com, msecs_to_jiffies(FD_INT_TIMEOUT));
		if(left_time == 0){
			ret = -EBUSY;
			fd_reg_dump(hw_handle);
			pr_err("FD INT timeout reached\n");
		}
		break;
	default:
		break;

	}
	return ret;
}

#if FD_DVFS_ENABLED
/* We skip the dvfs clk setting just
 * before writing to the hardware.
 * We do this here as we cannnot control the
 * IOCTL calling in user space
*/
int fd_drv_dvfs_idle_clk_cfg(void *handle,
	unsigned int index)
{
	struct fd_drv *drv_handle = NULL;

	drv_handle  = (struct fd_drv *)handle;
	if (index >= SPRD_FD_DVFS_INDEX_MAX) {
		pr_err("FD_ERR: invalid clk index %u\n",
			index);
		return -EINVAL;
	}
	/*TODO sync with dvfs implemenatation*/

	FD_REG_MWR(drv_handle->io_dvfs_base,
		FD_REG_DVFS_IDLE_INDEX_CFG,
		FD_MASK_DVFS_INDEX_IDLE, index);
	return 0;
}

int fd_drv_dvfs_work_clk_cfg(void *handle,
	unsigned int index)
{
	struct fd_drv *drv_handle = NULL;


	if (index >= SPRD_FD_DVFS_INDEX_MAX) {
		pr_err("FD_ERR: invalid clk index %u\n", index);
		return -EINVAL;
	}

	drv_handle  = (struct fd_drv *)handle;
	FD_REG_MWR(drv_handle->io_dvfs_base,
		FD_REG_DVFS_WORK_INDEX_CFG,
		FD_MASK_DVFS_INDEX_WORK, index);
	return 0;
}

#else

int fd_drv_dvfs_idle_clk_cfg(void *handle,
	unsigned int index)
{
	pr_warn("FD_DRV, %s WARN: disabled DVFS\n", __func__);
	return 0;
}

int fd_drv_dvfs_work_clk_cfg(void *handle,
	unsigned int index)
{
	pr_warn("FD_DRV, %s WARN: disabled DVFS\n", __func__);
	return 0;
}
#endif

int fd_drv_reg_write_handler(void *handle,
	struct sprd_fd_cfg_param  *cfg_param)
{
	int ret  = 0;
	struct fd_reg_info *reg_info = NULL;
	struct fd_drv *drv_handle = NULL;

	drv_handle = (struct fd_drv *)handle;
	reg_info  =  &fd_reg_info_table[cfg_param->reg_param];

	if (reg_info->reg_write == NULL) {
		pr_err("FD_ERR: invalid reg write fun %d\n",
				cfg_param->reg_param);
		return -EINVAL;
	}
	ret = reg_info->reg_write(drv_handle,
			reg_info->reg_addr,
			cfg_param->reg_val);
	if (ret) {
		pr_err("FD_ERR: write reg 0x%x val 0x%x\n",
				reg_info->reg_addr, cfg_param->reg_val);
		return ret;
	}

	fd_post_write_proc(drv_handle, cfg_param->reg_param);
	return ret;
}

int fd_drv_reg_multi_write_handler(void *handle,
		struct sprd_fd_multi_reg_cfg_param *param)
{
	int i = 0;
	struct sprd_fd_cfg_param  param_tab[SPRD_FD_REG_PARAM_MAX];
	struct fd_reg_info *reg_info;
	int ret = 0;
	struct fd_drv *drv_handle = NULL;

	drv_handle = (struct fd_drv *)handle;

	ret = copy_from_user(param_tab,
			(struct sprd_fd_cfg_param *__user)param->reg_info_tab,
			param->size * sizeof(struct sprd_fd_cfg_param));
	if (param->size > SPRD_FD_REG_PARAM_MAX) {
		pr_err("FD_ERR: multi write size overbound\n");
		return -EINVAL;
	}
	for (i = 0; i < param->size; i++) {
		reg_info  =  &fd_reg_info_table[param_tab[i].reg_param];
		if (reg_info->reg_write == NULL) {
			pr_err("FD_ERR: invalid multi reg write fun %d\n",
					param_tab[i].reg_param);
			continue;
		}
		ret = reg_info->reg_write(drv_handle,
				reg_info->reg_addr, param_tab[i].reg_val);
		if (ret) {
			pr_err("FD_ERR: write reg 0x%x val 0x%x\n",
					reg_info->reg_addr,
					param_tab[i].reg_val);
			return ret;
		}
	}
	return ret;
}

int fd_drv_reg_read_handler(void *handle, unsigned long arg)
{
	int ret  = 0;
	struct fd_drv *drv_handle = NULL;
	struct fd_reg_info *reg_info = NULL;
	struct sprd_fd_cfg_param  cfg_param;

	ret = copy_from_user(&cfg_param,
			(struct sprd_fd_cfg_param *__user)arg,
			sizeof(struct sprd_fd_cfg_param));
	reg_info  =  &fd_reg_info_table[cfg_param.reg_param];

	drv_handle = (struct fd_drv *)handle;
	cfg_param.reg_val = FD_REG_RD(drv_handle->io_base, reg_info->reg_addr);
	ret = copy_to_user((void __user *)arg,
			&cfg_param, sizeof(struct sprd_fd_cfg_param));
	return ret;
}

int fd_drv_reg_multi_read_handler(void *handle,
	    struct sprd_fd_multi_reg_cfg_param  *param)
{
	int i = 0;
	struct sprd_fd_cfg_param  param_tab[SPRD_FD_REG_PARAM_MAX];
	struct fd_reg_info *reg_info;
	int ret = 0;
	struct fd_drv *drv_handle = NULL;

	drv_handle = (struct fd_drv *)handle;

	ret = copy_from_user(param_tab,
			(struct sprd_fd_cfg_param *__user)param->reg_info_tab,
			param->size * sizeof(struct sprd_fd_cfg_param));

	if (param->size > SPRD_FD_REG_PARAM_MAX) {
		pr_err("FD_ERR: multi read size overbound\n");
		return -EINVAL;
	}
	for (i = 0; i < param->size; i++) {
		reg_info  =  &fd_reg_info_table[param_tab[i].reg_param];
		param_tab[i].reg_val = FD_REG_RD(drv_handle->io_base,
				reg_info->reg_addr);
	}
	ret = copy_to_user(
			(struct sprd_fd_cfg_param __user *)param->reg_info_tab,
			param_tab,
			param->size * sizeof(struct sprd_fd_cfg_param)
			);
	return ret;
}
