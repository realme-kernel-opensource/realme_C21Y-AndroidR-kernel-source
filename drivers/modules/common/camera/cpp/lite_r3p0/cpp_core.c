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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <video/sprd_mmsys_pw_domain.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "cpp_common.h"
#include "cpp_reg.h"
#include "sprd_cpp.h"
#include "rot_drv.h"
#include "scale_drv.h"
#include "dma_drv.h"


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CPP_CORE: %d %d %s: " \
	fmt, current->pid, __LINE__, __func__

#define CPP_DEVICE_NAME         "sprd_cpp"
#define ROT_TIMEOUT             5000
#define SCALE_TIMEOUT           5000
#define DMA_TIMEOUT             5000

/*#define DEBUG_CPP_MMU*/

#ifdef DEBUG_CPP_MMU
#define CPP_IRQ_LINE_MASK       0x7FFUL
#else
#define CPP_IRQ_LINE_MASK       0x7UL
#endif

static const char * const syscon_name[] = {
	"cpp_rst"
};

enum cpp_irq_id {
	CPP_SCALE_DONE = 0,
	CPP_ROT_DONE,
	CPP_DMA_DONE,
#ifdef DEBUG_CPP_MMU
	CPP_MMU_0,
	CPP_MMU_1,
	CPP_MMU_2,
	CPP_MMU_3,
	CPP_MMU_4,
	CPP_MMU_5,
	CPP_MMU_6,
	CPP_MMU_7,
#endif
	CPP_IRQ_NUMBER
};

struct dmaif_device {
	atomic_t count;
	struct mutex dma_mutex;
	struct completion done_com;
	struct dma_drv_private drv_priv;
};

struct rotif_device {
	atomic_t count;
	struct mutex rot_mutex;
	struct completion done_com;
	struct rot_drv_private drv_priv;
};

struct scif_device {
	atomic_t count;
	struct mutex sc_mutex;
	struct completion done_com;
	struct scale_drv_private drv_priv;
};

typedef void (*cpp_isr_func) (void *);

struct register_gpr {
	struct regmap *gpr;
	unsigned int reg;
	unsigned int mask;
};

struct cpp_device {
	atomic_t users;
	spinlock_t hw_lock;
	struct mutex lock;
	spinlock_t slock;

	struct rotif_device *rotif;
	struct scif_device *scif;
	struct dmaif_device *dmaif;
	struct miscdevice md;

	int irq;
	cpp_isr_func isr_func[CPP_IRQ_NUMBER];
	void *isr_data[CPP_IRQ_NUMBER];

	void __iomem *io_base;

	struct platform_device *pdev;

	struct clk *cpp_clk;
	struct clk *cpp_clk_parent;
	struct clk *cpp_clk_default;

	struct clk *cpp_eb;
	struct clk *cpp_axi_eb;
	struct clk *clk_mm_eb;

	struct register_gpr syscon_regs[ARRAY_SIZE(syscon_name)];
};

typedef void (*cpp_isr) (struct cpp_device *dev);

static void cpp_scale_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_SCALE_DONE];
	void *priv = dev->isr_data[CPP_SCALE_DONE];

	if (user_func)
		(*user_func) (priv);
}

static void cpp_rot_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_ROT_DONE];
	void *priv = dev->isr_data[CPP_ROT_DONE];

	if (user_func)
		(*user_func) (priv);
}

static void cpp_dma_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_DMA_DONE];
	void *priv = dev->isr_data[CPP_DMA_DONE];

	if (user_func)
		(*user_func) (priv);
}

#ifdef DEBUG_CPP_MMU
static void cpp_mmu_status(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_MMU_0];
	void *priv = dev->isr_data[CPP_MMU_0];

	if (user_func)
		(*user_func) (priv);
}
#endif

static const cpp_isr cpp_isr_list[CPP_IRQ_NUMBER] = {
	cpp_scale_done,/* 0 */
	cpp_rot_done,
	cpp_dma_done,
#ifdef DEBUG_CPP_MMU
	cpp_mmu_status,
	cpp_mmu_status,
	cpp_mmu_status,/* 5 */
	cpp_mmu_status,
	cpp_mmu_status,
	cpp_mmu_status,
	cpp_mmu_status,
	cpp_mmu_status,/* 10 */
#endif
};

static irqreturn_t cpp_isr_root(int irq, void *priv)
{
	int i;
	unsigned int irq_line, status;
	unsigned long flag;
	struct cpp_device *dev = (struct cpp_device *)priv;

	status = reg_rd(dev, CPP_INT_STS) & CPP_IRQ_LINE_MASK;
	if (unlikely(status == 0))
		return IRQ_NONE;

	irq_line = status;
	reg_wr(dev, CPP_INT_CLR, status);

	spin_lock_irqsave(&dev->slock, flag);
	for (i = 0; i < CPP_IRQ_NUMBER; i++) {
		if (irq_line & (1 << (unsigned int)i)) {
			if (cpp_isr_list[i])
				cpp_isr_list[i](dev);
		}
		irq_line &= ~(unsigned int)(1 << (unsigned int)i);
		if (!irq_line)
			break;
	}
	spin_unlock_irqrestore(&dev->slock, flag);

	return IRQ_HANDLED;
}

static int cpp_module_enable(struct cpp_device *dev)
{
	int ret = 0;

	mutex_lock(&dev->lock);

	if (atomic_read(&dev->users) == 1) {
		ret = clk_prepare_enable(dev->cpp_eb);
		if (ret)
			goto fail;

		ret = clk_prepare_enable(dev->cpp_axi_eb);
		if (ret)
			goto fail;

		ret = clk_set_parent(dev->cpp_clk, dev->cpp_clk_parent);
		if (ret) {
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		ret = clk_prepare_enable(dev->cpp_clk);
		if (ret) {
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		regmap_update_bits(dev->syscon_regs[0].gpr,
			dev->syscon_regs[0].reg,
			dev->syscon_regs[0].mask,
			dev->syscon_regs[0].mask);
		udelay(2);
		regmap_update_bits(dev->syscon_regs[0].gpr,
			dev->syscon_regs[0].reg,
			dev->syscon_regs[0].mask,
			~dev->syscon_regs[0].mask);

		reg_awr(dev, CPP_MMU_EN, (0xfffffffe));
		reg_owr(dev, MMU_PPN_RANGE1, (0xfff));
		reg_owr(dev, MMU_PPN_RANGE2, (0xfff));
	}
	mutex_unlock(&dev->lock);
	return ret;

fail:
	pr_err("fail to enable cpp clk\n");
	mutex_unlock(&dev->lock);

	return ret;
}

static void cpp_module_disable(struct cpp_device *dev)
{
	mutex_lock(&dev->lock);

	clk_set_parent(dev->cpp_clk, dev->cpp_clk_default);
	clk_disable_unprepare(dev->cpp_clk);
	clk_disable_unprepare(dev->cpp_axi_eb);
	clk_disable_unprepare(dev->cpp_eb);

	mutex_unlock(&dev->lock);
}

static void cpp_register_isr(struct cpp_device *dev, enum cpp_irq_id id,
			cpp_isr_func user_func, void *priv)
{
	unsigned long flag;

	if (id < CPP_IRQ_NUMBER) {
		spin_lock_irqsave(&dev->slock, flag);
		dev->isr_func[id] = user_func;
		dev->isr_data[id] = priv;
		if (user_func)
			reg_mwr(dev, CPP_INT_MASK, (1 << id), ~(1 << id));
		else
			reg_mwr(dev, CPP_INT_MASK, (1 << id), (1 << id));
		spin_unlock_irqrestore(&dev->slock, flag);
	} else {
		pr_err("fail to get valid cpp isr irq\n");
	}
}

static void rot_isr(void *priv)
{
	struct rotif_device *rotif = (struct rotif_device *)priv;

	if (!rotif)
		return;

	complete(&rotif->done_com);
}

static void scale_isr(void *priv)
{
	struct scif_device *scif = (struct scif_device *)priv;

	if (!scif)
		return;

	complete(&scif->done_com);
}

static void dma_isr(void *priv)
{
	struct dmaif_device *dmaif = (struct dmaif_device *)priv;

	if (!dmaif)
		return;

	complete(&dmaif->done_com);
}

#ifdef DEBUG_CPP_MMU
static void mmu_isr(void *priv)
{
	pr_err("fail to get cpp MMU status!!!\n");
}
#endif

int cpp_get_sg_table(struct cpp_iommu_info *pfinfo)
{
	int i, ret;

	for (i = 0; i < 2; i++) {
		if (pfinfo->mfd[i] > 0) {
			ret = sprd_ion_get_buffer(pfinfo->mfd[i],
						NULL,
						&pfinfo->buf[i],
						&pfinfo->size[i]);
			if (ret) {
				pr_err("fail to get sg table\n");
				return -EFAULT;
			}

		}
	}
	return 0;
}

int cpp_get_addr(struct cpp_iommu_info *pfinfo)
{
	int i, ret;
	struct sprd_iommu_map_data iommu_data;

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			memset(&iommu_data, 0,
				sizeof(struct sprd_iommu_map_data));
			iommu_data.buf = pfinfo->buf[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.sg_offset = pfinfo->offset[i];

			ret = sprd_iommu_map(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("fail to get iommu kaddr %d\n", i);
				return -EFAULT;
			}

			pfinfo->iova[i] = iommu_data.iova_addr
					+ pfinfo->offset[i];
		} else {
			ret = sprd_ion_get_phys_addr(pfinfo->mfd[i],
					NULL,
					&pfinfo->iova[i],
					&pfinfo->size[i]);
			if (ret) {
				pr_err("fail to get iommu phy addr %d mfd 0x%x\n",
				       i, pfinfo->mfd[i]);
				return -EFAULT;
			}
			pfinfo->iova[i] += pfinfo->offset[i];
		}
	}

	return 0;
}

int cpp_free_addr(struct cpp_iommu_info *pfinfo)
{
	int i, ret;
	struct sprd_iommu_unmap_data iommu_data;

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			iommu_data.iova_addr = pfinfo->iova[i]
					- pfinfo->offset[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.buf = NULL;

			ret = sprd_iommu_unmap(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("fail to free iommu %d\n", i);
				return -EFAULT;
			}

		}
	}

	return 0;
}

static long sprd_cpp_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	unsigned long rtn = 0;
	struct cpp_device *dev = NULL;
	struct rotif_device *rotif = NULL;
	struct scif_device *scif = NULL;
	struct dmaif_device *dmaif = NULL;
	struct sprd_cpp_rot_cfg_parm rot_parm;
	struct sprd_cpp_scale_cfg_parm *sc_parm;
	struct sprd_cpp_dma_cfg_parm dma_parm;
	struct sprd_cpp_scale_capability sc_cap_param;
	struct sprd_cpp_size s_sc_cap = {0, 0};
	int cpp_dimension = 0;

	dev = file->private_data;
	if (!dev)
		return -EFAULT;

	memset(&rot_parm, 0x00, sizeof(rot_parm));
	memset(&dma_parm, 0x00, sizeof(dma_parm));
	memset(&sc_cap_param, 0x00, sizeof(sc_cap_param));
	memset(&s_sc_cap, 0x00, sizeof(s_sc_cap));

	switch (cmd) {
	case SPRD_CPP_IO_OPEN_ROT:
		break;
	case SPRD_CPP_IO_START_ROT:
	{
		rotif = dev->rotif;
		if (!rotif) {
			pr_err("fail to get valid rotif: null!\n");
			return -EINVAL;
		}

		mutex_lock(&rotif->rot_mutex);

		ret = copy_from_user(&rot_parm,
				(void __user *)arg, sizeof(rot_parm));
		if (ret) {
			mutex_unlock(&rotif->rot_mutex);
			return -EFAULT;
		}

		ret = cpp_rot_check_parm(&rot_parm);
		if (ret) {
			pr_err("fail to check rot parm\n");
			mutex_unlock(&rotif->rot_mutex);
			return -EINVAL;
		}

		rotif->drv_priv.iommu_src.dev = &dev->pdev->dev;
		rotif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		pr_debug("start to rot y\n");
		ret = cpp_rot_set_y_parm(&rot_parm, &rotif->drv_priv);
		if (ret) {
			pr_err("fail to set rot y parm\n");
			cpp_rot_stop(&rotif->drv_priv);
			mutex_unlock(&rotif->rot_mutex);
			return -EINVAL;
		}
		cpp_rot_start(&rotif->drv_priv);

		if (!cpp_rot_is_end(&rot_parm)) {
			rtn = wait_for_completion_timeout
				(&rotif->done_com,
				msecs_to_jiffies(ROT_TIMEOUT));
			if (rtn == 0) {
				cpp_rot_stop(&rotif->drv_priv);
				mutex_unlock(&rotif->rot_mutex);
				return -EBUSY;
			}

			cpp_rot_set_uv_parm(&rotif->drv_priv);
			cpp_rot_start(&rotif->drv_priv);
		}

		rtn = wait_for_completion_timeout
			(&rotif->done_com,
			msecs_to_jiffies(ROT_TIMEOUT));
		if (rtn == 0) {
			cpp_rot_stop(&rotif->drv_priv);
			mutex_unlock(&rotif->rot_mutex);
			return -EBUSY;
		}

		cpp_rot_stop(&rotif->drv_priv);
		mutex_unlock(&rotif->rot_mutex);
		break;
	}

	case SPRD_CPP_IO_OPEN_SCALE:
	{
		get_cpp_max_size(&s_sc_cap.w, &s_sc_cap.h);
		cpp_dimension = (s_sc_cap.h << 16) | s_sc_cap.w;
		if (arg)
			ret = copy_to_user(
				(void __user *)arg, (void *)&cpp_dimension,
				sizeof(cpp_dimension));
		if (ret) {
			pr_err("fail to get max size from user\n");
			return -EFAULT;
		}
		break;
	}
	case SPRD_CPP_IO_START_SCALE:
	{
		scif = dev->scif;
		if (!scif) {
			pr_err("fail to get valid scif: null!\n");
			return -EFAULT;
		}

		sc_parm = kzalloc(sizeof(*sc_parm), GFP_KERNEL);
		if (sc_parm == NULL) {
			pr_err("fail to alloc memory\n");
			return -EFAULT;
		}

		mutex_lock(&scif->sc_mutex);

		ret = copy_from_user(sc_parm,
					(void __user *)arg, sizeof(*sc_parm));
		if (ret) {
			mutex_unlock(&scif->sc_mutex);
			kfree(sc_parm);
			return -EFAULT;
		}

		scif->drv_priv.iommu_src.dev = &dev->pdev->dev;
		scif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		ret = cpp_scale_start(sc_parm, &scif->drv_priv);
		if (ret) {
			cpp_scale_stop(&scif->drv_priv);
			mutex_unlock(&scif->sc_mutex);
			kfree(sc_parm);
			pr_err("fail to start scaler\n");
			return -EFAULT;
		}

		rtn = wait_for_completion_timeout(&scif->done_com,
						msecs_to_jiffies
						(SCALE_TIMEOUT));
		if (rtn == 0) {
			cpp_scale_stop(&scif->drv_priv);
			mutex_unlock(&scif->sc_mutex);
			kfree(sc_parm);
			pr_err("fail to get scaling done com\n");
			return -EBUSY;
		}

		cpp_scale_stop(&scif->drv_priv);
		mutex_unlock(&scif->sc_mutex);
		kfree(sc_parm);
		break;
	}

	case SPRD_CPP_IO_OPEN_DMA:
		break;
	case SPRD_CPP_IO_START_DMA:
	{
		dmaif = dev->dmaif;
		if (!dmaif) {
			pr_err("fail to get valid dmaif: null!\n");
			return -EFAULT;
		}

		mutex_lock(&dmaif->dma_mutex);

		ret = copy_from_user(&dma_parm,
				(void __user *)arg, sizeof(dma_parm));
		if (ret) {
			mutex_unlock(&dmaif->dma_mutex);
			return -EFAULT;
		}

		dmaif->drv_priv.iommu_src.dev = &dev->pdev->dev;
		dmaif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

		/* need to add */
		ret = cpp_dma_start(&dma_parm, &dmaif->drv_priv);
		if (ret) {
			cpp_dma_stop(&dmaif->drv_priv);
			mutex_unlock(&dmaif->dma_mutex);
			pr_err("fail to start dma\n");
			return -EFAULT;
		}

		rtn = wait_for_completion_timeout(&dmaif->done_com,
						msecs_to_jiffies
						(DMA_TIMEOUT));
		if (rtn == 0) {
			cpp_dma_stop(&dmaif->drv_priv);
			mutex_unlock(&dmaif->dma_mutex);
			pr_err("fail to get dma done com\n");
			return -EBUSY;
		}

		cpp_dma_stop(&dmaif->drv_priv);
		mutex_unlock(&dmaif->dma_mutex);
		break;
	}
	case SPRD_CPP_IO_STOP_SCALE:
		break;
	case SPRD_CPP_IO_SCALE_CAPABILITY:
	{
		if (!arg) {
			pr_err("fail to get arg\n");
			return -EFAULT;
		}

		memset(&sc_cap_param, 0,
				sizeof(struct sprd_cpp_scale_capability));
		ret = copy_from_user(&sc_cap_param, (void __user *)arg,
				     sizeof(sc_cap_param));
		if (ret != 0) {
			pr_err("fail to copy from user, ret = %d\n", ret);
			return -1;
		}
		ret = cpp_scale_capability(&sc_cap_param);
		if (ret == 0)
			sc_cap_param.is_supported = 1;
		else
			sc_cap_param.is_supported = 0;

		ret = copy_to_user((void  __user *)arg,
				&sc_cap_param, sizeof(sc_cap_param));

		if (ret)
			return -EFAULT;
		break;
	}
	default:
		pr_err("fail to get valid cpp cmd %d\n",
			_IOC_NR(cmd));
		break;
	}

	return ret;
}

static int sprd_cpp_open(struct inode *node, struct file *file)
{
	int ret = 0;
	struct cpp_device *dev = NULL;
	struct miscdevice *md = (struct miscdevice *)file->private_data;
	struct rotif_device *rotif = NULL;
	struct scif_device *scif = NULL;
	struct dmaif_device *dmaif = NULL;

	if (!md)
		return -EFAULT;

	dev = md->this_device->platform_data;

	file->private_data = (void *)dev;
	if (atomic_inc_return(&dev->users) != 1) {
		pr_info("cpp device node has been opened\n");
		return ret;
	}

	ret = sprd_cam_pw_on();
	if (ret) {
		pr_err("fail to camera power on\n");
		goto exit;
	}

	sprd_cam_domain_eb();

	ret = cpp_module_enable(dev);
	if (ret) {
		pr_err("fail to cpp module enable\n");
		goto exit;
	}

	rotif = vzalloc(sizeof(*rotif));
	if (unlikely(!rotif)) {
		ret = -EFAULT;
		pr_err("fail to rotif vzalloc\n");
		goto exit;
	}

	memset(rotif, 0, sizeof(*rotif));

	rotif->drv_priv.io_base = dev->io_base;
	rotif->drv_priv.priv = (void *)rotif;
	dev->rotif = rotif;
	rotif->drv_priv.hw_lock = &dev->hw_lock;
	cpp_register_isr(dev, CPP_ROT_DONE, rot_isr, (void *)rotif);
	init_completion(&rotif->done_com);
	mutex_init(&rotif->rot_mutex);

	scif = vzalloc(sizeof(*scif));
	if (unlikely(!scif)) {
		ret = -EFAULT;
		pr_err("fail to scif vzalloc\n");
		goto exit;
	}

	memset(scif, 0, sizeof(*scif));

	scif->drv_priv.io_base = dev->io_base;
	scif->drv_priv.pdev = dev->pdev;
	scif->drv_priv.hw_lock = &dev->hw_lock;
	scif->drv_priv.priv = (void *)scif;
	dev->scif = scif;
	init_completion(&scif->done_com);
	mutex_init(&scif->sc_mutex);
	cpp_register_isr(dev, CPP_SCALE_DONE, scale_isr, (void *)scif);

	dmaif = vzalloc(sizeof(*dmaif));
	if (unlikely(!dmaif)) {
		ret = -EFAULT;
		pr_err("fail to scif vzalloc\n");
		goto exit;
	}

	memset(dmaif, 0, sizeof(*dmaif));
	dmaif->drv_priv.io_base = dev->io_base;
	dmaif->drv_priv.pdev = dev->pdev;
	dmaif->drv_priv.hw_lock = &dev->hw_lock;
	dmaif->drv_priv.priv = (void *)dmaif;
	dev->dmaif = dmaif;
	init_completion(&dmaif->done_com);
	mutex_init(&dmaif->dma_mutex);
	spin_lock_init(&dev->hw_lock);
	cpp_register_isr(dev, CPP_DMA_DONE, dma_isr, (void *)dmaif);

#ifdef DEBUG_CPP_MMU
	/* only for debugging */
	cpp_register_isr(dev, CPP_MMU_0, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_1, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_2, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_3, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_4, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_5, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_6, mmu_isr, NULL);
	cpp_register_isr(dev, CPP_MMU_7, mmu_isr, NULL);
#endif

	ret = devm_request_irq(&dev->pdev->dev, dev->irq, cpp_isr_root,
				IRQF_SHARED, "CPP", (void *)dev);
	if (ret < 0) {
		pr_err("fail to install IRQ %d\n", ret);
		goto exit;
	}

	pr_info("open sprd_cpp success\n");

	return ret;

exit:
	if (rotif) {
		vfree(rotif);
		dev->rotif = NULL;
	}

	if (scif) {
		vfree(scif);
		dev->scif = NULL;
	}

	if (dmaif) {
		vfree(dmaif);
		dev->dmaif = NULL;
	}
	return ret;
}

static int sprd_cpp_release(struct inode *node, struct file *file)
{
	int ret = 0;
	struct cpp_device *dev = NULL;

	dev = file->private_data;

	if (dev == NULL) {
		pr_err("fail to close cpp device\n");
		return -EFAULT;
	}

	if (atomic_dec_return(&dev->users) != 0) {
		pr_info("others is using cpp device\n");
		return ret;
	}

	if (dev->rotif) {
		mutex_destroy(&dev->rotif->rot_mutex);
		vfree(dev->rotif);
		dev->rotif = NULL;
	}
	if (dev->scif) {
		mutex_destroy(&dev->scif->sc_mutex);
		vfree(dev->scif);
		dev->scif = NULL;
	}

	if (dev->dmaif) {
		vfree(dev->dmaif);
		dev->dmaif = NULL;
	}

	devm_free_irq(&dev->pdev->dev, dev->irq, (void *)dev);
	cpp_module_disable(dev);

	sprd_cam_domain_disable();
	ret = sprd_cam_pw_off();
	if (ret) {
		pr_err("fail to camera power off\n");
		return ret;
	}

	file->private_data = NULL;
	pr_info("cpp release success\n");

	return ret;
}

static const struct file_operations cpp_fops = {
	.open = sprd_cpp_open,
	.unlocked_ioctl = sprd_cpp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_cpp_ioctl,
#endif
	.release = sprd_cpp_release,
};

static int sprd_cpp_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;
	int irq = 0;
	struct cpp_device *dev = NULL;
	struct resource *res = NULL;
	struct device_node *np_isp;
	struct device_node *np;
	const char *pname;
	struct regmap *tregmap;
	unsigned int args[2];

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->lock);
	spin_lock_init(&dev->slock);
	atomic_set(&dev->users, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dev->io_base = devm_ioremap_nocache(&pdev->dev, res->start,
				resource_size(res));
	if (IS_ERR(dev->io_base))
		return PTR_ERR(dev->io_base);

	dev->cpp_eb = devm_clk_get(&pdev->dev, "cpp_eb");
	if (IS_ERR(dev->cpp_eb))
		return PTR_ERR(dev->cpp_eb);

	dev->cpp_axi_eb = devm_clk_get(&pdev->dev, "cpp_axi_eb");
	if (IS_ERR(dev->cpp_axi_eb))
		return PTR_ERR(dev->cpp_axi_eb);

	/* hw: cpp,isp use the same clk
	 * so read isp_clk, isp_clk_parent from isp node
	 * If change clk, need change isp_clk_parent
	 */
	np_isp = of_parse_phandle(pdev->dev.of_node, "ref-node", 0);
	if (IS_ERR(np_isp)) {
		pr_err("fail to get isp node\n");
		return PTR_ERR(np_isp);
	}

	dev->cpp_clk = of_clk_get_by_name(np_isp, "isp_clk");
	if (IS_ERR(dev->cpp_clk)) {
		pr_err("fail to get isp_clk, %p\n", dev->cpp_clk);
		return PTR_ERR(dev->cpp_clk);
	}

	dev->cpp_clk_parent = of_clk_get_by_name(np_isp, "isp_clk_parent");
	if (IS_ERR(dev->cpp_clk_parent)) {
		pr_err("fail to get isp_clk_parent %p\n", dev->cpp_clk_parent);
		return PTR_ERR(dev->cpp_clk_parent);
	}

	dev->cpp_clk_default = clk_get_parent(dev->cpp_clk);
	if (IS_ERR(dev->cpp_clk_default))
		return PTR_ERR(dev->cpp_clk_default);

	dev->md.minor = MISC_DYNAMIC_MINOR;
	dev->md.name = CPP_DEVICE_NAME;
	dev->md.fops = &cpp_fops;
	dev->md.parent = NULL;
	ret = misc_register(&dev->md);
	if (ret) {
		pr_err("fail to register misc devices\n");
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		pr_err("fail to get IRQ\n");
		ret = -ENXIO;
		goto fail;
	}

	dev->irq = irq;

	dev->pdev = pdev;
	dev->md.this_device->platform_data = (void *)dev;
	platform_set_drvdata(pdev, (void *)dev);
	np = pdev->dev.of_node;

	/* read global register */
	for (i = 0; i < ARRAY_SIZE(syscon_name); i++) {
		pname = syscon_name[i];
		tregmap =  syscon_regmap_lookup_by_name(np, pname);
		if (IS_ERR_OR_NULL(tregmap)) {
			pr_err("fail to read %s regmap\n", pname);
			continue;
		}
		ret = syscon_get_args_by_name(np, pname, 2, args);
		if (ret != 2) {
			pr_err("fail to read %s args, ret %d\n",
				pname, ret);
			continue;
		}
		dev->syscon_regs[i].gpr = tregmap;
		dev->syscon_regs[i].reg = args[0];
		dev->syscon_regs[i].mask = args[1];
		pr_info("dts[%s] 0x%x 0x%x\n", pname,
			dev->syscon_regs[i].reg, dev->syscon_regs[i].mask);
	}

	pr_info("cpp probe OK\n");

	return 0;

fail:
	misc_deregister(&dev->md);
	return ret;
}

static int sprd_cpp_remove(struct platform_device *pdev)
{
	struct cpp_device *dev = platform_get_drvdata(pdev);
	mutex_destroy(&dev->lock);
	misc_deregister(&dev->md);
	return 0;
}

static const struct of_device_id of_match_table[] = {
	{ .compatible = "sprd,cpp", },
	{},
};

static struct platform_driver sprd_cpp_driver = {
	.probe = sprd_cpp_probe,
	.remove = sprd_cpp_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = CPP_DEVICE_NAME,
		.of_match_table = of_match_ptr(of_match_table),
	},
};

module_platform_driver(sprd_cpp_driver);

MODULE_DESCRIPTION("R3P0 Lite CPP Driver");
MODULE_AUTHOR("Multimedia_Camera@Unisoc");
MODULE_LICENSE("GPL");
