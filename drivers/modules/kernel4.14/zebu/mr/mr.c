/*
 * Copyright (C) 2018 UNISOC Communications Inc.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/compiler.h>
#include <linux/slab.h>
#include <linux/arm-smccc.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "mr.h"

static char *path = "/system/etc/zrn.def";

static unsigned long get_mask_val(unsigned int f, unsigned int t,
					  unsigned int width)
{

	unsigned long type_mask = 0;
	unsigned long mask_val = 0;

	if (width == 32)
		type_mask = 0xffffffff;
	else if (width == 64)
		type_mask = 0xffffffffffffffff;

	mask_val = (type_mask << f) & (type_mask >> (width-1-t));

	return mask_val;
}

static unsigned long monitor_regiter_call(unsigned long function_id,
					 unsigned long arg0, unsigned long arg1,
					 unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return res.a0;
}

static int get_file_size(void *f)
{
	int error;
	struct kstat stat;

	error = vfs_stat(f, &stat);
	if (error) {
		pr_err("get conf file stat error\n");
		return error;
	}
	return stat.size;
}

void mr_main(struct regisiter_nodes *node, int inx)
{
	unsigned long register_addr = 0;
	unsigned long register_value = 0;
	unsigned long final_value = 0;

	register_addr = (node+inx)->register_base+(node+inx)->register_offset;
	register_value = monitor_regiter_call(ARM_STD_SVC_MR,
						  register_addr, 0, 0);
	(node+inx)->bit_mask = get_mask_val((node+inx)->from_bit,
						(node+inx)->to_bit,
						(node+inx)->register_width);
	final_value = register_value & ((node+inx)->bit_mask);

	if (final_value !=
		(node+inx)->expected_value) {
		pr_emerg("0x%lx=0x%lx [%02d:%02d] expect 0x%lx but 0x%lx -%s wrong!!\n",
				register_addr,
				register_value,
				(node+inx)->from_bit, (node+inx)->to_bit,
				(node+inx)->expected_value,
				final_value,
				(node+inx)->description);
	}
}

static int mrkernel_read(struct file *file, loff_t offset,
					char *addr, unsigned long count)
{
	mm_segment_t old_fs;
	loff_t pos = offset;
	int result;

	old_fs = get_fs();
	set_fs(get_ds());
	/* The cast to a user pointer is valid due to the set_fs() */
	result = vfs_read(file, (void __user *)addr, count, &pos);
	set_fs(old_fs);

	return result;
}

static int monitor_regnodes(void)
{
	struct regisiter_nodes *node = NULL;
	char *buf = NULL;
	int len = 0;
	struct file *fp = NULL;
	mm_segment_t fs;
	int ret, c_len, reg_num;

	fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("open file error!");
		return -ENOENT;
	}

	len = get_file_size(path);
	if (len <= 0) {
		pr_err("get size error\n");
		goto out;
	}

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		pr_err("no mem error\n");
		goto out;
	}

	node = kzalloc(len, GFP_KERNEL);
	if (!node) {
		pr_err("no mem error\n");
		goto err1;
	}
	ret = mrkernel_read(fp, 0, buf, len);
	if (ret < 0) {
		pr_err("read erro\n");
		goto err;
	}

	for (c_len = 0, reg_num = 0; c_len < len; c_len++) {
		if (buf[c_len] == '\n') {
			ret = sscanf(&buf[c_len+1],
					"%d,0x%lx,0x%lx,%d,%d,0x%lx,%s",
				    &((node+reg_num)->register_width),
				    &((node+reg_num)->register_base),
					&((node+reg_num)->register_offset),
					&((node+reg_num)->from_bit),
					&((node+reg_num)->to_bit),
					&((node+reg_num)->expected_value),
					(node+reg_num)->description);
			if (ret != 7)
				continue;
			mr_main(node, reg_num);
			reg_num++;
		}
	}

err:
	kfree(node);
err1:
	kfree(buf);
out:
	filp_close(fp, NULL);
	set_fs(fs);

	return 0;
}

static int __init mr_init(void)
{
	int ret;

	ret = monitor_regnodes();
	if (ret)
		return -1;

	return 0;
}

static void __exit mr_exit(void)
{
	pr_info("mr Module exit!\n");
}

module_init(mr_init);
module_exit(mr_exit);

MODULE_AUTHOR("XiXin Liu<xixin.liu@unisoc.com>");
MODULE_DESCRIPTION("MR:Monitor Register driver");
MODULE_LICENSE("GPL");

module_param(path, charp, 0444);
