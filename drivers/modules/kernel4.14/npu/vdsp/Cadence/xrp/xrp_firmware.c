/*
 * xrp_firmware: firmware manipulation for the XRP
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Alternatively you can use and distribute this file under the terms of
 * the GNU General Public License version 2 or later.
 */

#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "vdsp_hw.h"
#include "xrp_internal.h"
#include "xrp_firmware.h"
#include "xrp_kernel_dsp_interface.h"

#define DOWNLOAD_FIRMWARE

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "sprd-vdsp: firmware %d %d %s : "\
        fmt, current->pid, __LINE__, __func__

static int xrp_load_segment_to_sysmem_ion(struct xvp *xvp, Elf32_Phdr *phdr)
{
#ifdef DOWNLOAD_FIRMWARE
	int32_t offset;
	uint8_t *virstart = NULL;
#endif

	pr_debug("phdr->p_paddr:%lx, firmware viraddr:%lx\n",
	(unsigned long)phdr->p_paddr, (unsigned long)xvp->dsp_firmware_addr);
#ifdef DOWNLOAD_FIRMWARE
	if (unlikely(phdr->p_paddr < xvp->dsp_firmware_addr)) {
		pr_err("[ERROR] func:%s , p_paddr:%lx , dsp_firmware_addr:%lx" , __func__ , phdr->p_paddr , xvp->dsp_firmware_addr);
		return -EFAULT;
	}
	offset = phdr->p_paddr - xvp->dsp_firmware_addr;
	virstart = (uint8_t*)xvp->firmware_viraddr;

	pr_debug("virstart:%p, offset:%x, poffset:%x, pmemsz:%x, pfilesz:%x\n",
		virstart, offset, phdr->p_offset, phdr->p_memsz, phdr->p_filesz);
	memcpy((void*)(virstart + offset), xvp->firmware->data + phdr->p_offset, phdr->p_filesz);
	if (phdr->p_memsz > phdr->p_filesz){
		memset((void*)(virstart + offset + phdr->p_filesz), 0,
			(phdr->p_memsz - phdr->p_filesz));
	}
	wmb();
#endif

	return 0;
}
static int xrp_load_segment_to_iomem(struct xvp *xvp, Elf32_Phdr *phdr)
{
	phys_addr_t pa = phdr->p_paddr;
	void __iomem *p = ioremap(pa, phdr->p_memsz);

	if (unlikely(!p)) {
		pr_err("couldn't ioremap %pap x 0x%08x\n", &pa, (u32)phdr->p_memsz);
		return -EINVAL;
	}
	if (xvp->hw_ops->memcpy_tohw)
		xvp->hw_ops->memcpy_tohw(p, (void *)xvp->firmware->data +
			phdr->p_offset, phdr->p_filesz);
	else
		memcpy_toio(p, (void *)xvp->firmware->data + phdr->p_offset,
			ALIGN(phdr->p_filesz, 4));

	if (xvp->hw_ops->memset_hw)
		xvp->hw_ops->memset_hw(p + phdr->p_filesz, 0,
			phdr->p_memsz - phdr->p_filesz);
	else
		memset_io(p + ALIGN(phdr->p_filesz, 4), 0,
			ALIGN(phdr->p_memsz - ALIGN(phdr->p_filesz, 4), 4));

	iounmap(p);

	return 0;
}
static inline bool xrp_section_bad(struct xvp *xvp, const Elf32_Shdr *shdr)
{
	return shdr->sh_offset > xvp->firmware->size ||
		shdr->sh_size > xvp->firmware->size - shdr->sh_offset;
}

static int xrp_firmware_find_symbol(struct xvp *xvp, const char *name,
	void **paddr, size_t *psize)
{
	const Elf32_Ehdr *ehdr = (Elf32_Ehdr *)xvp->firmware->data;
	const void *shdr_data = xvp->firmware->data + ehdr->e_shoff;
	const Elf32_Shdr *sh_symtab = NULL;
	const Elf32_Shdr *sh_strtab = NULL;
	const void *sym_data;
	const void *str_data;
	const Elf32_Sym *esym;
	void *addr = NULL;
	unsigned i;

	if (unlikely(ehdr->e_shoff == 0)) {
		pr_err("[ERROR]no section header in the firmware image");
		return -ENOENT;
	}
	if (unlikely(ehdr->e_shoff > xvp->firmware->size ||
		(((size_t)ehdr->e_shnum) * ((size_t)ehdr->e_shentsize)) > xvp->firmware->size - ehdr->e_shoff)) {
		pr_err("[ERROR]bad firmware SHDR information");
		return -EINVAL;
	}

	/* find symbols and string sections */
	for (i = 0; i < ehdr->e_shnum; ++i) {
		const Elf32_Shdr *shdr = shdr_data + i * ehdr->e_shentsize;

		switch (shdr->sh_type) {
		case SHT_SYMTAB:
			sh_symtab = shdr;
			break;
		case SHT_STRTAB:
			sh_strtab = shdr;
			break;
		default:
			break;
		}
	}

	if (unlikely(!sh_symtab || !sh_strtab)) {
		pr_debug("no symtab or strtab in the firmware image");
		return -ENOENT;
	}

	if (unlikely(xrp_section_bad(xvp, sh_symtab))) {
		pr_err("[ERROR]bad firmware SYMTAB section information");
		return -EINVAL;
	}

	if (unlikely(xrp_section_bad(xvp, sh_strtab))) {
		pr_err("[ERROR]bad firmware STRTAB section information");
		return -EINVAL;
	}

	/* iterate through all symbols, searching for the name */
	sym_data = xvp->firmware->data + sh_symtab->sh_offset;
	str_data = xvp->firmware->data + sh_strtab->sh_offset;

	for (i = 0; i < sh_symtab->sh_size; i += sh_symtab->sh_entsize) {
		esym = sym_data + i;

		if (!(ELF_ST_TYPE(esym->st_info) == STT_OBJECT &&
			esym->st_name < sh_strtab->sh_size &&
			strncmp(str_data + esym->st_name, name,
			sh_strtab->sh_size - esym->st_name) == 0))
			continue;

		if (esym->st_shndx > 0 && esym->st_shndx < ehdr->e_shnum) {
			const Elf32_Shdr *shdr = shdr_data +
				esym->st_shndx * ehdr->e_shentsize;
			Elf32_Off in_section_off = esym->st_value - shdr->sh_addr;

			if (unlikely(xrp_section_bad(xvp, shdr))) {
				pr_err("[ERROR]bad firmware section #%d information",
					esym->st_shndx);
				return -EINVAL;
			}

			if (unlikely(esym->st_value < shdr->sh_addr ||
				in_section_off > shdr->sh_size ||
				esym->st_size > shdr->sh_size - in_section_off)) {
				pr_err("[ERROR]bad symbol information");
				return -EINVAL;
			}
			addr = (void *)xvp->firmware->data + shdr->sh_offset +
				in_section_off;

			pr_debug("found symbol, st_shndx = %d, "
				"sh_offset = 0x%08x, sh_addr = 0x%08x, "
				"st_value = 0x%08x, address = %p",
				esym->st_shndx, shdr->sh_offset,
				shdr->sh_addr, esym->st_value, addr);
		}else {
			pr_err("unsupported section index in found symbol: 0x%x",
				esym->st_shndx);
			return -EINVAL;
		}
		break;
	}

	if (unlikely(!addr)) {
		pr_err("[ERROR] addr is NULL\n");
		return -ENOENT;
	}

	*paddr = addr;
	*psize = esym->st_size;

	return 0;
}

static int xrp_firmware_fixup_symbol(struct xvp *xvp, const char *name,
	phys_addr_t v)
{
	u32 v32 = XRP_DSP_COMM_BASE_MAGIC;
	void *addr;
	size_t sz;
	int rc;

	rc = xrp_firmware_find_symbol(xvp, name, &addr, &sz);
	if (unlikely(rc < 0)) {
		pr_err("[ERROR]symbol \"%s\" is not found", name);
		return rc;
	}

	if (unlikely(sz != sizeof(u32))) {
		pr_err("[ERROR]symbol \"%s\" has wrong size: %zu", name, sz);
		return -EINVAL;
	}

	/* update data associated with symbol */
	if (memcmp(addr, &v32, sz) != 0)
		pr_debug("value pointed to by symbol is incorrect: %*ph",
			(int)sz, addr);

	v32 = v;
	memcpy(addr, &v32, sz);

	return 0;
}

static int xrp_load_firmware(struct xvp *xvp)
{
	Elf32_Ehdr *ehdr = (Elf32_Ehdr *)xvp->firmware->data;
	int i;

	if (unlikely(memcmp(ehdr->e_ident, ELFMAG, SELFMAG))) {
		pr_err("bad firmware ELF magic\n");
		return -EINVAL;
	}

	if (unlikely(ehdr->e_type != ET_EXEC)) {
		pr_err("bad firmware ELF type\n");
		return -EINVAL;
	}

	if (unlikely(ehdr->e_machine != 94 /*EM_XTENSA*/)) {
		pr_err("bad firmware ELF machine\n");
		return -EINVAL;
	}

	if (unlikely(ehdr->e_phoff >= xvp->firmware->size ||
		ehdr->e_phoff +
		ehdr->e_phentsize * ehdr->e_phnum > xvp->firmware->size)) {
		pr_err("bad firmware ELF PHDR information\n");
		return -EINVAL;
	}
	xrp_firmware_fixup_symbol(xvp, "xrp_dsp_comm_base", xvp->dsp_comm_addr);
	for (i = 0; i < ehdr->e_phnum; ++i) {
		Elf32_Phdr *phdr = (void *)xvp->firmware->data +
			ehdr->e_phoff + i * ehdr->e_phentsize;
		int rc;

		/* Only load non-empty loadable segments, R/W/X */
		if (!(phdr->p_type == PT_LOAD &&
			(phdr->p_flags & (PF_X | PF_R | PF_W)) &&
			phdr->p_memsz > 0))
			continue;

		if (unlikely(phdr->p_offset >= xvp->firmware->size ||
			phdr->p_offset + phdr->p_filesz > xvp->firmware->size)) {
			pr_err("bad firmware ELF program header entry %d\n", i);
			return -EINVAL;
		}
		if (phdr->p_paddr >= xvp->dsp_firmware_addr)
			rc = xrp_load_segment_to_sysmem_ion(xvp, phdr);
		else
			rc = xrp_load_segment_to_iomem(xvp, phdr);

		if (rc < 0)
			return rc;
	}

	return 0;
}

int xrp_request_firmware(struct xvp *xvp)
{
	int ret;
	s64 tv0, tv1, tv2;

	tv0 = ktime_to_us(ktime_get());

	ret = request_firmware(&xvp->firmware, xvp->firmware_name, xvp->dev);
	if (unlikely(ret < 0)){
		pr_err("[ERROR]request firmware fail\n");
		return ret;
	}
	tv1 = ktime_to_us(ktime_get());

	ret = xrp_load_firmware(xvp);
	release_firmware(xvp->firmware);
	tv2 = ktime_to_us(ktime_get());
	pr_info("[TIME]request&load firmware,total:%lld(us),request:%lld(us),load:%lld(us)\n",
		tv2 - tv0, tv1 - tv0, tv2 - tv1);

	return ret;
}
