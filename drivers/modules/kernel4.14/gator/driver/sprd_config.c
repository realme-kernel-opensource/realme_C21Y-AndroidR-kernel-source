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
#ifndef __GATOR_SPRD_CONFIG_C__
#define __GATOR_SPRD_CONFIG_C__

static unsigned int sprd_aon_apb_read(unsigned int reg_off)
{
#if defined(CONFIG_OF)
	unsigned int val;
	regmap_read(busmon.aon_apb_regmap, reg_off, &val);
	return val;
#else
	return readl_relaxed(busmon.aon_apb_base+reg_off);
#endif
}

static int sprd_aon_apb_update_bit(unsigned int reg_off, unsigned int bit, unsigned int val)
{
#if defined(CONFIG_OF)
	return regmap_update_bits(busmon.aon_apb_regmap, reg_off, (1 << bit), (val << bit));
#else
	writel_relaxed((1 << bit), busmon.aon_apb_base+reg_off+(0x2000>>(val&0x1)));
	return 0;
#endif
}

static int sprd_pmu_apb_update_bit(unsigned int reg_off, unsigned int bit, unsigned int val)
{
#if defined(CONFIG_OF)
	return regmap_update_bits(busmon.pmu_apb_regmap, reg_off, (1 << bit), (val << bit));
#else
	writel_relaxed((1 << bit), busmon.pmu_apb_base+reg_off+(0x2000>>(val&0x1)));
	return 0;
#endif
}

#define AS(a) (sizeof(a)/sizeof(a[0]))
#define MATCHID(r_id, id) ((id) == DONOT_CARE ? 1 : (id) == (r_id))
static int sprd_busmon_map_match(unsigned int type)
{
	unsigned int i;
	unsigned int chipid1; //0x00FC
	unsigned int chipid0; //0x00F8
	unsigned int chipid3; //0x00E4
	unsigned int chipid2; //0x00E0
	chipid1 = sprd_aon_apb_read(0x00FC);
	chipid0 = sprd_aon_apb_read(0x00F8);
	chipid3 = sprd_aon_apb_read(0x00E4);
	chipid2 = sprd_aon_apb_read(0x00E0);
	for (i = 0; i < AS(busmon_map); i++) {
		if (type == busmon_map[i].type) {
			if (MATCHID(chipid1, busmon_map[i].chipid1) && MATCHID(chipid0, busmon_map[i].chipid0) &&
					MATCHID(chipid3, busmon_map[i].chipid3) && MATCHID(chipid2, busmon_map[i].chipid2)) {
				busmon.map = &busmon_map[i];
				busmon.cfg = bm_gp_cfg[busmon.map->gp_id].bm_cfg;
				if (busmon.cfg[0].bm_tmr) {
					busmon.tmr = busmon.cfg[0].bm_tmr;
				}
				if (busmon.cfg[0].bm_pmu) {
					busmon.pmu = busmon.cfg[0].bm_pmu;
				}
				return 0;
			}
		}
	}
	return -1;
}

static int sprd_linux_get_base_addr(unsigned int type)
{
#if defined(CONFIG_OF)
	struct device_node *node = of_find_all_nodes(NULL);
	if (node) {
		of_node_put(node);
		node = of_find_compatible_node(NULL, NULL, "sprd,clk-default");
		if (!node) {
			pr_err("can't get clk-default node!\n");
			return -1;
		}
		busmon.aon_apb_regmap = syscon_regmap_lookup_by_phandle(node, "sprd,syscon-aon-apb");
		if (IS_ERR(busmon.aon_apb_regmap)) {
			pr_err("can't get aon_apb_controller node!\n");
			of_node_put(node);
			return -1;
		}
		busmon.pmu_apb_regmap = syscon_regmap_lookup_by_phandle(node, "sprd,syscon-pmu-apb");
		if (IS_ERR(busmon.pmu_apb_regmap)) {
			pr_err("can't get pmu_apb_controller node!\n");
			of_node_put(node);
			return -1;
		}
		of_node_put(node);
	}
#else
	pr_err("gator.ko use own way to write/read aon apb! \n");

#ifdef CONFIG_GATOR_SPRD_X86_SUPPORT
	busmon.aon_apb_base = ioremap_nocache(0xE42E0000, SZ_4K);
	busmon.pmu_apb_base = ioremap_nocache(0xE42B0000, SZ_4K);
#else
	busmon.aon_apb_base = ioremap_nocache(0x402E0000, SZ_4K);
	busmon.pmu_apb_base = ioremap_nocache(0x402B0000, SZ_4K);
#endif
	if ((!(busmon.aon_apb_base))||(!(busmon.pmu_apb_base))) {
		pr_err("allocate io memory failed! aon/pmu apb\n");
		return -1;
	}
#endif
	return sprd_busmon_map_match(type);
}

#endif
