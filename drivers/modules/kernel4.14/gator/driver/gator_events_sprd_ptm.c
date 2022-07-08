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
/*
 * Similar entries to those below must be present in the events.xml file.
 * To add them to the events.xml, create an events-sprd-ptm.xml with the
 * following contents and rebuild gatord:
 *
 * event = <31:16><15:0>
 * <15:0> the base address
 * <31:16>  the counter offset
 * <counter_set name="SLE_PUB0_BM0_cnt" count="6" />
 * <category name="SPRD MM PUB" counter_set="SLE_PUB0_BM0_cnt" per_cpu="no">
 *   <!--
 * 	PUB_MM Performence Bus Monitor
 *   -->
 *   <event event="0x00DC3004" title="MM" name="PUB_RTRANS" display="accumulate" class="incident" description="PUB MM Read Transaction"/>
 *   <event event="0x009C3004" title="MM" name="PUB_RBW" display="accumulate" class="incident" description="PUB MM Read BandWidth"/>
 *   <event event="0x00C03004" title="MM" name="PUB_RLATENCY" display="accumulate" class="incident" description="PUB MM Read Latency"/>
 *   <event event="0x011C3004" title="MM" name="PUB_WTRANS" display="accumulate" class="incident" description="PUB MM Write Transaction"/>
 *   <event event="0x00803004" title="MM" name="PUB_WBW" display="accumulate" class="incident" description="PUB MM Write BandWidth"/>
 *   <event event="0x01003004" title="MM" name="PUB_WLATENCY" display="accumulate" class="incident" description="PUB MM Write Latency"/>
 * </category>
 *
 * When adding custom events, be sure to do the following:
 * - add any needed .c files to the gator driver Makefile
 * - call gator_events_install in the events init function
 * - add the init function to GATOR_EVENTS_LIST in gator_main.c
 * - add a new events-*.xml file to the gator daemon and rebuild
 *
 * Troubleshooting:
 * - verify the new events are part of events.xml, which is created when building the daemon
 * - verify the new events exist at /dev/gator/events/ once gatord is launched
 * - verify the counter name in the XML matches the name at /dev/gator/events
 */

#define pr_fmt(fmt) "[gator.ptm]" fmt
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/ratelimit.h>
#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_address.h>
#endif

#include "gator.h"

#define busmon_dbg pr_info

#include "sprd_config.h"

#define BUSMON_GLB_EN_BIT_OFF (busmon.cfg[pub_id].bm_glb_en_bit_off)

#define BUSMON_PUB_NUM_MAX 2
#define BUSMON_NUM_MAX 15
#define BUSMON_COUNTERS_NUM_PER_MAX 8
#define BUSMON_COUNTERS_NUM_MAX (BUSMON_PUB_NUM_MAX * BUSMON_NUM_MAX * BUSMON_COUNTERS_NUM_PER_MAX)

#define BUSMON_PUB_NUM (bm_gp_cfg[busmon.map->gp_id].busmon_gp_cnt)
#define BUSMON_NUM (busmon.cfg[pub_id].bm_cnt)
#define BUSMON_COUNTERS_NUM_PER BUSMON_COUNTERS_NUM_PER_MAX

#define GET_BASE_ADDR(e) ((e&0x0000FFFF)<<16)
#define GET_COUNTER_ID(e) ((e&0xFFFF0000)>>16)

static struct {
	unsigned long enabled;
	unsigned long event;
	unsigned long key;
} busmon_counters[BUSMON_PUB_NUM_MAX][BUSMON_NUM_MAX][BUSMON_COUNTERS_NUM_PER_MAX];

static struct {
	unsigned long is_enabled[BUSMON_PUB_NUM_MAX];
	unsigned long is_glb_enabled;
	unsigned long prev_count[BUSMON_PUB_NUM_MAX][BUSMON_NUM_MAX][BUSMON_COUNTERS_NUM_PER_MAX];
	void __iomem* base_reg[BUSMON_PUB_NUM_MAX];
	void __iomem* pub_busmon_cfg[BUSMON_PUB_NUM_MAX];
	void __iomem* busmon_timer_base;
#if !defined(CONFIG_OF)
	void __iomem* aon_apb_base;
	void __iomem* pmu_apb_base;
#else
	struct regmap* aon_apb_regmap;
	struct regmap* pmu_apb_regmap;
#endif
	struct sprd_chipid_map* map;
	struct sprd_busmon_config* cfg;
	struct sprd_busmon_ext_config* tmr;
	struct sprd_busmon_ext_config* pmu;
} busmon;

static int busmon_buffer[BUSMON_COUNTERS_NUM_MAX * 2];

#include "sprd_config.c"

/* Adds busmon_cntX directories and enabled, event, and key files to /dev/gator/events */
static int gator_events_busmon_create_files(struct super_block *sb,
					struct dentry *root)
{
	int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);

	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
				int len;
				char buf[32];
				struct dentry *dir;

				len = snprintf(buf, sizeof(buf), "%s_PUB%d_BM%d_cnt%d", busmon.map->name, pub_id, busmon_id, counter_id);
				BUG_ON(len >= 32);
				buf[len] = 0;
				busmon_dbg("Create dir %s\n", buf);
				dir = gatorfs_mkdir(sb, root, buf);
				if (WARN_ON(!dir))
					return -1;
				gatorfs_create_ulong(sb, dir, "enabled",
						&busmon_counters[pub_id][busmon_id][counter_id].enabled);
				gatorfs_create_ulong(sb, dir, "event",
						&busmon_counters[pub_id][busmon_id][counter_id].event);
				gatorfs_create_ro_ulong(sb, dir, "key",
						&busmon_counters[pub_id][busmon_id][counter_id].key);
			}
		}
	}

	return 0;
}

static void busmon_pmu_ctrl(int val)
{
	if (busmon.pmu) {
		if (DONOT_CARE != busmon.pmu->reg_base) {
			sprd_pmu_apb_update_bit(busmon.pmu->reg_base, busmon.pmu->en_bit, val);
		}
		if (DONOT_CARE != busmon.pmu->glb_reg_off) {
			sprd_pmu_apb_update_bit(busmon.pmu->glb_reg_off, busmon.pmu->glb_en_bit, val);
		}
		if (DONOT_CARE != busmon.pmu->ext_reg_off) {
			sprd_pmu_apb_update_bit(busmon.pmu->ext_reg_off, busmon.pmu->ext_en_bit, val);
		}
		if (DONOT_CARE != busmon.pmu->ext1_reg_off) {
			sprd_pmu_apb_update_bit(busmon.pmu->ext1_reg_off, busmon.pmu->ext1_en_bit, val);
		}
		if (!val) {
			sprd_aon_apb_read(0x00FC);
		}
	}
}

static int busmon_raw_read(int p_id, int b_id, int c_id)
{
	unsigned long ret_cnt;
	int offset;
	offset = GET_COUNTER_ID(busmon_counters[p_id][b_id][c_id].event);
	ret_cnt = readl_relaxed(busmon.base_reg[p_id] + offset);
	return ret_cnt;
}

static int gator_events_busmon_start(void)
{
	int enabled = 0;
	int p_id;
	void __iomem * reg;
	unsigned long temp;
	unsigned int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);

	busmon_pmu_ctrl(0);
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
				if (busmon_counters[pub_id][busmon_id][counter_id].enabled) {
					busmon_dbg("enable PUB%d.BM%d.Counter%d \n", pub_id, busmon_id, counter_id);
					/* enable busmon */
					p_id = pub_id;
					if (!busmon.is_enabled[p_id]) {
						reg = busmon.pub_busmon_cfg[p_id] + (busmon.cfg[p_id].bm_glb_reg&0xFFFF);
						writel_relaxed(readl_relaxed(reg)|BIT(BUSMON_GLB_EN_BIT_OFF),reg);
						if (busmon.base_reg[p_id])
							iounmap(busmon.base_reg[p_id]);
						temp = GET_BASE_ADDR(busmon_counters[pub_id][busmon_id][counter_id].event);
						busmon.base_reg[p_id] = ioremap_nocache(temp, SZ_4K);
						reg = busmon.base_reg[p_id];
						if (!reg) {
							pr_err("allocate io memory failed! pub_id=%d\n", p_id);
							busmon_pmu_ctrl(1);
							return -1;
						}
						/* set window */
						writel_relaxed(0x10000,reg+0x001C);
						/* set group mapping */
						writel_relaxed((0<<0)|(1<<3)|(2<<6)|(3<<9)|(4<<12)|(5<<15)|(6<<18),reg+0x0020);
						/* select PTM mode */
						writel_relaxed(0x1,reg+0x000C);
						/* PTM enable */
						writel_relaxed(readl_relaxed(reg)|BIT(0)|BIT(1),reg);
						busmon.is_enabled[p_id] = 1;
					}
					enabled = 1;
					busmon.prev_count[p_id][busmon_id][counter_id] = busmon_raw_read(p_id, busmon_id, counter_id);
				}
			}
		}
	}
	busmon_pmu_ctrl(1);

	if (enabled && (!busmon.is_glb_enabled) && busmon.tmr) {
		if (DONOT_CARE != busmon.tmr->glb_reg_off) {
			sprd_aon_apb_update_bit(busmon.tmr->glb_reg_off, busmon.tmr->glb_en_bit, 1);
		}
		if (DONOT_CARE != busmon.tmr->ext_reg_off) {
			sprd_aon_apb_update_bit(busmon.tmr->ext_reg_off, busmon.tmr->ext_en_bit, 1);
		}
		if (DONOT_CARE != busmon.tmr->ext1_reg_off) {
			sprd_aon_apb_update_bit(busmon.tmr->ext1_reg_off, busmon.tmr->ext1_en_bit, 1);
		}
		/* enable bus monitor timer, defualt 1ms */
		reg = busmon.busmon_timer_base;
		writel_relaxed(readl_relaxed(reg)|BIT(busmon.tmr->en_bit),reg);
		busmon.is_glb_enabled = 1;
	}

	return 0;
}

static void gator_events_busmon_stop(void)
{
	int p_id;
	void __iomem * reg;
	int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);

	busmon_pmu_ctrl(0);
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
				busmon_dbg("disable PUB%d.BM%d.Counter%d \n", pub_id, busmon_id, counter_id);
				/* disable busmon */
				p_id = pub_id;
				if (busmon.is_enabled[p_id]) {
					reg = busmon.base_reg[p_id];
					writel_relaxed(readl_relaxed(reg)&~BIT(0),reg);
					if (busmon.base_reg[p_id]) {
						iounmap(busmon.base_reg[p_id]);
						busmon.base_reg[p_id] = 0;
					}
#if 0
					reg = busmon.pub_busmon_cfg[p_id] + (busmon.cfg[p_id].bm_glb_reg&0xFFFF);
					writel_relaxed(readl_relaxed(reg)&~BIT(BUSMON_GLB_EN_BIT_OFF),reg);
#endif
					busmon.is_enabled[p_id] = 0;
				}
			}
		}
	}
	busmon_pmu_ctrl(1);

	if (busmon.is_glb_enabled && busmon.tmr) {
		/* disable bus monitor timer */
		reg = busmon.busmon_timer_base;
		writel_relaxed(readl_relaxed(reg)&~BIT(busmon.tmr->en_bit),reg);

		if (DONOT_CARE != busmon.tmr->glb_reg_off) {
			sprd_aon_apb_update_bit(busmon.tmr->glb_reg_off, busmon.tmr->glb_en_bit, 0);
		}
#if 0
		/* We don't close the external register. It maybe cause power issue, but, gator.ko don't take care! */
		if (DONOT_CARE != busmon.tmr->ext_reg_off) {
			sprd_aon_apb_update_bit(busmon.tmr->ext_reg_off, busmon.tmr->ext_en_bit, 0);
		}
		if (DONOT_CARE != busmon.tmr->ext1_reg_off) {
			sprd_aon_apb_update_bit(busmon.tmr->ext1_reg_off, busmon.tmr->ext1_en_bit, 0);
		}
#endif
		busmon.is_glb_enabled = 0;
	}
}

static int busmon_read(int p_id, int b_id, int c_id)
{
	unsigned long ret_cnt, prev_cnt;
	ret_cnt = busmon_raw_read(p_id, b_id, c_id);
	prev_cnt = busmon.prev_count[p_id][b_id][c_id];
	busmon.prev_count[p_id][b_id][c_id] = ret_cnt;
	if (ret_cnt > prev_cnt ) {
		ret_cnt -= prev_cnt;
	} else {
		ret_cnt += 0xFFFFFFFF - prev_cnt;
	}
	return ret_cnt;
}

static int gator_events_busmon_read(int **buffer, bool sched_switch)
{
	unsigned int pub_id, busmon_id, counter_id;
	int p_id, b_id, c_id;
	int len = 0;

	/* System wide counters - read from one core only */
	if (!on_primary_core() || sched_switch)
		return 0;

	busmon_pmu_ctrl(0);
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
				if (busmon_counters[pub_id][busmon_id][counter_id].enabled) {
					p_id = pub_id;
					b_id = busmon_id;
					c_id = counter_id;
					busmon_buffer[len++] = busmon_counters[pub_id][busmon_id][counter_id].key;
					busmon_buffer[len++] = busmon_read(p_id, b_id, c_id);
				}
			}
		}
	}
	busmon_pmu_ctrl(1);

	if (buffer)
		*buffer = busmon_buffer;

	return len;
}

static void gator_events_busmon_shutdown(void)
{
	int pub_id;
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		if (busmon.base_reg[pub_id]) {
			iounmap(busmon.base_reg[pub_id]);
			busmon.base_reg[pub_id] = 0;
		}
		if (busmon.pub_busmon_cfg[pub_id]) {
			iounmap(busmon.pub_busmon_cfg[pub_id]);
			busmon.pub_busmon_cfg[pub_id] = 0;
		}
	}
#if !defined(CONFIG_OF)
	if (busmon.aon_apb_base) {
		iounmap(busmon.aon_apb_base);
		busmon.aon_apb_base = 0;
	}
	if (busmon.pmu_apb_base) {
		iounmap(busmon.pmu_apb_base);
		busmon.pmu_apb_base = 0;
	}
#endif
	if (busmon.busmon_timer_base) {
		iounmap(busmon.busmon_timer_base);
		busmon.busmon_timer_base = 0;
	}
}

static struct gator_interface gator_events_busmon_interface = {
	.name = "sprd_perf_busmon",
	.shutdown = gator_events_busmon_shutdown,
	.create_files = gator_events_busmon_create_files,
	.start = gator_events_busmon_start,
	.stop = gator_events_busmon_stop,
	.read = gator_events_busmon_read,
};

/* Must not be static! */
int gator_events_sprd_ptm_init(void)
{
	unsigned int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);
	memset((void*)&busmon, 0, sizeof(busmon));
	memset((void*)&busmon_counters, 0, sizeof(busmon_counters));

	if (0 == sprd_linux_get_base_addr(PTM)) {
		for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
			for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
				for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
					busmon_counters[pub_id][busmon_id][counter_id].enabled = 0;
					busmon_counters[pub_id][busmon_id][counter_id].key = gator_events_get_key();
				}
			}
			busmon.pub_busmon_cfg[pub_id] = ioremap_nocache(busmon.cfg[pub_id].bm_glb_reg&0xFFFF0000, SZ_4K);
			if (!busmon.pub_busmon_cfg[pub_id]) {
				pr_err("allocate io memory failed! pub_id=%d\n", pub_id);
				while(pub_id--) {
					iounmap(busmon.pub_busmon_cfg[pub_id]);
					busmon.pub_busmon_cfg[pub_id] = 0;
				}
				return 0;
			}
		}

		if (busmon.tmr) {
			busmon.busmon_timer_base = ioremap_nocache(busmon.tmr->reg_base, SZ_4K);
			if (!busmon.busmon_timer_base) {
				pr_err("allocate io memory failed! busmon timer\n");
				return 0;
			}
		}

		return gator_events_install(&gator_events_busmon_interface);
	}
	return 0;
}
