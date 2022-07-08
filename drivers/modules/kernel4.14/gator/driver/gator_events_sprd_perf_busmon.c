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
 * To add them to the events.xml, create an events-sprd-perf-busmon.xml with the
 * following contents and rebuild gatord:
 *
 * event = <30:28><27:24><23:16><15:0>
 * <30:28>  the busmon PUB ID
 * <27:24>  the busmon ID
 * <23:16>   the counter ID
 * <15:0> the base address
 *<counter_set name="PUB0_BM0_cnt" count="7" />
 * <category name="SPRD A53 PUB0" counter_set="PUB0_BM0_cnt" per_cpu="no">
 *   <!--
 *	 PUB0_A53 Performence Bus Monitor
 *   -->
 *   <event event="0x00003002" title="A53" name="PUB0_RTRANS" display="accumulate" class="incident" description="PUB0 A53 Read Transaction"/>
 *   <event event="0x00013002" title="A53" name="PUB0_RBW" display="accumulate" class="incident" description="PUB0 A53 Read BandWidth"/>
 *   <event event="0x00023002" title="A53" name="PUB0_RLATENCY" display="accumulate" class="incident" description="PUB0 A53 Read Latency"/>
 *   <event event="0x00033002" title="A53" name="PUB0_WTRANS" display="accumulate" class="incident" description="PUB0 A53 Write Transaction"/>
 *   <event event="0x00043002" title="A53" name="PUB0_WBW" display="accumulate" class="incident" description="PUB0 A53 Write BandWidth"/>
 *   <event event="0x00053002" title="A53" name="PUB0_WLATENCY" display="accumulate" class="incident" description="PUB0 A53 Write Latency"/>
 *   <event event="0x00063002" title="A53" name="PUB0_PEAKBW" display="maximum" class="absolute" description="PUB0 A53 Peak BandWidth"/>
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

#define pr_fmt(fmt) "[gator.pref.busmon]" fmt
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
#define BUSMON_COUNTERS_NUM_PER_MAX 7
#define BUSMON_COUNTERS_NUM_MAX (BUSMON_PUB_NUM_MAX * BUSMON_NUM_MAX * BUSMON_COUNTERS_NUM_PER_MAX)

#define BUSMON_PUB_NUM (bm_gp_cfg[busmon.map->gp_id].busmon_gp_cnt)
#define BUSMON_NUM (busmon.cfg[pub_id].bm_cnt)
#define BUSMON_COUNTERS_NUM_PER BUSMON_COUNTERS_NUM_PER_MAX

#define GET_BASE_ADDR(e) ((e&0x0000FFFF)<<16)
#define GET_BUSMON_PUB_ID(e) ((e&0x70000000)>>28)
#define GET_BUSMON_ID(e) ((e&0x0F000000)>>24)
#define GET_COUNTER_ID(e) ((e&0x00FF0000)>>16)

static struct {
	unsigned long enabled;
	unsigned long event;
	unsigned long key;
} busmon_counters[BUSMON_PUB_NUM_MAX][BUSMON_NUM_MAX][BUSMON_COUNTERS_NUM_PER_MAX];

static struct {
	unsigned long is_enabled[BUSMON_PUB_NUM_MAX][BUSMON_NUM_MAX];
	unsigned long is_glb_enabled;
	void __iomem* base_reg[BUSMON_PUB_NUM_MAX][BUSMON_NUM_MAX];
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

/* Adds busmon_cntX directories and enabled, event, and key files to /dev/gator/events */
static int gator_events_busmon_create_files(struct super_block *sb,
					struct dentry *root)
{
	unsigned int pub_id, busmon_id, counter_id;

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

static int gator_events_busmon_start(void)
{
	int enabled = 0;
	int id, p_id;
	void __iomem * reg;
	unsigned long temp;
	unsigned int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);

	busmon_pmu_ctrl(0);
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
				if (busmon_counters[pub_id][busmon_id][counter_id].enabled) {
					busmon_dbg("enable PUB%d.BM%d.Counter%d event=0x%lx\n", pub_id, busmon_id, counter_id, (busmon_counters[pub_id][busmon_id][counter_id].event));
					/* enable busmon */
					p_id = GET_BUSMON_PUB_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
					id = GET_BUSMON_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
					busmon_dbg("Physical PUB%d.BM%d\n", p_id, id);
					if (!busmon.is_enabled[p_id][id]) {
						reg = busmon.pub_busmon_cfg[p_id] + (busmon.cfg[p_id].bm_glb_reg&0xFFFF);
						writel_relaxed(readl_relaxed(reg)|BIT(id+BUSMON_GLB_EN_BIT_OFF),reg);
						if (busmon.base_reg[p_id][id])
							iounmap(busmon.base_reg[p_id][id]);
						temp = GET_BASE_ADDR(busmon_counters[pub_id][busmon_id][counter_id].event);
						busmon.base_reg[p_id][id] = ioremap_nocache(temp, SZ_4K);
						reg = busmon.base_reg[p_id][id];
						if (!reg) {
							pr_err("allocate io memory failed! pub_id=%d bm_id=%d\n", p_id, id);
							busmon_pmu_ctrl(1);
							return -1;
						}
						/* setting peak window: 700000 */
						writel_relaxed(0xAAE60,reg+0x0004);
						/* enable busmon */
						writel_relaxed(readl_relaxed(reg)|BIT(0),reg);
						busmon.is_enabled[p_id][id] = 1;
					}
					enabled = 1;
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
	int id, p_id;
	void __iomem * reg;
	unsigned int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);

	busmon_pmu_ctrl(0);
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			for (counter_id = 0; counter_id < BUSMON_COUNTERS_NUM_PER; counter_id++) {
				busmon_dbg("disable PUB%d.BM%d.Counter%d event=0x%lx\n", pub_id, busmon_id, counter_id, (busmon_counters[pub_id][busmon_id][counter_id].event));
				/* disable busmon */
				p_id = GET_BUSMON_PUB_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
				id = GET_BUSMON_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
				busmon_dbg("Physical PUB%d.BM%d\n", p_id, id);
				if (busmon.is_enabled[p_id][id]) {
					reg = busmon.base_reg[p_id][id];
					writel_relaxed(readl_relaxed(reg)&~BIT(0),reg);
					if (busmon.base_reg[p_id][id]) {
						iounmap(busmon.base_reg[p_id][id]);
						busmon.base_reg[p_id][id] = 0;
					}
#if 0
					reg = busmon.pub_busmon_cfg[p_id] + (busmon.cfg[p_id].bm_glb_reg&0xFFFF);
					writel_relaxed(readl_relaxed(reg)&~BIT(id+BUSMON_GLB_EN_BIT_OFF),reg);
#endif
					busmon.is_enabled[p_id][id] = 0;
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
	int offset_addr[] = {
		0x0028,
		0x002C,
		0x0030,
		0x0034,
		0x0038,
		0x003C,
		0x0040,
	};
	return readl_relaxed(busmon.base_reg[p_id][b_id] + offset_addr[c_id]);
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
					p_id = GET_BUSMON_PUB_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
					b_id = GET_BUSMON_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
					c_id = GET_COUNTER_ID(busmon_counters[pub_id][busmon_id][counter_id].event);
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
	unsigned int pub_id, busmon_id;
	for (pub_id = 0; pub_id < BUSMON_PUB_NUM; pub_id++) {
		for (busmon_id = 0; busmon_id < BUSMON_NUM; busmon_id++) {
			if (busmon.base_reg[pub_id][busmon_id]) {
				iounmap(busmon.base_reg[pub_id][busmon_id]);
				busmon.base_reg[pub_id][busmon_id] = 0;
			}
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
int gator_events_sprd_perf_busmon_init(void)
{
	unsigned int pub_id, busmon_id, counter_id;

	busmon_dbg("entry %s\n", __func__);
	memset((void*)&busmon, 0, sizeof(busmon));
	memset((void*)&busmon_counters, 0, sizeof(busmon_counters));

	if (0 == sprd_linux_get_base_addr(PBM)) {
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
