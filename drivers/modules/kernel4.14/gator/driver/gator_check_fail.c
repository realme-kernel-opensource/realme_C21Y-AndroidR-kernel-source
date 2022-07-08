/**
 * This is a dummy file
 */
#define pr_fmt(fmt) "[gator.failed]" fmt
#include <linux/module.h>

static int __init gator_module_init(void)
{
#ifndef CONFIG_PROFILING
	pr_err("CONFIG_PROFILING failed!");
#endif
#ifndef CONFIG_PERF_EVENTS
	pr_err("CONFIG_PERF_EVENTS failed!");
#endif
#ifndef CONFIG_HIGH_RES_TIMERS
	pr_err("CONFIG_HIGH_RES_TIMERS failed!");
#endif
#ifndef CONFIG_MODULES
	pr_err("CONFIG_MODULES failed!");
#endif
#ifndef CONFIG_MODULE_UNLOAD
	pr_err("CONFIG_MODULE_UNLOAD failed!");
#endif
#ifndef CONFIG_HW_PERF_EVENTS
	pr_err("CONFIG_HW_PERF_EVENTS failed!");
#endif
#if !((defined CONFIG_GENERIC_TRACER) || (defined CONFIG_TRACING) || (defined CONFIG_CONTEXT_SWITCH_TRACER))
	pr_err("CONFIG_GENERIC_TRACER || CONFIG_TRACING || CONFIG_CONTEXT_SWITCH_TRACER failed!");
#endif
	return -1;
}

static void __exit gator_module_exit(void)
{
}

module_init(gator_module_init);
module_exit(gator_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SPRD");
MODULE_DESCRIPTION("Gator Check Failed!");
