ifneq ($(KERNELRELEASE),)

FAIL_REASON :=
GATOR_KO_PREBUILD_CHECK := pass
ifneq ($(CONFIG_PROFILING),y)
FAIL_REASON += CONFIG_PROFILING=$(CONFIG_PROFILING)
GATOR_KO_PREBUILD_CHECK := fail
endif
ifneq ($(CONFIG_PERF_EVENTS),y)
FAIL_REASON +=CONFIG_PERF_EVENTS=$(CONFIG_PERF_EVENTS)
GATOR_KO_PREBUILD_CHECK := fail
endif
ifneq ($(CONFIG_HIGH_RES_TIMERS),y)
FAIL_REASON +=CONFIG_HIGH_RES_TIMERS=$(CONFIG_HIGH_RES_TIMERS)
GATOR_KO_PREBUILD_CHECK := fail
endif
ifneq ($(CONFIG_MODULES),y)
FAIL_REASON +=CONFIG_MODULES=$(CONFIG_MODULES)
GATOR_KO_PREBUILD_CHECK := fail
endif
ifneq ($(CONFIG_MODULE_UNLOAD),y)
FAIL_REASON +=CONFIG_MODULE_UNLOAD=$(CONFIG_MODULE_UNLOAD)
GATOR_KO_PREBUILD_CHECK := fail
endif

ifneq ($(CONFIG_GATOR_SPRD_X86_SUPPORT),y)
ifneq ($(CONFIG_HW_PERF_EVENTS),y)
FAIL_REASON +=CONFIG_HW_PERF_EVENTS=$(CONFIG_HW_PERF_EVENTS)
GATOR_KO_PREBUILD_CHECK := fail
endif
endif

TRACER_CHECK := fail
ifeq ($(CONFIG_GENERIC_TRACER),y)
TRACER_CHECK := pass
endif
ifeq ($(CONFIG_TRACING),y)
TRACER_CHECK := pass
endif
ifeq ($(CONFIG_CONTEXT_SWITCH_TRACER),y)
TRACER_CHECK := pass
endif

ifneq ($(TRACER_CHECK),pass)
FAIL_REASON +=CONFIG_GENERIC_TRACER=$(CONFIG_GENERIC_TRACER)
FAIL_REASON +=CONFIG_TRACING=$(CONFIG_TRACING)
FAIL_REASON +=CONFIG_CONTEXT_SWITCH_TRACER=$(CONFIG_CONTEXT_SWITCH_TRACER)
GATOR_KO_PREBUILD_CHECK := fail
endif

ifneq ($(GATOR_KO_PREBUILD_CHECK),pass)
CONFIG_GATOR ?= m
obj-$(CONFIG_GATOR) := gator.o
gator-y := gator_check_fail.o
$(info [gator.ko] the Required kernel configuration check failed!)
$(info [gator.ko] because $(FAIL_REASON))
else

EXTRA_CFLAGS += -DSPRD_LINUX_BACK_PORTING_walk_stackfram

# Uncomment the following line to enable kernel stack unwinding within gator, or update gator_backtrace.c
# EXTRA_CFLAGS +=   -DGATOR_KERNEL_STACK_UNWINDING

CONFIG_GATOR ?= m
obj-$(CONFIG_GATOR) := gator.o

gator-y :=  gator_main.o \
            gator_events_block.o \
            gator_events_irq.o \
            gator_events_meminfo.o \
            gator_events_mmapped.o \
            gator_events_net.o \
            gator_events_sched.o

       chk_events.h = :
 quiet_chk_events.h = echo '  CHK     $@'
silent_chk_events.h = :


ifeq ($(CONFIG_GATOR_SPRD_SUPPORT),y)
  gator-y += gator_events_sprd_perf_busmon.o \
            gator_events_sprd_ptm.o
  EXTRA_CFLAGS += -DSPRD_SUPPORT
endif

# Convert the old GATOR_WITH_MALI_SUPPORT to the new kernel flags
ifneq ($(GATOR_WITH_MALI_SUPPORT),)
  CONFIG_GATOR_WITH_MALI_SUPPORT := y
  ifeq ($(GATOR_WITH_MALI_SUPPORT),MALI_MIDGARD)
    CONFIG_GATOR_MALI_4XXMP := n
    CONFIG_GATOR_MALI_MIDGARD := y
    EXTRA_CFLAGS += -DMALI_SUPPORT=MALI_MIDGARD_OR_BIFROST
  else
    ifeq ($(GATOR_WITH_MALI_SUPPORT),MALI_BIFROST)
      CONFIG_GATOR_MALI_4XXMP := n
      CONFIG_GATOR_MALI_MIDGARD := y
      EXTRA_CFLAGS += -DMALI_SUPPORT=MALI_MIDGARD_OR_BIFROST
    else
      # Neither Midgard, nor Bifrost, must be 4xx then
      CONFIG_GATOR_MALI_4XXMP := y
      CONFIG_GATOR_MALI_MIDGARD := n
      EXTRA_CFLAGS += -DMALI_SUPPORT=$(GATOR_WITH_MALI_SUPPORT)
    endif
  endif
  ifneq ($(GATOR_MALI_INTERFACE_STYLE),)
    EXTRA_CFLAGS += -DGATOR_MALI_INTERFACE_STYLE=$(GATOR_MALI_INTERFACE_STYLE)
  endif
  ifneq ($(GATOR_MALI_4XXMP_PATH),)
    CONFIG_GATOR_MALI_4XXMP_PATH = $(GATOR_MALI_4XXMP_PATH)
  endif
  CONFIG_GATOR_MALI_4XXMP_PATH ?= drivers/gpu/arm/mali400mp
  ifneq ($(GATOR_MALI_MIDGARD_PATH),)
    CONFIG_GATOR_MALI_MIDGARD_PATH = $(GATOR_MALI_MIDGARD_PATH)
  endif
  CONFIG_GATOR_MALI_MIDGARD_PATH ?= drivers/gpu/arm/midgard
endif

ifeq ($(CONFIG_GATOR_WITH_MALI_SUPPORT),y)
  ifeq ($(CONFIG_GATOR_MALI_MIDGARD),y)
    gator-y +=  gator_events_mali_midgard.o \
		gator_events_mali_midgard_hw.o
    include $(src)/mali_midgard.mk
  else
    gator-y +=  gator_events_mali_4xx.o
  endif
  gator-y +=    gator_events_mali_common.o

  ifneq ($(CONFIG_GATOR_MALI_4XXMP_PATH),)
    ccflags-$(CONFIG_GATOR_MALI_4XXMP) += -I$(CONFIG_GATOR_MALI_4XXMP_PATH)
  endif
  ifneq ($(CONFIG_GATOR_MALI_MIDGARD_PATH),)
    ccflags-$(CONFIG_GATOR_MALI_MIDGARD) += -I$(CONFIG_GATOR_MALI_MIDGARD_PATH)
  endif
  ccflags-$(CONFIG_GATOR_MALI_4XXMP) += -DMALI_SUPPORT=MALI_4xx
  ccflags-$(CONFIG_GATOR_MALI_MIDGARD) += -DMALI_SUPPORT=MALI_MIDGARD_OR_BIFROST
endif

gator-additional-dependencies := $(obj)/generated_gator_src_md5.h
clean-files := generated_gator_src_md5.h

# Should the original or new block_rq_complete API be used?
OLD_BLOCK_RQ_COMPLETE := $(shell grep -A3 block_rq_complete $(srctree)/include/trace/events/block.h | grep nr_bytes -q; echo $$?)
EXTRA_CFLAGS += -DOLD_BLOCK_RQ_COMPLETE=$(OLD_BLOCK_RQ_COMPLETE)

gator-$(CONFIG_ARM) +=  gator_events_l2c-310.o

gator-$(CONFIG_ARM64) +=

$(obj)/gator_main.o: $(gator-additional-dependencies)

# Note, in the recipe below we use "cd $(srctree) && cd $(src)" rather than
# "cd $(srctree)/$(src)" because under DKMS $(src) is an absolute path, and we
# can't just use $(src) because for normal kernel builds this is relative to
# $(srctree)
$(obj)/generated_gator_src_md5.h: $(wildcard $(src)/gator_*.h $(src)/gator_*.c $(src)/mali/*.h)
	@$($(quiet)chk_events.h)
	$(Q)cd $(srctree) && cd $(src) ; $(CONFIG_SHELL) -c "echo 'static char * gator_src_md5 = \"'\`ls gator_*.c gator_*.h mali/*.h | grep -Ev '^(generated_gator_src_md5\.h|gator\.mod\.c)$$' | LC_ALL=C sort | xargs cat | md5sum | cut -b 1-32\`'\";'" > $(abspath $@)

endif

else
KO_MODULE_NAME := gator
include bsp.mk

all:
	@echo
	@echo "usage:"
	@echo "      make -C <kernel_build_dir> M=\`pwd\` ARCH=arm CROSS_COMPILE=<...> modules"
	@echo
	$(error)

endif
