
GATOR_FLAGS := CONFIG_GATOR_SPRD_SUPPORT=y
ifeq ($(strip $(TARGET_ARCH)),x86_64)
GATOR_FLAGS += CONFIG_GATOR_SPRD_X86_SUPPORT=y
endif


#TODO set true to support mali
my_GATOR_MALI_SUPPORT := false
GATOR_MALI_FLAGS :=

ifeq ($(strip $(my_GATOR_MALI_SUPPORT)),true)
$(info [gator.ko]*** support GPU platform: $(TARGET_GPU_PLATFORM) ***)
ifeq ($(strip $(TARGET_GPU_PLATFORM)),midgard)
GATOR_MALI_FLAGS += CONFIG_GATOR_WITH_MALI_SUPPORT=y
GATOR_MALI_FLAGS += CONFIG_GATOR_MALI_MIDGARD=y
GATOR_MALI_FLAGS += DDK_DIR=$(ANDROID_BUILD_TOP)/vendor/sprd/external/drivers/gpu/$(TARGET_GPU_PLATFORM)
GATOR_MALI_FLAGS += CONFIG_GATOR_MALI_MIDGARD_PATH=mali
endif
$(info [gator.ko]*** $(GATOR_MALI_FLAGS) ***)
endif
GATOR_FLAGS += $(GATOR_MALI_FLAGS)

GATOR_FLAGS += EXTRA_CFLAGS=-Wno-error

KO_MODULE_OUT := $(BSP_MODULES_OUT)/$(KO_MODULE_NAME)
KO_MODULE_KBUILD := $(CURDIR)/Makefile

.PHONY: modules modules_install clean

modules:
	@mkdir -p $(KO_MODULE_OUT) && ln -snf $(KO_MODULE_KBUILD) $(KO_MODULE_OUT)/Kbuild
	@ln -snf $(CURDIR) $(KO_MODULE_OUT)/source
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $(GATOR_FLAGS) $@

modules_install:
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) $(GATOR_FLAGS) $@

# Remove the out directory wholly
clean:
	@#$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@
	rm -rf $(KO_MODULE_OUT)
