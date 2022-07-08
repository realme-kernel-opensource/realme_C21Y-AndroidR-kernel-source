ifeq ($(BSP_SYSTEM_VERSION), sharkl3)
# pvrsrvkm.o
CURRENT_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/gpu/rogue/driver/build/linux/sharkl3_android; pwd)#$(MAKEFILE_PATH)/rogue/driver/build/linux/sharkl3_android

#include $(CLEAR_UNISOC_KO)
KO_MODULE_PATH      := $(shell cd $(CURRENT_PATH); pwd)
KO_MODULE_NAME      := pvrsrvkm
PLATFORM_VERSION := 10
export KERNELDIR ?= $(BSP_KERNEL_OUT)
export OUT := $(BSP_OUT_PLATFORM)/$(BSP_PRODUCT_NAME)/obj/modules/pvrsrvkm
ANDROID_ROOT := $(BSP_ROOT_DI)
export KERNEL_PATH := kernel4.14
TARGET_KERNEL_ARCH := $(BSP_BOARD_ARCH)
PRODUCT_OUT := $(BSP_ROOT_DI)
MALI_PLATFORM_NAME := sharkl3
export PVRSRV_APPHINT_HWRDEBUGDUMPLIMIT := 4

$(warning OUT: $(OUT))
$(warning CURRENT_PATH: $(CURRENT_PATH))
$(warning KERNEL_PATH: $(KERNEL_PATH))
#$(warning PLATFORM_VERSION: $(PLATFORM_VERSION))
#$(warning PLATFORM_VERSION_CODENAME: $(PLATFORM_VERSION_CODENAME))
#$(warning MALI_PLATFORM_NAME: $(MALI_PLATFORM_NAME))
#$(warning TARGET_ARCH: $(TARGET_ARCH))

BUILD_ := release
RGX_NOHW_ := 0
KO_MODULE_FLAGS     := BUILD=$(BUILD_)
KO_MODULE_FLAGS     += PLATFORM_VERSION=$(PLATFORM_VERSION)
KO_MODULE_FLAGS     += PLATFORM_VERSION_CODENAME=$(PLATFORM_VERSION_CODENAME)
KO_MODULE_FLAGS     += RGX_NOHW=$(RGX_NOHW_)
KO_MODULE_FLAGS     += BOARD_NAME=$(MALI_PLATFORM_NAME)
KO_MODULE_EXPORT    := export PVRSRV_NEED_PVR_DPF=1;
KO_MODULE_MAKEFILE  := true
ifeq ($(TARGET_KERNEL_ARCH), arm64)
TARGET_PRIMARY_ARCH := target_aarch64
else
TARGET_PRIMARY_ARCH := target_armv7-a
endif
KO_MODULE_TARGET    := $(OUT)/$(TARGET_PRIMARY_ARCH)/kbuild/$(KO_MODULE_NAME).ko
#include $(BUILD_UNISOC_KO)


UNISOC_CCFLAGS      := $(KO_MODULE_CCFLAGS)

#we can't set DEBUG default, becuase of re-define errors
#ifneq ($(TARGET_BUILD_VARIANT),user)
#UNISOC_CCFLAGS      += -DDEBUG
#endif

KO_MODULE_EXPORT    += export UNISOC_CCFLAGS="$(UNISOC_CCFLAGS)";
KO_MODULE_EXPORT    += export UNISOC_EXTRA_FLAGS="$(KO_MODULE_EXTRA_FLAGS)";

#convert to absolute directory
PRODUCT_OUT_ABSOLUTE:=$(shell cd $(PRODUCT_OUT); pwd)
ifeq ($(KO_MODULE_MAKEFILE), true)
COMMON_MODULE_FLAGS := -C $(KO_MODULE_PATH)
UNISOC_MAKEFILE_ABSOLUTE :=
else
UNISOC_MAKEFILE_ABSOLUTE:=$(shell cd $(OUT_DIR)/..; pwd)/$(shell dirname $(BUILD_UNISOC_KO))/Makefile
COMMON_MODULE_FLAGS := -f $(UNISOC_MAKEFILE_ABSOLUTE)
ifeq ($(KO_MODULE_MAKEFILE_RELOCATE), true)
UNISOC_MAKEFILE_ABSOLUTE :=
endif
endif
COMMON_MODULE_FLAGS += UNISOC_MAKEFILE="$(UNISOC_MAKEFILE_ABSOLUTE)"
COMMON_MODULE_FLAGS += UNISOC_MODULE="$(KO_MODULE_NAME)"
COMMON_MODULE_FLAGS += UNISOC_PREFIX="$(PRODUCT_OUT_ABSOLUTE)/kernel_modules/$(KO_MODULE_NAME)"
COMMON_MODULE_FLAGS += UNISOC_SRC_PATH="$(KO_MODULE_PATH)"
COMMON_MODULE_FLAGS += UNISOC_MODULE_OBJS="$(KO_MODULE_OBJS)"
COMMON_MODULE_FLAGS += UNISOC_MODULE_OBJS_EXCLUDE="$(KO_MODULE_OBJS_EXCLUDE) $(KO_MODULE_NAME).o"
COMMON_MODULE_FLAGS += ARCH="$(BSP_BOARD_ARCH)"
COMMON_MODULE_FLAGS += CROSS_COMPILE="$(BSP_KERNEL_CROSS_COMPILE)"
#COMMON_MODULE_FLAGS += KDIR="$(KERNELDIR)"
#COMMON_MODULE_FLAGS += KERNELDIR="$(KERNELDIR)"
ifeq ($(CC),clang)
COMMON_MODULE_FLAGS += KERNEL_CC=clang USE_CCACHE=
CROSS_TRIPLE := $(patsubst %-,%,$(notdir $(CROSS_COMPILE)))
CROSS_TRIPLE := $(CROSS_TRIPLE:aarch64-linux-androidkernel=aarch64-linux-android)
CROSS_TRIPLE := $(CROSS_TRIPLE:arm-linux-androidkernel=arm-linux-androideabi)
COMMON_MODULE_FLAGS += CROSS_TRIPLE=$(CROSS_TRIPLE)
endif

$(shell cd $(CURRENT_PATH))
$(BSP_MODULES_DIST)/$(KO_MODULE_NAME).ko: $(TARGET_PREBUILT_KERNEL)
	echo "current module name --- $(notdir $(basename $@)) ---"
	$(BUILT_$(notdir $(basename $@)))

.PHONY:modules

modules:
	$(MAKE) -f $(CURRENT_PATH)/Makefile.rogue $(KO_MODULE_FLAGS) $(COMMON_MODULE_FLAGS)

modules_install:
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(OUT)/$(TARGET_PRIMARY_ARCH)/kbuild $@

clean:
	rm -rf $(OUT)
else
.PHONY:modules
modules:
	echo "--skip sharkl3 pvrsrvkm modules--"
modules_install:
	echo "--skip sharkl3 pvrsrvkm modules_install--"
clean:
	echo "--skip sharkl3 pvrsrvkm clean--"
endif
