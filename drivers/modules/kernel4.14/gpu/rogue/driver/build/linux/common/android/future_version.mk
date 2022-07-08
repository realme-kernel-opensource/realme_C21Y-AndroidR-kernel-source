########################################################################### ###
#@Title         Non-public feature checks
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

# Put feature checks requiring NDA/partner source access here.
#

# Support Mapper 4.0 hal since R
#
PVR_ANDROID_HAS_MAPPER_4 := 1

# Disable nnhal temporarily to pass the build.
#
COMPONENTS := $(filter-out nnhal nnhal_unit_test,$(COMPONENTS))

# Google removed GCC from Android R prebuilts. Use clang for kbuild.
# Using set of LLVM tools by default. But they could be commented out
# depending on platforms.
#
override KERNEL_CC := clang
KERNEL_LD := ld.lld
KERNEL_NM := llvm-nm
KERNEL_OBJCOPY := llvm-objcopy

# CCACHE doesn't work with clang for kernel build.
# Disable for now
#
override USE_CCACHE := 0

# Disable PVR_APPINTS_IPC as
# android::hardware::details::return_status::onValueRetrieval() const
# is undefined.
#
override PVR_APPHINTS_IPC := 0

JAVA_VERSION := 11

# Support gralloc v4 by default on Android R.
#
PVR_ANDROID_HAS_GRALLOC_4 ?= 1
ifeq ($(PVR_ANDROID_HAS_GRALLOC_4),1)
  override PVR_ANDROID_HAS_GRALLOC_METADATA := 1
endif
