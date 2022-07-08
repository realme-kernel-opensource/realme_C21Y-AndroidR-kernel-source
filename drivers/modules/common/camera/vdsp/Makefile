#
# sprd_vdsp.ko
#
# Makefile: for external make invocation
#
# Note:
# - Please refer to modules/sample/Makefile to find out what should be
#   done in this Makefile
#

KO_MODULE_NAME := sprd_vdsp
KO_MODULE_OUT := $(BSP_MODULES_OUT)/$(KO_MODULE_NAME)
KO_MODULE_KBUILD := $(CURDIR)/Kbuild

.PHONY: modules modules_install clean

ifneq ($(BSP_BOARD_CAMERA_MODULE_VDSP_DEVICE),Ceva)
modules modules_install clean:
	@echo "Skipped, Kbuild only for Ceva vdsp!"
else
modules:
	@mkdir -p $(KO_MODULE_OUT) && rsync -az $(KO_MODULE_KBUILD) $(KO_MODULE_OUT)/Kbuild
	@#ln -snf $(CURDIR) $(KO_MODULE_OUT)/source
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@

modules_install:
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) $@

# Remove the out directory wholly
clean:
	@#$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@
	rm -rf $(KO_MODULE_OUT)
endif
