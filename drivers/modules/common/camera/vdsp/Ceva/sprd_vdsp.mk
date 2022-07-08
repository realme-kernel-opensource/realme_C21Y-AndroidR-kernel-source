#ifeq ($(strip $(TARGET_BOARD_VDSP_MODULAR_KERNEL)),ceva1.0)
PRODUCT_PACKAGES += vdsp.ko

PRODUCT_COPY_FILES += vendor/sprd/modules/libcamera/kernel_module/vdsp/Ceva/init.sprd_vdsp.rc:vendor/etc/init/init.sprd_vdsp.rc
#endif

