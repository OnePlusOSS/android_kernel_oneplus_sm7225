# oem device deriver
obj-y += input/
#obj-y += framework/
obj-y += kernel/
obj-y += coretech/
obj-y += fs/
obj-y += misc/
obj-y += power/
obj-y += vibrator/
obj-y += oneplus_healthinfo/
obj-y += mm/
obj-y += opslalib/
ifeq ($(filter billie2%, $(OEM_TARGET_PRODUCT)),)
obj-y += vl53L1/
endif
obj-$(CONFIG_OP_FREEZER) += op_freezer/
obj-y += hung_task_enhance/
ifneq ($(filter ebba%, $(OEM_TARGET_PRODUCT)),)
obj-y += wt_sys/
endif
