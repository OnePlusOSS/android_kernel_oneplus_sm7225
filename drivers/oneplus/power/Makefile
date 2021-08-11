# oem device driver for charge
#
ifneq ($(filter ebba%, $(OEM_TARGET_PRODUCT)),)
obj-y += oplus_chg/
else
obj-$(CONFIG_OP_POWER_SUPPLY) += supply/
obj-$(CONFIG_OP_POWER_SUPPLY) += OP_power_cut_test.o
endif

