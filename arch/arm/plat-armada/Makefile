
ifeq ($(CONFIG_PLAT_ARMADA),y)
	include $(srctree)/$(MACHINE)/config/mvRules.mk
endif

# This will never compile, because DUMMY will never by defined.
obj-$(DUMMY)   				:= dummy.o

obj-$(CONFIG_SHEEVA_DEEP_IDLE)		+= cpuidle.o armada_suspend.o
obj-$(CONFIG_PLAT_ARMADA)		+= pmu.o
