# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/magnetometer/inc
ifeq ($(CONFIG_CUSTOM_KERNEL_GYROSCOPE_MODULE),y)
obj-y   +=  mag_module.o
else
obj-y   +=  mag.o
endif
obj-y   +=  mag_factory.o

ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-$(CONFIG_MTK_MAGHUB)        +=  maghub/
else
obj-$(CONFIG_MTK_QMC6983_NEW)	+=  qmc6983-new/
obj-$(CONFIG_MTK_QMCX983)	+=  qmcX983/
endif
