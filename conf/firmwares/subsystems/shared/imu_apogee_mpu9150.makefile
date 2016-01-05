# Hey Emacs, this is a -*- makefile -*-
#
# Apogee IMU
#

include $(CFG_SHARED)/imu_apogee.makefile

IMU_APOGEE_MPU9150_CFLAGS = -DAPOGEE_USE_MPU9150
IMU_APOGEE_MPU9150_SRCS = peripherals/ak8975.c

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_APOGEE_MPU9150_CFLAGS)
$(TARGET).srcs += $(IMU_APOGEE_MPU9150_SRCS)
endif

