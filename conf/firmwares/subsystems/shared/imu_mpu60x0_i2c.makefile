# Hey Emacs, this is a -*- makefile -*-
#
# MPU60X0 IMU via I2C
#

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_MPU60X0_CFLAGS  = -DUSE_IMU
endif

IMU_MPU60X0_CFLAGS += -DIMU_TYPE_H=\"imu/imu_mpu60x0_i2c.h\"
IMU_MPU60X0_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_MPU60X0_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_mpu60x0_i2c.c
IMU_MPU60X0_SRCS   += peripherals/mpu60x0.c
IMU_MPU60X0_SRCS   += peripherals/mpu60x0_i2c.c


# set default i2c bus
ifeq ($(ARCH), lpc21)
IMU_MPU60X0_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
IMU_MPU60X0_I2C_DEV ?= i2c2
endif

ifeq ($(TARGET), ap)
ifndef MPU60X0_I2C_DEV
$(error Error: MPU60X0_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
IMU_MPU60X0_I2C_DEV_UPPER=$(shell echo $(IMU_MPU60X0_I2C_DEV) | tr a-z A-Z)
IMU_MPU60X0_I2C_DEV_LOWER=$(shell echo $(IMU_MPU60X0_I2C_DEV) | tr A-Z a-z)

IMU_MPU60X0_CFLAGS += -DIMU_MPU60X0_I2C_DEV=$(IMU_MPU60X0_I2C_DEV_LOWER)
IMU_MPU60X0_CFLAGS += -DUSE_$(IMU_MPU60X0_I2C_DEV_UPPER)


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_MPU60X0_CFLAGS)
$(TARGET).srcs += $(IMU_MPU60X0_SRCS)
endif


#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
