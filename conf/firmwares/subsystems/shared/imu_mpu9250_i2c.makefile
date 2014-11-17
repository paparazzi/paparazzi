# Hey Emacs, this is a -*- makefile -*-
#
# MPU9250 IMU via I2C
#

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_MPU9250_CFLAGS  = -DUSE_IMU
endif

IMU_MPU9250_CFLAGS += -DIMU_TYPE_H=\"imu/imu_mpu9250_i2c.h\"
IMU_MPU9250_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_MPU9250_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_mpu9250_i2c.c
IMU_MPU9250_SRCS   += peripherals/mpu9250.c
IMU_MPU9250_SRCS   += peripherals/mpu9250_i2c.c

# Magnetometer
IMU_MPU9250_SRCS   += peripherals/ak8963.c


# set default i2c bus
ifeq ($(ARCH), lpc21)
MPU9250_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
MPU9250_I2C_DEV ?= i2c2
endif

ifeq ($(TARGET), ap)
ifndef MPU9250_I2C_DEV
$(error Error: MPU9250_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
MPU9250_I2C_DEV_UPPER=$(shell echo $(MPU9250_I2C_DEV) | tr a-z A-Z)
MPU9250_I2C_DEV_LOWER=$(shell echo $(MPU9250_I2C_DEV) | tr A-Z a-z)

IMU_MPU9250_CFLAGS += -DIMU_MPU9250_I2C_DEV=$(MPU9250_I2C_DEV_LOWER)
IMU_MPU9250_CFLAGS += -DUSE_$(MPU9250_I2C_DEV_UPPER)


# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets

ap.CFLAGS += $(IMU_MPU9250_CFLAGS)
ap.srcs   += $(IMU_MPU9250_SRCS)

test_imu.CFLAGS += $(IMU_MPU9250_CFLAGS)
test_imu.srcs   += $(IMU_MPU9250_SRCS)


#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
