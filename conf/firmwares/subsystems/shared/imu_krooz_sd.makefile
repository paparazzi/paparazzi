# Hey Emacs, this is a -*- makefile -*-
#
# KroozSD IMU
#

IMU_KROOZ_CFLAGS  = -DUSE_IMU
IMU_KROOZ_CFLAGS += -DIMU_TYPE_H=\"boards/krooz/imu_krooz.h\"

IMU_KROOZ_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
										 $(SRC_BOARD)/imu_krooz.c             \
										 $(SRC_ARCH)/subsystems/imu/imu_krooz_sd_arch.c

IMU_KROOZ_I2C_DEV=i2c2
IMU_KROOZ_CFLAGS += -DUSE_I2C -DUSE_I2C2 -DI2C2_CLOCK_SPEED=400000

IMU_KROOZ_CFLAGS += -DIMU_KROOZ_I2C_DEV=$(IMU_KROOZ_I2C_DEV)
IMU_KROOZ_SRCS += peripherals/mpu60x0.c
IMU_KROOZ_SRCS += peripherals/mpu60x0_i2c.c
IMU_KROOZ_SRCS += peripherals/hmc58xx.c

AHRS_PROPAGATE_FREQUENCY ?= 512
AHRS_CORRECT_FREQUENCY ?= 512
ap.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
ap.CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)

ap.CFLAGS += $(IMU_KROOZ_CFLAGS)
ap.srcs   += $(IMU_KROOZ_SRCS)

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
