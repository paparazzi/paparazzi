# Hey Emacs, this is a -*- makefile -*-
#
# Bebop onboard IMU
#
#
# if ACCEL and GYRO SENS/NEUTRAL are not defined,
# the defaults from the datasheet will be used
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="MAG_X_NEUTRAL" value="2358"/>
#    <define name="MAG_Y_NEUTRAL" value="2362"/>
#    <define name="MAG_Z_NEUTRAL" value="2119"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#
#  </section>
#

IMU_CFLAGS  = -DUSE_IMU -DIMU_BEBOP -DIMU_TYPE_H=\"imu/imu_bebop.h\"
IMU_SRCS    = $(SRC_SUBSYSTEMS)/imu.c $(SRC_SUBSYSTEMS)/imu/imu_bebop.c
IMU_SRCS    += peripherals/mpu60x0.c peripherals/mpu60x0_i2c.c
IMU_SRCS		+= peripherals/ak8963.c


BEBOP_MAG_I2C_DEV ?= i2c1
BEBOP_MPU_I2C_DEV ?= i2c2

BEBOP_MAG_I2C_DEV_UPPER=$(shell echo $(BEBOP_MAG_I2C_DEV) | tr a-z A-Z)
BEBOP_MAG_I2C_DEV_LOWER=$(shell echo $(BEBOP_MAG_I2C_DEV) | tr A-Z a-z)
BEBOP_MPU_I2C_DEV_UPPER=$(shell echo $(BEBOP_MPU_I2C_DEV) | tr a-z A-Z)
BEBOP_MPU_I2C_DEV_LOWER=$(shell echo $(BEBOP_MPU_I2C_DEV) | tr A-Z a-z)

IMU_CFLAGS  += -DBEBOP_MAG_I2C_DEV=$(BEBOP_MAG_I2C_DEV_LOWER) -DBEBOP_MPU_I2C_DEV=$(BEBOP_MPU_I2C_DEV_LOWER)
IMU_CFLAGS  += -DUSE_$(BEBOP_MAG_I2C_DEV_UPPER)=1 -DUSE_$(BEBOP_MPU_I2C_DEV_UPPER)=1

ap.CFLAGS += $(IMU_CFLAGS)
ap.srcs   += $(IMU_SRCS)

test_imu.CFLAGS += $(IMU_CFLAGS)
test_imu.srcs   += $(IMU_SRCS)

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
