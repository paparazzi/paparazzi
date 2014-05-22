# Hey Emacs, this is a -*- makefile -*-
#
# Navstik onboard IMU
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
$(TARGET).CFLAGS  += -DUSE_IMU -DIMU_NAVSTIK -DIMU_TYPE_H=\"imu/imu_navstik.h\"
$(TARGET).srcs    += $(SRC_SUBSYSTEMS)/imu.c $(SRC_SUBSYSTEMS)/imu/imu_navstik.c
$(TARGET).srcs    += peripherals/hmc58xx.c
$(TARGET).srcs    += peripherals/mpu60x0.c peripherals/mpu60x0_i2c.c


NAVSTIK_MAG_I2C_DEV ?= i2c3
NAVSTIK_MPU_I2C_DEV ?= i2c1

NAVSTIK_MAG_I2C_DEV_UPPER=$(shell echo $(NAVSTIK_MAG_I2C_DEV) | tr a-z A-Z)
NAVSTIK_MAG_I2C_DEV_LOWER=$(shell echo $(NAVSTIK_MAG_I2C_DEV) | tr A-Z a-z)
NAVSTIK_MPU_I2C_DEV_UPPER=$(shell echo $(NAVSTIK_MPU_I2C_DEV) | tr a-z A-Z)
NAVSTIK_MPU_I2C_DEV_LOWER=$(shell echo $(NAVSTIK_MPU_I2C_DEV) | tr A-Z a-z)

$(TARGET).CFLAGS  += -DNAVSTIK_MAG_I2C_DEV=$(NAVSTIK_MAG_I2C_DEV_LOWER) -DNAVSTIK_MPU_I2C_DEV_UPPER=$(NAVSTIK_MPU_I2C_DEV_LOWER)
$(TARGET).CFLAGS  += -DUSE_$(NAVSTIK_MAG_I2C_DEV_UPPER)=1 -DUSE_$(NAVSTIK_MPU_I2C_DEV_UPPER)=1
