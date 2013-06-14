# Hey Emacs, this is a -*- makefile -*-
#
# Drotek 10DOF V2 IMU via I2C
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <!-- these gyro and accel calib values are the defaults for aspirin2.1 -->
#    <define name="GYRO_X_NEUTRAL" value="0"/>
#    <define name="GYRO_Y_NEUTRAL" value="0"/>
#    <define name="GYRO_Z_NEUTRAL" value="0"/>
#
#    <define name="GYRO_X_SENS" value="4.359" integer="16"/>
#    <define name="GYRO_Y_SENS" value="4.359" integer="16"/>
#    <define name="GYRO_Z_SENS" value="4.359" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="0"/>
#    <define name="ACCEL_Y_NEUTRAL" value="0"/>
#    <define name="ACCEL_Z_NEUTRAL" value="0"/>
#
#    <define name="ACCEL_X_SENS" value="4.905" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="4.905" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="4.905" integer="16"/>
#
#    <!-- replace the mag calibration with your own-->
#    <define name="MAG_X_NEUTRAL" value="-45"/>
#    <define name="MAG_Y_NEUTRAL" value="334"/>
#    <define name="MAG_Z_NEUTRAL" value="7"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#
#  </section>
#
#


# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_DROTEK_2_CFLAGS  = -DUSE_IMU
endif

IMU_DROTEK_2_CFLAGS += -DIMU_TYPE_H=\"imu/imu_drotek_10dof_v2.h\"
IMU_DROTEK_2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_DROTEK_2_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_drotek_10dof_v2.c
IMU_DROTEK_2_SRCS   += peripherals/mpu60x0.c
IMU_DROTEK_2_SRCS   += peripherals/mpu60x0_i2c.c

# Magnetometer
IMU_DROTEK_2_SRCS   += peripherals/hmc58xx.c


# set default i2c bus
ifndef DROTEK_2_I2C_DEV
ifeq ($(ARCH), lpc21)
DROTEK_2_I2C_DEV=i2c0
else ifeq ($(ARCH), stm32)
DROTEK_2_I2C_DEV=i2c2
endif
endif

# convert i2cx to upper case
DROTEK_2_I2C_DEV_UPPER=$(shell echo $(DROTEK_2_I2C_DEV) | tr a-z A-Z)

IMU_DROTEK_2_CFLAGS += -DDROTEK_2_I2C_DEV=$(DROTEK_2_I2C_DEV)
IMU_DROTEK_2_CFLAGS += -DUSE_$(DROTEK_2_I2C_DEV_UPPER)


# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example

ap.CFLAGS += $(IMU_DROTEK_2_CFLAGS)
ap.srcs   += $(IMU_DROTEK_2_SRCS)


#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
