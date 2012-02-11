# Hey Emacs, this is a -*- makefile -*-
#
# Booz2 IMU booz2v1.2
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_X_NEUTRAL" value="33924"/>
#    <define name="GYRO_Y_NEUTRAL" value="33417"/>
#    <define name="GYRO_Z_NEUTRAL" value="32809"/>
#
#    <define name="GYRO_X_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Y_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Z_SENS" value="1.01" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="32081"/>
#    <define name="ACCEL_Y_NEUTRAL" value="33738"/>
#    <define name="ACCEL_Z_NEUTRAL" value="32441"/>
#
#    <define name="ACCEL_X_SENS" value="2.50411474" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="2.48126183" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="2.51396167" integer="16"/>
#
#    <define name="MAG_X_NEUTRAL" value="2358"/>
#    <define name="MAG_Y_NEUTRAL" value="2362"/>
#    <define name="MAG_Z_NEUTRAL" value="2119"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#    <define name="MAG_45_HACK" value="1"/>
#
#  </section>
#
#

#
# param: MAX_1168_DRDY_PORT


# common Booz2 IMU files
include $(CFG_SHARED)/imu_b2_common.makefile

# imu Booz2 v1.2
imu_CFLAGS += -DIMU_B2_VERSION_1_2


ifeq ($(ARCH), lpc21)
imu_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC58XX
imu_CFLAGS += -DHMC58XX_I2C_DEVICE=i2c1 -DUSE_I2C1
imu_srcs += peripherals/hmc58xx.c
else ifeq ($(ARCH), stm32)
imu_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC5843
imu_srcs += peripherals/hmc5843.c
imu_srcs += $(SRC_ARCH)/peripherals/hmc5843_arch.c
imu_CFLAGS += -DUSE_I2C2 -DUSE_EXTI9_5_IRQ
endif

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)
