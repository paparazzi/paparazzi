# Hey Emacs, this is a -*- makefile -*-
#
# Common part for all Aspirin IMUs using I2C only
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
#

IMU_ASPIRIN_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_aspirin_i2c.h\"
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_ASPIRIN_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_aspirin_i2c.c

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ASPIRIN_CFLAGS  += -DUSE_IMU
endif

# Accelerometer via I2C
IMU_ASPIRIN_SRCS   += peripherals/adxl345_i2c.c

# Gyro
IMU_ASPIRIN_SRCS   += peripherals/itg3200.c

# Magnetometer
IMU_ASPIRIN_SRCS   += peripherals/hmc58xx.c

ifeq ($(ARCH), lpc21)
IMU_ASPIRIN_CFLAGS += -DASPIRIN_I2C_DEV=i2c1
IMU_ASPIRIN_CFLAGS += -DUSE_I2C1
IMU_ASPIRIN_CFLAGS += -DI2C1_VIC_SLOT=12
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN_CFLAGS += -DUSE_I2C2
endif

include $(CFG_SHARED)/imu_nps.makefile
