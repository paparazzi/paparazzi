# Hey Emacs, this is a -*- makefile -*-
#
# IMU from Goodluckbuy
#


IMU_GL1_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_gl1.h\"
IMU_GL1_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_GL1_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_gl1.c

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_GL1_CFLAGS  += -DUSE_IMU
endif

# Accelerometer
IMU_GL1_SRCS   += peripherals/adxl345_i2c.c

# Gyro
IMU_GL1_SRCS   += peripherals/l3g4200.c

# Magnetometer
IMU_GL1_SRCS   += peripherals/hmc58xx.c

ifeq ($(ARCH), lpc21)
ifndef GL1_I2C_DEV
GL1_I2C_DEV=i2c0
endif
else ifeq ($(ARCH), stm32)
ifndef GL1_I2C_DEV
GL1_I2C_DEV=i2c2
endif
endif

# convert i2cx to upper case
GL1_I2C_DEV_UPPER=$(shell echo $(GL1_I2C_DEV) | tr a-z A-Z)

IMU_GL1_CFLAGS += -DGL1_I2C_DEV=$(GL1_I2C_DEV)
IMU_GL1_CFLAGS += -DUSE_$(GL1_I2C_DEV_UPPER)

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(IMU_GL1_CFLAGS)
ap.srcs   += $(IMU_GL1_SRCS)



#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
