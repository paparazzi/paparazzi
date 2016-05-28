# Hey Emacs, this is a -*- makefile -*-
#
# IMU with MPU6000 via SPI and HMC5883 via I2c on the OpenPilot Revolution board.
# The IMU positive X-axis is as inidcated with arrows on the board,
# Z-axis is negative towards the side of the MPU and LEDs (that side is up)
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


include $(CFG_SHARED)/spi_master.makefile


IMU_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_mpu6000_hmc5883.h\"
IMU_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_mpu6000_hmc5883.c

# MPU
IMU_SRCS   += peripherals/mpu60x0.c
IMU_SRCS   += peripherals/mpu60x0_spi.c

# Magnetometer
IMU_SRCS   += peripherals/hmc58xx.c

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_CFLAGS  += -DUSE_IMU
endif

# HMC is on I2C1 on OpenPilot Revolution
IMU_HMC_I2C_DEV = i2c1

# convert i2cx to upper/lower case
IMU_HMC_I2C_DEV_UPPER=$(shell echo $(IMU_HMC_I2C_DEV) | tr a-z A-Z)
IMU_HMC_I2C_DEV_LOWER=$(shell echo $(IMU_HMC_I2C_DEV) | tr A-Z a-z)

IMU_CFLAGS += -DIMU_HMC_I2C_DEV=$(IMU_HMC_I2C_DEV_LOWER)
IMU_CFLAGS += -DUSE_$(IMU_HMC_I2C_DEV_UPPER)


# MPU600 is on SPI1 using SPI_SLAVE_2 as defined in openpilot_revo_1.0.h
IMU_MPU_SPI_DEV = spi1
IMU_MPU_SPI_SLAVE_IDX = SPI_SLAVE2

# convert spix to upper/lower case
IMU_MPU_SPI_DEV_UPPER=$(shell echo $(IMU_MPU_SPI_DEV) | tr a-z A-Z)
IMU_MPU_SPI_DEV_LOWER=$(shell echo $(IMU_MPU_SPI_DEV) | tr A-Z a-z)

IMU_CFLAGS += -DIMU_MPU_SPI_DEV=$(IMU_MPU_SPI_DEV_LOWER)
IMU_CFLAGS += -DUSE_$(IMU_MPU_SPI_DEV_UPPER)
IMU_CFLAGS += -DIMU_MPU_SPI_SLAVE_IDX=$(IMU_MPU_SPI_SLAVE_IDX)
IMU_CFLAGS += -DUSE_$(IMU_MPU_SPI_SLAVE_IDX)

# set channels and signs so that positive x-axis is indicated by arrows on board
# and the side with MPU and LEDs is up (negative z-axis)
IMU_CFLAGS += -DIMU_MPU_CHAN_X=1 -DIMU_MPU_CHAN_Y=0 -DIMU_MPU_CHAN_Z=2
IMU_CFLAGS += -DIMU_MPU_X_SIGN=-1 -DIMU_MPU_Y_SIGN=-1 -DIMU_MPU_Z_SIGN=-1
IMU_CFLAGS += -DIMU_HMC_CHAN_X=1 -DIMU_HMC_CHAN_Y=0 -DIMU_HMC_CHAN_Z=2
IMU_CFLAGS += -DIMU_HMC_X_SIGN=1 -DIMU_HMC_Y_SIGN=1 -DIMU_HMC_Z_SIGN=-1

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_CFLAGS)
$(TARGET).srcs += $(IMU_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
