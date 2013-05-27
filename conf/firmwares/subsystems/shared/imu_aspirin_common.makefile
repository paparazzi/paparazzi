# Hey Emacs, this is a -*- makefile -*-
#
# Common part for all Aspirin IMUs
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

IMU_ASPIRIN_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_aspirin.h\"
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_ASPIRIN_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c

#IMU_ASPIRIN_SRCS   += $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c
IMU_ASPIRIN_CFLAGS += -DASPIRIN_ARCH_INDEP

include $(CFG_SHARED)/spi_master.makefile

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ASPIRIN_CFLAGS  += -DUSE_IMU
endif

# Accelerometer
IMU_ASPIRIN_SRCS   += peripherals/adxl345_spi.c

# Gyro
IMU_ASPIRIN_SRCS   += peripherals/itg3200.c

# Magnetometer
#IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_ASPIRIN_SRCS   += peripherals/hmc58xx.c

ifeq ($(ARCH), lpc21)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI_SLAVE0
IMU_ASPIRIN_CFLAGS += -DASPIRIN_SPI_SLAVE_IDX=SPI_SLAVE0
IMU_ASPIRIN_CFLAGS += -DASPIRIN_SPI_DEV=spi1
IMU_ASPIRIN_CFLAGS += -DUSE_SPI1
ifndef ASPIRIN_I2C_DEV
ASPIRIN_I2C_DEV=i2c0
endif
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI2
# Slave select configuration
# SLAVE2 is on PB12 (NSS) (ADXL345 CS)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI_SLAVE2
ifndef ASPIRIN_I2C_DEV
ASPIRIN_I2C_DEV=i2c2
endif
endif

# convert i2cx to upper case
ASPIRIN_I2C_DEV_UPPER=$(shell echo $(ASPIRIN_I2C_DEV) | tr a-z A-Z)

IMU_ASPIRIN_CFLAGS += -DASPIRIN_I2C_DEV=$(ASPIRIN_I2C_DEV)
IMU_ASPIRIN_CFLAGS += -DUSE_$(ASPIRIN_I2C_DEV_UPPER)

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
