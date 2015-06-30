# Hey Emacs, this is a -*- makefile -*-
#
# IMU with MPU6000 via SPI.
#
# if ACCEL and GYRO SENS/NEUTRAL are not defined,
# the defaults from the datasheet will be used
#

include $(CFG_SHARED)/spi_master.makefile


IMU_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_mpu6000.h\"
IMU_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_mpu6000.c

# MPU
IMU_SRCS   += peripherals/mpu60x0.c
IMU_SRCS   += peripherals/mpu60x0_spi.c

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_CFLAGS  += -DUSE_IMU
endif


# set default SPI device and slave index
IMU_MPU_SPI_DEV ?= spi1
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE0

# convert spix to upper/lower case
IMU_MPU_SPI_DEV_UPPER=$(shell echo $(IMU_MPU_SPI_DEV) | tr a-z A-Z)
IMU_MPU_SPI_DEV_LOWER=$(shell echo $(IMU_MPU_SPI_DEV) | tr A-Z a-z)

IMU_CFLAGS += -DIMU_MPU_SPI_DEV=$(IMU_MPU_SPI_DEV_LOWER)
IMU_CFLAGS += -DUSE_$(IMU_MPU_SPI_DEV_UPPER)
IMU_CFLAGS += -DIMU_MPU_SPI_SLAVE_IDX=$(IMU_MPU_SPI_SLAVE_IDX)
IMU_CFLAGS += -DUSE_$(IMU_MPU_SPI_SLAVE_IDX)


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_CFLAGS)
$(TARGET).srcs += $(IMU_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
