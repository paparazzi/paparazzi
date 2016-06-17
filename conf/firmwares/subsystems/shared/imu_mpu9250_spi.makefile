# Hey Emacs, this is a -*- makefile -*-
#
# MPU9250 IMU via SPI
# Should  @ conf/firmwares/subsystems/shared/imu_mpu9250_spi.makefile
#

include $(CFG_SHARED)/spi_master.makefile

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_MPU9250_CFLAGS  = -DUSE_IMU
endif

IMU_MPU9250_CFLAGS += -DIMU_TYPE_H=\"imu/imu_mpu9250_spi.h\"
IMU_MPU9250_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_MPU9250_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_mpu9250_spi.c
IMU_MPU9250_SRCS   += peripherals/mpu9250.c
IMU_MPU9250_SRCS   += peripherals/mpu9250_spi.c


# set default SPI device and slave index
ifeq ($(ARCH), lpc21)
IMU_MPU9250_SPI_DEV ?= spi1
IMU_MPU9250_SPI_SLAVE_IDX ?= SPI_SLAVE0
else ifeq ($(ARCH), stm32)
IMU_MPU9250_SPI_DEV ?= spi2
IMU_MPU9250_SPI_SLAVE_IDX ?= SPI_SLAVE2
endif


ifeq ($(TARGET), ap)
ifndef IMU_MPU9250_SPI_DEV
$(error Error: IMU_MPU9250_SPI_DEV not configured!)
endif
ifndef IMU_MPU9250_SPI_SLAVE_IDX
$(error Error: IMU_MPU9250_SPI_SLAVE_IDX not configured!)
endif
endif

# convert spix to upper/lower case
IMU_MPU9250_SPI_DEV_UPPER=$(shell echo $(IMU_MPU9250_SPI_DEV) | tr a-z A-Z)
IMU_MPU9250_SPI_DEV_LOWER=$(shell echo $(IMU_MPU9250_SPI_DEV) | tr A-Z a-z)

IMU_MPU9250_CFLAGS += -DIMU_MPU9250_SPI_DEV=$(IMU_MPU9250_SPI_DEV_LOWER)
IMU_MPU9250_CFLAGS += -DUSE_$(IMU_MPU9250_SPI_DEV_UPPER)
IMU_MPU9250_CFLAGS += -DIMU_MPU9250_SPI_SLAVE_IDX=$(IMU_MPU9250_SPI_SLAVE_IDX)
IMU_MPU9250_CFLAGS += -DUSE_$(IMU_MPU9250_SPI_SLAVE_IDX)

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_MPU9250_CFLAGS)
$(TARGET).srcs += $(IMU_MPU9250_SRCS)
endif


#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
