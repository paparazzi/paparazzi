# Hey Emacs, this is a -*- makefile -*-
#
# PX4 Pixhawk IMUconsists of two (internal) IMU's and one internal magneto. Also an optional external magneto
#
# L3GD20 +  LSM303D

include $(CFG_SHARED)/spi_master.makefile

IMU_PX4FMU_CFLAGS += -DIMU_TYPE_H=\"imu/imu_px4fmu_v2.4.h\"
IMU_PX4FMU_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_PX4FMU_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_px4fmu_v2.4.c

#L3GD20 gyro
IMU_PX4FMU_SRCS   += peripherals/l3gd20_spi.c

#LSM303D accelero + magneto
IMU_PX4FMU_SRCS   += peripherals/lsm303dlhc_spi.c

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_PX4FMU_CFLAGS  += -DUSE_IMU
endif

# set default SPI device
IMU_PX4FMU_SPI_DEV ?= spi1
# convert spix to upper/lower case
IMU_PX4FMU_SPI_DEV_UPPER=$(shell echo $(IMU_PX4FMU_SPI_DEV) | tr a-z A-Z)
IMU_PX4FMU_SPI_DEV_LOWER=$(shell echo $(IMU_PX4FMU_SPI_DEV) | tr A-Z a-z)
IMU_PX4FMU_CFLAGS += -DIMU_PX4FMU_SPI_DEV=$(IMU_PX4FMU_SPI_DEV_LOWER)
IMU_PX4FMU_CFLAGS += -DUSE_$(IMU_PX4FMU_SPI_DEV_UPPER)

#********** L3GD20 ***********
IMU_L3G_SPI_SLAVE_IDX ?= SPI_SLAVE0
IMU_PX4FMU_CFLAGS += -DIMU_L3G_SPI_SLAVE_IDX=$(IMU_L3G_SPI_SLAVE_IDX)
IMU_PX4FMU_CFLAGS += -DUSE_$(IMU_L3G_SPI_SLAVE_IDX)

#********** LSM303dlhc ***********
IMU_LSM_SPI_SLAVE_IDX ?= SPI_SLAVE1
IMU_PX4FMU_CFLAGS += -DIMU_LSM_SPI_SLAVE_IDX=$(IMU_LSM_SPI_SLAVE_IDX)
IMU_PX4FMU_CFLAGS += -DUSE_$(IMU_LSM_SPI_SLAVE_IDX)

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_PX4FMU_CFLAGS)
$(TARGET).srcs += $(IMU_PX4FMU_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
