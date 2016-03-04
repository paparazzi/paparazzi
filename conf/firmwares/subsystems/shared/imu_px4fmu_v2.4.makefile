# Hey Emacs, this is a -*- makefile -*-
#
# PX4 Pixhawk IMUconsists of two (internal) IMU's and one internal magneto. Also an optional external magneto
#
#MPU6000 +  L3GD20 +  LSM303D + HMC5883

include $(CFG_SHARED)/spi_master.makefile

IMU_PX4FMU_CFLAGS += -DIMU_TYPE_H=\"imu/imu_px4fmu_v2.4.h\"
IMU_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_px4fmu_v2.4.h\"
IMU_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_px4fmu_v2.4.c

# MPU
IMU_SRCS   += peripherals/mpu60x0.c
IMU_SRCS   += peripherals/mpu60x0_spi.c

#L3GD20 gyro
IMU_SRCS   += peripherals/l3gd20_spi.c

#LSM303D accelero + magneto
IMU_SRCS   += peripherals/lsm303dlhc_spi.c

# Magnetometer
IMU_SRCS   += peripherals/hmc58xx.c

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_CFLAGS  += -DUSE_IMU
endif

# set default SPI device
IMU_SPI_DEV ?= spi1
# convert spix to upper/lower case
IMU_SPI_DEV_UPPER=$(shell echo $(IMU_SPI_DEV) | tr a-z A-Z)
IMU_SPI_DEV_LOWER=$(shell echo $(IMU_SPI_DEV) | tr A-Z a-z)
IMU_CFLAGS += -DIMU_SPI_DEV=$(IMU_SPI_DEV_LOWER)
IMU_CFLAGS += -DUSE_$(IMU_SPI_DEV_UPPER)

#********** MPU6000 ***********
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE2
IMU_CFLAGS += -DIMU_MPU_SPI_SLAVE_IDX=$(IMU_MPU_SPI_SLAVE_IDX)
IMU_CFLAGS += -DUSE_$(IMU_MPU_SPI_SLAVE_IDX)

#********** L3GD20 ***********
IMU_L3G_SPI_SLAVE_IDX ?= SPI_SLAVE0
IMU_CFLAGS += -DIMU_L3G_SPI_SLAVE_IDX=$(IMU_L3G_SPI_SLAVE_IDX)
IMU_CFLAGS += -DUSE_$(IMU_L3G_SPI_SLAVE_IDX)

#********** LSM303dlhc ***********
IMU_LSM_SPI_SLAVE_IDX ?= SPI_SLAVE1
IMU_CFLAGS += -DIMU_LSM_SPI_SLAVE_IDX=$(IMU_LSM_SPI_SLAVE_IDX)
IMU_CFLAGS += -DUSE_$(IMU_LSM_SPI_SLAVE_IDX)

#********** HMC5883 ***********
IMU_HMC_I2C_DEV ?= i2c1

# convert i2cx to upper/lower case
IMU_HMC_I2C_DEV_UPPER=$(shell echo $(IMU_HMC_I2C_DEV) | tr a-z A-Z)
IMU_HMC_I2C_DEV_LOWER=$(shell echo $(IMU_HMC_I2C_DEV) | tr A-Z a-z)
IMU_CFLAGS += -DIMU_HMC_I2C_DEV=$(IMU_HMC_I2C_DEV_LOWER)
IMU_CFLAGS += -DUSE_$(IMU_HMC_I2C_DEV_UPPER)

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_CFLAGS)
$(TARGET).srcs += $(IMU_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
