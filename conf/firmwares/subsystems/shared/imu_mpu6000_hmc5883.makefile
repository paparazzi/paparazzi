# Hey Emacs, this is a -*- makefile -*-
#
# IMU with MPU6000 via SPI and HMC5883 via I2c.
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

# set default i2c bus
ifeq ($(ARCH), lpc21)
IMU_HMC_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
IMU_HMC_I2C_DEV ?= i2c2
endif

ifeq ($(TARGET), ap)
ifndef IMU_HMC_I2C_DEV
$(error Error: IMU_HMC_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
IMU_HMC_I2C_DEV_UPPER=$(shell echo $(IMU_HMC_I2C_DEV) | tr a-z A-Z)
IMU_HMC_I2C_DEV_LOWER=$(shell echo $(IMU_HMC_I2C_DEV) | tr A-Z a-z)

IMU_CFLAGS += -DIMU_HMC_I2C_DEV=$(IMU_HMC_I2C_DEV_LOWER)
IMU_CFLAGS += -DUSE_$(IMU_HMC_I2C_DEV_UPPER)


# set default SPI device and slave index
ifeq ($(ARCH), lpc21)
IMU_MPU_SPI_DEV ?= spi1
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE0
else ifeq ($(ARCH), stm32)
IMU_MPU_SPI_DEV ?= spi2
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE2
endif

ifeq ($(TARGET), ap)
ifndef IMU_MPU_SPI_DEV
$(error Error: IMU_MPU_SPI_DEV not configured!)
endif
ifndef IMU_MPU_SPI_SLAVE_IDX
$(error Error: IMU_MPU_SPI_SLAVE_IDX not configured!)
endif
endif

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
