
IMU_HBMINI_CFLAGS  = -DUSE_IMU
IMU_HBMINI_CFLAGS += -DIMU_TYPE_H=\"boards/hbmini/imu_hbmini.h\"

IMU_HBMINI_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                     $(SRC_BOARD)/imu_hbmini.c

IMU_HBMINI_I2C_DEV ?= i2c1

IMU_HBMINI_CFLAGS += -DUSE_I2C
IMU_HBMINI_CFLAGS += -DUSE_I2C1 -DI2C1_SCLL=25 -DI2C1_SCLH=25

IMU_HBMINI_CFLAGS += -DIMU_HBMINI_I2C_DEV=$(IMU_HBMINI_I2C_DEV)

IMU_HBMINI_SRCS += peripherals/hmc58xx.c


include $(CFG_SHARED)/spi_master.makefile

IMU_HBMINI_CFLAGS += -DUSE_SPI_SLAVE0
IMU_HBMINI_CFLAGS += -DUSE_SPI1
IMU_HBMINI_CFLAGS += -DMAX1168_EOC_VIC_SLOT=11

IMU_HBMINI_SRCS += peripherals/max1168.c
IMU_HBMINI_SRCS += $(SRC_ARCH)/peripherals/max1168_arch.c

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_HBMINI_CFLAGS)
$(TARGET).srcs += $(IMU_HBMINI_SRCS)
endif

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
