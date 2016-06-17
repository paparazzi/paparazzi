
IMU_NAVGO_CFLAGS  = -DUSE_IMU
IMU_NAVGO_CFLAGS += -DIMU_TYPE_H=\"boards/navgo/imu_navgo.h\"

IMU_NAVGO_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                    $(SRC_BOARD)/imu_navgo.c

IMU_NAVGO_I2C_DEV=i2c1
IMU_NAVGO_CFLAGS += -DUSE_I2C
IMU_NAVGO_CFLAGS += -DUSE_I2C1 -DI2C1_SCLL=25 -DI2C1_SCLH=25

IMU_NAVGO_CFLAGS += -DIMU_NAVGO_I2C_DEV=$(IMU_NAVGO_I2C_DEV)
IMU_NAVGO_SRCS += peripherals/itg3200.c
IMU_NAVGO_SRCS += peripherals/adxl345_i2c.c
IMU_NAVGO_SRCS += peripherals/hmc58xx.c

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_NAVGO_CFLAGS)
$(TARGET).srcs += $(IMU_NAVGO_SRCS)
endif

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
