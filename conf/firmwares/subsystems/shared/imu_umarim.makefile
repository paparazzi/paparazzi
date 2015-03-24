
IMU_UMARIM_CFLAGS  = -DUSE_IMU
IMU_UMARIM_CFLAGS += -DIMU_TYPE_H=\"boards/umarim/imu_umarim.h\"

IMU_UMARIM_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                     $(SRC_BOARD)/imu_umarim.c

IMU_UMARIM_I2C_DEV=i2c1
IMU_UMARIM_CFLAGS += -DUSE_I2C
IMU_UMARIM_CFLAGS += -DUSE_I2C1 -DI2C1_SCLL=25 -DI2C1_SCLH=25

IMU_UMARIM_CFLAGS += -DIMU_UMARIM_I2C_DEV=$(IMU_UMARIM_I2C_DEV)
IMU_UMARIM_SRCS += peripherals/itg3200.c
IMU_UMARIM_SRCS += peripherals/adxl345_i2c.c

ap.CFLAGS += $(IMU_UMARIM_CFLAGS)
ap.srcs   += $(IMU_UMARIM_SRCS)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
