# Hey Emacs, this is a -*- makefile -*-

IMU_ASPIRIN2_CFLAGS  = -DUSE_IMU
IMU_ASPIRIN2_CFLAGS += -DIMU_TYPE_H=\"modules/sensors/imu_aspirin2.h\"

IMU_ASPIRIN2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_MODULES)/sensors/imu_aspirin2.c


IMU_ASPIRIN2_CFLAGS += -DUSE_I2C
ifeq ($(ARCH), stm32)
	IMU_ASPIRIN2_CFLAGS += -DUSE_I2C2
	IMU_ASPIRIN2_CFLAGS += -DPPZUAVIMU_I2C_DEVICE=i2c2
else ifeq ($(ARCH), lpc21)
	IMU_ASPIRIN2_CFLAGS += -DUSE_I2C0
	IMU_ASPIRIN2_CFLAGS += -DPPZUAVIMU_I2C_DEVICE=i2c0
endif

ap.CFLAGS += $(IMU_ASPIRIN2_CFLAGS)
ap.srcs   += $(IMU_ASPIRIN2_SRCS)

ap.CFLAGS += -DAHRS_MAG_UPDATE_YAW_ONLY

