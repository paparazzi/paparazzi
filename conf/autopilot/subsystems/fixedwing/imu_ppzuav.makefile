# Hey Emacs, this is a -*- makefile -*-

IMU_PPZUAV_CFLAGS  = -DUSE_IMU
IMU_PPZUAV_CFLAGS += -DIMU_TYPE_H=\"modules/sensors/imu_ppzuav.h\"

IMU_PPZUAV_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_MODULES)/sensors/imu_ppzuav.c


IMU_PPZUAV_CFLAGS += -DUSE_I2C
ifeq ($(ARCH), stm32)
	IMU_PPZUAV_CFLAGS += -DUSE_I2C2
	IMU_PPZUAV_CFLAGS += -DPPZUAVIMU_I2C_DEVICE=i2c2
else ifeq ($(ARCH), lpc21)
	IMU_PPZUAV_CFLAGS += -DUSE_I2C0
	IMU_PPZUAV_CFLAGS += -DPPZUAVIMU_I2C_DEVICE=i2c0
endif

ap.CFLAGS += $(IMU_PPZUAV_CFLAGS)
ap.srcs   += $(IMU_PPZUAV_SRCS)

ap.CFLAGS += -DAHRS_MAG_UPDATE_YAW_ONLY

