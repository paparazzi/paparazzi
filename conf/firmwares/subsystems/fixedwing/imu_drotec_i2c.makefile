# Hey Emacs, this is a -*- makefile -*-

IMU_DROTEC_CFLAGS  = -DUSE_IMU
IMU_DROTEC_CFLAGS += -DIMU_TYPE_H=\"modules/sensors/imu_drotec.h\"

IMU_DROTEC_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_MODULES)/sensors/imu_drotec.c


IMU_DROTEC_CFLAGS += -DUSE_I2C
ifeq ($(ARCH), stm32)
	IMU_DROTEC_CFLAGS += -DUSE_I2C2
	IMU_DROTEC_CFLAGS += -DDROTECIMU_I2C_DEVICE=i2c2
else ifeq ($(ARCH), lpc21)
	IMU_DROTEC_CFLAGS += -DUSE_I2C0
	IMU_DROTEC_CFLAGS += -DDROTECIMU_I2C_DEVICE=i2c0
endif

ap.CFLAGS += $(IMU_DROTEC_CFLAGS)
ap.srcs   += $(IMU_DROTEC_SRCS)
