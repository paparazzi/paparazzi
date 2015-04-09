# Hey Emacs, this is a -*- makefile -*-

IMU_PPZUAV_CFLAGS  = -DUSE_IMU
IMU_PPZUAV_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_ppzuav.h\"

IMU_PPZUAV_SRCS  = $(SRC_SUBSYSTEMS)/imu.c
IMU_PPZUAV_SRCS += $(SRC_SUBSYSTEMS)/imu/imu_ppzuav.c
IMU_PPZUAV_SRCS += peripherals/adxl345_i2c.c
IMU_PPZUAV_SRCS += peripherals/itg3200.c
IMU_PPZUAV_SRCS += peripherals/hmc58xx.c

IMU_PPZUAV_CFLAGS += -DUSE_I2C
ifeq ($(ARCH), stm32)
	IMU_PPZUAV_CFLAGS += -DUSE_I2C2
	IMU_PPZUAV_CFLAGS += -DIMU_PPZUAV_I2C_DEV=i2c2
else ifeq ($(ARCH), lpc21)
	IMU_PPZUAV_CFLAGS += -DUSE_I2C0
	IMU_PPZUAV_CFLAGS += -DIMU_PPZUAV_I2C_DEV=i2c0
endif


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_PPZUAV_CFLAGS)
$(TARGET).srcs += $(IMU_PPZUAV_SRCS)
endif
