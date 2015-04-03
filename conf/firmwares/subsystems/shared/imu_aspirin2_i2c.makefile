# Hey Emacs, this is a -*- makefile -*-

IMU_ASPIRIN2_CFLAGS  = -DUSE_IMU
IMU_ASPIRIN2_CFLAGS += -DIMU_TYPE_H=\"modules/sensors/imu_aspirin2.h\"

IMU_ASPIRIN2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_MODULES)/sensors/imu_aspirin2.c


# set default i2c bus
ifeq ($(ARCH), lpc21)
IMU_ASPIRIN2_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN2_I2C_DEV ?= i2c2
endif

ifeq ($(TARGET), ap)
ifndef IMU_ASPIRIN2_I2C_DEV
$(error Error: IMU_ASPIRIN2_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
IMU_ASPIRIN2_I2C_DEV_UPPER=$(shell echo $(IMU_ASPIRIN2_I2C_DEV) | tr a-z A-Z)
IMU_ASPIRIN2_I2C_DEV_LOWER=$(shell echo $(IMU_ASPIRIN2_I2C_DEV) | tr A-Z a-z)

IMU_ASPIRIN2_CFLAGS += -DIMU_ASPIRIN2_I2C_DEV=$(IMU_ASPIRIN2_I2C_DEV_LOWER)
IMU_ASPIRIN2_CFLAGS += -DUSE_$(IMU_ASPIRIN2_I2C_DEV_UPPER)


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_ASPIRIN2_CFLAGS)
$(TARGET).srcs += $(IMU_ASPIRIN2_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
