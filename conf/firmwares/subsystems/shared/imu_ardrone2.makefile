# imu AR.Drone2

ifeq ($(BOARD_TYPE), sdk)
imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_ardrone2_sdk.h\" -DUSE_IMU
imu_srcs   += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs   += $(SRC_SUBSYSTEMS)/imu/imu_ardrone2_sdk.c
else ifeq ($(BOARD_TYPE), raw)
imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_ardrone2_raw.h\" -DUSE_IMU
imu_srcs   += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs   += $(SRC_SUBSYSTEMS)/imu/imu_ardrone2_raw.c
imu_srcs   += $(SRC_BOARD)/navdata.c
endif


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(imu_CFLAGS)
$(TARGET).srcs += $(imu_srcs)
endif


# Set the AHRS propegation frequencies
AHRS_PROPAGATE_FREQUENCY ?= 200
AHRS_CORRECT_FREQUENCY ?= 200
ap.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
ap.CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
