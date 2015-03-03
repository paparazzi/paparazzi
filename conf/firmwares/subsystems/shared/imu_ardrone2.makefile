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

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
