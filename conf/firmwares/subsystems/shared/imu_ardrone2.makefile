# imu AR.Drone2

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_ardrone2.h\" -DUSE_IMU
imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_ardrone2.c

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
