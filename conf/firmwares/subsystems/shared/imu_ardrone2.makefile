# imu AR.Drone2

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_ardrone2.h\"
imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
#imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_ardrone2.c
imu_srcs += $(SRC_ARCH)/subsystems/imu/imu_ardrone2_arch.c

#imu_CFLAGS += -DUSE_AMI601
#imu_srcs   += peripherals/ami601.c
#imu_CFLAGS += -DUSE_I2C1


# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
