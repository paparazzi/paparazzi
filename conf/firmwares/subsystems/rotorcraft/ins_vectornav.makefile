# Vectornav INS Driver
# 2015, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/

include $(CFG_SHARED)/ins_vectornav.makefile



#
# NPS simulator
#
nps.CFLAGS += -DIMU_TYPE_H=\"imu/imu_nps.h\" -DUSE_IMU
nps.srcs   += $(SRC_SUBSYSTEMS)/imu.c $(SRC_SUBSYSTEMS)/imu/imu_nps.c

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c

nps.CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_gps_passthrough.h\"
nps.srcs   += $(SRC_SUBSYSTEMS)/ins.c
nps.srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough.c

nps.CFLAGS += -DNPS_BYPASS_AHRS -DNPS_BYPASS_INS

