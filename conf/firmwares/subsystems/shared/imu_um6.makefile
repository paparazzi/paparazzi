# Attitude estimation for fixedwings and rotorcrafts via CHR-UM6
# 2012, Utah State University, http://aggieair.usu.edu/

ifndef UM6_PORT
  UM6_PORT=UART3
endif
ifndef UM6_BAUD
  UM6_BAUD=B115200
endif
UM6_PORT_LOWER=$(shell echo $(UM6_PORT) | tr A-Z a-z)

IMU_UM6_CFLAGS += -DUSE_IMU
IMU_UM6_CFLAGS += -DIMU_TYPE_H=\"imu/imu_um6.h\"
IMU_UM6_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
IMU_UM6_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_um6.c

IMU_UM6_CFLAGS += -DUSE_$(UM6_PORT) -D$(UM6_PORT)_BAUD=$(UM6_BAUD)
IMU_UM6_CFLAGS += -DUM6_LINK=$(UM6_PORT_LOWER)

ap.CFLAGS += $(IMU_UM6_CFLAGS)
ap.srcs   += $(IMU_UM6_SRCS)
