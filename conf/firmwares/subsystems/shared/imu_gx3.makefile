# Rotorcraft IMU module for GX3
# 2013, Utah State University, http://aggieair.usu.edu/

GX3_PORT ?= UART3
GX3_BAUD ?= B921600

IMU_GX3_CFLAGS  = -DUSE_AHRS
IMU_GX3_CFLAGS += -DUSE_IMU
IMU_GX3_CFLAGS += -DUSE_IMU_FLOAT
IMU_GX3_CFLAGS += -DUSE_GX3

IMU_GX3_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_gx3.h\"
IMU_GX3_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
IMU_GX3_SRCS   += subsystems/imu/imu_gx3.c

GX3_PORT_LOWER=$(shell echo $(GX3_PORT) | tr A-Z a-z)
IMU_GX3_CFLAGS += -DUSE_$(GX3_PORT) -D$(GX3_PORT)_BAUD=$(GX3_BAUD)
IMU_GX3_CFLAGS += -DUSE_GX3 -DGX3_PORT=$(GX3_PORT_LOWER)

ap.CFLAGS += $(IMU_GX3_CFLAGS)
ap.srcs += $(IMU_GX3_SRCS)
