# Rotorcraft IMU module for GX3
# ChibiOS DMA driver
# 2013, Utah State University, http://aggieair.usu.edu/

GX3_DMA_PORT ?= UARTD2
GX3_DMA_BAUD ?= B921600

IMU_GX3_CFLAGS += -DUSE_IMU
IMU_GX3_CFLAGS += -DUSE_IMU_FLOAT
IMU_GX3_CFLAGS += -DUSE_GX3

IMU_GX3_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_gx3.h\"
IMU_GX3_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
IMU_GX3_SRCS   += subsystems/imu/imu_gx3_dma.c

IMU_GX3_CFLAGS += -DGX3_DMA_BAUD=$(GX3_DMA_BAUD)
IMU_GX3_CFLAGS += -DGX3_DMA_PORT=$(GX3_DMA_PORT)

ap.CFLAGS += $(IMU_GX3_CFLAGS)
ap.srcs += $(IMU_GX3_SRCS)
