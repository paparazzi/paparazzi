# Fixedwing AHRS module for GX3
# 2013, Utah State University, http://aggieair.usu.edu/

GX3_PORT ?= UART3
GX3_BAUD ?= B921600

AHRS_CFLAGS  = -DUSE_AHRS
AHRS_CFLAGS += -DUSE_IMU

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_gx3.h\"
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs.c
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
AHRS_SRCS   += subsystems/ahrs/ahrs_gx3.c

GX3_PORT_LOWER=$(shell echo $(GX3_PORT) | tr A-Z a-z)
AHRS_CFLAGS += -DUSE_$(GX3_PORT) -D$(GX3_PORT)_BAUD=$(GX3_BAUD)
AHRS_CFLAGS += -DGX3_PORT=$(GX3_PORT_LOWER)

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)
