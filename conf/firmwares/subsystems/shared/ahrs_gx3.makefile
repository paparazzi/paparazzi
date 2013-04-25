# AHRS module for GX3
# 2013, Utah State University, http://aggieair.usu.edu/

GX3_PORT ?= UART3
GX3_BAUD ?= B921600

AHRS_CFLAGS  = -DUSE_AHRS
AHRS_CFLAGS += -DUSE_IMU
AHRS_CFLAGS += -DUSE_IMU_FLOAT

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_gx3.h\"
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs.c
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
AHRS_SRCS   += subsystems/ahrs/ahrs_gx3.c

AHRS_CFLAGS += -DUSE_$(GX3_PORT) -D$(GX3_PORT)_BAUD=$(GX3_BAUD)
AHRS_CFLAGS += -DUSE_GX3 -DGX3_LINK=$(GX3_PORT)

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)
