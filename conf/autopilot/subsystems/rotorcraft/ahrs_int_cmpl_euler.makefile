# Hey Emacs, this is a -*- makefile -*-
#
# Fixed point complementary filter using euler angles for attitude estimation
#

ifndef USE_MAGNETOMETER
USE_MAGNETOMETER = 1
endif

AHRS_CFLAGS  = -DUSE_AHRS -DUSE_AHRS_CMPL
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER

ifneq ($(USE_MAGNETOMETER),0)
  AHRS_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler.h\"
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_int_cmpl_euler.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)

sim.CFLAGS += $(AHRS_CFLAGS)
sim.srcs += $(AHRS_SRCS)
