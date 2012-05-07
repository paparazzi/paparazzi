# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_PROPAGATE_FREQUENCY
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

ifndef USE_MAGNETOMETER
USE_MAGNETOMETER = 1
endif

AHRS_CFLAGS  = -DUSE_AHRS
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER

ifneq ($(USE_MAGNETOMETER),0)
  AHRS_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_quat.h\"
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_int_cmpl_quat.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)

sim.CFLAGS += $(AHRS_CFLAGS)
sim.srcs += $(AHRS_SRCS)
