# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

USE_MAGNETOMETER ?= 0
AHRS_ALIGNER_LED ?= none

AHRS_CFLAGS  = -DUSE_AHRS -DAHRS_FLOAT
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER -DAHRS_GRAVITY_UPDATE_COORDINATED_TURN

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_CFLAGS += -DUSE_MAGNETOMETER
else
  AHRS_CFLAGS += -DAHRS_USE_GPS_HEADING
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl.h\"
AHRS_CFLAGS += -DAHRS_PROPAGATE_QUAT
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_float_cmpl.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c

ifdef AHRS_PROPAGATE_FREQUENCY
AHRS_CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
endif

ifdef AHRS_CORRECT_FREQUENCY
AHRS_CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)
endif

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)

#
# NPS uses the real algorithm
#
nps.CFLAGS += $(AHRS_CFLAGS)
nps.srcs += $(AHRS_SRCS)

#
# Simple simulation of the AHRS result
#
ahrssim_CFLAGS  = -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
ahrssim_CFLAGS += -DUSE_AHRS

ahrssim_srcs    = $(SRC_SUBSYSTEMS)/ahrs.c
ahrssim_srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.CFLAGS += $(ahrssim_CFLAGS)
sim.srcs += $(ahrssim_srcs)

jsbsim.CFLAGS += $(ahrssim_CFLAGS)
jsbsim.srcs += $(ahrssim_srcs)
