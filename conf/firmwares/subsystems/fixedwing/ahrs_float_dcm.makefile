# Hey Emacs, this is a -*- makefile -*-

# attitude estimation for fixedwings via dcm algorithm

USE_MAGNETOMETER ?= 0
AHRS_ALIGNER_LED ?= none

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm_wrapper.h\"
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER
AHRS_CFLAGS += -DUSE_AHRS

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
AHRS_CFLAGS += -DUSE_MAGNETOMETER
endif

AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs.c
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm.c
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm_wrapper.c


ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
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

