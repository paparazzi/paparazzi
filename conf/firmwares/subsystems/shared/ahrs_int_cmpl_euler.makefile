# Hey Emacs, this is a -*- makefile -*-
#
# Fixed point complementary filter using euler angles for attitude estimation
#

USE_MAGNETOMETER ?= 1
AHRS_ALIGNER_LED ?= none

AHRS_ICE_CFLAGS  = -DUSE_AHRS
AHRS_ICE_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_ICE_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_ICE_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS), ice int_cmpl_euler))
# this is the secondary AHRS
AHRS_ICE_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler_wrapper.h\"
AHRS_ICE_CFLAGS += -DSECONDARY_AHRS=ahrs_ice
else
# this is the primary AHRS
AHRS_ICE_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler_wrapper.h\"
AHRS_ICE_CFLAGS += -DPRIMARY_AHRS=ahrs_ice
endif
else
# plain old single AHRS usage
AHRS_ICE_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler_wrapper.h\"
endif

AHRS_ICE_SRCS   += subsystems/ahrs.c
AHRS_ICE_SRCS   += subsystems/ahrs/ahrs_int_cmpl_euler.c
AHRS_ICE_SRCS   += subsystems/ahrs/ahrs_int_cmpl_euler_wrapper.c
AHRS_ICE_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_ICE_CFLAGS)
ap.srcs += $(AHRS_ICE_SRCS)

nps.CFLAGS += $(AHRS_ICE_CFLAGS)
nps.srcs += $(AHRS_ICE_SRCS)

#
# Simple simulation of the AHRS result
#
ahrssim_CFLAGS  = -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
ahrssim_CFLAGS += -DUSE_AHRS

ahrssim_srcs    = $(SRC_SUBSYSTEMS)/ahrs.c
ahrssim_srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.CFLAGS += $(ahrssim_CFLAGS)
sim.srcs += $(ahrssim_srcs)

test_ahrs.CFLAGS += $(AHRS_ICE_CFLAGS)
test_ahrs.srcs += $(AHRS_ICE_SRCS)
