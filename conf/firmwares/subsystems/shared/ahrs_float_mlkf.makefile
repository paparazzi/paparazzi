# Hey Emacs, this is a -*- makefile -*-
#
#

USE_MAGNETOMETER ?= 1
AHRS_ALIGNER_LED ?= none

AHRS_MLKF_CFLAGS  = -DUSE_AHRS
AHRS_MLKF_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_MLKF_CFLAGS += -DUSE_MAGNETOMETER
else
$(error ahrs_float_mlkf needs a magnetometer)
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_MLKF_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS), mlkf))
# this is the secondary AHRS
AHRS_MLKF_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_float_mlkf_wrapper.h\"
AHRS_MLKF_CFLAGS += -DSECONDARY_AHRS=ahrs_mlkf
else
# this is the primary AHRS
AHRS_MLKF_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_mlkf_wrapper.h\"
AHRS_MLKF_CFLAGS += -DPRIMARY_AHRS=ahrs_mlkf
endif
else
# plain old single AHRS usage
AHRS_MLKF_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_mlkf_wrapper.h\"
endif

AHRS_MLKF_SRCS   += subsystems/ahrs.c
AHRS_MLKF_SRCS   += subsystems/ahrs/ahrs_float_mlkf.c
AHRS_MLKF_SRCS   += subsystems/ahrs/ahrs_float_mlkf_wrapper.c
AHRS_MLKF_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_MLKF_CFLAGS)
ap.srcs += $(AHRS_MLKF_SRCS)

nps.CFLAGS += $(AHRS_MLKF_CFLAGS)
nps.srcs += $(AHRS_MLKF_SRCS)

test_ahrs.CFLAGS += $(AHRS_MLKF_CFLAGS)
test_ahrs.srcs += $(AHRS_MLKF_SRCS)
