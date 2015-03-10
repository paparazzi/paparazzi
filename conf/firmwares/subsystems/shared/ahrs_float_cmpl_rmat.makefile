# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

USE_MAGNETOMETER ?= 1

AHRS_FC_CFLAGS  = -DUSE_AHRS
AHRS_FC_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_FC_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_FC_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS), fcr float_cmpl_rmat))
# this is the secondary AHRS
AHRS_FC_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_wrapper.h\"
AHRS_FC_CFLAGS += -DSECONDARY_AHRS=ahrs_fc
else
# this is the primary AHRS
AHRS_FC_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_wrapper.h\"
AHRS_FC_CFLAGS += -DPRIMARY_AHRS=ahrs_fc
endif
else
# plain old single AHRS usage
AHRS_FC_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_wrapper.h\"
endif

AHRS_FC_CFLAGS += -DAHRS_PROPAGATE_RMAT
AHRS_FC_SRCS   += subsystems/ahrs.c
AHRS_FC_SRCS   += subsystems/ahrs/ahrs_float_cmpl.c
AHRS_FC_SRCS   += subsystems/ahrs/ahrs_float_cmpl_wrapper.c
AHRS_FC_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_FC_CFLAGS)
ap.srcs += $(AHRS_FC_SRCS)

nps.CFLAGS += $(AHRS_FC_CFLAGS)
nps.srcs += $(AHRS_FC_SRCS)

test_ahrs.CFLAGS += $(AHRS_FC_CFLAGS)
test_ahrs.srcs += $(AHRS_FC_SRCS)
