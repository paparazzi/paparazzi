# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

USE_MAGNETOMETER ?= 1
AHRS_ALIGNER_LED ?= none

AHRS_FINV_CFLAGS  = -DUSE_AHRS
AHRS_FINV_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_FINV_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_FINV_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS), fcq float_cmpl_quat))
# this is the secondary AHRS
AHRS_FINV_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_float_invariant_wrapper.h\"
AHRS_FINV_CFLAGS += -DSECONDARY_AHRS=ahrs_float_invariant
else
# this is the primary AHRS
AHRS_FINV_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_invariant_wrapper.h\"
AHRS_FINV_CFLAGS += -DPRIMARY_AHRS=ahrs_float_invariant
endif
else
# plain old single AHRS usage
AHRS_FINV_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_invariant_wrapper.h\"
endif

AHRS_FINV_SRCS   += subsystems/ahrs.c
AHRS_FINV_SRCS   += subsystems/ahrs/ahrs_float_invariant.c
AHRS_FINV_SRCS   += subsystems/ahrs/ahrs_float_invariant_wrapper.c
AHRS_FINV_SRCS   += subsystems/ahrs/ahrs_aligner.c

# add it for all targets except sim and fbw
ifeq (,$(findstring $(TARGET),sim fbw))
$(TARGET).CFLAGS += $(AHRS_FINV_CFLAGS)
$(TARGET).srcs += $(AHRS_FINV_SRCS)
endif
