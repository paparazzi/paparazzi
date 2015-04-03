# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

USE_MAGNETOMETER ?= 1
AHRS_ALIGNER_LED ?= none

AHRS_ICQ_CFLAGS  = -DUSE_AHRS
AHRS_ICQ_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_ICQ_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_ICQ_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS),ahrs_icq int_cmpl_quat))
# this is the secondary AHRS
AHRS_ICQ_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.h\"
AHRS_ICQ_CFLAGS += -DSECONDARY_AHRS=ahrs_icq
else
# this is the primary AHRS
AHRS_ICQ_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.h\"
AHRS_ICQ_CFLAGS += -DPRIMARY_AHRS=ahrs_icq
endif
else
# plain old single AHRS usage
AHRS_ICQ_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.h\"
endif

AHRS_ICQ_SRCS   += subsystems/ahrs.c
AHRS_ICQ_SRCS   += subsystems/ahrs/ahrs_int_cmpl_quat.c
AHRS_ICQ_SRCS   += subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.c
AHRS_ICQ_SRCS   += subsystems/ahrs/ahrs_aligner.c

# add it for all targets except sim and fbw
ifeq (,$(findstring $(TARGET),sim fbw))
$(TARGET).CFLAGS += $(AHRS_ICQ_CFLAGS)
$(TARGET).srcs += $(AHRS_ICQ_SRCS)
endif
