# Hey Emacs, this is a -*- makefile -*-

# attitude estimation for fixedwings via dcm algorithm

USE_MAGNETOMETER ?= 0
AHRS_ALIGNER_LED ?= none

AHRS_DCM_CFLAGS  = -DUSE_AHRS
AHRS_DCM_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_DCM_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_DCM_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS), dcm float_dcm))
# this is the secondary AHRS
AHRS_DCM_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm_wrapper.h\"
AHRS_DCM_CFLAGS += -DSECONDARY_AHRS=ahrs_dcm
else
# this is the primary AHRS
AHRS_DCM_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm_wrapper.h\"
AHRS_DCM_CFLAGS += -DPRIMARY_AHRS=ahrs_dcm
endif
else
# plain old single AHRS usage
AHRS_DCM_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm_wrapper.h\"
endif

AHRS_DCM_SRCS   += $(SRC_SUBSYSTEMS)/ahrs.c
AHRS_DCM_SRCS   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
AHRS_DCM_SRCS   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm.c
AHRS_DCM_SRCS   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm_wrapper.c

# add it for all targets except sim and fbw
ifeq (,$(findstring $(TARGET),sim fbw))
$(TARGET).CFLAGS += $(AHRS_DCM_CFLAGS)
$(TARGET).srcs += $(AHRS_DCM_SRCS)
endif


#
# Simple simulation of the AHRS result
#
ahrssim_CFLAGS  = -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
ahrssim_CFLAGS += -DUSE_AHRS

ahrssim_srcs    = $(SRC_SUBSYSTEMS)/ahrs.c
ahrssim_srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.CFLAGS += $(ahrssim_CFLAGS)
sim.srcs += $(ahrssim_srcs)

