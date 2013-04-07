# Hey Emacs, this is a -*- makefile -*-

# attitude estimation for fixedwings via dcm algorithm

USE_MAGNETOMETER ?= 0

ifeq ($(TARGET), ap)

ap.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm.h\"
ap.CFLAGS += -DUSE_AHRS_ALIGNER
ap.CFLAGS += -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR

ifneq ($(USE_MAGNETOMETER),0)
ap.CFLAGS += -DUSE_MAGNETOMETER
endif

ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm.c


ifneq ($(AHRS_ALIGNER_LED),none)
  ap.CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

endif


#
# Simple simulation of the AHRS result
#
ahrssim_CFLAGS  = -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
ahrssim_CFLAGS += -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR

ahrssim_srcs    = $(SRC_SUBSYSTEMS)/ahrs.c
ahrssim_srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.CFLAGS += $(ahrssim_CFLAGS)
sim.srcs += $(ahrssim_srcs)

jsbsim.CFLAGS += $(ahrssim_CFLAGS)
jsbsim.srcs += $(ahrssim_srcs)

