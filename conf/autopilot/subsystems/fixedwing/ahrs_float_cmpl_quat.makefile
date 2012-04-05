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

AHRS_CFLAGS  = -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER -DAHRS_GRAVITY_UPDATE_COORDINATED_TURN

ifneq ($(USE_MAGNETOMETER),0)
  AHRS_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl.h\"
AHRS_CFLAGS += -DAHRS_PROPAGATE_QUAT
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_float_cmpl.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)



# Extra stuff for fixedwings

ifdef CPU_LED
  ap.CFLAGS += -DAHRS_CPU_LED=$(CPU_LED)
endif

ifdef AHRS_PROPAGATE_FREQUENCY
else
  AHRS_PROPAGATE_FREQUENCY = 60
endif

ifdef AHRS_CORRECT_FREQUENCY
else
  AHRS_CORRECT_FREQUENCY = 60
endif

ap.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
ap.CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)


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
