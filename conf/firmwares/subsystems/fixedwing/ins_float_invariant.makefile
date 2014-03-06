# Hey Emacs, this is a -*- makefile -*-

# attitude and speed estimation for fixedwings via invariant filter

INS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ins/ins_float_invariant.h\"
INS_CFLAGS += -DUSE_AHRS_ALIGNER
INS_CFLAGS += -DUSE_AHRS
INS_CFLAGS += -DINS_UPDATE_FW_ESTIMATOR

INS_SRCS += $(SRC_SUBSYSTEMS)/ahrs.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins/ins_float_invariant.c


ifneq ($(AHRS_ALIGNER_LED),none)
  INS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_PROPAGATE_FREQUENCY ?= 125
AHRS_CORRECT_FREQUENCY ?= 125

INS_CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
INS_CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)

ap.CFLAGS += $(INS_CFLAGS)
ap.srcs += $(INS_SRCS)

#
# NPS uses the real algorithm
#
nps.CFLAGS += $(INS_CFLAGS)
nps.srcs += $(INS_SRCS)


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

