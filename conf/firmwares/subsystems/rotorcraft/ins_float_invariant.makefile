# Hey Emacs, this is a -*- makefile -*-

# attitude and speed estimation for fixedwings via invariant filter

USE_MAGNETOMETER ?= 1

INS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ins/ins_float_invariant.h\"
INS_CFLAGS += -DUSE_AHRS_ALIGNER
INS_CFLAGS += -DUSE_AHRS
# for geo mag
INS_CFLAGS += -DAHRS_FLOAT

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  INS_CFLAGS += -DUSE_MAGNETOMETER
endif

INS_SRCS += $(SRC_SUBSYSTEMS)/ahrs.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins/ins_float_invariant.c


ifneq ($(AHRS_ALIGNER_LED),none)
  INS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef AHRS_PROPAGATE_FREQUENCY
else
  AHRS_PROPAGATE_FREQUENCY = 125
endif

ifdef AHRS_CORRECT_FREQUENCY
else
  AHRS_CORRECT_FREQUENCY = 125
endif

INS_CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
INS_CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)

ap.CFLAGS += $(INS_CFLAGS)
ap.srcs += $(INS_SRCS)

#
# NPS uses the real algorithm
#
nps.CFLAGS += $(INS_CFLAGS)
nps.srcs += $(INS_SRCS)


