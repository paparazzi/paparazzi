# Hey Emacs, this is a -*- makefile -*-

# attitude and speed estimation via invariant filter

USE_MAGNETOMETER ?= 1

INS_CFLAGS += -DUSE_AHRS_ALIGNER
INS_CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_float_invariant_wrapper.h\"

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  INS_CFLAGS += -DUSE_MAGNETOMETER
endif

INS_SRCS += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins/ins_float_invariant.c
INS_SRCS += $(SRC_SUBSYSTEMS)/ins/ins_float_invariant_wrapper.c

ifneq ($(AHRS_ALIGNER_LED),none)
  INS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ap.CFLAGS += $(INS_CFLAGS)
ap.srcs += $(INS_SRCS)

#
# NPS uses the real algorithm
#
nps.CFLAGS += $(INS_CFLAGS)
nps.srcs += $(INS_SRCS)
