# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_PROPAGATE_FREQUENCY
#

AHRS_CFLAGS  = -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR -DUSE_AHRS_CMPL
ifdef AHRS_ALIGNER_LED
AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler.h\"
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_int_cmpl_euler.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)

sim.CFLAGS += $(AHRS_CFLAGS)
sim.srcs += $(AHRS_SRCS)


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

