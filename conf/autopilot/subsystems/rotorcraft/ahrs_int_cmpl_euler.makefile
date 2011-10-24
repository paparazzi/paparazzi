# Hey Emacs, this is a -*- makefile -*-
#
# Fixed point complementary filter using euler angles for attitude estimation
#

AHRS_CFLAGS  = -DUSE_AHRS -DUSE_AHRS_CMPL
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER
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
