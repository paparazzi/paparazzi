# Rotorcraft AHRS module for AggieNav
# 2013, Utah State University, http://aggieair.usu.edu/
AHRS_ALIGNER_LED ?= none

AHRS_CFLAGS  = -DUSE_AHRS
AHRS_CFLAGS += -DAHRS_FLOAT

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_aggienav.h\"
AHRS_SRCS   += $(SRC_SUBSYSTEMS)/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aggienav.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)
