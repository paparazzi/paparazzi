#
# AHRS wrapper for AHRS devices, such as GX3 or UM6
# 2013, Utah State University, http://aggieair.usu.edu/

AHRS_CFLAGS  = -DUSE_AHRS

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_extern_quat.h\"
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c
AHRS_SRCS   += subsystems/ahrs/ahrs_extern_quat.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)
