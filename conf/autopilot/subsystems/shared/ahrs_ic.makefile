#
# AHRS_PROPAGATE_FREQUENCY
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

AHRS_CFLAGS  = -DUSE_AHRS
ifdef AHRS_ALIGNER_LED
AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl.h\"
AHRS_SRCS   += subsystems/ahrs.c 
AHRS_SRCS   += subsystems/ahrs/ahrs_int_cmpl.c 
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c 

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)
