# AHRS Transparent
# 2014, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/
AHRS_ALIGNER_LED ?= none

AHRS_TRANSPARENT_CFLAGS  = -DUSE_AHRS
AHRS_TRANSPARENT_CFLAGS += -DAHRS_FLOAT
AHRS_TRANSPARENT_CFLAGS += -DUSE_AHRS_ALIGNER

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_TRANSPARENT_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS), transparent))
# this is the secondary AHRS
AHRS_TRANSPARENT_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ahrs/ahrs_transparent_wrapper.h\"
AHRS_TRANSPARENT_CFLAGS += -DSECONDARY_AHRS=ahrs_transparent
else
# this is the primary AHRS
AHRS_TRANSPARENT_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_transparent_wrapper.h\"
AHRS_TRANSPARENT_CFLAGS += -DPRIMARY_AHRS=ahrs_transparent
endif
else
# plain old single AHRS usage
AHRS_TRANSPARENT_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_transparent_wrapper.h\"
endif

AHRS_TRANSPARENT_SRCS   += subsystems/ahrs.c
AHRS_TRANSPARENT_SRCS   += subsystems/ahrs/ahrs_transparent.c
AHRS_TRANSPARENT_SRCS   += subsystems/ahrs/ahrs_transparent_wrapper.c
AHRS_TRANSPARENT_SRCS   += subsystems/ahrs/ahrs_aligner.c

# add it for all targets except sim and fbw
ifeq (,$(findstring $(TARGET),sim fbw))
$(TARGET).CFLAGS += $(AHRS_TRANSPARENT_CFLAGS)
$(TARGET).srcs += $(AHRS_TRANSPARENT_SRCS)
endif
