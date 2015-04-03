# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

# for fixedwings disable mag by default
USE_MAGNETOMETER ?= 0


include $(CFG_SHARED)/ahrs_float_cmpl_rmat.makefile

# add some fixedwing specific flags
ifeq (,$(findstring $(TARGET),sim fbw))
$(TARGET).CFLAGS += -DAHRS_GRAVITY_UPDATE_COORDINATED_TURN
ifneq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
$(TARGET).CFLAGS += -DAHRS_USE_GPS_HEADING
endif
endif

#
# Simple simulation of the AHRS result
#
ahrssim_CFLAGS  = -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
ahrssim_CFLAGS += -DUSE_AHRS

ahrssim_srcs    = $(SRC_SUBSYSTEMS)/ahrs.c
ahrssim_srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.CFLAGS += $(ahrssim_CFLAGS)
sim.srcs += $(ahrssim_srcs)
