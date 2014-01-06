# Hey Emacs, this is a -*- makefile -*-
#
# Fixed point complementary filter using euler angles for attitude estimation
#

AHRS_CFLAGS  = -DUSE_AHRS -DUSE_AHRS_ARDRONE2

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_ardrone2.h\"
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_ardrone2.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)

nps.CFLAGS += $(AHRS_CFLAGS)
nps.srcs += $(AHRS_SRCS)
