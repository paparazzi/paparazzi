STAB_ATT_CFLAGS  = -DSTABILIZATION_ATTITUDE_NO_REF
STAB_ATT_CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_INT
STAB_ATT_CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_H=\"stabilization/stabilization_attitude_passthrough.h\"
STAB_ATT_SRCS  = $(SRC_FIRMWARE)/stabilization/stabilization_attitude_passthrough.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_rc_setpoint.c
ap.CFLAGS += $(STAB_ATT_CFLAGS)
ap.srcs += $(STAB_ATT_SRCS)

nps.CFLAGS += $(STAB_ATT_CFLAGS)
nps.srcs += $(STAB_ATT_SRCS)
