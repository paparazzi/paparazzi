STAB_ATT_CFLAGS  = -DSTABILIZATION_ATTITUDE_TYPE_FLOAT
STAB_ATT_CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_H=\"stabilization/stabilization_attitude_euler_float.h\"
STAB_ATT_SRCS  = $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_float.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_float.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_rc_setpoint.c

ap.CFLAGS += $(STAB_ATT_CFLAGS)
ap.srcs += $(STAB_ATT_SRCS)

nps.CFLAGS += $(STAB_ATT_CFLAGS)
nps.srcs += $(STAB_ATT_SRCS)
