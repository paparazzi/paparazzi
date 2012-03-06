ap.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
ap.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_int.h\"
ap.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_quat_int.h\"
ap.CFLAGS += -DUSE_SETPOINTS_WITH_TRANSITIONS
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_int.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_int.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/quat_setpoint_int.c
