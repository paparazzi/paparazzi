ap.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_FLOAT
ap.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_float.h\"
ap.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_quat_float.h\"
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_float.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_float.c
