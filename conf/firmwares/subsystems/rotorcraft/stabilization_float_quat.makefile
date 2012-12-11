ap.CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_FLOAT
ap.CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_QUAT
ap.CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_H=\"stabilization/stabilization_attitude_float.h\"
ap.CFLAGS += -DSTABILIZATION_ATTITUDE_REF_TYPE_H=\"stabilization/stabilization_attitude_ref_quat_float.h\"
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_float.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_float.c
