ap.CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_INT
ap.CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_H=\"stabilization/stabilization_attitude_int.h\"
ap.CFLAGS += -DSTABILIZATION_ATTITUDE_REF_TYPE_H=\"stabilization/stabilization_attitude_ref_euler_int.h\"
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_int.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_int.c
