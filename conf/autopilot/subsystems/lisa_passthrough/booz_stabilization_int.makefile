stm_passthrough.srcs += $(SRC_FIRMWARE)/autopilot.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/actuators/supervision.c

stm_passthrough.srcs += $(SRC_FIRMWARE)/stabilization.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c

stm_passthrough.srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c

stm_passthrough.CFLAGS += -DUSE_NAVIGATION
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ins.c
stm_passthrough.srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/navigation.c
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c
stm_passthrough.CFLAGS += -DUSE_VFF -DDT_VFILTER='(1./512.)'

stm_passthrough.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
stm_passthrough.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_int.h\"
stm_passthrough.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_int.h\"
stm_passthrough.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_int.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_int.c
