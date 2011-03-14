#
# Fixed point complementary filter using euler angles for attitude estimation
#

ifdef AHRS_ALIGNER_LED
ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED) -DAHRS_FIXED_POINT
else
ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_FIXED_POINT
endif
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_int_cmpl_euler.c

sim.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=3 -DAHRS_FIXED_POINT
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_int_cmpl_euler.c
