#
# Complementary filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED) -DAHRS_FIXED_POINT
ap.srcs += $(SRC_FIRMWARE)/ahrs.c
ap.srcs += $(SRC_FIRMWARE)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_FIRMWARE)/ahrs/ahrs_cmpl_euler.c

sim.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=3 -DAHRS_FIXED_POINT
sim.srcs += $(SRC_FIRMWARE)/ahrs.c
sim.srcs += $(SRC_FIRMWARE)/ahrs/ahrs_aligner.c
sim.srcs += $(SRC_FIRMWARE)/ahrs/ahrs_cmpl_euler.c
