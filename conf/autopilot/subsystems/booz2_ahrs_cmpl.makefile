#
# Complementary filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=3 -DBOOZ_AHRS_FIXED_POINT
ap.srcs += $(SRC_BOOZ)/booz_ahrs.c
ap.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_aligner.c
ap.srcs += $(SRC_BOOZ)/ahrs/booz2_filter_attitude_cmpl_euler.c

sim.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=3 -DBOOZ_AHRS_FIXED_POINT
sim.srcs += $(SRC_BOOZ)/booz_ahrs.c
sim.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_aligner.c
sim.srcs += $(SRC_BOOZ)/ahrs/booz2_filter_attitude_cmpl_euler.c
