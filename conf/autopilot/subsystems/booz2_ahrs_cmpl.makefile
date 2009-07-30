#
# Complementary filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=3
ap.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_aligner.c
ap.srcs += $(SRC_BOOZ)/ahrs/booz2_filter_attitude_cmpl_euler.c

sim.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=3
sim.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_aligner.c
sim.srcs += $(SRC_BOOZ)/ahrs/booz2_filter_attitude_cmpl_euler.c
