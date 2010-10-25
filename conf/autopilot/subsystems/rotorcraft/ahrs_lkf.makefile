#
# Error State Space Kalman filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_LKF -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_lkf.c

sim.CFLAGS += -DUSE_AHRS_LKF -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_lkf.c
