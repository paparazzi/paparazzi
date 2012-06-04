#
# Error State Space Kalman filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_LKF -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_lkf.c

nps.CFLAGS += -DUSE_AHRS_LKF -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
nps.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
nps.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
nps.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_lkf.c
