ap.CFLAGS += -DUSE_AHRS_LKF -DAHRS_ALIGNER_LED=3
ap.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_aligner.c
ap.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_float_lkf.c