#
#
#

ap.CFLAGS += -DAHRS_ALIGNER_LED=3
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_mlkf.c
ap.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_opt.c

nps.CFLAGS += -I$(SRC_BOOZ_PRIV)
nps.CFLAGS += -DAHRS_ALIGNER_LED=3
nps.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
nps.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
nps.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_mlkf.c
nps.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_mlkf_opt.c
