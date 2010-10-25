#
#
#

ap.CFLAGS += -DAHRS_ALIGNER_LED=3
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_mlkf.c
ap.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_opt.c

sim.CFLAGS += -I$(SRC_BOOZ_PRIV)
sim.CFLAGS += -DAHRS_ALIGNER_LED=3
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
sim.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
sim.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_mlkf.c
sim.srcs += $(SRC_BOOZ_PRIV)/ahrs/booz_ahrs_mlkf_opt.c
