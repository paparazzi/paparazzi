# mikrokopter controllers
ap.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ap.srcs += $(BOOZ_PRIV_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
# supervision
ap.srcs += $(BOOZ_PRIV)/booz_supervision_int.c
