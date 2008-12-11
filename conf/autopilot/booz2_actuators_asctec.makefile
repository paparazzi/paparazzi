# asctec controllers
ap.CFLAGS += -DACTUATORS=\"actuators_asctec_twi_blmc_hw.h\"
ap.srcs += $(BOOZ_PRIV_ARCH)/actuators_asctec_twi_blmc_hw.c actuators.c
# supervision
ap.srcs += $(BOOZ_PRIV)/booz_supervision_int_nomix.c


# asctec controllers
sim.CFLAGS += -DACTUATORS=\"actuators_asctec_twi_blmc_hw.h\"
sim.srcs += $(BOOZ_PRIV_ARCH)/actuators_asctec_twi_blmc_hw.c actuators.c
# supervision
sim.srcs += $(BOOZ_PRIV)/booz_supervision_int_nomix.c

