# asctec controllers
ap.CFLAGS += -DACTUATORS=\"actuators_asctec_twi_blmc_hw.h\"
ap.srcs += $(BOOZ_PRIV_ARCH)/actuators_asctec_twi_blmc_hw.c
# on I2C0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


# asctec controllers
sim.CFLAGS += -DACTUATORS=\"actuators_asctec_twi_blmc_hw.h\"
sim.srcs += $(BOOZ_PRIV_ARCH)/actuators_asctec_twi_blmc_hw.c actuators.c

