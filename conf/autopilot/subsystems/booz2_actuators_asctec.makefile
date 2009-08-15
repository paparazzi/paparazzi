# asctec controllers
ap.srcs += $(SRC_BOOZ)/actuators/booz_actuators_asctec.c
# on I2C0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


# asctec controllers
sim.CFLAGS += -DACTUATORS=\"actuators_asctec_twi_blmc_hw.h\"
sim.srcs += $(SRC_BOOZ_SIM)/actuators_asctec_twi_blmc_hw.c actuators.c

