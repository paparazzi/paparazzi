# mikrokopter controllers
ap.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ap.srcs += $(BOOZ_PRIV_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
# on I2C0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
# supervision
ap.srcs += $(BOOZ_PRIV)/booz_supervision_int.c


# Simulator
sim.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
sim.srcs += $(BOOZ_PRIV_SIM)/actuators_buss_twi_blmc_hw.c actuators.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
sim.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c