#
# Booz2 Mikrokopter Actuators
#
#
# required xml:
#  <section name="BUSS_BLMC" prefix="BUSS_BLMC_">
#   <define name="ADDR" value="{ 0x52, 0x54, 0x56, 0x58 }"/>
#  </section>
#
#  <section name="SUPERVISION" prefix="SUPERVISION_">
#    <define name="FRONT_ROTOR_CW" value="1"/>
#    <define name="TRIM_A" value="2"/>
#    <define name="TRIM_E" value="-1"/>
#    <define name="TRIM_R" value="3"/>
#  </section>
#
#

#
ap.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ap.srcs += $(BOOZ_PRIV_ARCH)/actuators_buss_twi_blmc_hw.c
# on I2C0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


# Simulator
sim.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
sim.srcs += $(BOOZ_PRIV_SIM)/actuators_buss_twi_blmc_hw.c actuators.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
sim.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c