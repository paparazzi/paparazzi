# asctec controllers
ap.srcs += $(SRC_BOOZ)/actuators/booz_actuators_asctec.c
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

ifeq ($(ARCHI), arm7)
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
endif

ifeq ($(ARCHI), stm32) 
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
ap.CFLAGS += -DUSE_I2C1
endif


# Simulator
sim.srcs += $(SRC_BOOZ)/actuators/booz_actuators_asctec.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
sim.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

