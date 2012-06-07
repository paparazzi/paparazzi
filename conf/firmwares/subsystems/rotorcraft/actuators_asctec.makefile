# asctec controllers
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
endif

ifeq ($(ARCH), stm32)
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
ap.CFLAGS += -DUSE_I2C1
endif


# Simulator
nps.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_ASCTEC_DEVICE=i2c0

