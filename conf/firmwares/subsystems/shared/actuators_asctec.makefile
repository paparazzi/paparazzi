# asctec controllers
#
# required xml configuration:
#
#  servo section with driver="Asctec"
#  command_laws section to map motor_mixing commands to servos
#

$(TARGET).CFLAGS += -DACTUATORS
ap.srcs += subsystems/actuators/actuators_asctec.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150
endif

ifeq ($(ARCH), stm32)
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
ap.CFLAGS += -DUSE_I2C1
endif


# Simulator
nps.srcs += subsystems/actuators/actuators_asctec.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_ASCTEC_DEVICE=i2c0

