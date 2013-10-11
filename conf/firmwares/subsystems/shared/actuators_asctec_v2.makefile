# asctec controllers v2
#
# required xml configuration:
#
#  servo section with driver="Asctec_v2"
#  command_laws section to map motor_mixing commands to servos
#

$(TARGET).CFLAGS += -DACTUATORS
ACTUATORS_ASCTEC_V2_SRCS = subsystems/actuators/actuators_asctec_v2.c


# set default i2c device if not already configured
ifeq ($(ARCH), lpc21)
ACTUATORS_ASCTEC_V2_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
ACTUATORS_ASCTEC_V2_I2C_DEV ?= i2c1
endif

ifeq ($(TARGET), ap)
ifndef ACTUATORS_ASCTEC_V2_I2C_DEV
$(error Error: ACTUATORS_ASCTEC_V2_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
ACTUATORS_ASCTEC_V2_I2C_DEV_UPPER=$(shell echo $(ACTUATORS_ASCTEC_V2_I2C_DEV) | tr a-z A-Z)
ACTUATORS_ASCTEC_V2_I2C_DEV_LOWER=$(shell echo $(ACTUATORS_ASCTEC_V2_I2C_DEV) | tr A-Z a-z)

ACTUATORS_ASCTEC_V2_CFLAGS += -DACTUATORS_ASCTEC_V2_I2C_DEV=$(ACTUATORS_ASCTEC_V2_I2C_DEV_LOWER)
ACTUATORS_ASCTEC_V2_CFLAGS += -DUSE_$(ACTUATORS_ASCTEC_V2_I2C_DEV_UPPER)

ifeq ($(ARCH), lpc21)
# set default i2c timing if not already configured
ACTUATORS_ASCTEC_V2_I2C_SCL_TIME ?= 150
ACTUATORS_ASCTEC_V2_CFLAGS += -D$(ACTUATORS_ASCTEC_V2_I2C_DEV_UPPER)_SCLL=$(ACTUATORS_ASCTEC_V2_I2C_SCL_TIME)
ACTUATORS_ASCTEC_V2_CFLAGS += -D$(ACTUATORS_ASCTEC_V2_I2C_DEV_UPPER)_SCLH=$(ACTUATORS_ASCTEC_V2_I2C_SCL_TIME)
endif

ap.CFLAGS += $(ACTUATORS_ASCTEC_V2_CFLAGS)
ap.srcs   += $(ACTUATORS_ASCTEC_V2_SRCS)



# Simulator
nps.srcs += subsystems/actuators/actuators_asctec_v2.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_ASCTEC_V2_I2C_DEV=i2c0
