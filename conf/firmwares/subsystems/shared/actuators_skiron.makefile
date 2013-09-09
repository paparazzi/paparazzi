#
# Skiron Actuators
#
# enable the subsystem for your firmware:
# <firmware name="rotorcraft">
#   ...
#   <subsystem name="actuators"     type="skiron">
#     <configure name="SKIRON_I2C_SCL_TIME" value="50"/> <!-- this is optional, 150 is default -->
#   </subsystem>
#   ...
# </firmware>
#
#
# required xml configuration:
#
#  servo section with driver="Skiron"
#  command_laws section to map motor_mixing commands to servos
#

$(TARGET).CFLAGS += -DACTUATORS
ACTUATORS_SKIRON_SRCS = subsystems/actuators/actuators_skiron.c

# set default i2c device if not already configured
ifeq ($(ARCH), lpc21)
ACTUATORS_SKIRON_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
ACTUATORS_SKIRON_I2C_DEV ?= i2c1
endif

ifeq ($(TARGET), ap)
ifndef ACTUATORS_SKIRON_I2C_DEV
$(error Error: ACTUATORS_SKIRON_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
ACTUATORS_SKIRON_I2C_DEV_UPPER=$(shell echo $(ACTUATORS_SKIRON_I2C_DEV) | tr a-z A-Z)
ACTUATORS_SKIRON_I2C_DEV_LOWER=$(shell echo $(ACTUATORS_SKIRON_I2C_DEV) | tr A-Z a-z)

ACTUATORS_SKIRON_CFLAGS += -DACTUATORS_SKIRON_I2C_DEV=$(ACTUATORS_SKIRON_I2C_DEV_LOWER)
ACTUATORS_SKIRON_CFLAGS += -DUSE_$(ACTUATORS_SKIRON_I2C_DEV_UPPER)

ifeq ($(ARCH), lpc21)
# set default i2c timing if not already configured
ACTUATORS_SKIRON_I2C_SCL_TIME ?= 150
ACTUATORS_SKIRON_CFLAGS += -D$(ACTUATORS_SKIRON_I2C_DEV_UPPER)_SCLL=$(ACTUATORS_SKIRON_I2C_SCL_TIME)
ACTUATORS_SKIRON_CFLAGS += -D$(ACTUATORS_SKIRON_I2C_DEV_UPPER)_SCLH=$(ACTUATORS_SKIRON_I2C_SCL_TIME)
endif

ap.CFLAGS += $(ACTUATORS_SKIRON_CFLAGS)
ap.srcs   += $(ACTUATORS_SKIRON_SRCS)


# Simulator
nps.srcs += subsystems/actuators/actuators_skiron.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_SKIRON_I2C_DEV=i2c0

