#
# Mikrokopter Actuators
#
# enable the subsystem for your firmware:
# <firmware name="rotorcraft">
#   ...
#   <subsystem name="actuators"     type="mkk">
#     <configure name="MKK_I2C_SCL_TIME" value="50"/> <!-- this is optional, 150 is default, use 50 for 8 motors-->
#   </subsystem>
#   <define name="I2C_TRANSACTION_QUEUE_LEN" value="10"/> <!-- default is 8, increase to 10 or more for 8 motors-->
# </firmware>
#
#
# required xml configuration:
#  <section name="ACTUATORS_MKK" prefix="ACTUATORS_MKK_">
#    <define name="NB" value="4"/>
#    <define name="ADDR" value="{ 0x52, 0x54, 0x56, 0x58 }"/>
#  </section>
#
#  servo section with driver="Mkk"
#  command_laws section to map motor_mixing commands to servos
#  max command = 255

$(TARGET).CFLAGS += -DACTUATORS

ACTUATORS_MKK_SRCS = subsystems/actuators/actuators_mkk.c


ifeq ($(ARCH), lpc21)
ACTUATORS_MKK_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
ACTUATORS_MKK_I2C_DEV ?= i2c1
endif

ifeq ($(TARGET), ap)
ifndef ACTUATORS_MKK_I2C_DEV
$(error Error: ACTUATORS_MKK_I2C_DEV not configured!)
endif
endif

# convert i2cx to upper/lower case
ACTUATORS_MKK_I2C_DEV_UPPER=$(shell echo $(ACTUATORS_MKK_I2C_DEV) | tr a-z A-Z)
ACTUATORS_MKK_I2C_DEV_LOWER=$(shell echo $(ACTUATORS_MKK_I2C_DEV) | tr A-Z a-z)

ACTUATORS_MKK_CFLAGS += -DACTUATORS_MKK_I2C_DEV=$(ACTUATORS_MKK_I2C_DEV_LOWER)
ACTUATORS_MKK_CFLAGS += -DUSE_$(ACTUATORS_MKK_I2C_DEV_UPPER)

ifeq ($(ARCH), lpc21)
# set default i2c timing if not already configured
ACTUATORS_MKK_I2C_SCL_TIME ?= 150
ACTUATORS_MKK_CFLAGS += -D$(ACTUATORS_MKK_I2C_DEV_UPPER)_SCLL=$(ACTUATORS_MKK_I2C_SCL_TIME)
ACTUATORS_MKK_CFLAGS += -D$(ACTUATORS_MKK_I2C_DEV_UPPER)_SCLH=$(ACTUATORS_MKK_I2C_SCL_TIME)
endif

ap.CFLAGS += $(ACTUATORS_MKK_CFLAGS)
ap.srcs   += $(ACTUATORS_MKK_SRCS)

# Simulator
nps.srcs += subsystems/actuators/actuators_mkk.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_MKK_I2C_DEV=i2c0
