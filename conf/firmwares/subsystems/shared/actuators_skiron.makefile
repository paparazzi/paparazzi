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

# set default i2c timing if not already configured
ifeq ($(SKIRON_I2C_SCL_TIME), )
SKIRON_I2C_SCL_TIME=150
endif

$(TARGET).CFLAGS += -DACTUATORS
ap.srcs += subsystems/actuators/actuators_skiron.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DACTUATORS_SKIRON_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(SKIRON_I2C_SCL_TIME) -DI2C0_SCLH=$(SKIRON_I2C_SCL_TIME)
endif

# Simulator
nps.srcs += subsystems/actuators/actuators_skiron.c
nps.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(SKIRON_I2C_SCL_TIME) -DI2C0_SCLH=$(SKIRON_I2C_SCL_TIME) -DACTUATORS_SKIRON_DEVICE=i2c0

