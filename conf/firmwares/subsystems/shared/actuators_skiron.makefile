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
#  <section name="SUPERVISION" prefix="SUPERVISION_">
#    <define name="TRIM_A" value="0"/>
#    <define name="TRIM_E" value="0"/>
#    <define name="TRIM_R" value="0"/>
#    <define name="NB_MOTOR" value="4"/>
#    <define name="SCALE" value="256"/>
#    <define name="ROLL_COEF"  value="{    0,    0, -256,  256}"/>
#    <define name="PITCH_COEF" value="{  256, -256,    0,    0}"/>
#    <define name="YAW_COEF"   value="{ -256, -256,  256,  256}"/>
#    <define name="THRUST_COEF" value="{ 256,  256,  256,  256}"/>
#  </section>
#
#  servo section with driver="Skiron"
#  command_laws section to map supervision commands to servos
#

# set default i2c timing if not already configured
ifeq ($(SKIRON_I2C_SCL_TIME), )
SKIRON_I2C_SCL_TIME=150
endif

$(TARGET).CFLAGS += -DACTUATORS
ap.srcs += subsystems/actuators/supervision.c
ap.srcs += subsystems/actuators/actuators_skiron.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DACTUATORS_SKIRON_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(SKIRON_I2C_SCL_TIME) -DI2C0_SCLH=$(SKIRON_I2C_SCL_TIME) -DI2C0_VIC_SLOT=10
endif

# Simulator
nps.srcs += subsystems/actuators/supervision.c
nps.srcs += subsystems/actuators/actuators_skiron.c
nps.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(SKIRON_I2C_SCL_TIME) -DI2C0_SCLH=$(SKIRON_I2C_SCL_TIME) -DI2C0_VIC_SLOT=10 -DACTUATORS_SKIRON_DEVICE=i2c0

