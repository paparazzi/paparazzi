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
#  <section name="ACTUATORS_SKIRON" prefix="ACTUATORS_SKIRON_">
#    <define name="NB" value="4"/>
#    <define name="IDX" value="{ 0, 1, 2, 3 }"/>
#  </section>
#
#  <section name="SUPERVISION" prefix="SUPERVISION_">
#    <define name="MIN_MOTOR" value="20"/>
#    <define name="MAX_MOTOR" value="255"/>
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
#

# set default i2c timing if not already configured
ifeq ($(SKIRON_I2C_SCL_TIME), )
SKIRON_I2C_SCL_TIME=150
endif

ap.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_skiron.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DACTUATORS_SKIRON_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(SKIRON_I2C_SCL_TIME) -DI2C0_SCLH=$(SKIRON_I2C_SCL_TIME) -DI2C0_VIC_SLOT=10
endif

# Simulator
sim.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
sim.srcs += $(SRC_FIRMWARE)/actuators/actuators_skiron.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(SKIRON_I2C_SCL_TIME) -DI2C0_SCLH=$(SKIRON_I2C_SCL_TIME) -DI2C0_VIC_SLOT=10 -DACTUATORS_MKK_DEVICE=i2c0
sim.srcs += mcu_periph/i2c.c
sim.srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c

