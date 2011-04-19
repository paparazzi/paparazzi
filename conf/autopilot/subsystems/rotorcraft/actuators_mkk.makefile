#
# Mikrokopter Actuators
#
# enable the subsystem for your firmware:
# <firmware name="rotorcraft">
#   ...
#   <subsystem name="actuators"     type="mkk">
#     <configure name="MKK_I2C_SCL_TIME" value="50"/> <!-- this is optional, 150 is default, use 50 for 8 motors-->
# </firmware>
#
#
# required xml configuration:
#  <section name="ACTUATORS_MKK" prefix="ACTUATORS_MKK_">
#    <define name="NB" value="4"/>
#    <define name="ADDR" value="{ 0x52, 0x54, 0x56, 0x58 }"/>
#  </section>
#
#  <section name="SUPERVISION" prefix="SUPERVISION_">
#    <define name="MIN_MOTOR" value="2"/>
#    <define name="MAX_MOTOR" value="210"/>
#    <define name="TRIM_A" value="2"/>
#    <define name="TRIM_E" value="-1"/>
#    <define name="TRIM_R" value="3"/>
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
ifeq ($(MKK_I2C_SCL_TIME), )
MKK_I2C_SCL_TIME=150
endif

ap.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(MKK_I2C_SCL_TIME) -DI2C0_SCLH=$(MKK_I2C_SCL_TIME) -DI2C0_VIC_SLOT=11
else ifeq ($(ARCH), stm32)
ap.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1
ap.CFLAGS += -DUSE_I2C1
endif

# Simulator
sim.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
sim.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(MKK_I2C_SCL_TIME) -DI2C0_SCLH=$(MKK_I2C_SCL_TIME) -DI2C0_VIC_SLOT=10 -DACTUATORS_MKK_DEVICE=i2c1
sim.srcs += mcu_periph/i2c.c
sim.srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
