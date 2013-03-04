#
# Mikrokopter v2 Actuators
#
# enable the subsystem for your firmware:
# <firmware name="rotorcraft">
#   ...
#   <subsystem name="actuators"     type="mkk">
#     <configure name="MKK_V2_I2C_SCL_TIME" value="50"/> <!-- this is optional, 150 is default, use 50 for 8 motors-->
#   </subsystem>
#   <define name="I2C_TRANSACTION_QUEUE_LEN" value="10"/> <!-- default is 8, increase to 10 or more for 8 motors-->
# </firmware>
#
#
# required xml configuration:
#  <section name="ACTUATORS_MKK_V2" prefix="ACTUATORS_MKK_V2_">
#    <define name="NB" value="4"/>
#    <define name="ADDR" value="{ 0x52, 0x54, 0x56, 0x58 }"/>
#  </section>
#
#  servo section with driver="Mkk_v2"
#  command_laws section to map motor_mixing commands to servos
#  max command = 2047

$(TARGET).CFLAGS += -DACTUATORS
ap.srcs += subsystems/actuators/actuators_mkk_v2.c

ifeq ($(ARCH), lpc21)

# set default i2c timing if not already configured
ifeq ($(MKK_V2_I2C_SCL_TIME), )
MKK_V2_I2C2_SCL_TIME=150
endif

ap.CFLAGS += -DACTUATORS_MKK_V2_DEVICE=i2c0
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=$(MKK_V2_I2C_SCL_TIME) -DI2C0_SCLH=$(MKK_V2_I2C_SCL_TIME)

else ifeq ($(ARCH), stm32)
ap.CFLAGS += -DACTUATORS_MKK_V2_DEVICE=i2c1
ap.CFLAGS += -DUSE_I2C1
endif

# Simulator:
nps.srcs += subsystems/actuators/actuators_mkk_v2.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_MKK_V2_DEVICE=i2c0
