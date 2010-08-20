#
# Booz Mikrokopter Actuators
#
#
# required xml:
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


#
ap.srcs += $(SRC_BOOZ)/actuators/booz_supervision.c
ap.srcs += $(SRC_BOOZ)/actuators/booz_actuators_mkk.c
ap.srcs += $(SRC_BOOZ_ARCH)/actuators/booz_actuators_mkk_arch.c
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

ifeq ($(ARCHI), arm7)
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
ap.CFLAGS += -DI2C0_STOP_HANDLER=ActuatorsMkkI2cHandler
ap.CFLAGS += -DI2C0_STOP_HANDLER_HEADER=\"actuators/booz_actuators_mkk.h\"
else ifeq ($(ARCHI), stm32) 
ap.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1  -DUSE_TIM2_IRQ
ap.CFLAGS += -DUSE_I2C1
endif

# Simulator
sim.srcs += $(SRC_BOOZ)/actuators/booz_supervision.c
sim.srcs += $(SRC_BOOZ)/actuators/booz_actuators_mkk.c
sim.srcs += $(SRC_BOOZ_SIM)/actuators/booz_actuators_mkk_arch.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
sim.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
