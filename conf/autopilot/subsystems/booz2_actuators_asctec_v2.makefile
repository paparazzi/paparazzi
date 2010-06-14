# asctec controllers v2
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

ap.srcs += $(SRC_BOOZ)/actuators/booz_supervision.c
ap.CFLAGS += -DACTUATORS_ASCTEC_V2_PROTOCOL
ap.srcs += $(SRC_BOOZ)/actuators/booz_actuators_asctec.c
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

ifeq ($(ARCHI), stm32) 
ap.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
ap.CFLAGS += -DUSE_I2C1
endif
