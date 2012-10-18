#  Motor Mixing
#
#  <section name="MIXING" prefix="MOTOR_MIXING_">
#    <define name="TRIM_ROLL" value="2"/>
#    <define name="TRIM_PITCH" value="-1"/>
#    <define name="TRIM_YAW" value="3"/>
#    <define name="NB_MOTOR" value="4"/>
#    <define name="SCALE" value="256"/>
#    <define name="ROLL_COEF"   value="{    0,    0, -256,  256}"/>
#    <define name="PITCH_COEF"  value="{ 256,  -256,    0,    0}"/>
#    <define name="YAW_COEF"    value="{-256,  -256,  256,  256}"/>
#    <define name="THRUST_COEF" value="{ 256,   256,  256,  256}"/>
#  </section>
#
#

$(TARGET).CFLAGS += -DUSE_MOTOR_MIXING
$(TARGET).srcs   += subsystems/actuators/motor_mixing.c

