# AutoQuad ESC32
#
# required xml configuration:
#
#  servo section with driver="ESC32"
#  command_laws section to map motor_mixing commands to servos
#

$(TARGET).CFLAGS += -DACTUATORS -DUSE_CAN_EXT_ID
ACTUATORS_ESC32_SRCS = mcu_periph/can.c $(SRC_ARCH)/mcu_periph/can_arch.c subsystems/actuators/actuators_esc32.c


ap.srcs   += $(ACTUATORS_ESC32_SRCS)


# Simulator
#nps.srcs += subsystems/actuators/actuators_asctec_v2.c
#nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_ASCTEC_V2_I2C_DEV=i2c0
