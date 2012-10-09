

$(TARGET).CFLAGS += -DACTUATORS=\"subsystems/actuators/actuators_pwm.h\" -DSERVOS_DIRECT
$(TARGET).srcs   += subsystems/actuators/actuators_pwm.c actuators.c
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
