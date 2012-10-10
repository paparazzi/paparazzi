

$(TARGET).CFLAGS += -DACTUATORS
$(TARGET).srcs   += subsystems/actuators/actuators_pwm.c
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
