

$(TARGET).CFLAGS += -DACTUATORS -DSERVOS_DIRECT
$(TARGET).srcs   += subsystems/actuators/actuators_pwm.c subsystems/actuators.c
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
