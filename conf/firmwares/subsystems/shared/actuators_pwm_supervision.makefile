
$(TARGET).CFLAGS += -DACTUATORS -DUSE_SUPERVISION
$(TARGET).srcs   += subsystems/actuators/supervision.c
$(TARGET).srcs   += subsystems/actuators/actuators_pwm.c
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c

