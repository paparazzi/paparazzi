

$(TARGET).CFLAGS += -DACTUATORS
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
ifeq ($(ARCH), stm32)
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_shared_arch.c
endif
