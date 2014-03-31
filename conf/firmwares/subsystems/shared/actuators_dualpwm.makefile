

ifeq ($(ARCH), stm32)
$(TARGET).CFLAGS += -DACTUATORS
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_dualpwm_arch.c
else
$(error Error: dualpwm actuators only implemented for stm32)
endif



