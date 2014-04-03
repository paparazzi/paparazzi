

ifeq ($(ARCH), stm32)
$(TARGET).CFLAGS += -DACTUATORS
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_dualpwm_arch.c $(SRC_ARCH)/subsystems/actuators/actuators_shared_arch.c
else
ifeq ($(TARGET), ap)
$(error Error: dualpwm actuators only implemented for stm32)
endif
endif



