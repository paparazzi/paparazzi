ifeq ($(ARCH), lpc21)
$(error Error: dualpwm actuators only implemented for stm32)
endif

$(TARGET).CFLAGS += -DACTUATORS
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_dualpwm_arch.c
ifeq ($(ARCH), stm32)
$(TARGET).srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_shared_arch.c
endif

