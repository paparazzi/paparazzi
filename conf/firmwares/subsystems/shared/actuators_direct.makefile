# for lisa_l

$(TARGET).CFLAGS += -DACTUATORS=\"servos_direct_hw.h\" -DSERVOS_DIRECT
$(TARGET).srcs += $(SRC_ARCH)/servos_direct_hw.c actuators.c

ifeq ($(ARCH), stm32)
$(TARGET).srcs    += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
endif
