# for lisa_l

$(TARGET).CFLAGS += -DACTUATORS=\"servos_direct_hw.h\" -DSERVOS_DIRECT
$(TARGET).srcs += $(SRC_ARCH)/servos_direct_hw.c actuators.c


# TODO TODO UGLY HACK: We re-use the booz actuators: Should become universal actuator code!!
# Carefull: paths might get broken with this silly rotorcraft/fixedwing mixup of directories

ifeq ($(ARCH), stm32)
$(TARGET).srcs    += firmwares/rotorcraft/actuators/arch/stm32/actuators_pwm_arch.c
$(TARGET).CFLAGS  += -Ifirmwares/rotorcraft/actuators/arch/stm32
endif
