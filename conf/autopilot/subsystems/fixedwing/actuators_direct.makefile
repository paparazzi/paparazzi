# for lisa_l

ap.CFLAGS += -DACTUATORS=\"servos_direct_hw.h\" -DSERVOS_DIRECT
ap.srcs += $(SRC_ARCH)/servos_direct_hw.c $(SRC_FIXEDWING)/actuators.c


# TODO TODO HELP HELP TERRIBLE HORRIBLE HACK!!!!
ifeq ($(ARCHI), stm32)
ap.srcs    += $(SRC_FIXEDWING)/booz/arch/stm32/actuators/booz_actuators_pwm_arch.c
ap.CFLAGS  += -I$(SRC_FIXEDWING)/booz/arch/stm32/
endif
