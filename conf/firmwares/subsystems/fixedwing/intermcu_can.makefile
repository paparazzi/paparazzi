# Hey Emacs, this is a -*- makefile -*-

# InterMCU type CAN

ifneq ($(TARGET),sim)
$(TARGET).CFLAGS += -DINTER_MCU -DMCU_CAN_LINK
$(TARGET).srcs += link_mcu_can.c
$(TARGET).srcs += mcu_periph/can.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/can_arch.c
endif

SEPARATE_FBW=1
