# Hey Emacs, this is a -*- makefile -*-

# InterMCU type CAN

ifneq ($(TARGET),sim)

# make sure that SEPARATE_FBW is configured
ifeq (,$(findstring $(SEPARATE_FBW),1 TRUE))
$(error Using intermcu via CAN, so dual mcu with separate fbw. Please add <configure name="SEPARATE_FBW" value="1"/>)
endif


$(TARGET).CFLAGS += -DINTER_MCU -DMCU_CAN_LINK
$(TARGET).srcs += link_mcu_can.c
$(TARGET).srcs += mcu_periph/can.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/can_arch.c
endif
