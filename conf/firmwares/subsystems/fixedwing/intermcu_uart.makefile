# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART


ifeq ($(TARGET),fbw)
  ifeq ($(INTERMCU_PORT),none)
    INTERMCU_PORT = UART2
  endif
  fbw.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B57600
else
  ifeq ($(INTERMCU_PORT),none)
    INTERMCU_PORT = UART5
  endif
  ap.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B57600
endif

ifneq ($(TARGET),sim)
$(TARGET).CFLAGS += -DINTER_MCU -DMCU_UART_LINK
$(TARGET).srcs += ./link_mcu_usart.c
endif



#############################
# CAN:
# fbw.srcs += ./link_mcu_can.c ./mcu_periph/can.c ./arch/stm32/mcu_periph/can_arch.c
# fbw.CFLAGS += -DINTER_MCU -DMCU_CAN_LINK
# $(TARGET).CFLAGS += -DCAN_SJW_TQ=CAN_SJW_1tq -DCAN_BS1_TQ=CAN_BS1_3tq -DCAN_BS2_TQ=CAN_BS2_4tq -DCAN_PRESCALER=12 -DUSE_CAN -DUSE_CAN1 -DUSE_USB_LP_CAN1_RX0_IRQ -DCAN_ERR_RESUME=DISABLE
