# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART

# make sure that SEPARATE_FBW is configured
ifeq (,$(findstring $(SEPARATE_FBW),1 TRUE))
$(error Using intermcu via UART, so dual mcu with separate fbw. Please add <configure name="SEPARATE_FBW" value="1"/>)
endif

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
