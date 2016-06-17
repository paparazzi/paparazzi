# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART

# make sure that SEPARATE_FBW is configured
ifeq (,$(findstring $(SEPARATE_FBW),1 TRUE))
$(error Using intermcu via UART, so dual mcu with separate fbw. Please add <configure name="SEPARATE_FBW" value="1"/>)
endif

FBW_MODE_LED ?= none
INTERMCU_BAUD ?= B57600

ifeq ($(TARGET),fbw)
  INTERMCU_PORT ?= UART2
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  INTERMCU_PORT_UPPER = $(shell echo $(INTERMCU_PORT) | tr a-z A-Z)
  fbw.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT_UPPER) -D$(INTERMCU_PORT_UPPER)_BAUD=$(INTERMCU_BAUD)
ifneq ($(FBW_MODE_LED),none)
  fbw.CFLAGS += -DFBW_MODE_LED=$(FBW_MODE_LED)
endif
else
  INTERMCU_PORT ?= UART5
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  INTERMCU_PORT_UPPER = $(shell echo $(INTERMCU_PORT) | tr a-z A-Z)
  ap.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT_UPPER) -D$(INTERMCU_PORT_UPPER)_BAUD=$(INTERMCU_BAUD)
endif

ifneq ($(TARGET),sim)
$(TARGET).CFLAGS += -DINTER_MCU -DMCU_UART_LINK
$(TARGET).srcs += ./link_mcu_usart.c
endif
