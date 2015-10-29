# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART

ifeq ($(TARGET),fbw)
  INTERMCU_PORT ?= UART2
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  fbw.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B57600
  fbw.CFLAGS += -DINTER_MCU_FBW
else
  INTERMCU_PORT ?= UART5
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  ap.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B57600
  ap.CFLAGS += -DINTER_MCU_AP
endif

ifneq ($(TARGET),sim)
$(TARGET).srcs += ./subsystems/intermcu/intermcu.c
endif
