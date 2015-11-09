# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART

ifeq ($(TARGET),fbw)
  INTERMCU_PORT ?= UART2
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  fbw.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B230400
  fbw.CFLAGS += -DINTER_MCU_FBW
  fbw.srcs += subsystems/datalink/pprz_transport.c
  fbw.srcs += subsystems/intermcu/intermcu_fbw.c
else
  INTERMCU_PORT ?= UART2
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  ap.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B230400
  ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/intermcu/intermcu_ap.h\" -DRADIO_CONTROL
  ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED) -DINTER_MCU_AP

  ifneq ($(TARGET),sim)
		$(TARGET).srcs += subsystems/intermcu/intermcu_ap.c
		$(TARGET).srcs += subsystems/radio_control.c
		$(TARGET).srcs += subsystems/datalink/pprz_transport.c
	endif
endif
