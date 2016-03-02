# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART

ifeq ($(TARGET),fbw)
  INTERMCU_PORT ?= UART3
  INTERMCU_BAUD ?= B230400
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  fbw.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT)  -D$(INTERMCU_PORT)_BAUD=$(INTERMCU_BAUD)
  fbw.CFLAGS += -DINTER_MCU_FBW -DDOWNLINK
  fbw.srcs += pprzlink/src/pprz_transport.c
  fbw.srcs += subsystems/intermcu/intermcu_fbw.c
else
  INTERMCU_PORT ?= UART3
  INTERMCU_BAUD ?= B230400
  INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
  ap.CFLAGS += -DINTER_MCU_AP -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER)
  ap.CFLAGS += -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=$(INTERMCU_BAUD)
  $(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/intermcu/intermcu_ap.h\" -DRADIO_CONTROL
  $(TARGET).CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)

	ap.srcs += subsystems/intermcu/intermcu_ap.c
	ap.srcs += pprzlink/src/pprz_transport.c
	$(TARGET).srcs += subsystems/radio_control.c
endif
