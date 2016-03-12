# Hey Emacs, this is a -*- makefile -*-

# InterMCU type UART

FBW_MODE_LED ?= none
INTERMCU_PORT ?= UART3
INTERMCU_BAUD ?= B230400

INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
INTERMCU_PORT_UPPER = $(shell echo $(INTERMCU_PORT) | tr a-z A-Z)

ifeq ($(TARGET),fbw)
  fbw.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER)
  fbw.CFLAGS += -DUSE_$(INTERMCU_PORT_UPPER) -D$(INTERMCU_PORT_UPPER)_BAUD=$(INTERMCU_BAUD)
  fbw.CFLAGS += -DINTER_MCU_FBW -DDOWNLINK
ifneq ($(FBW_MODE_LED),none)
  fbw.CFLAGS += -DFBW_MODE_LED=$(FBW_MODE_LED)
endif
  fbw.srcs += pprzlink/src/pprz_transport.c
  fbw.srcs += subsystems/intermcu/intermcu_fbw.c
else
  ap.CFLAGS += -DINTER_MCU_AP -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER)
  ap.CFLAGS += -DUSE_$(INTERMCU_PORT_UPPER) -D$(INTERMCU_PORT_UPPER)_BAUD=$(INTERMCU_BAUD)
  $(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/intermcu/intermcu_ap.h\" -DRADIO_CONTROL
  RADIO_CONTROL_LED ?= none
ifneq ($(RADIO_CONTROL_LED),none)
    $(TARGET).CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

  ap.srcs += subsystems/intermcu/intermcu_ap.c
  ap.srcs += pprzlink/src/pprz_transport.c
  $(TARGET).srcs += subsystems/radio_control.c
endif
