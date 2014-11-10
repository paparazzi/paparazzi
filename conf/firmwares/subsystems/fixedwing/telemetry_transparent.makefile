# Hey Emacs, this is a -*- makefile -*-

PPRZ_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)

telemetry_CFLAGS = -DUSE_$(MODEM_PORT)
telemetry_CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
telemetry_CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(PPRZ_MODEM_PORT_LOWER) -DPPRZ_UART=$(MODEM_PORT)
telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
telemetry_srcs = subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c

ap.CFLAGS += $(telemetry_CFLAGS)
ap.srcs += $(telemetry_srcs) $(SRC_FIRMWARE)/datalink.c

