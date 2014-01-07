# Hey Emacs, this is a -*- makefile -*-

telemetry_CFLAGS = -DUSE_$(MODEM_PORT)
telemetry_CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
telemetry_CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
telemetry_srcs = subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c

ap.CFLAGS += $(telemetry_CFLAGS)
ap.srcs += $(telemetry_srcs) $(SRC_FIRMWARE)/datalink.c

