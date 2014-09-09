
#serial UDP

include $(CFG_SHARED)/udp.makefile

MODEM_DEV         ?= UDP0
MODEM_PORT_OUT    ?= 4242
MODEM_PORT_IN     ?= 4243
MODEM_BROADCAST   ?= TRUE

MODEM_CFLAGS  = -DUSE_$(MODEM_DEV) -D$(MODEM_DEV)_PORT_OUT=$(MODEM_PORT_OUT) -D$(MODEM_DEV)_PORT_IN=$(MODEM_PORT_IN)
MODEM_CFLAGS += -D$(MODEM_DEV)_BROADCAST=$(MODEM_BROADCAST) -D$(MODEM_DEV)_HOST=$(MODEM_HOST)

TELEM_CFLAGS  = -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(MODEM_DEV) -DPPRZ_UART=$(MODEM_DEV)
TELEM_CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ -DDefaultPeriodic='&telemetry_Main'


ap.CFLAGS += $(MODEM_CFLAGS) $(TELEM_CFLAGS)
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
