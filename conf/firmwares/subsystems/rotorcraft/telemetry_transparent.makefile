#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

PPRZ_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(PPRZ_MODEM_PORT_LOWER) -DPPRZ_UART=$(PPRZ_MODEM_PORT_LOWER)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
