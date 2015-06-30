#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

PPRZ_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)

$(TARGET).CFLAGS += -DUSE_$(MODEM_PORT)
$(TARGET).CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

$(TARGET).CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(PPRZ_MODEM_PORT_LOWER) -DPPRZ_UART=$(PPRZ_MODEM_PORT_LOWER)
$(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c

