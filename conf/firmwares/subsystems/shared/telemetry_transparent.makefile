#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

PPRZ_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
PPRZ_MODEM_PORT_UPPER=$(shell echo $(MODEM_PORT) | tr a-z A-Z)

$(TARGET).CFLAGS += -DUSE_$(PPRZ_MODEM_PORT_UPPER)
$(TARGET).CFLAGS += -D$(PPRZ_MODEM_PORT_UPPER)_BAUD=$(MODEM_BAUD)

$(TARGET).CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(PPRZ_MODEM_PORT_LOWER) -DPPRZ_UART=$(PPRZ_MODEM_PORT_LOWER)
$(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/datalink.c pprzlink/src/pprz_transport.c subsystems/datalink/telemetry.c

