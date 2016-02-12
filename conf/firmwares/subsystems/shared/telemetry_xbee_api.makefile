# XBee modems in API mode
#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

XBEE_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
XBEE_MODEM_PORT_UPPER=$(shell echo $(MODEM_PORT) | tr a-z A-Z)

$(TARGET).CFLAGS += -DUSE_$(XBEE_MODEM_PORT_UPPER)
$(TARGET).CFLAGS += -D$(XBEE_MODEM_PORT_UPPER)_BAUD=$(MODEM_BAUD) -DXBEE_BAUD=$(MODEM_BAUD)

$(TARGET).CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(XBEE_MODEM_PORT_LOWER) -DXBEE_UART=$(XBEE_MODEM_PORT_LOWER)
$(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=xbee_tp -DDATALINK=XBEE
$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/datalink.c pprzlink/src/xbee_transport.c subsystems/datalink/telemetry.c
