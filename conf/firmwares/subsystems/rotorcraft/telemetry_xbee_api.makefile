# XBee modems in API mode
#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD) -DXBEE_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(MODEM_PORT) -DXBEE_UART=$(MODEM_PORT)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=XBeeTransport -DDATALINK=XBEE -DDefaultPeriodic='&telemetry_Main'
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/xbee.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
