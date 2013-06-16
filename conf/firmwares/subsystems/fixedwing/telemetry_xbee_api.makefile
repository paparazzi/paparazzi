# Hey Emacs, this is a -*- makefile -*-

# XBee modems in API mode

ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD) -DXBEE_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(MODEM_PORT) -DXBEE_UART=$(MODEM_PORT)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=XBeeTransport -DDATALINK=XBEE
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/xbee.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c
