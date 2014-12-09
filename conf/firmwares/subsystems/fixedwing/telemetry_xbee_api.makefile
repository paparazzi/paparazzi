# Hey Emacs, this is a -*- makefile -*-

# XBee modems in API mode

XBEE_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD) -DXBEE_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(XBEE_MODEM_PORT_LOWER) -DXBEE_UART=$(MODEM_PORT)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=xbee_tp -DDATALINK=XBEE -DDefaultPeriodic='&telemetry_Ap'
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/xbee.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/ap_downlink.c

# avoid fbw_telemetry_mode error
ap.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c
