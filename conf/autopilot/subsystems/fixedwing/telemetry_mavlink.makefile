# Hey Emacs, this is a -*- makefile -*-


ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_FBW_DEVICE=$(MODEM_PORT) -DDOWNLINK_AP_DEVICE=$(MODEM_PORT) -DDOWNLINK_MAV_DEVICE=$(MODEM_PORT) -DMAV_UART=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=MavlinkTransport -DDATALINK=MAV
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/mavlink_transport.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c
