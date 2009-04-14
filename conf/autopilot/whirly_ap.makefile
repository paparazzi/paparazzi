
ARCHI=geode

SRC_RDY=readyboard
SRC_FMS=fms

ap.ARCHDIR = $(ARCHI)

ap.LDFLAGS = -lm -levent

ap.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS)

ap.srcs=$(SRC_RDY)/ready_main.c
ap.srcs+=$(SRC_RDY)/rdyb_xsens.c
ap.srcs+=$(SRC_FMS)/fms_serial_port.c
ap.srcs+=$(SRC_RDY)/rdyb_ahrs.c
ap.srcs+=$(SRC_RDY)/rdyb_gpio.c

ap.CFLAGS += -DDOWNLINK
ap.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
ap.srcs += $(SRC_FMS)/fms_network.c
ap.srcs += $(SRC_FMS)/udp_transport.c
ap.srcs += downlink.c