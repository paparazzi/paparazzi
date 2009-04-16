
ARCHI=geode

SRC_RDY=readyboard
SRC_FMS=fms

ap.ARCHDIR = $(ARCHI)

ap.LDFLAGS = -lm -levent -lrt

ap.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS)

ap.srcs=$(SRC_RDY)/ready_main.c
ap.srcs+=$(SRC_RDY)/rdyb_xsens.c
ap.srcs+=$(SRC_FMS)/fms_serial_port.c
ap.srcs+=$(SRC_RDY)/rdyb_ahrs.c
ap.srcs+=$(SRC_RDY)/rdyb_gpio.c
ap.srcs+=$(SRC_RDY)/rdyb_can.c
ap.srcs+=$(SRC_RDY)/rdyb_control.c

ap.CFLAGS += -DDOWNLINK
ap.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
ap.srcs += $(SRC_FMS)/fms_network.c
ap.srcs += $(SRC_FMS)/udp_transport.c
ap.srcs += downlink.c


test_timing.ARCHDIR = $(ARCHI)

test_timing.LDFLAGS = -lm -levent -lrt

test_timing.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS)

test_timing.srcs=$(SRC_RDY)/rdyb_test_timing.c
test_timing.srcs+=$(SRC_RDY)/rdyb_gpio.c


test_timing2.ARCHDIR = $(ARCHI)

test_timing2.LDFLAGS = -lm -levent -lrt

test_timing2.CFLAGS += -I$(SRC_RDY)

test_timing2.srcs=$(SRC_RDY)/test_timing2.c
test_timing2.srcs+=$(SRC_RDY)/rdyb_gpio.c
