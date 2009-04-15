ARCHI=geode

SRC_FMS=fms

ap.ARCHDIR = $(ARCHI)

ap.LDFLAGS = -lm -levent -lrt

ap.CFLAGS += -I$(SRC_FMS)

ap.srcs=$(SRC_FMS)/fms_test_datalink.c

ap.CFLAGS += -DDOWNLINK
ap.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
ap.srcs += $(SRC_FMS)/fms_network.c
ap.srcs += $(SRC_FMS)/udp_transport.c
ap.srcs += downlink.c