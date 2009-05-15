
ARCHI=geode

SRC_RDY=readyboard
SRC_FMS=fms
SRC_BOOZ=booz

PERIODIC_FREQ = 512

ap.ARCHDIR = $(ARCHI)

ap.LDFLAGS = -lm -levent -lrt

ap.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS) -I$(SRC_BOOZ) -DPERIODIC_FREQ=$(PERIODIC_FREQ)

ap.srcs =$(SRC_RDY)/ready_main.c
ap.srcs+=$(SRC_RDY)/rdyb_gpio.c
ap.srcs+=$(SRC_RDY)/rdyb_timer.c

#ap.CFLAGS += -DUSE_XSENS_AHRS
ap.srcs+=$(SRC_RDY)/rdyb_xsens.c
ap.srcs+=$(SRC_FMS)/fms_serial_port.c
ap.srcs+=$(SRC_RDY)/rdyb_estimator.c
ap.srcs+=$(SRC_RDY)/rdyb_ahrs.c
#ap.srcs+=$(SRC_RDY)/rdyb_control.c
ap.srcs+=$(SRC_RDY)/ahrs_quat_fast_ekf.c

ap.srcs+=$(SRC_RDY)/elevator_control.c
ap.srcs+=$(SRC_RDY)/pid_generic.c
ap.srcs+= commands.c

ap.CFLAGS+= -DACTUATORS=\"rdyb_actuators.h\"
ap.srcs+= actuators.c $(SRC_RDY)/rdyb_actuators.c

ap.CFLAGS  += `pkg-config --cflags glib-2.0`
ap.LDFLAGS += `pkg-config --libs glib-2.0`
ap.srcs+= $(SRC_RDY)/rdyb_event_manager.c


ap.srcs+=$(SRC_RDY)/rdyb_can.c
ap.srcs+=$(SRC_RDY)/rdyb_throttle.c

ap.CFLAGS += -DDOWNLINK
ap.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
ap.srcs += $(SRC_FMS)/fms_network.c
ap.srcs += $(SRC_FMS)/udp_transport.c
ap.srcs += downlink.c

ap.srcs += $(SRC_RDY)/rdyb_telemetry.c
ap.srcs += $(SRC_RDY)/rdyb_datalink.c
ap.CFLAGS += -DLINK_HOST=\"255.255.255.255\"




#
#
# test timer
#
#
test_timer.ARCHDIR = $(ARCHI)

test_timer.LDFLAGS = -lm -levent -lrt

test_timer.CFLAGS += -I$(SRC_RDY)

test_timer.srcs=$(SRC_RDY)/test_timer.c
test_timer.srcs+=$(SRC_RDY)/rdyb_timer.c
test_timer.srcs+=$(SRC_RDY)/rdyb_gpio.c

#
#
# test telemetry
#
#
test_telemetry.ARCHDIR = $(ARCHI)

test_telemetry.LDFLAGS = -lm -levent -lrt

test_telemetry.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS)

test_telemetry.srcs =$(SRC_RDY)/test_telemetry.c
test_telemetry.srcs+=$(SRC_RDY)/rdyb_timer.c
test_telemetry.srcs+=$(SRC_RDY)/rdyb_gpio.c

test_telemetry.CFLAGS += -DDOWNLINK
test_telemetry.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
test_telemetry.srcs += $(SRC_FMS)/fms_network.c
test_telemetry.srcs += $(SRC_FMS)/udp_transport.c
test_telemetry.srcs += downlink.c


#
#
# test xtend
#
#
test_xtend.ARCHDIR = $(ARCHI)

test_xtend.LDFLAGS = -lm -levent -lrt

test_xtend.CFLAGS += -I$(SRC_RDY)

test_xtend.srcs=$(SRC_RDY)/test_xtend.c
test_xtend.srcs+=$(SRC_RDY)/rdyb_timer.c
test_xtend.srcs+=$(SRC_RDY)/rdyb_gpio.c

test_xtend.CFLAGS += -DXTEND_DEVICE=\"/dev/ttyS2\"
#test_xtend.srcs+=$(SRC_RDY)/rdyb_xtend.c
test_xtend.srcs+=$(SRC_FMS)/fms_serial_port.c

include $(PAPARAZZI_SRC)/conf/autopilot/csc.makefile
