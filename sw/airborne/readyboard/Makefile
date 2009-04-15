
SRC_INC=$(PAPARAZZI_SRC)/sw/include
SRC_VAR_INC=$(PAPARAZZI_SRC)/var/include
SRC_AF=$(PAPARAZZI_SRC)/var/BOOZ2_A1
SRC_AB=$(PAPARAZZI_SRC)/sw/airborne
SRC_FMS=$(SRC_AB)/fms

CFLAGS  = -Wall  -g -mtune=geode -I$(SRC_INC) -I$(SRC_VAR_INC) -I$(SRC_AF) -I$(SRC_AB) -I$(SRC_FMS)
LDFLAGS = -lm -levent

SRCS =   ready_main.c            \
	rdyb_xsens.c             \
        rdyb_serial_port.c       \
	rdyb_ahrs.c              \
	rdyb_gpio.c              \

CFLAGS += -DDOWNLINK
CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
SRCS += $(SRC_FMS)/fms_network.c
SRCS += $(SRC_FMS)/udp_transport.c
SRCS += $(SRC_AB)/downlink.c


ready_main: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)



clean:
	rm -f *~ ready_main


