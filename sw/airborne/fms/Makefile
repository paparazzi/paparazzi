
CFLAGS = -Wall `pkg-config --cflags glib-2.0` -g
LDFLAGS = `pkg-config --libs glib-2.0`

fms: fms_main.c fms_ap_link.c fms_serial_port.c fms_gs_link.c fms_network.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)



TT_CFLAGS = -Wall 
TT_CFLAGS += -I../../include
TT_CFLAGS += -I..
TT_CFLAGS += -I../../../var/include
TT_CFLAGS += -I../../../var/BOOZ2_A2
TT_SRCS = test_telemetry.c
TT_LDFLAGS = -levent
TT_CFLAGS += -DDOWNLINK
TT_CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
TT_SRCS += fms_network.c udp_transport.c ../downlink.c
test_telemetry: $(TT_SRCS)
	$(CC) $(TT_CFLAGS) -o $@ $^ $(TT_LDFLAGS)





clean:
	rm -f *~ fms test_telemetry