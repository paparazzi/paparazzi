
CFLAGS = -Wall `pkg-config --cflags glib-2.0` -g
LDFLAGS = `pkg-config --libs glib-2.0`

fms: fms_main.c fms_ap_link.c fms_serial_port.c fms_gs_link.c fms_network.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *~ fms