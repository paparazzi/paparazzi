
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
#test_telemetry: $(TT_SRCS)
#	$(CC) $(TT_CFLAGS) -o $@ $^ $(TT_LDFLAGS)

# http://groups.google.com/group/beagleboard/browse_frm/thread/15d9488c1ec314ef/ca50640b9c51cef2
# http://www.nabble.com/Overo-oe-SPI-questions-td22194463.html
# http://docwiki.gumstix.org/Sample_code/C/SPI
# http://www.nabble.com/SPI-Bus-td23307852.html
# http://groups.google.com/group/beagleboard/browse_thread/thread/42988f0e14db0f01/3200cce7400bb6b8?lnk=gst&q=mcspi#3200cce7400bb6b8
#
# http://www.nabble.com/Making-Changes-permanent-td23358477.html

#<s4wrxttcs2> "export PATH=${PATH}:/home/gumstix/oe/tmp/staging/i686-linux/usr/bin"
#<s4wrxttcs2> I just use "export PATH=${PATH}:/home/gumstix/oe/tmp/cross/armv7a/bin" and
#<s4wrxttcs2> or "make ARCH=arm CROSS_COMPILE=arm-angstrom-linux-gnueabi-" if I have a makefile

OVERO_OE=/overo-oe

CROSS_CC=$(OVERO_OE)/tmp/cross/armv7a/bin/arm-angstrom-linux-gnueabi-gcc

CROSS_CFLAGS  = -Wall
CROSS_LDFLAGS = 

test_telemetry: test_telemetry.c
	$(CROSS_CC) $(CROSS_CFLAGS) -o $@ $^ $(CROSS_LDFLAGS)

test_spi: test_spi.c
	$(CROSS_CC) $(CROSS_CFLAGS) -o $@ $^ $(CROSS_LDFLAGS)

onboard_logger: onboard_logger.c
	$(CC) $(CFLAGS) -o $@ $^ -lpcap

clean:
	rm -f *~ fms test_telemetry
