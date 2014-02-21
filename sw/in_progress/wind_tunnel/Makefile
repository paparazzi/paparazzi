
# Quiet compilation
Q=@

CC = gcc
GLIB_CFLAGS  = -Wall $(shell pkg-config glib-2.0 --cflags)
GLIB_LDFLAGS = $(shell pkg-config glib-2.0 --libs) -lglibivy $(shell pcre-config --libs)

main: main.c serial_port.c
	$(CC) $(GLIB_CFLAGS) -o $@ main.c serial_port.c $(GLIB_LDFLAGS)


clean:
	$(Q)rm -f main
