
CC = gcc
GLIB_CFLAGS  = -Wall  `pkg-config glib-2.0 --cflags`
GLIB_LDFLAGS =  `pkg-config glib-2.0 --libs` -lglibivy -lpcre

main: main.c serial_port.c
	$(CC) $(GLIB_CFLAGS) -o $@ main.c serial_port.c $(GLIB_LDFLAGS)


clean:
	rm -f main