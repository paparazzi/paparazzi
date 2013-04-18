# Quiet compilation
Q=@

CC = gcc
GLIB_CFLAGS  = -Wall  `pkg-config glib-2.0 --cflags`
GLIB_LDFLAGS =  `pkg-config glib-2.0 --libs` -lglibivy -lpcre

ttx2scilab: ttx2scilab.c
	$(CC) -Wall $(GLIB_CFLAGS) -o $@ $< $(GLIB_LDFLAGS)

clean:
	$(Q)rm -f ttx2scilab

