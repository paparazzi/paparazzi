

CFLAGS = -static
LINK   = -lpthread
CFILES = serial.c \
	 chdk_pipe.c \
	 socket.c \
	 protocol.c
HFILES = $(CFILES:.c=.h)


all:	catia

catia:	$(CFILES) $(HFILES) catia.c
	gcc $(CFLAGS) $(CFILES) catia.c -o catia $(LINK)

clean:
	rm -rf ./catia
