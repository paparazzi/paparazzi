# Quiet compilation
Q=@

all: email2udp udp2tcp tcp2ivy

email2udp: email2udp.c
	gcc -g -O2 -Wall -o $@ $^
udp2tcp: udp2tcp.c
	gcc -g -O2 -Wall -o $@ $^
tcp2ivy: tcp2ivy.c
	gcc -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy -lm
tcp2ivy_generic: tcp2ivy_generic.c
	gcc -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -I../../../var/aircrafts/${AIRCRAFT} -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy -lm

clean:
	$(Q)rm -f email2udp udp2tcp tcp2ivy
