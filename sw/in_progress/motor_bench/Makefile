CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config gtk+-2.0 --cflags`
LDFLAGS=`pkg-config gtk+-2.0 --libs` -s `pcre-config --libs` -lglibivy


motor_bench : main.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

# -lgtkdatabox

clean:
	rm *~ motor_bench