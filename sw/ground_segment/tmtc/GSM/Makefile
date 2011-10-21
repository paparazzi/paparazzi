all: SMS_GS

SMS_GS: SMS_Ground_UDtest_final.c
	gcc -g -O2 -Wall `pkg-config --cflags glib-2.0 gtk+-2.0` -L/usr/lib -lglibivy -o SMS_GS SMS_Ground_UDtest_final.c `pkg-config --libs glib-2.0 gtk+-2.0` -lglibivy

clean:
	rm SMS_GS
