all : ant_track_Pololu.c
	gcc `pkg-config --cflags --libs gtk+-2.0` -I/usr/include/Ivy/\
	    -livy ant_track_Pololu.c -o ant_track
clean:
	rm ant_track

