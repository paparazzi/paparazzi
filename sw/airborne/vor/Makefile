all: 

CFLAGS = -Wall -I..
LDFLAGS = -lm

i86_vor_test_float_demod: i86_vor_test_float_demod.c vor_float_demod.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS) -lsndfile

i86_vor_test_int_demod: i86_vor_test_int_demod.c vor_int_demod.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS) -lsndfile

i86_vor_test_filters: i86_vor_test_filters.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS) -lsndfile

vor_filter_params.c:
	scilab -nw -nogui -nwni -f gen_filter_params.sce

play_vor: play_vor.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)  -lsndfile -lasound

play_audio: sndfile-play.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)  -lsndfile -lasound

clean:
	rm -f i86_vor_test_float_demod \
              i86_vor_test_int_demod   \
              i86_vor_test_filters     \
              *~ \#*