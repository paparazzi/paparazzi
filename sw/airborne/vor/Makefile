all: vor_test_samples.h vor_filter_params.c

CFLAGS = -Wall -I.. -DFLOAT_T=float
LDFLAGS = -lm

VOR_LF_SRCS = main.c \
              vor_lf_filter_params.c \
	      vor_lf_filter.c \


vor_lf: $(VOR_LF_SRCS) 
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

gen_test_samples: gen_test_samples.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS) -lsndfile

vor_test_samples.h: gen_test_samples
	./$^  > $@

vor_filter_params.c:
	scilab -nw -nogui -nwni -f gen_filter_params.sce

test_filter: test_filter.c vor_filter.c vor_filter_params.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

play_vor: play_vor.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)  -lsndfile -lasound

play_audio: sndfile-play.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)  -lsndfile -lasound

clean:
	rm -f vor_lf              \
              gen_test_samples    \
              vor_test_samples.h  \
              vor_filter_params.c \
              vor_filter_params.h \
              test_filter *~