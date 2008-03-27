all: sample_sound.h vor_lf

CFLAGS = -Wall -I.. -DFLOAT_T=float
LDFLAGS = -lm

VOR_LF_SRCS = main.c \
              vor_lf_filter.c \

vor_lf: $(VOR_LF_SRCS) 
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

gen_header: gen_header.c
	gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)  -lsndfile

sample_sound.h: gen_header
	./gen_header  > sample_sound.h

clean:
	rm -f vor_lf gen_header sample_sound.h *~