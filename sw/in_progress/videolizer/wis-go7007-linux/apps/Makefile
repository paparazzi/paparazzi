all: gorecord modet

gorecord: gorecord.c tv-freq.c
	gcc -Wall $(CFLAGS) -o gorecord gorecord.c tv-freq.c

modet: modet.c
	gcc -Wall $(CFLAGS) -o modet modet.c -lncurses

install:
	@echo "No install for apps yet"

clean:
	rm -f gorecord modet *.o
