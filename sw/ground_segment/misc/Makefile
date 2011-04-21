all: davis2ivy

davis2ivy: davis2ivy.o
	g++ -o davis2ivy davis2ivy.o -s -livy

%.o : %.c
	gcc -c -O2 -Wall $<
