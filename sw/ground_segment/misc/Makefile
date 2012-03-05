UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  LIBRARYS = $(shell if test -d /opt/local/lib; then echo "-L/opt/local/lib"; elif test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; fi)
  INCLUDES = $(shell if test -d /opt/local/include; then echo "-I/opt/local/include"; elif test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; fi)
else
  LIBRARYS = -s
endif


all: davis2ivy kestrel2ivy

clean:
	rm -f *.o davis2ivy kestrel2ivy

davis2ivy: davis2ivy.o
	g++ -o davis2ivy davis2ivy.o  $(LIBRARYS)  -livy

kestrel2ivy: kestrel2ivy.o
	g++ -o kestrel2ivy kestrel2ivy.o $(LIBRARYS) -livy

%.o : %.c
	gcc -c -O2 -Wall $(INCLUDES) $<
