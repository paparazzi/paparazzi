UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  LIBRARYS = -L /opt/paparazzi/lib -L /opt/local/lib
  INCLUDES = -I /opt/paparazzi/include -I /opt/local/include/
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
