UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  LIBRARYS = -L/opt/local/lib
else 
  LIBRARYS = -s 
endif


all: davis2ivy

davis2ivy: davis2ivy.o
	g++ -o davis2ivy davis2ivy.o  $(LIBRARYS)  -livy

%.o : %.c
	gcc -c -O2 -Wall -I /opt/local/include/  $<
