# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2010-2012  The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# Quiet compilation
# Launch with "make Q=''" to get full command display
Q=@

CC = gcc

PAPARAZZI_SRC=../../..
UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  LIBRARYS = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
else
  LIBRARYS = -s
endif

CFLAGS += -Wall -fPIC
GTK_CFLAGS  = $(shell pkg-config gtk+-2.0 --cflags)
GTK_LDFLAGS = $(shell pkg-config gtk+-2.0 --libs) $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs) -fPIC

# fallback to ivy-glib pkg-config info if there is no ivy-c.pc
IVY_INC = $(shell pkg-config --cflags-only-I ivy-c 2> /dev/null)
IVY_LDFLAGS = $(shell pkg-config --libs ivy-c 2> /dev/null)
ifeq ($(strip $(IVY_INC)),)
IVY_INC = $(shell pkg-config --cflags-only-I ivy-glib)
IVY_LDFLAGS = $(shell pkg-config --libs-only-L ivy-glib) -livy
endif

# Optitrack specific librarys and includes
NATNET_LIBRARYS = $(shell pkg-config glib-2.0 --libs) -lglibivy -lm $(shell pcre-config --libs)

INCLUDES += $(shell pkg-config glib-2.0 --cflags) -I$(PAPARAZZI_SRC)/sw/airborne/ -I$(PAPARAZZI_SRC)/sw/include/ $(IVY_INC)

all: davis2ivy kestrel2ivy natnet2ivy video_synchronizer

clean:
	$(Q)rm -f *.o davis2ivy kestrel2ivy natnet2ivy video_synchronizer

davis2ivy: davis2ivy.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o davis2ivy davis2ivy.o $(LIBRARYS) $(IVY_LDFLAGS)

kestrel2ivy: kestrel2ivy.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o kestrel2ivy kestrel2ivy.o $(LIBRARYS) $(IVY_LDFLAGS)

natnet2ivy: natnet2ivy.o pprz_geodetic_double.o pprz_algebra_double.o fms_network.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o natnet2ivy natnet2ivy.o pprz_geodetic_double.o pprz_algebra_double.o fms_network.o $(LIBRARYS) $(NATNET_LIBRARYS) $(IVY_LDFLAGS)

video_synchronizer: video_synchronizer.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) $(GTK_CFLAGS) $(LIBRARYS) $(INCLUDES) -o $@ $< $(GTK_LDFLAGS)

pprz_algebra_double.o : $(PAPARAZZI_SRC)/sw/airborne/math/pprz_algebra_double.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

pprz_geodetic_double.o : $(PAPARAZZI_SRC)/sw/airborne/math/pprz_geodetic_double.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

fms_network.o : $(PAPARAZZI_SRC)/sw/airborne/fms/fms_network.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

%.o : %.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

.PHONY: all clean
