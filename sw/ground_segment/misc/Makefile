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
  LIBRARYS =
endif

CFLAGS += -Wall -fPIC -g
GTK_CFLAGS  = $(shell pkg-config gtk+-2.0 --cflags)
GTK_LDFLAGS = $(shell pkg-config gtk+-2.0 --libs) $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs) -fPIC

GLIBIVY_CFLAGS = $(shell pkg-config --cflags ivy-glib)
GLIBIVY_LDFLAGS = $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs)

# fallback to ivy-glib pkg-config info if there is no ivy-c.pc
IVY_INC = $(shell pkg-config --cflags-only-I ivy-c 2> /dev/null)
IVY_LDFLAGS = $(shell pkg-config --libs ivy-c 2> /dev/null)
ifeq ($(strip $(IVY_INC)),)
IVY_INC = $(shell pkg-config --cflags-only-I ivy-glib)
IVY_LDFLAGS = $(shell pkg-config --libs-only-L ivy-glib) -livy
endif

# GLIB libraries
GLIB_LDFLAGS = $(shell pkg-config glib-2.0 --libs) -lglibivy -lm $(shell pcre-config --libs)

# Paparazzi includes
INCLUDES += $(shell pkg-config glib-2.0 --cflags) -I$(PAPARAZZI_SRC)/sw/airborne/ -I$(PAPARAZZI_SRC)/sw/include/ $(IVY_INC)
INCLUDES += -I$(PAPARAZZI_SRC)/sw/ext/libsbp/c/include/ -I$(PAPARAZZI_SRC)/sw/airborne/arch/linux/

all: davis2ivy kestrel2ivy natnet2ivy sbp2ivy video_synchronizer sbs2ivy

clean:
	$(Q)rm -f *.o davis2ivy kestrel2ivy natnet2ivy sbp2ivy video_synchronizer sbs2ivy

davis2ivy: davis2ivy.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o $@ $^ $(LIBRARYS) $(IVY_LDFLAGS)

kestrel2ivy: kestrel2ivy.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o $@ $^ $(LIBRARYS) $(IVY_LDFLAGS)

natnet2ivy: natnet2ivy.o pprz_geodetic_double.o pprz_algebra_double.o udp_socket.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o $@ $^ $(LIBRARYS) $(GLIB_LDFLAGS) $(IVY_LDFLAGS)

sbp2ivy: sbp2ivy.o serial_port.o sbp.o edc.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o $@ $^ $(INCLUDES) $(LIBRARYS) $(GLIB_LDFLAGS) $(IVY_LDFLAGS)

video_synchronizer: video_synchronizer.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) $(GTK_CFLAGS) $(LIBRARYS) $(INCLUDES) -o $@ $< $(GTK_LDFLAGS)

sbs2ivy: sbs2ivy.o pprz_geodetic_double.o pprz_geodetic_float.o sbs_parser.o
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) $(GTK_CFLAGS) -o $@ $^ $(LIBRARYS) $(GLIBIVY_LDFLAGS) $(GTK_LDFLAGS) -lm

pprz_algebra_double.o : $(PAPARAZZI_SRC)/sw/airborne/math/pprz_algebra_double.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

pprz_geodetic_double.o : $(PAPARAZZI_SRC)/sw/airborne/math/pprz_geodetic_double.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

pprz_geodetic_float.o : $(PAPARAZZI_SRC)/sw/airborne/math/pprz_geodetic_float.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

udp_socket.o : $(PAPARAZZI_SRC)/sw/airborne/arch/linux/udp_socket.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

serial_port.o : $(PAPARAZZI_SRC)/sw/airborne/arch/linux/serial_port.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

sbp.o : $(PAPARAZZI_SRC)/sw/ext/libsbp/c/src/sbp.c
	$(Q)$(CC) $(CFLAGS) -c -std=c99 -O2 -Wall $(INCLUDES) $<

edc.o : $(PAPARAZZI_SRC)/sw/ext/libsbp/c/src/edc.c
	$(Q)$(CC) $(CFLAGS) -c -std=c99 -O2 -Wall $(INCLUDES) $<

sbs2ivy.o : sbs2ivy.c
	$(Q)$(CC) $(CFLAGS) $(GTK_CFLAGS) -c -std=gnu99 -O2 -Wall $(INCLUDES) $<

sbs_parser.o : sbs_parser.c
	$(Q)$(CC) $(CFLAGS) -c -std=gnu99 -O2 -Wall $(INCLUDES) $<

%.o : %.c
	$(Q)$(CC) $(CFLAGS) -c -O2 -Wall $(INCLUDES) $<

.PHONY: all clean
