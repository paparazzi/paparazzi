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

UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  LIBRARYS = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
else
  LIBRARYS = -s
endif


all: davis2ivy kestrel2ivy

clean:
	$(Q)rm -f *.o davis2ivy kestrel2ivy

davis2ivy: davis2ivy.o
	$(Q)$(CC) -o davis2ivy davis2ivy.o  $(LIBRARYS)  -livy

kestrel2ivy: kestrel2ivy.o
	$(Q)$(CC) -o kestrel2ivy kestrel2ivy.o $(LIBRARYS) -livy

%.o : %.c
	$(Q)$(CC) -c -O2 -Wall $(INCLUDES) $<

.PHONY: all clean
