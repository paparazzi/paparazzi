#
#   $Id$
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
#

# Quiet compilation
Q=@

CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config gtk+-2.0 --cflags` $(FPIC)
LDFLAGS=`pkg-config gtk+-2.0 --libs` -s `pcre-config --libs` -lglibivy

ant_track_pmm : ant_track_pmm.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

clean:
	rm -f *~* *.cm* *.o *.out *.opt .depend 


