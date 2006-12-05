# 
#   $Id$
#   Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
INCLUDES= -I +xml-light

all: play

play : play.ml
	$(OCAMLC) $(INCLUDES) -custom -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma -I +lablgtk2 -I ../lib/ocaml lablgtk.cma lib-pprz.cma gtkInit.cmo  $^




pt : ahrsview imuview ahrs2fg

CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config gtk+-2.0 --cflags`
LDFLAGS=`pkg-config gtk+-2.0 --libs` -s -lgtkdatabox `pcre-config --libs` -lglibivy

ahrsview : ahrsview.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

imuview : imuview.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

test_sliding_plot: test_sliding_plot.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

#MORE_FLAGS = -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include -rdynamic  /home/poine/tmp/gtkglarea-1.2.3/gtkgl/.libs/libgtkgl.so -L/usr/lib -L/usr/X11R6/lib /usr/lib/libgtk.so /usr/lib/libgdk.so /usr/lib/libgmodule.so /usr/lib/libglib.so -ldl -lXi -lXext -lX11 -lm -lGLU -lGL  -Wl,--rpath -Wl,/usr/local/lib
MORE_FLAGS = -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include -rdynamic  /usr/lib/libgtkgl.so -L/usr/lib -L/usr/X11R6/lib /usr/lib/libgtk.so /usr/lib/libgdk.so /usr/lib/libgmodule.so /usr/lib/libglib.so -ldl -lXi -lXext -lX11 -lm -lGLU -lGL  -Wl,--rpath -Wl,/usr/local/lib `pcre-config --libs` -lglibivy
 
MORE_CFLAGS = -DHAVE_DLFCN_H=1 -DSTDC_HEADERS=1  -I. -I.  -I..    -g -O2 -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include

disp3d: disp3d.c
	$(CC) $(MORE_CFLAGS) -g -o $@ $^ $(MORE_FLAGS)

test1: test1.c
	$(CC) $(MORE_CFLAGS) -g -o $@ $^ $(MORE_FLAGS) -lglut


clean:
	rm -f *.opt *.out *~ core *.o *.bak .depend *.cm* play ahrsview imuview ahrs2fg

FGFS_ROOT = /home/poine/work/flightgear_cvs
#FGFS = $(FGFS_ROOT)/bin/fgfs
FGFS = fgfs
FGFS_ENV = LD_LIBRARY_PATH=/usr/local/lib:$(FGFS_ROOT)/lib
#FGFS_COMMON_ARGS = --fg-root=$(FGFS_ROOT)  --aircraft=A320 --timeofday=noon
FGFS_COMMON_ARGS = --aircraft=737-300 --timeofday=noon
FGFS_IN_FDM_ARGS = $(FGFS_COMMON_ARGS) --fdm=null --native-fdm=socket,in,30,,5501,udp
FGFS_OUT_FDM_ARGS = $(FGFS_COMMON_ARGS) --native-fdm=socket,out,30,,5500,udp
FGFS_IN_GUI_ARGS = $(FGFS_COMMON_ARGS) --fdm=null --native-gui=socket,in,30,,5501,udp
FGFS_OUT_GUI_ARGS = $(FGFS_COMMON_ARGS) --native-gui=socket,out,30,,5500,udp
FGFS_IN_CTRLS_ARGS = $(FGFS_COMMON_ARGS) --native-ctrls=socket,in,30,,5501,udp
FGFS_OUT_CTRLS_ARGS = $(FGFS_COMMON_ARGS) --native-ctrls=socket,out,30,,5500,udp
FGFS_OUT_MULTIPLAY_ARGS = $(FGFS_COMMON_ARGS) --multiplay=out,10,127.0.0.1,5500 --callsign=F-POINE
FGFS_IN_MULTIPLAY_ARGS = $(FGFS_COMMON_ARGS) --multiplay=in,10,127.0.0.1,5501 --callsign=F-POINE
FGFS_RCAM_ARGS =  $(FGFS_COMMON_ARGS) --fdm=null --native-ctrls=socket,out,30,,5500,udp --native-fdm=socket,in,30,,5501,udp
FGFS_GAME_ARGS = $(FGFS_COMMON_ARGS) --control=joystick

FGFS_ARGS = $(FGFS_COMMON_ARGS) $(FGFS_IN_GUI_ARGS)
#FGFS_GAME_ARGS)

run_fg:
	$(FGFS_ENV) $(FGFS) $(FGFS_ARGS)

ahrs2fg: ahrs2fg.c network.c flight_gear.c utils.c 
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)