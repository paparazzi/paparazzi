# Hey Emacs, this is a -*- makefile -*-
#
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

# Quiet compilation
Q=@

include ../Makefile.ocaml
INCLUDES=
PKG = -package glibivy,pprz
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz
XPKG = -package pprz.xlib
XLINKPKG = $(XPKG) -linkpkg -dllpath-pkg pprz.xlib

all: play plotter plot sd2log plotprofile openlog2tlm

play : log_file.cmo play_core.cmo play.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) gtkInit.cmo $^

play-nox : play_core.cmo play-nox.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $^

plotter : plotter.cmo $(LIBPPRZCMA) $(XLIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(XLINKPKG) gtkInit.cmo $^

plot : log_file.cmo gtk_export.cmo export.cmo plot.cmo $(LIBPPRZCMA) $(XLIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(XLINKPKG) gtkInit.cmo $^

sd2log : sd2log.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $^

CC = gcc
CFLAGS=-g -O2 -Wall
LDFLAGS=

openlog2tlm: openlog2tlm.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -g -o $@ $^

# Target for bytecode executable (if ocamlopt is not available)
# plot : log_file.cmo gtk_export.cmo export.cmo plot.cmo
#	@echo OL $@
#	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma lablgtk.cma lib-pprz.cma lablglade.cma gtkInit.cmo  $^

%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $(PKG) $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $(PKG) $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -c $(PKG) $<

export.cmo : gtk_export.cmo
export.cmx : gtk_export.cmx

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	MKTEMP = gmktemp
else
	MKTEMP = mktemp
endif

gtk_export.ml : export.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)lablgladecc2 -root export -hide-default $($@_TMP) | grep -B 1000000 "  end" > $@
	$(Q)rm -f $($@_TMP)


pt : ahrsview imuview ahrs2fg

CC = gcc
CFLAGS=-g -O2 -Wall $(shell pkg-config gtk+-2.0 --cflags)
LDFLAGS=$(shell pkg-config gtk+-2.0 --libs) -s -lgtkdatabox -lglibivy

motor_bench : motor_bench.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

ahrsview : ahrsview.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

imuview : imuview.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

plot_roll_loop : plot_roll_loop.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

test_sliding_plot: test_sliding_plot.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)


UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  IVY_C_LIBRARYS = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  IVY_C_INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
endif

MORE_FLAGS = -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include -rdynamic  /usr/lib/libgtkgl.so -L/usr/lib -L/usr/X11R6/lib /usr/lib/libgtk.so /usr/lib/libgdk.so /usr/lib/libgmodule.so /usr/lib/libglib.so -ldl -lXi -lXext -lX11 -lm -lGLU -lGL  -Wl,--rpath -Wl,/usr/local/lib -lglibivy $(shell pcre-config --libs)

MORE_CFLAGS = -DHAVE_DLFCN_H=1 -DSTDC_HEADERS=1  -I. -I.  -I..    -g -O2 -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include

disp3d: disp3d.c
	@echo CC $@
	$(Q)$(CC) $(MORE_CFLAGS) -g -o $@ $^ $(MORE_FLAGS)

plotprofile: plotprofile.c
	@echo CC $@
	$(Q)$(CC) $(IVY_C_LIBRARYS) $(IVY_C_INCLUDES) -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy $(shell pcre-config --libs)

test1: test1.c
	@echo CC $@
	$(Q)$(CC) $(MORE_CFLAGS) -g -o $@ $^ $(MORE_FLAGS) -lglut

test3: test3.c sliding_plot.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)


#FGFS_PREFIX=/home/poine/local
FGFS_PREFIX=/home/poine/flightgear
#FGFS_PREFIX=/usr/local

#FGFS_ROOT = /home/poine/local
FGFS_ROOT = /home/poine/flightgear
FGFS = $(FGFS_PREFIX)/bin/fgfs
#FGFS = /usr/games/fgfs
#FGFS_ENV = LD_LIBRARY_PATH=/usr/local/lib:$(FGFS_ROOT)/lib
FGFS_ENV = LD_LIBRARY_PATH=$(FGFS_ROOT)/lib
FGFS_COMMON_ARGS = --fg-root=$(FGFS_ROOT)  --aircraft=A320 --timeofday=noon
#FGFS_COMMON_ARGS = --aircraft=737-300 --timeofday=noon
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

test_samere: test_samere.c network.c flight_gear.c utils.c
	$(CC) $(CFLAGS) -I../airborne -I../airborne/test -I../include -g -o $@ $^ $(LDFLAGS)

ivy_example: ivy_example.c
	$(CC) $(IVY_C_LIBRARYS) $(IVY_C_INCLUDES) -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy

tmclient: tmclient.c
	$(CC) $(IVY_C_LIBRARYS) $(IVY_C_INCLUDES) -g -O1 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy

ffjoystick: ffjoystick.c
	$(CC) -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy -lm

ctrlstick: ctrlstick.c
	$(CC) -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy

clean:
	$(Q)rm -f *.opt *.out *~ core *.o *.bak .depend *.cm* play ahrsview imuview ahrs2fg plot plotter gtk_export.ml openlog2tlm disp3d plotprofile test1 test3 test_samere ivy_example tmclient ffjoystick ctrlstick sd2log motor_bench

.PHONY: all clean pt run_fg

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
