#
#   $Id: Makefile 5874 2010-09-15 03:41:04Z aibara $
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

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
INCLUDES= $(shell ocamlfind query -r -i-format xml-light) $(shell ocamlfind query -r -i-format lablgtk2) -I ../../lib/ocaml

all: panic

$^


play play-nox plotter sd2log : ../../lib/ocaml/lib-pprz.cma
plot : ../../lib/ocaml/lib-pprz.cmxa

# Target for bytecode executable (if ocamlopt is not available)
# plot : log_file.cmo gtk_export.cmo export.cmo plot.cmo
#	@echo OL $@
#	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma lablgtk.cma lib-pprz.cma lablglade.cma gtkInit.cmo  $^

%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -c $<

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

MORE_FLAGS = -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include -rdynamic  /usr/lib/libgtkgl.so -L/usr/lib -L/usr/X11R6/lib /usr/lib/libgtk.so /usr/lib/libgdk.so /usr/lib/libgmodule.so /usr/lib/libglib.so -ldl -lXi -lXext -lX11 -lm -lGLU -lGL  -Wl,--rpath -Wl,/usr/local/lib -lglibivy $(shell pcre-config --libs)

MORE_CFLAGS = -DHAVE_DLFCN_H=1 -DSTDC_HEADERS=1  -I. -I.  -I..    -g -O2 -I/usr/include/gtk-1.2 -I/usr/include/glib-1.2 -I/usr/lib/glib/include

clean:
	$(Q)rm -f *.opt *.out *~ core *.o *.bak .depend *.cm* panic

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

panic: panic.c
	gcc -g -O2 -Wall $(shell pkg-config glib-2.0 --cflags) -o $@ $^ $(shell pkg-config glib-2.0 --libs) -lglibivy -lhid

#
# Dependencies
#

.depend: Makefile
	ocamldep -I ../../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
