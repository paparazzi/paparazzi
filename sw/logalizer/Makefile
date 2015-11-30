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

all: play plotter logplotter sd2log plotprofile openlog2tlm sdlogger_download

play : log_file.cmo play_core.cmo play.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) gtkInit.cmo $^

play-nox : play_core.cmo play-nox.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $^

plotter : plotter.cmo $(LIBPPRZCMA) $(XLIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(XLINKPKG) gtkInit.cmo $^

logplotter : log_file.cmo gtk_export.cmo export.cmo logplotter.cmo $(LIBPPRZCMA) $(XLIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(XLINKPKG) gtkInit.cmo $^

sd2log : sd2log.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $^

sdlogger_download: sdlogger_download.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -std=gnu99 -o $@ $^

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



CC = gcc
CFLAGS=-g -O2 -Wall

openlog2tlm: openlog2tlm.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -o $@ $^

DISP3D_CFLAGS = $(shell pkg-config --cflags ivy-glib gtk+-2.0 gtkgl-2.0)
DISP3D_LDFLAGS = $(shell pkg-config --libs ivy-glib gtk+-2.0 gtkgl-2.0) $(shell pcre-config --libs)

disp3d: disp3d.c
	@echo CC $@
	$(Q)$(CC)  $(CFLAGS) $(DISP3D_CFLAGS) -o $@ $^ $(DISP3D_LDFLAGS)


IVY_C_LIBS = $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs)
GLIBIVY_CFLAGS = $(shell pkg-config --cflags ivy-glib)

#
# try to find include locations on OSX
#
UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
  IVY_C_LIB_INC = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  IVY_C_INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
  GLIBIVY_CFLAGS += $(IVY_C_LIB_INC) $(IVY_C_INCLUDES)
endif


plotprofile: plotprofile.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) $(GLIBIVY_CFLAGS) -o $@ $^ $(IVY_C_LIBS)

ahrs2fg: ahrs2fg.c network.c flight_gear.c utils.c
	$(CC) $(CFLAGS) $(GLIBIVY_CFLAGS) -o $@ $^ $(IVY_C_LIBS) -lm


tmclient: tmclient.c
	$(CC) -g -O1 -Wall $(GLIBIVY_CFLAGS) -o $@ $^ $(IVY_C_LIBS)

ffjoystick: ffjoystick.c
	$(CC) -g -O2 -Wall $(GLIBIVY_CFLAGS) -o $@ $^ $(IVY_C_LIBS) -lm

ctrlstick: ctrlstick.c
	$(CC) -g -O2 -Wall $(GLIBIVY_CFLAGS) -o $@ $^ $(IVY_C_LIBS)


clean:
	$(Q)rm -f *.opt *.out *~ core *.o *.bak .depend *.cm* play ahrs2fg logplotter plotter gtk_export.ml openlog2tlm disp3d plotprofile tmclient ffjoystick ctrlstick sd2log sdlogger_download

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
