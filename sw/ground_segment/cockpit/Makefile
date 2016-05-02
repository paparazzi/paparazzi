# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

OCAMLCFLAGS=-thread -ccopt -fPIC

include ../../Makefile.ocaml

INCLUDES=
LIBS=
LIBSX=$(LIBS:.cma=.cmxa)

INCLUDES=
PKG = -package pprzlink,pprz.xlib
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz.xlib,pprzlink

LABLGTK2INIT = $(shell ocamlfind query -p-format lablgtk2.init 2>/dev/null)
ifeq ($(LABLGTK2INIT),)
LABLGTK2INIT = $(shell ocamlfind query -p-format lablgtk2.auto-init 2>/dev/null)
endif

LABLGTK2_VER := $(shell ocamlfind query -format '%v' lablgtk2)
# major.minor version
LABLGTK2_MM := $(shell echo $(LABLGTK2_VER) | cut -f1,2 -d.)
#LABLGTK2_NATIVE_WINDOW := $(shell echo '$(LABLGTK2_MM)>=2.18' | bc)
ifeq ($(shell echo '$(LABLGTK2_MM)>=2.18' | bc),1)
CAMLP4_DEFS = -DGDK_NATIVE_WINDOW
endif
# workaround wrong reported version of lablgtk
# see https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=822949 and https://bugs.launchpad.net/ubuntu/+source/lablgtk2/+bug/1577236
LABLGTK2_DEB := $(shell LC_ALL=C apt-cache policy liblablgtk2-ocaml | grep Installed | awk '{print $$2}' 2>/dev/null)
ifneq (,$(findstring 2.18.3+dfsg-1, $(LABLGTK2_DEB)))
CAMLP4_DEFS = -DGDK_NATIVE_WINDOW
endif
CAMLP4_DEFS ?=
PP_OPTS = -pp "camlp4o pa_macro.cmo $(CAMLP4_DEFS)"

# which source files to run through camlp4
PP_SRC = gcs.ml
PP_CMO = $(PP_SRC:.ml=.cmo)
PP_CMX = $(PP_SRC:.ml=.cmx)

ML= gtk_setting_time.ml gtk_strip.ml horizon.ml strip.ml gtk_save_settings.ml saveSettings.ml page_settings.ml pages.ml speech.ml plugin.ml sectors.ml map2d.ml editFP.ml intruders.ml live.ml particules.ml papgets.ml gcs.ml
MAIN=gcs
CMO=$(ML:.ml=.cmo)
CMX=$(ML:.ml=.cmx)

all : $(MAIN)

opt : $(MAIN).opt

$(MAIN) : $(CMO) $(LIBPPRZCMA) $(LIBPPRZLINKCMA) $(XLIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(LIBS) $(LINKPKG) $(CMO) -o $@

$(MAIN).opt : $(CMX) $(LIBPPRZCMXA) $(LIBPPRZLINKCMXA) $(XLIBPPRZCMXA)
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) $(LIBSX) -package pprz.xlib,$(LABLGTK2INIT) -linkpkg $(CMX) -o $@


# bit of a hack: only run some specifc files through caml4p
$(PP_CMO): $(PP_SRC)
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(PP_OPTS) $(PKG) -c $<
$(PP_CMX): $(PP_SRC)
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) $(PP_OPTS) $(PKG) -c $<

%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(PKG) -c $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(PKG) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) $(PKG) -c $<

saveSettings.cmo : gtk_save_settings.cmo
saveSettings.cmx: gtk_save_settings.cmx

gtk_strip.ml : gcs.glade
	@echo GLADE $@
	$(Q)lablgladecc2 -root eventbox_strip -hide-default $< | grep -B 1000000 "  end" > $@

gtk_setting_time.ml : gcs.glade
	@echo GLADE $@
	$(Q)lablgladecc2 -root setting_time -hide-default $< | grep -B 1000000 "  end" > $@

gtk_save_settings.ml : gcs.glade
	@echo GLADE $@
	$(Q)lablgladecc2 -root save_settings -hide-default $< | grep -B 1000000 "  end" > $@

strip.cmo : gtk_strip.cmo gtk_setting_time.cmo

compass : compass.ml
	@echo OL $@
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(LINKPKG) gtkInit.cmo $^ -o $@


clean:
	$(Q)rm -f *~* *.cm* *.o *.out *.opt map2d gcs .depend gtk_strip.ml gtk_setting_time.ml gtk_save_settings.ml compass ant_track ant_track_pmm actuators

.PHONY: all opt clean


CC = gcc
CFLAGS=-g -O2 -Wall $(shell pkg-config gtk+-2.0 --cflags) -fPIC
LDFLAGS=$(shell pkg-config gtk+-2.0 --libs) -s -lglibivy -lm -lpcre


ant_track : ant_track.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

ant_track_pmm : ant_track_pmm.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

actuators : actuators.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)


#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) $(PP_OPTS) $(PP_SRC) > .depend
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) $(filter-out $(PP_SRC), $(ML)) *.mli >> .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
