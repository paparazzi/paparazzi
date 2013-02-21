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

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
   FPIC=-fPIC
   OCAMLCFLAGS=-thread -ccopt -fPIC
else
   FPIC=
   OCAMLCFLAGS=-thread
endif
FPIC=-fPIC

include ../../Makefile.ocaml

INCLUDES=
LIBS= lablglade.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)

INCLUDES= -I $(LIBPPRZDIR) -I ../multimon
PKGCOMMON=unix,str,lablgtk2,ivy,xml-light,netstring,netclient,threads

ML= gtk_setting_time.ml gtk_strip.ml horizon.ml strip.ml gtk_save_settings.ml saveSettings.ml page_settings.ml pages.ml speech.ml plugin.ml sectors.ml map2d.ml editFP.ml live.ml particules.ml papgets.ml gcs.ml
MAIN=gcs
CMO=$(ML:.ml=.cmo)
CMX=$(ML:.ml=.cmx)

all : $(MAIN)

opt : $(MAIN).opt

$(MAIN) : $(CMO) $(XLIBPPRZCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -package $(PKGCOMMON) -linkpkg $(LIBS) gtkThread.cmo myGtkInit.cmo $(CMO) -o $@

$(MAIN).opt : $(CMX)
	@echo OOL $@
	$(Q)$(OCAMLFIND) $(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -package $(PKGCOMMON) -linkpkg $(LIBS:.cma=.cmxa) gtkThread.cmx gtkInit.cmx $(CMX) -o $@


%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLFIND) $(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -package $(PKGCOMMON) -c $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLFIND) $(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -package $(PKGCOMMON) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLFIND) $(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -package $(PKGCOMMON) -c $<

saveSettings.cmo : gtk_save_settings.cmo
saveSettings.cmx: gtk_save_settings.cmx

gtk_strip.ml : gcs.glade
	lablgladecc2 -root eventbox_strip -hide-default $< | grep -B 1000000 "  end" > $@

gtk_setting_time.ml : gcs.glade
	lablgladecc2 -root setting_time -hide-default $< | grep -B 1000000 "  end" > $@

gtk_save_settings.ml : gcs.glade
	lablgladecc2 -root save_settings -hide-default $< | grep -B 1000000 "  end" > $@

strip.cmo : gtk_strip.cmo gtk_setting_time.cmo

compass : compass.ml
	$(OCAMLC) -custom $(OCAMLCFLAGS) $(INCLUDES) unix.cma str.cma xml-light.cma $(LIBS) gtkInit.cmo $^ -o $@


clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d gcs .depend gtk_strip.ml gtk_setting_time.ml gtk_save_settings.ml compass ant_track ant_track_pmm test_enose actuators

.PHONY: all opt clean


CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config gtk+-2.0 --cflags` $(FPIC)
LDFLAGS=`pkg-config gtk+-2.0 --libs` -s -lglibivy


ant_track : ant_track.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

ant_track_pmm : ant_track_pmm.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

test_enose : test_enose.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS) -lgtkdatabox

actuators : actuators.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)


#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
