# 
#   $Id$
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

OCAMLC=ocamlc
OCAMLCFLAGS=-thread
OCAMLOPT=ocamlopt
OCAMLOPTFLAGS=-thread
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)
INCLUDES=-I +lablgtk2 -I ../../lib/ocaml -I +xml-light $(OCAMLNETINCLUDES)
LIBS=$(OCAMLNETCMA) glibivy-ocaml.cma lablgtk.cma lablglade.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)

ML= gtk_setting_time.ml gtk_strip.ml horizon.ml strip.ml gtk_save_settings.ml saveSettings.ml pages.ml speech.ml plugin.ml sectors.ml map2d.ml editFP.ml live.ml particules.ml gcs.ml
MAIN=gcs
CMO=$(ML:.ml=.cmo)
CMX=$(ML:.ml=.cmx)

all : $(MAIN)

opt : $(MAIN).opt

$(MAIN) : $(CMO) ../../lib/ocaml/xlib-pprz.cma ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) -custom $(INCLUDES) unix.cma str.cma xml-light.cma $(LIBS) threads.cma gtkThread.cmo gtkInit.cmo $(CMO) -o $@

$(MAIN).opt : $(CMX)
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) str.cmxa unix.cmxa xml-light.cmxa $(LIBS:.cma=.cmxa) threads.cmxa gtkThread.cmx gtkInit.cmx $(CMX) -o $@


%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -c $<

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
	rm -f *~* *.cm* *.o *.out *.opt map2d gcs .depend gtk_strip.ml gtk_setting_time.ml gtk_save_settings.ml




CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config gtk+-2.0 --cflags`
LDFLAGS=`pkg-config gtk+-2.0 --libs` -s `pcre-config --libs` -lglibivy


ant_track : ant_track.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

test_enose : test_enose.c sliding_plot.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS) -lgtkdatabox 

actuators : actuators.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS) 


#
# Dependencies
#

.depend: Makefile
	ocamldep -I ../../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
