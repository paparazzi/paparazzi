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

Q=@

INCLUDES= -I +xml-light -I +pcre -I +netstring
XINCLUDES= -I +lablgl -I +lablgtk2 -I +xml-light
OCAMLC=ocamlc
OCAMLOPT=ocamlopt
OCAMLLIBDIR=$(shell ocamlc -where)


SRC = debug.ml base64.ml serial.ml ocaml_tools.ml extXml.ml env.ml xml2h.ml latlong.ml egm96.ml srtm.ml http.ml gm.ml iGN.ml geometry_2d.ml cserial.o convert.o ubx.ml pprz.ml xbee.ml logpprz.ml xmlCom.ml editAirframe.ml
CMO = $(SRC:.ml=.cmo)
CMX = $(SRC:.ml=.cmx)

XSRC = gtk_tools.ml platform.ml wind_sock.ml gtk_papget_editor.ml gtk_papget_text_editor.ml gtk_papget_gauge_editor.ml papget_common.ml papget_renderer.ml papget.ml mapCanvas.ml mapWaypoints.ml mapTrack.ml mapGoogle.ml mapIGN.ml ml_gtk_drag.o xmlEdit.ml mapFP.ml
XCMO = $(XSRC:.ml=.cmo)
XCMX = $(XSRC:.ml=.cmx)

TESTS_SRC = test/test_logpprz.ml
TESTS_CMO = $(TESTS_SRC:.ml=.cmo)

$(XCMO) $(XCMX) myGtkInit.cmo : INCLUDES=$(XINCLUDES)

all : lib-pprz.cma xlib-pprz.cma myGtkInit.cmo xml_get.out opt
opt : lib-pprz.cmxa xlib-pprz.cmxa


lib-pprz.cma : $(CMO)
	@echo OL $@
	$(Q)ocamlmklib $(INCLUDES) -o lib-pprz $^

lib-pprz.cmxa : $(CMX)
	@echo OOL $@
	$(Q)ocamlmklib $(INCLUDES) -o lib-pprz $^

xlib-pprz.cma : $(XCMO)
	@echo OL $@
	$(Q)ocamlmklib $(XINCLUDES) -o xlib-pprz $^

xlib-pprz.cmxa : $(XCMX)
	@echo OOL $@
	$(Q)ocamlmklib $(XINCLUDES) -o xlib-pprz $^

xml_get.out : lib-pprz.cma xml_get.cmo
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ str.cma xml-light.cma -I . $^

tests : lib-pprz.cma $(TESTS_CMO)
	$(OCAMLC) $(INCLUDES) -custom -I . -o $@ str.cma unix.cma ivy-ocaml.cma xml-light.cma $^

GTKCFLAGS := -I /usr/lib/gtk-2.0/include -I/usr/include/gtk-2.0 -I/usr/include/atk-1.0 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -I/usr/include/pango-1.0 -I /usr/include/cairo
# GTKCFLAGS := $(shell gtk-config --cflags)

%.o   : %.c
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

ml_gtk_drag.o : ml_gtk_drag.c
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c -ccopt "$(GTKCFLAGS)" $<

ml_gtkgl_hack.o   : ml_gtkgl_hack.c
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c -ccopt "$(GTKCFLAGS)" $<

camltm.o : register_example.cmo
	$(OCAMLC) $(INCLUDES) -output-obj -o $@ unix.cma str.cma xml-light.cma ivy-ocaml.cma debug.cmo serial.cmo extXml.cmo env.cmo pprz.cmo tm.cmo

caml_from_c_example : cserial.o convert.o caml_from_c_example.o camltm.o
	$(CC) -o $@ $^ -L$(OCAMLLIBDIR) -lunix -lstr -livy-ocaml -lcamlrun -lm -livy -lcurses

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(XINCLUDES) $(INCLUDES) $<

%.cmi : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(XINCLUDES) $<

gtk_papget_editor.ml : widgets.glade
	lablgladecc2 -root papget_editor -hide-default $< | grep -B 1000000 "  end" > $@

gtk_papget_text_editor.ml : widgets.glade
	lablgladecc2 -root table_text_editor -hide-default $< | grep -B 1000000 "  end" > $@

gtk_papget_gauge_editor.ml : widgets.glade
	lablgladecc2 -root table_gauge_editor -hide-default $< | grep -B 1000000 "  end" > $@

clean :
	rm -f *~ *.cm* *.out *.opt .depend *.a *.o *.so tests gtk_papget_*.ml


#
# Dependencies
#

.depend: Makefile
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
