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

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  FPIC = -ccopt -fPIC
else
  FPIC =
endif

INCLUDES=  $(shell ocamlfind query -r -i-format xml-light)  $(shell ocamlfind query -r -i-format netstring) $(shell ocamlfind query -r -i-format pcre)
XINCLUDES= $(shell ocamlfind query -r -i-format lablgtk2) $(shell ocamlfind query -r -i-format xml-light)
OCAMLC=ocamlc
OCAMLOPT=ocamlopt
OCAMLLEX=ocamllex
OCAMLYACC=ocamlyacc
OCAMLLIBDIR=$(shell $(OCAMLC) -where)


SRC = fig.ml debug.ml base64.ml serial.ml ocaml_tools.ml expr_syntax.ml expr_parser.ml expr_lexer.ml extXml.ml env.ml xml2h.ml latlong.ml egm96.ml srtm.ml http.ml maps_support.ml gm.ml iGN.ml geometry_2d.ml cserial.o convert.o ubx.ml pprz.ml xbee.ml logpprz.ml xmlCom.ml os_calls.ml editAirframe.ml defivybus.ml
CMO = $(SRC:.ml=.cmo)
CMX = $(SRC:.ml=.cmx)

XSRC = gtk_tools.ml platform.ml wind_sock.ml gtk_papget_editor.ml gtk_papget_text_editor.ml gtk_papget_gauge_editor.ml gtk_papget_led_editor.ml papget_common.ml papget_renderer.ml papget.ml mapCanvas.ml mapWaypoints.ml mapTrack.ml mapGoogle.ml mapIGN.ml ml_gtk_drag.o xmlEdit.ml mapFP.ml
XCMO = $(XSRC:.ml=.cmo)
XCMX = $(XSRC:.ml=.cmx)

TESTS_SRC = test/test_latlong.ml
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

GTKCFLAGS := `pkg-config --cflags gtk+-2.0`
#GTKCFLAGS := -I/usr/lib/gtk-2.0/include -I/usr/include/gtk-2.0 -I/usr/include/atk-1.0 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -I/usr/include/pango-1.0 -I/usr/include/cairo
# GTKCFLAGS := $(shell gtk-config --cflags)

%.o   : %.c
	@echo OC $<
	$(Q)$(OCAMLC) $(FPIC) $(INCLUDES) -c $<

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
	$(Q)$(OCAMLC) $(XINCLUDES) -c $<

%.ml : %.mll
	@echo OCL $<
	$(Q)$(OCAMLLEX) $<

%.ml %.mli : %.mly
	@echo OCY $<
	$(Q)$(OCAMLYACC) $<

expr_parser.cmo expr_parser.cmx : expr_parser.cmi expr_syntax.cmi
expr_parser.cmi : expr_parser.ml expr_syntax.cmi
expr_lexer.ml : expr_parser.ml
expr_lexer.cmi : expr_lexer.ml expr_syntax.cmi
expr_lexer.cmo : expr_lexer.cmi
expr_syntax.cmo : expr_syntax.cmi


gtk_papget_editor.ml : widgets.glade
	grep -v invisible_char $< > /tmp/$@_$<
	lablgladecc2 -root papget_editor -hide-default /tmp/$@_$< | grep -B 1000000 "  end" > $@
	@rm /tmp/$@_$<

gtk_papget_text_editor.ml : widgets.glade
	grep -v invisible_char $< > /tmp/$@_$<
	lablgladecc2 -root table_text_editor -hide-default /tmp/$@_$< | grep -B 1000000 "  end" > $@
	@rm /tmp/$@_$<

gtk_papget_gauge_editor.ml : widgets.glade
	grep -v invisible_char $< > /tmp/$@_$<
	lablgladecc2 -root table_gauge_editor -hide-default /tmp/$@_$< | grep -B 1000000 "  end" > $@
	@rm /tmp/$@_$<

gtk_papget_led_editor.ml : widgets.glade
	grep -v invisible_char $< > /tmp/$@_$<
	lablgladecc2 -root table_led_editor -hide-default /tmp/$@_$< | grep -B 1000000 "  end" > $@
	@rm /tmp/$@_$<

clean :
	rm -f *~ *.cm* *.out *.opt .depend *.a *.o *.so tests gtk_papget_*.ml expr_parser.ml expr_parser.mli expr_lexer.ml expr_lexer.mli

.PHONY: all opt clean

#
# Dependencies
#

.depend: Makefile
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
