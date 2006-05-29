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

INCLUDES=
XINCLUDES= -I +lablgl -I +lablgtk2
OCAMLC=ocamlc $(INCLUDES)
OCAMLOPT=ocamlopt $(INCLUDES)


SRC = debug.ml env.ml serial.ml ocaml_tools.ml extXml.ml xml2h.ml latlong.ml srtm.ml http.ml gm.ml iGN.ml wavecard.ml geometry_2d.ml geometry_3d.ml cserial.o convert.o ubx.ml pprz.ml xbee.ml
CMO = $(SRC:.ml=.cmo)
CMX = $(SRC:.ml=.cmx)

XSRC = platform.ml gtkgl_Hack.ml ml_gtkgl_hack.o gtk_tools_icons.ml gtk_tools.ml gtk_draw.ml gtk_tools_GL.ml gtk_3d.ml mapCanvas.ml mapWaypoints.ml mapTrack.ml mapGoogle.ml mapIGN.ml ml_gtk_drag.o xmlEdit.ml mapFP.ml
XCMO = $(XSRC:.ml=.cmo)
XCMX = $(XSRC:.ml=.cmx)

$(XCMO) $(XCMX) : INCLUDES=$(XINCLUDES)

all : lib-pprz.cma xlib-pprz.cma xml_get.out
opt : all lib-pprz.cmxa xlib-pprz.cmxa


lib-pprz.cma : $(CMO)
	@echo OL $@
	$(Q)ocamlmklib -o lib-pprz str.cma unix.cma xml-light.cma $^

lib-pprz.cmxa : $(CMX)
	@echo OOL $@
	$(Q)ocamlmklib -o lib-pprz $^

xlib-pprz.cma : $(XCMO)
	@echo OL $@
	$(Q)ocamlmklib -o xlib-pprz $^

xlib-pprz.cmxa : $(XCMX)
	@echo OOL $@
	$(Q)ocamlmklib -o xlib-pprz $^

xml_get.out : lib-pprz.cma xml_get.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -o $@ str.cma xml-light.cma -I . $^

GTKCFLAGS := -I /usr/lib/gtk-2.0/include -I/usr/include/gtk-2.0 -I/usr/include/atk-1.0 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -I/usr/include/pango-1.0 -I /usr/include/cairo
# GTKCFLAGS := $(shell gtk-config --cflags)

%.o   : %.c
	@echo OC $<
	$(Q)$(OCAMLC) -c $<

ml_gtk_drag.o : ml_gtk_drag.c
	@echo OC $<
	$(Q)$(OCAMLC) -c -ccopt "$(GTKCFLAGS)" $<

ml_gtkgl_hack.o   : ml_gtkgl_hack.c
	@echo OC $<
	$(Q)$(OCAMLC) -c -ccopt "$(GTKCFLAGS)" $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $<

%.cmi : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $<

clean :
	rm -f *~ *.cm* *.out *.opt .depend *.a *.o *.so


#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
