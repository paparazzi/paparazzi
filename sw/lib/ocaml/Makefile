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

INCLUDES= -I +lablgl -I +camlimages -I +lablgtk2
OCAMLC=ocamlc $(INCLUDES)
OCAMLOPT=ocamlopt $(INCLUDES)


SRC = debug.ml env.ml serial.ml ocaml_tools.ml extXml.ml xml2h.ml latlong.ml srtm.ml wavecard.ml geometry_2d.ml geometry_3d.ml cserial.o convert.o ubx.ml pprz.ml
CMO = $(SRC:.ml=.cmo)
CMX = $(SRC:.ml=.cmx)

XSRC = platform.ml gtkgl_Hack.ml ml_gtkgl_hack.o gtk_image.ml gtk_tools_icons.ml gtk_tools.ml gtk_draw.ml gtk_tools_GL.ml gtk_3d.ml mapCanvas.ml mapWaypoints.ml mapTrack.ml
XCMO = $(XSRC:.ml=.cmo)
XCMX = $(XSRC:.ml=.cmx)


all : lib-pprz.cma xlib-pprz.cma xml_get.out lib-pprz.cmxa
# lib-pprz.cmxa xlib-pprz.cmxa


lib-pprz.cma : $(CMO)
	ocamlmklib -o lib-pprz str.cma xml-light.cma unix.cma $^

lib-pprz.cmxa : $(CMX)
	ocamlmklib -o lib-pprz $^

xlib-pprz.cma : $(XCMO)
	ocamlmklib -o xlib-pprz $^

xlib-pprz.cmxa : $(XCMX)
	ocamlmklib -o xlib-pprz $^

xml_get.out : lib-pprz.cma xml_get.cmo
	$(OCAMLC) -o $@ str.cma xml-light.cma -I . $^

ignutm.opt : latlong.cmx ignutm.ml
	$(OCAMLOPT) -o $@ -I +camlimages ci_core.cmxa ci_png.cmxa xml-light.cmxa $^

utm_of.opt : latlong.cmx utm_of.ml
	$(OCAMLOPT) -o $@ $^

GTKCFLAGS := $(shell gtk-config --cflags)

%.o   : %.c
	$(OCAMLC) -c $<

ml_gtkgl_hack.o   : ml_gtkgl_hack.c
	$(OCAMLC) -c -ccopt "$(GTKCFLAGS)" $<

%.cmo : %.ml
	$(OCAMLC) -c $<

%.cmx : %.ml
	$(OCAMLOPT) -c $<

%.cmi : %.mli
	$(OCAMLC) $<

%.cmi : %.ml
	$(OCAMLC) $<

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
