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

OCAMLC=ocamlc -thread
OCAMLOPT=ocamlopt -thread
INCLUDES=-I +lablgtk2 -I +camlimages -I ../../lib/ocaml
LIBS=glibivy-ocaml.cma lablgtk.cma ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)


all : map2d

opt : map2d.opt


map2d : map2d.ml ../../lib/ocaml/lib-pprz.cma ../../lib/ocaml/xlib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) $(LIBS) threads.cma gtkThread.cmo gtkInit.cmo $< -o $@

map2d.opt : map2d.cmx
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(INCLUDES) str.cmxa unix.cmxa xml-light.cmxa $(LIBS:.cma=.cmxa) threads.cmxa gtkThread.cmx gtkInit.cmx $< -o $@


.SUFFIXES: .ml .mli .cmo .cmi .cmx

%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -labels -w s -c $<
%.cmi: %.ml
	@echo OCI $<
	$(Q)$(OCAMLC) $(INCLUDES) -labels -w s -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -labels -w s -c $<

clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d
