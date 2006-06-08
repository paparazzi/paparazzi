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
INCLUDES=-I +lablgtk2 -I ../../lib/ocaml
LIBS=glibivy-ocaml.cma lablgtk.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)

ML=horizon.ml pages.ml map2d.ml
CMO=$(ML:.ml=.cmo)

all : map2d

opt : map2d.opt


map2d : $(CMO)
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) $(LIBS) threads.cma gtkThread.cmo gtkInit.cmo $(CMO) -o $@

map2d.opt : map2d.cmx
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(INCLUDES) str.cmxa unix.cmxa xml-light.cmxa $(LIBS:.cma=.cmxa) threads.cmxa gtkThread.cmx gtkInit.cmx $< -o $@


%.cmo: %.ml  ../../lib/ocaml/lib-pprz.cma ../../lib/ocaml/xlib-pprz.cma
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

map2d.cmo : pages.cmi
pages.cmo : pages.cmi horizon.cmi

clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d
