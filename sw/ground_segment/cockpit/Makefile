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
INCLUDES=-I +lablgtk2 -I ../../lib/ocaml -I +xml-light
LIBS=glibivy-ocaml.cma lablgtk.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)

ML= horizon.ml strip.ml pages.ml speech.ml plugin.ml sectors.ml map2d.ml editFP.ml live.ml gcs.ml
MAIN=gcs
CMO=$(ML:.ml=.cmo)
CMX=$(ML:.ml=.cmx)

all : $(MAIN)

opt : $(MAIN).opt

$(MAIN) : $(CMO)
	@echo OL $@
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) -custom $(INCLUDES) unix.cma str.cma xml-light.cma $(LIBS) threads.cma gtkThread.cmo gtkInit.cmo $(CMO) -o $@

$(MAIN).opt : $(CMX)
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) str.cmxa unix.cmxa xml-light.cmxa $(LIBS:.cma=.cmxa) threads.cmxa gtkThread.cmx gtkInit.cmx $(CMX) -o $@


%.cmo: %.ml  ../../lib/ocaml/lib-pprz.cma ../../lib/ocaml/xlib-pprz.cma
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $<
%.cmi: %.mli  ../../lib/ocaml/lib-pprz.cma ../../lib/ocaml/xlib-pprz.cma
	@echo OCI $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -c $<


clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d gcs .depend


#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
