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

include ../../../conf/Makefile.local

all: wind

INCLUDES= -I ../../lib/ocaml -I +lablgtk2

OCAMLC= ocamlc $(INCLUDES)
OCAMLMLI= ocamlc $(INCLUDES)
OCAMLOPT= ocamlopt $(INCLUDES)
OCAMLDEP= ocamldep $(INCLUDES)

wind : wind.cmi

wind : wind.ml
	$(OCAMLC) -o $@ xml-light.cma glibivy-ocaml.cma lablgtk.cma str.cma lib-pprz.cma $< # To check
	cat ../../../pprz_src_test.sh > $@
	echo 'ocaml -I +lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml xml-light.cma glibivy-ocaml.cma lablgtk.cma str.cma lib-pprz.cma $$PAPARAZZI_SRC/sw/ground_segment/wind/$< $$*' >> $@
	chmod a+x $@


.SUFFIXES:
.SUFFIXES: .ml .mli .mly .mll .cmi .cmo .cmx .out .opt .p.cmx .popt

.ml.cmo :
	$(OCAMLC) -c $<
.mli.cmi :
	$(OCAMLMLI) -c $<
.ml.cmx :
	$(OCAMLOPT) -c $<
# To produce profiled objects
.ml.p.cmx :
	$(OCAMLOPT) -p -c $<
	mv $*.cmx $@
	mv $*.o $*.p.o
.cmo.out :
	$(OCAMLC) -o $@ $<
# To produce profiled binaries
.p.cmx.popt :
	$(OCAMLOPT) -p -o $@ $<
.cmx.opt :
	$(OCAMLOPT) -o $@ $<

clean:
	\rm -f *.cmo *.cmi *.cmx *.o *~ *.opt *.out .depend *.popt wind

.depend:
	$(OCAMLDEP) *.mli *.ml > $@

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif

