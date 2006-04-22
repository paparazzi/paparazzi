# Paparazzi simulator $Id$
#  
# Copied from autopilot (autopilot.sf.net) thanx alot Trammell
#
# Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

include ../../conf/Makefile.local

SIMML = stdlib.ml data.ml flightModel.ml gps.ml
SIMHML = $(SIMML) hitl.ml sim.ml
SIMHCMO=$(SIMHML:%.ml=%.cmo)
SIMSML = $(SIMML) sitl.ml sim.ml
SIMSCMO=$(SIMSML:%.ml=%.cmo)
SIMSCMX=$(SIMSML:%.ml=%.cmx)


OCAMLC = ocamlc
OCAMLOPT=ocamlopt -p
INCLUDES= -I +lablgtk2 -I ../lib/ocaml

AIRBORNE = ../airborne
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

all : gaia sitl.cma fg.o

simhitl.out : $(SIMHCMO) simhitl.cmo
	$(OCAMLC) $(INCLUDES) -o $@ str.cma xml-light.cma unix.cma lib.cma lablgtk.cma gtkInit.cmo $^

sitl.cma : fg.o $(SIMSCMO)
	ocamlmklib -o sitl $^ 

sitl.cmxa : $(SIMSCMX)
	ocamlopt -o $@ -a $^ 

gaia : gaia.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $<

%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.o : %.c
	$(OCAMLC) -c $<

%.cmx : %.ml
	$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	$(OCAMLC) $(INCLUDES) -c $<

clean :
	\rm -f *.cm* *~ *.out .depend *.o *.a *.so gaia

#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
