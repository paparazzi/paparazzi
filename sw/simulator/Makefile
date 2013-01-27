# Hey Emacs, this is a -*- makefile -*-
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

# Quiet compilation
Q=@

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  FPIC = -ccopt -fPIC
else
  FPIC =
endif

include ../../conf/Makefile.local

SIMML = stdlib.ml data.ml flightModel.ml gps.ml
SIMHML = $(SIMML) hitl.ml sim.ml
SIMHCMO=$(SIMHML:%.ml=%.cmo)
SIMSML = $(SIMML) sitl.ml sim.ml
SIMSCMO=$(SIMSML:%.ml=%.cmo)
SIMSCMX=$(SIMSML:%.ml=%.cmx)


OCAMLC = ocamlc -g
OCAMLOPT = ocamlopt
OCAMLDEP = ocamldep
OCAML=$(shell which ocaml)
OCAMLRUN=$(shell which ocamlrun)
LIBPPRZDIR=../lib/ocaml
INCLUDES= $(shell ocamlfind query -r -i-format lablgtk2) -I $(LIBPPRZDIR) $(shell ocamlfind query -r -i-format xml-light)
LIBPPRZCMA=$(LIBPPRZDIR)/lib-pprz.cma

AIRBORNE = ../airborne
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)


all : gaia sitl.cma simhitl

simhitl : fg.o $(SIMHCMO) simhitl.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ str.cma xml-light.cma unix.cma glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $^

sitl.cma : fg.o $(SIMSCMO)
	@echo OL $@
	$(Q)ocamlmklib -o sitl $^

sitl.cmxa : $(SIMSCMX)
	ocamlopt -o $@ -a $^

gaia : gaia.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $<

diffusion : stdlib.cmo diffusion.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $^

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.o : %.c
	@echo OC $<
	$(Q)$(OCAMLC) $(FPIC) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

# dependency on lib-pprz
simhitl diffusion gaia:  $(LIBPPRZCMA)

clean :
	rm -f *.cm* *~ *.out .depend *.o *.a *.so gaia simhitl diffusion

.PHONY: all clean


#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
