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


include ../../conf/Makefile.local
include ../Makefile.ocaml

OCAMLC += -g
INCLUDES =
PKG = -package glibivy,pprz
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz

SIMML = stdlib.ml data.ml flightModel.ml gps.ml
SIMHML = $(SIMML) hitl.ml sim.ml
SIMHCMO=$(SIMHML:%.ml=%.cmo)
SIMSML = $(SIMML) sitl.ml sim.ml
SIMSCMO=$(SIMSML:%.ml=%.cmo)
SIMSCMX=$(SIMSML:%.ml=%.cmx)

AIRBORNE = ../airborne
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

CAML_CFLAGS = -I $(shell $(OCAMLC) -where)


all : gaia sitl.cma

sitl.cma : fg.o $(SIMSCMO) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLMKLIB) -o sitl $^

sitl.cmxa : $(SIMSCMX) $(LIBPPRZCMXA)
	@echo OC $@
	$(Q)$(OCAMLOPT) -o $@ -a $^

gaia : gaia.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) gtkInit.cmo $<

diffusion : stdlib.cmo diffusion.cmo
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) gtkInit.cmo $^

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $(PKG) $<

%.o : %.c
	@echo CC $<
	$(Q)$(CC) -fPIC -c $< $(CAML_CFLAGS)

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $(PKG) $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $(PKG) $<

clean :
	$(Q)rm -f *.cm* *~ *.out .depend *.o *.a *.so gaia simhitl diffusion

.PHONY: all clean


#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
