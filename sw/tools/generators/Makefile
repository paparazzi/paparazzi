# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2003-2014 The Paparazzi Team
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

include ../../Makefile.ocaml

INCLUDES =
PKG = -package pprz
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz

all: gen_aircraft.out gen_airframe.out gen_messages.out gen_ubx.out gen_mtk.out gen_flight_plan.out gen_radio.out gen_periodic.out gen_settings.out gen_xsens.out gen_modules.out gen_autopilot.out gen_abi.out gen_srtm.out

gen_flight_plan.out : gen_flight_plan.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $^

gen_srtm.out : gen_srtm.ml $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $<

%.out : %.ml $(LIBPPRZCMA)
	@echo OL $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) -c $<

clean:
	$(Q)rm -f *.cm* *.out *~ .depend

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml *.mli > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
