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

OCAML=ocaml
OCAMLC=ocamlc
OCAMLDEP=ocamldep
LIBPPRZDIR=../lib/ocaml
INCLUDES=-I $(LIBPPRZDIR) $(shell ocamlfind query -r -i-format xml-light)
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring) $(shell ocamlfind query -r -i-format netclient)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring) $(shell ocamlfind query -r -a-format -predicates byte netclient)
LIBPPRZCMA=$(LIBPPRZDIR)/lib-pprz.cma

all: gen_common.cmo gen_aircraft.out gen_airframe.out gen_messages2.out gen_messages.out gen_ubx.out gen_mtk.out gen_flight_plan.out gen_radio.out gen_periodic.out gen_settings.out gen_xsens.out gen_modules.out gen_autopilot.out gen_abi.out find_free_msg_id.out gen_srtm.out mergelogs

FP_CMO = fp_proc.cmo gen_flight_plan.cmo
ABS_FP = $(FP_CMO:%=$$PAPARAZZI_SRC/sw/tools/%)

gen_flight_plan.out : $(FP_CMO)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -custom -o $@ unix.cma str.cma xml-light.cma ivy-ocaml.cma lib-pprz.cma $^

gen_srtm.out : gen_common.cmo
	@echo OC $@
	$(Q)$(OCAMLC) $(INCLUDES) $(OCAMLNETINCLUDES) -custom -o $@ unix.cma str.cma netstring.cma netclient.cma xml-light.cma ivy-ocaml.cma lib-pprz.cma gen_common.cmo $^

mergelogs: mergelogs.c
	gcc mergelogs.c -o mergelogs

%.out : %.ml gen_common.cmo $(LIBPPRZCMA)
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma ivy-ocaml.cma xml-light.cma lib-pprz.cma gen_common.cmo $<
	@cat ../../pprz_src_test.sh > $@
	@echo '$(OCAML) -I $$PAPARAZZI_SRC/sw/tools $(shell ocamlfind query -r -i-format xml-light) $(OCAMLNETINCLUDES) -I $$PAPARAZZI_SRC/sw/lib/ocaml unix.cma str.cma ivy-ocaml.cma xml-light.cma $(OCAMLNETCMA) lib-pprz.cma gen_common.cmo $$PAPARAZZI_BIN/$< $$*' >> $@
	@chmod a+x $@

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

# dependency on lib-pprz
gen_flight_plan.out gen_srtm.out : $(LIBPPRZCMA)


clean:
	rm -f *.cm* *.out *~ .depend fp_parser.ml fp_parser.mli mergelogs

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml *.mli > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
