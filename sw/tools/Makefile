# $Id$
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
INCLUDES=-I ../lib/ocaml $(shell ocamlfind query -r -i-format xml-light) -I .
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)
OCAMLLEX=ocamllex
OCAMLYACC=ocamlyacc

all: gen_common.cmo gen_aircraft.out gen_airframe.out gen_messages2.out gen_messages.out gen_ubx.out gen_mtk.out gen_flight_plan.out gen_radio.out gen_periodic.out gen_settings.out gen_tuning.out gen_xsens.out gen_modules.out find_free_msg_id.out

FP_CMO = fp_proc.cmo gen_flight_plan.cmo
ABS_FP = $(FP_CMO:%=$$PAPARAZZI_SRC/sw/tools/%)

gen_flight_plan.out : $(FP_CMO) ../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -custom -o $@ unix.cma str.cma xml-light.cma ivy-ocaml.cma lib-pprz.cma $^

fp_proc.cmo : fp_proc.cmi
gen_flight_plan.cmo : fp_proc.cmi

gen_common.cmo : gen_common.cmi

%.out : %.ml gen_common.ml Makefile
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma ivy-ocaml.cma xml-light.cma lib-pprz.cma gen_common.cmo $<
	@cat ../../pprz_src_test.sh > $@
	@echo '$(OCAML) -I $$PAPARAZZI_SRC/sw/tools $(shell ocamlfind query -r -i-format xml-light) $(OCAMLNETINCLUDES) -I $$PAPARAZZI_SRC/sw/lib/ocaml unix.cma str.cma ivy-ocaml.cma xml-light.cma $(OCAMLNETCMA) lib-pprz.cma gen_common.cmo $$PAPARAZZI_BIN/$< $$*' >> $@
	@chmod a+x $@

%.cmo : %.ml ../lib/ocaml/lib-pprz.cma
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.ml : %.mll
	@echo OC $<
	$(Q)$(OCAMLLEX) $<

%.ml : %.mly
	@echo OC $<
	$(Q)$(OCAMLYACC) $<

%.mli : %.mly
	@echo OC $<
	$(Q)$(OCAMLYACC) $<

clean:
	rm -f *.cm* *.out *~ fp_parser.ml fp_parser.mli
