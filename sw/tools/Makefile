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
INCLUDES=-I ../lib/ocaml -I +xml-light
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)
OCAMLLEX=ocamllex
OCAMLYACC=ocamlyacc

all: gen_aircraft.out gen_airframe.out gen_messages.out gen_ubx.out gen_flight_plan.out gen_radio.out extract_makefile.out gen_periodic.out gen_settings.out gen_tuning.out gen_xsens.out

FP_CMO = fp_syntax.cmo fp_parser.cmo fp_lexer.cmo fp_proc.cmo gen_flight_plan.cmo
ABS_FP = $(FP_CMO:%=$$PAPARAZZI_SRC/sw/tools/%)

gen_flight_plan.out : $(FP_CMO) ../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -custom -o $@ unix.cma str.cma xml-light.cma ivy-ocaml.cma lib-pprz.cma $^

fp_parser.cmo : fp_parser.cmi fp_syntax.cmi
fp_parser.cmi : fp_parser.ml fp_syntax.cmi
fp_lexer.cmi : fp_syntax.cmi
fp_lexer.cmo : fp_lexer.cmi
gen_flight_plan.cmo : fp_parser.cmi fp_proc.cmi
fp_syntax.cmo : fp_syntax.cmi


%.out : %.ml Makefile
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma ivy-ocaml.cma xml-light.cma lib-pprz.cma $<
	@cat ../../pprz_src_test.sh > $@
	@echo '$(OCAML) -I +xml-light $(OCAMLNETINCLUDES) -I $$PAPARAZZI_SRC/sw/lib/ocaml unix.cma str.cma ivy-ocaml.cma xml-light.cma $(OCAMLNETCMA) lib-pprz.cma $$PAPARAZZI_BIN/$< $$*' >> $@
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
