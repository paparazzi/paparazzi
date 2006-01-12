OCAML=ocaml
OCAMLC=ocamlc -I ../lib/ocaml
OCAMLLEX=ocamllex
OCAMLYACC=ocamlyacc

all: gen_aircraft.out gen_airframe.out gen_calib.out gen_messages.out gen_ubx.out gen_flight_plan.out gen_radio.out gen_sim_downlink.out gen_dl.out extract_makefile.out gen_control.out

FP_CMO = fp_syntax.cmo fp_parser.cmo fp_lexer.cmo fp_proc.cmo gen_flight_plan.ml
ABS_FP = $(FP_CMO:%=$$PAPARAZZI_SRC/sw/tools/%)

gen_flight_plan.out : $(FP_CMO)
	$(OCAMLC) -o $@ ivy-ocaml.cma lib-pprz.cma $^
	@cat ../../pprz_src_test.sh > $@
	@echo '$(OCAML) -I $$PAPARAZZI_SRC/sw/lib/ocaml -I $$PAPARAZZI_SRC/sw/tools ivy-ocaml.cma lib-pprz.cma $(ABS_FP) $$*' >> $@
	@chmod a+x $@

fp_parser.cmo : fp_parser.cmi fp_syntax.cmi
fp_parser.cmi : fp_parser.ml fp_syntax.cmi
fp_lexer.cmi : fp_syntax.cmi
fp_lexer.cmo : fp_lexer.cmi
gen_flight_plan.cmo : fp_parser.cmi fp_proc.cmi
fp_syntax.cmo : fp_syntax.cmi


%.out : %.ml
	$(OCAMLC) -o $@ ivy-ocaml.cma lib-pprz.cma $<
	@cat ../../pprz_src_test.sh > $@
	@echo '$(OCAML) -I $$PAPARAZZI_SRC/sw/lib/ocaml ivy-ocaml.cma lib-pprz.cma $$PAPARAZZI_SRC/sw/tools/$< $$*' >> $@
	@chmod a+x $@

%.cmo : %.ml
	$(OCAMLC) -c $<

%.cmi : %.mli
	$(OCAMLC) -c $<

%.ml : %.mll
	$(OCAMLLEX) $<

%.ml : %.mly
	$(OCAMLYACC) $<

%.mli : %.mly
	$(OCAMLYACC) $<

clean:
	rm -f *.cm* *.out *~ fp_parser.ml fp_parser.mli
