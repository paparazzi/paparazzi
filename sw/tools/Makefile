OCAMLC=ocamlc -g -I ../lib/ocaml
OCAMLLEX=ocamllex
OCAMLYACC=ocamlyacc

all: gen_aircraft.out gen_airframe.out gen_calib.out gen_messages.out gen_ubx.out gen_flight_plan.out gen_radio.out

FP_CMO = fp_syntax.cmo fp_parser.cmo fp_lexer.cmo fp_proc.cmo gen_flight_plan.cmo

gen_flight_plan.out : $(FP_CMO)
	$(OCAMLC) lib.cma $^ -o $@

fp_parser.cmo : fp_parser.cmi fp_syntax.cmi
fp_parser.cmi : fp_parser.ml fp_syntax.cmi
fp_lexer.cmi : fp_syntax.cmi
fp_lexer.cmo : fp_lexer.cmi
gen_flight_plan.cmo : fp_parser.cmi fp_proc.cmi
fp_syntax.cmo : fp_syntax.cmi


%.out : %.ml
	$(OCAMLC) lib.cma $< -o $@

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
