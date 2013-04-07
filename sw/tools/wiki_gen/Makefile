

all: 	wiki_gen
	cd ../../../conf/autopilot/subsystems/fixedwing; ls -1 > list_of_files.txt; ../../../../sw/tools/wiki_gen/wiki_gen list_of_files.txt > ../../../../sw/tools/wiki_gen/fixedwing.wiki; rm list_of_files.txt

wiki_gen:
	g++ wiki_gen.c -o wiki_gen



# Quiet compilation
Q=@

OCAML=ocaml
OCAMLC=ocamlc
INCLUDES=-I ../../lib/ocaml $(shell ocamlfind query -r -i-format xml-light)
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring) $(shell ocamlfind query -r -i-format netclient)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)


%.out : %.ml Makefile
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma ivy-ocaml.cma xml-light.cma lib-pprz.cma $<
	@cat ../../../pprz_src_test.sh > $@
	@echo '$(OCAML) $(shell ocamlfind query -r -i-format xml-light) $(OCAMLNETINCLUDES) -I $$PAPARAZZI_SRC/sw/lib/ocaml unix.cma str.cma ivy-ocaml.cma xml-light.cma $(OCAMLNETCMA) lib-pprz.cma $$PAPARAZZI_BIN/$< $$*' >> $@
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
	$(Q)rm -f *.cm* *.out *~
