

all: 
	g++ wiki_gen.c -o wiki_gen


parse:
	ls -1 > list_of_files.txt
	./wiki_gen list_of_files.txt > dir.wiki






# Quiet compilation
Q=@

OCAML=ocaml
OCAMLC=ocamlc
INCLUDES=-I ../../lib/ocaml -I +xml-light
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)


%.out : %.ml Makefile
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ unix.cma str.cma ivy-ocaml.cma xml-light.cma lib-pprz.cma $<
	@cat ../../../pprz_src_test.sh > $@
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
	rm -f *.cm* *.out *~ 
