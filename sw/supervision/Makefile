# Quiet
Q=@

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLLIB = ../lib/ocaml
INCLUDES= -I $(OCAMLLIB) -I ../multimon -I +lablgtk2 -I +xml-light
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma
PAPARAZZICENTERCMO = gui.cmo paparazzicenter.cmo

all: paparazzicenter

paparazzicenter : $(PAPARAZZICENTERCMO) $(OCAMLLIB)/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma gtkInit.cmo lib-pprz.cma $^

gui.ml : paparazzicenter.glade
	lablgladecc2 -root window $< > $@

program.ml : paparazzicenter.glade
	lablgladecc2 -root hbox_program $< > $@

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

paparazzicenter.cmo : gui.cmo

clean:
	\rm *.cm* gui.ml
