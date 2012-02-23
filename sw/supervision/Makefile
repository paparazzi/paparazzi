# Quiet
Q=@

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLLIB = ../lib/ocaml
INCLUDES= -I $(OCAMLLIB) $(shell ocamlfind query -r -i-format lablgtk2) $(shell ocamlfind query -r -i-format xml-light)
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma
PAPARAZZICENTERCMO = gtk_pc.cmo gtk_process.cmo pc_common.cmo pc_aircraft.cmo pc_control_panel.cmo paparazzicenter.cmo

all: paparazzicenter

paparazzicenter : $(PAPARAZZICENTERCMO) $(OCAMLLIB)/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma gtkInit.cmo lib-pprz.cma xlib-pprz.cma lablgnomeui.cma $^

gtk_pc.ml : paparazzicenter.glade
	grep -v invisible_char $< > /tmp/$<
	lablgladecc2 -hide-default -root window /tmp/$< > $@

gtk_process.ml : paparazzicenter.glade
	grep -v invisible_char $< > /tmp/$<
	lablgladecc2 -hide-default -root hbox_program /tmp/$< | grep -B 1000000 "  end" > $@

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $<

paparazzicenter.cmo : gtk_pc.cmo gtk_process.cmo pc_common.cmo pc_control_panel.cmo pc_aircraft.cmo

pc_control_panel.cmo pc_aircraft.cmo : pc_common.cmo

clean:
	\rm -f *.cm* gtk_pc.ml gtk_process.ml .depend paparazzicenter

#
# Dependencies
#

.depend:
	ocamldep -I ../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
