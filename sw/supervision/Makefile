# Quiet
Q=@

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLLIB = ../lib/ocaml
INCLUDES= -I $(OCAMLLIB) -I ../multimon -I +lablgtk2 -I +xml-light
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma
PAPARAZZICENTERCMO = gtk_pc.cmo gtk_process.cmo pc_common.cmo pc_aircraft.cmo pc_control_panel.cmo paparazzicenter.cmo

all: paparazzicenter

paparazzicenter : $(PAPARAZZICENTERCMO) $(OCAMLLIB)/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma gtkInit.cmo lib-pprz.cma lablgnomeui.cma $^

gtk_pc.ml : paparazzicenter.glade
	lablgladecc2 -root window $< > $@

gtk_process.ml : paparazzicenter.glade
	lablgladecc2 -root hbox_program $< | grep -B 1000000 "  end" > $@

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

paparazzicenter.cmo : gtk_pc.cmo gtk_process.cmo pc_common.cmo pc_control_panel.cmo pc_aircraft.cmo

pc_control_panel.cmo pc_aircraft.cmo : pc_common.cmo

clean:
	\rm *.cm* gtk_pc.ml gtk_process.ml

#
# Dependencies
#

.depend:
	ocamldep -I ../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
