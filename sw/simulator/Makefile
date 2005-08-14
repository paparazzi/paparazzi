# Paparazzi simulator $Id$
#  
# Copied from autopilot (autopilot.sf.net) thanx alot Trammell
#
# Copyright (C) 2003 Trammell Hudson <hudson@rotomotion.com>
# Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

include ../../conf/Makefile.local

ACDIR= $(PAPARAZZI_HOME)/var/$(AIRCRAFT)
OBJDIR= $(ACDIR)/sim

SIMHML = stdlib.ml data.ml flightModel.ml gps.ml hitl.ml sim.ml
SIMHCMO=$(SIMHML:%.ml=%.cmo)
SIMSML = stdlib.ml data.ml flightModel.ml gps.ml sitl.ml sim.ml
SIMSCMO=$(SIMSML:%.ml=%.cmo)
SIMSCMX=$(SIMSML:%.ml=%.cmx)
SIMSC = sim_ir.c sim_gps.c sim_ap.c estimator.c pid.c nav.c main.c cam.c
SIMSO=$(SIMSC:%.c=$(OBJDIR)/%.o)
SIMSA=sims.cma

OCAMLC = ocamlc
OCAMLOPT=ocamlopt -p
INCLUDES= -I +lablgtk2 -I $(SIMDIR)/../lib/ocaml
OCAMLCC = gcc -O2 -I /usr/include/glib-2.0 -I /usr/lib/glib-2.0/include -DUBX -DCTL_BRD_V1_2 -I $(OBJDIR) -I $(ACDIR) 

FBW = ../airborne/fly_by_wire
AP = ../airborne/autopilot
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

MESSAGES = ../../conf/messages.xml
GEN_DOWNLINK = $(TOOLS)/gen_sim_downlink.out

SIMDIR=$(shell echo `pwd`)



#all : simhitl.out sitl.cma $(GEN_DOWNLINK)
all : sitl.cma $(GEN_DOWNLINK)

sim_sitl : $(OBJDIR)/simsitl

simhitl.out : $(SIMHCMO) simhitl.cmo
	$(OCAMLC) $(INCLUDES) -o $@ str.cma xml-light.cma unix.cma lib.cma lablgtk.cma gtkInit.cmo $^

sitl.cma : $(SIMSCMO)
	ocamlc -o $@ -a $^ 

sitl.cmxa : $(SIMSCMX)
	ocamlopt -o $@ -a $^ 

$(OBJDIR)/$(SIMSA) : $(SIMSO)
	touch $(OBJDIR)/dummy.ml
	cd $(OBJDIR); ocamlmklib -o sims $^ dummy.ml

$(OBJDIR)/simsitl : $(OBJDIR)/$(SIMSA) $(OBJDIR)/simsitl.ml
	$(OCAMLC) $(INCLUDES) -o $@ glibivy-ocaml.cma xml-light.cma unix.cma lib-pprz.cma lablgtk.cma gtkInit.cmo -I $(OBJDIR) $(SIMSA) sitl.cma $(OBJDIR)/simsitl.ml # To check
	cat ../../pprz_src_test.sh > $@
	echo 'lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml -I $(OBJDIR) glibivy-ocaml.cma xml-light.cma lib-pprz.cma $(SIMSA) $$PAPARAZZI_SRC/sw/simulator/sitl.cma -I $$PAPARAZZI_SRC/sw/simulator $(OBJDIR)/simsitl.ml $$*' >> $@
	chmod a+x $@

$(OBJDIR)/simsitl.opt : $(SIMSO) $(OBJDIR)/simsitl.cmx
	$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa glibivy-ocaml.cmxa xml-light.cmxa unix.cmxa lib.cmxa lablgtk.cmxa gtkInit.cmx $(SIMSO) sitl.cmxa  $(OBJDIR)/simsitl.cmx

$(OBJDIR)/%.o : %.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(FBW) -I $(AP) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/%.o : $(AP)/%.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(FBW) -I $(AP) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/main.o : $(OBJDIR)/main.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(FBW) -I $(AP) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/sim_gps.o $(OBJDIR)/nav.o $(OBJDIR)/main.o $(OBJDIR)/sim_ir.o $(OBJDIR)/sim_ap.o $(OBJDIR)/pid.o $(OBJDIR)/estimator.o : $(ACINCLUDE)/flight_plan.h  $(ACINCLUDE)/airframe.h

$(OBJDIR)/main.c : $(OBJDIR)/downlink.h $(AP)/main.c
	cp $(AP)/main.c $(@)

$(OBJDIR)/downlink.h : $(MESSAGES) $(GEN_DOWNLINK)
	$(GEN_DOWNLINK) $< > $@

$(OBJDIR)/simsitl.cmo : $(OBJDIR)/simsitl.ml
	$(OCAMLC) $(INCLUDES) -c -o $@ $<

$(OBJDIR)/simsitl.cmx : $(OBJDIR)/simsitl.ml
	$(OCAMLOPT) $(INCLUDES) -c -o $@ $<

$(OBJDIR)/simsitl.ml : simsitl.ml
	echo "Sim.ac_name := \"$(AIRCRAFT)\"" > $@
	cat $< >> $@

%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.cmx : %.ml
	$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	$(OCAMLC) $(INCLUDES) -c $<

clean :
	\rm -f *.cm* *~ *.out .depend *.o *.a *.so

flightModel.cmo: flightModel.cmi 
hitl.cmo: hitl.cmi 
sim.cmo: flightModel.cmi sim.cmi 
simhitl.cmo: hitl.cmi sim.cmi
simsitl.cmo: sim.cmi sitl.cmi 
sitl.cmo: sitl.cmi 
sitl.cmi: sim.cmi 
data.cmo : data.cmi
stdlib.cmo : stdlib.cmi
gps.cmo : gps.cmi
