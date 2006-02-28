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
SIMSC = sim_ir.c sim_gps.c sim_ap.c sys_time.c estimator.c infrared.c gps.c pid.c nav.c radio_control.c main_ap.c cam.c traffic_info.c
SIMSO=$(SIMSC:%.c=$(OBJDIR)/%.o)
SIMSA=sims.cma

OCAMLC = ocamlc
OCAMLOPT=ocamlopt -p
INCLUDES= -I +lablgtk2 -I ../lib/ocaml
OCAMLCC = gcc -Werror -O2 -I /usr/include/glib-2.0 -I /usr/lib/glib-2.0/include -DUBX -DMOBILE_CAM -DAP -DDOWNLINK -DSITL -DINFRARED -DGPS -I $(OBJDIR) -I $(ACDIR) 

AIRBORNE = ../airborne
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

MESSAGES = ../../conf/messages.xml
GEN_DOWNLINK = $(TOOLS)/gen_sim_downlink.out

SIMDIR=$(shell echo `pwd`)



#all : simhitl.out sitl.cma $(GEN_DOWNLINK)
all : gaia sitl.cma $(GEN_DOWNLINK) fg.o

sim_sitl : $(OBJDIR)/simsitl

simhitl.out : $(SIMHCMO) simhitl.cmo
	$(OCAMLC) $(INCLUDES) -o $@ str.cma xml-light.cma unix.cma lib.cma lablgtk.cma gtkInit.cmo $^

sitl.cma : fg.o $(SIMSCMO)
	ocamlc -o $@ -a $^ 

fg.o : fg.c
	$(OCAMLC) -c fg.c

sitl.cmxa : $(SIMSCMX)
	ocamlopt -o $@ -a $^ 

$(OBJDIR)/$(SIMSA) : $(SIMSO)
	@touch $(OBJDIR)/dummy.ml
	@cd $(OBJDIR); ocamlmklib -o sims $^ dummy.ml

$(OBJDIR)/simsitl : $(OBJDIR)/$(SIMSA) $(OBJDIR)/simsitl.ml
	$(OCAMLC) -custom $(INCLUDES) -o $@ glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo -I $(OBJDIR) $(SIMSA) sitl.cma $(OBJDIR)/simsitl.ml

gaia : gaia.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $<

$(OBJDIR)/%.o : %.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(AIRBORNE) -I $(AIRBORNE) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/%.o : $(AIRBORNE)/%.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(AIRBORNE) -I $(AIRBORNE) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/main_ap.o : $(OBJDIR)/main_ap.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(AIRBORNE) -I $(AIRBORNE) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/sim_gps.o $(OBJDIR)/nav.o $(OBJDIR)/main_ap.o $(OBJDIR)/sim_ir.o $(OBJDIR)/sim_ap.o $(OBJDIR)/pid.o $(OBJDIR)/estimator.o $(OBJDIR)/cam.o $(OBJDIR)/gps.o : $(OBJDIR)/sitl_messages.h  $(ACINCLUDE)/flight_plan.h  $(ACINCLUDE)/airframe.h

$(OBJDIR)/sim_ap.o : $(AIRBORNE)/traffic_info.h $(AIRBORNE)/nav.h 

$(OBJDIR)/main_ap.c : $(OBJDIR)/sitl_messages.h $(AIRBORNE)/main_ap.c
	cp $(AIRBORNE)/main_ap.c $(@)

$(OBJDIR)/sitl_messages.h : $(MESSAGES) $(GEN_DOWNLINK)
	$(GEN_DOWNLINK) $< > $@

$(OBJDIR)/simsitl.cmo : $(OBJDIR)/simsitl.ml
	$(OCAMLC) $(INCLUDES) -c -o $@ $<

$(OBJDIR)/simsitl.cmx : $(OBJDIR)/simsitl.ml
	$(OCAMLOPT) $(INCLUDES) -c -o $@ $<

$(OBJDIR)/simsitl.ml : simsitl.ml
	@echo "Sim.ac_name := \"$(AIRCRAFT)\"" > $@
	@cat $< >> $@

%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.o : %.c
	$(OCAMLC) -c $<

%.cmx : %.ml
	$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	$(OCAMLC) $(INCLUDES) -c $<

clean :
	\rm -f *.cm* *~ *.out .depend *.o *.a *.so gaia

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
